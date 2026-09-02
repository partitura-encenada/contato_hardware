// ═════════ MONITOR DE FLUXO - contato ═════════
// Sniffer passivo de canal + identificação de origem por MAC + gravação
// de sessões de log em flash (LittleFS), acessível por:
//   - Wi-Fi proprio (AP): conecte no SSID/senha abaixo e mande comandos
//     via TCP (porta PORTA_TCP), ex.: `nc <ip_do_monitor> 5000`
//   - Serial (USB): os mesmos comandos funcionam digitados no
//     monitor serial (pio device monitor), sem precisar de Wi-Fi.
//
// Comandos: START | STOP | STATUS | SETTIME <epoch_unix> |
//           LISTAR | BAIXAR <arquivo> | APAGAR <arquivo|TUDO>

#include <WiFi.h>
#include "esp_wifi.h"
#include <LittleFS.h>
#include <string.h>
#include <time.h>

// ═════════ Configuração ═════════
const int CANAL = 1;
const char* SSID_MONITOR  = "contato_monitor";
const char* SENHA_MONITOR = "monitor123"; // troque se quiser
const uint16_t PORTA_TCP  = 5000;

// ═════════ Tabela de MACs conhecidos (equips do sistema contato) ═════════
// O MAC de cada equip é o mesmo valor usado em macTransmissor nos
// arquivos base_X.cpp. Atualize sempre que um equip mudar de MAC.
typedef struct {
    uint8_t mac[6];
    const char* label;
} equip_conhecido_t;

const equip_conhecido_t EQUIPS_CONHECIDOS[] = {
    {{0x1C, 0x69, 0x20, 0xA4, 0x14, 0x94}, "Equip (base 1/5)"},
    {{0x84, 0x1F, 0xE8, 0x1C, 0x72, 0x5C}, "Equip (base 2/6)"},
    {{0x68, 0x25, 0xDD, 0x32, 0x88, 0xB4}, "Equip (base 3)"},
    {{0x14, 0x33, 0x5C, 0x52, 0x4D, 0xE0}, "Equip (base 4)"},
    {{0xF8, 0xB3, 0xB7, 0x50, 0xCC, 0xEC}, "Equip (base 7/7accel/9)"},
    {{0x3C, 0x8A, 0x1F, 0x80, 0x76, 0xA4}, "Equip (base 8accel/7_teste-1)"},
    {{0xA0, 0xDD, 0x6C, 0x0F, 0xBB, 0x3C}, "Equip (base 7_teste-2)"},
    {{0x1C, 0x69, 0x20, 0xA2, 0xE2, 0x14}, "Equip (base 8)"},
};
const int NUM_EQUIPS_CONHECIDOS = sizeof(EQUIPS_CONHECIDOS) / sizeof(EQUIPS_CONHECIDOS[0]);

// ═════════ Cabeçalho 802.11 (para extrair endereço de origem) ═════════
typedef struct {
    uint16_t frame_ctrl;
    uint16_t duration_id;
    uint8_t  addr1[6]; // destino
    uint8_t  addr2[6]; // origem
    uint8_t  addr3[6];
    uint16_t seq_ctrl;
} wifi_ieee80211_mac_hdr_t;

typedef struct {
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0];
} wifi_ieee80211_packet_t;

// ═════════ Estado por estação observada na janela atual ═════════
#define MAX_ESTACOES 32

typedef struct {
    uint8_t  mac[6];
    uint32_t pacotes;
    uint32_t bytes;
    int32_t  somaRSSI;
} estacao_t;

estacao_t estacoes[MAX_ESTACOES];
int numEstacoes = 0;

uint32_t pacotesTransbordo = 0;
uint32_t bytesTransbordo   = 0;

// ═════════ Contadores agregados do canal ═════════
uint32_t pacotesTotal = 0;
uint32_t pacotesForte = 0; // acima de -60 dBm
uint32_t bytesTotal   = 0;
int32_t  somaRSSI     = 0;
int8_t   maiorRSSI    = -127;
int8_t   menorRSSI    = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint8_t macAP[6];

// ═════════ Estado de rede / gravação / horário ═════════
WiFiServer servidorTCP(PORTA_TCP);
WiFiClient clienteAtivo;

File arquivoLog;
bool gravando = false;
String arquivoAtualNome = "";
uint32_t linhasGravadas = 0;

bool    horarioDefinido = false;
time_t  horarioReferenciaEpoch = 0;
unsigned long horarioReferenciaMillis = 0;

// Começa desligado: assim o terminal fica "estático" (sem flood de
// relatorio a cada segundo) e da pra digitar START/STOP com calma.
// A gravacao em arquivo (gravando/gravarLinhaCSV) NAO depende disso -
// continua rodando mesmo com a saida ao vivo pausada.
bool saidaAoVivo = false;

// ═════════ Utilitários ═════════
bool macIgual(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 6) == 0;
}

const char* labelParaMac(const uint8_t *mac) {
    for (int i = 0; i < NUM_EQUIPS_CONHECIDOS; i++) {
        if (macIgual(mac, EQUIPS_CONHECIDOS[i].mac)) {
            return EQUIPS_CONHECIDOS[i].label;
        }
    }
    return nullptr;
}

String obterTimestamp() {
    char buf[24];
    if (horarioDefinido) {
        time_t agora = horarioReferenciaEpoch + (millis() - horarioReferenciaMillis) / 1000;
        struct tm tmInfo;
        gmtime_r(&agora, &tmInfo);
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmInfo);
    } else {
        unsigned long s = millis() / 1000;
        snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu(boot)", s / 3600, (s / 60) % 60, s % 60);
    }
    return String(buf);
}

// ═════════ Sniffer promíscuo ═════════
void sniffer(void* buf, wifi_promiscuous_pkt_type_t type) {

    if (type != WIFI_PKT_MGMT &&
        type != WIFI_PKT_DATA &&
        type != WIFI_PKT_CTRL) {
        return;
    }

    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
    int8_t   rssi    = pkt->rx_ctrl.rssi;
    uint16_t tamanho = pkt->rx_ctrl.sig_len;

    portENTER_CRITICAL(&mux);
    pacotesTotal++;
    bytesTotal += tamanho;
    somaRSSI += rssi;
    if (rssi > -60) pacotesForte++;
    if (rssi > maiorRSSI) maiorRSSI = rssi;
    if (menorRSSI == 0 || rssi < menorRSSI) menorRSSI = rssi;
    portEXIT_CRITICAL(&mux);

    if (type == WIFI_PKT_CTRL) return;

    wifi_ieee80211_packet_t* ipkt = (wifi_ieee80211_packet_t*)pkt->payload;
    const uint8_t* origem = ipkt->hdr.addr2;

    // Ignora o próprio tráfego do AP do monitor (beacons, ACKs ao cliente
    // conectado) para não poluir a métrica de "outras fontes".
    if (macIgual(origem, macAP)) return;

    portENTER_CRITICAL(&mux);
    bool achou = false;
    for (int i = 0; i < numEstacoes; i++) {
        if (macIgual(estacoes[i].mac, origem)) {
            estacoes[i].pacotes++;
            estacoes[i].bytes += tamanho;
            estacoes[i].somaRSSI += rssi;
            achou = true;
            break;
        }
    }
    if (!achou) {
        if (numEstacoes < MAX_ESTACOES) {
            memcpy(estacoes[numEstacoes].mac, origem, 6);
            estacoes[numEstacoes].pacotes  = 1;
            estacoes[numEstacoes].bytes    = tamanho;
            estacoes[numEstacoes].somaRSSI = rssi;
            numEstacoes++;
        } else {
            pacotesTransbordo++;
            bytesTransbordo += tamanho;
        }
    }
    portEXIT_CRITICAL(&mux);
}

// ═════════ Snapshot de uma janela de 1s (usado por print e gravação) ═════════
struct RelatorioJanela {
    uint32_t pacotesTotal;
    uint32_t bytesTotal;
    uint32_t pacotesForte;
    int32_t  rssiSoma;
    int8_t   rssiMax;
    int8_t   rssiMin;
    estacao_t estacoes[MAX_ESTACOES];
    int numEstacoes;
    uint32_t transbordoPacotes;
    uint32_t transbordoBytes;
    String timestamp;
};

void gravarLinhaCSV(const RelatorioJanela &r) {
    if (!arquivoLog) return;

    uint32_t outrosPacotes = r.transbordoPacotes;
    uint32_t outrosBytes   = r.transbordoBytes;
    int outrasFontes       = 0;
    for (int i = 0; i < r.numEstacoes; i++) {
        if (labelParaMac(r.estacoes[i].mac) == nullptr) {
            outrosPacotes += r.estacoes[i].pacotes;
            outrosBytes   += r.estacoes[i].bytes;
            outrasFontes++;
        }
    }

    arquivoLog.print(r.timestamp); arquivoLog.print(",");
    arquivoLog.print(r.pacotesTotal); arquivoLog.print(",");
    arquivoLog.print(r.bytesTotal); arquivoLog.print(",");
    arquivoLog.print(r.pacotesTotal > 0 ? (float)r.rssiSoma / r.pacotesTotal : 0); arquivoLog.print(",");
    arquivoLog.print(r.pacotesForte); arquivoLog.print(",");
    arquivoLog.print(outrosPacotes); arquivoLog.print(",");
    arquivoLog.print(outrosBytes); arquivoLog.print(",");
    arquivoLog.print(outrasFontes);

    for (int e = 0; e < NUM_EQUIPS_CONHECIDOS; e++) {
        uint32_t pacotesDesseEquip = 0;
        for (int i = 0; i < r.numEstacoes; i++) {
            if (macIgual(r.estacoes[i].mac, EQUIPS_CONHECIDOS[e].mac)) {
                pacotesDesseEquip = r.estacoes[i].pacotes;
                break;
            }
        }
        arquivoLog.print(",");
        arquivoLog.print(pacotesDesseEquip);
    }
    arquivoLog.println();

    linhasGravadas++;
    if (linhasGravadas % 10 == 0) arquivoLog.flush(); // não grava fisicamente a cada linha
}

void imprimirRelatorio(Print &saida, const RelatorioJanela &r) {
    saida.println("----------------------------------------");
    saida.print("["); saida.print(r.timestamp); saida.println("]");
    saida.print("Pacotes/s (canal todo): "); saida.println(r.pacotesTotal);
    saida.print("Bytes/s (canal todo):   "); saida.println(r.bytesTotal);

    if (r.pacotesTotal > 0) {
        saida.print("RSSI medio: "); saida.print((float)r.rssiSoma / r.pacotesTotal); saida.println(" dBm");
        saida.print("RSSI (max/min): "); saida.print(r.rssiMax); saida.print(" / "); saida.println(r.rssiMin);
        saida.print("Pacotes > -60 dBm: "); saida.println(r.pacotesForte);
    }

    saida.println();
    saida.println("Por origem (equips do contato):");

    uint32_t pacotesConhecidos = 0;
    for (int i = 0; i < r.numEstacoes; i++) {
        const char* label = labelParaMac(r.estacoes[i].mac);
        if (label == nullptr) continue;
        pacotesConhecidos += r.estacoes[i].pacotes;

        saida.print("  "); saida.print(label);
        saida.print("  pacotes/s: "); saida.print(r.estacoes[i].pacotes);
        saida.print("  bytes/s: "); saida.print(r.estacoes[i].bytes);
        saida.print("  RSSI medio: "); saida.print((float)r.estacoes[i].somaRSSI / r.estacoes[i].pacotes);
        saida.println(" dBm");
    }
    if (pacotesConhecidos == 0) saida.println("  (nenhum pacote de equip conhecido nesta janela)");

    uint32_t outrosPacotes = r.transbordoPacotes;
    uint32_t outrosBytes   = r.transbordoBytes;
    int outrasFontes       = 0;
    for (int i = 0; i < r.numEstacoes; i++) {
        if (labelParaMac(r.estacoes[i].mac) != nullptr) continue;
        outrosPacotes += r.estacoes[i].pacotes;
        outrosBytes   += r.estacoes[i].bytes;
        outrasFontes++;
    }

    saida.println();
    saida.print("Outras fontes: "); saida.print(outrasFontes);
    saida.print(" MACs, "); saida.print(outrosPacotes);
    saida.print(" pacotes/s, "); saida.print(outrosBytes); saida.println(" bytes/s");

    if (r.pacotesTotal > 0) {
        float pct = 100.0f * pacotesConhecidos / r.pacotesTotal;
        saida.print("Fatia do canal usada pelos equips do contato: "); saida.print(pct, 1); saida.println("%");
    }
    if (gravando) {
        saida.print("[gravando em "); saida.print(arquivoAtualNome); saida.println("]");
    }
    saida.println();
}

// ═════════ Comandos ═════════
void iniciarGravacao(Print &saida) {
    if (gravando) {
        saida.println("Ja existe uma gravacao em andamento: " + arquivoAtualNome);
        return;
    }

    char nome[40];
    if (horarioDefinido) {
        time_t agora = horarioReferenciaEpoch + (millis() - horarioReferenciaMillis) / 1000;
        snprintf(nome, sizeof(nome), "/log_%lu.csv", (unsigned long)agora);
    } else {
        snprintf(nome, sizeof(nome), "/log_boot%lu.csv", millis());
    }

    arquivoLog = LittleFS.open(nome, "w");
    if (!arquivoLog) {
        saida.println("ERRO: nao foi possivel criar o arquivo de log.");
        return;
    }

    arquivoLog.print("timestamp,pacotes_total,bytes_total,rssi_medio,pacotes_forte,outros_pacotes,outros_bytes,outros_fontes");
    for (int i = 0; i < NUM_EQUIPS_CONHECIDOS; i++) {
        arquivoLog.print(",");
        arquivoLog.print(EQUIPS_CONHECIDOS[i].label);
    }
    arquivoLog.println();
    arquivoLog.flush();

    arquivoAtualNome = String(nome);
    linhasGravadas = 0;
    gravando = true;

    saida.println("OK - gravando em " + arquivoAtualNome);
}

void pararGravacao(Print &saida) {
    if (!gravando) {
        saida.println("Nenhuma gravacao em andamento.");
        return;
    }
    arquivoLog.close();
    gravando = false;
    saida.println("OK - gravacao encerrada: " + arquivoAtualNome + " (" + String(linhasGravadas) + " linhas)");
}

void imprimirStatus(Print &saida) {
    saida.println("---- status ----");
    saida.print("Saida ao vivo: "); saida.println(saidaAoVivo ? "ligada" : "pausada");
    saida.print("Gravando: "); saida.println(gravando ? "sim" : "nao");
    if (gravando) {
        saida.print("Arquivo: "); saida.println(arquivoAtualNome);
        saida.print("Linhas gravadas: "); saida.println(linhasGravadas);
    }
    saida.print("Horario definido: "); saida.println(horarioDefinido ? "sim" : "nao (usando tempo desde o boot)");
    saida.print("Espaco livre na flash: ");
    saida.print(LittleFS.totalBytes() - LittleFS.usedBytes());
    saida.println(" bytes");
    saida.print("Cliente TCP conectado: "); saida.println((clienteAtivo && clienteAtivo.connected()) ? "sim" : "nao");
    saida.println("-----------------");
}

void definirHorario(long epoch, Print &saida) {
    if (epoch <= 0) {
        saida.println("Uso: SETTIME <epoch unix em segundos>. Ex: SETTIME 1756823400");
        saida.println("Dica: 'date +%s' (Linux/Mac) ou 'python3 -c \"import time;print(int(time.time()))\"'");
        return;
    }
    horarioReferenciaEpoch = (time_t)epoch;
    horarioReferenciaMillis = millis();
    horarioDefinido = true;

    struct tm tmInfo;
    gmtime_r(&horarioReferenciaEpoch, &tmInfo);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmInfo);
    saida.print("OK - horario definido: ");
    saida.println(buf);
}

void listarArquivos(Print &saida) {
    File raiz = LittleFS.open("/");
    File arquivo = raiz.openNextFile();
    bool algum = false;
    while (arquivo) {
        String caminho = String(arquivo.name());
        if (caminho.indexOf("log_") >= 0) {
            saida.print(caminho);
            saida.print("  (");
            saida.print(arquivo.size());
            saida.println(" bytes)");
            algum = true;
        }
        arquivo = raiz.openNextFile();
    }
    raiz.close();
    if (!algum) saida.println("Nenhum log salvo.");
}

void baixarArquivo(String nome, Print &saida) {
    if (nome.length() == 0) {
        saida.println("Uso: BAIXAR <nome_do_arquivo> (veja com LISTAR)");
        return;
    }
    if (!nome.startsWith("/")) nome = "/" + nome;

    if (!LittleFS.exists(nome)) {
        saida.println("Arquivo nao encontrado: " + nome);
        return;
    }

    File arquivo = LittleFS.open(nome, "r");
    saida.println("---- inicio " + nome + " ----");
    while (arquivo.available()) {
        saida.write(arquivo.read());
    }
    saida.println();
    saida.println("---- fim " + nome + " ----");
    arquivo.close();
}

void apagarArquivo(String nome, Print &saida) {
    if (nome.length() == 0) {
        saida.println("Uso: APAGAR <nome_do_arquivo> ou APAGAR TUDO");
        return;
    }

    String nomeUpper = nome;
    nomeUpper.toUpperCase();

    if (nomeUpper == "TUDO") {
        String paraApagar[20];
        int total = 0;

        File raiz = LittleFS.open("/");
        File arquivo = raiz.openNextFile();
        while (arquivo && total < 20) {
            String caminho = String(arquivo.name());
            if (caminho.indexOf("log_") >= 0 && caminho != arquivoAtualNome) {
                paraApagar[total++] = caminho;
            }
            arquivo = raiz.openNextFile();
        }
        raiz.close();

        for (int i = 0; i < total; i++) {
            LittleFS.remove(paraApagar[i]);
        }
        saida.println("OK - " + String(total) + " arquivo(s) apagado(s).");
        return;
    }

    if (!nome.startsWith("/")) nome = "/" + nome;
    if (nome == arquivoAtualNome && gravando) {
        saida.println("Nao e possivel apagar o arquivo em gravacao. Rode STOP primeiro.");
        return;
    }
    if (LittleFS.remove(nome)) {
        saida.println("OK - apagado: " + nome);
    } else {
        saida.println("ERRO ao apagar (arquivo existe?): " + nome);
    }
}

void processarComando(String cmd, Print &saida) {
    cmd.trim();
    if (cmd.length() == 0) return;

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

    if (cmdUpper == "START") {
        iniciarGravacao(saida);
    } else if (cmdUpper == "STOP") {
        pararGravacao(saida);
    } else if (cmdUpper == "PAUSAR") {
        saidaAoVivo = false;
        saida.println("OK - saida ao vivo pausada (gravacao em arquivo continua, se ativa)");
    } else if (cmdUpper == "CONTINUAR") {
        saidaAoVivo = true;
        saida.println("OK - saida ao vivo retomada");
    } else if (cmdUpper == "STATUS") {
        imprimirStatus(saida);
    } else if (cmdUpper.startsWith("SETTIME")) {
        String valor = cmd.substring(7);
        valor.trim();
        definirHorario(valor.toInt(), saida);
    } else if (cmdUpper == "LISTAR") {
        listarArquivos(saida);
    } else if (cmdUpper.startsWith("BAIXAR")) {
        String nome = cmd.substring(6);
        nome.trim();
        baixarArquivo(nome, saida);
    } else if (cmdUpper.startsWith("APAGAR")) {
        String nome = cmd.substring(6);
        nome.trim();
        apagarArquivo(nome, saida);
    } else {
        saida.println("Comandos: START, STOP, PAUSAR, CONTINUAR, STATUS, SETTIME <epoch>, LISTAR, BAIXAR <arquivo>, APAGAR <arquivo|TUDO>");
    }
}

// ═════════ setup ═════════
void setup() {
    Serial.begin(115200);

    if (!LittleFS.begin(true)) {
        Serial.println("Falha ao montar LittleFS - gravacao em arquivo desativada.");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID_MONITOR, SENHA_MONITOR, CANAL);
    WiFi.softAPmacAddress(macAP);

    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(&sniffer);
    esp_wifi_set_promiscuous(true);

    servidorTCP.begin();

    Serial.println();
    Serial.println("================================");
    Serial.println(" MONITOR DE FLUXO - contato");
    Serial.println("================================");
    Serial.print("Rede Wi-Fi: ");   Serial.println(SSID_MONITOR);
    Serial.print("Senha: ");        Serial.println(SENHA_MONITOR);
    Serial.print("IP do monitor: ");Serial.println(WiFi.softAPIP());
    Serial.print("Porta TCP: ");    Serial.println(PORTA_TCP);
    Serial.print("Canal monitorado: "); Serial.println(CANAL);
    Serial.println();
    Serial.println("Comandos: START, STOP, PAUSAR, CONTINUAR, STATUS, SETTIME <epoch>, LISTAR, BAIXAR <arquivo>, APAGAR <arquivo|TUDO>");
    Serial.println("Saida ao vivo comeca PAUSADA - digite CONTINUAR para ver os relatorios.");
    Serial.println();
}

// ═════════ loop ═════════
void loop() {

    if (servidorTCP.hasClient()) {
        WiFiClient novo = servidorTCP.available();
        if (!clienteAtivo || !clienteAtivo.connected()) {
            clienteAtivo = novo;
            clienteAtivo.println("Conectado ao monitor contato.");
            clienteAtivo.println("Comandos: START, STOP, PAUSAR, CONTINUAR, STATUS, SETTIME <epoch>, LISTAR, BAIXAR <arquivo>, APAGAR <arquivo|TUDO>");
        } else {
            novo.println("Ja existe uma conexao ativa. Tente novamente depois.");
            novo.stop();
        }
    }

    if (clienteAtivo && clienteAtivo.connected() && clienteAtivo.available()) {
        String linha = clienteAtivo.readStringUntil('\n');
        processarComando(linha, clienteAtivo);
    }

    if (Serial.available()) {
        String linha = Serial.readStringUntil('\n');
        processarComando(linha, Serial);
    }

    static uint32_t ultimo = 0;

    if (millis() - ultimo >= 1000) {
        ultimo = millis();

        RelatorioJanela r;

        portENTER_CRITICAL(&mux);
        r.pacotesTotal = pacotesTotal;
        r.bytesTotal   = bytesTotal;
        r.pacotesForte = pacotesForte;
        r.rssiSoma     = somaRSSI;
        r.rssiMax      = maiorRSSI;
        r.rssiMin      = menorRSSI;
        r.numEstacoes  = numEstacoes;
        memcpy(r.estacoes, estacoes, sizeof(estacao_t) * numEstacoes);
        r.transbordoPacotes = pacotesTransbordo;
        r.transbordoBytes   = bytesTransbordo;

        pacotesTotal = 0; bytesTotal = 0; pacotesForte = 0;
        somaRSSI = 0; maiorRSSI = -127; menorRSSI = 0;
        numEstacoes = 0; pacotesTransbordo = 0; bytesTransbordo = 0;
        portEXIT_CRITICAL(&mux);

        r.timestamp = obterTimestamp();

        if (saidaAoVivo) {
            imprimirRelatorio(Serial, r);
            if (clienteAtivo && clienteAtivo.connected()) {
                imprimirRelatorio(clienteAtivo, r);
            }
        }

        if (gravando) {
            gravarLinhaCSV(r);
        }
    }
}