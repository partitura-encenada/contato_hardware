#include <WiFi.h>
#include "esp_wifi.h"

// Se true: mostra so os equips conhecidos, esconde "outras fontes",
// desliga o scan de redes Wi-Fi e para de variar de canal (fica so no
// canal de producao) - use quando so importa a saude do seu sistema,
// nao o ambiente ao redor.
const bool MODO_FILTRO_EQUIPS = true;
const int CANAL_PRODUCAO = 1; // ALTERAR se seu canal de producao mudar

// Canais varridos em sequencia quando MODO_FILTRO_EQUIPS = false: seu
// canal de producao + os que se sobrepoem com ele no espectro 2.4GHz.
// Canal 1 sobrepoe com 2,3,4,5 (so 1/6/11 sao mutuamente independentes).
const int CANAIS_SOBREPOSTOS[] = {1, 2, 3, 4, 5};

const int* CANAIS_MONITORADOS = MODO_FILTRO_EQUIPS ? &CANAL_PRODUCAO : CANAIS_SOBREPOSTOS;
const int NUM_CANAIS = MODO_FILTRO_EQUIPS
    ? 1
    : sizeof(CANAIS_SOBREPOSTOS) / sizeof(CANAIS_SOBREPOSTOS[0]);
int indiceCanalAtual = 0;

typedef struct {
    uint8_t mac[6];
    uint8_t id;
} equip_conhecido_t;

const equip_conhecido_t EQUIPS_CONHECIDOS[] = {
    {{0x1C, 0x69, 0x20, 0xA4, 0x14, 0x94}, 1},
    {{0x84, 0x1F, 0xE8, 0x1C, 0x72, 0x5C}, 2},
    {{0x68, 0x25, 0xDD, 0x32, 0x88, 0xB4}, 3},
    {{0x14, 0x33, 0x5C, 0x52, 0x4D, 0xE0}, 4},
    {{0xF8, 0xB3, 0xB7, 0x50, 0xCC, 0xEC}, 7},
    {{0x1C, 0x69, 0x20, 0xA2, 0xE2, 0x14}, 8},
};
const int NUM_EQUIPS_CONHECIDOS = sizeof(EQUIPS_CONHECIDOS) / sizeof(EQUIPS_CONHECIDOS[0]);

typedef struct {
    uint16_t frame_ctrl;
    uint16_t duration_id;
    uint8_t  addr1[6];
    uint8_t  addr2[6];
    uint8_t  addr3[6];
    uint16_t seq_ctrl;
} wifi_ieee80211_mac_hdr_t;

typedef struct {
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0];
} wifi_ieee80211_packet_t;

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

uint32_t pacotesTotal = 0;
uint32_t pacotesForte = 0;
uint32_t bytesTotal   = 0;
int32_t  somaRSSI     = 0;
int8_t   maiorRSSI    = -127;
int8_t   menorRSSI    = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

bool macIgual(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 6) == 0;
}

int idParaMac(const uint8_t *mac) {
    for (int i = 0; i < NUM_EQUIPS_CONHECIDOS; i++) {
        if (macIgual(mac, EQUIPS_CONHECIDOS[i].mac)) {
            return EQUIPS_CONHECIDOS[i].id;
        }
    }
    return -1;
}

void sniffer(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA && type != WIFI_PKT_CTRL) {
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

struct RelatorioJanela {
    int      canal;
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
};

void imprimirRelatorio(const RelatorioJanela &r) {
    Serial.println("----------------------------------------");
    Serial.print("Canal: "); Serial.println(r.canal);

    if (!MODO_FILTRO_EQUIPS) {
        Serial.print("Pacotes/s (canal todo): "); Serial.println(r.pacotesTotal);
        Serial.print("Bytes/s (canal todo):   "); Serial.println(r.bytesTotal);

        if (r.pacotesTotal > 0) {
            Serial.print("RSSI medio: "); Serial.print((float)r.rssiSoma / r.pacotesTotal); Serial.println(" dBm");
            Serial.print("RSSI (max/min): "); Serial.print(r.rssiMax); Serial.print(" / "); Serial.println(r.rssiMin);
            Serial.print("Pacotes > -60 dBm: "); Serial.println(r.pacotesForte);
        }
    }

    Serial.println();
    Serial.println("Por origem (equips do contato):");

    uint32_t pacotesConhecidos = 0;
    for (int i = 0; i < r.numEstacoes; i++) {
        int id = idParaMac(r.estacoes[i].mac);
        if (id == -1) continue;
        pacotesConhecidos += r.estacoes[i].pacotes;

        Serial.print("  Equip ID "); Serial.print(id);
        Serial.print("  pacotes/s: "); Serial.print(r.estacoes[i].pacotes);
        Serial.print("  bytes/s: "); Serial.print(r.estacoes[i].bytes);
        Serial.print("  RSSI medio: "); Serial.print((float)r.estacoes[i].somaRSSI / r.estacoes[i].pacotes);
        Serial.println(" dBm");
    }
    if (pacotesConhecidos == 0) Serial.println("  (nenhum pacote de equip conhecido nesta janela)");

    if (!MODO_FILTRO_EQUIPS) {
        uint32_t outrosPacotes = r.transbordoPacotes;
        uint32_t outrosBytes   = r.transbordoBytes;
        int outrasFontes       = 0;
        for (int i = 0; i < r.numEstacoes; i++) {
            if (idParaMac(r.estacoes[i].mac) != -1) continue;
            outrosPacotes += r.estacoes[i].pacotes;
            outrosBytes   += r.estacoes[i].bytes;
            outrasFontes++;
        }

        Serial.println();
        Serial.print("Outras fontes: "); Serial.print(outrasFontes);
        Serial.print(" MACs, "); Serial.print(outrosPacotes);
        Serial.print(" pacotes/s, "); Serial.print(outrosBytes); Serial.println(" bytes/s");

        if (r.pacotesTotal > 0) {
            float pct = 100.0f * pacotesConhecidos / r.pacotesTotal;
            Serial.print("Fatia do canal usada pelos equips do contato: "); Serial.print(pct, 1); Serial.println("%");
        }
    }
    Serial.println();
}

void escanearRedes() {
    // Scan usa o mesmo radio do sniffer - pausa a captura de pacotes
    // por alguns segundos enquanto varre todos os canais.
    esp_wifi_set_promiscuous(false);

    int n = WiFi.scanNetworks();

    Serial.println("==== Redes Wi-Fi visiveis ====");
    if (n <= 0) {
        Serial.println("Nenhuma rede encontrada.");
    } else {
        for (int i = 0; i < n; i++) {
            Serial.print("Canal "); Serial.print(WiFi.channel(i));
            Serial.print(": \""); Serial.print(WiFi.SSID(i));
            Serial.print("\" (RSSI: "); Serial.print(WiFi.RSSI(i));
            Serial.println(")");
        }
    }
    Serial.println("===============================");
    Serial.println();

    WiFi.scanDelete();

    // Volta pro canal em que estava e retoma a captura de pacotes.
    esp_wifi_set_channel(CANAIS_MONITORADOS[indiceCanalAtual], WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(true);
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(false);
    esp_wifi_set_channel(CANAIS_MONITORADOS[0], WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous_rx_cb(&sniffer);
    esp_wifi_set_promiscuous(true);

    Serial.println();
    Serial.println("================================");
    Serial.println(" MONITOR DE FLUXO - contato");
    Serial.println("================================");
    Serial.print("Canais varridos: ");
    for (int i = 0; i < NUM_CANAIS; i++) {
        Serial.print(CANAIS_MONITORADOS[i]);
        if (i < NUM_CANAIS - 1) Serial.print(", ");
    }
    Serial.println();
    Serial.println();
}

void loop() {
    static uint32_t ultimo = 0;
    static uint32_t ultimoScan = 0;
    const uint32_t INTERVALO_SCAN_MS = 30000; // varre as redes visiveis a cada 30s

    if (!MODO_FILTRO_EQUIPS && millis() - ultimoScan >= INTERVALO_SCAN_MS) {
        ultimoScan = millis();
        escanearRedes();
    }

    if (millis() - ultimo >= 1000) {
        ultimo = millis();

        RelatorioJanela r;
        r.canal = CANAIS_MONITORADOS[indiceCanalAtual];

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

        imprimirRelatorio(r);

        indiceCanalAtual = (indiceCanalAtual + 1) % NUM_CANAIS;
        esp_wifi_set_channel(CANAIS_MONITORADOS[indiceCanalAtual], WIFI_SECOND_CHAN_NONE);
    }
}