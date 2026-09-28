#include <WiFi.h>
#include "esp_wifi.h"

// ═════════════════════════════════════════════════════════════════════════════
// Monitor passivo do canal ESP-NOW do Contato
// ═════════════════════════════════════════════════════════════════════════════
//
// Este firmware roda num ESP32 separado, ligado por USB ao PC. Ele NÃO faz parte
// do sistema (não é equip, base, ponte nem TDMA): apenas escuta o canal 11 em
// modo promíscuo, ou seja, recebe TODOS os quadros de rádio que passam no canal,
// inclusive os que não são endereçados a ele. Como só escuta, não atrapalha o
// sinal. A única vez que o rádio sai do canal é na varredura de redes Wi-Fi, que
// também é passiva (não envia nada).
//
// Passo a passo:
//   1. setup(): faz uma varredura de redes Wi-Fi (quem mais está no ar?) e depois
//      liga o modo promíscuo no canal 11.
//   2. sniffer(): chamado pelo driver Wi-Fi para CADA quadro captado. Descobre de
//      quem é o quadro (equip, base, mestre TDMA, ACK ou fonte externa) e soma os
//      números na "janela" atual.
//   3. loop(): a cada JANELA_MS (1 s) copia a janela, zera os contadores e envia o
//      resultado pela serial em linhas CSV. A cada INTERVALO_SCAN_MS repete a
//      varredura de redes.
//   4. No PC, o comando `contato monitor` (contato_cli/monitor.py) lê essas linhas,
//      grava o arquivo de log e, ao final, escreve um resumo com indicadores.
//
// Linhas enviadas pela serial (campos separados por vírgula):
//   INFO       configuração do monitor (a cada 10 s e ao responder "ID?")
//   CANAL      ocupação do canal na janela e quanto cada tipo de tráfego usou
//   TDMA       beacons do mestre TDMA: quantidade, intervalo e posições puladas
//   EQUIP      números de cada equip conhecido (pacotes, perdas, atraso, toque...)
//   FONTE      fontes externas (fora do Contato) que mais ocuparam o canal
//   FIM_JANELA marca o fim do bloco de uma janela
//   WIFI       uma rede encontrada na varredura (canal, RSSI, nome)
//   SCAN       início/fim da varredura de redes
// Também responde ao comando "ID?" com "ID/MONITOR", para o PC achar a porta.

// ═════════ Configuração ═════════
const int      CANAL             = 11;
const uint32_t SLOT_US           = 1500;    // duração de cada posição; igual ao TDMA.cpp
const int      NUM_SLOTS         = 6;       // número de posições; igual ao TDMA.cpp
const uint32_t JANELA_MS         = 1000;    // de quanto em quanto tempo os números são enviados
const uint32_t INTERVALO_SCAN_MS = 300000;  // varredura de redes Wi-Fi a cada 5 min; 0 = só ao ligar
const uint32_t INTERVALO_INFO_MS = 10000;   // reenvia a linha INFO (para o PC que conectar depois)
const uint32_t ORFAO_MS          = 5000;    // equip transmitindo sem controle da base há mais que isso = órfão
const int      TAM_BEACON        = 8;       // sizeof(beacon_t) do TDMA.cpp (slot_atual + timestamp)
const int      MAX_FONTES        = 24;      // quantas fontes externas diferentes são contadas por janela
const int      FONTES_REPORTADAS = 5;       // quantas delas vão para o log (as que mais ocupam)

// ═════════ Equips conhecidos ═════════
// Cada equip transmite para a sua base. O monitor usa o MAC para saber de quem é
// cada quadro e a posição (slot) para medir se o equip respeita o TDMA.
typedef struct {
    uint8_t id;
    uint8_t slot;           // MEU_SLOT do equip_N.cpp
    uint8_t mac[6];         // MAC do equip
    uint8_t macBase[6];     // MAC da base que recebe esse equip
} equip_conhecido_t;

// mac = macTransmissor de base_N.cpp; macBase = broadcastAddress de equip_N.cpp
// ATUALIZAR aqui se trocar a placa de algum equip ou base.
const equip_conhecido_t EQUIPS[] = {
    {1, 0, {0x80, 0xF3, 0xDA, 0x61, 0xCD, 0xAC}, {0x84, 0x1F, 0xE8, 0x1B, 0xA0, 0x48}},
    {2, 1, {0x1C, 0x69, 0x20, 0xA3, 0xF0, 0xBC}, {0x84, 0x1F, 0xE8, 0x15, 0xF2, 0x00}},
    {3, 2, {0x68, 0x25, 0xDD, 0x32, 0x88, 0xB4}, {0x84, 0x1F, 0xE8, 0x1A, 0x83, 0x1C}},
    {4, 3, {0x14, 0x33, 0x5C, 0x52, 0x4D, 0xE0}, {0xCC, 0xDB, 0xA7, 0x91, 0x6D, 0x9C}},
    {5, 4, {0x1C, 0x69, 0x20, 0xA4, 0x14, 0x94}, {0x88, 0x57, 0x21, 0xAD, 0x59, 0x40}},
    {6, 5, {0x84, 0x1F, 0xE8, 0x1B, 0xBD, 0x40}, {0x14, 0x08, 0x08, 0xA4, 0x59, 0xE8}},
};
const int NUM_EQUIPS = sizeof(EQUIPS) / sizeof(EQUIPS[0]);

// ═════════ Contadores de uma janela ═════════
// Tudo que é somado durante 1 s. No fim da janela é copiado, enviado e zerado.

// Números de um equip na janela
typedef struct {
    uint32_t pacotes;         // quadros novos (sem bit de retry)
    uint32_t retries;         // retransmissões: a tentativa anterior não recebeu ACK da base
    uint32_t acks;            // ACKs que a base mandou para o equip (= "recebi")
    uint32_t perdidos;        // buracos no número de sequência 802.11 (quadros que o monitor não viu)
    int32_t  somaRssi;        // soma do RSSI de todos os quadros (para a média)
    int8_t   minRssi;         // pior RSSI da janela (0 = nenhum ainda)
    uint32_t somaAtraso;      // soma dos atrasos: do beacon da posição do equip até o quadro do equip
    uint32_t maxAtraso;
    uint32_t amostrasAtraso;
    uint32_t foraSlot;        // quantas vezes o atraso passou de SLOT_US (transmitiu na posição de outro)
    uint32_t trocasToque;     // quantas vezes o campo touch mudou (0→1 ou 1→0)
    uint32_t controles;       // controle_t (ativo=0/1) que a base mandou ao equip
    uint8_t  taxa;            // taxa de transmissão do equip (codificação de rx_ctrl.rate; 0xFF = 802.11n)
} janela_equip_t;

// Números do mestre TDMA na janela
typedef struct {
    uint32_t beacons;           // beacons captados
    uint32_t somaIntervalo;     // soma do tempo entre beacons consecutivos
    uint32_t maxIntervalo;
    uint32_t amostrasIntervalo;
    uint32_t slotsPulados;      // posições que deveriam ter vindo e não vieram (beacon perdido/atrasado)
    int32_t  somaRssi;
    uint8_t  mac[6];            // MAC do mestre TDMA visto
} janela_tdma_t;

// Números do canal inteiro na janela. "ar" = tempo no ar estimado, em µs.
// Dividindo pelo tempo da janela temos a ocupação do canal em %.
typedef struct {
    uint32_t pacotes;
    uint32_t arTotal;
    uint32_t arEquips;        // quadros de dados dos equips
    uint32_t arBeacons;       // beacons do mestre TDMA
    uint32_t arBases;         // controles das bases
    uint32_t arAcks;          // ACKs trocados entre equips e bases
    uint32_t arOutros;        // tudo que não é do Contato (roteadores, celulares...)
    int32_t  somaRssi;
    int32_t  somaRuido;       // ruído de fundo medido pelo rádio
} janela_canal_t;

// Uma fonte externa (MAC que não é do Contato)
typedef struct {
    uint8_t  mac[6];
    uint32_t pacotes;
    uint32_t arUs;
    int32_t  somaRssi;
    bool     espnow;          // true = é ESP-NOW mas não está na tabela (ex: equip variante B, ponte)
} fonte_t;

// A janela completa
typedef struct {
    janela_canal_t canal;
    janela_tdma_t  tdma;
    janela_equip_t equips[NUM_EQUIPS];
    fonte_t        fontes[MAX_FONTES];
    int            numFontes;
    uint32_t       fontesTransbordo;  // quadros de fontes que não couberam na tabela
} janela_t;

// ═════════ Estado que atravessa as janelas ═════════
// Não é zerado a cada segundo, porque depende do quadro anterior.

typedef struct {
    int32_t  ultimaSeq;         // último número de sequência visto; -1 = ainda não visto
    int8_t   ultimoToque;       // último valor de touch; -1 = ainda não visto
    uint32_t ultimoControleMs;  // millis() do último controle da base; 0 = nunca
} estado_equip_t;

typedef struct {
    int32_t  ultimoSlot;              // posição do último beacon; -1 = ainda não visto
    uint32_t ultimoTs;                // instante (µs) do último beacon
    uint32_t tsSlot[NUM_SLOTS];       // instante do último beacon de cada posição
    bool     tsSlotValido[NUM_SLOTS];
} estado_tdma_t;

janela_t       janela;                    // janela sendo preenchida pelo sniffer
janela_t       copia;                     // cópia da janela que terminou, usada para imprimir
estado_equip_t estadoEquips[NUM_EQUIPS];
estado_tdma_t  estadoTdma;
uint32_t       controleCopia[NUM_EQUIPS]; // ultimoControleMs no momento da cópia

// O sniffer roda na tarefa do Wi-Fi e o loop() em outra; o mux impede que os dois
// mexam na janela ao mesmo tempo.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t inicioJanela = 0;
uint32_t ultimoScan   = 0;
uint32_t ultimoInfo   = 0;

// ═════════ Funções auxiliares ═════════

bool macIgual(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 6) == 0;
}

// Índice do equip com esse MAC na tabela EQUIPS, ou -1
int equipPorMac(const uint8_t *mac) {
    for (int i = 0; i < NUM_EQUIPS; i++) {
        if (macIgual(mac, EQUIPS[i].mac)) return i;
    }
    return -1;
}

// Índice do equip cuja BASE tem esse MAC, ou -1
int equipPorMacBase(const uint8_t *mac) {
    for (int i = 0; i < NUM_EQUIPS; i++) {
        if (macIgual(mac, EQUIPS[i].macBase)) return i;
    }
    return -1;
}

// Taxa em Mbps dos códigos OFDM (802.11g) de rx_ctrl.rate
int taxaOfdmMbps(uint8_t codigo) {
    switch (codigo) {
        case 0x0B: return 6;
        case 0x0F: return 9;
        case 0x0A: return 12;
        case 0x0E: return 18;
        case 0x09: return 24;
        case 0x0D: return 36;
        case 0x08: return 48;
        case 0x0C: return 54;
        default:   return 6;
    }
}

// Nome legível da taxa (mesmos nomes de WIFI_PHY_RATE_* usados nos firmwares)
const char *taxaTexto(uint8_t codigo) {
    switch (codigo) {
        case 0x00: return "1M_L";
        case 0x01: return "2M_L";
        case 0x02: return "5.5M_L";
        case 0x03: return "11M_L";
        case 0x05: return "2M_S";
        case 0x06: return "5.5M_S";
        case 0x07: return "11M_S";
        case 0x0B: return "6M";
        case 0x0F: return "9M";
        case 0x0A: return "12M";
        case 0x0E: return "18M";
        case 0x09: return "24M";
        case 0x0D: return "36M";
        case 0x08: return "48M";
        case 0x0C: return "54M";
        case 0xFF: return "HT";
        default:   return "?";
    }
}

// Estimativa do tempo que o quadro ocupou o ar (preâmbulo + dados), em µs.
// Não conta a espera pelo canal livre, então a ocupação real é um pouco maior.
//   - 802.11b (1, 2, 5.5, 11 Mbps): preâmbulo de 192 µs (longo) ou 96 µs (curto)
//     + bits / taxa. É o caso do Contato hoje (1 Mbps, preâmbulo longo).
//   - 802.11g (OFDM, 6 a 54 Mbps): preâmbulo de 20 µs + símbolos de 4 µs.
//   - 802.11n (HT): aproximação com preâmbulo de 36 µs.
uint32_t tempoNoAr(const wifi_pkt_rx_ctrl_t &rx) {
    uint32_t bits = (uint32_t)rx.sig_len * 8;

    if (rx.sig_mode != 0) {
        static const uint16_t MCS_X10[] = {65, 130, 195, 260, 390, 520, 585, 650};  // Mbps × 10
        return 36 + bits * 10 / MCS_X10[rx.mcs & 7];
    }

    switch (rx.rate) {
        case 0x00: return 192 + bits;             // 1 Mbps, preâmbulo longo
        case 0x01: return 192 + bits / 2;
        case 0x02: return 192 + bits * 2 / 11;
        case 0x03: return 192 + bits / 11;
        case 0x05: return 96 + bits / 2;          // preâmbulo curto
        case 0x06: return 96 + bits * 2 / 11;
        case 0x07: return 96 + bits / 11;
        default: {
            uint32_t bitsPorSimbolo = 4 * taxaOfdmMbps(rx.rate);
            uint32_t simbolos = (16 + bits + 6 + bitsPorSimbolo - 1) / bitsPorSimbolo;
            return 20 + 4 * simbolos;
        }
    }
}

// ═════════ Registro de cada tipo de quadro ═════════
// Estas funções são chamadas pelo sniffer, já dentro do mux.

// Fonte externa: soma na tabela de fontes (procura o MAC; se não achar, cria)
void contarFonte(const uint8_t *mac, uint32_t ar, int8_t rssi, bool espnow) {
    for (int i = 0; i < janela.numFontes; i++) {
        if (macIgual(janela.fontes[i].mac, mac)) {
            janela.fontes[i].pacotes++;
            janela.fontes[i].arUs += ar;
            janela.fontes[i].somaRssi += rssi;
            janela.fontes[i].espnow |= espnow;
            return;
        }
    }
    if (janela.numFontes < MAX_FONTES) {
        fonte_t &f = janela.fontes[janela.numFontes++];
        memcpy(f.mac, mac, 6);
        f.pacotes  = 1;
        f.arUs     = ar;
        f.somaRssi = rssi;
        f.espnow   = espnow;
    } else {
        janela.fontesTransbordo++;
    }
}

// Beacon do mestre TDMA. O corpo é o beacon_t: corpo[0] = slot_atual.
// Mede o intervalo entre beacons e se alguma posição foi pulada, e guarda o
// instante do beacon de cada posição para medir o atraso dos equips depois.
void registrarBeacon(const uint8_t *origem, const uint8_t *corpo, uint32_t ar, int8_t rssi, uint32_t ts) {
    janela_tdma_t &t = janela.tdma;
    janela.canal.arBeacons += ar;
    t.beacons++;
    t.somaRssi += rssi;
    memcpy(t.mac, origem, 6);

    uint8_t slot = corpo[0];

    if (estadoTdma.ultimoSlot >= 0) {
        uint32_t intervalo = ts - estadoTdma.ultimoTs;
        if (intervalo < 100000) {  // ignora buracos grandes (ex: depois da varredura de redes)
            t.somaIntervalo += intervalo;
            t.amostrasIntervalo++;
            if (intervalo > t.maxIntervalo) t.maxIntervalo = intervalo;

            // Os beacons vêm em sequência 0,1,2,...,5,0,1... Se veio outro número,
            // o monitor não viu os do meio (perdidos ou nunca enviados).
            uint8_t esperado = (estadoTdma.ultimoSlot + 1) % NUM_SLOTS;
            if (slot < NUM_SLOTS && slot != esperado) {
                t.slotsPulados += (slot - esperado + NUM_SLOTS) % NUM_SLOTS;
            }
        }
    }

    estadoTdma.ultimoSlot = slot;
    estadoTdma.ultimoTs   = ts;
    if (slot < NUM_SLOTS) {
        estadoTdma.tsSlot[slot]       = ts;
        estadoTdma.tsSlotValido[slot] = true;
    }
}

// Quadro enviado por um equip conhecido (índice e na tabela EQUIPS)
void registrarEquip(int e, const wifi_pkt_rx_ctrl_t &rx, bool retry, uint16_t seq,
                    bool espnow, const uint8_t *corpo, int tamCorpo, uint32_t ar) {
    janela_equip_t &j = janela.equips[e];
    estado_equip_t &s = estadoEquips[e];
    int8_t rssi = rx.rssi;

    janela.canal.arEquips += ar;
    j.somaRssi += rssi;
    if (j.minRssi == 0 || rssi < j.minRssi) j.minRssi = rssi;
    j.taxa = (rx.sig_mode == 0) ? rx.rate : 0xFF;

    // Bit de retry ligado = o equip está reenviando porque a base não confirmou.
    // Muitos retries = colisões, sinal fraco ou base desligada.
    if (retry) {
        j.retries++;
        return;
    }

    j.pacotes++;

    // Cada quadro novo do equip leva um número de sequência (0 a 4095) que
    // aumenta de 1 em 1. Se pulou, o monitor deixou de ver quadros.
    if (s.ultimaSeq >= 0) {
        uint16_t esperado = (s.ultimaSeq + 1) & 0x0FFF;
        uint16_t diff = (seq - esperado) & 0x0FFF;
        if (diff < 200) j.perdidos += diff;  // saltos enormes = reinício do equip, não perda
    }
    s.ultimaSeq = seq;

    // Lê o campo touch da message_t para contar oscilações do toque.
    // message_t padrão tem 12 bytes (touch no byte 8, por causa do alinhamento);
    // a variante B tem 20 bytes (touch no byte 16).
    int posToque = !espnow ? -1 : (tamCorpo == 12) ? 8 : (tamCorpo == 20) ? 16 : -1;
    if (posToque >= 0) {
        int8_t toque = corpo[posToque] ? 1 : 0;
        if (s.ultimoToque >= 0 && toque != s.ultimoToque) j.trocasToque++;
        s.ultimoToque = toque;
    }

    // Atraso = tempo entre o beacon da posição do equip e o quadro do equip.
    // Se passou de SLOT_US, o equip transmitiu na posição de outro (TDMA não respeitado).
    uint8_t slot = EQUIPS[e].slot;
    if (slot < NUM_SLOTS && estadoTdma.tsSlotValido[slot]) {
        uint32_t atraso = rx.timestamp - estadoTdma.tsSlot[slot];
        if (atraso < 100000) {
            j.somaAtraso += atraso;
            j.amostrasAtraso++;
            if (atraso > j.maxAtraso) j.maxAtraso = atraso;
            if (atraso > SLOT_US) j.foraSlot++;
        }
    }
}

// ═════════ Sniffer ═════════
// Chamado pelo driver Wi-Fi para cada quadro captado no canal. Precisa ser
// rápido: só classifica o quadro e soma contadores; nada de Serial aqui.
void sniffer(void *buf, wifi_promiscuous_pkt_type_t tipo) {
    if (tipo == WIFI_PKT_MISC) return;

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    const wifi_pkt_rx_ctrl_t &rx = pkt->rx_ctrl;  // metadados do rádio: RSSI, taxa, tamanho, instante
    const uint8_t *p = pkt->payload;              // o quadro 802.11 em si
    uint16_t len = rx.sig_len;
    if (len < 10) return;

    int8_t   rssi = rx.rssi;
    uint32_t ar   = tempoNoAr(rx);

    portENTER_CRITICAL(&mux);

    // Todo quadro conta para a ocupação do canal
    janela.canal.pacotes++;
    janela.canal.arTotal   += ar;
    janela.canal.somaRssi  += rssi;
    janela.canal.somaRuido += rx.noise_floor;

    // Quadro de controle: só pedimos ACKs (ver setup). O ACK tem apenas o
    // endereço de quem está sendo confirmado (bytes 4..9).
    if (tipo == WIFI_PKT_CTRL) {
        const uint8_t *destino = p + 4;
        int e = equipPorMac(destino);
        if (p[0] == 0xD4 && (e >= 0 || equipPorMacBase(destino) >= 0)) {
            janela.canal.arAcks += ar;
            if (e >= 0) janela.equips[e].acks++;  // base confirmou um quadro do equip
        } else {
            janela.canal.arOutros += ar;
        }
        portEXIT_CRITICAL(&mux);
        return;
    }

    if (len < 28) {
        janela.canal.arOutros += ar;
        portEXIT_CRITICAL(&mux);
        return;
    }

    // Cabeçalho 802.11: byte 1 = flags (bit 3 = retry), bytes 4..9 = destino,
    // 10..15 = origem, 22..23 = número de sequência (12 bits de cima).
    const uint8_t *destino = p + 4;
    const uint8_t *origem  = p + 10;
    bool     retry = p[1] & 0x08;
    uint16_t seq   = (p[22] | (p[23] << 8)) >> 4;

    // ESP-NOW é um "action frame" (0xD0) com categoria vendor (127), OUI da
    // Espressif (18:FE:34) e elemento do tipo 4. Os dados enviados pelo
    // esp_now_send (message_t, beacon_t, controle_t...) começam no byte 39.
    bool espnow = tipo == WIFI_PKT_MGMT && p[0] == 0xD0 && len >= 43
               && p[24] == 127 && p[25] == 0x18 && p[26] == 0xFE && p[27] == 0x34
               && p[32] == 0xDD && p[37] == 0x04;
    const uint8_t *corpo = p + 39;
    int tamCorpo = espnow ? (int)p[33] - 5 : 0;  // tamanho dos dados do esp_now_send

    static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    int e = equipPorMac(origem);
    int b = (e >= 0) ? -1 : equipPorMacBase(origem);

    if (e >= 0) {
        // veio de um equip conhecido
        registrarEquip(e, rx, retry, seq, espnow, corpo, tamCorpo, ar);
    } else if (espnow && tamCorpo == TAM_BEACON && macIgual(destino, BROADCAST)) {
        // ESP-NOW de 8 bytes em broadcast = beacon do mestre TDMA
        registrarBeacon(origem, corpo, ar, rssi, rx.timestamp);
    } else if (b >= 0) {
        // veio de uma base; 1 byte de dados = controle_t (START/STOP para o equip)
        janela.canal.arBases += ar;
        if (espnow && tamCorpo == 1 && !retry) {
            janela.equips[b].controles++;
            uint32_t agora = millis();
            estadoEquips[b].ultimoControleMs = agora ? agora : 1;
        }
    } else {
        // qualquer outra coisa: roteador, celular, ponte, equip fora da tabela...
        janela.canal.arOutros += ar;
        contarFonte(origem, ar, rssi, espnow);
    }

    portEXIT_CRITICAL(&mux);
}

// Zera a janela e o estado que depende do quadro anterior. Usado ao ligar e
// depois da varredura de redes (quando o rádio saiu do canal e perdeu quadros).
void zerarEstado() {
    memset(&janela, 0, sizeof(janela));
    estadoTdma.ultimoSlot = -1;
    for (int s = 0; s < NUM_SLOTS; s++) estadoTdma.tsSlotValido[s] = false;
    for (int i = 0; i < NUM_EQUIPS; i++) {
        estadoEquips[i].ultimaSeq   = -1;
        estadoEquips[i].ultimoToque = -1;
    }
}

// ═════════ Saída pela serial ═════════

// Tempo no ar (µs) → % da janela
float porcento(uint32_t arUs, uint32_t duracaoMs) {
    return duracaoMs ? arUs / (duracaoMs * 10.0f) : 0.0f;
}

void imprimirMac(const uint8_t *mac) {
    Serial.printf("%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// INFO,ms,monitor,canal=..,slot_us=..,num_slots=..,janela_ms=..,equips=..
void imprimirInfo() {
    Serial.printf("INFO,%lu,monitor,canal=%d,slot_us=%lu,num_slots=%d,janela_ms=%lu,equips=%d\n",
                  (unsigned long)millis(), CANAL, (unsigned long)SLOT_US, NUM_SLOTS,
                  (unsigned long)JANELA_MS, NUM_EQUIPS);
}

// Envia todas as linhas de uma janela: CANAL, TDMA, EQUIP (um por equip visto),
// FONTE (as maiores) e FIM_JANELA.
void imprimirJanela(const janela_t &j, uint32_t duracao, uint32_t agora) {
    unsigned long t = agora;
    const janela_canal_t &c = j.canal;

    // CANAL,ms,duracao_ms,pacotes,ocupacao_pct,equips_pct,beacons_pct,bases_pct,acks_pct,outros_pct,rssi_med,ruido_med
    Serial.printf("CANAL,%lu,%lu,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                  t, (unsigned long)duracao, (unsigned long)c.pacotes,
                  porcento(c.arTotal, duracao), porcento(c.arEquips, duracao),
                  porcento(c.arBeacons, duracao), porcento(c.arBases, duracao),
                  porcento(c.arAcks, duracao), porcento(c.arOutros, duracao),
                  c.pacotes ? (float)c.somaRssi / c.pacotes : 0.0f,
                  c.pacotes ? (float)c.somaRuido / c.pacotes : 0.0f);

    // TDMA,ms,beacons,esperados,intervalo_med_us,intervalo_max_us,slots_pulados,rssi_med,mac
    // esperados = quantos beacons caberiam na janela se o TDMA mantivesse 1 a cada SLOT_US
    const janela_tdma_t &td = j.tdma;
    Serial.printf("TDMA,%lu,%lu,%lu,%lu,%lu,%lu,%.1f,",
                  t, (unsigned long)td.beacons, (unsigned long)(duracao * 1000UL / SLOT_US),
                  (unsigned long)(td.amostrasIntervalo ? td.somaIntervalo / td.amostrasIntervalo : 0),
                  (unsigned long)td.maxIntervalo, (unsigned long)td.slotsPulados,
                  td.beacons ? (float)td.somaRssi / td.beacons : 0.0f);
    imprimirMac(td.mac);
    Serial.println();

    // EQUIP,ms,id,pacotes,perdidos,retries,acks,rssi_med,rssi_min,atraso_med_us,atraso_max_us,
    //       amostras_atraso,fora_slot,trocas_toque,controles,orfao,taxa
    // Só imprime equips com alguma atividade na janela.
    for (int i = 0; i < NUM_EQUIPS; i++) {
        const janela_equip_t &e = j.equips[i];
        uint32_t quadros = e.pacotes + e.retries;
        if (quadros == 0 && e.controles == 0 && e.acks == 0) continue;

        // Órfão: o equip está transmitindo, mas a base não manda controle há mais
        // de ORFAO_MS (a base reenvia o controle a cada 2 s enquanto está ativa).
        bool orfao = quadros > 0
                  && (controleCopia[i] == 0 || agora - controleCopia[i] > ORFAO_MS);

        Serial.printf("EQUIP,%lu,%d,%lu,%lu,%lu,%lu,%.1f,%d,%lu,%lu,%lu,%lu,%lu,%lu,%d,%s\n",
                      t, EQUIPS[i].id, (unsigned long)e.pacotes, (unsigned long)e.perdidos,
                      (unsigned long)e.retries, (unsigned long)e.acks,
                      quadros ? (float)e.somaRssi / quadros : 0.0f, e.minRssi,
                      (unsigned long)(e.amostrasAtraso ? e.somaAtraso / e.amostrasAtraso : 0),
                      (unsigned long)e.maxAtraso, (unsigned long)e.amostrasAtraso,
                      (unsigned long)e.foraSlot, (unsigned long)e.trocasToque,
                      (unsigned long)e.controles, orfao ? 1 : 0,
                      quadros ? taxaTexto(e.taxa) : "-");
    }

    // FONTE,ms,mac,pacotes,ocupacao_pct,rssi_med,espnow
    // As FONTES_REPORTADAS fontes externas que mais ocuparam o canal (da maior para a menor).
    bool usada[MAX_FONTES] = {false};
    for (int n = 0; n < FONTES_REPORTADAS && n < j.numFontes; n++) {
        int maior = -1;
        for (int i = 0; i < j.numFontes; i++) {
            if (usada[i]) continue;
            if (maior < 0 || j.fontes[i].arUs > j.fontes[maior].arUs) maior = i;
        }
        usada[maior] = true;
        const fonte_t &f = j.fontes[maior];
        Serial.printf("FONTE,%lu,", t);
        imprimirMac(f.mac);
        Serial.printf(",%lu,%.1f,%.1f,%d\n",
                      (unsigned long)f.pacotes, porcento(f.arUs, duracao),
                      (float)f.somaRssi / f.pacotes, f.espnow ? 1 : 0);
    }

    // FIM_JANELA,ms — o PC usa esta linha para saber que a janela terminou
    Serial.printf("FIM_JANELA,%lu\n", t);
}

// ═════════ Varredura de redes Wi-Fi ═════════
// Lista as redes de todos os canais (nome, canal, RSSI). É passiva: o rádio
// só escuta os beacons dos roteadores, sem enviar nada. Leva ~3 s, durante os
// quais o monitor fica fora do canal 11; por isso a janela parcial é descartada.
// WIFI,ms,canal,rssi,ssid  (vírgulas no nome da rede viram ';')
void escanearRedes() {
    Serial.printf("SCAN,%lu,inicio\n", (unsigned long)millis());
    esp_wifi_set_promiscuous(false);

    int n = WiFi.scanNetworks(false, true, true, 200);  // síncrona, inclui ocultas, passiva, 200 ms/canal
    unsigned long t = millis();
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        ssid.replace(",", ";");
        if (ssid.length() == 0) ssid = "(oculta)";
        Serial.printf("WIFI,%lu,%d,%d,%s\n", t, WiFi.channel(i), WiFi.RSSI(i), ssid.c_str());
    }
    WiFi.scanDelete();
    Serial.printf("SCAN,%lu,fim,%d\n", (unsigned long)millis(), n < 0 ? 0 : n);

    esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

    // o rádio saiu do canal durante a varredura: descarta a janela parcial
    portENTER_CRITICAL(&mux);
    zerarEstado();
    portEXIT_CRITICAL(&mux);

    esp_wifi_set_promiscuous(true);
}

// ═════════ Comandos pela serial ═════════
// "ID?" → "ID/MONITOR" + INFO. É assim que o `contato monitor` reconhece a porta.
// (O `contato scan-com` também manda "ID?", mas ignora a resposta por não ser número.)
void lerComandos() {
    static char cmd[16];
    static int n = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd[n] = '\0';
            if (strcmp(cmd, "ID?") == 0) {
                Serial.println("ID/MONITOR");
                imprimirInfo();
            }
            n = 0;
        } else if (n < (int)sizeof(cmd) - 1) {
            cmd[n++] = c;
        }
    }
}

// ═════════ setup ═════════
void setup() {
    Serial.begin(115200);

    zerarEstado();
    for (int i = 0; i < NUM_EQUIPS; i++) estadoEquips[i].ultimoControleMs = 0;

    // Modo estação, sem conectar em rede nenhuma
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // 1) quem mais está no ar?
    imprimirInfo();
    escanearRedes();

    // 2) quais quadros o modo promíscuo entrega: gerenciamento (onde vive o
    //    ESP-NOW), dados (roteadores/celulares) e controle...
    wifi_promiscuous_filter_t filtro = {};
    filtro.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_CTRL;
    esp_wifi_set_promiscuous_filter(&filtro);

    // ...e, dos de controle, só os ACKs (confirmação de recebimento)
    wifi_promiscuous_filter_t filtroCtrl = {};
    filtroCtrl.filter_mask = WIFI_PROMIS_CTRL_FILTER_MASK_ACK;
    esp_wifi_set_promiscuous_ctrl_filter(&filtroCtrl);

    // 3) liga a escuta no canal do Contato
    esp_wifi_set_promiscuous_rx_cb(&sniffer);
    esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(true);

    ultimoScan   = millis();
    ultimoInfo   = millis();
    inicioJanela = millis();
}

// ═════════ loop ═════════
void loop() {
    lerComandos();

    uint32_t agora = millis();

    // Varredura periódica de redes
    if (INTERVALO_SCAN_MS > 0 && agora - ultimoScan >= INTERVALO_SCAN_MS) {
        escanearRedes();
        ultimoScan   = millis();
        inicioJanela = millis();
        return;
    }

    // INFO periódico, para quem conectar no meio
    if (agora - ultimoInfo >= INTERVALO_INFO_MS) {
        ultimoInfo = agora;
        imprimirInfo();
    }

    // Fim da janela: copia e zera dentro do mux (rápido) e imprime fora dele
    // (lento), para não travar o sniffer enquanto a serial escreve.
    if (agora - inicioJanela >= JANELA_MS) {
        uint32_t duracao = agora - inicioJanela;
        inicioJanela = agora;

        portENTER_CRITICAL(&mux);
        memcpy(&copia, &janela, sizeof(janela));
        memset(&janela, 0, sizeof(janela));
        for (int i = 0; i < NUM_EQUIPS; i++) controleCopia[i] = estadoEquips[i].ultimoControleMs;
        portEXIT_CRITICAL(&mux);

        imprimirJanela(copia, duracao, agora);
    }

    delay(5);
}
