# Contato (hardware)
[![en](https://img.shields.io/badge/lang-en-red.svg)](README.en.md)

Código embarcado para o dispositivo **Contato**, desenvolvido pelo curso de Dança da Universidade Federal do Rio de Janeiro em parceria com o Parque Tecnológico UFRJ.

O sistema é baseado em módulos **ESP32 DEVKIT V1**. Cada **Equip** (vestível) leva um **IMU MPU6050** (giroscópio + acelerômetro, 6 graus de liberdade) e um sensor de toque capacitivo, e envia os dados do movimento por **ESP-NOW** para uma **Base** ligada por USB ao computador. O software **[contato_cli](../contato_cli)** lê a Base pela porta serial e converte os dados em **MIDI**.

## Conteúdo

* Arquitetura
* Como funciona
* Protocolo
* Organização
* Build e upload
* Atualização por rádio (OTA)
* Calibração
* Equip e Base 6DOF (diagnóstico)
* Hardware

## Arquitetura

```text
Equip (ESP32 + MPU6050)
        ↓ ESP-NOW (canal 11)
Base (ESP32 USB)
        ↓ Serial (115200 baud)
contato_cli
        ↓ MIDI
DAW / Instrumentos Virtuais
```

Há também a **Ponte** (`ponte.cpp`), um ESP32 ligado ao PC usado apenas para enviar firmware aos Equips e às Bases pelo ar (OTA), sem cabo.

## Como funciona

**Equip** (`equip_N.cpp`)

- Usa o DMP do MPU6050 para obter o quaternion e calcula o ângulo de rolagem (*roll*, em graus) e a aceleração linear no eixo X.
- Lê o sensor capacitivo (pino `T3`); o toque é detectado quando a leitura fica abaixo de `touch_sensitivity`.
- Só transmite depois de receber o comando de controle da Base (`ativo = 1`). O LED azul (GPIO 2) acende quando há transmissão ativa e toque.
- Os offsets de calibração do MPU6050 ficam fixos no código de cada `equip_N.cpp`.

**Base** (`base_N.cpp`)

- Recebe os pacotes do Equip cujo MAC está configurado em `macTransmissor` e descarta os demais.
- Responde aos comandos recebidos pela serial:

| Comando | Ação |
|---|---|
| `START` | Ativa a saída serial e manda o Equip começar a transmitir (reenvia a cada 2 s) |
| `STOP` | Desativa a saída serial e manda o Equip parar |
| `ID?` | Responde `ID/<BASE_ID>` |

- Cada leitura é escrita na serial como uma linha `id/gyro/accel/touch`.

**Mestre TDMA** (`TDMA.cpp`, também usado como `src/main.cpp`)

- Envia em broadcast um *beacon* com o slot atual, percorrendo os slots dos Equips em ciclo (`NUM_EQUIPS = 6`, `SLOT_US = 1500` µs).
- Cada Equip só transmite quando o beacon indica o seu `MEU_SLOT`, evitando colisões entre vários Equips no mesmo canal.

## Protocolo

Mensagem do Equip para a Base (`message_t`):

| Campo | Tipo | Descrição |
|---|---|---|
| `id` | `uint8_t` | ID do Equip |
| `gyro` | `int16_t` | Ângulo de rolagem, em graus |
| `accel` | `int32_t` | Aceleração linear no eixo X |
| `touch` | `uint8_t` | 1 se o sensor capacitivo está tocado |

Outros pacotes ESP-NOW (canal 11, taxa 1 Mbps, sem criptografia):

- **Beacon** (`beacon_t`): `slot_atual` + `timestamp`, enviado pelo mestre TDMA.
- **Controle** (`controle_t`): `ativo` (0/1), enviado pela Base ao Equip.
- **OTA** (`ota_pacote_t`): pacotes `INICIO` (`0xAA`), `DADO` (`0xBB`) e `FIM` (`0xCC`), com até 230 bytes de dados cada.

## 📁 Organização

```text
contato_hardware/
├── arduino/              # implementações legadas (ESP-NOW P2P, sem manutenção ativa)
└── platformio/           # projeto ativo
    ├── platformio.ini
    ├── src/
    │   └── main.cpp      # cópia do script escolhido (padrão: TDMA.cpp)
    ├── include/
    │   ├── config.h      # constantes da versão BLE anterior (não usado pelos scripts atuais)
    │   ├── types.h       # structs de dados
    │   └── ota_receptor.h  # receptor de OTA usado por Equips, Bases e mestre TDMA
    ├── lib/              # MPU6050 e MadgwickAHRS
    ├── scripts/          # firmwares gravados nos dispositivos
    │   ├── equip_1..6.cpp
    │   ├── base_1..6.cpp
    │   ├── equip_6DOF.cpp / base_6DOF.cpp  # diagnóstico: 6 eixos do MPU6050
    │   ├── ponte.cpp
    │   ├── TDMA.cpp
    │   ├── monitor.cpp
    │   ├── B/            # variantes (equip_1B, equip_5B, base_1B, base_5B)
    │   └── upload_script.py
    └── util/             # calibração, benchmarks, modelos e tabela de MACs
```

## Build e upload

Requer o [PlatformIO](https://platformio.org/) (CLI ou extensão VSCode).

O firmware a gravar é escolhido pela variável de ambiente `SCRIPT`. Antes do build, o `scripts/upload_script.py` procura `<SCRIPT>.cpp` em `util/`, `scripts/` e `scripts/B/` e copia o arquivo para `src/main.cpp`.

```powershell
cd platformio

# Equip 1
$env:SCRIPT="equip_1"; pio run --target upload -e esp32doit-devkit-v1

# Base 1
$env:SCRIPT="base_1"; pio run --target upload -e esp32doit-devkit-v1

# Mestre TDMA
$env:SCRIPT="TDMA"; pio run --target upload -e esp32doit-devkit-v1

# Monitor serial (115200 baud)
pio device monitor --speed 115200
```

Se `SCRIPT` não estiver definida (por exemplo, um build feito pela IDE), o `src/main.cpp` é mantido como está.

> **Atenção:** o `upload_script.py` **sobrescreve** `src/main.cpp` a cada build com `SCRIPT` definida.

Cada Equip e cada Base têm o seu próprio arquivo (`equip_N.cpp`, `base_N.cpp`), que diferem em ID, slot e endereços MAC. Para criar um novo dispositivo, copie `util/equip_modelo.cpp` ou `util/base_modelo.cpp`. Os MACs das Bases estão em `platformio/util/README.md`.

## Atualização por rádio (OTA)

Os firmwares incluem `ota_receptor.h` e podem ser atualizados sem cabo USB. A `ponte.cpp` recebe o binário do PC e o envia por ESP-NOW ao dispositivo de destino, com confirmação de status a cada etapa.

O fluxo normalmente é feito pelo `contato_cli` (`contato ota`, `contato update-bases`), que compila o script e faz o envio. Para isso, a `ponte.cpp` precisa estar gravada no ESP32 conectado ao PC.

## Calibração

O `util/calibrate.cpp` calcula os offsets do MPU6050. O comando `contato calibrate` do `contato_cli` o executa no Equip e mostra os offsets obtidos. Depois, copie os valores para as chamadas `setXAccelOffset`, `setYAccelOffset`, `setZAccelOffset`, `setXGyroOffset`, `setYGyroOffset` e `setZGyroOffset` do `equip_N.cpp` correspondente.

## Equip e Base 6DOF (diagnóstico)

Par de firmwares para **ver todos os eixos do MPU6050**, usado só em testes. Funciona igual ao `equip_N`/`base_N` (TDMA, START/STOP, OTA, toque, LED), mas em vez de mandar só o *roll* e a aceleração em X, manda os **6 eixos**: 3 de rotação e 3 de aceleração.

| Arquivo | Função |
|---|---|
| `scripts/equip_6DOF.cpp` | Lê o sensor e envia os 6 eixos na posição do TDMA |
| `scripts/base_6DOF.cpp` | Recebe e repassa ao PC; é a mesma para os dois modos |

### Brutos ou tratados

Quem escolhe é a constante `RAW_DATA` no topo do `equip_6DOF.cpp`. A base não muda.

| `RAW_DATA` | Modo | `rot` (3 valores) | `acc` (3 valores) |
|---|---|---|---|
| `true` (padrão) | **B** — brutos | giroscópio X/Y/Z, ±2000 °/s → **÷ 16,4 = °/s** | acelerômetro X/Y/Z **com** gravidade, ±2 g → **÷ 16384 = g** |
| `false` | **T** — tratados pelo DMP | yaw/pitch/roll em **graus** | aceleração linear X/Y/Z **sem** gravidade → **÷ 8192 = g** |

- No modo **T** o tratamento é o mesmo do `equip_N`: o *roll* é o `gyro` e a aceleração X é o `accel` que o `equip_N` envia.
- No modo **B** os offsets de calibração continuam aplicados (ficam gravados no próprio sensor), e a faixa e o filtro (DLPF 42 Hz) são os mesmos que o DMP usa, para os dois modos serem comparáveis. O sensor é lido a cada 2 ms (`RAW_READ_INTERVAL_US`).
- Para trocar de modo: mude `RAW_DATA` e grave o `equip_6DOF` de novo.

### Mensagem (`message_6dof_t`, 15 bytes)

| Campo | Tipo | Descrição |
|---|---|---|
| `id` | `uint8_t` | ID do Equip |
| `mode` | `uint8_t` | `'B'` (brutos) ou `'T'` (tratados) |
| `rot[3]` | `int16_t` | rotação X/Y/Z |
| `acc[3]` | `int16_t` | aceleração X/Y/Z |
| `touch` | `uint8_t` | 1 se o sensor capacitivo está tocado |

A struct é `packed` (sem bytes de alinhamento), então equip e base leem os campos sempre nas mesmas posições. Por isso o equip lê o sensor em variáveis locais antes de copiar: um ponteiro para campo `packed` pode ficar desalinhado e travar o ESP32. A mensagem tem 3 bytes a mais que a `message_t` (~24 µs a mais no ar a 1 Mbps), bem dentro da posição de 1500 µs do TDMA.

A base escreve na serial uma linha por mensagem:

```text
id/modo/rot_x/rot_y/rot_z/acc_x/acc_y/acc_z/touch
6/B/164/-328/0/0/0/16384/1
```

O `contato connect` **não** lê esse formato (descarta as linhas). Para ver os dados use `contato diag-6dof` do [contato_cli](../contato_cli).

### Como usar

1. Ajuste as linhas marcadas com `ALTERAR` nos dois arquivos. Os valores atuais são do equip 6 / base 6: ID, `MY_SLOT`, MACs e offsets.
2. Grave os dois, por exemplo por OTA:
   ```powershell
   contato ota --id 6 --script equip_6DOF --port COM7
   contato ota --base 6 --script base_6DOF --port COM7
   ```
3. Leia os dados: `contato diag-6dof --id 6`.

O `monitor.cpp` mede tudo desse equip (pacotes, perdas, atraso), menos as trocas de toque, porque reconhece a mensagem pelo tamanho (12 ou 20 bytes).

## Hardware

| Componente | Descrição |
|---|---|
| ESP32 DEVKIT V1 | Microcontrolador (Equip, Base, Ponte e mestre TDMA) |
| MPU6050 | IMU 6 eixos (I2C, 400 kHz), presente nos Equips |
| GPIO 2 | LED azul de indicação de transmissão |
| GPIO T3 | Sensor de toque capacitivo |
