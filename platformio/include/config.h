#pragma once

// ─── Hardware ─────────────────────────────────────────────────────────────────
#define LED_PIN         2
#define TOUCH_PIN       T3
#define TOUCH_THRESHOLD 30   // leitura capacitiva abaixo deste valor = tocado
#define I2C_CLOCK_HZ    400000

// ─── BLE ─────────────────────────────────────────────────────────────────────
static const char *DEVICE_NAME         = "Contato";

// UUIDs do serviço e característica BLE MIDI padrão (especificação Apple / MIDI Association).
// Usar estes UUIDs permite que qualquer aplicativo MIDI descubra o dispositivo como periférico MIDI.
static const char *MAIN_SERVICE_UUID   = "03B80E5A-EDE8-4B33-A751-6CE34EC4C700";
static const char *MIDI_CHAR_UUID      = "7772E5DB-3868-4112-A1A9-F2669D106BF3";

// Características de sensor e controle personalizadas, hospedadas no mesmo serviço.
static const char *SECTIONS_CHAR_UUID  = "251beea3-1c81-454f-a9dd-8561ec692ded";
static const char *ACCEL_SENS_CHAR_UUID= "c7f2b2e2-1a2b-4c3d-9f0a-123456abcdef";
static const char *STATUS_CHAR_UUID    = "f8d968fe-99d7-46c4-a61c-f38093af6ec8";
static const char *DIR_CHAR_UUID       = "a1b2c3d4-0001-4b33-a751-6ce34ec4c701";
static const char *CALIBRATE_CHAR_UUID = "b4d0c9f8-3b9a-4a4e-93f2-2a8c9f5ee7a2";

// ─── Chaves NVS ──────────────────────────────────────────────────────────────
static const char *PREF_NAMESPACE    = "mpu";
static const char *PREF_KEY_OFFS     = "offs";
static const char *PREF_KEY_SECTIONS = "sections";
static const char *PREF_KEY_SENS     = "sens";
static const char *PREF_KEY_DIR      = "dir";

// ─── Temporização ────────────────────────────────────────────────────────────
// Intervalo de amostragem dos sensores e envio via notify BLE (~333 Hz).
static const unsigned long STATUS_INTERVAL_MS = 3;

// Tempo mínimo entre eventos de percussão pelo acelerômetro.
// Evita disparos repetidos causados por um único gesto de sacudida.
static const unsigned long ACCEL_DEBOUNCE_MS = 2000;

// ─── Sensor ──────────────────────────────────────────────────────────────────
// Ângulo de rolagem limitado a ±GYRO_MAX_DEG para evitar instabilidade próxima à vertical.
static const float GYRO_MAX_DEG = 89.0f;

// ─── Padrões musicais ────────────────────────────────────────────────────────
// Usados quando nenhuma seção de notas foi salva na NVS ainda.
static const int     DEFAULT_SECTION_COUNT    = 6;
static const uint8_t DEFAULT_NOTE             = 60;  // Dó central (MIDI 60)

// Nota de percussão: MIDI 36 = Bumbo (Bass Drum 1), canal 8 = canal de percussão GM.
static const uint8_t PERC_NOTE    = 36;
static const uint8_t PERC_CHANNEL = 8;

// Limiar padrão do acelerômetro (unidade bruta IMU eixo X ÷ 10) para disparar percussão.
static const int32_t DEFAULT_ACCEL_THRESHOLD = 1250;

// Intervalo válido do limiar do acelerômetro configurável via BLE.
static const int32_t MIN_ACCEL_THRESHOLD = 100;
static const int32_t MAX_ACCEL_THRESHOLD = 32767;
