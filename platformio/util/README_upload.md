# Upload Automático por Script — PlatformIO

## Como funciona

O `upload_script.py` é um script extra do PlatformIO que copia automaticamente o arquivo desejado para `src/main.cpp` antes de compilar e fazer upload. Basta definir a variável de ambiente `SCRIPT` com o nome do arquivo antes do comando `pio run`.

## Pré-requisitos

- PlatformIO instalado e `pio` disponível no PATH
- `upload_script.py` na pasta `scripts/`
- `platformio.ini` com a linha `extra_scripts = scripts/upload_script.py`

## Estrutura de pastas esperada

```
scripts/
├── base/
│   ├── base_3.cpp
│   ├── base_4.cpp
│   └── ...
├── equip/
│   ├── equip_3.cpp
│   ├── equip_4.cpp
│   └── ...
└── tdma/
    ├── base/
    │   ├── base_3_tdma.cpp
    │   ├── base_4_tdma.cpp
    │   └── ...
    ├── equip/
    │   ├── equip_3_tdma.cpp
    │   ├── equip_4_tdma.cpp
    │   └── ...
    └── relogio.cpp
```

## Como usar

No terminal PowerShell, rode o comando com `$env:SCRIPT` definido na mesma linha:

```powershell
# Scripts base
$env:SCRIPT="base_3"; pio run --target upload

# Scripts equip
$env:SCRIPT="equip_3"; pio run --target upload

# Scripts TDMA base
$env:SCRIPT="base_3_tdma"; pio run --target upload

# Scripts TDMA equip
$env:SCRIPT="equip_3_tdma"; pio run --target upload

# Relogio (base mestre TDMA)
$env:SCRIPT="relogio"; pio run --target upload
```

## O que o script faz automaticamente

1. Detecta a pasta correta pelo nome do script (`base`, `equip`, `tdma/base`, `tdma/equip` ou `relogio`)
2. Copia o arquivo `.cpp` para `src/main.cpp`
3. Limpa o cache de build (`.pio/build/`) para forçar recompilação completa
4. O PlatformIO compila e faz upload normalmente

## Mensagens no terminal

- `scripts\equip\equip_3.cpp -> src\main.cpp` — arquivo copiado com sucesso
- `Cache de build limpo.` — cache apagado, recompilação forçada
- `Arquivo não encontrado: ...` — nome do script incorreto ou arquivo não existe
- `Nenhum script definido.` — comando rodado sem definir `$env:SCRIPT`
- `Prefixo não reconhecido.` — nome não começa com `base`, `equip` ou `relogio`

## Observação

A pasta `src/` deve ser mantida vazia — o `main.cpp` é gerado automaticamente pelo script a cada upload e não deve ser versionado no git. Adicione ao `.gitignore`:

```
src/main.cpp
```
