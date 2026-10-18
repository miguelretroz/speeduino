# Speeduino Firmware

Firmware da Speeduino, uma ECU (gerenciamento eletrônico de motor) open source. O código controla injeção e ignição em tempo real, via interrupções. Um bug aqui pode danificar um motor de verdade, então correção vem antes de elegância.

## Stack

- **C++14** sobre o **framework Arduino**, buildado com **PlatformIO**
- Placas: **Arduino Mega 2560** (alvo principal, env padrão), **Teensy 3.5/3.6/4.1**, **STM32** (F407VE, BlackPill F401/F411, FCR Micro F4)
- Libs: SimplyAtomic, avr-fast-shift, avr-fast-div, libdivide, FastCRC; no AVR, `avr-libstdcpp` fornece os headers da STL
- Testes: **Unity** (via PlatformIO). No ambiente nativo usa **ArduinoFake** + **fakeit** (`lib/ArduinoFake/`)
- Software de tuning: **TunerStudio**, que lê a definição em `reference/speeduino.ini`

## Comandos

- `pio run`: build do env padrão (`megaatmega2560`)
- `pio run -e teensy41` / `pio run -e black_F407VE`: build de outras placas (lista completa em `platformio.ini`)
- `pio run -e megaatmega2560 -t upload`: gravar na placa
- `pio test -e megaatmega2560_sim_unittest -f test_math`: roda uma suite no simulador AVR (simavr). **Funciona no macOS**. A primeira execução baixa o toolchain AVR e o simavr; depois cada suite leva de 3 a 10 s. Sem `-f`, roda todas (~3 min)
- `pio test -e native_code_coverage -f test_fancontrol`: testes nativos com coverage (é o que o CI roda). **Só no Linux/GCC**: no macOS o Apple clang não compila (`noexcept` em type alias no `decoder_t.h` e templates do fakeit). Use o simulador, ou Linux/Docker
  - O CI roda os nativos duas vezes: `PLATFORMIO_BUILD_FLAGS="-D INJ_CHANNELS=8 -D IGN_CHANNELS=8"` e depois `4`/`4`
- `misra/check_misra.sh -c <pasta do cppcheck> -o misra/.results`: scan MISRA C:2012 (cppcheck)
- `codespell --ignore-words=.github/workflows/codespell-ignored-words.txt ./speeduino/`: checagem de ortografia (roda no CI)
- `doxygen Doxyfile`: gera a documentação em `reference/doxygen/` (gitignored)
- Configurações locais do PlatformIO vão em `local.ini` (gitignored, incluído via `extra_configs`)

## Estrutura de Pastas

```
speeduino/                      # src_dir do PlatformIO — todo o firmware
├── speeduino.ino               # setup() e loop() principal
├── init.cpp                    # initialiseAll(): carrega o tune, pinos, schedulers, controllers
├── board_definition.h          # escolhe o board_*.h pela CPU (ou EXTERNAL_BOARD_H)
├── board_*.{h,cpp}             # camada de abstração por placa (avr2560, teensy35/41, stm32, native)
├── decoders.cpp                # decoders de trigger/roda fônica (ISRs) — arquivo grande (~6k linhas)
├── decoder_t / decoder_builder / decoder_init  # interface e construção do decoder ativo
├── scheduler*.cpp, schedule_state_machine.*, scheduledIO_*  # agendamento de injeção/ignição por timer
├── fuel_calcs, corrections, dwell, crankMaths, secondaryTables  # cálculos do motor
├── sensors.cpp                 # leitura de sensores (ADC, polled, interrupções de flex/VSS)
├── comms.cpp / comms_legacy.cpp  # protocolo serial do TunerStudio (novo e legado)
├── comms_CAN.cpp / comms_secondary.cpp  # CAN e serial secundária
├── config_pages.h / pages.cpp  # structs das páginas de tune (espelham o INI) e mapa de páginas
├── storage*.cpp / updates.cpp  # EEPROM e migração entre versões de dados
├── statuses.h / globals.h      # estado em tempo real (currentStatus) e configPageN globais
├── table2d* / table3d*         # tabelas 2D/3D com interpolação
├── unit_testing.h              # macros TESTABLE_* e INTERNAL_TEST_ASSERT
└── src/
    ├── controllers/<nome>/     # controles auxiliares: idle, boost, fan, vvt, launch, aircon, nitrous, tacho...
    ├── pins/                   # abstração de pinos (outputPin, fastOutputPin, invertable...)
    ├── pwm/                    # canais PWM por software
    ├── PID/                    # controladores PID inteiros
    ├── decoders/               # calculadoras de ângulo de virabrequim
    └── SPIAsEEPROM/ FRAM/ BackupSram/ STM32_CAN/  # drivers de storage e CAN
test/
├── test_<area>/                # uma pasta por suite (main.cpp + test_*.cpp)
├── test_utils.h                # helpers (RUN_TEST_P, TEST_DATA_P, tabelas)
└── test_harness_*.h            # harness para device/simulador e nativo
lib/ArduinoFake/                # fakes do Arduino para o env nativo
reference/speeduino.ini         # definição do TunerStudio — tem que bater com config_pages.h
boards/                         # board JSON + linker script do FCR Micro F4
misra/                          # script, regras e supressões MISRA
```

## Restrições de Plataforma

O AVR é o denominador comum: **todo código precisa compilar e caber no Mega 2560**.

- Mega 2560: 8 KB de RAM, 256 KB de flash, 16 MHz, sem FPU. Em PR, o CI reporta o delta de RAM/flash (`megaatmega2560`, `teensy41`, `black_F407VE`). Fique de olho nele
- Sem `float`/`double` no caminho quente. Use inteiros, ponto fixo, shifts e as libs de divisão rápida (`avr-fast-div`, `libdivide`)
- Constantes e strings em flash: `PROGMEM`, `PSTR()`, `memcpy_P()`
- Sem alocação dinâmica (`new`/`malloc`), sem exceções, sem RTTI
- O AVR compila em `-O3`. Código frio (init, migrações) usa `__attribute__((optimize("Os")))` ou `#pragma GCC optimize ("Os")` para economizar flash
- Número de canais é fixado em compilação: `INJ_CHANNELS` / `IGN_CHANNELS`. O padrão no Mega é 4 injeção / 5 ignição, e no Teensy/STM32 grandes é 8/8. A mudança precisa funcionar em todas as combinações (o CI builda o Mega em 4-5, 6-3 e 8-1, e roda os testes nativos em 4x4 e 8x8)
- Código específico de placa fica atrás de `#if defined(CORE_AVR)`, `CORE_TEENSY`, `CORE_STM32` ou `NATIVE_BOARD`, e de preferência dentro dos `board_*` em vez de espalhado
- O build de todas as placas do `.github/workflows/build-firmware.yml` precisa passar

## Tempo Real e Interrupções

- Decoders, schedulers e PWMs por software rodam em ISR. Variáveis compartilhadas entre ISR e `loop()` são `volatile`, e leituras/escritas multi-byte ficam dentro de `ATOMIC() { ... }`
- ISR curta: nada de `Serial`, `delay()`, laços longos ou divisões pesadas
- Tarefas periódicas no `loop()` usam os bits de `currentStatus.LOOP_TIMER`, ex.: `BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_15HZ)` (1 Hz a 1 kHz). Não use `millis()`/`delay()` para isso
- Proteja divisões e cálculos contra entradas zero ou fora de faixa (ex.: tempo de revolução `0` com o motor parado). Vários fixes recentes são exatamente isso

## Tune, INI e EEPROM

Esta é a área mais fácil de quebrar sem perceber.

- Os structs `configN` em `config_pages.h` são gravados na EEPROM como estão e lidos pelo TunerStudio pelos offsets definidos em `reference/speeduino.ini`. **Layout do struct e INI têm que bater byte a byte**
- Não mova nem remova campos existentes. Reaproveite campos `unused`/spare. Bitfields ficam em blocos do tamanho de um byte
- Mudou o tamanho de uma página: atualize `pageSize` no INI **e** `ini_page_sizes` em `test/test_pages/test_page.cpp`
- Mudou o significado de dados já gravados: escreva uma migração `upgradeVXXtoVYY()` em `updates.cpp` e incremente `CURRENT_DATA_VERSION`
- A assinatura `signature` do INI tem que bater com `codeVersion` em `comms.cpp`
- O CI valida o INI em todo PR (`validate-ini.yml`)

## Arquitetura e Padrões

O projeto está sendo refatorado aos poucos para ficar testável. Siga a direção dos commits recentes:

- Controles auxiliares ficam em `src/controllers/<nome>/<nome>Controller.{h,cpp}`. Detalhes internos expostos só para testes ficam em `*_details.h` / `*_detail.h`
- Funções novas recebem estado e config **por parâmetro** em vez de ler globais diretamente, ex.: `fanControl(statuses &current, const config2 &page2, const config6 &page6, const config15 &page15)`. Use `const &` quando não modificam
- Prefira reduzir globais a criar novos. Estado de um módulo fica `static` dentro do `.cpp`
- Quando um teste precisa acessar algo interno, use `TESTABLE_STATIC`, `TESTABLE_INLINE_STATIC` ou `TESTABLE_CONSTEXPR` (de `unit_testing.h`) no lugar de `static`/`constexpr`, e declare com `extern` no teste
- `setup()`/`loop()` ficam fora do build de teste (`#ifndef UNIT_TEST`)

## Estilo de Código

- Guia oficial: https://wiki.speeduino.com/dev/Style_code
- Indentação de 2 espaços, chaves em linha própria (Allman)
- Inteiros de largura fixa (`uint8_t`, `int16_t`...). Literais unsigned com sufixo `U`
- `constexpr` em vez de `#define` para constantes novas. Headers novos com `#pragma once`
- Regras MISRA usadas no código: retorno ignorado com `(void)`, funções sem parâmetros declaradas como `f(void)`, `if/else` sempre com chaves
- Doxygen: `@file` no topo e `/** @brief ... */` nas funções públicas dos headers
- Mantenha a grafia britânica dos identificadores existentes (`initialise...`). O codespell roda no CI
- Comentários e identificadores em inglês

## Testes

- Toda feature ou refactor vem com teste. O projeto está aumentando cobertura ativamente (Codecov)
- Cada suite é uma pasta `test/test_<area>/`. O `main.cpp` inclui `../test_harness_device.h` e `../test_harness_native.h`, declara as funções com `extern void testX(void);`, chama tudo em `runAllTests()` e termina com `TEST_HARNESS(runAllTests)`
- Use `RUN_TEST_P(fn)` em vez de `RUN_TEST(fn)` (guarda o nome em flash, economiza RAM no AVR) e `TEST_DATA_P` para dados de teste
- Os testes também rodam no AVR simulado, então as mesmas restrições de RAM valem para o código de teste
- Fixtures de teste montam `statuses` e `configN` locais e passam para a função testada (veja `test/test_fancontrol/` ou `test/test_launchcontrol/launch_fixture.h`)

## Mudanças do Fork

- **Layout do fork**
  - Config no fim da página 15 (bytes 198–255 do INI, `Unused15_106_197` foi encolhido). Campos novos do fork vão **antes** dos existentes (do fim para o começo da área livre), para não colidir com o que o upstream adicionar
  - EEPROM, no vão entre o fim da página 15 e os CRCs de calibração: 3564–3663 calibração do óleo (CRC, bins, valores), 3668–3671 reservados para o DS3231, 3672 versão da config do fork, 3673 contador de tentativas da partida a frio. `static_assert`s em `storage.cpp` e `test_oil_calibration_no_overlap` (`test_layout.cpp`) protegem
  - Os campos do fork **não** usam `CURRENT_DATA_VERSION` (isso conflitaria com as migrações do upstream). `fork_updates.cpp` (`doForkUpdates()`, chamado depois do `doUpdates()`) tem a versão própria: `0xC5` = v1 (partida a frio), `0xC6` = v2 (+ bateria/chave ligada), `0xC7` = v3 (+ temperatura do óleo). Placa sem versão recebe todos os padrões; placa em versão anterior só os padrões dos campos novos. Campo novo = nova versão + `setVnDefaults()` + caso em `doForkUpdates()` + teste de upgrade
  - Pacote ao vivo: byte 139 (partida a frio), 140–141 (óleo), por isso `ochBlockSize = 142` no INI. O `LOG_ENTRY_SIZE` do firmware ficou em 138 de propósito: já diverge do INI no upstream e só afeta o comando legado `A` e o programmable outputs
  - `test_ini_offsets` (`test/test_coldstart/test_config.cpp`) confere os offsets do INI contra o struct. A EEPROM falsa dos testes do fork é compartilhada: `test/fake_fork_storage.{h,cpp}`
- **Partida a frio** (`src/controllers/coldStart/`, testes em `test/test_coldstart/`): sistema de gasolina auxiliar (bomba + válvula) para motor a etanol. Tela no TunerStudio em Accessories → Cold Start. Status também nos bits 6 e 7 do `status5` (`coldStartActive`, `coldStartLocked`)
- **Temperatura do óleo** (`src/controllers/oilTemp/`, testes em `test/test_oiltemp/`): sensor NTC de 2 fios com pull-up numa entrada analógica, calibrado como CLT/IAT pela ferramenta do TunerStudio (tabela 3 = `SensorCalibrationTable::OilTempSensor`, aceita no comando `t` em `comms.cpp`). Alerta de óleo quente com histerese, indicador e saída opcional. Tela em Accessories → Oil Temperature. A tabela do óleo **não** entra em `saveAllCalibrationTables()` (só as migrações do upstream chamam e um teste delas conta as gravações)

- **Pull-up no RX da serial secundária (AVR)**: com a serial secundária no protocolo TunerStudio e o pino 15 (RX3) solto, o ruído da rede (~60 bytes/s) sequestrava a serial principal e o TunerStudio não conectava. `init.cpp` liga o pull-up interno depois do `secondarySerial.begin()`
- ⚠️ **Gravar um release antigo (ex.: SpeedyLoader 202501, dados v24) numa placa com dados v27 só rebaixa o byte de versão (endereço 0) sem converter.** Antes de voltar para este firmware, corrija o byte 0 para 27, senão as migrações rodam de novo e corrompem o acerto

## Git e PRs

- Este repositório é um **fork para uso próprio**: `origin` = `miguelretroz/speeduino`, `upstream` = `speeduino/speeduino`
- Trabalho novo vai numa branch própria (ex.: `feat/cold-start`) com push para o `origin`. Não fazer merge no `master` nem abrir PR, a menos que pedido
- Atualizar com o upstream: `git fetch upstream` e merge/rebase de `upstream/master`. Conflitos esperados em `config_pages.h`, `reference/speeduino.ini` e `init.cpp`
- Mensagem de commit: `tipo: descricao` em inglês, minúsculas, curta e sem jargão. Tipos: `feat`, `fix`, `style`, `content`, `config`, `refactor`. Ex.: `feat: cold start gasoline system for ethanol engines`
- **Commits nunca levam coautoria**: sem `Co-Authored-By` nem nenhum outro trailer de atribuição
- Um commit por intenção. Não misturar frentes de trabalho no mesmo commit
- Se um dia contribuir com o upstream: PR atômico, mensagem no imperativo (ex.: `Refactor fan controller`) e o CLA do projeto (`contributing.md`)
