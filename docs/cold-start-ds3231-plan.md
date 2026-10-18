# Plano: zerar o contador da partida a frio por tempo desligado (DS3231)

**Status:** planejado, não implementado. Implementar em cima da `feat/oil-temp` (config do fork na versão 3, `0xC7`). Esta branch ainda está sobre a `feat/cold-start`: antes de começar, refazer em cima da `feat/oil-temp`.

## Objetivo

Hoje o contador de tentativas da partida a frio zera de três jeitos: motor funcionando por um tempo, gesto do pedal ou chave ligada por um tempo. A Speeduino v0.4.3d (Mega) desliga por completo com a chave, então não tem como saber quanto tempo ficou desligada.

Com um relógio DS3231 com bateria, o contador também zera depois de X minutos com a chave **desligada**. Exemplo: afogou de manhã e você volta à tarde; a partida a frio volta com dose cheia.

## Já conferido

- **VSS:** desligado no acerto (`vssMode` = Off), então o firmware não usa o pino 20.
- **Pinos 20 e 21 na v0.4.3d** (arquivo oficial da placa):
  - D21 (SCL) vai direto ao **J5 pino 3**, sem nenhum componente.
  - D20 (SDA) vai ao **J5 pino 4** (marcado VSS) e tem o condicionamento do VSS: **R54** (1 kΩ de pull-up para o 5 V) e **C28** (0,1 µF para o GND). Com o C28, a subida do sinal leva cerca de 100 µs; o I2C a 100 kHz precisa de menos de 1 µs. **Do jeito que está, o I2C de hardware não funciona no pino 20.**
- **Leitura da bateria com o motor parado:** a detecção de chave ligada depende dela. Estava travada em 12,5 V por um bug do upstream, corrigido no commit `fix: battery voltage stuck at 12.5v with the engine stopped`.

## Antes de começar: confirmar

1. **Modelo do módulo e da bateria:** o módulo comum (ZS-042) tem um circuito que carrega a bateria. Com uma CR2032 comum (não recarregável), isso é perigoso: remover o diodo/resistor de carga ou usar uma LIR2032.
2. **Qual ligação usar** (abaixo): tirar o C28 e o R54 da placa ou usar I2C por software em outros pinos.

## Hardware

### Opção A (recomendada): I2C de hardware no J5

| Módulo DS3231 | Speeduino v0.4.3d |
|---|---|
| VCC | 5 V (J1, qualquer pino) |
| GND | GND (J6, qualquer pino) |
| SDA | J5 pino 4 (D20, VSS) |
| SCL | J5 pino 3 (D21) |

- **Tirar o C28 e o R54** da placa. Os dois só servem ao VSS, que está desligado. Se um dia precisar do VSS no pino 20, o DS3231 vai para a opção B.
- Os pull-ups de 4,7 kΩ do próprio módulo bastam.
- O J5 pino 1 já tem o sensor de óleo e o resistor de 1 kΩ. Use contatos fêmea individuais nos pinos 3 e 4, não um conector de 4 vias.

### Opção B: I2C por software, sem mexer na placa

- Dois pinos digitais livres do Mega. Os pinos 14 e 15 (Serial3) não estão ligados a nada na placa e a serial secundária está desligada no acerto. Se ela for religada, escolher outros pinos.
- O acesso é soldando nos furos dos pinos do Mega na placa.
- O DS3231 funciona bem assim, só é mais lento, o que aqui não faz diferença.

## Como vai funcionar

1. **Não precisa acertar data nem hora.** Só importa o tempo que passou entre duas leituras. O relógio só precisa continuar contando.
2. **Relógio inválido:** se o relógio parou (bateria morta, primeira ligação), o DS3231 liga o flag OSF (oscillator stop flag). Nesse caso o firmware não confia na hora e **não zera nada**: falha para o lado seguro, que é manter o contador.
3. **Ao contar uma tentativa:** o firmware grava a hora atual do relógio na EEPROM, junto com o contador.
4. **A cada chave ligada:** compara a hora atual com a da última tentativa. Se passou do tempo configurado, zera o contador. Também cobre o caso da placa ligada pelo USB com a chave desligada, porque a checagem é na chave ligada e não no boot.
5. **Hora "voltou para trás"** (relógio trocado ou reiniciado): não zera.

## Mudanças no código

### Configuração (versão 4 do fork)

- Campo novo `coldStartOffReset` no **byte 197** da página 15 (minutos, 0 = desligado), logo antes do óleo (198–203), seguindo a regra de alocar do fim para o começo. `Unused15_106_197[92]` vira `Unused15_106_196[91]`.
- Versão `0xC8` = v4 em `fork_updates.cpp`: `setV4Defaults()` e um caso v3 → v4 em `doForkUpdates()`. Uma placa na v3 só ganha o padrão do campo novo; a partida a frio e o óleo ficam intactos. Placa sem versão conhecida continua recebendo todos os padrões.
- `FORK_CONFIG_V4` em `test/fake_fork_storage.h`, offset 197 no INI e em `test_ini_offsets`.
- Padrão sugerido: 30 minutos (a decidir).

### EEPROM

- Bytes **3668–3671**: hora da última tentativa contada (`uint32_t`, segundos desde 2000-01-01). `0xFFFFFFFF` = desconhecida.
- Essa faixa já está reservada: fica entre a calibração do óleo (até 3663) e a versão do fork (3672). Acrescentar um `static_assert` igual aos que já existem em `storage.cpp`.
- Uma gravação de 4 bytes por tentativa: desgaste desprezível.

### Driver do DS3231

- Arquivos novos em `speeduino/src/rtc/ds3231.{h,cpp}` (ou dentro de `src/controllers/coldStart/`).
- Opção A: `Wire.begin()`, 100 kHz, e `Wire.setWireTimeout(25000, true)` para um módulo solto ou em curto **não travar a ECU**. O framework AVR instalado (5.3.0) já tem essa função. Opção B: I2C por software com o mesmo cuidado de timeout.
- Lê os registradores de hora/data (0x00–0x06) e o status (0x0F, bit OSF). Converte para segundos desde 2000 com uma função própria: não precisa de biblioteca de data.
- Ao encontrar o OSF ligado, limpa o flag (a partir daí a contagem vale) e marca como desconhecida a hora da última tentativa.
- Custo estimado: cerca de 200 bytes de RAM (buffers da Wire) e 1–2 KB de flash. Na `feat/oil-temp` sobram cerca de 1,6 KB de RAM (6537 de 8192 bytes usados).

### Controller da partida a frio

- O controller recebe a leitura do relógio por um ponteiro de função (`bool (*getRtcSeconds)(uint32_t &seconds)`), para os testes poderem simular o relógio sem hardware.
- `openAttempt()` grava a hora junto com o contador.
- Na borda de chave ligada: com tentativas > 0, `coldStartOffReset` ≠ 0, relógio válido, hora salva válida e `agora - salva ≥ minutos × 60`, chama `clearAttempts()`.
- **Leitura do relógio só nesses dois momentos** (chave ligada e tentativa contada), nunca a cada volta do loop. Uma leitura leva cerca de 1 ms a 100 kHz. Injeção e ignição rodam por interrupção e não são afetadas.

### Datalog e TunerStudio

- Bit 6 do byte 139 (livre: bits 0–1 fase, 2–4 tentativas, 5 bateria baixa): relógio OK. Indicador "Cold Start RTC OK/ERR" e entrada no datalog.
- Campo "Off Time To Clear Counter (0 = off)" no grupo Attempts, com texto de ajuda.

## Testes

**Unitários (simulador AVR), com relógio simulado**

- Passou o tempo → zera.
- Não passou → mantém.
- Relógio inválido (OSF) → mantém.
- Hora salva desconhecida → mantém.
- Hora voltou para trás → mantém.
- Campo em 0 → nunca zera pelo relógio.
- Chave religada com a placa ligada pelo USB → checa e zera se passou o tempo.
- Upgrade v3 → v4 preservando a partida a frio e o óleo.

**Bancada**

1. Com o módulo ligado, o indicador RTC OK acende.
2. Fazer uma tentativa, desligar tudo, esperar o tempo configurado (use 1 min no teste) e religar: o contador deve estar zerado.
3. Religar antes do tempo: o contador continua.
4. Desconectar o módulo com a placa ligada: a ECU **não pode travar**, e o indicador mostra erro. Ao reconectar, o indicador volta.
5. Tirar a bateria do módulo e religar: relógio inválido, o contador não zera.

## Ordem de implementação

1. Refazer esta branch em cima da `feat/oil-temp`.
2. Escolher a ligação (A ou B), preparar a placa e ligar o módulo.
3. Driver do DS3231 com os testes de conversão de data → segundos.
4. Configuração v4 + EEPROM + teste de upgrade.
5. Lógica no controller + testes com relógio simulado.
6. INI (campo, bit, indicador, datalog).
7. Bateria completa do simulador, build e teste em bancada.
