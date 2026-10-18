# Plano: zerar o contador da partida a frio por tempo desligado (DS3231)

**Status:** planejado, não implementado. Base: branch `feat/cold-start` (config do fork na versão 2).

## Objetivo

Hoje o contador de tentativas da partida a frio zera de três jeitos: motor funcionando por um tempo, gesto do pedal ou chave ligada por um tempo. A Speeduino v0.4.3d (Mega) desliga por completo com a chave, então não tem como saber quanto tempo ficou desligada.

Com um relógio DS3231 com bateria, o contador também zera depois de X minutos com a chave **desligada**. Exemplo: afogou de manhã e você volta à tarde; a partida a frio volta com dose cheia.

## Antes de começar: confirmar

1. **VSS no pino 20?** Na v0.4, o pino 20 é a entrada padrão de VSS (velocidade). Ele também é o SDA do I2C do Mega. Se o VSS estiver em uso nesse pino, o I2C de hardware não funciona ali.
2. **O que a v0.4.3d tem nos pinos 20 e 21:** conferir no esquema da placa se há algum circuito (resistor, capacitor, condicionamento de VSS) ligado a esses pinos, e se dá para acessá-los.
3. **Modelo do módulo e da bateria:** o módulo comum (ZS-042) tem um circuito que carrega a bateria. Com uma CR2032 comum (não recarregável), isso é perigoso: remover o diodo/resistor de carga ou usar uma LIR2032.

## Hardware

| Módulo DS3231 | Speeduino (Mega) |
|---|---|
| VCC | 5 V |
| GND | GND |
| SDA | pino 20 |
| SCL | pino 21 |

- Os resistores de pull-up do próprio módulo (4,7 kΩ) bastam.
- **Se os pinos 20/21 não estiverem livres:** I2C por software em dois pinos digitais livres quaisquer. O DS3231 funciona bem assim, só é mais lento, o que aqui não faz diferença.

## Como vai funcionar

1. **Não precisa acertar data nem hora.** Só importa o tempo que passou entre duas leituras. O relógio só precisa continuar contando.
2. **Relógio inválido:** se o relógio parou (bateria morta, primeira ligação), o DS3231 liga o flag OSF (oscillator stop flag). Nesse caso o firmware não confia na hora e **não zera nada**: falha para o lado seguro, que é manter o contador.
3. **Ao contar uma tentativa:** o firmware grava a hora atual do relógio na EEPROM, junto com o contador.
4. **A cada chave ligada:** compara a hora atual com a da última tentativa. Se passou do tempo configurado, zera o contador. Também cobre o caso da placa ligada pelo USB com a chave desligada, porque a checagem é na chave ligada e não no boot.
5. **Hora "voltou para trás"** (relógio trocado ou reiniciado): não zera.

## Mudanças no código

### Configuração (versão 3 do fork)

- Campo novo `coldStartOffReset` no **byte 203** da página 15 (minutos, 0 = desligado). Fica antes dos campos atuais, seguindo a regra de alocar do fim para o começo. `Unused15_106_203[98]` vira `Unused15_106_202[97]`.
- Marcador `0xC7` = v3, com `setV3Defaults()` e um caso novo em `initialiseColdStartConfig()`. Uma placa na v2 só ganha o padrão do campo novo; os demais ajustes ficam intactos.
- Padrão sugerido: 30 minutos (a decidir).

### EEPROM

- Bytes **3668–3671**: hora da última tentativa contada (`uint32_t`, segundos desde 2000-01-01). `0xFFFFFFFF` = desconhecida.
- Ficam na mesma faixa livre do marcador (3672) e do contador (3673). Acrescentar um `static_assert` igual aos que já existem em `storage.cpp`.
- Uma gravação de 4 bytes por tentativa: desgaste desprezível.

### Driver do DS3231

- Arquivos novos em `speeduino/src/rtc/ds3231.{h,cpp}` (ou dentro de `src/controllers/coldStart/`).
- `Wire.begin()`, 100 kHz, e `Wire.setWireTimeout(25000, true)` para um módulo solto ou em curto **não travar a ECU**. O framework AVR instalado (5.3.0) já tem essa função.
- Lê os registradores de hora/data (0x00–0x06) e o status (0x0F, bit OSF). Converte para segundos desde 2000 com uma função própria: não precisa de biblioteca de data.
- Ao encontrar o OSF ligado, limpa o flag (a partir daí a contagem vale) e marca como desconhecida a hora da última tentativa.
- Custo estimado: cerca de 200 bytes de RAM (buffers da Wire) e 1–2 KB de flash. Hoje sobram cerca de 1,7 KB de RAM.

### Controller da partida a frio

- O controller recebe a leitura do relógio por um ponteiro de função (`bool (*getRtcSeconds)(uint32_t &seconds)`), para os testes poderem simular o relógio sem hardware.
- `openAttempt()` grava a hora junto com o contador.
- Na borda de chave ligada: com tentativas > 0, `coldStartOffReset` ≠ 0, relógio válido, hora salva válida e `agora - salva ≥ minutos × 60`, chama `clearAttempts()`.
- **Leitura do relógio só nesses dois momentos** (chave ligada e tentativa contada), nunca a cada volta do loop. Uma leitura leva cerca de 1 ms a 100 kHz. Injeção e ignição rodam por interrupção e não são afetadas.

### Datalog e TunerStudio

- Bit 6 do byte 139: relógio OK. Indicador "Cold Start RTC OK/ERR" e entrada no datalog.
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
- Upgrade v2 → v3 preservando os ajustes.

**Bancada**

1. Com o módulo ligado, o indicador RTC OK acende.
2. Fazer uma tentativa, desligar tudo, esperar o tempo configurado (use 1 min no teste) e religar: o contador deve estar zerado.
3. Religar antes do tempo: o contador continua.
4. Desconectar o módulo com a placa ligada: a ECU **não pode travar**, e o indicador mostra erro. Ao reconectar, o indicador volta.
5. Tirar a bateria do módulo e religar: relógio inválido, o contador não zera.

## Ordem de implementação

1. Responder as perguntas do topo e ligar o módulo.
2. Driver do DS3231 com os testes de conversão de data → segundos.
3. Configuração v3 + EEPROM + teste de upgrade.
4. Lógica no controller + testes com relógio simulado.
5. INI (campo, bit, indicador, datalog).
6. Bateria completa do simulador, build e teste em bancada.
