# Ações de proteção e falha de sensor

Telas no TunerStudio: **Tuning → Engine Protection → Protection Actions (fork)** e **→ Sensor Faults (fork)**.

No firmware original toda proteção faz a mesma coisa: corta combustível e/ou faísca acima do "Protection RPM Limit" (limp home). Aqui cada condição escolhe o que fazer, pode guardar a ação até o motor parar, e há condições novas. Separado disso, um sensor preso num limite elétrico vira falha e passa a ler um valor fixo seguro.

Nada muda no comportamento até você mexer nas telas: as proteções do upstream continuam em "Limit RPM" e as condições novas vêm desligadas.

## Ações de proteção

| Ação | O que faz |
|---|---|
| Limit RPM | Como sempre: corta acima do Protection RPM Limit. |
| Warn only | Mostra a condição (indicador, datalog, saída de aviso) e não corta nada. |
| Stop engine | Corta em qualquer rotação: o motor para. Dá para ligar de novo. |

- "Protection Cut" (Common Engine Protection) precisa estar ligado para Limit RPM e Stop engine cortarem, como no upstream. Os avisos funcionam mesmo com ele desligado, mas as condições do upstream (óleo, boost, AFR) só são avaliadas com ele ligado.
- **Memory**: com memória, a ação continua depois que a condição passa, até o motor parar. Serve para não perder uma queda rápida de pressão de óleo, por exemplo. Sem memória a ação acaba com a condição.
- Se mais de uma condição estiver ativa, vale a mais forte: Stop > Limit > Warn.

### Condições

| Condição | De onde vem | Padrão |
|---|---|---|
| Oil Pressure | Proteção de pressão de óleo do upstream (Engine Protection → Oil Pressure) | Limit RPM |
| Boost Limit | Boost cut do upstream | Limit RPM |
| AFR Protection | Proteção de AFR do upstream | Limit RPM |
| Hot Coolant | Nova: água acima de "Coolant Temperature" (sai 5 °C abaixo) | Warn only, desligada |
| Hot Oil | Novo: o alerta de óleo quente de Accessories → Oil Temperature | Warn only, desligada |
| Low Fuel Pressure | Nova: pressão de combustível **nos bicos** abaixo do mínimo por um tempo, só acima de uma rotação | Limit RPM, desligada |
| Sensor Fault | Nova: qualquer sensor em falha (abaixo) | Limit RPM, desligada |

A pressão nos bicos é a pressão da flauta menos o boost (o vácuo soma): serve para regulador referenciado ao coletor e para regulador fixo. Precisa do sensor de pressão de combustível ligado (Accessories → Fuel/Oil pressure). Para uma linha de 3 bar (43 psi), 30 psi é um mínimo seguro.

### Saída de aviso

"Warning Output": um pino para lâmpada ou buzina, aceso enquanto alguma proteção está ativa ou na memória, ou algum sensor está em falha. Diferente das saídas programáveis, ele enxerga as condições do fork.

### No TunerStudio

Indicadores na tela mostram cada condição, se há memória, a ação em vigor (WARNING / RPM LIMITED / ENGINE STOPPED) e a saída. No datalog: "Protection Reason (bits)", "Protection Action" e "Protection Remembered". Na tela inicial: PROTECTION WARNING, PROTECTION CUTTING e PROTECTION REMEMBERED.

## Falha de sensor

Para CLT, IAT, MAP e TPS, cada um com a sua chave:

- Leitura abaixo de "Shorted Below" (curto para o terra) ou acima de "Open Above" (sensor desligado: o pull-up leva a entrada a 5 V) por "Time At The Limit Before A Fault" (padrão 1 s) = falha. Ela sai depois do mesmo tempo dentro da faixa. Ao ligar a ECU, um sensor já no limite vira falha na hora.
- Enquanto em falha, a ECU usa o valor fixo do campo "... While Faulted": água 60 °C, ar 30 °C, MAP 100 kPa, TPS 20 % (sem movimento de acelerador, então sem enriquecimento de aceleração). Os padrões são conservadores: rico, nunca pobre.
- Com a água em falha, a ventoinha roda ("Run The Fan While Faulted"), porque a temperatura real é desconhecida.
- O MAP merece atenção: o firmware original já descarta leituras nos limites, então um sensor quebrado só congelava o MAP no último valor. Com a falha ligada ele vai para 100 kPa (rico em carga baixa) e, com "Sensor Fault" em Limit RPM nas ações de proteção, o motor não chega ao boost com a carga real desconhecida.

Indicadores na tela e na tela inicial (CLT/IAT/MAP/TPS SENSOR FAULT) e entradas no datalog.

## Limites (volts)

Os limites são em volts na entrada da ECU. Os padrões: NTC (água e ar) 0,04 V e 4,94 V; MAP 0,23 V e 4,88 V (sensores de 0,5–4,5 V); TPS 0,04 V e 4,94 V. Antes de ligar a falha do TPS, confira na calibração do TPS a tensão com o acelerador fechado e aberto: ela precisa ficar dentro dos limites com folga.

## Como testar sem risco

1. Motor parado, chave ligada, TunerStudio conectado. Ligue a detecção do sensor de água e desligue o conector dele: em 1 s aparece COOLANT SENSOR FAULT, a temperatura vai para 60 °C e a ventoinha liga (se "Fan When Off" permitir a ventoinha com o motor parado; senão só ao ligar o motor). Reconecte: a falha sai em 1 s.
2. Ligue "Sensor Fault" nas ações de proteção em Warn only e repita: aparece SENSOR FAULT e WARNING, e a saída de aviso acende se estiver configurada.
3. Com o motor ligado e parado, ponha "Hot Coolant" em Warn only com uma temperatura abaixo da atual: aparece HOT COOLANT e WARNING. Volte a temperatura e a condição sai 5 °C abaixo dela.
4. Para ver um corte, use a água quente em Limit RPM com Protection Cut ligado e o Protection RPM Limit em, por exemplo, 2500: acelere e a rotação não passa dele. Com memória, o limite continua depois de a condição sair, até desligar o motor.
5. Stop engine: teste só parado, em ponto morto. Com a água quente em Stop engine e o limite de temperatura abaixo da atual, o motor para na hora. Desligue a condição antes de dar partida de novo, senão ele para outra vez.

## Limitações

- Sem memória entre ciclos de chave: a memória é só até o motor parar.
- Os valores fixos de falha não sabem a condição real do motor: são um modo de emergência para chegar em casa, não para rodar normalmente.
- A falha do O2, da pressão de óleo e da pressão de combustível (os sensores em si) não é detectada; a proteção de pressão de combustível só vê a pressão lida.
- Testado em simulação (suíte completa do simulador AVR). Falta a validação no carro.
