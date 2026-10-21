# Auto-tune e adaptação do PID (lenta e lambda)

Tela no TunerStudio: **Accessories → PID Auto Tune (idle / AFR)**.

O auto-tune faz um ensaio curto com o motor ligado: move a válvula da lenta (ou a correção de combustível do lambda) um pouco para cima e para baixo em volta de onde ela está, mede como o motor responde (o período e o tamanho da oscilação) e calcula P, I e D. **Nada muda no acerto até você apertar "Apply Result".**

A adaptação contínua (opcional, uma chave para a lenta e outra para o lambda) ajusta os três ganhos juntos, devagar, enquanto o motor roda: baixa se a malha oscila, sobe se ela fica fora do alvo sem corrigir.

## Antes de começar

- Grave o firmware da branch `feat/pid-autotune` e use o INI dela (`reference/speeduino.ini`). Na primeira partida a ECU cria os padrões novos sozinha (versão de config do fork v4); nada do acerto atual muda.
- **Lenta:** precisa estar num modo closed loop. O recomendado é o **modo 7, "Stepper OL+CL"**: a tabela de passos que você já tem vira a base e o PID só corrige a diferença. Em Idle → Idle Control, mude "Idle control type" para *Stepper OL+CL*. Os ganhos atuais (P 0,03 / I 0,09 / D 0) servem para começar.
- **Lambda:** precisa da sonda wideband e do algoritmo **PID** (em Fuel → AFR/O2, "Algorithm"). Hoje está em *No correction*.

## Lenta, passo a passo

1. Motor quente (acima da temperatura de fim do enriquecimento de aquecimento), em ponto morto, **A/C e ventoinha desligados**, sem encostar no acelerador.
2. Abra a tela PID Auto Tune e aperte **Tune Idle**.
3. Os primeiros 3 s a válvula fica parada (a ECU mede a rotação média). Depois a rotação começa a oscilar uns ±30–80 RPM, com período de 1 a 4 s. O gráfico "Idle" mostra isso.
4. Em 20 s a 1 minuto aparece **DONE** e os valores sugeridos de P, I e D. Dá para trocar a "Tuning Rule" agora: os valores são recalculados na hora com as mesmas medidas, sem refazer o ensaio.
5. Aperte **Apply Result**. A ECU grava só P, I e D (outras mudanças que você não gravou ficam como estão) e o TunerStudio recarrega a página sozinho (os campos de Idle mudam). Observe a lenta por um minuto. O Apply só vale para o mesmo modo de lenta em que o ensaio foi feito.

Acelerador acima de 3 % (ou do "Integral reset above TPS", se for menor), ligar o A/C, a ventoinha ou o idle-up, a rotação cair abaixo de 400 RPM, ou fechar a janela interrompe o ensaio, e o PID volta a controlar de onde estava.

## Lambda, passo a passo

1. Wideband aquecida, motor quente, lambda em closed loop (o indicador de correção EGO mexendo).
2. Rotação e carga estáveis: a marcha lenta serve. Aperte **Tune AFR**.
3. A correção de combustível passa a oscilar ±5 % e o AFR acompanha (±0,3–0,6 AFR). Em 20–40 s aparece DONE.
4. É normal ver **GAIN LIMITED**: o P do lambda vai até 200 no TunerStudio e o ensaio quase sempre pede mais. O I sugerido é o que mais importa.
5. **Apply Result**.

Se o ensaio do lambda for feito na lenta, o resultado vale melhor para a lenta. A adaptação (abaixo) corrige aos poucos para as outras condições.

## Se der falha

| Indicador | O que fazer |
|---|---|
| NOT READY | Motor ligado, lenta em closed loop (modo 3, 5, 6 ou 7, já fora do "crank to run taper"), acelerador fechado (até 3 %). Lambda: algoritmo PID, sonda wideband e o closed loop ativo naquele momento (temperatura, RPM, TPS, MAP dentro da janela do O2). |
| WRONG MODE | Modo da lenta ou algoritmo do lambda errado, ou foi trocado entre o ensaio e o Apply. |
| NO ROOM TO SWING | A válvula está perto do limite (Minimum/Maximum valve value) ou a correção do lambda perto do Controller Auth. Diminua a "Swing" ou abra os limites. |
| SWING < 2x STEP HYST. | Aumente "Idle Valve Swing" ou diminua "Step Hysteresis" do stepper. |
| CONDITIONS CHANGED | Algo mudou no meio (acelerador, A/C, ventoinha, RPM/MAP do lambda). Repita. |
| NO RESPONSE: MORE SWING | A rotação (ou o AFR) não saiu da banda. Aumente a "Swing" (ex.: 8 → 12 passos). |
| TOO SMALL: MORE SWING | A oscilação ficou do tamanho da banda, a medida não serve. Aumente a "Swing" ou diminua a "Band". |
| TOO LARGE: LESS SWING | Passou de 300 RPM (ou 2,0 AFR) do alvo. Diminua a "Swing". |
| DID NOT SETTLE | A oscilação não ficou regular até o tempo limite. Condições instáveis (ex.: motor ainda esquentando). Repita com o motor mais estável. |
| AFR LOOP TOO SLOW | O closed loop do lambda parou de rodar por 8 s (saiu da janela de operação). |

## Ajustes da tela

- **Tuning Rule**: *Balanced PI* (padrão, assenta em poucos períodos e aguenta a malha ficar duas vezes mais sensível), *Gentle PI* (Tyreus–Luyben, o mais seguro e o mais lento) e *Fast PI* (Ziegler–Nichols, mais rápido, pode oscilar se a malha ficar mais sensível). O D fica sempre 0: o PID do firmware não filtra a derivada e, nas simulações, qualquer D deixava a lenta instável.
- **Idle Valve Swing**: quanto a válvula anda para cada lado (passos no stepper, % no PWM). Padrão 8.
- **Idle RPM Band**: quanto a rotação precisa passar do alvo para a válvula inverter. Um pouco acima do ruído de RPM da lenta. Padrão 15.
- **AFR Correction Swing / AFR Band**: o mesmo para o lambda. Padrões 5 % e 0,1 AFR.
- **Cycles To Average / Give Up After**: ciclos medidos (2–4) e tempo máximo do ensaio.

## Adaptação contínua

- Ligue em "Continuous Gain Adaptation": **Idle** e/ou **AFR**.
- A cada janela (padrão 15 s) com a malha em regime, olha o erro: se ele cruza a banda de um lado para o outro 4 vezes ou mais (cada lado só conta depois de 2 amostras seguidas, para o ruído não contar como oscilação), os ganhos baixam 5 %; se ele fica fora do alvo em média sem cruzar, sobem 5 %. Fica entre 50 % e 150 % dos valores dos campos.
- Só conta com a malha em regime: acelerador fechado (lenta) e sem transiente de acelerador (lambda), sem DFCO, sem A/C, ventoinha ou idle-up mudando, e sem a saída do PID no limite. No lambda, uma janela recomeça se a rotação mudar mais de 15 %.
- O multiplicador aparece nos medidores e no datalog ("Idle/AFR PID Gain Adaptation"), fica gravado na EEPROM (depois de 1 minuto sem mudar, no máximo a cada 10 minutos, ou quando o motor para com a chave ligada) e volta a 100 % quando você muda P, I ou D, aplica um auto-tune ou aperta Reset.
- Se a lenta tiver muito ruído de RPM e o multiplicador ficar só descendo, aumente "Idle On Target Band".

## Limitações conhecidas

- **Modo 5 (Stepper Closed Loop)**: o PID desse modo trabalha na variação da saída e cada variação é arredondada para baixo em ¼ de passo. Com a rotação abaixo do alvo por menos de 32/P RPM ele não corrige (com P = 1, até 32 RPM abaixo), e acima do alvo sempre corrige, então a lenta tende a ficar um pouco baixa. É do firmware original. Por isso o modo 7 é o recomendado.
- Como a ECU pode escrever os 6 ganhos (Apply), eles ficam com "prioridade do controlador" no TunerStudio: ao carregar um arquivo de acerto (.msq) com a ECU conectada, esses 6 valores ficam os da ECU. Para voltar valores antigos, digite-os nos campos.
- O ensaio do relé subestima um pouco o ganho crítico em malhas com atraso (até ~25 % nas simulações), o que deixa os ganhos sugeridos um pouco mais conservadores.
- Testado em simulação (2273 testes no simulador AVR, incluindo plantas simuladas de lenta e lambda). Falta a validação no carro.
