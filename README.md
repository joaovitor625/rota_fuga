## Resumo do Pacote
 Este trabalho prático visa aplicar os conceitos fundamentais de ROS em um cenário real, utilizando o robô Pioneer 3DX e seu sensor LIDAR. O foco do projeto é ir além da simples detecção de obstáculos, implementando uma lógica de controle que permita ao robô tomar uma decisão informada sobre qual o melhor caminho a seguir quando sua rota atual está bloqueada.

Objetivo Principal: Desenvolver um sistema em ROS onde o robô navega de forma autônoma, e ao encontrar um obstáculo, ele para, analisa todo o seu entorno com o sensor laser para identificar a rota de fuga mais promissora (o caminho mais livre) e, então, executa a manobra para seguir por essa nova rota.

## Descrição do Pacote
  1. O sistema completo deve ser iniciado com um único comando roslaunch.

  2. Inicialmente, o robô deve mover-se para frente com velocidade constante.

  3. O robô deve monitorar continuamente os obstáculos em um cone frontal.

  4. SE um obstáculo for detectado dentro deste cone a uma distância menor que um limiar de alerta (ex: 1.2 metros), o robô deve: a. Parar completamente. b. Iniciar um estado de "Análise", no qual ele processa os dados de uma varredura completa (360º) do laser. c. Na análise, o nó deve encontrar o ângulo que corresponde à maior distância livre (a melhor rota de fuga). d. Uma vez identificado o melhor ângulo, o robô entra no estado "Girando" e rotaciona sobre seu eixo até estar alinhado com essa nova direção. e. Após o alinhamento, ele retorna ao estado de "mover-se para frente".

  5. Mecanismo de Segurança: SE um obstáculo for detectado dentro de um limiar crítico (ex: 0.4 metros, uma distância muito perigosa), o robô deve primeiro executar uma manobra de marcha a ré por uma curta distância fixa (ex: recuar 20 cm) antes de iniciar o procedimento de Análise e Rotação descrito acima.

## Como rodar esse pacote

entre na pasta src do catkin_ws, com o seguinte comando
```bash
cd ~/catkin_ws/src
```
Dentro dessa pasta rode os seguintes comandos:
```bash
git clone https://github.com/joaovitor625/rota_fuga.git
cd ~/catkin_ws
catkin_make rota_fuga
```
Para rodar o programa, você tem duas opções rodar cada nó indiviualmente
rosrun rota_fuga navega_ativa.py
ou rodar o arquivo de launch
```bash
roslaunch rota_fuga navegacao.launch
```

## Quem criou esse pacote?
Esse pacote foi criado pelos seguintes alunos da disciplina de robotica avançada da UNIFEI

Henrique Xavier Vincetini 

João Vitor Barbosa Pinheiro

Julia Da Cruz Viana

| Alunos  | 
| -------- |
| Henrique Xavier Vincetini |
|João Vitor Barbosa Pinheiro |
| Julia Da Cruz Viana |


Professor Responsável pela disciplina:
|Professor|
|--------|
|Guilherme de Souza Bastos|
