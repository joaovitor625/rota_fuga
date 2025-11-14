# Rota Fuga — Navegação Reativa para Pioneer 3DX

Resumo
------
Este pacote implementa um comportamento reativo em ROS para o robô Pioneer 3DX equipado com sensor LIDAR. O objetivo é navegar autonomamente em frente e, ao encontrar um obstáculo, determinar a melhor rota de fuga (o setor com mais espaço livre) a partir de uma varredura do laser, alinhar-se com essa direção e seguir adiante. Em caso de risco iminente (obstáculo muito próximo), o robô executa uma marcha-a-ré curta antes de analisar o entorno.

Principais características
-------------------------
- Nó ROS em Python que implementa uma máquina de estados reativa.
- Controle de movimentos via Twist publicado em /RosAria/cmd_vel.
- Leitura de LIDAR em /scan (sensor_msgs/LaserScan) e odometria em /RosAria/pose (nav_msgs/Odometry).
- Estados:
  - AVANCANDO — deslocamento para frente.
  - MAPEANDO_AMBIENTE — gira para analisar 360º e identificar a melhor rota.
  - ALINHANDO_ROTA — gira até alinhar com o ângulo escolhido.
  - RECUPERANDO — marcha-a-ré quando um obstáculo crítico é detectado.
- Seleção da melhor rota por suavização (convolução) das leituras LIDAR e escolha do segmento com maior espaço livre.

Arquivos principais
-------------------
- src/navegacao_ativa.py — nó principal (nome do nó: `reactive_navigator_v3`).
- launch/navegacao.launch — (opcional) arquivo de lançamento para iniciar o sistema completo com um comando roslaunch. (Verifique se existe no repositório; caso não exista, ver as instruções abaixo para executar o nó manualmente.)

Requisitos
----------
- ROS (ex.: Noetic, Melodic — ajuste conforme sua distro).
- Python 2.7/3.x compatível com a versão do ROS em uso.
- Pacotes ROS: rospy, sensor_msgs, geometry_msgs, nav_msgs, tf.
- Biblioteca Python: numpy.

Instalação
----------
1. Entre na pasta src do seu workspace catkin:
```bash
cd ~/catkin_ws/src
```

2. Clone o repositório:
```bash
git clone https://github.com/joaovitor625/rota_fuga.git
```

3. Volte para a raiz do workspace e compile:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Observação: o `catkin_make` geralmente compila todo o workspace; não há necessidade de passar o nome do package para o catkin_make.

Execução
--------
Opção A — usar roslaunch (recomendado se houver arquivo de launch):
```bash
roslaunch rota_fuga navegacao.launch
```

Opção B — executar o nó diretamente:
1. Torne o script executável (se necessário):
```bash
chmod +x ~/catkin_ws/src/rota_fuga/src/navegacao_ativa.py
```
2. Execute com rosrun:
```bash
rosrun rota_fuga navegacao_ativa.py
```

Tópicos usados
--------------
- Subscriber: `/scan` (sensor_msgs/LaserScan)
- Subscriber: `/RosAria/pose` (nav_msgs/Odometry)
- Publisher: `/RosAria/cmd_vel` (geometry_msgs/Twist)

Parâmetros e constantes (valores padrão no código)
--------------------------------------------------
Os valores abaixo estão definidos diretamente em src/navegacao_ativa.py. Recomenda-se expô-los via rosparam para facilitar ajustes.
- Velocidades:
  - forward_speed: 0.2 m/s
  - rotation_speed: 0.5 rad/s
  - mapping_rotation_speed: 0.3 rad/s
  - backward_speed: 0.15 m/s
- Distâncias/limiares:
  - alert_distance: 0.7 m (inicia mapeamento)
  - critical_distance: 0.4 m (inicia recuperação/marcha-a-ré)
  - recovery_distance: 0.2 m (distância a recuar)
- Outros:
  - frontal_cone_angle: 50° (ângulo do cone frontal)
  - rotation_tolerance: 5° (tolerância ao alinhar)

Algoritmo (visão geral)
-----------------------
1. Enquanto em AVANCANDO, o nó verifica a menor distância dentro de um cone frontal.  
   - Se distância <= critical_distance: muda para RECUPERANDO (marcha-a-ré por recovery_distance).  
   - Se critical_distance < distância <= alert_distance: para e entra em MAPEANDO_AMBIENTE.  
   - Caso contrário: avança em frente.

2. Em RECUPERANDO, usa odometria para medir o deslocamento durante a marcha-a-ré; quando atingir recovery_distance, para e passa para MAPEANDO_AMBIENTE.

3. Em MAPEANDO_AMBIENTE, gira no próprio eixo enquanto coleta leituras LIDAR, aplica suavização por janela e registra o ângulo com maior espaço livre. Depois de completar uma varredura (lógica baseada em diferença de yaw), seleciona o melhor ângulo e passa para ALINHANDO_ROTA.

4. Em ALINHANDO_ROTA, gira até que a diferença entre yaw atual e yaw alvo esteja dentro da rotation_tolerance; então retorna ao estado AVANCANDO.

Problemas conhecidos e recomendações
-----------------------------------
- check_frontal_obstacle() usa índices derivados de divisões sem conversão para inteiros, o que pode causar exceções ao fatiar a lista de ranges. Recomenda-se converter índices para int().
- O código substitui valores inf e NaN por 0 nas leituras do LIDAR. Isso pode fazer com que áreas amplas sejam interpretadas como 0 m. Alternativa melhor: substituir inf por um valor alto (ex.: alcance máximo do sensor) e descartar 0s que representem leituras inválidas.
- Parâmetros estão hard-coded; mover para rosparams/arquivo de configuração facilita testes e ajustes em diferentes robôs/ambientes.
- O comportamento é reativo e local (não há mapa global ou planejamento de alto nível). Em ambientes complexos, o robô pode entrar em ciclos de repetição. Considere adicionar lógica de escape ou integração com um planner global.
- Verifique se os tópicos (nomes e tipos) batem com sua plataforma/simulação (ex.: alguns simuladores usam /odom em vez de /RosAria/pose).

Contribuidores
--------------
- Henrique Xavier Vincetini
- João Vitor Barbosa Pinheiro
- Julia Da Cruz Viana

Professor responsável
---------------------
- Guilherme de Souza Bastos

Exemplos rápidos de uso
-----------------------
Executar o nó diretamente:
```bash
# torne o script executável (uma vez)
chmod +x ~/catkin_ws/src/rota_fuga/src/navegacao_ativa.py

# executar com rosrun
rosrun rota_fuga navegacao_ativa.py
```

Executar via launch (se disponível):
```bash
roslaunch rota_fuga navegacao.launch
```
