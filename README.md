## Como rodar esse arquivo

entre na pasta src do catkin_ws, com o seguinte comando
cd ~/catkin_ws/src
dentro dessa pasta rode o seguinte comando 
git clone https://github.com/joaovitor625/rota_fuga.git
cd ~/catkin_ws
catkin_make rota_fuga

Para rodar o programa, você tem duas opções rodar cada nó indiviualmente
rosrun rota_fuga navega_ativa.py
ou rodar o arquivo de launch
roslaunch rota_fuga navegacao.launch
