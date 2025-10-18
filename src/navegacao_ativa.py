#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np

class ReactiveNavigator:
    def __init__(self):
        rospy.init_node('reactive_navigator_v3')
        rospy.on_shutdown(self.shutdown_hook)

        # --- Parametros de Comportamento ---
        self.forward_speed = 0.2
        self.rotation_speed = 0.5
        self.mapping_rotation_speed = 0.3
        self.backward_speed = 0.15 # Velocidade para marcha a re

        self.alert_distance = 0.7    # Limiar de alerta (inicia mapeamento)
        self.critical_distance = 0.4 # Limiar critico (inicia marcha a re)
        self.recovery_distance = 0.2 # Distancia para recuar (metros)
        
        self.frontal_cone_angle = math.radians(50.0)
        self.rotation_tolerance = math.radians(5.0)

        # --- Variaveis da Maquina de Estados ---
        self.states = ['AVANCANDO', 'MAPEANDO_AMBIENTE', 'ALINHANDO_ROTA', 'RECUPERANDO']
        self.state = 'AVANCANDO'
        rospy.loginfo("Iniciando no estado: {}".format(self.state))

        # --- Variaveis de Controle ---
        self.latest_scan = None
        self.latest_odom = None # Guarda a mensagem de odometria completa
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.map_start_yaw = None
        self.has_completed_half_turn = False
        self.best_yaw_so_far = 0.0
        self.best_dist_so_far = 0.0
        self.recovery_start_position = None # Guarda a posicao ao iniciar a re

        # --- Publishers e Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/RosAria/pose', Odometry, self.odom_callback)

        rospy.Timer(rospy.Duration(0.1), self.run_state_machine)

    def shutdown_hook(self):
        rospy.loginfo("Recebido sinal de desligamento. Parando o robo...")
        self.stop_robot()

    def scan_callback(self, msg):
        self.latest_scan = msg

    def odom_callback(self, msg):
        self.latest_odom = msg # Guarda a mensagem completa para usar na re
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        
    def run_state_machine(self, event):
        if self.latest_scan is None: return
        
        if self.state == 'AVANCANDO': self.handle_avancando()
        elif self.state == 'RECUPERANDO': self.handle_recuperando()
        elif self.state == 'MAPEANDO_AMBIENTE': self.handle_mapeando_ambiente()
        elif self.state == 'ALINHANDO_ROTA': self.handle_alinhando_rota()

    def change_state(self, new_state):
        if new_state in self.states and self.state != new_state:
            rospy.loginfo("==========================================================")
            rospy.loginfo("Mudando de estado: {} -> {}".format(self.state, new_state))
            rospy.loginfo("==========================================================")
            self.state = new_state

    def handle_avancando(self):
        """
        Logica do estado AVANCANDO.
        Agora com dois limiares de distancia (critico e de alerta).
        """
        min_dist_frontal = self.check_frontal_obstacle()
        
        # 1. Checa a condicao mais perigosa primeiro
        if min_dist_frontal <= self.critical_distance:
            self.stop_robot()
            self.change_state('RECUPERANDO')
            return
        # 2. Se nao for perigosa, checa a condicao de alerta
        elif min_dist_frontal <= self.alert_distance:
            self.stop_robot()
            self.change_state('MAPEANDO_AMBIENTE')
            return
        
        # 3. Se tudo estiver livre, avanca
        twist_msg = Twist()
        twist_msg.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(twist_msg)

    def handle_recuperando(self):
        """
        NOVO ESTADO: Logica da marcha a re.
        Recua por uma distancia fixa e depois vai para o mapeamento.
        """
        # Na primeira vez que entra no estado, guarda a posicao inicial
        if self.recovery_start_position is None:
            if self.latest_odom is not None:
                self.recovery_start_position = self.latest_odom.pose.pose.position
            else:
                rospy.logwarn("Aguardando odometria para iniciar recuperacao...")
                self.stop_robot()
                return # Espera o proximo ciclo

        # Calcula a distancia percorrida desde o inicio da re
        current_pos = self.latest_odom.pose.pose.position
        dist_traveled = math.sqrt(
            (current_pos.x - self.recovery_start_position.x)**2 +
            (current_pos.y - self.recovery_start_position.y)**2
        )

        # Se ainda nao recuou o suficiente
        if dist_traveled < self.recovery_distance:
            twist_msg = Twist()
            twist_msg.linear.x = -self.backward_speed # Velocidade negativa
            self.cmd_vel_pub.publish(twist_msg)
            rospy.loginfo("Recuperando... Distancia: {:.2f}m / {:.2f}m".format(dist_traveled, self.recovery_distance))
        # Se ja recuou o suficiente
        else:
            self.stop_robot()
            self.recovery_start_position = None # Reseta para a proxima vez
            self.change_state('MAPEANDO_AMBIENTE')

    def handle_mapeando_ambiente(self):
        if self.map_start_yaw is None:
            self.map_start_yaw = self.current_yaw
            self.has_completed_half_turn = False
            self.best_dist_so_far = 0.0
            self.best_yaw_so_far = self.current_yaw
            rospy.loginfo("Iniciando mapeamento do ambiente...")

        twist_msg = Twist()
        twist_msg.angular.z = self.mapping_rotation_speed
        self.cmd_vel_pub.publish(twist_msg)
        
        relative_angle, current_best_dist = self.find_best_escape_route(self.latest_scan)

        if current_best_dist > self.best_dist_so_far:
            self.best_dist_so_far = current_best_dist
            self.best_yaw_so_far = self.current_yaw + relative_angle
            if self.best_yaw_so_far > math.pi: self.best_yaw_so_far -= 2 * math.pi
            if self.best_yaw_so_far < -math.pi: self.best_yaw_so_far += 2 * math.pi

        rospy.loginfo("Mapeando... Angulo Atual: {:.0f} | Melhor Rota Encontrada: Angulo {:.0f} com {:.2f}m".format(
            math.degrees(self.current_yaw), math.degrees(self.best_yaw_so_far), self.best_dist_so_far))
        
        angle_diff = self.current_yaw - self.map_start_yaw
        if angle_diff > math.pi: angle_diff -= 2 * math.pi
        if angle_diff < -math.pi: angle_diff += 2 * math.pi
        
        if not self.has_completed_half_turn and abs(angle_diff) > (math.pi / 2.0):
            self.has_completed_half_turn = True
            rospy.loginfo("--- Mapeamento passou da metade do giro ---")

        if self.has_completed_half_turn and abs(angle_diff) < self.rotation_tolerance:
            self.stop_robot()
            self.target_yaw = self.best_yaw_so_far
            rospy.loginfo("Mapeamento completo. Rota final escolhida: Angulo {:.0f} com {:.2f}m".format(
                math.degrees(self.target_yaw), self.best_dist_so_far))
            self.map_start_yaw = None
            self.change_state('ALINHANDO_ROTA')

    def find_best_escape_route(self, scan_data):
        if scan_data is None: return 0.0, 0.0
        ranges = np.array(scan_data.ranges)
        ranges[np.isinf(ranges)] = 0
        ranges[np.isnan(ranges)] = 0
        window_size = 20
        smoothed_ranges = np.convolve(ranges, np.ones(window_size), 'same')
        best_index = np.argmax(smoothed_ranges)
        best_angle = scan_data.angle_min + best_index * scan_data.angle_increment
        best_distance = ranges[best_index]
        return best_angle, best_distance

    def handle_alinhando_rota(self):
        angle_diff = self.target_yaw - self.current_yaw
        if angle_diff > math.pi: angle_diff -= 2 * math.pi
        if angle_diff < -math.pi: angle_diff += 2 * math.pi

        if abs(angle_diff) < self.rotation_tolerance:
            self.stop_robot()
            self.change_state('AVANCANDO')
        else:
            rospy.loginfo("Alinhando... Atual: {:.1f} -> Alvo: {:.1f}".format(
                math.degrees(self.current_yaw), math.degrees(self.target_yaw)))
            twist_msg = Twist()
            twist_msg.angular.z = self.rotation_speed if angle_diff > 0 else -self.rotation_speed
            self.cmd_vel_pub.publish(twist_msg)

    def check_frontal_obstacle(self):
        if self.latest_scan is None: return float('inf')
        num_readings = len(self.latest_scan.ranges)
        center_index = num_readings / 2
        cone_indices = int(self.frontal_cone_angle / self.latest_scan.angle_increment)
        start_index = center_index - (cone_indices / 2)
        end_index = center_index + (cone_indices / 2)
        frontal_ranges = [r for r in self.latest_scan.ranges[start_index:end_index] if r > 0 and not math.isinf(r)]
        return min(frontal_ranges) if frontal_ranges else float('inf')

    def stop_robot(self):
        rospy.loginfo("Enviando comando para PARAR o robo.")
        self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        navigator = ReactiveNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("No finalizado pelo usuario.")
