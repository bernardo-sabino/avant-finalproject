# Importação dos módulos básicos do ROS2
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 

# Importação dos módulos do MAVROS
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State

# Importação da interfaces
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, Bool 

# Importação de módulos padrões 
import math, time
import numpy as np
 
class DroneNode(Node):

    def __init__(self):
        super().__init__("drone_mission_node") 

        # Configurações da Comunicação 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.current_state = State() 
        self.current_pose = None 
        self.current_orientation = None 

        # Parâmetros de Controle
        self.foward_velocity = 0.3  # Velocidade de avanço 
        self.kp_yaw = 0.003         # Ganho Proporcional 
        self.max_yaw_rate = 0.8     # Velocidade máxima de rotação
        self.search_yaw_rate = 0.3  # Velocidade de giro para procurar a linha 

        # Variáveis de Estado da Missão 
        self.line_error = 0.0
        self.line_lost = True
        self.crossbar_detected = False

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, \
        '/mavros/setpoint_position/local', qos_profile)  
        
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            qos_profile
        )

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', \
        self.state_cb, qos_profile)
        
        self.pose_sub = self.create_subscription(PoseStamped, \
        '/mavros/local_position/pose', self.pose_cb, qos_profile)
        
        self.error_sub = self.create_subscription(
            Float32,
            '/line_follower/error',
            self.error_callback,
            qos_profile,
        )
        self.crossbar_sub = self.create_subscription(
            Bool,
            '/crossbar_detected',
            self.crossbar_callback,
            qos_profile
        )

        self.get_logger().info("Nó drone_mission_node inicializado com sucesso") 
        self.wait_for_connection()

    # ===== Métodos Callback =====
    
    def state_cb(self, msg):
        self.current_state = msg
    def pose_cb(self, msg):
        if msg and msg.pose: 
            self.current_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.current_orientation = msg.pose.orientation
        else:
            self.current_pose = None
            self.current_orientation = None
    def error_callback(self, msg):
        if math.isnan(msg.data):
            self.line_lost = True
            self.line_error = 0.0
        else:
            self.line_lost = False
            self.line_error = msg.data
    def crossbar_callback(self, msg):
        if msg.data:
            self.crossbar_detected = True
            self.get_logger().info("!!! TRAVESSÃO DETECTADO !!! Parando a missão.")    
    
    # ===== Método de Conexão com a FCU =====
    def wait_for_connection(self):
        self.get_logger().info("FCU sendo conectada...")
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self)
            time.sleep(0.2)
        self.get_logger().info("A FCU foi conectada com sucesso!")

    # ===== Serviços do Drone =====
    def set_mode(self, custom_mode):
        client = self.create_client(SetMode, '/mavros/set_mode')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Serviço /mavros/set_mode não está disponível')
            return False
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().mode_sent

    def arm_drone(self):
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Serviço /mavros/cmd/arming não disponível')
            return False
        req = CommandBool.Request()
        req.value = True
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def takeoff_drone(self, altitude):
        client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Serviço /mavros/cmd/takeoff não disponível')
            return False
        req = CommandTOL.Request()
        req.altitude = altitude
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def land_drone(self):
        client = self.create_client(CommandTOL, '/mavros/cmd/land')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Serviço /mavros/cmd/land não disponível')
            return False
        req = CommandTOL.Request()
        req.altitude = 0.0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    # ===== Métodos de Movimentação =====
    
    def go_to_position(self, x, y, z, tolerance=0.3): 
        """
        Move o drone para uma posição (x, y, z) e espera até chegar,
        MANTENDO a orientação (yaw) que tinha no início do comando.
        """
        self.get_logger().info(f"[Navegação] Movendo para Posição: (x={x}, y={y}, z={z})")

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map' 

        # Espera até termos uma leitura da pose e orientação atuais
        while rclpy.ok() and (self.current_pose is None or self.current_orientation is None):
            self.get_logger().warn("[Navegação] Aguardando Posição e Orientação atuais do drone...")
            rclpy.spin_once(self, timeout_sec=0.5)
            if not rclpy.ok(): return False # Sai se o ROS parar
        
        # "Trava" a orientação alvo
        target_orientation_to_keep = self.current_orientation

        while rclpy.ok():
            # Atualiza o timestamp
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Define a posição alvo
            target_pose.pose.position.x = float(x)
            target_pose.pose.position.y = float(y)
            target_pose.pose.position.z = float(z)
            # Define a orientação alvo 
            target_pose.pose.orientation = target_orientation_to_keep

            self.local_pos_pub.publish(target_pose)

            # Processa callbacks 
            rclpy.spin_once(self, timeout_sec=0.1) 

            if self.current_pose is None:
                continue # Pula este ciclo se a pose for perdida

            # Calcula a distância
            current_x, current_y, current_z = self.current_pose
            distance = math.sqrt((current_x - x)**2 + (current_y - y)**2 + (current_z - z)**2)

            if distance < tolerance:
                self.get_logger().info("[Navegação] Posição alcançada com sucesso!")
                break
        
        time.sleep(1) # Espera 1s para estabilizar
        return True

    
    # ===== Iniciando o Drone =====
    def arm_and_takeoff(self, altitude=1.0):   
        self.get_logger().info("Iniciando sequência de decolagem...")
        if not self.set_mode('GUIDED'):
            self.get_logger().error("Falha ao definir modo GUIDED")
            return False 
        time.sleep(1) # Pequena pausa
        if not self.arm_drone():
            self.get_logger().error("Falha ao armar o drone")
            return False 
        time.sleep(2)
        if not self.takeoff_drone(altitude):
            self.get_logger().error("Falha na decolagem")
            return False 
        time.sleep(8) # Tempo para o drone subir e estabilizar
        self.get_logger().info("Decolagem concluída com sucesso.")
        return True
    
    # ===== Missão =====

    def follow_line(self):
        """
        Inicia o loop de controle principal para seguir a linha.
        O loop para QUANDO self.crossbar_detected == True.
        """
        self.get_logger().info("[MISSÃO] MODO SEGUIDOR DE LINHA ATIVADO!")
        
        twist_msg = Twist() # Cria a mensagem de velocidade
        
        # Frequência do loop de controle 
        rate = self.create_rate(10) 

        # Roda ENQUANTO o ROS estiver OK E o travessão NÃO for detectado
        while rclpy.ok() and not self.crossbar_detected:
            
            # Processa callbacks 
            rclpy.spin_once(self, timeout_sec=0.0) 

            if self.line_lost:
                # LINHA PERDIDA: Parar de avançar e girar para procurar
                twist_msg.linear.x = 0.0  
                twist_msg.angular.z = self.search_yaw_rate 
                self.get_logger().warn("[MISSÃO] Linha perdida! Procurando...", throttle_duration_sec=1.0)
            
            else:
                # LINHA ENCONTRADA: Aplicar controle Proporcional
                twist_msg.linear.x = self.foward_velocity 
                
                # Cálculo do Controle Proporcional
                # error = centro - cx
                # Se erro > 0 (linha à esquerda), queremos yaw > 0 (girar p/ esquerda)
                target_yaw_rate = self.kp_yaw * self.line_error
                
                # Limita a velocidade de rotação
                twist_msg.angular.z = np.clip(target_yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

            # Publica o comando de velocidade
            self.vel_pub.publish(twist_msg)
            
            # Espera pelo próximo ciclo 
            rate.sleep()
                
        self.get_logger().info("[MISSÃO] Loop de seguir linha terminado. Parando o drone.")
        
        # Envia um comando de parada total
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        
        # Publica o comando de parada algumas vezes para garantir
        for _ in range(5):
            self.vel_pub.publish(twist_msg)
            time.sleep(0.05)
        
        return True


# ===== Main =====
def main():
    rclpy.init()
    node = DroneNode()

    # Parâmetros da Missão
    ALTITUDE_DE_VOO = 2.5 # Altitude da missão 
    
    # Posição inicial para começar a ver a linha 
    START_POS_X = 0.0
    START_POS_Y = -2.0 
    
    try:
        # Decolar para a altitude da missão
        if node.arm_and_takeoff(ALTITUDE_DE_VOO): 
            
            # Mover para a posição inicial (onde a câmera vê a linha)
            node.get_logger().info("[MISSÂO] Movendo para a posição inicial da linha...")
            node.go_to_position(START_POS_X, START_POS_Y, ALTITUDE_DE_VOO)

            node.get_logger().info("[MISSÃO] Posição inicial alcançada. Aguardando 2s para estabilizar.")
            time.sleep(2.0)
            
            # Iniciar o modo de seguir a linha
            # Esta função só termina quando o travessão é visto
            node.follow_line()
        else:
            node.get_logger().error("[MISSÃO] Falha na decolagem. Abortando missão.")

    except Exception as e:
        node.get_logger().error(f"[MISSÃO] Erro inesperado na missão: {e}. Acionando pouso de emergência.")
        node.land_drone() 
    finally:
        node.get_logger().info("[MISSÃO] Encerrando missão.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()