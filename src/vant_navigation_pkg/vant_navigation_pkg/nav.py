# Importação dos módulos básicos do ROS2
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 

# Importação dos módulos do MAVROS
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State

# Importação da interfaces
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from std_msgs.msg import Float32, Bool 

# Importação de módulos padrões 
import math, time
import numpy as np

# ===== A MÁQUINA DE ESTADOS DA MISSÃO =====
class MissionState:
    INIT = 0          # Estado inicial, esperando conexão
    ARM_AND_TAKEOFF = 1 # Enviando comandos de decolagem
    WAIT_FOR_TAKEOFF = 2 # Esperando o drone subir
    MOVING_TO_START = 3  # Indo para o waypoint inicial
    TURNING_TO_SOUTH = 4 # Girando para -Y
    FOLLOWING_LINE = 5   # Executando o seguidor de linha
    MISSION_COMPLETE = 6 # Travessão detectado, pousando
    EMERGENCY_LAND = 7   # Pouso de emergência

class DroneNode(Node):

    def __init__(self):
        super().__init__("drone_mission_node") 

        # Configurações da Comunicação 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado atual do Drone (MAVROS)
        self.current_state_mav = State() 
        self.current_pose = None 
        self.current_orientation = None 

        # --- Parâmetros da Missão ---
        self.ALTITUDE_DE_VOO = 1.0 
        self.START_POS_X = 0.0
        self.START_POS_Y = -2.0 
        self.tolerance = 0.3 # Tolerância de 30cm para waypoints

        # --- Parâmetros de Controle (Seguidor de Linha) ---
        self.kp_strafe = 0.002 # Ganho Proporcional para o movimento lateral
        self.foward_velocity = 0.25 # Anda 25cm para frente a cada passo
        self.frequencia_missao = 5.0 # Hz (5 "passos" por segundo)

        # --- Variáveis de Estado da Visão ---
        self.line_error = 0.0
        self.line_lost = True
        self.crossbar_detected = False

        # --- Variáveis da Máquina de Estados da Missão ---
        self.mission_state = MissionState.INIT
        self.last_state_change_time = self.get_clock().now()
        
        # --- Alvos de Posição/Orientação ---
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.south_orientation_q = Quaternion()
        target_yaw_rad_south = -math.pi / 2.0
        self.south_orientation_q.z = math.sin(target_yaw_rad_south / 2.0)
        self.south_orientation_q.w = math.cos(target_yaw_rad_south / 2.0)


        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, \
        '/mavros/setpoint_position/local', qos_profile)  
        
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
        
        # --- O "Coração" da Missão ---
        # Este timer roda a lógica da missão X vezes por segundo
        self.mission_timer = self.create_timer(1.0 / self.frequencia_missao, self.mission_loop)
        
        # Timer para o MAVROS (necessário para manter o stream)
        self.mavros_stream_timer = self.create_timer(0.1, self.mavros_stream_loop) 

        self.get_logger().info("Nó drone_mission_node inicializado com Máquina de Estados.") 

    # ===== Funções de Callback (Lógica Assíncrona) =====
    
    def state_cb(self, msg):
        self.current_state_mav = msg
    
    def pose_cb(self, msg):
        if msg and msg.pose: 
            self.current_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.current_orientation = msg.pose.orientation
    
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
            
    # ===== Serviços do Drone (Funções de "Ação") =====
    
    def call_service_async(self, client, request):
        """Função helper para chamar um serviço sem bloquear."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Serviço {client.srv_name} não está disponível")
            self.set_mission_state(MissionState.EMERGENCY_LAND)
            return
        
        future = client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """Verifica se a chamada de serviço foi bem-sucedida."""
        try:
            response = future.result()
            
            is_successful = False
            
            # Verifica o atributo 'success' (usado por CommandBool, CommandTOL)
            if hasattr(response, 'success'):
                is_successful = response.success
            
            # Verifica o atributo 'mode_sent' (usado por SetMode)
            elif hasattr(response, 'mode_sent'):
                is_successful = response.mode_sent
            
            if not is_successful:
                 self.get_logger().error(f"Serviço MAVROS falhou (success=False ou mode_sent=False): {future}")
                 self.set_mission_state(MissionState.EMERGENCY_LAND)

        except Exception as e:
            # Pega outros erros (como o AttributeError que vimos)
            self.get_logger().error(f"Exceção na chamada de serviço: {e}")
            self.set_mission_state(MissionState.EMERGENCY_LAND)

    def set_mode(self, custom_mode):
        client = self.create_client(SetMode, '/mavros/set_mode')
        req = SetMode.Request()
        req.custom_mode = custom_mode
        self.call_service_async(client, req)

    def arm_drone(self):
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        req = CommandBool.Request()
        req.value = True
        self.call_service_async(client, req)

    def takeoff_drone(self, altitude):
        client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        req = CommandTOL.Request()
        req.altitude = altitude
        self.call_service_async(client, req)

    def land_drone(self):
        client = self.create_client(CommandTOL, '/mavros/cmd/land')
        req = CommandTOL.Request()
        req.altitude = 0.0
        self.call_service_async(client, req)

    # ===== Lógica da Máquina de Estados (O "Cérebro") =====

    def set_mission_state(self, new_state):
        """Muda o estado da missão e loga."""
        if self.mission_state != new_state:
            self.get_logger().info(f"[MISSÃO] Mudando de Estado: {self.mission_state} -> {new_state}")
            self.mission_state = new_state
            self.last_state_change_time = self.get_clock().now()
            
    def get_time_in_state(self):
        """Retorna o tempo (em segundos) desde a última mudança de estado."""
        return (self.get_clock().now() - self.last_state_change_time).nanoseconds / 1e9

    def check_distance_to_target(self):
        """Verifica se o drone chegou ao target_pose."""
        if self.current_pose is None:
            return False
        
        pos = self.target_pose.pose.position
        curr = self.current_pose
        
        distance = math.sqrt((curr[0] - pos.x)**2 + (curr[1] - pos.y)**2 + (curr[2] - pos.z)**2)
        
        return distance < self.tolerance

    def mavros_stream_loop(self):
        """Publica o target_pose 10x/seg para manter o MAVROS feliz."""
        if self.mission_state > MissionState.WAIT_FOR_TAKEOFF and self.mission_state < MissionState.MISSION_COMPLETE:
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.local_pos_pub.publish(self.target_pose)

    def mission_loop(self):
        """O "Coração" da missão, chamado pelo timer."""
        
        # Estado 0: Inicialização
        if self.mission_state == MissionState.INIT:
            if self.current_state_mav.connected and self.current_pose is not None:
                self.get_logger().info("FCU conectada e pose recebida. Iniciando missão.")
                self.set_mission_state(MissionState.ARM_AND_TAKEOFF)
            else:
                self.get_logger().info("Aguardando conexão com FCU e primeira pose...", throttle_duration_sec=2.0)
            return

        # Estado 1: Armar e Decolar
        if self.mission_state == MissionState.ARM_AND_TAKEOFF:
            self.set_mode('GUIDED')
            time.sleep(0.5) # Pequena pausa entre comandos
            self.arm_drone()
            time.sleep(1.0)
            self.takeoff_drone(self.ALTITUDE_DE_VOO)
            # Define o "alvo" inicial como a posição de decolagem + altitude
            self.target_pose.pose.position.x = self.current_pose[0]
            self.target_pose.pose.position.y = self.current_pose[1]
            self.target_pose.pose.position.z = self.ALTITUDE_DE_VOO
            self.set_mission_state(MissionState.WAIT_FOR_TAKEOFF)
            return

        # Estado 2: Esperando a Decolagem
        if self.mission_state == MissionState.WAIT_FOR_TAKEOFF:
            # Espera 8 segundos para o drone subir
            if self.get_time_in_state() > 8.0:
                self.get_logger().info("Decolagem concluída. Movendo para a posição inicial.")
                # Define o próximo alvo (waypoint inicial)
                self.target_pose.pose.position.x = self.START_POS_X
                self.target_pose.pose.position.y = self.START_POS_Y
                self.target_pose.pose.position.z = self.ALTITUDE_DE_VOO
                self.target_pose.pose.orientation = self.current_orientation # Mantém orientação
                self.set_mission_state(MissionState.MOVING_TO_START)
            return

        # Estado 3: Movendo para a Posição Inicial
        if self.mission_state == MissionState.MOVING_TO_START:
            if self.check_distance_to_target():
                self.get_logger().info("Posição inicial alcançada. Girando para o Sul.")
                # Define o próximo alvo (mesma posição, nova orientação)
                self.target_pose.pose.position.x = self.START_POS_X
                self.target_pose.pose.position.y = self.START_POS_Y
                self.target_pose.pose.position.z = self.ALTITUDE_DE_VOO
                self.target_pose.pose.orientation = self.south_orientation_q # Gira para o Sul
                self.set_mission_state(MissionState.TURNING_TO_SOUTH)
            return

        # Estado 4: Girando para o Sul
        if self.mission_state == MissionState.TURNING_TO_SOUTH:
            # Espera 3 segundos para o giro completar
            if self.get_time_in_state() > 3.0:
                self.get_logger().info("Giro concluído. Iniciando seguidor de linha.")
                self.set_mission_state(MissionState.FOLLOWING_LINE)
            return

        # Estado 5: Seguindo a Linha
        if self.mission_state == MissionState.FOLLOWING_LINE:
            # Verifica a condição de parada
            if self.crossbar_detected:
                self.get_logger().info("Travessão detectado! Missão completa.")
                self.set_mission_state(MissionState.MISSION_COMPLETE)
                self.land_drone()
                return

            # Lógica do seguidor (Sua lógica de "passo-a-passo")
            if self.line_lost:
                self.get_logger().warn("[MISSÃO] Linha perdida! Pairando no local...", throttle_duration_sec=1.0)
                # O loop mavros_stream_timer continua publicando a última pose,
                # então o drone paira (hover) automaticamente.
            else:
                # LINHA ENCONTRADA: Calcular e atualizar o próximo waypoint
                # Suas orientações: -Y (frente), +X (esquerda)
                # Nosso erro: error = centro - cx.
                # Se linha está à ESQUERDA (cx < centro), error é POSITIVO (ex: 70.0)
                # Precisamos corrigir para a ESQUERDA (mover drone no eixo +X).
                
                # 1. Cálculo do passo para frente (no eixo -Y)
                dt = 1.0 / self.frequencia_missao
                step_frente = self.foward_velocity * dt
                self.target_pose.pose.position.y -= step_frente # Move no eixo -Y
                
                # 2. Cálculo da correção lateral (no eixo +X)
                correcao_x = self.kp_strafe * self.line_error
                # O novo alvo X é a *posição inicial* + correção
                self.target_pose.pose.position.x = self.START_POS_X + correcao_x
            return

        # Estado 6: Missão Completa (Pousando)
        if self.mission_state == MissionState.MISSION_COMPLETE:
            # Apenas espera...
            if self.get_time_in_state() > 10.0: # 10s para pousar
                self.get_logger().info("Pouso concluído. Encerrando.")
                rclpy.shutdown()
            return
            
        # Estado 7: Pouso de Emergência
        if self.mission_state == MissionState.EMERGENCY_LAND:
            self.get_logger().error("ESTADO DE EMERGÊNCIA: Pousando agora.")
            self.land_drone()
            self.set_mission_state(MissionState.MISSION_COMPLETE) # Reusa o estado de pouso
            return

# ===== Main =====
def main():
    rclpy.init()
    node = DroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("[MAIN] Missão interrompida pelo usuário (Ctrl+C). Acionando pouso de emergência.")
        node.set_mission_state(MissionState.EMERGENCY_LAND)
        # Dá um segundo para o comando de pouso ser enviado
        time.sleep(1.0)
    finally:
        node.get_logger().info("[MAIN] Encerrando missão.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()