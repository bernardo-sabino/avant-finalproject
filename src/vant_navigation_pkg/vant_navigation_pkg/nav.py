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
from geometry_msgs.msg import Point


# Importação de módulos padrões 
import math, time
import numpy as np

# ===== A MÁQUINA DE ESTADOS DA MISSÃO =====
class MissionState:
    INIT = 0          # Estado inicial, esperando conexão
    ARM_AND_TAKEOFF = 1 # Enviando comandos de decolagem
    WAIT_FOR_TAKEOFF = 2 # Esperando o drone subir
    MOVING_TO_START = 3  # Indo para o waypoint inicial
    FOLLOWING_LINE = 4   # Executando o seguidor de linha
    CENTERING_ON_CROSSBAR = 5 # Centralizando na crossbar
    HOVERING_ON_TARGET = 6 # Ficando parado no centro
    MISSION_COMPLETE = 7 # Pousando, missão finalizada
    EMERGENCY_LAND = 8   # Pouso de emergência

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
        self.ALTITUDE_DE_VOO = 4.0
        self.START_POS_X = 0.0
        self.START_POS_Y = -2.0 
        self.tolerance = 0.3 # Tolerância de 30cm para waypoints

        # --- Parâmetros de Controle (Seguidor de Linha) ---
        self.kp_strafe_left = 0.002 # Ganho Proporcional para o movimento lateral esquerdo
        self.kp_strafe_right = 0.07 # Ganho Proporcional para o movimento lateral direito
        self.foward_velocity = 0.3 # Anda 25cm para frente a cada passo
        self.frequencia_missao = 10.0 # Hz (10 "passos" por segundo)

        # --- Variáveis de Estado da Visão ---
        self.line_error = 0.0
        self.line_lost = True
        self.crossbar_detected = False

        # --- Parâmetros de Controle (Centralização no Travessão) ---
        self.kp_centering_x = 0.008 # Ganho para corrigir o X do travessão
        self.kp_centering_y = -0.0025 # Ganho para corrigir o Y do travessão (NEGATIVO)
        self.centering_tolerance_x = 15 # Tolerância (em pixels) para X (final)
        self.centering_tolerance_y = 20 # Tolerância (em pixels) para Y (para passar para o Estágio 2)

        # --- Variáveis de Estado da Centralização ---
        self.crossbar_error_x = 0.0
        self.crossbar_error_y = 0.0
        self.crossbar_lost = True
        


        # --- Variáveis da Máquina de Estados da Missão ---
        self.mission_state = MissionState.INIT
        self.last_state_change_time = self.get_clock().now()
        
        # --- Alvos de Posição/Orientação ---
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.orientation_to_keep = None # Trava a orientação inicial aqui


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
            '/crossbar_detected/boolean',
            self.crossbar_callback,
            qos_profile
        )
        self.crossbar_error_sub = self.create_subscription(Point,'/crossbar_detected/error',self.crossbar_error_callback,qos_profile)
        
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
            
            # Trava a orientação inicial na primeira vez que a pose é recebida
            if self.orientation_to_keep is None:
                self.orientation_to_keep = self.current_orientation
                self.get_logger().info("Orientação inicial travada.")
    
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

    def crossbar_error_callback(self, msg):
        if math.isnan(msg.x) or math.isnan(msg.y):
            self.crossbar_lost = True
            self.crossbar_error_x = 0.0
            self.crossbar_error_y = 0.0
        else:
            self.crossbar_lost = False
            self.crossbar_error_x = msg.x
            self.crossbar_error_y = msg.y
            
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
            if self.current_state_mav.connected and self.orientation_to_keep is not None:
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
            self.target_pose.pose.orientation = self.orientation_to_keep
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
                self.target_pose.pose.orientation = self.orientation_to_keep # Mantém orientação
                self.set_mission_state(MissionState.MOVING_TO_START)
            return

        # Estado 3: Movendo para a Posição Inicial
        if self.mission_state == MissionState.MOVING_TO_START:
            if self.check_distance_to_target():
                self.get_logger().info("Posição inicial alcançada. Iniciando seguidor de linha.")
                self.set_mission_state(MissionState.FOLLOWING_LINE)
            return

        # Estado 4: Seguindo a Linha
        if self.mission_state == MissionState.FOLLOWING_LINE:
            # Verifica a condição de parada
            if self.crossbar_detected:
                self.get_logger().info("Travessão detectado! Começando missão de centralização.")
                self.set_mission_state(MissionState.CENTERING_ON_CROSSBAR)
                return

            if self.line_lost:
                self.get_logger().warn("[MISSÃO] Linha perdida! Pairando no local...", throttle_duration_sec=1.0)
                # Não faz nada, fica pairando no waypoint atual
                # O mavros_stream_timer continua publicando a pose atual
            
            elif self.current_pose is None:
                 self.get_logger().warn("Seguindo linha sem pose! Pairando...", throttle_duration_sec=1.0)
                 # Paira se perder a pose
            
            else:
                # LINHA ENCONTRADA: Calcular o próximo waypoint
                
                # Suas orientações: -Y (frente), +X (esquerda)
                # Nosso erro: error = centro - cx.
                # Se linha está à ESQUERDA (cx < centro), error é POSITIVO 
                # Precisamos corrigir para a ESQUERDA (mover drone no eixo +X).
                
                base_x = self.current_pose[0]
                base_y = self.current_pose[1]

                # Cálculo da correção lateral (no eixo +X)
                if self.line_error > 0:
                    correcao_x = self.kp_strafe_left * self.line_error
                else:
                    correcao_x = self.kp_strafe_right * self.line_error

                target_x = base_x + correcao_x
                
                # Cálculo do passo para frente (no eixo -Y)
                target_y = base_y - self.foward_velocity

                # Atualizar o alvo
                self.target_pose.pose.position.x = target_x
                self.target_pose.pose.position.y = target_y
                
            return
        
        # Estado 5: Centralizando no Travessão
        if self.mission_state == MissionState.CENTERING_ON_CROSSBAR:
            
            # Condição de parada: Se perdermos o travessão, pairar
            if self.crossbar_lost:
                self.get_logger().warn("Perdi o travessão! Pairando no último local.")
                self.set_mission_state(MissionState.HOVERING_ON_TARGET)
                return

            # Lógica de Controle:
            if self.current_pose is None:
                self.get_logger().warn("Centralizando sem pose! Pairando...", throttle_duration_sec=1.0)
                return
            
            base_x = self.current_pose[0]
            base_y = self.current_pose[1]

            # Começa com o alvo parado (manter posição atual)
            target_x = base_x
            target_y = base_y
            
            # --- Lógica de 2 Estágios (Y primeiro, depois X) ---

            # ESTÁGIO 1: Corrigir Y (Frente/Trás)
            # Primeiro, nos posicionamos "sobre" o travessão
            if abs(self.crossbar_error_y) > self.centering_tolerance_y:
                self.get_logger().info(f"Estágio 1 (Y): Corrigindo Y... Erro: {self.crossbar_error_y:.1f}")
                
                # Corrige APENAS o Y
                correcao_y = self.kp_centering_y * self.crossbar_error_y
                target_y = base_y + correcao_y
                # Mantém o X parado (target_x = base_x)
            
            # ESTÁGIO 2: Corrigir X (Lateral)
            # Só executamos se o Y já estiver bom (temos a "visão inteira")
            elif abs(self.crossbar_error_x) > self.centering_tolerance_x:
                self.get_logger().info(f"Estágio 2 (X): Corrigindo X... Erro: {self.crossbar_error_x:.1f}")
                
                # Corrige APENAS o X
                correcao_x = self.kp_centering_x * self.crossbar_error_x
                target_x = base_x + correcao_x
                # Mantém o Y parado (target_y = base_y)

            # ESTÁGIO 3: Sucesso
            # Se Y está bom E X está bom
            else:
                self.get_logger().info("Centralização no travessão concluída! Pairando.")
                self.set_mission_state(MissionState.HOVERING_ON_TARGET)
                return
            
            # Atualizar o alvo
            self.target_pose.pose.position.x = target_x
            self.target_pose.pose.position.y = target_y
            return
    
        # Estado 6: Pairando no Alvo
        if self.mission_state == MissionState.HOVERING_ON_TARGET:
            # Não faz nada. Apenas paira.
            # O mavros_stream_loop continua publicando a última pose.
            # O usuário pode dar Ctrl+C para pousar.
            # Nesse estado ele vai diminuir sua altura
            self.target_pose.pose.position.z = 3.0
            return

        # Estado 7: Missão Completa (Pousando)
        if self.mission_state == MissionState.MISSION_COMPLETE:
            # Apenas espera...
            if self.get_time_in_state() > 10.0: # 10s para pousar
                self.get_logger().info("Pouso concluído. Encerrando.")
                rclpy.shutdown()
            return
            
        # Estado 6: Pouso de Emergência
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