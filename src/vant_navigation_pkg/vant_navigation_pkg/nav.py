# Importação dos módulos básicos do ROS2
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # QOS: Controle de comunicação  por definição de políticas e configurações

# Importação dos módulos do MAVROS para comunicação com o "FCU"
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State

# Importação da interface de posição 
from geometry_msgs.msg import PoseStamped

# Importação de módulos padrões 
import math, time, sys, signal
 
class DroneNode(Node):

    # Constructor de definição do nó e os atributos iniciais
    def __init__(self):
        super().__init__("drone_mission_node") # Inicializando o nó

        # Configurações da Comunicação 
        qos_profile = QoSProfile(

            reliability=ReliabilityPolicy.BEST_EFFORT, # Essa política define que o sistema não vai garantir que as mensagens serão entregues
            # Se um pacote se perder ele não vai tentar reenviar esses dados


            history=HistoryPolicy.KEEP_LAST, # O nó só armazenerá em seu buffer um número fixo das mensagens mais rescentes


            depth=10 # Esse é o número fixo de mensagens armazenadas no buffer
        )
        
        self.current_state = State() # O estado atual retornado pelo pela MAVLink (apenas inicializando antes de pegar a resposta no topic)
        self.current_pose = None # Inicializando a posição atual do drone
        self.current_orientation = None # Orientação atual do Drone (rotation)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, \
        '/mavros/setpoint_position/local', qos_profile)  # Nó que publica comandos de localização do drone

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', \
        self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, \
        '/mavros/local_position/pose', self.pose_cb, qos_profile)

        self.get_logger().info("Nó drone_mission_node inicializado com sucesso") # Validando que se o constructor
                                                                                 # chegou até aqui. 
        self.wait_for_connection()

    # ===== Método Callback =====
    def state_cb(self, msg):
        self.current_state = msg
    def pose_cb(self, msg):
        if msg and msg.pose: 
            self.current_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.current_orientation = msg.pose.orientation
        else:
            self.current_pose = None
            self.current_orientation = None
    
    # ===== Método de Conexão com a FCU =====

        """
        A logica desse método é simples, testar duas afirmações: enquanto o nó tiver rodando & a conexão com o MAVROS inválida
        tente continuamente atender as pendências do nó. Ou seja, atender todas as tentativas de publicação e inscrição em canais
        incluindo o canal do MAVROS (e principalmente ele)
        """

    def wait_for_connection(self):
        self.get_logger().info("FCU sendo conectada...")
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self)
            time.sleep(0.2)
        self.get_logger().info("A FCU foi conectada com sucesso!")

    # ===== Serviços do Drone =====

        """
        O set_mode atende uma lógica de client básica. Ele faz a requisição do modo que o drone deve estar
        e espera uma resposta, validando se a resposta está demorando para vir ou não.
        """

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

        """
        O arm_drone atende uma lógica de client. Ele envia para o service que o drone deve ser armado e depois
        retorna, através de uma expressão booleana, o sucesso desse request.
        """

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

        """
        O método takeoff_drone também executa uma relação de client, definindo que o drone deve voar e a altitude 
        definida para esse voo.
        """

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

        """
        Intuitivamente, land_drone é um client que faz o request da aterrisagem do drone. Na prática, ele usa
        a mesma lógica do takeoff_drone, mas definindo sua altitude para 0.
        """

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

    # ===== MÉTODOS DE MOVIMENTAÇÃO =====
    
    def go_to_position(self, x, y, z, tolerance=0.2):

        """
        Move o drone para uma posição (x, y, z) e espera até chegar.
        """
        self.get_logger().info(f"[MISSÃO] Iniciando a missão do travessão: movendo para uma visualização melhor das linhas")

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map' # Usando o frame 'map' (coordenadas do mundo)
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)

        # Orientação padrão (drone virado para a frente, sem rotação)
        target_pose.pose.orientation = self.current_orientation

        while rclpy.ok():
            # Atualiza o timestamp (importante para o MAVROS)
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Publica a posição alvo
            self.local_pos_pub.publish(target_pose)

            # Processa callbacks (ESSENCIAL para atualizar self.current_pose)
            rclpy.spin_once(self, timeout_sec=0.1) 

            # Verifica se já temos uma Posição Atual
            if self.current_pose is None:
                self.get_logger().warn("[POSIÇÃO] Aguardando posição atual do drone...")
                time.sleep(0.5)
                continue

            # Calcula a distância até o alvo
            current_x, current_y, current_z = self.current_pose
            dx = current_x - x
            dy = current_y - y
            dz = current_z - z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)

            # Verifica se o drone chegou à posição (dentro da tolerância)
            if distance < tolerance:
                self.get_logger().info("Posição alcançada com sucesso!")
                break
        
        # Espera 1 segundo para estabilizar no ponto antes de prosseguir
        time.sleep(1)
        return True

    # ===== Iniciando o Drone =====

    """

    Esses métodos prepara todas as etapas necessárias para o drone
    funcionar.
    
    A lógica é: 
    1. Mudar o modo para "GUIDED" (para aceitar comandos).
    2. Armar o drone.
    3. Esperar os motores estabilizarem.
    4. Comandar a decolagem para uma altitude.
    5. Esperar o drone subir.
    
    Se qualquer etapa falhar, a função para imediatamente e retorna False.
    """

    def arm_and_takeoff(self, altitude=1.0):   # altitude=1.0 define uma altura padrão de 1 metro se nenhuma for especificada
        self.get_logger().info("A decolagem está começando...")

        # MUDAR O MODO
        # Tenta chamar o serviço 'set_mode' para "GUIDED".
        # Este é o primeiro ponto de verificação.
        if not self.set_mode('GUIDED'):
            # Se 'set_mode' retornar False, loga o erro e aborta a função.
            self.get_logger().error("Falha ao definir modo GUIDED")
            return False # Indica falha na sequência

        # ARMAR O DRONE
        # Se a mudança de modo foi bem-sucedida, tenta armar o drone.
        # Este é o segundo ponto de verificação.
        if not self.arm_drone():
            # Se 'arm_drone' retornar False, loga o erro e aborta.
            self.get_logger().error("Falha ao armar o drone")
            return False # Indica falha na sequência

        # PAUSA DE ESTABILIZAÇÃO
        # Se o drone armou, espera 2 segundos.
        # Isso dá tempo para os motores atingirem a rotação de marcha lenta
        # e o controlador se estabilizar antes de subir.
        time.sleep(2)

        # COMANDAR DECOLAGEM
        # Tenta chamar o serviço 'takeoff_drone' com a altitude alvo.
        # Este é o terceiro ponto de verificação.
        if not self.takeoff_drone(altitude):
            # Se 'takeoff_drone' retornar False, loga o erro e aborta.
            self.get_logger().error("Falha na decolagem")
            return False # Indica falha na sequência

        # PAUSA DE EXECUÇÃO
        # Se o comando de decolagem foi ENVIADO com sucesso, espera 8 segundos.
        # O serviço 'takeoff_drone' retorna True quando o comando é enviado,
        # NÃO quando o drone CHEGA na altitude.
        # Esta pausa dá tempo para o drone fisicamente subir.
        time.sleep(8)

        # SUCESSO
        # Se todas as etapas passaram, retorna True.
        self.get_logger().info("Decolagem concluída com sucesso.")
        return True


# ===== Main =====
def main():
    rclpy.init()
    node = DroneNode()
    try:
        if node.arm_and_takeoff(3.0): 
            time.sleep(8)
            node.go_to_position(0,-1,3)
            node.go_to_position(0,0,3)
            node.land_drone()
    except Exception as e:
        node.get_logger().error(f"Erro na missão: {e}. Acionando pouso.")
        node.land_drone() 
    finally:
        node.get_logger().info("Encerrando missão.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

