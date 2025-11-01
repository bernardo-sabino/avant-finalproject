# Importação dos módulos básicos do ROS2
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # QOS: Controle de comunicação  por definição de políticas e configurações

# Importações de Visão Computacional
import cv2 # Biblioteca OpenCV para processamento de imagem
import numpy as np # Numpy para manipulação de arrays
from cv_bridge import CvBridge # Ponte para converter imagem ROS -> Imagem Open CV

# Interfaces
from sensor_msgs.msg import Image 
from std_msgs.msg import Float32, Bool

# Importação de módulos padrões 
import time
 
class VisionNode(Node):

    # Constructor de definição do nó e os atributos iniciais
    def __init__(self):
        super().__init__("drone_vision_node") # Inicializando o nó

        # Configurações da Comunicação 
        qos_profile = QoSProfile(

            reliability=ReliabilityPolicy.BEST_EFFORT, # Essa política define que o sistema não vai garantir que as mensagens serão entregues
            # Se um pacote se perder ele não vai tentar reenviar esses dados


            history=HistoryPolicy.KEEP_LAST, # O nó só armazenerá em seu buffer um número fixo das mensagens mais rescentes


            depth=1 # Esse é o número fixo de mensagens armazenadas no buffer
        )

        self.bridge = CvBridge()   # Inicilizando a ponte (CVBridge)
        self.imagem_recebida = False # Flag para saber se a imagem foi recebida

        # Dimensões da imagem (definidas no primeiro frame)
        self.imagem_largura = 0
        self.imagem_altura = 0
        self.imagem_centro = 0

        # Publisher
        self.error_pub = self.create_publisher(Float32, '/line_follower/error', qos_profile)
        self.crossbar_pub = self.create_publisher(Bool, '/crossbar_detected', 10) 

        # Subscriber
        self.imagem_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.imagem_callback,
            qos_profile
        )


        self.get_logger().info("Nó drone_vision_node inicializado com sucesso")  # Validando que se o constructor
                                                                                 # chegou até aqui. 
    def imagem_callback(self, msg):

        """
        Método chamado a cada frame da imagem carregado, é o lugar que todo processamento de imagem ocorre
        """

        # Conversão de Imagem ROS2 para Imagem OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")      
            self.imagem_recebida = True
        except Exception as e:
            self.get_logger().error(f"Falha ao converter a imagem: {e}")

        # Definição das dimensões
        if self.imagem_largura == 0:
            h,w,d = frame.shape
            self.imagem_largura = w
            self.imagem_altura = h
            self.imagem_centro = w // 2 

        # Processamento de imagem

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Converter para HSV

        lower_blue = np.array([100, 150, 50]) # Definir o range da cor Azul (lower)
        upper_blue = np.array([140, 255, 255]) # Definir o range da cor Azul (upper)

        mask = cv2.inRange(hsv_frame, lower_blue, upper_blue) # Cria a máscara

        M = cv2.moments(mask) # Encontrar o Centro da Linha (Centróide)

        error_msg = Float32() # Cria a mensagem de erro
        
        if M["m00"] > 0: # A linha foi detectada

            cx = int(M["m10"] / M["m00"]) # Calcula o centro (cx) da linha na imagem
             
            error = self.imagem_centro - cx   # Erro = Posição do Centro da Imagem - Posição do Centro da Linha
            
            error_msg.data = float(error)
        
        else:   # A linha não foi detectada
            error_msg.data = float('nan')
        
        self.error_pub.publish(error_msg) # Publicando o erro para manipulação por parte do nó de navegação

        # Detecção do Travessão 

        lower_crossbar = np.array([0, 150, 100])   
        upper_crossbar = np.array([10, 255, 255])  

        
        mask_crossbar = cv2.inRange(hsv_frame, lower_crossbar, upper_crossbar)
        
        M_crossbar = cv2.moments(mask_crossbar) # Calcular a "área" (peso) da máscara do travessão
        
        crossbar_msg = Bool() # Cria a mensagem booleana

        # Se a área de pixels for maior que nosso threshold (para evitar ruído)
        if M_crossbar["m00"] > 100:
            crossbar_msg.data = True 
        else:
            crossbar_msg.data = False 
            
        # Publica o status (True ou False)
        self.crossbar_pub.publish(crossbar_msg)


# ===== Main =====
def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Erro na validação de imagem: {e}.")
    finally:
        node.get_logger().info("Encerrando nó de visão computacional, o drone está cego!.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

