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


            depth=10 # Esse é o número fixo de mensagens armazenadas no buffer
        )

        self.bridge = CvBridge()   # Inicilizando a ponte (CVBridge)
        self.imagem_recebida = False # Flag para saber se a imagem foi recebida

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
        Método chamado a cada frame da imagem carregado
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")      
            self.imagem_recebida = True
        except Exception as e:
            self.get_logger().error(f"Falha ao converter a imagem: {e}")

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
