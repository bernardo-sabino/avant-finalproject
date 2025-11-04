# Importação dos módulos básicos do ROS2
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 

# Importações de Visão Computacional
import cv2 
import numpy as np 
from cv_bridge import CvBridge 

# Interfaces
from sensor_msgs.msg import Image 
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point


# Importação de módulos padrões 
import time
 
class VisionNode(Node):

    def __init__(self):
        super().__init__("drone_vision_node") 

        # Configurações da Comunicação 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 
        )

        self.bridge = CvBridge()   
        self.imagem_recebida = False 

        # Dimensões da imagem (definidas no primeiro frame)
        self.imagem_largura = 0
        self.imagem_altura = 0
        self.imagem_centro = 0

        # Limite de área para detecção (Tuning)
        self.crossbar_area_threshold = 100.0 


        # Offset de centralização 
        self.Y_TARGET_OFFSET_PX = -220
        self.X_TARGET_OFFSET_PX = -255

        # Máscara da Região de Interesse (ROI)
        self.roi_mask = None # Inicializado como None

        # Publisher
        self.error_pub = self.create_publisher(Float32, '/line_follower/error', qos_profile) 
        self.crossbar_pub = self.create_publisher(Bool, '/crossbar_detected/boolean', qos_profile) 
        self.crossbar_error_pub = self.create_publisher(Point, '/crossbar_detected/error', qos_profile)

        # Subscriber
        self.imagem_sub = self.create_subscription(
            Image,
            '/camera/image', 
            self.imagem_callback,
            qos_profile
        )

        self.get_logger().info("Nó drone_vision_node (ROI Corrigido) inicializado.")
                                                                                 
    def imagem_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")      
            self.imagem_recebida = True
        except Exception as e:
            self.get_logger().error(f"Falha ao converter a imagem: {e}")
            return # Sai da função se a imagem falhar

        # Lógica do ROI 
        if self.roi_mask is None:
            h, w, d = frame.shape
            self.imagem_largura = w
            self.imagem_altura = h
            self.imagem_centro = w // 2 
            self.get_logger().info(f"Dimensões da imagem definidas: {w}x{h}")

            # Cria uma "folha" preta do tamanho da imagem
            self.roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            
            # Vamos olhar apenas para a METADE DE CIMA da imagem
            start_y = 0                         # Começa no topo
            end_y = self.imagem_altura // 2     # Vai até o meio
            
            # Desenha um retângulo branco nessa área
            cv2.rectangle(self.roi_mask, (0, start_y), (self.imagem_largura, end_y), 255, -1)
            
            self.get_logger().info(f"Máscara ROI criada. Vendo apenas de y={start_y} até y={end_y}")

        # Converta para HSV 
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

        # --- Tarefa 1: Detecção da Linha Azul ---
        lower_blue = np.array([100, 150, 50]) 
        upper_blue = np.array([140, 255, 255]) 
        mask_blue_original = cv2.inRange(hsv_frame, lower_blue, upper_blue) 

        # Aplica a Máscara ROI (só nos importamos com o azul na metade de cima)
        mask_blue = cv2.bitwise_and(mask_blue_original, mask_blue_original, mask=self.roi_mask)

        M_blue = cv2.moments(mask_blue) 

        error_msg = Float32() 
        
        if M_blue["m00"] > 0: # A linha azul foi detectada (dentro do ROI)
            cx = int(M_blue["m10"] / M_blue["m00"]) 
            error = self.imagem_centro - cx   
            error_msg.data = float(error)
        else:   # A linha azul NÃO foi detectada (no ROI)
            error_msg.data = float('nan') 
        
        self.error_pub.publish(error_msg) 

        # --- Tarefa 2: Detecção do Travessão (Vermelho) ---
        lower_red_1 = np.array([0, 150, 100])   
        upper_red_1 = np.array([10, 255, 255])  
        mask_red_1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)

        lower_red_2 = np.array([170, 150, 100]) 
        upper_red_2 = np.array([180, 255, 255]) 
        mask_red_2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
    
        mask_crossbar = mask_red_1 + mask_red_2
        M_crossbar = cv2.moments(mask_crossbar)

        crossbar_msg = Bool() 
        crossbar_error_msg = Point() # Mensagem para o erro X,Y

        if M_crossbar["m00"] > self.crossbar_area_threshold:        
            # VIMOS O TRAVESSÃO
            crossbar_msg.data = True 

            # CALCULAR O CENTRO DELE
            cx_red = int(M_crossbar["m10"] / M_crossbar["m00"])
            cy_red = int(M_crossbar["m01"] / M_crossbar["m00"])

            # CALCULAR O ERRO X, Y (em pixels)

            target_cx = (self.imagem_largura // 2) + self.X_TARGET_OFFSET_PX
            error_x = float(target_cx - cx_red)
        
            target_cy = (self.imagem_altura // 2) + self.Y_TARGET_OFFSET_PX
            error_y = float(target_cy - cy_red) 
        
            crossbar_error_msg.x = error_x
            crossbar_error_msg.y = error_y

        else:

            # NÃO VEMOS O TRAVESSÃO
            crossbar_msg.data = False 

            # Publica "nan" para o erro
            crossbar_error_msg.x = float('nan')
            crossbar_error_msg.y = float('nan')

        # Publica OS DOIS sinais
        self.crossbar_pub.publish(crossbar_msg)
        self.crossbar_error_pub.publish(crossbar_error_msg)
        
# ===== Main =====
def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        node.get_logger().info("Nó de visão encerrado pelo usuário (Ctrl+C).")
    except Exception as e:
        node.get_logger().error(f"Erro na validação de imagem: {e}.")
    finally:
        node.get_logger().info("Encerrando nó de visão computacional.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()