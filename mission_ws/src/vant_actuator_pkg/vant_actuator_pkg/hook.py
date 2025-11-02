# Importação dos módulos básicos do ROS2
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 
from std_msgs.msg import Empty


# Importação da biblioteca de GPIO do Jetson
import RPi.GPIO as GPIO
import time

# --- Constantes de Hardware ---
# Mapeamento de pinos 
PIN_5V_VCC = 2  
PIN_GND = 6     
PIN_RELAY_IN = 7 

class ActuatorNode(Node):

    def __init__(self):
        super().__init__("actuator_node")

        # Configurações da Comunicação 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.relay_pin = PIN_RELAY_IN
        self.hook_released = False 
        self.setup_gpio()

        # Subscriber
        self.subscription = self.create_subscription(
            Empty,
            '/drone_centered',
            self.drop_hook_callback,
            qos_profile)
        
        self.get_logger().info("Nó atuador inicializado. Eletroímã LIGADO. Aguardando sinal...")

    def setup_gpio(self):
        """Configura os pinos do Jetson Orin Nano."""
        try:
            GPIO.setmode(GPIO.BOARD) # Usa a numeração física dos pinos 
            
            # Configura o pino 7 como SAÍDA
            # Começa em LOW para manter o eletroímã LIGADO (segurando o gancho)
            GPIO.setup(self.relay_pin, GPIO.OUT, initial=GPIO.LOW)

        except Exception as e:
            self.get_logger().error(f"Falha ao configurar GPIO: {e}")
            rclpy.shutdown()

    def drop_hook_callback(self, msg):
        """Callback chamado quando a mensagem /drone_centered chega."""
        
        if self.hook_released:
            self.get_logger().warn("Sinal recebido, mas gancho já foi solto.")
            return

        self.get_logger().info("Sinal de centralização recebido! Soltando o gancho...")
        self.hook_released = True

        try:
            # Envia HIGH para o relé (Active LOW)
            # Isso DESLIGA o relé, abrindo o circuito e desligando o eletroímã
            GPIO.output(self.relay_pin, GPIO.HIGH)
            
            self.get_logger().info("ELETROÍMÃ DESLIGADO. Gancho solto.")
            
            # Espera 5 segundos antes de desligar (apenas para garantir)
            time.sleep(5)
            
            # Desliga o nó após a tarefa estar completa
            self.get_logger().info("Missão de soltura concluída. Desligando nó atuador.")
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Falha ao acionar o relé: {e}")

    def on_shutdown(self):
        """Limpa os pinos GPIO ao desligar."""
        self.get_logger().info("Limpando pinos GPIO...")
        GPIO.cleanup()

# ===== MAIN =====

def main():
    rclpy.init()
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()