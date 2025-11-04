# Importação dos módulos básicos do ROS2
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 

# Importações do MatplotLib
import matplotlib.pyplot as plt 
import matplotlib.animation as animation

# Interfaces
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point  

# Importação de módulos padrões 
import time
from collections import deque
import threading

class DataNode(Node):

    def __init__(self):
        super().__init__("data_analysis_node") 

        # ==== Configurações da Comunicação ==== 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 
        )

        # ==== Atributos Essenciais ====
        self.is_crossbar_detected = False 
        self.start_time = time.time()
        
        # --- Deques para o Gráfico 1 (Seguidor de Linha) ---
        self.line_time_data = deque(maxlen=200)
        self.line_error_data = deque(maxlen=200)

        # --- Deques para o Gráfico 2 (Centralização no Alvo) ---
        self.crossbar_time_data = deque(maxlen=100) 
        self.crossbar_error_x_data = deque(maxlen=100)
        self.crossbar_error_y_data = deque(maxlen=100)


        # ==== Configuração do Matplot (Gráfico 1) ==== 
        self.fig1, self.ax1 = plt.subplots()
        fig1_manager = self.fig1.canvas.manager
        if fig1_manager:
            fig1_manager.set_window_title('Gráfico 1: Seguimento de Linha')
        self.line1, = self.ax1.plot([], [], label='Erro Lateral')
        self.ax1.axhline(y=0, color='r', linestyle='--', linewidth=1)
        self.ax1.set_xlabel("Tempo (s)")
        self.ax1.set_ylabel("Erro (pixels)")
        self.ax1.set_title("Estado: Seguindo a Linha")
        self.ax1.set_ylim(-350, 350) 
        self.ax1.set_xlim(0, 30)
        self.ax1.legend()
        self.ani1 = animation.FuncAnimation(self.fig1, self.update_plot1, interval=100, cache_frame_data=False)

        # ==== Configuração do Matplot (Gráfico 2) ==== 
        self.fig2, self.ax2 = plt.subplots()
        fig2_manager = self.fig2.canvas.manager
        if fig2_manager:
            fig2_manager.set_window_title('Gráfico 2: Centralização no Alvo')
        self.line2_x, = self.ax2.plot([], [], label='Erro X (Lateral)')
        self.line2_y, = self.ax2.plot([], [], label='Erro Y (Frontal/Paralaxe)')
        self.ax2.axhline(y=0, color='r', linestyle='--', linewidth=1)
        self.ax2.set_xlabel("Tempo (s)")
        self.ax2.set_ylabel("Erro (pixels)")
        self.ax2.set_title("Estado: Centralizando no Alvo")
        self.ax2.set_ylim(-350, 350) 
        self.ax2.set_xlim(0, 10) 
        self.ax2.legend()
        self.ani2 = animation.FuncAnimation(self.fig2, self.update_plot2, interval=100, cache_frame_data=False)

        # ==== Subscribers (Ouvindo os 3 tópicos) ====
        self.line_error_sub = self.create_subscription(
            Float32,
            '/line_follower/error', 
            self.line_error_callback,
            qos_profile
        )
        
        self.crossbar_detected_sub = self.create_subscription(
            Bool,
            '/crossbar_detected/boolean',
            self.crossbar_detected_callback,
            qos_profile
        )
        
        self.crossbar_error_sub = self.create_subscription(
            Point,
            '/crossbar_detected/error',
            self.crossbar_error_callback,
            qos_profile
        )

        # ==== Log ==== 
        self.get_logger().info("Nó data_analysis_node inicializado com dois gráficos.")
    
    # === Callbacks ===
    
    def crossbar_detected_callback(self, msg):
        if msg.data != self.is_crossbar_detected:
            self.is_crossbar_detected = msg.data
            if self.is_crossbar_detected:
                self.get_logger().info("Travessão detectado! Trocando para Gráfico 2 (Centralização).")
                self.crossbar_time_data.clear()
                self.crossbar_error_x_data.clear()
                self.crossbar_error_y_data.clear()
            else:
                self.get_logger().info("Travessão perdido. Trocando para Gráfico 1 (Seguimento de Linha).")
                self.line_time_data.clear()
                self.line_error_data.clear()

    def line_error_callback(self, msg):
        if not self.is_crossbar_detected:
            current_time = time.time() - self.start_time
            self.line_time_data.append(current_time)
            self.line_error_data.append(msg.data)

    def crossbar_error_callback(self, msg):
        if self.is_crossbar_detected:
            current_time = time.time() - self.start_time
            self.crossbar_time_data.append(current_time)
            self.crossbar_error_x_data.append(msg.x) 
            self.crossbar_error_y_data.append(msg.y) 

    # === Funções de Atualização dos Gráficos ===

    def update_plot1(self, frame): 
        if len(self.line_time_data) > 0:
            self.line1.set_data(list(self.line_time_data), list(self.line_error_data))
            min_t = max(0.0, self.line_time_data[-1] - 30.0)
            max_t = min_t + 30.0
            self.ax1.set_xlim(min_t, max_t)
            self.fig1.canvas.draw_idle()

    def update_plot2(self, frame): 
        if len(self.crossbar_time_data) > 0:
            self.line2_x.set_data(list(self.crossbar_time_data), list(self.crossbar_error_x_data))
            self.line2_y.set_data(list(self.crossbar_time_data), list(self.crossbar_error_y_data))
            min_t = self.crossbar_time_data[0] 
            max_t = min_t + 10.0
            if self.crossbar_time_data[-1] > max_t:
                 min_t = max(0.0, self.crossbar_time_data[-1] - 10.0)
                 max_t = min_t + 10.0
            
            self.ax2.set_xlim(min_t, max_t)
            self.fig2.canvas.draw_idle()

        
# ===== Main =====
def main():
    rclpy.init()
    node = DataNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    try:
        plt.show(block=True) 
    except KeyboardInterrupt: 
        node.get_logger().info("Nó de análise de dados encerrado pelo usuário (Ctrl+C).")
    except Exception as e:
        node.get_logger().error(f"Erro: {e}.")
    finally:
        node.get_logger().info("Encerrando nó de análise de dados.")
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join() 

if __name__ == '__main__':
    main()