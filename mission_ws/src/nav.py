#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import math, time, sys, signal


class DroneMissionNode(Node):
    def __init__(self):
        super().__init__('drone_mission_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.current_state = State()
        self.current_pose = None

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)

        self.get_logger().info('DroneMissionNode inicializado')
        self.wait_for_connection()

        signal.signal(signal.SIGINT, self.emergency_shutdown)

    # ==================== CALLBACKS ====================
    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        if msg and msg.pose:
            self.current_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        else:
            self.current_pose = None

    # ==================== CONEXÃO ====================
    def wait_for_connection(self):
        self.get_logger().info("Aguardando conexão com FCU...")
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self)
            time.sleep(0.2)
        self.get_logger().info("FCU conectado!")

    # ==================== SERVIÇOS ====================
    def set_mode(self, custom_mode):
        client = self.create_client(SetMode, '/mavros/set_mode')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Serviço /mavros/set_mode não disponível')
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

    def emergency_land(self):
        self.get_logger().warn(">>> EMERGÊNCIA: tentando pouso imediato!")
        client = self.create_client(CommandTOL, '/mavros/cmd/land')
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(">>> Serviço /mavros/cmd/land não disponível")
            return
        req = CommandTOL.Request()
        req.altitude = 0.0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().warn(">>> Drone pousando com sucesso!")
        else:
            self.get_logger().error(">>> Falha no pouso de emergência!")

    # ==================== MOVIMENTOS ====================
    def set_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.local_pos_pub.publish(msg)

    def move_to_target(self, target, tol=0.5, timeout=30):
        rate = self.create_rate(10)
        self.get_logger().info(f"Movendo para {target}")
        start_time = time.time()

        while rclpy.ok():
            if not self.current_pose:
                rclpy.spin_once(self)
                continue

            error = [target[i] - self.current_pose[i] for i in range(3)]
            if all(abs(e) < tol for e in error):
                self.get_logger().info("Destino alcançado!")
                break

            self.set_pose(*target)

            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout ao tentar alcançar o destino")
                break

            rclpy.spin_once(self)
            rate.sleep()

    # ==================== MISSÃO ====================
    def arm_and_takeoff(self, altitude=1.0):
        self.get_logger().info("Preparando para decolagem...")
        if not self.set_mode('GUIDED'):
            self.get_logger().error("Falha ao definir modo GUIDED")
            return False
        if not self.arm_drone():
            self.get_logger().error("Falha ao armar o drone")
            return False
        time.sleep(2)
        if not self.takeoff_drone(altitude):
            self.get_logger().error("Falha na decolagem")
            return False
        time.sleep(8)
        return True

    # ==================== SEGURANÇA ====================
    def emergency_shutdown(self, sig, frame):
        self.get_logger().warn(">>> Ctrl+C detectado — executando pouso de emergência...")
        self.emergency_land()
        sys.exit(0)


# ==================== MAIN ====================
def main(args=None):
    rclpy.init(args=args)
    node = DroneMissionNode()

    try:
        if node.arm_and_takeoff(1.0):
            node.move_to_target([0.0, 2.0, 1.0])
            node.move_to_target([2.0, 2.0, 1.0])
            node.move_to_target([2.0, 0.0, 1.0])
            node.move_to_target([0.0, 0.0, 1.0])
            node.land_drone()
    except Exception as e:
        node.get_logger().error(f"Erro inesperado: {e}")
        node.emergency_land()
    finally:
        node.get_logger().info("Encerrando missão.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()