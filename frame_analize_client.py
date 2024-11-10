import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class FrameAnalysisClient(Node):
    def __init__(self):
        super().__init__('frame_analysis_client')
        self.client = self.create_client(Trigger, 'analyze_frame')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio de análisis de frame...')
        
        self.request = Trigger.Request()

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    node = FrameAnalysisClient()
    response = node.send_request()
    if response.success:
        node.get_logger().info('Análisis completado: ' + response.message)
    else:
        node.get_logger().info('Falló el análisis: ' + response.message)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
