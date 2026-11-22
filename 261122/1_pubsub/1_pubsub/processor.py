import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Processor(Node):
    def __init__(self):
        super().__init__('processor')
        self.x, self.y = 0,0
        self.subscription_ = self.create_subscription(String, 'key_input', self.callback, 10)
        self.publisher_ = self.create_publisher(String, 'key_output', 10)


    def callback(self, msg):
        key = msg.data
        if key == 'w' : self.y +=1
        elif key == 'a' : self.x -=1
        elif key == 's' : self.y -=1
        elif key == 'd' : self.x +=1
        
        out = String()
        out.data = f"({self.x},{self.y})"
        self.publisher_.publish(out)

def main():
    rclpy.init()
    node = Processor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
