import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from chassis_msgs.msg import ChassisCommand
from chassis_msgs.msg import ChassisStatus
import serial
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
import time


class ChassisControllerPublisher(Node):
    baud=115200
    ser=serial.Serial('/dev/ttyUSB1', baud, timeout=1)
    
    def __init__(self):
        super().__init__('chassis_controller_publisher')
        self.chassis_cmd_publisher_ = self.create_publisher(ChassisCommand, 'chassis_cmd', 10)
        self.yaw = 0
        self.pitch = 0
        self.dis = 0
        self.t1=0
        self.t2=0
        self.t3=0
        self.t4=0
        
        
    def send_data(self, data):
        # 打包数据，这里假设你想将三个浮点数打包为字符串，用逗号分隔
        packed_data = ','.join(map(str, data))
        # 发送数据
        self.ser.write(packed_data.encode())
        self.get_logger().info('Sent data: {}'.format(packed_data))


    def send_command(self, data: List[float]):
        msg = ChassisCommand()
        msg.linear_velocity = linear_velocity
        msg.angular_velocity = angular_velocityS
        msg.model = 4
        msg.shoot_model = 10
        self.chassis_cmd_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s,%s,%s,%s"' 
        %(msg.model,
        msg.shoot_model,
        msg.linear_velocity,
        msg.angular_velocity ))     # 输出日志信息，提示已经完成话题发布
        
        send_data =struct.pack('ddfffff',
        %(msg.model,
        msg.shoot_model,
        self.yaw,
        self.pitch ,
        self.dis,
        msg.linear_velocity,
        msg.angular_velocity))
        self.send_data(send_data)
        
        
    def spin1(self,data):
         while rclpy.ok(): 
            linear_vel = data.linear.x
            angular_vel = data.angular.z
            self.t3=time.time()
            self.send_command()
            rclpy.spin_once(self)
            
            
    def spin2(self,data):
        while rclpy.ok(): 
            self.yaw = data.yaw 
            self.pitch = data.pitch
            self.dis = data.dis
            msg.is_aimed = data.is_aimed
            msg.target_exists = data.is_target_exists
        
        self.t1=time.time()
        self.send_command()  # 调用处理数据的方法
        
        
    def publish_initial_pose():
    	rospy.init_node('amcl_initial_pose_publisher', anonymous=True)
    	pub_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    	rospy.sleep(0.5)  # 等待1秒，确保发布者和订阅者之间建立连接
    	initial_pose = PoseWithCovarianceStamped()
    	initial_pose.header.stamp = rospy.Time.now()
   	initial_pose.header.frame_id = "map"
    # 设置初始位置的坐标
    	initial_pose.pose.pose.position.x = 22  # 设置x坐标
    	initial_pose.pose.pose.position.y = 8.5  # 设置y坐标
    	initial_pose.pose.pose.position.z = 0.0  # 设置z坐标
    # 设置初始位置的方向
    	initial_pose.pose.pose.orientation.x = 0.0
    	initial_pose.pose.pose.orientation.y = 0.0
    	initial_pose.pose.pose.orientation.z = -1.0
    	initial_pose.pose.pose.orientation.w = 0
    	pub_amcl.publish(initial_pose)
    	rospy.loginfo("发布初始位置%s,%s",initial_pose.pose.pose.position.x,initial_pose.pose.pose.position.y)

    	baud = 115200
    	ser = serial.Serial('/dev/UART2', baud, timeout=1)

    	number=0  # 发布初始化激活
    	q=struct.pack('i',number)
    	ser.write(q)
    
             

    
        
    

def main(args=None):
    rclpy.init(args=args)
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
    chassis_controller_publisher = ChassisControllerPublisher()
    rclpy.spin1(chassis_controller_publisher)
    rospy.Subscriber('/cmd_vel', Twist, send.callback1,queue_size=10)  
    #rclpy.spin2(chassis_controller_publisher)
    chassis_controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':	
    main()
