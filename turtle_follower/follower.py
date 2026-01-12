import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Follower(Node):
    def __init__(self):
        super().__init__("follower")
        self.leader_sub=self.create_subscription(Pose,'/leader_pose',self.leader_pose_callback,10)
        self.follower_pose_sub=self.create_subscription(Pose,'/turtle2/pose',self.follower_pose_callback,10)
        self.follower_cmd_vel=self.create_publisher(Twist,'/turtle2/cmd_vel',10)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.follower_x = 0.0
        self.follower_y = 0.0
        self.follower_theta = 0.0
        
        self.get_logger().info(
            'follower: following /leader_pose and commanding /turtle2/cmd_vel'
        )
        
    def leader_pose_callback(self,msg:Pose):
        self.leader_x=msg.x
        self.leader_y=msg.y

    def follower_pose_callback(self, msg: Pose):
        self.follower_x = msg.x
        self.follower_y = msg.y
        self.follower_theta = msg.theta

    def control_loop(self):
        dx = self.leader_x - self.follower_x
        dy = self.leader_y - self.follower_y
        distance = math.sqrt(dx*dx + dy*dy)
        cmd = Twist()
    
        if distance > 0.2:
            target_theta = math.atan2(dy, dx)
            angle_error = math.atan2(math.sin(target_theta - self.follower_theta), math.cos(target_theta - self.follower_theta))
            cmd.linear.x = max(min(1.5 * distance, 2.0), -2.0)
            cmd.angular.z = max(min(4.0 * angle_error, 4.0), -4.0)
        elif distance > 0.15:
            target_theta = math.atan2(dy, dx)
            angle_error = math.atan2(math.sin(target_theta - self.follower_theta), math.cos(target_theta - self.follower_theta))
            cmd.linear.x = max(min(1.0 * distance, 1.5), -1.5)
            cmd.angular.z = max(min(3.0 * angle_error, 3.0), -3.0)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
    
        self.follower_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node=Follower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    