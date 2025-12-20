import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class Teleop_Leader(Node):
    def __init__(self):
        super().__init__("teleop_leader")
        self.leader_pub=self.create_publisher(Pose,'/leader_pose',10)
        self.turtle1_sub=self.create_subscription(Pose,'/turtle1/pose',self.republish_as_leader_pose,10)
        self.get_logger().info(
            'teleop_leader: bridging /turtle1/pose -> /leader_pose'
        )



    def republish_as_leader_pose(self,msg:Pose):
        self.leader_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=Teleop_Leader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
