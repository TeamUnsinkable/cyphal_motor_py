import rclpy
from rclpy.node import Node
from cyphal_motor_py.cyphal_motor import CyphalMotor

from std_msgs.msg import Float32

import asyncio

class CyphalMotorNode(Node):
    def __init__(self):
        super().__init__("motor")
        self.motor = CyphalMotor(self.motor_id, self.status_id, self.rat_setpoint_id, self.readiness_id)

        # Declare Subscriber
        self.sub_setpoint = self.create_subscription(Float32, "setpoint", self.cb_setpoint)
        self.setpoint = 0.0

        self.timer = self.create_timer(self.deadman_timeout, self.cb_timer)

    def cb_setpoint(self, msg):
        self.setpoint = msg.data
        
    
    def cb_arm(self, msg):
        # enable timer 
        self.timer.reset()
        # send arm readiness command - 3
        asyncio.run( self.motor.publish_readiness(3) )
        

    def cb_disarm(self, msg):
        # disable timer
        self.timer.cancel()
        # send disarm readiness command - 2
        asyncio.run( self.motor.publish_readiness(2) )
        
    
    def cb_timer(self):
        asyncio.run( self.motor.publish_setpoint(self.setpoint, 0) )

    def _get_parameters(self):
        # Declare Parameters
        self.declare_parameter("motor_id", 120)
        self.declare_parameter("status_id", 1200)
        self.declare_parameter("rat_setpoint_id", 1100)
        self.declare_parameter("readiness_id", 0)
        self.declare_parameter("deadman_timeout", 0.5)

        # Get Parameters
        self.motor_id        =  self.get_parameter("motor_id")
        self.status_id       =  self.get_parameter("status_id")
        self.rat_setpoint_id =  self.get_parameter("rat_setpoint_id")
        self.readiness_id    =  self.get_parameter("readiness_id")
        self.deadman_timeout =  self.get_parameter("deadman_timeout")

def main():
    rclpy.init()
    node = CyphalMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()