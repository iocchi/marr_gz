import time, threading, math, os
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

NODE_NAME = 'test_control'

class TestControl(Node):

    def __init__(self, args):
        
        rclpy.init()

        super().__init__(NODE_NAME)
        
        # rates
        self.rate10 = self.create_rate(10) # Hz
        self.rate100 = self.create_rate(100) # Hz

        # function to run
        self.fn = args.fn
        self.control_interface = 'position'
        if 'velocity' in  self.fn:
            self.control_interface = 'velocity'
        if 'effort' in  self.fn:
            self.control_interface = 'effort'

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()
        time.sleep(0.2)

        print("Setting use_sim_time to True ...")
        os.system(f"ros2 param set {NODE_NAME} use_sim_time True")
        '''  DOES NOT WORK !!!
        pp = Parameter('use_sim_time', ParameterType.PARAMETER_BOOL, True)
        self.set_parameters([pp])
        '''    

        print(f"use_sim_time = {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")

        # Wait for /clock to be active if using sim time
        print("Waiting for clock ...")
        if self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            self.get_logger().info('Using simulated time. Waiting for /clock to be active...')
            while not self.get_clock().now().to_msg().sec > 0 and rclpy.ok():
                time.sleep(0.1) # Use real-time sleep while waiting for sim time
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(f"Simulated time is now active: {self.get_clock().now().to_msg().sec} seconds")

        self.rate10.sleep()

        # publishers

        self.pub_cmd_vel = self.create_publisher(TwistStamped, f'/ddrive_controller/cmd_vel', 10)
        self.pub_arm_cmd = self.create_publisher(Float64MultiArray, f'/arm_{self.control_interface}_controller/commands', 10)

        self.rate10.sleep()
 
        self.get_logger().info(f'{NODE_NAME} node initialized ')



    # Publish

    def publish_cmd_vel(self, lx, az, ts=1):
        msg = TwistStamped()
        msg.twist.linear.x = float(lx)
        msg.twist.angular.z = float(az)
        self.get_logger().info(f'Publishing cmd_vel: {lx:.3f} {az:.3f} time: {ts:.2f} s')
        for _ in range(int(ts*100)):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd_vel.publish(msg)
            self.rate100.sleep()

    def publish_arm_command(self, farray, ts=1):
        msg = Float64MultiArray()
        msg.data = farray
        self.get_logger().info(f'Publishing arm {self.control_interface}: {farray}')
        for _ in range(int(ts*100)):
            self.pub_arm_cmd.publish(msg)
            self.rate100.sleep()

    def sleep(self, ts=1):
        for _ in range(int(ts*10)):
            self.rate10.sleep()

    def quit(self):
        self.destroy_node()
        rclpy.shutdown()

    def run(self, ):
        if '(' in self.fn:
            eval(f'self.{self.fn}')
        else:
            eval(f'self.{self.fn}()')


    def wheeled_diffdrive(self):
        self.publish_cmd_vel(0.2,0.0,5)
        self.publish_cmd_vel(0.0,0.0,0.5)
        self.publish_cmd_vel(0.0,math.pi/8,4)
        self.publish_cmd_vel(0.0,0.0,0.5)


    def arm_position(self):
        self.publish_arm_command([-0.5,  0.5], 3)
        self.publish_arm_command([ 0.5, -0.5], 3)
        self.publish_arm_command([ 0, 0], 3)

    def arm_velocity(self):
        self.publish_arm_command([-0.5,  0.5], 3)
        self.publish_arm_command([ 0.5, -0.5], 3)
        self.publish_arm_command([ 0, 0], 3)

    def arm_effort(self):
        self.publish_arm_command([-0.05,  0.05], 3)
        self.publish_arm_command([ 0.05, -0.05], 3)
        self.publish_arm_command([ 0, 0], 3)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('fn', type=str, help='Test function to run')
    args = parser.parse_args()

    robot = TestControl(args)

    robot.run()

    robot.quit()


if __name__ == '__main__':
    main()

