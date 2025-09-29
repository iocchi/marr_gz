import time, threading, math, os
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

from controller_manager_msgs.srv import ListControllers

NODE_NAME = 'test_control'

class TestControl(Node):

    def __init__(self, args):
        
        rclpy.init()

        super().__init__(NODE_NAME)
        
        # autodetect controller and number of joints

        self.controller_name = None
        self.control_interface = None
        self.njoints = 0

        ctrls = self.list_controllers()

        self.fn = []
        
        if ctrls is not None and len(ctrls)>0:
            for (cname, cmdint) in ctrls:
                self.controller_name = cname
                vc = self.controller_name.split('_')
                if vc[0]=='arm':
                    self.control_interface = vc[1]
                    self.njoints = len(cmdint)
                    # test function to run
                    self.fn.append(vc[0]+"_"+self.control_interface)
                elif vc[0]=='ddrive':
                    # test function to run
                    self.fn.insert(0,vc[0])
                    self.njoints = 2   # ???
                elif vc[0] in ['pan', 'tilt']:
                    self.control_interface = vc[1]
                    # test function to run
                    self.fn.append(vc[0]+"_"+self.control_interface)


        print(f"Controller: {self.controller_name} - interface: {self.control_interface} - n. joints: {self.njoints}")

        if args.fn is not None:
            self.fn = args.fn

        print(f"Test: {self.fn}")

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

        rate10 = self.create_rate(10) # Hz
    
        rate10.sleep()

        # publishers

        self.pub_cmd_vel = self.create_publisher(TwistStamped, f'/ddrive_controller/cmd_vel', 10)
        self.pub_arm_cmd = self.create_publisher(Float64MultiArray, f'/arm_{self.control_interface}_controller/commands', 10)
        self.pub_pan_cmd = self.create_publisher(Float64MultiArray, f'/pan_{self.control_interface}_controller/commands', 10)
        self.pub_tilt_cmd = self.create_publisher(Float64MultiArray, f'/tilt_{self.control_interface}_controller/commands', 10)

        rate10.sleep()
 
        self.get_logger().info(f'{NODE_NAME} node initialized ')



    # detect controllers
    
    def list_controllers(self):

        # Create a temporary node to call the service
        service_caller_node = rclpy.create_node('controller_lister_node')

        # Create a client for the ListControllers service
        cli = service_caller_node.create_client(
            ListControllers, '/controller_manager/list_controllers')

        # Wait for the service to be available
        if not cli.wait_for_service(timeout_sec=5.0):
            service_caller_node.get_logger().error(
                'Service /controller_manager/list_controllers not available.')
            service_caller_node.destroy_node()
            return None

        # Create a request (it's empty for this service)
        req = ListControllers.Request()

        # Call the service asynchronously
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(service_caller_node, future)

        lc = []

        # Process the result
        if future.result() is not None:
            result = future.result()
            if not result.controller:
                service_caller_node.get_logger().info('No controllers found.')
            else:
                #service_caller_node.get_logger().info('Found controllers:')
                for controller in result.controller:
                #    service_caller_node.get_logger().info(
                #        f"  - Name: {controller.name}, State: {controller.state} - {controller.required_command_interfaces}")
                    if 'controller' in controller.name:
                        lc.append((controller.name,controller.
                            required_command_interfaces))
        else:
            service_caller_node.get_logger().error(
                f'Exception while calling service: {future.exception()}')

        # Clean up
        service_caller_node.destroy_node()
    
        return lc



    # Publish

    def publish_cmd_vel(self, lx, az, ts=1):
        rate100 = self.create_rate(100) # Hz
        msg = TwistStamped()
        msg.twist.linear.x = float(lx)
        msg.twist.angular.z = float(az)
        self.get_logger().info(f'Publishing cmd_vel: {lx:.3f} {az:.3f} time: {ts:.2f} s')
        for _ in range(int(ts*100)):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd_vel.publish(msg)
            rate100.sleep()

    def publish_arm_command(self, farray, ts=1):
        rate100 = self.create_rate(100) # Hz
        msg = Float64MultiArray()
        msg.data = farray
        self.get_logger().info(f'Publishing arm {self.control_interface}: {farray}')
        for _ in range(int(ts*100)):
            self.pub_arm_cmd.publish(msg)
            rate100.sleep()

    def publish_pan_command(self, v, ts=1):
        rate100 = self.create_rate(100) # Hz
        msg = Float64MultiArray()
        msg.data = [ float(v) ]
        self.get_logger().info(f'Publishing pan {self.control_interface}: {v}')
        for _ in range(int(ts*100)):
            self.pub_pan_cmd.publish(msg)
            rate100.sleep()

    def publish_tilt_command(self, v, ts=1):
        rate100 = self.create_rate(100) # Hz
        msg = Float64MultiArray()
        msg.data = [ float(v) ]
        self.get_logger().info(f'Publishing tilt {self.control_interface}: {v}')
        for _ in range(int(ts*100)):
            self.pub_tilt_cmd.publish(msg)
            rate100.sleep()


    def sleep(self, ts=1):
        rate10 = self.create_rate(10) # Hz
        for _ in range(int(ts*10)):
            self.rate10.sleep()

    def quit(self):
        self.destroy_node()
        rclpy.shutdown()

    def run(self):
    
        if type(self.fn) == list:
            for fn in self.fn:
                eval(f'self.{fn}()')
        else:
            if '(' in self.fn:
                eval(f'self.{self.fn}')
            else:
                eval(f'self.{self.fn}()')

    def ddrive(self):
        self.publish_cmd_vel(0.2,0.0,5)
        self.publish_cmd_vel(0.0,0.0,0.5)
        self.publish_cmd_vel(0.0,math.pi/8,4)
        self.publish_cmd_vel(0.0,0.0,0.5)
        self.publish_cmd_vel(-0.2,0.0,5)
        self.publish_cmd_vel(0.0,0.0,0.5)
        self.publish_cmd_vel(0.0,-math.pi/8,4)
        self.publish_cmd_vel(0.0,0.0,0.5)


    def arm_position(self, value=math.pi/4, tm=3):
        p = [0] * self.njoints
        self.publish_arm_command(p, tm)
        for i in range(self.njoints):
            p[i] = -math.pi/4
            self.publish_arm_command(p, tm)
        p = [0] * self.njoints
        self.publish_arm_command(p, tm)


    def arm_velocity(self, value=0.5):
        ts = 10
        p0 = [0] * self.njoints
        p1 = [0] * self.njoints
        p2 = [0] * self.njoints
        for i in range(self.njoints):
            p1[i] = value * (1 if i%2==0 else -1)
            p2[i] = -p1[i]
        self.publish_arm_command(p1, ts)
        self.publish_arm_command(p0, 1)  # vel 0
        self.publish_arm_command(p2, ts)
        self.publish_arm_command(p0, 1)  # vel 0

    def arm_effort(self, value=0.07):
        ts = 5
        p0 = [0] * self.njoints
        p1 = [0] * self.njoints
        p2 = [0] * self.njoints
        if self.njoints<=3:
            for i in range(self.njoints):            
                p1[i] = value * (1 if i%2==0 else -1)
                p2[i] = -p1[i]
        elif self.njoints==4:
            p1 = [-0.2,-0.8,-0.8,-0.3]
            p2 = [0.2,0.8,0.4,0.1]
        elif self.njoints==5:
            p1 = [-0.1,-0.8,-0.8,-0.7,-0.2]
            p2 = [0.1,0.8,0.8,1.2,0.2]
        elif self.njoints==6:
            p1 = [-0.1,-0.8,-0.8,-0.8,-0.7,-0.2]
            p2 = [0.1,0.8,0.8,0.8,1.2,0.2]

        self.publish_arm_command(p1, ts)
        self.publish_arm_command(p0, 1)  # eff 0
        self.publish_arm_command(p2, ts)
        self.publish_arm_command(p0, 1)  # eff 0



    def pan_position(self):
        self.publish_pan_command(-math.pi/2, 2)
        self.publish_pan_command(math.pi/2, 4)
        self.publish_pan_command(0, 2)
        
    def tilt_position(self):
        self.publish_tilt_command(-math.pi/4, 1)
        self.publish_tilt_command(math.pi/4, 2)
        self.publish_tilt_command(0, 1)

    def pan_velocity(self):
        self.publish_pan_command(-math.pi/4, 2)
        self.publish_pan_command(math.pi/4, 4)
        self.publish_pan_command(-math.pi/4, 2)
        self.publish_pan_command(0, 1)
        
    def tilt_velocity(self):
        self.publish_tilt_command(-math.pi/4, 2)
        self.publish_tilt_command(math.pi/4, 4)
        self.publish_tilt_command(-math.pi/4, 2)
        self.publish_tilt_command(0, 1)
        
    def pan_effort(self):
        self.publish_pan_command(-math.pi/10, 2)
        self.publish_pan_command(math.pi/10, 4)
        self.publish_pan_command(-math.pi/10, 2)
        self.publish_pan_command(0, 1)
        
    def tilt_effort(self):
        self.publish_tilt_command(-math.pi/10, 2)
        self.publish_tilt_command(math.pi/10, 4)
        self.publish_tilt_command(-math.pi/10, 2)
        self.publish_tilt_command(0, 1)

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('-fn', type=str, default=None, help='Test function to run')
    args = parser.parse_args()

    robot = TestControl(args)

    robot.run()

    robot.quit()


if __name__ == '__main__':
    main()

