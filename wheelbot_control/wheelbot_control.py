import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

import odrive
from odrive.enums import *
import time
import math

class botwheel_controller(Node):

    UPDATE_RATE = 10

    CALIBON = False
    TREAD = 0.480
    PULSE_PER_ROTATE = 3200
    WHEEL_DIA = 0.171 

    MOVE_PER_PULSE = WHEEL_DIA / PULSE_PER_ROTATE
    CIRCUMFERENCE = 2 * math.pi * WHEEL_DIA 

    def __init__(self,**args):    
        super().__init__('botwheel_controller_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # self.publisher = self.create_publisher(Odometry, 'odom', 10)

        #ODrive setup start
        print("Connecting to ODrive...")
        self.odrive_left = odrive.find_any(serial_number = "394F35773231")#decimal "63012362203697"
        self.odrive_right = odrive.find_any(serial_number = "393535663231")#decimal "62900691939889"
        print("Connected to ODrive.")
    
        if(self.CALIBON):
            self.get_logger().info("Starting calibration...")
            self.odrive_left.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.odrive_right.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while self.odrive_left.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            while self.odrive_right.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            self.get_logger().info("Calibration complete.")

        self.odrive_left.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrive_right.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.position_x = 0.0
        self.position_y = 0.0
        self.angular_z = 0.0

        # self.create_timer(1.0 / self.UPDATE_RATE, self.publish_odometry)

    def cmd_vel_callback(self,msg):
        linear_x = msg.linear.x
        angular_z = -msg.angular.z
        
        ref_left_wheel_speed = linear_x - (self.TREAD/2) * angular_z
        ref_right_wheel_speed = linear_x + (self.TREAD/2) * angular_z

        ref_left_wheel_speed_rpm = int((ref_left_wheel_speed/self.CIRCUMFERENCE) * 60)
        ref_right_wheel_speed_rpm = int((ref_right_wheel_speed/self.CIRCUMFERENCE) * 60)

        #For debug
        print("left : %3d[rpm] %lf[m/s]\tright : %3d[rpm] %lf[m/s]" % (ref_left_wheel_speed_rpm, ref_left_wheel_speed, ref_right_wheel_speed_rpm, ref_right_wheel_speed))
        
        self.odrive_left.axis0.controller.input_vel=ref_left_wheel_speed
        self.odrive_right.axis0.controller.input_vel=-ref_right_wheel_speed

    # def publish_odometry(self):
    #     left_enc = self.odrive_left.axis0.pos_estimate
    #     right_enc = self.odrive_right.axis0.pos_estimate

    #     left_distance = left_enc * self.MOVE_PER_PULSE
    #     right_distance = right_enc * self.MOVE_PER_PULSE

    #     distance = (left_distance + right_distance) / 2.0
    #     theta = (right_distance - left_distance) / self.TREAD

    #     self.position_x += distance * math.cos(self.angular_z)
    #     self.position_y += distance * math.sin(self.angular_z)
    #     self.angular_z += theta

    #     quaternion = quaternion_from_euler(0, 0, self.angular_z)

    #     odom_msg = Odometry()
    #     odom_msg.header.stamp = self.get_clock().now().to_msg()
    #     odom_msg.header.frame_id = 'odom'
    #     odom_msg.child_frame_id = 'base_link'

    #     odom_msg.pose.pose.position.x = self.position_x
    #     odom_msg.pose.pose.position.y = self.position_y
    #     odom_msg.pose.pose.position.z = 0.0
    #     odom_msg.pose.pose.orientation.x = quaternion[0]
    #     odom_msg.pose.pose.orientation.y = quaternion[1]
    #     odom_msg.pose.pose.orientation.z = quaternion[2]
    #     odom_msg.pose.pose.orientation.w = quaternion[3]

    #     odom_msg.twist.twist.linear.x = (left_distance + right_distance) / (2.0 * self.UPDATE_RATE)
    #     odom_msg.twist.twist.angular.z = theta * self.UPDATE_RATE

    #     self.publisher.publish(odom_msg)


def main():
    try:
        rclpy.init()
        node = botwheel_controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print("Releasing ODrive connection...")
    #todo : Add disconnect to odirve funciton
    time.sleep(0.1)
    print("ODrive connection closed.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


