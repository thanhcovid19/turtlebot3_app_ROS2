import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped

class SubscriberClass(Node):

    def __init__(self):
        super().__init__('sub_node')
        rclpy.logging.set_logger_level('sub_node', rclpy.logging.LoggingSeverity.DEBUG)
        
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = MutuallyExclusiveCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.group1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group2)
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10, callback_group=self.group4)

        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.group3)
        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.vel_msg = Twist()
        self.initialpose_msg = PoseWithCovarianceStamped()
        self.publisher = PublisherClass()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # self.publisher = PublisherClass()

        self.first_call = True
        self.first_call_clockwise = True

        self.rotate_part1_completed = False
        self.rotate_part1_completed_clockwise = False
        self.rotating = False
        self.rotating_clockwise = False

        self.rotate_completed = False
        self.rotate_completed_clockwise = False

        self.move_straight_flag = False
        self.move_backward_flag = False
        # self.moving_backward = False
        self.rotate_clk_safe = False
        self.rotate_counter_clk_safe = False
        self.move_zigzag_completed = False

        self.rotation = 0.0
        self.rotation1 = 999.0
        self.rotation2 = 999.0

        self.rotation_clockwise = 0.0
        self.rotation1_clockwise = 999.0
        self.rotation2_clockwise = 999.0

        self.first_read = True

        self.count_rotate_clk_time = 0
        self.count_rotate_counter_clk_time = 0
        self.move_straight_1s_done = False
        self.move_straight_period_done = False

        self.count_time = 0
        self.move_straight_period = 3

        self.initial_pose_is_set = False

    def initialpose_callback(self, msg):
        self.initialpose_msg = msg
        # self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.point.x, msg.point.y, msg.point.z))
        # self.get_logger().info(f'X : {self.clicked_point_msg.x}, Y : {self.clicked_point_msg.y}, Z : {self.clicked_point_msg.z}')
        self.get_logger().info(f'initialpose callback')
        self.get_logger().info(f'initialpose_msg: {self.initialpose_msg}')
        if self.initialpose_msg.pose.pose.position.x != -999:
            self.initial_pose_is_set = True

    def odom_callback(self, msg):
        # self.get_logger().debug("Odom CallBack")
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion (orientation_list)

    def scan_callback(self, msg):
        # self.get_logger().debug("Scan CallBack")
        self.laser_msg = msg

    def get_front_laser(self):
        return self.laser_msg.ranges[360]

    def get_yaw(self):
        return self.yaw

    def stop_robot(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.publisher_.publish(self.vel_msg)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        """"
        reference from: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        """

        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def move_zigzag(self):
        # self.get_logger().info('In move_zigzag function')
        if self.move_zigzag_completed == False:
            front_distance = self.laser_msg.ranges[359]
            left_wheel_distance = self.laser_msg.ranges[22]
            right_wheel_distance = self.laser_msg.ranges[332]

            if (front_distance > 0.25 or front_distance == 0) and \
                    (left_wheel_distance > 0.25 or left_wheel_distance == 0) and \
                    (right_wheel_distance > 0.25 or right_wheel_distance == 0):
                        self.move_straight_flag = True
            else:
                self.move_straight_flag = False

            if self.move_backward_flag == True:
                self.move_straight_flag = False

            if self.move_straight_flag == True:
                if self.rotating == True:
                    self.turn_counter_clockwise(90)
                elif self.rotating_clockwise == True:
                    self.turn_clockwise(90)
                elif (self.count_rotate_clk_time == 1 and self.move_straight_period_done == False) or \
                        (self.count_rotate_counter_clk_time == 1 and self.move_straight_period_done == False):
                    # self.publisher.move_straight_3s()
                    if self.count_time < self.move_straight_period and self.move_straight_flag == True:
                        if self.publisher.moving_straight_1s == False:
                            self.publisher.move_straight_1s()
                            self.get_logger().info(f'front distance = {front_distance}')
                            self.count_time += 1
                    else:
                        self.count_time = 0
                        self.move_straight_period_done = True
                        self.publisher.stop_robot()
                    # if self.publisher.moving_straight_1s == False:
                    #     self.publisher.move_straight_1s()
                    #     self.move_stright_1s_done = True

                elif self.move_straight_period_done == True and (self.count_rotate_clk_time == 1 or self.count_rotate_counter_clk_time == 1):
                    if self.count_rotate_clk_time == 1:
                        self.rotate_completed_clockwise = False
                        self.turn_clockwise(90)
                    elif self.count_rotate_counter_clk_time == 1:
                        self.rotate_completed = False
                        self.turn_counter_clockwise(90)
                else:
                    self.publisher.move_straight()
                    # if self.count_rotate_clk_time > 1:
                    #     self.count_rotate_clk_time = 0
                    # if self.count_rotate_counter_clk_time > 1:
                    #     self.count_rotate_counter_clk_time = 0
            else:
                # self.get_logger().info('move_straight_flag = False')
                if self.rotating_clockwise == True:
                    self.turn_clockwise(90)
                elif self.rotating == True:
                    self.turn_counter_clockwise(90)
                else:
                    if self.move_backward_flag == False:
                        self.publisher.stop_robot()

                    self.get_logger().info(f'count_rotate_counter_clk_time: {self.count_rotate_counter_clk_time}')
                    self.get_logger().info(f'count_rotate_clk_time: {self.count_rotate_clk_time}')

                    self.move_straight_period_done = False
                    counter_clk_90_degrees_distance = self.laser_msg.ranges[79]
                    clk_90_degrees_distance = self.laser_msg.ranges[269]
                    rear_distance = self.laser_msg.ranges[179]
                    left_rear_distance = self.laser_msg.ranges[151]
                    right_rear_distance = self.laser_msg.ranges[199]

                    if (rear_distance > 0.25 or rear_distance == 0.0) and \
                            (left_rear_distance > 0.25 or left_rear_distance == 0.0)  and \
                                (right_rear_distance > 0.25 or right_rear_distance == 0.0):
                                self.move_backward_flag = True
                    # else:
                    #     self.moving_backward = False

                    if counter_clk_90_degrees_distance > 0.25 or counter_clk_90_degrees_distance == 0.0:
                        self.rotate_counter_clk_safe = True
                    else:
                        self.rotate_counter_clk_safe = False

                    if clk_90_degrees_distance > 0.25 or clk_90_degrees_distance == 0.0:
                        self.rotate_clk_safe = True
                    else:
                        self.rotate_clk_safe = False

                    if self.count_rotate_counter_clk_time == 0 and self.count_rotate_clk_time == 0:

                        if self.rotate_counter_clk_safe == True:
                            # if self.moving_backward == True:
                            #     self.publisher.move_back_1s()
                            
                            if self.publisher.moving_backward == True:
                                self.publisher.move_back_1s()
                            else:
                                self.move_backward_flag = False
                                self.publisher.stop_robot()
                                self.rotate_completed = False
                                self.turn_counter_clockwise(90)

                            
                            # if self.publisher.moving_backward_1s == False:
                            #     self.moving_backward = False
                            #     self.publisher.stop_robot()
                            #     self.rotate_completed = False
                            #     self.turn_counter_clockwise(90)
                        elif self.rotate_clk_safe:
                            if self.publisher.moving_backward == True:
                                self.publisher.move_back_1s()
                            else:
                                self.move_backward_flag = False
                                self.publisher.stop_robot()
                                self.rotate_completed_clockwise = False
                                self.turn_clockwise(90)
                            # if self.publisher.moving_backward_1s == False:
                            #     self.moving_backward = False
                            #     self.publisher.stop_robot()
                            #     self.rotate_completed_clockwise = False
                            #     self.turn_clockwise(90)
                        else:
                            if self.move_backward_flag == True:
                                self.publisher.move_backward()
                            else:
                                # self.moving_backward = False
                                self.publisher.stop_robot()
                                self.move_zigzag_completed = True

                    elif self.count_rotate_clk_time == 2:
                        # self.move_stright_3s_done = True
                        # counter_clk_90_degrees_distance
                        # self.rotate_completed = False
                        # self.turn_counter_clockwise(90)
                        if self.rotate_counter_clk_safe == True:
                            if self.publisher.moving_backward == True:
                                self.publisher.move_back_1s()
                            else:
                                self.move_backward_flag = False
                                self.count_rotate_clk_time = 0
                                self.publisher.stop_robot()
                                self.rotate_completed = False
                                self.turn_counter_clockwise(90)

                            # if self.publisher.moving_backward_1s == False:
                            #     self.count_rotate_clk_time = 0
                            #     self.moving_backward = False
                            #     self.publisher.stop_robot()
                            #     self.rotate_completed = False
                            #     self.turn_counter_clockwise(90)
                        else:
                            if self.move_backward_flag == True:
                                self.publisher.move_backward()
                            else:
                                self.publisher.stop_robot()
                                self.move_zigzag_completed = True
                                # self.rotate_completed_clockwise = False
                                # self.turn_clockwise(90)
                    elif self.count_rotate_counter_clk_time == 2:
                        # self.get_logger().info('HERE')
                        # self.move_stright_3s_done = True
                        # clk_90_degrees_distance
                        # self.rotate_completed_clockwise = False
                        # self.turn_clockwise(90)
                        if self.rotate_clk_safe == True:
                            if self.publisher.moving_backward == True:
                                self.publisher.move_back_1s()
                            else:
                                self.move_backward_flag = False
                                self.count_rotate_counter_clk_time = 0
                                self.publisher.stop_robot()
                                self.rotate_completed_clockwise = False
                                self.turn_clockwise(90)

                            # if self.publisher.moving_backward_1s == False:
                            #     self.count_rotate_counter_clk_time = 0
                            #     self.moving_backward = False
                            #     self.publisher.stop_robot()
                            #     self.rotate_completed_clockwise = False
                            #     self.turn_clockwise(90)
                        else:
                            if self.move_backward_flag == True:
                                self.publisher.move_backward()
                            else:
                                # if self.rotate_counter_clk_safe == True:
                                #     self.count_rotate_counter_clk_time = 0
                                #     self.publisher.stop_robot()
                                #     self.rotate_completed = False
                                #     self.turn_counter_clockwise(90)
                                # else:
                                self.publisher.stop_robot()
                                self.move_zigzag_completed = True
                    # elif self.count_rotate_clk_time == 1 or self.count_rotate_counter_clk_time == 1:
                    #     if self.count_rotate_clk_time == 1:
                    #         self.rotate_completed_clockwise = False
                    #         self.turn_clockwise(90)
                    #     elif self.count_rotate_counter_clk_time == 1:
                    #         self.rotate_completed = False
                    #         self.turn_counter_clockwise(90)
                    elif self.count_rotate_clk_time == 1:
                        # self.rotate_completed_clockwise = False
                        # self.turn_clockwise(90)
                        if self.rotate_clk_safe == True:
                            if self.publisher.moving_backward == True:
                                self.publisher.move_back_1s()
                            else:
                                self.move_backward_flag = False
                                self.publisher.stop_robot()
                                self.rotate_completed_clockwise = False
                                self.turn_clockwise(90)

                            
                            # if self.publisher.moving_backward_1s == False:
                            #     self.moving_backward = False
                            #     self.publisher.stop_robot()
                            #     self.rotate_completed_clockwise = False
                            #     self.turn_clockwise(90)
                        else:
                            if self.move_backward_flag == True:
                                self.publisher.move_backward()

                    elif self.count_rotate_counter_clk_time == 1:
                        # self.rotate_completed = False
                        # self.turn_counter_clockwise(90)

                        # if self.rotate_clk_safe == True:
                        #     if self.publisher.moving_backward == True:
                        #         self.publisher.move_back_1s()
                        #     else:
                        #         self.move_backward_flag = False
                        #         self.publisher.stop_robot()
                        #         self.rotate_completed_clockwise = False
                        #         self.turn_clockwise(90)

                            
                        #     # if self.publisher.moving_backward_1s == False:
                        #     #     self.moving_backward = False
                        #     #     self.publisher.stop_robot()
                        #     #     self.rotate_completed_clockwise = False
                        #     #     self.turn_clockwise(90)
                        # else:
                        #     if self.move_backward_flag == True:
                        #         self.publisher.move_backward()



                        if self.rotate_counter_clk_safe == True:
                            if self.publisher.moving_backward == True:
                                self.publisher.move_back_1s()
                            else:
                                self.move_backward_flag = False
                                self.publisher.stop_robot()
                                self.rotate_completed = False
                                self.turn_counter_clockwise(90)

                            # if self.publisher.moving_backward_1s == False:
                            #     self.moving_backward = False
                            #     self.publisher.stop_robot()
                            #     self.rotate_completed = False
                            #     self.turn_counter_clockwise(90)
                        else:
                            if self.move_backward == True:
                                self.publisher.move_backward()

    def turn_counter_clockwise(self, target_angle):
        self.rotating = True

        while self.first_call == True:
            if self.yaw < 0:
                if abs(self.yaw) > 0.01:
                    self.rotation = target_angle*(np.pi)/180.0 + self.yaw
                    self.get_logger().info(f'rotation: {self.rotation}')
                    if abs(self.rotation + (np.pi)) > 0.02 and self.rotation > 0.0:
                        self.rotation1 = self.yaw
                        self.rotation2 = target_angle*(np.pi)/180.0 + self.rotation1
                    self.first_call = False
                else:
                    self.publisher.rotate_counter_clockwise_small_angle()
            else:
                if abs(np.pi - self.yaw) > 0.01:
                    self.rotation = target_angle*(np.pi)/180.0 + self.yaw
                    if abs(self.rotation - (np.pi)) > 0.02 and self.rotation > 3.0:
                        self.rotation1 = (np.pi) - self.yaw
                        self.rotation2 = (np.pi) - (target_angle*np.pi/180.0 - self.rotation1)
                    self.first_call = False
                else:
                    self.publisher.rotate_counter_clockwise_small_angle()

        while True:

            if self.rotation1 != 999.0 and self.rotation1 < 0:
                if self.rotate_part1_completed == False:
                    error_angle = self.yaw
                else:
                    while self.yaw < 0:
                        self.publisher.rotate_counter_clockwise_small_angle()

                    if self.yaw > 0:
                        error_angle = self.rotation2 - self.yaw

            elif self.rotation1 != 999.0 and self.rotation1 > 0:
                if self.rotate_part1_completed == False:
                    error_angle = np.pi - self.yaw
                else:
                    while self.yaw > 0:
                        self.publisher.rotate_counter_clockwise_small_angle()

                    if self.yaw < 0:
                        error_angle = self.rotation2 + self.yaw
            else:
                # compare target angle with actual angle (yaw) of the robot
                error_angle = self.rotation - self.yaw
                self.rotate_part1_completed = True
            self.get_logger().info(f'yaw = {self.yaw}')
            self.get_logger().info(f'error_angle: {error_angle}')
            # self.get_logger().info(f'rotation: {self.rotation}')
            # self.get_logger().info(f'rotation1: {self.rotation1}')
            # self.get_logger().info(f'rotation2: {self.rotation2}')

            if self.rotate_completed == False:
                if abs(error_angle) > 0.015:
                    # command_angle = 0.12*abs(error_angle)
                    if abs(error_angle) > 0.03:
                        if abs(error_angle) < np.pi:
                            if abs(error_angle) > 2.5:
                                command_angle = 0.1*abs(error_angle)
                            elif abs(error_angle) > 1.5 and abs(error_angle) <= 2.5:
                                command_angle = 0.2*abs(error_angle)
                            else:
                                command_angle = 0.5*abs(error_angle)
                        # else:
                        #     self.stop_robot()
                    else:
                        command_angle = 0.01
                    command_vel = 0.0
                    # self.publish_vel(command_vel, command_angle)
                    self.publisher.publish_vel(command_vel, command_angle)
                    time.sleep(0.1)
                else:
                    if self.rotate_part1_completed == False:
                        self.rotate_part1_completed = True
                        self.get_logger().info('ROTATE PARTIALLY COMPLETED')
                    else:
                        self.get_logger().info('ROTATE COMPLETED')
                        self.rotating = False
                        self.rotate_completed = True
                        self.rotation = 0.0
                        self.rotation1 = 999.0
                        self.rotation2 = 999.0
                        self.rotate_part1_completed = False
                        self.first_call = True
                        # self.stop_robot()
                        self.count_rotate_counter_clk_time += 1
                        self.publisher.stop_robot()
                        break

        # while True:
        #     # self.get_logger().info(f'rotate in {i} second')
        #     if self.rotate_part1_completed == False:
        #         if self.rotation1 != 999.0 and self.rotation1 < 0:
        #             error_angle = self.yaw
        #         elif self.rotation1 != 999.0 and self.rotation1 > 0:
        #             error_angle = np.pi - self.yaw
        #         else:
        #             error_angle = self.rotation - self.yaw
        #             self.rotate_part1_completed = True

        #     self.get_logger().info(f'error_angle: {error_angle}')
        #     if abs(error_angle) > 0.005:
        #         command_angle = 0.12*abs(error_angle)
        #         command_vel = 0.0
        #         self.publisher.publish_vel(command_vel, command_angle)
        #         time.sleep(0.5)
        #     else:
        #         if self.rotate_part1_completed == False:
        #             self.get_logger().info('ROTATE PARTIALLY COMPLETED')
        #             if self.rotation1 < 0:
        #                 error_angle = self.rotation2 - self.yaw
        #             else:
        #                 error_angle = self.rotate + self.yaw
        #             self.rotate_part1_completed = True
        #         else:
        #             self.get_logger().info('ROTATE COMPLETED')
        #             self.publisher.stop_robot()
        #             self.rotating = False
        #             self.roatate_completed = True
        #             break

    def turn_clockwise(self,target_angle):
        self.rotating_clockwise = True

        while self.first_call_clockwise == True:
            self.get_logger().info(f'yaw: {self.yaw}')
            if self.yaw <= 0:
                if abs(self.yaw + (np.pi)) > 0.01:
                    self.rotation_clockwise = -target_angle*(np.pi)/180.0 + self.yaw
                    if abs(self.rotation_clockwise + np.pi) > 0.02 and abs(self.rotation_clockwise) > np.pi:
                        self.rotation1_clockwise = -self.yaw - (np.pi)
                        # self.get_logger().info(f'rotation1_clockwise: {self.rotation1_clockwise}')
                        self.rotation2_clockwise = (np.pi) - (target_angle*np.pi/180.0 + self.rotation1_clockwise)
                        # self.get_logger().info(f'rotation2_clockwise: {self.rotation2_clockwise}')

                    self.first_call_clockwise = False
                    # self.publisher.stop_robot()
                else:
                    self.publisher.rotate_clockwise_small_angle()
            else:
                if abs(self.yaw) > 0.01:
                    self.rotation_clockwise = self.yaw - target_angle*(np.pi)/180.0
                    if abs(self.rotation_clockwise) > 0.02 and self.rotation_clockwise < 0.0:
                        self.rotation1_clockwise = self.yaw
                        self.rotation2_clockwise = - (target_angle*np.pi/180.0 - self.rotation1_clockwise)
                        self.get_logger().info('yaw greater than 0.02')
                        self.get_logger().info(f'rotation_clockwise: {self.rotation_clockwise}')
                        self.get_logger().info(f'rotation1_clockwise: {self.rotation1_clockwise}')
                        self.get_logger().info(f'rotation2_clockwise: {self.rotation2_clockwise}')
                        
                    self.first_call_clockwise = False
                    self.publisher.stop_robot()
                else:
                    self.get_logger().info('rotating clk small angle')
                    self.publisher.rotate_clockwise_small_angle()

        while True:

            if self.rotation1_clockwise != 999.0 and self.rotation1_clockwise < 0:
                if self.rotate_part1_completed_clockwise == False:
                    error_angle = self.yaw + (np.pi)
                else:
                    while self.yaw < 0:
                        self.publisher.rotate_clockwise_small_angle()

                    if self.yaw > 0:
                        error_angle = -self.rotation2_clockwise + self.yaw

            elif self.rotation1_clockwise != 999.0 and self.rotation1_clockwise > 0:
                if self.rotate_part1_completed_clockwise == False:
                    error_angle = self.yaw
                else:
                    while self.yaw > 0:
                        self.publisher.rotate_clockwise_small_angle()

                    if self.yaw <= 0:
                        error_angle = self.rotation2_clockwise - self.yaw
            else:
                # compare target angle with actual angle (yaw) of the robot
                error_angle = self.rotation_clockwise - self.yaw
                self.rotate_part1_completed_clockwise = True
            self.get_logger().info(f'yaw = {self.yaw}')
            self.get_logger().info(f'error_angle: {error_angle}')
            self.get_logger().info(f'rotation_clockwise: {self.rotation_clockwise}')
            self.get_logger().info(f'rotation1_clockwise: {self.rotation1_clockwise}')
            self.get_logger().info(f'rotation2_clockwise: {self.rotation2_clockwise}')
            # break 

            if self.rotate_completed_clockwise == False:
                if abs(error_angle) > 0.015:
                    # command_angle = 0.12*abs(error_angle)
                    if abs(error_angle) > 0.03:
                        if abs(error_angle) < np.pi:
                            if abs(error_angle) > 2.5:
                                command_angle = -0.1*abs(error_angle)
                            elif abs(error_angle) > 1.5 and abs(error_angle) <= 2.5:
                                command_angle = -0.2*abs(error_angle)
                            else:
                                command_angle = -0.5*abs(error_angle)
                        # else:
                        #     self.stop_robot()
                    else:
                        command_angle = -0.01
                    command_vel = 0.0
                    self.publisher.publish_vel(command_vel, command_angle)
                    # self.get_logger().info('publish angle velocity successful')
                    time.sleep(0.1)
                    # if self.rotate_part1_completed_clockwise == True:
                    #     self.get_logger().info('rotate part 2 here')
                    #     self.get_logger().info(f'yaw = {self.yaw}')
                    #     self.get_logger().info(f'error_angle: {error_angle}')
                    #     self.get_logger().info(f'rotation_clockwise: {self.rotation_clockwise}')
                    #     self.get_logger().info(f'rotation1_clockwise: {self.rotation1_clockwise}')
                    #     self.get_logger().info(f'rotation2_clockwise: {self.rotation2_clockwise}')
                    #     break
            # break 
                else:
                    if self.rotate_part1_completed_clockwise == False:
                        self.rotate_part1_completed_clockwise = True
                        self.get_logger().info('ROTATE PARTIALLY COMPLETED')
                        # break
                    else:
                        self.get_logger().info('ROTATE COMPLETED')
                        self.rotating_clockwise = False
                        self.rotate_completed_clockwise = True
                        self.rotation = 0.0
                        self.rotation1_clockwise = 999.0
                        self.rotation2_clockwise = 999.0
                        self.rotate_part1_completed_clockwise = False
                        self.first_call_clockwise = True
                        # self.count_rotate_counter_clk_time += 1
                        # if self.count_rotate_clk_time > 1:
                        #     self.count_rotate_clk_time = 0
                        self.count_rotate_clk_time += 1
                        self.publisher.stop_robot()
                        break

        # if self.first_call_clockwise == True:
        #     if self.yaw <= 0:
        #         if abs(self.yaw + (np.pi)) > 0.02:
        #             self.rotation_clockwise = -target_angle*(np.pi)/180.0 + self.yaw
        #             if abs(self.rotation_clockwise + np.pi) > 0.02 and abs(self.rotation_clockwise) > np.pi:
        #                 self.rotation1_clockwise = -self.yaw - (np.pi)
        #                 # self.get_logger().info(f'rotation1_clockwise: {self.rotation1_clockwise}')
        #                 self.rotation2_clockwise = (np.pi) - (target_angle*np.pi/180.0 + self.rotation1_clockwise)
        #                 # self.get_logger().info(f'rotation2_clockwise: {self.rotation2_clockwise}')

        #             self.first_call_clockwise = False
        #             # self.publisher.stop_robot()
        #         else:
        #             self.publisher.rotate_clockwise_small_angle()
        #     else:
        #         if abs(self.yaw) > 0.02:
        #             self.rotation_clockwise = self.yaw - target_angle*(np.pi)/180.0
        #             if abs(self.rotation_clockwise) > 0.02 and self.rotation_clockwise < 0.0:
        #                 self.rotation1_clockwise = self.yaw
        #                 self.rotation2_clockwise = - (target_angle*np.pi/180.0 - self.rotation1_clockwise)

        #             self.first_call_clockwise = False
        #             self.publisher.stop_robot()
        #         else:
        #             self.publisher.rotate_clockwise_small_angle()

        # if self.first_call_clockwise == False:
        #     if self.rotation1_clockwise != 999.0 and self.rotation1_clockwise < 0:
        #         if self.rotate_part1_completed_clockwise == False:
        #             error_angle = self.yaw + (np.pi)
        #         else:
        #             if self.yaw > 0:
        #                 error_angle = -self.rotation2_clockwise + self.yaw

        #     elif self.rotation1_clockwise != 999.0 and self.rotation1_clockwise > 0:
        #         if self.rotate_part1_completed_clockwise == False:
        #             error_angle = self.yaw
        #         else:
        #             if self.yaw <= 0:
        #                 error_angle = self.rotation2_clockwise - self.yaw
        #     else:
        #         # compare target angle with actual angle (yaw) of the robot
        #         error_angle = self.rotation_clockwise - self.yaw
        #         self.rotate_part1_completed_clockwise = True
        #     self.get_logger().info(f'yaw = {self.yaw}')
        #     self.get_logger().info(f'error_angle: {error_angle}')
        #     # self.get_logger().info(f'rotation: {self.rotation}')
        #     # self.get_logger().info(f'rotation1: {self.rotation1}')
        #     # self.get_logger().info(f'rotation2: {self.rotation2}')

        #     if self.rotate_completed_clockwise == False:
        #         if abs(error_angle) > 0.015:
        #             # command_angle = 0.12*abs(error_angle)
        #             if abs(error_angle) > 0.03:
        #                 if abs(error_angle) < np.pi:
        #                     if abs(error_angle) > 2.5:
        #                         command_angle = -0.1*abs(error_angle)
        #                     elif abs(error_angle) > 1.5 and abs(error_angle) <= 2.5:
        #                         command_angle = -0.2*abs(error_angle)
        #                     else:
        #                         command_angle = -0.5*abs(error_angle)
        #                 # else:
        #                 #     self.stop_robot()
        #             else:
        #                 command_angle = -0.01
        #             command_vel = 0.0
        #             self.publisher.publish_vel(command_vel, command_angle)
        #             self.get_logger().info('publish angle velocity successful')
        #         else:
        #             if self.rotate_part1_completed_clockwise == False:
        #                 self.rotate_part1_completed_clockwise = True
        #                 self.get_logger().info('ROTATE PARTIALLY COMPLETED')
        #             else:
        #                 self.get_logger().info('ROTATE COMPLETED')
        #                 self.rotating_clockwise = False
        #                 self.rotate_completed_clockwise = True
        #                 self.rotation = 0.0
        #                 self.rotation1_clockwise = 999.0
        #                 self.rotation2_clockwise = 999.0
        #                 self.rotate_part1_completed_clockwise = False
        #                 self.first_call_clockwise = True
        #                 self.count_rotate_counter_clk_time += 1
        #                 # if self.count_rotate_clk_time > 1:
        #                 #     self.count_rotate_clk_time = 0
        #                 self.publisher.stop_robot()

    def move_straight_and_turn_clockwise(self):
        front_distance = self.laser_msg.ranges[359]
        left_wheel_distance = self.laser_msg.ranges[22]
        right_wheel_distance = self.laser_msg.ranges[330]

        if (front_distance > 0.25 or front_distance == 0) and \
                (left_wheel_distance > 0.25 or left_wheel_distance == 0) and \
                (right_wheel_distance > 0.25 or right_wheel_distance == 0):
                    self.move_straight_flag = True
        else:
            self.move_straight_flag = False

        if self.move_straight_flag == True:
            if self.rotating_clockwise == False:  
                self.get_logger().info(f'front_distance: {front_distance}')
                self.get_logger().info(f'left_wheel_distance: {left_wheel_distance}')
                self.get_logger().info(f'right_wheel_distance: {right_wheel_distance}')
                # self.move_straight()     
                self.publisher.move_straight()       
            else:
                if self.rotate_completed_clockwise == False:
                    self.turn_clockwise(90)
        else:  
            if self.rotating_clockwise == False: 
                self.publisher.stop_robot()
                self.rotate_completed_clockwise = False 
            if self.rotate_completed_clockwise == False:   
                self.turn_clockwise(90)

    def move_straight_and_turn_counter_clockwise(self):
        front_distance = self.laser_msg.ranges[359]
        left_wheel_distance = self.laser_msg.ranges[22]
        right_wheel_distance = self.laser_msg.ranges[330]

        if (front_distance > 0.25 or front_distance == 0) and \
                (left_wheel_distance > 0.25 or left_wheel_distance == 0) and \
                (right_wheel_distance > 0.25 or right_wheel_distance == 0):
                    self.move_straight_flag = True
        else:
            self.move_straight_flag = False

        if self.move_straight_flag == True:
            if self.rotating == False:  
                self.get_logger().info(f'front_distance: {front_distance}')
                self.get_logger().info(f'left_wheel_distance: {left_wheel_distance}')
                self.get_logger().info(f'right_wheel_distance: {right_wheel_distance}')
                # self.move_straight()     
                self.publisher.move_straight()       
            else:
                if self.rotate_completed == False:
                    self.turn_counter_clockwise(90)
        else:  
            if self.rotating == False: 
                self.publisher.stop_robot()
                self.rotate_completed = False 
            if self.rotate_completed == False:   
                self.turn_counter_clockwise(90)

    def timer_callback(self):
        self.get_logger().info("Timer CallBack")
        try:

            # self.move_zigzag()

            # self.move_straight_and_turn_clockwise()
            if self.initial_pose_is_set == True:
                self.move_zigzag()
            # # if self.publisher.move_stright_3s_done == False:
            # #     self.publisher.move_straight_3s()

            # for i in range(360):
            #     self.get_logger().info(f'distance at range {i} : {self.laser_msg._ranges[i]}')

            # front_distance = self.laser_msg.ranges[359]
            # left_wheel_distance = self.laser_msg.ranges[22]
            # right_wheel_distance = self.laser_msg.ranges[332]

            # self.get_logger().info(f'front_distance: {front_distance}')
            # self.get_logger().info(f'left_wheel_distance: {left_wheel_distance}')
            # self.get_logger().info(f'right_wheel_distance: {right_wheel_distance}')

        except:
            pass

class PublisherClass(Node):

    def __init__(self, seconds_sleeping=10):
        super().__init__('pub_node')
        self._seconds_sleeping = seconds_sleeping
        rclpy.logging.set_logger_level('pub_node', rclpy.logging.LoggingSeverity.DEBUG)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()

        self.moving_straight_1s = False
        self.moving_backward_1s = False
        self.moving_backward = False

        # self.move_stright_3s_done = False
    
    def stop_robot(self):
        self.get_logger().info("MOVE STOP")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def move_straight(self):
        self.get_logger().info("MOVE STRAIGHT")
        self.cmd.linear.x = 0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def move_backward(self):
        self.get_logger().info("MOVE BACK")
        self.cmd.linear.x = -0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
        self.moving_backward = True

    def publish_vel(self,vx,wz):
        self.cmd.linear.x = vx
        self.cmd.angular.z = wz
        self.vel_pub.publish(self.cmd)

    def move_back_1s(self):
        self.get_logger().info('in move_back_1s function')
        self.moving_backward_1s = True
        self.cmd.linear.x = -0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
        time.sleep(1.0)
        self.moving_backward_1s = False
        self.moving_backward = False

    def move_straight_1s(self):
        # for i in range(3):
        #     self.get_logger().info(f"move straight in {i+1} seconds")
        #     self.cmd.linear.x = 0.07
        #     self.cmd.angular.z = 0.0
        #     self.vel_pub.publish(self.cmd)
        #     time.sleep(1)
        # self.get_logger().info(f"move straight in {i+1} seconds")
        self.moving_straight_1s = True
        self.cmd.linear.x = 0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
        time.sleep(1)
        # self.move_stright_3s_done = True
        # self.move_straight_1s_done = True
        # self.stop_robot()
        # self.get_logger().info('move straight 1s completed')
        self.moving_straight_1s = False

    def rotate_clockwise_small_angle(self):
        self.get_logger().info('In rotate_clockwise_small_angle function')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = -0.01
        self.vel_pub.publish(self.cmd)
    
    def rotate_counter_clockwise_small_angle(self):
        self.get_logger().info('In rotate_counter_clockwise_small_angle function')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.01
        self.vel_pub.publish(self.cmd)
        
    def rotate(self):
        self.get_logger().info("Ex2 MOVE ROTATE")
        self.cmd.angular.z = -0.2
        self.cmd.linear.x = 0.0
        self.get_logger().info("PUBLISH COMMAND...")
        self.vel_pub.publish(self.cmd)        
        self.get_logger().info("PUBLISH COMMAND...FINISHED")
        self.get_logger().info("Ex2 Rotating for "+str(self._seconds_sleeping)+" seconds")
        for i in range(self._seconds_sleeping):
            self.get_logger().info("Ex2 SLEEPING=="+str(i)+" seconds")
            time.sleep(1)
        
        self.stop_robot()
    

def main(args=None):
    rclpy.init(args=args)
    try:
        subs_node = SubscriberClass()
        pub_node = PublisherClass()
        
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(subs_node)
        executor.add_node(pub_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            subs_node.destroy_node()
            pub_node.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
