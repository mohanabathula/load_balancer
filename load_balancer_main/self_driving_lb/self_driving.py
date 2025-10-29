#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/28
# @author:aiden
# 无人驾驶
import os
import cv2
import math
import rospy
import signal
import threading
import numpy as np
# import lane_detect
import time
import hiwonder_sdk.pid as pid
import hiwonder_sdk.misc as misc
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import geometry_msgs.msg as geo_msg
import hiwonder_sdk.common as common
from hiwonder_interfaces.msg import ObjectsInfo
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos
from servo_controller_arduino import ServoController
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class SelfDrivingNodeLB:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.depth_image = None
        self.is_running = True
        self.pid = pid.PID(0.01, 0.0, 0.0)

        self.detect_far_lane = False
        self.park_x = -1  # 停车标识的x像素坐标

        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.start_turn = False  # 开始转弯

        self.count_right = 0
        self.count_right_miss = 0
        self.turn_right = False  # 右转标志
        
        self.stop = False  # 停下标识
        self.start_park = False  # 开始泊车标识

        self.count_crosswalk = 0
        self.crosswalk_distance = 0  # 离斑马线距离
        self.crosswalk_length = 0.1 + 0.35  # 斑马线长度 + 车长

        self.start_slow_down = False  # 减速标识
        self.normal_speed = 0.15  # 正常前进速度
        self.slow_down_speed = 0.1  # 减速行驶的速度

        self.traffic_signs_status = None  # 记录红绿灯状态

        self.target_class="keyboard"
        self.object_distance = 1

        self.lane_data = {
            'horizontal_y': 0,
            'vertical_near': {'up': (0,0), 'down': (0,0), 'center_y': 0},
            'vertical_far': {'up': (0,0), 'down': (0,0)},
            'lane': {'x': -1, 'angle': 0}
        }

        self.colors = common.Colors()
        signal.signal(signal.SIGINT, self.shutdown)
        self.machine_type = os.environ.get('MACHINE_TYPE')
        # self.lane_detect = lane_detect.LaneDetector("yellow")
        
        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', geo_msg.Twist, queue_size=1)  # 底盘控制
        # self.result_publisher = rospy.Publisher(self.name + '/image_result', Image, queue_size=1)  # 图像处理结果发布
        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        # depth_camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')  # 获取参数

        # if rospy.get_param('~use_depth_cam', True):
        #     camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        #     self.image_sub_rgb = rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, self.image_callback, queue_size=1)
        #     self.image_sub_depth = rospy.Subscriber('/%s/depth/image_raw'%camera, Image, self.depth_callback, queue_size=1)
        # else:
        #     camera = rospy.get_param('/usb_cam_name', 'usb_cam')
        #     self.image_sub = rospy.Subscriber('/%s/image_raw'%camera, Image, self.image_callback, queue_size=1)
        
        # Subscribers - use minimal setup first
        rospy.loginfo("Setting up minimal subscribers...")

        camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        self.image_sub_rgb = rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, self.image_callback, queue_size=1)
        self.image_sub_depth = rospy.Subscriber('/%s/depth/image_raw'%camera, Image, self.depth_callback, queue_size=1)
        
        # Object detection subscribers
        # self.obj_sub = rospy.Subscriber('/load_balancer/yolov5/traffic_detect', ObjectsInfo, self.get_object_callback)
        # self.od_sub = rospy.Subscriber('/load_balancer/yolov5/object_detect', ObjectsInfo, self.get_od_callback)
        
        rospy.loginfo("Subscribers set up")
        
        #T
        rospy.Subscriber('/load_balancer/yolov5/traffic_detect', ObjectsInfo, self.get_object_callback)
        rospy.Subscriber('/load_balancer/yolov5/object_detect', ObjectsInfo, self.get_od_callback)
        
        #Lane detection results
        rospy.Subscriber('/lane_detection/result', String, self.lane_result_callback)
        rospy.Subscriber('/lane_detection/result_image', Image, self.lane_image_callback)
        rospy.Subscriber('/lane_detection/binary_image', Image, self.binary_image_callback)

        # if not rospy.get_param('~only_line_follow', False):
        #     while not rospy.is_shutdown():
        #         try:
        #             if rospy.get_param('/yolov5_k/init_finish'):
        #                 break
        #         except:
        #             rospy.sleep(0.1)
        #     rospy.ServiceProxy('/yolov5_k/start', Trigger)()
        # while not rospy.is_shutdown():
        #     try:
        #         if rospy.get_param('/hiwonder_servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
        #             break
        #     except:
        #         rospy.sleep(0.1)
        
        # Servo controller setup with timeout
        try:
            self.servo_controller = ServoController()
            self.servo_controller.connect()
            self.set_default_servo_position()
            rospy.loginfo("Servo controller initialized")
        except Exception as e:
            rospy.logwarn(f"Servo controller not available: {e}")
            self.servo_controller = None

        set_servos(self.joints_pub, 1000, ((6, 490),))  # 初始姿态
        rospy.sleep(1)
        self.mecanum_pub.publish(geo_msg.Twist())
        #self.park_action() 
        self.image_proc()

        # # Start control loop immediately without waiting for other nodes
        # self.control_thread = threading.Thread(target=self.control_loop)
        # self.control_thread.daemon = True
        # self.control_thread.start()
        
        rospy.loginfo(f"{self.name} initialized successfully - control loop started")

    def shutdown(self, signum, frame):  # ctrl+c关闭处理
        self.is_running = False
        rospy.loginfo('shutdown')
        self.mecanum_pub.publish(geo_msg.Twist())
        self.set_default_servo_position()
        cv2.destroyAllWindows()

    def control_loop(self):
        """Main control loop"""
        rate = rospy.Rate(10)  # 10 Hz
        control_count = 0
        self.image_proc()
        
        while self.is_running and not rospy.is_shutdown():
            control_count += 1
            self.image_proc()
            
            # Log status every 2 seconds
            if control_count % 20 == 0:
                rospy.loginfo(f"Control loop running - cycles: {control_count}, image: {self.image is not None}")
            
            rate.sleep()

    def image_callback(self, ros_image):  # 目标检查回调
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        self.image = rgb_image

    def depth_callback(self, ros_image):
        try:
            # Extract the height, width, and encoding from the message
            height = ros_image.height
            width = ros_image.width
            encoding = ros_image.encoding  # Commonly "16UC1" or "32FC1" for depth images
        
            #rospy.loginfo(f"Received depth image with encoding: {encoding}, height: {height}, width: {width}")

            # Check encoding type and convert data to a NumPy array
            if encoding == "16UC1":
                self.depth_image = np.frombuffer(ros_image.data, dtype=np.uint16).reshape(height, width)
            elif encoding == "32FC1":
                self.depth_image = np.frombuffer(ros_image.data, dtype=np.float32).reshape(height, width)
            else:
                print(f"Unsupported encoding: {encoding}")

        except Exception as e:
            print(f"Error processing depth image: {e}")
    
    def lane_result_callback(self, msg):
        """Callback for lane detection JSON results"""
        try:
            self.lane_results = json.loads(msg.data)
            if self.lane_results:
                self.lane_data = {
                    'horizontal_y': self.lane_results.get('horizontal_y', 0),
                    'vertical_near': {
                        'up': tuple(self.lane_results.get('vertical_near', {}).get('up', [0, 0])),
                        'down': tuple(self.lane_results.get('vertical_near', {}).get('down', [0, 0])),  # FIXED: gett → get
                        'center_y': self.lane_results.get('vertical_near', {}).get('center_y', 0)
                    },
                    'vertical_far': {
                        'up': tuple(self.lane_results.get('vertical_far', {}).get('up', [0, 0])),
                        'down': tuple(self.lane_results.get('vertical_far', {}).get('down', [0, 0]))  # FIXED: gett → get
                    },
                    'lane': {
                        'x': self.lane_results.get('lane', {}).get('x', -1),
                        'angle': self.lane_results.get('lane', {}).get('angle', None)
                    }
                }

            rospy.loginfo_throttle(1, f"Lane data updated from onboard: {self.lane_data}")

        except Exception as e:
            rospy.logerr(f"Error parsing lane result JSON: {e}")
                
    def lane_image_callback(self, msg):
        """Callback for processed lane image"""
        try:
            # Check if bridge exists, if not use manual conversion
            if hasattr(self, 'bridge') and self.bridge is not None:
                self.lane_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                # Manual conversion
                self.lane_image = self.manual_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting lane image: {e}")

    def binary_image_callback(self, msg):
        """Callback for binary lane image"""
        try:
            # Check if bridge exists, if not use manual conversion
            if hasattr(self, 'bridge') and self.bridge is not None:
                self.binary_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            else:
                # Manual conversion
                self.binary_image = self.manual_imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            rospy.logerr(f"Error converting binary image: {e}")

    def manual_imgmsg_to_cv2(self, ros_image, encoding='bgr8'):
        """Manual conversion from ROS Image to OpenCV without cv_bridge"""
        try:
            # Convert ROS Image to numpy array
            if encoding == 'bgr8':
                if ros_image.encoding == 'rgb8':
                    rgb_image = np.ndarray(
                        shape=(ros_image.height, ros_image.width, 3),
                        dtype=np.uint8,
                        buffer=ros_image.data
                    )
                    return cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                elif ros_image.encoding == 'bgr8':
                    return np.ndarray(
                        shape=(ros_image.height, ros_image.width, 3),
                        dtype=np.uint8,
                        buffer=ros_image.data
                    )
            elif encoding == 'mono8':
                if ros_image.encoding == 'mono8':
                    return np.ndarray(
                        shape=(ros_image.height, ros_image.width),
                        dtype=np.uint8,
                        buffer=ros_image.data
                    )
                else:
                    # Convert other encodings to mono8 if needed
                    rospy.logwarn(f"Converting {ros_image.encoding} to mono8")
                    # First convert to BGR, then to grayscale
                    bgr_image = self.manual_imgmsg_to_cv2(ros_image, 'bgr8')
                    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            
            rospy.logwarn(f"Unsupported conversion: {ros_image.encoding} to {encoding}")
            return None
            
        except Exception as e:
            rospy.logerr(f"Manual image conversion error: {e}")
            return None

    def get_coordinates(self, bbox):
        # center_pixels = []
        coordinates_in_bbox=[]
        x_min, y_min, x_max, y_max = map(int, bbox)
        for x in range(x_min,x_max+1):
            for y in range(y_min,y_max+1):
                 coordinates_in_bbox.append((x,y))
    #             # center_x = ((x_min + x_max) / 2) #* (640/1200)  # Wdepth/Wrgb
    #             # center_y = ((y_min + y_max) / 2) #* (480/800)   # Hdepth/Hrgb
    #             # center_pixels.append((round(center_x), round(center_y)))
        # return center_pixels
        return coordinates_in_bbox   
    
    def get_distance(self,depth_image,coordinates):        
        distance=[]
        distance_of_object=1
        for x,y in coordinates:
            height, width = depth_image.shape
            if 0 <= x < width and 0 <= y < height:
                depth = float(depth_image[y,x])
                #rospy.loginfo(f"Depth   :  {depth} ") 
                distance.append(depth)
            else:
                print(f"index x={x}, y={y}")
                print(f"Index out of bounds: x={x}, y={y}, height={height}, width={width}")
                
        non_zero_distances = [d for d in distance if d > 0]
        min_distance = min(non_zero_distances) if non_zero_distances else None
        rospy.loginfo(f"Minimum distance  :  {min_distance}")

        if min_distance is not None:
            distance_of_object = (min_distance) / 1000
            rospy.loginfo(f"Distance from the object  {distance_of_object}  meters")
        else:
            rospy.loginfo("No valid distance found.")
            return None
        return distance_of_object
    
    def set_default_servo_position(self):
        """Set servo to a neutral position."""
        self.send_servo_command(0)

    def send_servo_command(self, angular_z):
        """Send servo position command based on angular_z."""
        servo_position = self.map_angular_to_digital(angular_z)
        rospy.loginfo(f"Sending servo command: {servo_position}")
        commands = [(1, servo_position)]
        self.servo_controller.send_commands(commands)

    def map_angular_to_digital(self, angular_z):
        """Map angular velocity to servo digital position."""
        angular_z = max(-1, min(angular_z, 1))  # Clamp between -1 and 1
        mapped_value = int(10 + ((angular_z + 1) * 990) / 2)
        return str(mapped_value)

    # 泊车处理
    def park_action(self):
        twist = geo_msg.Twist()
        twist.linear.x = 0.15
        twist.angular.z = twist.linear.x*math.tan(-0.6)/0.213
        self.mecanum_pub.publish(twist)
        self.send_servo_command(twist.angular.z)
        rospy.sleep(3)

        twist = geo_msg.Twist()
        twist.linear.x = 0.15
        twist.angular.z = -twist.linear.x*math.tan(-0.6)/0.213
        self.mecanum_pub.publish(twist)
        self.send_servo_command(twist.angular.z)
        rospy.sleep(2)

        twist = geo_msg.Twist()
        twist.linear.x = -0.15
        twist.angular.z = twist.linear.x*math.tan(-0.6)/0.213
        self.mecanum_pub.publish(twist)
        self.send_servo_command(twist.angular.z)
        rospy.sleep(1.5)

        self.mecanum_pub.publish(geo_msg.Twist())
        self.set_default_servo_position()

    def image_proc(self):
        while self.is_running:
            if self.image is not None:
                # print("Entered image process")
                h, w = self.image.shape[:2]

                twist = geo_msg.Twist()
                self.start_object_detection= time.time()
                distance= self.object_distance
                if distance is None:
                    if self.stop:
                        rospy.loginfo(f"No close object detected. Resuming movement.")
                    self.stop = False

                if distance is not None and distance <= 0.5:
                    rospy.loginfo(f"Objects detected. Stopping vehicle.")
                    print("distance",distance)
                    self.stop = True
                    self.actuation_of_stop=time.time()
                    self.delay=self.actuation_of_stop - self.start_object_detection
                    rospy.loginfo(f"The time delay is: {self.delay}")
                    self.mecanum_pub.publish(geo_msg.Twist())

                # No object detected close by, resume moving
                else:
                    if self.stop:
                        rospy.loginfo(f"No close object detected. Resuming movement.")
                    self.stop = False

                # 获取车道线的二值化图
                # binary_image = self.lane_detect.get_binary(self.image)
                # 检测到斑马线,开启减速标志
                if 450 < self.crosswalk_distance and not self.start_slow_down:  # 只有足够近时才开始减速
                    self.count_crosswalk += 1
                    if self.count_crosswalk == 3:  # 多次判断，防止误检测
                        self.count_crosswalk = 0
                        self.start_slow_down = True  # 减速标识
                        self.count_slow_down = rospy.get_time()  # 减速固定时间
                else:  # 需要连续检测，否则重置
                    self.count_crosswalk = 0

                twist = geo_msg.Twist()
                # 减速行驶处理
                if self.start_slow_down:
                    if self.traffic_signs_status == 'red':  # 如果遇到红灯就停车
                        self.mecanum_pub.publish(geo_msg.Twist())
                        self.set_default_servo_position()
                        self.stop = True
                    
                    elif self.traffic_signs_status == 'green':  # 遇到绿灯，速度放缓
                        twist.linear.x = self.slow_down_speed
                        self.stop = False
                        twist.angular.z = 0
                        self.mecanum_pub.publish(twist)
                        self.send_servo_command(twist.angular.z)

                    elif not self.stop:  # 其他非停止的情况速度放缓， 同时计时，时间=斑马线的长度/行驶速度
                        twist.linear.x = self.slow_down_speed
                        if rospy.get_time() - self.count_slow_down > self.crosswalk_length/twist.linear.x:
                            self.start_slow_down = False
                else:
                    twist.linear.x = self.normal_speed  # 直走正常速度
                
                # 检测到 停车标识+斑马线 就减速, 让识别稳定
                if 0 < self.park_x and 150 < self.crosswalk_distance:
                    twist.linear.x = self.slow_down_speed
                    if not self.start_park and 255 < self.crosswalk_distance:  # 离斑马线足够近时就开启停车
                        self.mecanum_pub.publish(geo_msg.Twist())
                        self.set_default_servo_position()
                        self.start_park = True
                        self.stop = True
                        threading.Thread(target=self.park_action).start()                       
                
                # 右转及停车补线策略
                if self.turn_right:
                    # y = self.lane_detect.add_horizontal_line(binary_image)
                    y = self.lane_data['horizontal_y']
                    if 300 < y < 400:
                        roi = [(0, y), (w, y), (w, 0), (0, 0)]
                        cv2.fillPoly(self.binary_image, [np.array(roi)], [0, 0, 0])  # 将上面填充为黑色，防干扰
                        min_x = cv2.minMaxLoc(self.binary_image)[-1][0]
                        cv2.line(self.binary_image, (min_x, y), (w, y), (255, 255, 255), 30)  # 画虚拟线来驱使转弯
                elif 0 < self.park_x and not self.start_turn:  # 检测到停车标识需要填补线，使其保持直走
                    if not self.detect_far_lane:
                        # up, down, center = self.lane_detect.add_vertical_line_near(binary_image)
                        up= self.lane_data['vertical_near']['up']
                        down = self.lane_data['vertical_near']['down']
                        center = self.lane_data['vertical_near']['center_y']
                        self.binary_image[:, :] = 0  # 全置黑，防止干扰
                        if center > 130:  # 当将要看不到车道线时切换到识别较远车道线
                            self.detect_far_lane = True
                    else:
                        # up, down = self.lane_detect.add_vertical_line_far(binary_image)
                        up= self.lane_data['vertical_far']['up']
                        down = self.lane_data['vertical_far']['down']
                        self.binary_image[:, :] = 0
                    if up != down:
                        cv2.line(self.binary_image, up, down, (255, 255, 255), 20)  # 手动画车道线

                # result_image, lane_angle, lane_x = self.lane_detect(binary_image, self.image.copy())  # 在处理后的图上提取车道线中心
                # 巡线处理
                lane_x = self.lane_data['lane']['x']
                # print("lane",lane_x)
                lane_angle = self.lane_data['lane']['angle']
                if lane_x >= 0 and not self.stop:
                    if lane_x > 140:  # 转弯
                        if self.turn_right:  # 如果是检测到右转标识的转弯
                            self.count_right_miss += 1
                            if self.count_right_miss >= 100:
                                self.count_right_miss = 0
                                self.turn_right = False
                        self.count_turn += 1
                        if self.count_turn > 5 and not self.start_turn:  # 稳定转弯
                            self.start_turn = True
                            self.count_turn = 0
                            self.start_turn_time_stamp = rospy.get_time()
                        twist.angular.z = twist.linear.x*math.tan(-0.6)/0.213  # 转弯速度
                    else:  # 直道由pid计算转弯修正
                        self.count_turn = 0
                        if rospy.get_time() - self.start_turn_time_stamp > 2 and self.start_turn:
                            self.start_turn = False
                        if not self.start_turn:
                            if abs(lane_x - 90) < 15:
                                lane_x = 90
                            self.pid.SetPoint = 90  # 在车道中间时线的坐标
                            self.pid.update(lane_x)
                            twist.angular.z = twist.linear.x*math.tan(misc.set_range(self.pid.output, -0.1, 0.1))/0.213
                        else:
                            twist.angular.z = 0.15*math.tan(-0.6)/0.213  # 转弯速度
                    # print(twist)
                    self.send_servo_command(twist.angular.z)
                    self.mecanum_pub.publish(twist)
                else:
                    self.pid.clear()
                # bgr_image = cv2.cvtColor(self.lane_image, cv2.COLOR_RGB2BGR)
                # cv2.imshow('result', bgr_image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.is_running = False
                #ros_image.data = result_image.tostring()
                #self.result_publisher.publish(ros_image)
            else:
                rospy.sleep(0.01)

        # print(self.distance_of_object)
        self.mecanum_pub.publish(geo_msg.Twist())
        self.set_default_servo_position()

    def get_od_callback(self, od_msg):
        od_objects_info = od_msg.objects
        print(od_objects_info)
        if od_objects_info == []:  # 没有识别到时重置变量
            print("No objects detected.")
        else:
            # Filter class ID 0
            filtered_boxes = []
            filtered_scores = []

            for i in od_objects_info:
                class_name = i.class_name
                if class_name==self.target_class:
                    filtered_boxes.append(i.box)
                    filtered_scores.append(i.score)

            # print("Filtered:", filtered_boxes, filtered_scores)
            
            # Proceed only if we have at least one class name "PERSON"
            if filtered_boxes:
                for i in range(len(filtered_boxes)):
                    bbox = filtered_boxes[i]
                    
                    if bbox is not None:
                        bbox_list = list(bbox)
                        # print("BBox list:", bbox_list)

                        coordinates = self.get_coordinates(bbox_list)
                        if coordinates:
                            self.object_distance = self.get_distance(self.depth_image, coordinates)
                            # return self.object_distance
                        else:
                            # print("No coordinates found in bounding box.")
                            # return None
                            self.object_distance=None
            else:
                # print("No detections with target class found.")
                self.object_distance=None
                return None

    # 获取目标检测结果
    def get_object_callback(self, msg):
        objects_info = msg.objects
        print(objects_info)
        if objects_info == []:  # 没有识别到时重置变量
            self.traffic_signs_status = None
            self.crosswalk_distance = 0
        else:
            min_distance = 0
            for i in objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                
                if class_name == 'crosswalk':  
                    if center[1] > min_distance:  # 获取最近的人行道y轴像素坐标
                        min_distance = center[1]
                elif class_name == 'right':  # 获取右转标识
                    self.count_right += 1
                    self.count_right_miss = 0
                    if self.count_right >= 10:  # 检测到多次就将右转标志至真
                        self.turn_right = True
                        self.count_right = 0
                elif class_name == 'park':  # 获取停车标识中心坐标
                    self.park_x = center[0]
                elif class_name == 'red' or class_name == 'green':  # 获取红绿灯状态
                    self.traffic_signs_status = class_name
        
            self.crosswalk_distance = min_distance

if __name__ == "__main__":
    rospy.loginfo("Starting SelfDrivingNodeLB...")
    node = SelfDrivingNodeLB('self_driving_lb')
    
    try:
        rospy.loginfo("Entering ROS spin...")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        rospy.loginfo("Node shutdown complete")
