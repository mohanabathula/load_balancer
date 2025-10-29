#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# yolov5目标检测
import os
import cv2
import time
import rospy
import signal
import numpy as np
from std_msgs.msg import Float64
import hiwonder_sdk.fps as fps
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from yolov5_trt import YoLov5TRT, colors, plot_one_box
from hiwonder_interfaces.msg import ObjectInfo, ObjectsInfo
# REMOVED: from cv_bridge import CvBridge

MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        
        self.bgr_image = None
        self.running = True
        self.start = False

        self.last_yolo_image = None

        signal.signal(signal.SIGINT, self.shutdown)

        
        #TRAFFIC_SIGN_DETCTEION
        self.fps = fps.FPS()  # fps计算器
        engine = rospy.get_param('~engine')
        lib = rospy.get_param('~lib')
        conf_thresh = rospy.get_param('~conf_thresh', 0.8)
        self.classes = rospy.get_param('~classes')
        
        self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, engine), os.path.join(MODE_PATH, lib), self.classes, conf_thresh)

        #OBJECT_DETCTEION
        od_engine = rospy.get_param('~od_engine')
        self.od_classes = rospy.get_param('~od_classes')
        
        self.yolov5_od = YoLov5TRT(os.path.join(MODE_PATH, od_engine), os.path.join(MODE_PATH, lib), self.od_classes, conf_thresh)
        

        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        
        #SUBSCRIBE IMAGE FROM LB
        self.yolo_sub = rospy.Subscriber("/load_balancer/yolo_image", Image, self.yolo_callback, queue_size=1)
        
        #TRAFFIC SIGN DETECTION RESULTS PUBLISH
        self.object_pub = rospy.Publisher('/load_balancer/yolov5/traffic_detect', ObjectsInfo, queue_size=1)
        self.result_image_pub = rospy.Publisher('/load_balancer/yolov5/traffic_image', Image, queue_size=1)
        
        
        #OBJECT DETECTION RESULTS PUBLISH
        self.od_object_pub = rospy.Publisher('/load_balancer/yolov5/object_detect', ObjectsInfo, queue_size=1)
        self.od_result_image_pub = rospy.Publisher('/load_balancer/yolov5/object_image', Image, queue_size=1)
        self.start_time = rospy.Publisher('/load_balancer/yolov5/object_detection_start_time', Float64, queue_size=1)

        rospy.set_param('~init_finish', True)
        self.image_proc()

    def start_srv_callback(self, msg):
        rospy.loginfo("start yolov5 detect")
        self.start = True
        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop yolov5 detect')
        self.start = False
        return TriggerResponse(success=True)

    def yolo_callback(self, msg):
        """Receive YOLO image from load balancer using manual conversion."""
        try:
            # Manual conversion from ROS Image to OpenCV
            if msg.encoding == 'rgb8':
                rgb_image = np.ndarray(
                    shape=(msg.height, msg.width, 3),
                    dtype=np.uint8,
                    buffer=msg.data
                )
                self.bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                self.bgr_image = np.ndarray(
                    shape=(msg.height, msg.width, 3),
                    dtype=np.uint8,
                    buffer=msg.data
                )
            else:
                rospy.logwarn(f"Unsupported image encoding: {msg.encoding}")
                return

            # Show for debugging
            cv2.imshow("YOLO Image from Load Balancer", self.bgr_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error in YOLO image callback: {e}")
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_proc(self):
        while self.running:
            if self.bgr_image is not None:
                image = self.bgr_image
                try:
                    # Traffic sign detection
                    rospy.loginfo("Frame received to traffic-sign")
                    infer_yolo_image = image.copy()
                    objects_info = []
                    boxes, scores, classid = self.yolov5.infer(infer_yolo_image)
                    for box, cls_conf, cls_id in zip(boxes, scores, classid):
                        color = colors(cls_id, True)

                        object_info = ObjectInfo()
                        object_info.class_name = self.classes[cls_id]
                        object_info.box = box.astype(int)
                        object_info.score = cls_conf

                        objects_info.append(object_info)

                        plot_one_box(
                            box,
                            image,
                            color=color,
                            label="{}:{:.2f}".format(
                                self.classes[cls_id], cls_conf
                            ),
                        )
                    # print(boxes, scores, classid)
                    object_msg = ObjectsInfo()
                    object_msg.objects = objects_info
                    self.object_pub.publish(object_msg)
                    
                    rospy.sleep(0.01)

                    # Object detection
                    rospy.loginfo("Frame received to object-detection")
                    infer_yolo_od_image = image.copy()
                    objects_info = []
                    object_start_time = time.time()
                    boxes, scores, classid = self.yolov5_od.infer(infer_yolo_od_image)
                    for box, cls_conf, cls_id in zip(boxes, scores, classid):
                        color = colors(cls_id, True)

                        object_info = ObjectInfo()
                        object_info.class_name = self.od_classes[cls_id]
                        object_info.box = box.astype(int)
                        object_info.score = cls_conf

                        objects_info.append(object_info)

                        plot_one_box(
                            box,
                            image,
                            color=color,
                            label="{}:{:.2f}".format(
                                self.od_classes[cls_id], cls_conf
                            ),
                        )
                    # print(boxes, scores, classid)
                    object_msg = ObjectsInfo()
                    object_msg.objects = objects_info
                    self.od_object_pub.publish(object_msg)
                    self.start_time.publish(object_start_time)
                    

                except BaseException as e:
                    print(e)

                # Publish result images using manual conversion
                # self.result_image_pub.publish(self.cv2_to_ros_image(infer_yolo_image))
                # self.od_result_image_pub.publish(self.cv2_to_ros_image(infer_yolo_od_image))
                
            else:
                rospy.sleep(0.01)
        
        self.yolov5.destroy()
        self.yolov5_od.destroy()  
        rospy.signal_shutdown('shutdown')

    def cv2_to_ros_image(self, cv_image):
        """Convert CV2 image to ROS Image message without cv_bridge."""
        # Convert BGR to RGB for ROS
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        ros_image = Image()
        ros_image.height = rgb_image.shape[0]
        ros_image.width = rgb_image.shape[1]
        ros_image.encoding = 'rgb8'
        ros_image.is_bigendian = 0
        ros_image.step = rgb_image.shape[2] * rgb_image.shape[1]  # width * channels
        ros_image.data = rgb_image.tobytes()
        return ros_image

# REMOVED: def cv2_image2ros(image) - replaced by class method

if __name__ == "__main__":
    node = Yolov5Node('yolov5_lb')