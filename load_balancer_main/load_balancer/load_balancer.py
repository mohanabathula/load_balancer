#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import signal
import rospy
import time
import base64
import numpy as np
import requests
import threading
import json
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image
from hiwonder_interfaces.msg import ObjectsInfo
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from geometry_msgs.msg import Twist
from hiwonder_servo_controllers.bus_servo_control import set_servos

# Resource monitoring utils
from lb_inputs import get_usage_percent, get_network_bandwidth, get_application_table


class LoadBalancerNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.is_running = True
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lock = threading.Lock()

        signal.signal(signal.SIGINT, self.shutdown)

        # Publishers
        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', Twist, queue_size=1)
        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        
        # Publishers for onboard processing
        self.lane_img_pub = rospy.Publisher('/load_balancer/lane_image', Image, queue_size=1)
        self.yolo_img_pub = rospy.Publisher('/load_balancer/yolo_image', Image, queue_size=1)
        self.binary_img_pub = rospy.Publisher("/load_balancer/lane_binary_image", Image, queue_size=10)
        self.result_img_pub = rospy.Publisher("/load_balancer/lane_result_image", Image, queue_size=10)

        # Result publishers
        self.lane_pub = rospy.Publisher("/load_balancer/lane_detection", String, queue_size=10)
        self.obj_pub = rospy.Publisher("/load_balancer/object_detections", String, queue_size=10)
        self.traffic_pub = rospy.Publisher("/load_balancer/traffic_sign_detections", String, queue_size=10)

        # Subscribers
        depth_camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        rospy.Subscriber(f'/{depth_camera}/rgb/image_raw', Image, self.image_callback)

        self.server_url = rospy.get_param('~edge_server_url')
        
        # Configuration
        self.frame_width = 640
        self.frame_height = 480
        self.frame_id = 0
        self.edge_timeout = 2.0  # Reduced timeout
        
        # Data storage
        self.detection_results = []
        self.traffic_sign_results = []
        self.lane_data = {}
        self.binary_image = None
        self.result_image = None

        # Decision thresholds
        self.resource_threshold = rospy.get_param('~resource_threshold')  # 80%
        self.bandwidth_high_threshold = rospy.get_param('~bandwidth_high_threshold')  # 5 Mbps
        self.bandwidth_low_threshold = rospy.get_param('~bandwidth_low_threshold')   # 2 Mbps
        
        # Network status tracking
        self.edge_server_available = True
        self.last_edge_check = 0
        self.edge_check_interval = 5.0  # Check every 5 seconds
        
        # Rate limiting
        self.last_processing_time = 0
        self.min_processing_interval = 0.1  # 10 FPS max

        # Wait for services
        self.wait_for_services()
        
        # Init servos
        self.initialize_servos()
        
        rospy.loginfo("LoadBalancerNode initialized.")
        rospy.spin()

    def wait_for_services(self):
        """Wait for required services to be ready."""
        if not rospy.get_param('~only_line_follow', False):
            while not rospy.is_shutdown():
                try:
                    if rospy.get_param('/yolov5/init_finish'):
                        break
                except:
                    rospy.sleep(0.1)
            try:
                rospy.ServiceProxy('/yolov5/start', Trigger)()
            except:
                rospy.logwarn("Failed to start YOLOv5 service")

    def initialize_servos(self):
        """Initialize servos to initial position."""
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        set_servos(self.joints_pub, 1000, ((6, 490),))
        rospy.sleep(1)
        self.mecanum_pub.publish(Twist())

    def shutdown(self, signum, frame):
        self.is_running = False
        rospy.loginfo("Shutting down LoadBalancerNode...")

    def image_callback(self, ros_image):
        """Image callback with manual conversion (no cv_bridge)."""
        try:
            # Manual conversion from ROS Image to numpy array
            if ros_image.encoding == 'rgb8':
                rgb_image = np.ndarray(
                    shape=(ros_image.height, ros_image.width, 3),
                    dtype=np.uint8,
                    buffer=ros_image.data
                )
            else:
                # Convert other encodings if necessary
                rospy.logwarn(f"Unsupported encoding: {ros_image.encoding}")
                return
                
            self.image = rgb_image
            self.handle_frame(self.image, ros_image)
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def handle_frame(self, frame, ros_image):
        """Process decision logic for onboard vs edge execution following the workflow."""
        # Rate limiting
        # current_time = time.time()
        # if current_time - self.last_processing_time < self.min_processing_interval:
        #     return
        # self.last_processing_time = current_time

        try:
            # Get current system state
            usage = get_usage_percent()
            cpu_usage = usage['cpu_percent']
            ram_usage = usage['ram_percent']
            gpu_usage = usage['gpu_percent']
            bandwidth = get_network_bandwidth()
            upload_speed = bandwidth['upload_Mbps']
            download_speed = bandwidth['download_Mbps']

            # Check edge server availability
            edge_available = self.check_edge_server_available()

            applications = rospy.get_param("~applications")

            for app in applications:
                r = get_application_table(app)
                if r is None:
                    rospy.logwarn(f"Application '{app}' not found in table! Using onboard.")
                    self.forward_to_onboard(app, ros_image)
                    continue

                # Get application requirements
                latency_sensitivity = r.get("latency_sensitivity")
                accuracy_priority = r.get("accuracy_priority")
                print(f"App: {app}, Latency: {latency_sensitivity}, Accuracy: {accuracy_priority}")
                
                # Decision logic based on workflow
                processing_location = self.decide_processing_location(
                    latency_sensitivity=latency_sensitivity,
                    accuracy_priority=accuracy_priority,
                    cpu_usage=cpu_usage,
                    ram_usage=ram_usage,
                    gpu_usage=gpu_usage,
                    upload_speed=upload_speed,
                    download_speed=download_speed,
                    edge_available=edge_available
                )

                if processing_location == "onboard":
                    rospy.loginfo(f"{app}: Processing onboard")
                    self.forward_to_onboard(app, ros_image)
                elif processing_location == "offboard":
                    rospy.loginfo(f"{app}: Offloading to edge")
                    self.forward_to_edge(frame, app)
                elif processing_location == "lower_resolution":
                    rospy.loginfo(f"{app}: Lowering resolution and offloading to edge")
                    low_res_frame = self.lower_resolution(frame)
                    self.forward_to_edge(low_res_frame, app)
                else:
                    rospy.logwarn(f"{app}: Unknown processing location, using onboard")
                    self.forward_to_onboard(app, ros_image)
                        
        except Exception as e:
            rospy.logerr(f"Error in handle_frame: {e}")

    def decide_processing_location(self, latency_sensitivity, accuracy_priority, 
                                cpu_usage, ram_usage, gpu_usage, 
                                upload_speed, download_speed, edge_available):
        """
        Decide where to process based on the workflow:
        
        Workflow:
        1. Check latency sensitivity
        - high → check onboard resources
        - low → check accuracy priority
        
        2. For high latency sensitivity:
        - onboard resources sufficient → onboard
        - onboard resources insufficient → check bandwidth
            - bandwidth sufficient → offboard
            - bandwidth low → check accuracy priority
        
        3. For low latency sensitivity:
        - accuracy priority high → check bandwidth
            - bandwidth sufficient → offboard  
            - bandwidth low → onboard
        - accuracy priority low → check bandwidth
            - bandwidth sufficient → offboard
            - bandwidth low → lower resolution
        """
        
        # Define thresholds
        # RESOURCE_THRESHOLD = 80  # 80% usage considered "insufficient"
        # BANDWIDTH_HIGH_THRESHOLD = 5  # 5 Mbps considered "sufficient"
        # BANDWIDTH_LOW_THRESHOLD = 2   # 2 Mbps considered "low"
        
        # Check if edge is available at all
        if not edge_available:
            return "onboard"
        
        # Check if onboard resources are sufficient
        onboard_resources_sufficient = (
            cpu_usage < self.resource_threshold and 
            ram_usage < self.resource_threshold and 
            gpu_usage < self.resource_threshold
        )
        
        # Check bandwidth levels
        bandwidth_sufficient = (upload_speed >= self.bandwidth_high_threshold and 
                            download_speed >= self.bandwidth_high_threshold)
        bandwidth_low = (upload_speed < self.bandwidth_low_threshold or 
                    download_speed < self.bandwidth_low_threshold)
        
        # Main decision logic following the workflow
        if latency_sensitivity == "high":
            # High latency sensitivity path
            if onboard_resources_sufficient:
                return "onboard"
            else:
                # Resources insufficient
                if bandwidth_sufficient:
                    return "offboard"
                else:
                    # Bandwidth is low, check accuracy priority
                    if accuracy_priority == "high":
                        return "onboard"
                    else:
                        return "lower_resolution"
                        
        else:  # latency_sensitivity is "low" or "medium"
            # Low latency sensitivity path
            if accuracy_priority == "high":
                if bandwidth_sufficient:
                    return "offboard"
                else:
                    return "onboard"
            else:  # accuracy_priority is "low" or "medium"
                if bandwidth_sufficient:
                    return "offboard"
                else:
                    return "lower_resolution"

    def check_edge_server_available(self):
        """Check if edge server is reachable with caching."""
        current_time = time.time()
        if current_time - self.last_edge_check < self.edge_check_interval:
            return self.edge_server_available
            
        try:
            # Simple HEAD request to check server availability
            response = requests.head(self.server_url, timeout=2.0)
            self.edge_server_available = (response.status_code < 500)
            if not self.edge_server_available:
                rospy.logwarn(f"Edge server returned status {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.edge_server_available = False
            rospy.logwarn_once(f"Edge server unavailable: {e}")
        
        self.last_edge_check = current_time
        return self.edge_server_available

    def lower_resolution(self, frame, scale_factor=0.5):
        """Lower the resolution of the frame."""
        new_width = int(frame.shape[1] * scale_factor)
        new_height = int(frame.shape[0] * scale_factor)
        return cv2.resize(frame, (new_width, new_height))
    
    def forward_to_edge(self, frame, app):
        """Send frame to edge server for processing."""
        try:
            # Resize and encode image
            image = cv2.resize(frame, (self.frame_width, self.frame_height))
            success, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            if not success:
                rospy.logwarn("Failed to encode image")
                return

            # Prepare headers
            headers = {
                'Content-Type': 'application/octet-stream',
                'Frame-Width': str(self.frame_width),
                'Frame-Height': str(self.frame_height),
                'Client-Timestamp': str(time.time()),
                'Frame-Identifier': f"frame_{self.frame_id}",
                'detection-flag': app
            }
            
            self.frame_id += 1
            
            # Send request with shorter timeout
            response = requests.post(self.server_url, data=buffer.tobytes(), 
                                   headers=headers, timeout=self.edge_timeout)
            
            if response.status_code == 200:
                self.process_edge_response(response, app)
                # Mark edge as available on success
                self.edge_server_available = True
            else:
                rospy.logwarn(f"Edge server returned status {response.status_code}")
                self.edge_server_available = False
                
        except requests.exceptions.Timeout:
            rospy.logwarn("Edge request timed out")
            self.edge_server_available = False
        except requests.exceptions.RequestException as e:
            rospy.logwarn(f"Edge request failed: {e}")
            self.edge_server_available = False
        except Exception as e:
            rospy.logerr(f"Unexpected error in forward_to_edge: {e}")
            self.edge_server_available = False

    def process_edge_response(self, response, app):
        """Process the response from edge server with proper JSON handling."""
        try:
            # Try to parse JSON response
            try:
                result = response.json()
            except json.JSONDecodeError:
                # If JSON parsing fails, try to parse as string first
                response_text = response.text
                rospy.logwarn(f"Edge response is not valid JSON. Response: {response_text[:200]}...")
                
                # Try to see if it's a JSON string within a string
                if response_text.startswith('{') and response_text.endswith('}'):
                    try:
                        result = json.loads(response_text)
                    except json.JSONDecodeError as e:
                        rospy.logerr(f"Failed to parse edge response as JSON: {e}")
                        return
                else:
                    rospy.logerr(f"Edge response is not JSON: {response_text[:200]}...")
                    return

            with self.lock:
                # Process lane detection
                if 'Lane_detection_model' in result:
                    lane_data = result['Lane_detection_model']
                    # Check if lane_data is a string that needs parsing
                    if isinstance(lane_data, str):
                        try:
                            lane_data = json.loads(lane_data)
                        except json.JSONDecodeError:
                            rospy.logerr(f"Failed to parse lane data string: {lane_data[:200]}...")
                            return
                    self.process_lane_data(lane_data)
                    # print("Processed lane detection data from edge.", lane_data)
                
                # Process object detection
                if 'OBD_model' in result:
                    obd_data = result['OBD_model']
                    if isinstance(obd_data, str):
                        try:
                            obd_data = json.loads(obd_data)
                        except json.JSONDecodeError:
                            rospy.logerr(f"Failed to parse OBD data string: {obd_data[:200]}...")
                            return
                    self.detection_results = obd_data.get('detections', [])
                    obj_msg = String(data=json.dumps(self.detection_results))
                    self.obj_pub.publish(obj_msg)
                    print("Processed object detection data from edge.", self.detection_results)
                
                # Process traffic signs
                if 'TrafficSign_model' in result:
                    traffic_data = result['TrafficSign_model']
                    if isinstance(traffic_data, str):
                        try:
                            traffic_data = json.loads(traffic_data)
                        except json.JSONDecodeError:
                            rospy.logerr(f"Failed to parse traffic data string: {traffic_data[:200]}...")
                            return
                    self.traffic_sign_results = traffic_data.get('detections', [])
                    traffic_msg = String(data=json.dumps(self.traffic_sign_results))
                    self.traffic_pub.publish(traffic_msg)
                    print("Processed traffic sign data from edge.", self.traffic_sign_results)
                    
                rospy.loginfo(f"Successfully processed edge response for {app}")
                    
        except Exception as e:
            rospy.logerr(f"Error processing edge response: {e}")

    def process_lane_data(self, lane_data):
        """Process lane detection data from edge with robust type checking."""
        try:
            # Ensure lane_data is a dictionary
            if not isinstance(lane_data, dict):
                rospy.logerr(f"Lane data is not a dictionary: {type(lane_data)} - {str(lane_data)[:200]}")
                return

            # Safely extract lane data with defaults
            self.lane_data = {
                'horizontal_y': lane_data.get('horizontal_y', 0),
                'vertical_near': {
                    'up': tuple(lane_data.get('vertical_near', {}).get('up', [0, 0])),
                    'down': tuple(lane_data.get('vertical_near', {}).get('down', [0, 0])),
                    'center_y': lane_data.get('vertical_near', {}).get('center_y', 0)
                },
                'vertical_far': {
                    'up': tuple(lane_data.get('vertical_far', {}).get('up', [0, 0])),
                    'down': tuple(lane_data.get('vertical_far', {}).get('down', [0, 0]))
                },
                'lane': {
                    'x': lane_data.get('lane', {}).get('x', -1),
                    'angle': lane_data.get('lane', {}).get('angle', None)
                }
            }
            print("Lane Data from edge:", self.lane_data)
            
            # Publish lane data as JSON string
            lane_msg = String(data=json.dumps(self.lane_data))
            self.lane_pub.publish(lane_msg)
            
            rospy.loginfo(f"Lane data processed - X: {self.lane_data['lane']['x']}, Angle: {self.lane_data['lane']['angle']}")
            
            # Process images if available (using manual conversion)
            if 'binary_image' in lane_data:
                self.process_image_data(lane_data['binary_image'], self.binary_img_pub, "binary")
            
            if 'result_image' in lane_data:
                self.process_image_data(lane_data['result_image'], self.result_img_pub, "result")
                
        except Exception as e:
            rospy.logerr(f"Error processing lane data: {e}")

    def process_image_data(self, image_data, publisher, image_type):
        """Process and publish image data from edge using manual conversion."""
        try:
            # Check if image_data is a string (base64 encoded)
            if isinstance(image_data, str):
                image_bytes = base64.b64decode(image_data)
                np_arr = np.frombuffer(image_bytes, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if cv_image is not None:
                    # Convert CV image to ROS message manually
                    ros_image = self.cv2_to_ros_image(cv_image)
                    publisher.publish(ros_image)
                    rospy.loginfo(f"Published {image_type} image from edge")
                else:
                    rospy.logwarn(f"Failed to decode {image_type} image from edge")
        except Exception as e:
            rospy.logerr(f"Error processing {image_type} image: {e}")

    def cv2_to_ros_image(self, cv_image):
        """Convert CV2 image to ROS Image message without cv_bridge."""
        ros_image = Image()
        ros_image.height = cv_image.shape[0]
        ros_image.width = cv_image.shape[1]
        ros_image.encoding = 'bgr8'
        ros_image.is_bigendian = 0
        ros_image.step = cv_image.shape[2] * cv_image.shape[1]  # width * channels
        ros_image.data = cv_image.tobytes()
        return ros_image

    def forward_to_onboard(self, app, ros_image):
        """Forward image to onboard processing nodes."""
        try:
            if app == "lane_detection":
                self.lane_img_pub.publish(ros_image)
            elif app == "traffic_light_detection":
                self.yolo_img_pub.publish(ros_image)
            elif app == "traffic_sign_detection":
                self.yolo_img_pub.publish(ros_image)
            elif app == "collision_detection":
                self.yolo_img_pub.publish(ros_image)
            rospy.loginfo(f"Forwarded frame to onboard {app}.")
        except Exception as e:
            rospy.logerr(f"Error forwarding to onboard {app}: {e}")


if __name__ == "__main__":
    LoadBalancerNode('load_balancer')