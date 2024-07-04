
# ROS2 related
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo  # Add this line to import CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from pose_msgs.msg import BodypartDetection, PersonDetection  # For pose_msgs
from rclpy.duration import Duration
from cv_bridge import CvBridge

# TRT_pose related
import cv2
import numpy as np
import math
import os
from ros2_trt_pose.utils import preprocess, load_params, load_model, draw_objects


class TRTPose(Node):
    def __init__(self):
        super().__init__('trt_pose')
        self.hp_json_file = None
        self.model_weights = None
        self.width = 224
        self.height = 224
        self.i = 0
        self.image = None
        self.depth_image = None
        self.camera_info = None
        self.model_trt = None
        self.annotated_image = None
        self.counts = None
        self.peaks = None
        self.objects = None
        self.topology = None
        self.xy_circles = []
        self.p = None
        # ROS2 parameters
        self.declare_parameter('base_dir', '/home/victorkawai/trt_pose/tasks/human_pose') 
        # Based Dir should contain: model_file resnet/densenet, human_pose json file
        self.declare_parameter('model', 'resnet18') # default to Resnet18
        self.declare_parameter('point_range', 10) # default range is 0 to 10
        self.declare_parameter('show_image', False) # Show image in cv2.imshow
        self.base_dir = self.get_parameter('base_dir')._value
        self.model_name = self.get_parameter('model')._value
        self.point_range = self.get_parameter('point_range')._value
        self.show_image_param = self.get_parameter('show_image')._value

        # ROS2 related init
        # Image subscriber from cam2image
        self.rgb_subscriber = self.create_subscription(ImageMsg, '/rgb/image_raw', self.read_cam_callback, 10)
        self.depth_subscriber = self.create_subscription(ImageMsg, '/depth/image_raw', self.depth_callback, 10)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, '/rgb/camera_info', self.camera_info_callback, 10)
        self.image_pub = self.create_publisher(ImageMsg, 'detections_image', 10)
        # Publisher for Body Joints and Skeleton
        self.body_joints_pub = self.create_publisher(Marker, 'body_joints', 1000)
        self.body_skeleton_pub = self.create_publisher(Marker, 'body_skeleton', 10)
        # Publishing pose Message
        self.publish_pose = self.create_publisher(PersonDetection, 'pose_msgs', 100)
        # Transform Images
        self.bridge = CvBridge()
        self.target_height = 480
        self.target_width = 640

    def start(self):
        # Convert to TRT and Load Params
        json_file = os.path.join(self.base_dir, 'human_pose.json')
        self.get_logger().info("Loading model weights\n")
        self.num_parts, self.num_links, self.model_weights, self.parse_objects, self.topology = load_params(base_dir=self.base_dir,
                                                                                             human_pose_json=json_file,
                                                                                             model_name=self.model_name)
        self.model_trt, self.height, self.width = load_model(base_dir=self.base_dir, model_name=self.model_name, num_parts=self.num_parts, num_links=self.num_links,
                                    model_weights=self.model_weights)
        self.get_logger().info("Model weights loaded...\n Waiting for images...\n")

    def execute(self):
        data = preprocess(image=self.image, width=self.width, height=self.height)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        self.counts, self.objects, self.peaks = self.parse_objects(cmap,
                                                                   paf)  # , cmap_threshold=0.15, link_threshold=0.15)
        annotated_image = draw_objects(image=self.image, object_counts=self.counts, objects=self.objects, normalized_peaks=self.peaks, topology=self.topology)
        self.parse_k()

        return annotated_image

    # Subscribe and Publish to image topic
    def read_cam_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize the image to the target dimensions
            resized_image = cv2.resize(cv_image, (self.target_width, self.target_height))

            # Set the image for pose estimation
            self.image = resized_image

            # Perform pose estimation
            self.annotated_image = self.execute()

            # Convert back to ROS Image message
            image_msg = self.image_np_to_image_msg(self.annotated_image)
            self.image_pub.publish(image_msg)

            # Optionally show the image
            if self.show_image_param:
                cv2.imshow('frame', self.annotated_image)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    # Borrowed from OpenPose-ROS repo
    def image_np_to_image_msg(self, image_np):
        image_msg = ImageMsg()
        image_msg.height = image_np.shape[0]
        image_msg.width = image_np.shape[1]
        image_msg.encoding = 'bgr8'
        image_msg.data = image_np.tobytes()
        image_msg.step = len(image_msg.data) // image_msg.height
        image_msg.header.frame_id = 'map'
        return image_msg

    def init_body_part_msg(self):
        bodypart = BodypartDetection()
        bodypart.x = float('NaN')
        bodypart.y = float('NaN')
        bodypart.confidence = float('NaN')
        return bodypart

    def write_body_part_msg(self, pixel_location):
        body_part_pixel_loc = BodypartDetection()
        body_part_pixel_loc.y = float(pixel_location[0] * self.height)
        body_part_pixel_loc.x = float(pixel_location[1] * self.width)
        return body_part_pixel_loc
    
    def convert_to_3d(self, keypoints_2d):
        keypoints_3d = []
        for (u, v) in keypoints_2d:
            z = self.depth_image[v, u]  # Depth value at (u, v)
            x = (u - self.camera_info.k[2]) * z / self.camera_info.k[0]
            y = (v - self.camera_info.k[5]) * z / self.camera_info.k[4]
            keypoints_3d.append((x, y, z))
        return keypoints_3d

    def init_markers_spheres(self):
        marker_joints = Marker()
        marker_joints.header.frame_id = '/map'
        marker_joints.id = 1
        marker_joints.ns = "joints"
        marker_joints.type = marker_joints.SPHERE_LIST
        marker_joints.action = marker_joints.ADD
        marker_joints.scale.x = 0.7
        marker_joints.scale.y = 0.7
        marker_joints.scale.z = 0.7
        marker_joints.color.a = 1.0
        marker_joints.color.r = 1.0
        marker_joints.color.g = 0.0
        marker_joints.color.b = 0.0
        marker_joints.lifetime = Duration(seconds=3, nanoseconds=5e2).to_msg()
        return marker_joints

    def init_markers_lines(self):
        marker_line = Marker()
        marker_line.header.frame_id = '/map'
        marker_line.id = 1
        marker_line.ns = "joint_line"
        marker_line.header.stamp = self.get_clock().now().to_msg()
        marker_line.type = marker_line.LINE_LIST
        marker_line.action = marker_line.ADD
        marker_line.scale.x = 0.1
        marker_line.scale.y = 0.1
        marker_line.scale.z = 0.1
        marker_line.color.a = 1.0
        marker_line.color.r = 0.0
        marker_line.color.g = 1.0
        marker_line.color.b = 0.0
        marker_line.lifetime = Duration(seconds=3, nanoseconds=5e2).to_msg()
        return marker_line

    def init_all_body_msgs(self, _msg, count):
        _msg.person_id = count
        _msg.nose = self.init_body_part_msg()
        _msg.neck = self.init_body_part_msg()
        _msg.right_shoulder = self.init_body_part_msg()
        _msg.right_elbow = self.init_body_part_msg()
        _msg.right_wrist = self.init_body_part_msg()
        _msg.left_shoulder = self.init_body_part_msg()
        _msg.left_elbow = self.init_body_part_msg()
        _msg.left_wrist = self.init_body_part_msg()
        _msg.right_hip = self.init_body_part_msg()
        _msg.right_knee = self.init_body_part_msg()
        _msg.right_ankle = self.init_body_part_msg()
        _msg.left_hip = self.init_body_part_msg()
        _msg.left_knee = self.init_body_part_msg()
        _msg.left_ankle = self.init_body_part_msg()
        _msg.right_eye = self.init_body_part_msg()
        _msg.left_eye = self.init_body_part_msg()
        _msg.right_ear = self.init_body_part_msg()
        _msg.left_ear = self.init_body_part_msg()
        return _msg

    def add_point_to_marker(self, body_part_msg):
        p = Point()
        p.x = float((body_part_msg.x / self.width) * self.point_range)
        p.y = float((body_part_msg.y / self.height) * self.point_range)
        p.z = 0.0
        return p

    def valid_marker_point(self, body_part_msg):
        if math.isnan(body_part_msg.x) or math.isnan(body_part_msg.y):
            return False
        return True

    def parse_k(self):
        image_idx = 0
        try:
            count = int(self.counts[image_idx])
            primary_msg = PersonDetection()  # BodypartDetection()
            primary_msg.num_people_detected = count
            for i in range(count):
                primary_msg.person_id = i
                primary_msg = self.init_all_body_msgs(_msg=primary_msg, count=i)
                marker_joints = self.init_markers_spheres()
                marker_skeleton = self.init_markers_lines()
                for k in range(18):
                    _idx = self.objects[image_idx, i, k]
                    if _idx >= 0:
                        _location = self.peaks[image_idx, k, _idx, :]
                        if k == 0:
                            primary_msg.nose = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.nose))
                            self.get_logger().info(
                                "Body Part Detected: nose at X:{}, Y:{}".format(primary_msg.nose.x, primary_msg.nose.y))
                        if k == 1:
                            primary_msg.left_eye = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_eye))
                            self.get_logger().info(
                                "Body Part Detected: left_eye at X:{}, Y:{}".format(primary_msg.left_eye.x,
                                                                                    primary_msg.left_eye.y))
                            if self.valid_marker_point(primary_msg.nose):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.nose))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_eye))

                        if k == 2:
                            primary_msg.right_eye = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_eye))
                            self.get_logger().info(
                                "Body Part Detected: right_eye at X:{}, Y:{}".format(primary_msg.right_eye.x,
                                                                                     primary_msg.right_eye.y))
                            if self.valid_marker_point(primary_msg.nose):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.nose))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_eye))
                            if self.valid_marker_point(primary_msg.left_eye):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_eye))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_eye))

                        if k == 3:
                            primary_msg.left_ear = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_ear))
                            self.get_logger().info(
                                "Body Part Detected: left_ear at X:{}, Y:{}".format(primary_msg.left_ear.x,
                                                                                    primary_msg.left_ear.y))
                            if self.valid_marker_point(primary_msg.left_eye):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_eye))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_ear))

                        if k == 4:
                            primary_msg.right_ear = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_ear))
                            self.get_logger().info(
                                "Body Part Detected: right_ear at X:{}, Y:{}".format(primary_msg.right_ear.x,
                                                                                     primary_msg.right_ear.y))

                            if self.valid_marker_point(primary_msg.right_eye):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_eye))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_ear))

                        if k == 5:
                            primary_msg.left_shoulder = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_shoulder))
                            self.get_logger().info(
                                "Body Part Detected: left_shoulder at X:{}, Y:{}".format(primary_msg.left_shoulder.x,
                                                                                         primary_msg.left_shoulder.y))
                            if self.valid_marker_point(primary_msg.left_ear):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_ear))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_shoulder))

                        if k == 6:
                            primary_msg.right_shoulder = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_shoulder))
                            self.get_logger().info(
                                "Body Part Detected: right_shoulder at X:{}, Y:{}".format(primary_msg.right_shoulder.x,
                                                                                          primary_msg.right_shoulder.y))

                            if self.valid_marker_point(primary_msg.right_ear):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_ear))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_shoulder))

                        if k == 7:
                            primary_msg.left_elbow = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_elbow))
                            self.get_logger().info(
                                "Body Part Detected: left_elbow at X:{}, Y:{}".format(primary_msg.left_elbow.x,
                                                                                      primary_msg.left_elbow.y))

                            if self.valid_marker_point(primary_msg.left_shoulder):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_shoulder))

                        if k == 8:
                            primary_msg.right_elbow = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_elbow))
                            self.get_logger().info(
                                "Body Part Detected: right_elbow at X:{}, Y:{}".format(primary_msg.right_elbow.x,
                                                                                       primary_msg.right_elbow.y))

                            if self.valid_marker_point(primary_msg.right_shoulder):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_shoulder))

                        if k == 9:
                            primary_msg.left_wrist = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_wrist))
                            self.get_logger().info(
                                "Body Part Detected: left_wrist at X:{}, Y:{}".format(primary_msg.left_wrist.x,
                                                                                      primary_msg.left_wrist.y))

                            if self.valid_marker_point(primary_msg.left_elbow):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_wrist))

                        if k == 10:
                            primary_msg.right_wrist = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_wrist))
                            self.get_logger().info(
                                "Body Part Detected: right_wrist at X:{}, Y:{}".format(primary_msg.right_wrist.x,
                                                                                       primary_msg.right_wrist.y))

                            if self.valid_marker_point(primary_msg.right_elbow):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_wrist))

                        if k == 11:
                            primary_msg.left_hip = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_hip))
                            self.get_logger().info(
                                "Body Part Detected: left_hip at X:{}, Y:{}".format(primary_msg.left_hip.x,
                                                                                    primary_msg.left_hip.y))

                        if k == 12:
                            primary_msg.right_hip = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_hip))
                            self.get_logger().info(
                                "Body Part Detected: right_hip at X:{}, Y:{}".format(primary_msg.right_hip.x,
                                                                                     primary_msg.right_hip.y))

                            if self.valid_marker_point(primary_msg.left_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_hip))

                        if k == 13:
                            primary_msg.left_knee = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_knee))
                            self.get_logger().info(
                                "Body Part Detected: left_knee at X:{}, Y:{}".format(primary_msg.left_knee.x,
                                                                                     primary_msg.left_knee.y))

                            if self.valid_marker_point(primary_msg.left_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_knee))

                        if k == 14:
                            primary_msg.right_knee = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_knee))
                            self.get_logger().info(
                                "Body Part Detected: right_knee at X:{}, Y:{}".format(primary_msg.right_knee.x,
                                                                                      primary_msg.right_knee.y))

                            if self.valid_marker_point(primary_msg.right_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_knee))

                        if k == 15:
                            primary_msg.left_ankle = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_ankle))
                            self.get_logger().info(
                                "Body Part Detected: left_ankle at X:{}, Y:{}".format(primary_msg.left_ankle.x,
                                                                                      primary_msg.left_ankle.y))

                            if self.valid_marker_point(primary_msg.left_knee):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_ankle))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_knee))
                        if k == 16:
                            primary_msg.right_ankle = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_ankle))
                            self.get_logger().info(
                                "Body Part Detected: right_ankle at X:{}, Y:{}".format(primary_msg.right_ankle.x,
                                                                                       primary_msg.right_ankle.y))

                            if self.valid_marker_point(primary_msg.right_knee):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_ankle))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_knee))

                        if k == 17:
                            primary_msg.neck = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.neck))
                            self.get_logger().info(
                                "Body Part Detected: neck at X:{}, Y:{}".format(primary_msg.neck.x, primary_msg.neck.y))

                            if self.valid_marker_point(primary_msg.nose):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.nose))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.neck))
                            if self.valid_marker_point(primary_msg.right_shoulder):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_shoulder))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.neck))
                            if self.valid_marker_point(primary_msg.right_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.neck))
                            if self.valid_marker_point(primary_msg.left_shoulder):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_shoulder))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.neck))
                            if self.valid_marker_point(primary_msg.left_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.neck))

                        self.publish_pose.publish(primary_msg)
                        self.body_skeleton_pub.publish(marker_skeleton)
                        self.body_joints_pub.publish(marker_joints)

                self.get_logger().info("Published Message for Person ID:{}".format(primary_msg.person_id))
        except:
            pass
