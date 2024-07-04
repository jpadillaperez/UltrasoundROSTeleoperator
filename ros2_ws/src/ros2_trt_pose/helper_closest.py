# ROS2 related
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from pose_msgs.msg import BodypartDetection, PersonDetection
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
        self.camera_info_depth = None
        self.model_trt = None
        self.annotated_image = None
        self.counts = None
        self.peaks = None
        self.objects = None
        self.topology = None
        self.xy_circles = []
        self.p = None
        self.rgb_image_ready = False  # Add this attribute to track RGB image readiness

        # ROS2 parameters
        self.declare_parameter('base_dir', '/home/victorkawai/trt_pose/tasks/human_pose')
        self.declare_parameter('model', 'resnet18')
        self.declare_parameter('point_range', 10)
        self.declare_parameter('show_image', False)
        self.base_dir = self.get_parameter('base_dir')._value
        self.model_name = self.get_parameter('model')._value
        self.point_range = self.get_parameter('point_range')._value
        self.show_image_param = self.get_parameter('show_image')._value

        # ROS2 related init
        self.rgb_subscriber = self.create_subscription(ImageMsg, '/rgb/image_raw', self.read_cam_callback, 10)
        self.depth_subscriber = self.create_subscription(ImageMsg, '/depth/image_raw', self.depth_callback, 10)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, '/rgb/camera_info', self.camera_info_callback, 10)
        self.camera_info_depth_subscriber = self.create_subscription(CameraInfo, '/depth/camera_info', self.camera_info_depth_callback, 10)
        self.image_pub = self.create_publisher(ImageMsg, 'detections_image', 10)
        self.body_joints_pub = self.create_publisher(Marker, 'body_joints', 1000)
        self.body_skeleton_pub = self.create_publisher(Marker, 'body_skeleton', 10)
        self.publish_pose = self.create_publisher(PersonDetection, 'pose_msgs', 100)
        self.bridge = CvBridge()
        self.target_height = 480
        self.target_width = 640

    def start(self):
        json_file = os.path.join(self.base_dir, 'human_pose.json')
        self.get_logger().info("Loading model weights\n")
        self.num_parts, self.num_links, self.model_weights, self.parse_objects, self.topology = load_params(
            base_dir=self.base_dir,
            human_pose_json=json_file,
            model_name=self.model_name)
        self.model_trt, self.height, self.width = load_model(
            base_dir=self.base_dir,
            model_name=self.model_name,
            num_parts=self.num_parts,
            num_links=self.num_links,
            model_weights=self.model_weights)
        self.get_logger().info("Model weights loaded...\n Waiting for images...\n")

    def execute(self):
        data = preprocess(image=self.image, width=self.width, height=self.height)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        self.counts, self.objects, self.peaks = self.parse_objects(cmap, paf)
        annotated_image = draw_objects(image=self.image, object_counts=self.counts, objects=self.objects, normalized_peaks=self.peaks, topology=self.topology)
        self.get_logger().info('Pose estimation executed and keypoints parsed')
        self.parse_k()
        return annotated_image

    def read_cam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            resized_image = cv2.resize(cv_image, (self.target_width, self.target_height))
            self.image = resized_image
            self.annotated_image = self.execute()
            image_msg = self.image_np_to_image_msg(self.annotated_image)
            self.image_pub.publish(image_msg)

            if self.show_image_param:
                cv2.imshow('frame', self.annotated_image)
                cv2.waitKey(1)

            # Indicate that the RGB image is ready
            self.rgb_image_ready = True
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def depth_callback(self, msg):
        try:
            if not self.rgb_image_ready:
                # Skip processing if the RGB image is not ready
                return
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.array(self.depth_image, dtype=np.float32)
            min_depth = np.min(depth_array)
            max_depth = np.max(depth_array)
            # Print depth range for debugging
            self.get_logger().info(f'Depth Image Updated: Min Depth: {min_depth}, Max Depth: {max_depth}')
            self.depth_image = depth_array
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def camera_info_depth_callback(self, msg):
        self.camera_info_depth = msg

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
        bodypart.z = float('NaN')
        bodypart.confidence = float('NaN')
        return bodypart

    def write_body_part_msg(self, pixel_location):
        body_part_pixel_loc = BodypartDetection()
        u = float(pixel_location[1] * self.width)
        v = float(pixel_location[0] * self.height)
        body_part_pixel_loc.x = u
        body_part_pixel_loc.y = v
        if self.depth_image is not None and 0 <= v < self.depth_image.shape[0] and 0 <= u < self.depth_image.shape[1]:
            depth_value_mm = self.depth_image[int(v), int(u)]
            body_part_pixel_loc.z = float(depth_value_mm / 1000.0)  # Convert mm to meters
            self.get_logger().info(f'Depth value at ({u:.2f}, {v:.2f}): {body_part_pixel_loc.z:.2f} m')
        else:
            body_part_pixel_loc.z = float('NaN')
            self.get_logger().info(f'Invalid depth value at ({u:.2f}, {v:.2f})')
        return body_part_pixel_loc


    def convert_to_3d(self, keypoints_2d):
        keypoints_3d = []
        if self.depth_image is None or self.camera_info is None:
            self.get_logger().error('Depth image or camera info not available')
            return keypoints_3d
        for (u, v) in keypoints_2d:
            z = float(self.depth_image[int(v), int(u)] / 1000.0)  # Convert mm to meters
            x = float((u - self.camera_info.k[2]) * z / self.camera_info.k[0])
            y = float((v - self.camera_info.k[5]) * z / self.camera_info.k[4])
            keypoints_3d.append((x, y, z))
            self.get_logger().info(f'3D keypoint: ({x:.2f}, {y:.2f}, {z:.2f})')
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
        p.z = body_part_msg.z
        return p

    def valid_marker_point(self, body_part_msg):
        if math.isnan(body_part_msg.x) or math.isnan(body_part_msg.y):
            return False
        return True

    def parse_k(self):
        image_idx = 0
        try:
            count = int(self.counts[image_idx])
            self.get_logger().info(f'Number of people detected: {count}')
            primary_msg = PersonDetection()
            primary_msg.num_people_detected = count
            for i in range(count):
                primary_msg.person_id = i
                primary_msg = self.init_all_body_msgs(_msg=primary_msg, count=i)
                marker_joints = self.init_markers_spheres()
                marker_skeleton = self.init_markers_lines()
                for k in range(18):
                    _idx = self.objects[image_idx, i, k]
                    self.get_logger().info(f'Processing keypoint {k} for person {i}, index {_idx}')
                    if _idx >= 0:
                        _location = self.peaks[image_idx, k, _idx, :].numpy()
                        self.get_logger().info(f'Keypoint {k} location: {_location}')
                        if k == 0:
                            primary_msg.nose = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.nose))
                            self.get_logger().info(
                                f"Body Part Detected: nose at X:{primary_msg.nose.x}, Y:{primary_msg.nose.y}, Z:{primary_msg.nose.z}")
                        elif k == 1:
                            primary_msg.left_eye = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_eye))
                            self.get_logger().info(
                                f"Body Part Detected: left_eye at X:{primary_msg.left_eye.x}, Y:{primary_msg.left_eye.y}, Z:{primary_msg.left_eye.z}")
                            if self.valid_marker_point(primary_msg.nose):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.nose))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_eye))
                        elif k == 2:
                            primary_msg.right_eye = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_eye))
                            self.get_logger().info(
                                f"Body Part Detected: right_eye at X:{primary_msg.right_eye.x}, Y:{primary_msg.right_eye.y}, Z:{primary_msg.right_eye.z}")
                            if self.valid_marker_point(primary_msg.nose):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.nose))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_eye))
                            if self.valid_marker_point(primary_msg.left_eye):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_eye))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_eye))
                        elif k == 3:
                            primary_msg.left_ear = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_ear))
                            self.get_logger().info(
                                f"Body Part Detected: left_ear at X:{primary_msg.left_ear.x}, Y:{primary_msg.left_ear.y}, Z:{primary_msg.left_ear.z}")
                            if self.valid_marker_point(primary_msg.left_eye):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_eye))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_ear))
                        elif k == 4:
                            primary_msg.right_ear = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_ear))
                            self.get_logger().info(
                                f"Body Part Detected: right_ear at X:{primary_msg.right_ear.x}, Y:{primary_msg.right_ear.y}, Z:{primary_msg.right_ear.z}")
                            if self.valid_marker_point(primary_msg.right_eye):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_eye))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_ear))
                        elif k == 5:
                            primary_msg.left_shoulder = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_shoulder))
                            self.get_logger().info(
                                f"Body Part Detected: left_shoulder at X:{primary_msg.left_shoulder.x}, Y:{primary_msg.left_shoulder.y}, Z:{primary_msg.left_shoulder.z}")
                            if self.valid_marker_point(primary_msg.left_ear):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_ear))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_shoulder))
                        elif k == 6:
                            primary_msg.right_shoulder = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_shoulder))
                            self.get_logger().info(
                                f"Body Part Detected: right_shoulder at X:{primary_msg.right_shoulder.x}, Y:{primary_msg.right_shoulder.y}, Z:{primary_msg.right_shoulder.z}")
                            if self.valid_marker_point(primary_msg.right_ear):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_ear))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_shoulder))
                        elif k == 7:
                            primary_msg.left_elbow = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_elbow))
                            self.get_logger().info(
                                f"Body Part Detected: left_elbow at X:{primary_msg.left_elbow.x}, Y:{primary_msg.left_elbow.y}, Z:{primary_msg.left_elbow.z}")
                            if self.valid_marker_point(primary_msg.left_shoulder):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_shoulder))
                        elif k == 8:
                            primary_msg.right_elbow = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_elbow))
                            self.get_logger().info(
                                f"Body Part Detected: right_elbow at X:{primary_msg.right_elbow.x}, Y:{primary_msg.right_elbow.y}, Z:{primary_msg.right_elbow.z}")
                            if self.valid_marker_point(primary_msg.right_shoulder):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_shoulder))
                        elif k == 9:
                            primary_msg.left_wrist = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_wrist))
                            self.get_logger().info(
                                f"Body Part Detected: left_wrist at X:{primary_msg.left_wrist.x}, Y:{primary_msg.left_wrist.y}, Z:{primary_msg.left_wrist.z}")
                            if self.valid_marker_point(primary_msg.left_elbow):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_wrist))
                        elif k == 10:
                            primary_msg.right_wrist = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_wrist))
                            self.get_logger().info(
                                f"Body Part Detected: right_wrist at X:{primary_msg.right_wrist.x}, Y:{primary_msg.right_wrist.y}, Z:{primary_msg.right_wrist.z}")
                            if self.valid_marker_point(primary_msg.right_elbow):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_elbow))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_wrist))
                        elif k == 11:
                            primary_msg.left_hip = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_hip))
                            self.get_logger().info(
                                f"Body Part Detected: left_hip at X:{primary_msg.left_hip.x}, Y:{primary_msg.left_hip.y}, Z:{primary_msg.left_hip.z}")
                        elif k == 12:
                            primary_msg.right_hip = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_hip))
                            self.get_logger().info(
                                f"Body Part Detected: right_hip at X:{primary_msg.right_hip.x}, Y:{primary_msg.right_hip.y}, Z:{primary_msg.right_hip.z}")
                            if self.valid_marker_point(primary_msg.left_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_hip))
                        elif k == 13:
                            primary_msg.left_knee = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_knee))
                            self.get_logger().info(
                                f"Body Part Detected: left_knee at X:{primary_msg.left_knee.x}, Y:{primary_msg.left_knee.y}, Z:{primary_msg.left_knee.z}")
                            if self.valid_marker_point(primary_msg.left_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_knee))
                        elif k == 14:
                            primary_msg.right_knee = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_knee))
                            self.get_logger().info(
                                f"Body Part Detected: right_knee at X:{primary_msg.right_knee.x}, Y:{primary_msg.right_knee.y}, Z:{primary_msg.right_knee.z}")
                            if self.valid_marker_point(primary_msg.right_hip):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_hip))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_knee))
                        elif k == 15:
                            primary_msg.left_ankle = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.left_ankle))
                            self.get_logger().info(
                                f"Body Part Detected: left_ankle at X:{primary_msg.left_ankle.x}, Y:{primary_msg.left_ankle.y}, Z:{primary_msg.left_ankle.z}")
                            if self.valid_marker_point(primary_msg.left_knee):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_ankle))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.left_knee))
                        elif k == 16:
                            primary_msg.right_ankle = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.right_ankle))
                            self.get_logger().info(
                                f"Body Part Detected: right_ankle at X:{primary_msg.right_ankle.x}, Y:{primary_msg.right_ankle.y}, Z:{primary_msg.right_ankle.z}")
                            if self.valid_marker_point(primary_msg.right_knee):
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_ankle))
                                marker_skeleton.points.append(self.add_point_to_marker(primary_msg.right_knee))
                        elif k == 17:
                            primary_msg.neck = self.write_body_part_msg(_location)
                            marker_joints.points.append(self.add_point_to_marker(primary_msg.neck))
                            self.get_logger().info(
                                f"Body Part Detected: neck at X:{primary_msg.neck.x}, Y:{primary_msg.neck.y}, Z:{primary_msg.neck.z}")
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
        except Exception as e:
            self.get_logger().error(f'Error parsing keypoints: {e}')
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TRTPose()
    node.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
