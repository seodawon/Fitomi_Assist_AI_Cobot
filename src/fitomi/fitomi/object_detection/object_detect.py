
import numpy as np
import rclpy
from rclpy.node import Node
import os
from pathlib import Path
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import math
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

import DR_init
import sys
import time

from custom_interfaces.msg import Yolov8Inference, InferenceResult

from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
# Constants
RATIO_THRESHOLD = 0.05   # 길이 비율 차이 (10%)
ABS_THRESHOLD = 1.0     # 길이 절댓값 차이 기준 (단위: mm) -> 실제 적용해보고 값 변경 필요
DEPTH_OFFSET = -5.0 #+ 일수록 z축 위로 올라감 # 23
MIN_DEPTH = 0.2 # 2.0
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 120

# Paths & ROS init
package_path = get_package_share_directory("fitomi")
bridge = CvBridge()

# Doosan Init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import get_current_posx, trans, movel,posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()


class ObjectDetectionNode(Node):
    def __init__(self, model):
        super().__init__('object_detection_node')
        self.model = model
        self.intrinsics = None
        self.group = ReentrantCallbackGroup()

        # self.gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
        self.gripper2cam_path = '/home/rokey/ros2_ws/src/DoosanBootcamp3rd/fitomi/fitomi/resource/T_gripper2camera.npy'
        # Subscribers (message_filters)
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)

        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10, callback_group= self.group)
        self.robot_pose = Float64MultiArray()
        self.create_subscription(Float64MultiArray, '/dsr01/msg/current_posx',self.update_robot_pose,10, callback_group= self.group)  # 10Hz
        # Publishers
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1,callback_group= self.group)
        self.img_pub = self.create_publisher(Image, "/send_img", 1,callback_group= self.group)

        self.classNames = model.names
        self.get_logger().info("ObjectDetectionNode with sync initialized.")

    def update_robot_pose(self,msg):
        try:
            self.robot_pose = msg
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot pose: {e}")

    def camera_info_callback(self, msg):
        self.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "ppx": msg.k[2],
            "ppy": msg.k[5]
        }

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path):
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous
        # x, y, z, rx, ry, rz = robot_pos
        # print("hi")
        base2gripper = self.get_robot_pose_matrix(*self.robot_pose.data)
        # print("hi1")

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def _pixel_to_camera_coords(self, x, y, z):
        if self.intrinsics is None:
            self.get_logger().warn("Camera intrinsics not yet received.")
            return [0.0, 0.0, 0.0]
        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        ppx, ppy = self.intrinsics['ppx'], self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )

    def sync_callback(self, color_msg, depth_msg):
        img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_frame = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        results = self.model(img, conf=0.70)
        yolov8_msg = Yolov8Inference()
        yolov8_msg.header.frame_id = "inference"
        yolov8_msg.header.stamp = self.get_clock().now().to_msg()
        height, width = img.shape[:2]
        line_y = int(height * 0.45)
        for r in results:
            if r.obb is None:
                self.get_logger().info("No detection results.")
                continue

            xywhr = r.obb.xywhr.cpu().numpy()
            confs = r.obb.conf.cpu().numpy()
            clss = r.obb.cls.cpu().numpy()

            for i in range(xywhr.shape[0]):
                inference_result = InferenceResult()

                x, y, h, w, angle = xywhr[i]
                cx, cy = x, y
                confidence = float(confs[i])
                cls = int(clss[i])
                label = self.classNames.get(cls, f"class_{cls}")
                print("box", cls, x, y, w, h, angle)
                # 회전 박스 → 사각형 좌표
                # rect = ((x, y), (w, h), angle * 180 / math.pi)
                # points = np.int0(cv2.boxPoints(rect))
                # cv2.drawContours(img, [points], 0, (0, 0, 255), 2)

                u, v = int(x), int(y)
                if v >= depth_frame.shape[0] or u >= depth_frame.shape[1]:
                    self.get_logger().warn(f"Coordinates out of range: ({u}, {v})")
                    continue

                depth = depth_frame[v, u]
                if depth == 0:
                    self.get_logger().warn(f"Invalid depth at ({u}, {v})")
                    continue

                camera_coords = self._pixel_to_camera_coords(u, v, depth)
                td_coord = self.transform_to_base(camera_coords, self.gripper2cam_path)

                if td_coord[2] and sum(td_coord) != 0:
                    td_coord[2] += DEPTH_OFFSET
                    td_coord[2] = max(td_coord[2], MIN_DEPTH)
                # @0618 수정
                # 회전 적용 여부
                # 가로와 세로 길이가 어느 정도 차이가 나는 경우에만 회전 적용
                should_rotate = abs(w - h) > ABS_THRESHOLD and (min(w, h) / max(w, h)) < (1 - RATIO_THRESHOLD)                
                if should_rotate:
                    # OBB의 angle = w가 x축 기준으로 반시계 방향으로 회전된 각도
                    # 즉, w 방향 = angle 방향
                    # # w가 더 짧은 경우
                    # 이미 angle이 짧은 쪽 기준 → 그리퍼를 angle에 맞춰 평행하도록 회전
                    # # w가 더 긴 경우
                    # angle이 긴 쪽 기준 → 그리퍼를 (pi/2) + angle에 맞춰 평행하도록 회전
                    angle = angle if w <= h else angle + math.pi / 2
                    # 정규화: [-180, 180] 범위로 보정
                    angle = (angle + math.pi) % (2 * math.pi) - math.pi
                    angle = math.degrees(angle)
                    # # else: 차이가 거의 없으면 angle 그대로 유지(angle = angle)
                target_pos = list(td_coord)
                # if cls == 2:
                # print('target_pos', target_pos)
                # else:
                # cls 번호를 기준으로 
                    # 0: almond
                    # 1: apple
                    # 2: banana
                    # 8: rice
                    # 9: whelk -> 해당 클래스 번호를 바탕으로 판단
                robot_pose = list(self.robot_pose.data)
                if cy < line_y: # 사과의 경우 위든 아래든 yaw값은 현재 그리퍼를 따라 가도록 함 
                    # 낼 직접 해보고 위 일때 아래일때 roll, pitch 값을 조절해야함
                    target_pos[1] = target_pos[1] + 60 # 150 limit 위쪽 냉장고 85~90 위쪽 # 회전할때 55 # 안할떄 65 #95
                    if cls == 1 or cls == 9: # 골뱅이와 사과는 회전 x
                        pose = target_pos+ robot_pose[3:]
                    else:     
                        # robot_pose[5] = robot_pose[5] + 20
                        pose = target_pos+ robot_pose[3:5] + [float(angle)] # 옷의 경우 이 좌표를 안쓰기 때문에 상관 없음
                        
                    position = "위" 
                elif cy > line_y:
                    target_pos[2] = target_pos[2] - 10
                    target_pos[1] = target_pos[1] + 35 
                    if cls == 1 or cls == 9: # 골뱅이와 사과는 회전 x
                        pose = target_pos+ robot_pose[3:]
                    else:
                        pose = target_pos+ list(self.robot_pose.data[3:5]) + [float(angle)] # 옷의 경우 이 좌표를 안쓰기 때문에 상관 없음
                    position = "아래" # 아래 영역에 바나나, 아몬드, 골뱅이, 사과 집을때 경우 세팅 roll, pitch,yaw
                print("after angle", angle)
                # print("현재 위치",self.robot_pose)
                print("갈 위치", pose, position)
                inference_result.class_name = label
                inference_result.coordinates = pose
                inference_result.confidence = confidence
                yolov8_msg.yolov8_inference.append(inference_result)
                # movel(posx(pose), vel=VELOCITY, acc= ACC)
                # exit(1)
        # Publish inference and image
        self.yolov8_pub.publish(yolov8_msg)

        annotated_img = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_img, "bgr8")
        self.img_pub.publish(img_msg)

   
def main(args=None):
    # rclpy.init(args=args)
    model_path = "/home/rokey/choi_ws/src/fitomi/resource/fffinal.pt"

    if not os.path.exists(model_path):
        print(f"Model not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"Unsupported model format: {suffix}")
        exit(1)

    # node = ObjectDetectionNode(model)
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    node = ObjectDetectionNode(model)
    execute = MultiThreadedExecutor(num_threads=3)
    execute.add_node(node)
    execute.spin()
    # rclpy.spin(node)
    execute.shutdown()
    node.destroy_node()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
