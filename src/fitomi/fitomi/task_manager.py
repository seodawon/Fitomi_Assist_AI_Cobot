import rclpy
from rclpy.action import ActionServer
from custom_interfaces.action import TaskCommand
from custom_interfaces.msg import Yolov8Inference, InferenceResult
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import asyncio
import time
# import numpy as np
from collections import deque
from enum import Enum
# from dataclasses import dataclass, field
# from ament_index_python.packages import get_package_share_directory

from fitomi.motion import MotionController
import fitomi.tts as tts

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY, ACC = 60, 60
DEGREE_VELOCITY = 400
DEGREE_ACC = 400

OFF = 0
ON = 1

READY_POS = [0, 0, 90, 0, 90, 0]

MAX_RETRY = 2  # 재시도 횟수

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

food_keywords = ['사과', '바나나', '견과류', '골뱅이', '밥']
cloth_keywords = ['반팔', '반바지', '긴바지', '가디건', '바람막이']
# 한글 -> 영어
mapping = {'사과': 'apple',
           '바나나': 'banana',
           '견과류': 'almond',
           '골뱅이': 'whelk',
           '밥': 'rice',
           '가디건': 'brown_jacket',
           '긴바지': 'checked_pants',
           '반바지': 'gray_shorts',
           '반팔': 'printed_tee',
           '바람막이': 'white_jumper',
           '옷걸이': 'hanger'}
# 영어 -> 한글
unmapping = {v: k for k, v in mapping.items()} 

hangers = {
    0: [-404.797, -754.269, 746.592, 82.971, -77.491, -91.427],
    1: [18.142, -751.166, 737.534, 82.207, -74.817, -86.591],
    2: [460.775, -738.258, 740.923, 99.756, -71.797, -89.789],
    3: [210.736, -712.788, 372.671, 89.895, -107.016, -89.301],
    4: [-195.672, -711.136, 389.591, 89.639, -102.053, -88.396]
}

class RobotState(Enum):
    IDLE = 0
    BUSY = 1
    ERROR = 2

class TaskManager:
    def __init__(self, node):
        self.node = node
        self.task_queue = deque()
        self.rc = MotionController(node)

        self.state = RobotState.IDLE

        self._action_server = ActionServer(
            self.node, TaskCommand, '/task_command', self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.sub = self.node.create_subscription(
            Yolov8Inference, '/Yolov8_Inference', self.coordinates_callback, 10
        )

        self.latest_detection = {}
        self.c_coords = {}
        self.f_coords = {}
        self.scan_list = set()

    def alert_to_user(self, sentence:str):
        print(sentence)
        tts.speak(sentence)

    def coordinates_callback(self, msg: Yolov8Inference):
        self.latest_detection = {}
        for inference in msg.yolov8_inference:
            class_name = inference.class_name
            coordinates = list(inference.coordinates)
            confidence = inference.confidence

            # if coordinates[0] == 0:
            #     continue

            if unmapping[class_name] in food_keywords:
                if class_name in self.f_coords:
                    stored_conf, _ = self.f_coords[class_name]
                    if confidence > stored_conf:
                        self.scan_list.add(unmapping[class_name])
                        self.f_coords[class_name] = (confidence, coordinates)
                else:
                    self.scan_list.add(unmapping[class_name])
                    self.f_coords[class_name] = (confidence, coordinates)

            elif unmapping[class_name] in cloth_keywords:
                if class_name in self.c_coords:
                    stored_conf, _ = self.c_coords[class_name]
                    if confidence > stored_conf:
                        self.scan_list.add(unmapping[class_name])
                        self.c_coords[class_name] = (confidence, coordinates)
                else:
                    self.scan_list.add(unmapping[class_name])
                    self.c_coords[class_name] = (confidence, coordinates)

            if unmapping[class_name] in food_keywords:
                self.latest_detection[class_name] = (confidence, coordinates)

    def execute_callback(self, goal_handle):
        if self.state == RobotState.ERROR:
            tts.speak("로봇이 에러 상태입니다. 작업을 수행할 수 없습니다.")
            self.node.get_logger().warn("🦾 <(로봇이 현재 에러 상태이므로 새로운 작업을 수행할 수 없습니다.)")
            goal_handle.abort()
            return TaskCommand.Result() 

        if self.state == RobotState.BUSY:
            tts.speak("로봇이 이미 작업 중입니다. 잠시 후 다시 시도해주세요.")
            self.node.get_logger().warn("🦾 <(로봇이 현재 작업 중이므로 새로운 작업을 수행할 수 없습니다.)")
            goal_handle.abort()
            return TaskCommand.Result() 

        self.node.get_logger().info("🦾 <(로봇이 작업을 시작하여 <BUSY> 상태로 전환되었습니다.)")
        self.state = RobotState.BUSY

        scenario_type = goal_handle.request.keyword_type
        targets = goal_handle.request.keyword
        feedback_msg = TaskCommand.Feedback()
        feedback_msg.message = f"'{scenario_type}' 작업을 시작합니다."
        goal_handle.publish_feedback(feedback_msg)

        result = TaskCommand.Result()

        try:
            success, scan_list = self.step(scenario_type, targets)
            print(f"@@@@@ {success}, {scan_list}")
            time.sleep(0.3)
            if len(scan_list) == 0:
                self.alert_to_user(f"작업을 완료했습니다.") 
            else:
                self.alert_to_user(f"작업을 완료했습니다. 작업 과정에서 {scan_list}의 정보가 새로 갱신되었습니다.")

            if success:
                result.success = True
                result.scan_list = list(scan_list)
                goal_handle.succeed()
                return result
            else:
                goal_handle.abort()
                return TaskCommand.Result() 

        except Exception as e:
            tts.speak("작업 중 오류가 발생했습니다.")
            self.node.get_logger().error(f"작업 실패: {e}")
            goal_handle.abort()
            return TaskCommand.Result() 

        finally:
            self.node.get_logger().info("🦾🤠 <(작업이 종료되어 <IDLE> 상태로 전환되었습니다.)")
            self.state = RobotState.IDLE
            
    def re_detection(self, current_detected, targets):
        for name in list(self.f_coords.keys()):
            if name not in current_detected:
                print("삭제:", name)
                del self.f_coords[name]

        for target in targets:
            _target = mapping[target]
            if _target in self.latest_detection:
                print("기존:", _target, self.f_coords[_target])
                print("갱신:", _target, self.latest_detection[_target])
                conf, coords = self.latest_detection[_target]
                self.f_coords[_target] = (conf, coords)

    def step(self, scenario_type, targets):
        self.scan_list = set()
        print("scenario_type:", scenario_type, type(scenario_type))
        if not targets:
            if scenario_type == "냉장고":
                self.f_coords = {}
                self.rc.init_pos()
                self.rc.env_scan('refrigerator_scan')
                return True, self.scan_list
            elif scenario_type == "옷장":
                self.c_coords = {}
                self.rc.init_pos()
                self.rc.env_scan('cloth_scan')
                return True, self.scan_list
            else:
                self.node.get_logger().error(f"알 수 없는 시나리오: {scenario_type}")
                return False, set()

        current_detected = set(self.latest_detection.keys())

        if scenario_type == "냉장고":
            self.re_detection(current_detected, targets)
            self.rc.env_scan('refrigerator_scan')

            for target in targets:
                _target = mapping[target]
                target_info = self.f_coords.get(_target)

                if not target_info:
                    self.alert_to_user("🦾🤠 <(물체를 탐지하지 못했습니다. 행동 명령을 건너뛰겠습니다.)")
                    continue
                elif target_info[-1][0] == 0:
                    self.alert_to_user("🦾🤠 <(올바른 좌표를 수신하지 못했습니다. 행동 명령을 건너뛰겠습니다.)")
                    continue
                success = self.rc.detected_pick_and_place(list(target_info[-1]))
                if not success:
                    self.alert_to_user(f"{target} 을(를) 잡지 못했습니다. 다시 시도하겠습니다.")
                    current_detected = set(self.latest_detection.keys())
                    print("current_detected", current_detected)
                    self.re_detection(current_detected, targets)
                    _target = mapping[target]
                    target_info = self.f_coords.get(_target)
                    self.rc.detected_pick_and_place(list(target_info[-1]))
                    self.rc.init_pos()
                    return False, self.scan_list
            return True, self.scan_list
    
        elif scenario_type == "옷장":
            for target in targets:
                _target = mapping[target]
                target_info = self.c_coords.get(_target)
                if not target_info:
                    self.alert_to_user("🦾🤠 <(물체를 탐지하지 못했습니다. 행동 명령을 건너뛰겠습니다.)")
                    return False, self.scan_list
                elif target_info[-1][0] == 0:
                    self.alert_to_user("🦾🤠 <(올바른 좌표를 수신하지 못했습니다. 행동 명령을 건너뛰겠습니다.)")
                    return False, self.scan_list
                closest_hanger_coords = self.get_closest_hanger_coords(list(target_info[-1]))
                success = self.rc.place_cloth(closest_hanger_coords)
                if not success:
                    self.alert_to_user(f"{target} 잡지 못했습니다. 다시 시도하겠습니다.")
                    self.rc.place_cloth(closest_hanger_coords)
                    return False, self.scan_list
            return True, self.scan_list
            
        else:
            self.node.get_logger().error(f"알 수 없는 시나리오: {scenario_type}")
            return False, set()

    def get_closest_hanger_coords(self, target_coords):
        min_dist = float("inf")
        closest_idx = 0
        x1, y1, z1 = target_coords[0], target_coords[1], target_coords[2]

        for idx, h_coords in hangers.items():
            if (z1 > 400 and idx in [3, 4]) or (z1 <= 400 and idx in [0, 1, 2]):
                continue
            x2, y2 = h_coords[0], h_coords[1]
            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if distance < min_dist:
                min_dist = distance
                closest_idx = idx
        return hangers[closest_idx]

def main(args=None):
    # rclpy.init(args=args)
    node = rclpy.create_node('task_server', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    tm = TaskManager(node)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()


# 물품 2개 줬을 때 잡을지 말지 결정을 첫 번째 물체를 기준으로 결정
# STT a