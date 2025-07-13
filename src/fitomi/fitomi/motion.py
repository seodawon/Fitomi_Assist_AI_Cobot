from fitomi.config import *
from fitomi.onrobot import RG

import json
import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray

import time
import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# start pose -> READY_POS
rclpy.init()
node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import (
    movej, movejx, movel, movec, move_periodic, amove_periodic,
    set_tool, set_tcp, set_ref_coord,mwait,
    set_digital_output,
    DR_TOOL,
    task_compliance_ctrl,
    set_desired_force,
    check_force_condition,
    DR_FC_MOD_REL, DR_FC_MOD_ABS,
    DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z,
    DR_BASE, DR_QSTOP,
    release_force,
    release_compliance_ctrl,
    movesx, movesj, moveb,
    wait,
    DR_MV_MOD_REL, DR_MVS_VEL_NONE,
    get_current_posx
)
from DR_common2 import posx, posj

class MotionController:
    def __init__(self,node):
        self.node = node
        self.pose_lastest = Float64MultiArray()
        # path = get_package_share_directory('fitomi')
        # json_path = os.path.join(path, 'resource', 'pose_data.json')
        self.current_posex_history = []
        self.record_flag = False
        self.last_record_time = time.time()
        self.pose_data = self.load_pose_data()

        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        self.node.create_subscription(
            Float64MultiArray,
            '/dsr01/msg/current_posx',
            self.update_robot_pose,
            10,
        )

        # 함수와 상수들 alias
        self.release_compliance_ctrl = release_compliance_ctrl
        self.release_force = release_force
        self.check_force_condition = check_force_condition
        self.task_compliance_ctrl = task_compliance_ctrl
        self.set_desired_force = set_desired_force
        self.set_digital_output = set_digital_output
        self.set_ref_coord = set_ref_coord
        self.wait = wait
        self.mwait = mwait
        self.movec = movec
        self.movel = movel
        self.movesx = movesx
        self.movesj = movesj
        self.amove_periodic = amove_periodic
        self.posx = posx
        self.posj = posj
        self.movej = movej
        self.movejx = movejx
        self.moveb = moveb
        self.move_periodic = move_periodic
        # self.grip = self._grip
        # self.release = self._release
        self.get_current_posx = get_current_posx

        self.DR_QSTOP = DR_QSTOP
        self.DR_BASE = DR_BASE
        self.DR_TOOL = DR_TOOL
        self.DR_AXIS_X = DR_AXIS_X
        self.DR_AXIS_Y = DR_AXIS_Y
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.DR_FC_MOD_REL = DR_FC_MOD_REL
        
        
    def update_robot_pose(self, msg):
        if self.record_flag:
            now = time.time()
            self.pose_lastest = msg.data
            # 1초마다 한 번만 기록 (4번까지만)
            if now - self.last_record_time >= 1.0 and len(self.current_posex_history) < 4:
                pos = list(msg.data)
                self.current_posex_history.append(pos)
                self.last_record_time = now
                self.node.get_logger().info(f"[POSE RECORD] {len(self.current_posex_history)}: {pos}")

    def init_pos(self):
        self.movej(READY_POS, vel=VELOCITY, acc=ACC)
        self.mwait(0)
        self._release()

    def load_pose_data(self, filepath = "/home/rokey/choi_ws/src/fitomi/resource/pose_data.json"):
        with open(filepath, 'r') as f:
            return json.load(f)
        
    def get_env_scan_group(self,task_name):
        try:
            pose_dict = self.pose_data['env_scan'][task_name]['posj']
        except KeyError:
            self.node.get_logger().error(f"[POSE ERROR] {task_name}/posj list not found.")
            raise
        # case : replace to movesj -> change  
        # return [self.posx(pose_dict[k]) if pose_type == "posx" else self.posj(pose_dict[k]) for k in pose_dict.keys()]
        result = []
        for k in pose_dict.keys(): #poses[0] : poses가 "여러 포즈를 담은 리스트"인지, 아니면 "단일 포즈"인지 판별
            poses = pose_dict[k]
            if isinstance(poses[0], list):  # 여러 포즈가 있는 경우 -> movesj
                print("hi")
                result.extend([[self.posj(p) for p in poses]])
            else:  # 단일 포즈 -> movesj 
                result.append(self.posj(poses))
            print("poses",type(result[0]))
        # many posej save
        return result
    
    # this intermediate point(경로 중간에 있는 지점) to capture detect_class and detect_point in perception manager
    def env_scan(self, env_scan_type): # env_scan_type : refrigerator_scan, cloth_scan
        # self.init_pos()
        # ex) cloth_scan : [ [self.posj(p1),self.posj(p2),self.posj(p3)],[~] ]
        poses_list = self.get_env_scan_group(env_scan_type) # pose value : 1unit(refrigerator_scan) or 5unit(cloth_scan)
        # print(poses_list)
        for pose in poses_list: # poses sequence
            self.movesj(pose, vel=VELOCITY, acc=ACC) # replace to movesj? -> test
            self.mwait(0)
            # self.wait(1.5) # replace to time.sleep? -> test!

    def uni_cloth_path(self,select_cloth): # 단일 경로 pick
        self.init_pos()
        pose_dict = self.pose_data['env_scan']['cloth_scan']['posj'][select_cloth]
        self.movesj([self.posj(p) for p in pose_dict], vel=VELOCITY, acc=ACC)
        self.mwait(0)
        # self._grip()

    # @ 옷 pick and place 함수
    def place_cloth(self,coords):
        place_cloth = self.pose_data['cloth_point'][str(coords)]
        self.uni_cloth_path(place_cloth)
        self.movel(posx(coords), vel=VELOCITY, acc=ACC)
        self.mwait(0)
        self._grip()
        pose_dict1 = self.pose_data['place_cloth']['posj'][place_cloth]
        self.movesj([self.posj(p) for p in pose_dict1], vel=VELOCITY, acc=ACC)
        self.mwait(0)
        # 물건 제대로 집었는지 확인
        if not self.check_pick_state():
            self._release()
            return False
        self._release()

        return True

    # @ 냉장고 pick and place 함수
    def detected_pick_and_place(self, coords):
        # current_pos = self.get_current_posx()[0] # after change -> obb : rotation data\
        poses = posx(coords)
        self.current_posex_history = []  # 초기화
        self.record_flag = True
        self.last_record_time = time.time()  # 현재 시간 기록 -> 조금 빠르게 기록
        self.movel(poses, vel=VELOCITY, acc=ACC)
        self.mwait(0)
        # last_pose = self.pose_lastest
        #  자세 기록 시작
        self._grip() # pick
        
        # 물건 제대로 집었는지 확인
        if not self.check_pick_state():
            self._release() # place
            self.record_flag = False
            if not self.current_posex_history:
                self.node.get_logger().error("No recorded posex data available.")
                return

            for i in range(len(self.current_posex_history)):
                self.current_posex_history[i][2] += 100.0 # posex의 z 값 +3cm 안부딪히게 하기 위해
                self.current_posex_history[i][5] =  self.pose_lastest[5]
            reverse_pose = reversed(self.current_posex_history)
            self.movesx([self.posx(p) for p in reverse_pose],vel=VELOCITY, acc=ACC) # posx 값 4개 집어 넣음
            return False
        
        self.record_flag = False
        if not self.current_posex_history:
            self.node.get_logger().error("No recorded posex data available.")
            return

        for i in range(4):
            self.current_posex_history[i][2] += 100.0 # posex의 z 값 +3cm 안부딪히게 하기 위해
            self.current_posex_history[i][5] =  self.pose_lastest[5]

        reverse_pose = reversed(self.current_posex_history.copy())
        self.movesx([self.posx(p) for p in reverse_pose],vel=100, acc=100) # posx 값 4개 집어 넣음
        
        # self.movej(posj([17.191, 28.033, 26.879, 60.042, 90.167, coords[5]-20]), vel=VELOCITY, acc=ACC) # 위 중간
        # self.mwait(0)
        # posj([-20.838, 29.506, 83.054, 41.63, 88.66, coords[5]-20]
        # place position self.current_posex_history[-1]
        self.movel(posx([471.276, 30.73, 159.877, 88.722, 108.764, 91.789]), vel=VELOCITY, acc=ACC) # 놓는 위치
        self.mwait(0)
        self._release() # place
        self.init_pos()
        return True

    def _grip(self):
        self.gripper.close_gripper()
        while self.gripper.get_status()[0]: # 1: busy gripper, 0: not busy gripper
            time.sleep(0.1) # 0.5
        return self.gripper.get_width()
        # not grap object -> excepttion handling add

    def _release(self):
        self.gripper.open_gripper()
        while self.gripper.get_status()[0]:
            time.sleep(0.1)

    def check_pick_state(self): # 잡히면 True 안잡히면 False
        width = self.gripper.get_width()
        if width > 11:
            return True # 물건을 잡은 경우
        return False

####################################################################################

def main():

    test = MotionController(node)
    test._release()
    # test._grip()
#     # print(test.gripper.get_width())
#     # test.env_scan("refrigerator_scan") # scan refrigerator_scan or cloth_scan
#     # test.movel(posx([522.6583, 555.1196, 479.93713, 88.18334, 118.3691, 179.9020]), vel=VELOCITY, acc=ACC)
#     # test.place_cloth([-195.672, -711.136, 389.591, 89.639, -102.053, -88.396])
#     # test.init_pos()
#     # test.uni_cloth_path('cloth_point2') # cloth_point 1~5
#     # test.env_scan("cloth_scan") # scan refrigerator_scan or cloth_scan
#     # test.init_pos()
#     # movel([415.655, 142.952, 439.285, 78.183, 108.369, 41.796], vel=VELOCITY, acc=ACC)
#     # pose_dict = test.pose_data['env_scan']["refrigerator_scan"]['posj']["refrigerator_path"]
#     # print(pose_dict)
#     # test.movesj([test.posj(p) for p in reversed(pose_dict)], vel=VELOCITY, acc=ACC)
#     # node.destroy_node()

if __name__ == "__main__":
    main()
    rclpy.shutdown()