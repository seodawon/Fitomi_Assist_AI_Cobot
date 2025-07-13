import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from custom_interfaces.action import TaskCommand
from custom_interfaces.srv import LogMessage, ImgLoad

import cv2
from cv_bridge import CvBridge
import time

from fitomi.llm_query import LLMQuery
from fitomi.prompt_manager import *
from fitomi.knowledge_utils import *
from fitomi.vad_audio import Audio_record, Cumtom_faster_whisper
from fitomi.wakeup_word import WakeupWord
from fitomi.MicController import MicController
import fitomi.tts as tts
from fitomi.config import *
from langchain.chat_models import ChatOpenAI
import openai
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile

# 사용자 음성 명령으로부터 키워드 추출하기 위한 단어 리스트
food_keywords = ['사과', '바나나', '견과류', '골뱅이', '밥', '쌀밥', '먹을', '음식']
cloth_keywords = ['반팔', '반바지', '긴바지', '가디건', '바람막이', '옷', '옷장', '입을']


class ContextManagerNode(Node):
    def __init__(self):
        super().__init__('context_manager')
        self.get_logger().info("ContextManagerNode initialized!")
        self._action_client = ActionClient(self, TaskCommand, '/task_command')
        self.group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.detections = []

        self.openai_api_key = keys.get("OPENAI_API_KEY")
        self.duration = 5  # seconds
        self.samplerate = 16000  # Whisper는 16kHz를 선호
    # --------------------------------------------------------------------------------

    def alert_to_user(self, sentence:str):
        tts.speak(sentence)         # 음성 알림
        self.send_log(sentence)     # UI 로그 출력
        time.sleep(1.0)

    def send_log(self, message: str):
        """Flask 서버에 로그 메시지 전송"""
        client = self.create_client(LogMessage, '/log_message', callback_group=self.group)
        # 서비스 대기
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Flask Service...")

        req = LogMessage.Request()
        req.log = message
        future = client.call_async(req)  # 비동기 호출 생성
        rclpy.spin_until_future_complete(self, future)  # future 완료를 기다리는 역할
        if future.result() and future.result().answer:
            self.get_logger().info(f"로그 전송 성공: {future.result().message}")
        else:
            self.get_logger().warn(f"로그 전송 실패")
        
        # ---
        # @0616: Dummy for CM test
        # self.get_logger().warn(f"message: {message}")

    def request_image_from_server(self):
        """서버에서 이미지 요청 및 반환"""
        client = self.create_client(ImgLoad, '/img_load', callback_group=self.group)
        # 서비스 대기
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Flask Service...")

        req = ImgLoad.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            img_msg = future.result().img
            self.get_logger().info("이미지 응답 수신 성공")
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            save_path = "/home/rokey/choi_ws/src/fitomi/resource/user_from_server.jpg"
            cv2.imwrite(save_path, cv_img)
            return save_path
        else:
            self.get_logger().error("이미지 응답 실패")
            return None
        
        # ---
        # @0616: Dummy for CM test
        # save_path = "/home/choi/choi_ws/src/fitomi/resource/shorts.jpg"
        # return save_path

    # --------------------------------------------------------------------------------
    # TaskManager와 action 통신
    def send_task_keywords(self, source: str, keywords: list[str]):
        """TaskManager에 작업 전송"""
        # 서비스 대기
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("TaskManager Action Server 대기 중...")

        goal = TaskCommand.Goal()
        goal.keyword_type = source  # '옷장' 또는 '냉장고'
        goal.keyword = keywords     #  타겟 물체들

        self.get_logger().info(f"{keywords}와 함께 Task 전송: {goal.keyword_type}")
        
        future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

        # ---
        # @0616: Dummy for CM test
        # self.get_logger().info(f"{source} 시나리오: {keywords}와 함께 작업 전송")

    def feedback_callback(self, feedback):
        self.get_logger().info(f"피드백 수신: {feedback.feedback.message}")

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle:
                self.get_logger().info(f"goal_handle: {goal_handle}")
                
                # 비동기적으로 결과를 기다림
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.process_goal_result)
            else:
                self.get_logger().warn("Goal 처리가 실패했습니다.")
        except Exception as e:
            self.get_log

    # @0618 수정
    def process_goal_result(self, future):
        try:
            goal_handle = future.result()
            if goal_handle:
                res = goal_handle.result
                self.get_logger().info(f"res: {res}")
                if res.success:
                    self.get_logger().info(f"Task 처리 성공: {res.scan_list}")
                    self.detections = res.scan_list 
                    # print(f"self.detections: {self.detections}, type: {type(self.detections)}")
                    # print(f"len: {len(self.detections)}")
                    # if not len(self.detections):
                    #     self.alert_to_user(f"작업을 완료했습니다. 작업 과정에서 {self.detections}의 정보가 새로 갱신되었습니다.")
                    # else:
                    #     self.alert_to_user(f"작업을 완료했습니다.")
                else:
                    self.get_logger().warn("Task 처리 실패")
            else:
                self.get_logger().warn("Goal 처리 실패")
        except Exception as e:
            self.get_logger().error(f"Goal 결과 처리 중 오류: {e}")

    def extract_keywords(self, command: str):
        """명령에서 음식 또는 옷 관련 키워드 추출"""
        keywords = []
        if any(food in command for food in food_keywords):
            keywords += [food for food in food_keywords if food in command]
        elif any(cloth in command for cloth in cloth_keywords):
            keywords += [cloth for cloth in cloth_keywords if cloth in command]
        return keywords

    def classify_command(self, command: str):
        """명령 분류 및 작업 결정"""
        if any(keyword in command for keyword in ["스캔", "뭐있어", "있는 것", "둘러", "뭐 있어"]):
            if "냉장고" in command:
                return "스캔", "냉장고", []
            elif "옷장" in command:
                return "스캔", "옷장", []
            else:
                return "스캔", "냉장고", []

        elif any(keyword in command for keyword in ["갖다", "가져", "가져와", "전달"]):
            keywords = self.extract_keywords(command)
            if keywords:
                if any(food in keywords for food in food_keywords):
                    return "전달", "냉장고", keywords
                elif any(cloth in keywords for cloth in cloth_keywords):
                    return "전달", "옷장", keywords
            return "전달", "냉장고", []

        elif any(keyword in command for keyword in ["추천", "뭐입지", "뭐먹지"]):
            keywords = self.extract_keywords(command)
            if keywords:
                if any(food in keywords for food in food_keywords):
                    return "추천", "냉장고", keywords
                elif any(cloth in keywords for cloth in cloth_keywords):
                    return "추천", "옷장", keywords
            return "추천", "냉장고", []

        else:
            return "기타", "냉장고", []

    # --------------------------------------------------------------------------------

    def prompting(self, command: str):
        """명령 처리 및 작업 전달"""
        self.get_logger().info(f"입력 문장: {command}")
        
        command_type, scenario_type, extracted_keywords = self.classify_command(command)
        self.get_logger().info(f"사용자 명령: {command_type}, 시나리오 타입: {scenario_type}")

        if command_type == "스캔":
            self.get_logger().info(f"[{scenario_type}] 스캔 명령 인식")
            answer = f"{add_particles_to_list([scenario_type])} 스캔하겠습니다."
            self.alert_to_user(answer)
            self.send_task_keywords(scenario_type, [])
            return

        elif command_type == "전달":
            self.get_logger().info(f"{scenario_type}에서 가져올 항목: {extracted_keywords}")
            answer = f"네, {scenario_type}에서 {add_particles_to_list(extracted_keywords)} 가져오겠습니다."
            self.alert_to_user(answer)
            self.send_task_keywords(scenario_type, extracted_keywords)
            return

        elif command_type == "추천":
            self.get_logger().info(f"{scenario_type}에서 추천할 항목: {extracted_keywords}")
            scenario, system_prompt = get_prompt_by_command(command)
            self.get_logger().info(f"시나리오 종류: {scenario}")
            # self.get_logger().info(f"system_prompt: {system_prompt}")

            if scenario == "옷장":
                img_path = self.request_image_from_server()
                response, keywords = self.prompting_closet(command, system_prompt, img_path)
            elif scenario == "냉장고":
                response, keywords = self.prompting_fridge(command, system_prompt)

            if scenario == "냉장고":
                self.get_logger().info(f"LLM 응답: {response}")
                self.alert_to_user(response)

            self.send_task_keywords(scenario_type, keywords)
            return
        else:
            self.get_logger().info(f"추천 명령어가 아닌 일반 질의 명령어 처리")

    def prompting_fridge(self, command: str, system_prompt: str):
        """냉장고 관련 LLM 질의 처리"""
        context = {}
        context.update(get_fridge_caption())
        # DB 다 쓰기엔 너무 커서 에러 남 -> 어떻게 뽑아서 쓸까
        # context.update(get_nutrition_context_docs(type="food", query=command))
        # context.update(get_nutrition_context_docs(type="processed_food", query=command))

        # food_context = get_nutrition_context_docs(type="food", query=command)
        # processed_food_context = get_nutrition_context_docs(type="processed_food", query=command)
        # if "nutrition_context" in context:
        #     context["nutrition_context"] += "\n" + food_context.get("nutrition_context", "") + "\n" + processed_food_context.get("nutrition_context", "")
        # else:
        #     context["nutrition_context"] = food_context.get("nutrition_context", "") + "\n" + processed_food_context.get("nutrition_context", "")

        llm = LLMQuery(system_prompt)
        response = llm.query(command, context)
        print(response)
        # 문장 변환
        response_sentense = generate_sentence_message(
            scenario_type="냉장고", 
            response=response,
            additional_data=None)

        # 추천 키워드 추출3) SAFE_OFF

        recommend_foods = response.split('/')[-1]
        keywords = [item.strip() for item in recommend_foods.split(',')]
        return response_sentense, keywords

    # def prompting_closet(self, command: str, system_prompt: str, user_image: str):
    #     """옷장 관련 LLM 질의 처리"""
    #     closest_paths, closet_caption = get_closet_caption(self.detections)
    #     closest_paths.insert(0, user_image)

    #     context = {}
    #     context.update(closet_caption)
    #     weather_info = get_weather_info()
    #     context.update(weather_info)
    #     context.update(get_fashion_context_docs())
        
    #     llm = LLMQuery(system_prompt)
    #     response = None
    #     response = llm.query_with_images(command=command, image_paths=closest_paths, context=context)
    #     print(response)
    #     # 문장 변환
    #     response_sentense = generate_sentence_message(
    #         scenario_type="옷장", 
    #         response=response,
    #         additional_data=weather_info)

    #     # 추천 키워드 추출
    #     # keywords = response.split('/')[-2:]
    #     time.sleep(15)
    #     keywords = self.get_user_choice() # 사용자 입력으로부터 키워드 재추출

    #     return response_sentense, keywords
    

    def prompting_closet(self, command: str, system_prompt: str, user_image: str):
        """옷장 관련 LLM 질의 처리"""
        # 1. 옷장 이미지 및 캡션 추출
        closest_paths, closet_caption = get_closet_caption(self.detections)
        closest_paths.insert(0, user_image)

        # 2. LLM context 구성
        context = {}
        context.update(closet_caption)
        weather_info = get_weather_info()
        context.update(weather_info)
        context.update(get_fashion_context_docs())

        # 3. LLM 질의
        llm = LLMQuery(system_prompt)
        response = llm.query_with_images(command=command, image_paths=closest_paths, context=context)
        print(response)

        # 4. 문장 변환 (완료될 때까지 대기)
        response_sentense = generate_sentence_message(
            scenario_type="옷장", 
            response=response,
            additional_data=weather_info
        )

        # 5. 변환된 문장 사용자에게 먼저 보여준 뒤 → 사용자 입력 받기
        self.get_logger().info(f"LLM 응답: {response_sentense}")
        self.alert_to_user(response_sentense)

        # 6. 사용자 키워드 선택 (이제 실행)
        keywords = self.get_user_choice()

        return response_sentense, keywords



    def get_user_choice(self):
        stt = Cumtom_faster_whisper()
        stt.set_model("large")
    
        mic_controller = MicController()
        mic_controller.open_stream()

        recorder = Audio_record()
        recorder.record_start()
        
        print("녹음을 시작합니다. 5초 동안 말해주세요...")
        self.alert_to_user(f"녹음을 시작합니다. 5초 동안 말해주세요...")

        time.sleep(5)
        recorder.recording = False

        audio_data = recorder.record_stop(denoise_value=0.9)

        _, result_txt_denoise, _ = stt.run(audio_data["audio_denoise"], language="ko")
        print(f"STT 결과: {result_txt_denoise}")

        keywords = self.extract_keywords(result_txt_denoise)
        print(f"변환 후: {keywords}")

        return keywords

def stt_loop(context_node: ContextManagerNode):
    """Wakeword 감지 후 음성 명령을 처리"""
    Mic = MicController()
    Mic.open_stream()

    wakeup = WakeupWord(Mic.config.buffer_size)
    wakeup.set_stream(Mic.stream)

    print("Wakeword 대기 중...")

    while True:
        if wakeup.is_wakeup():
            time.sleep(0.1)
            print("Wakeword 감지됨! 음성 녹음 시작")

            recorder = Audio_record()
            recorder.record_start()
            while recorder.recording:
                time.sleep(0.1)

            audio_data = recorder.record_stop(denoise_value=0.9)

            stt = Cumtom_faster_whisper()
            stt.set_model("large")

            _, result_txt_denoise, _ = stt.run(audio_data['audio_denoise'], language="ko")
            print(f"인식 결과: {result_txt_denoise}")

            final_command = result_txt_denoise
            context_node.send_log("사용자 명령: " + final_command)

            if not final_command.strip():
                print("[INFO] 음성 명령이 인식되지 않았습니다. 다시 시도하세요.")
                continue
            elif "종료" in final_command:
                print("종료 명령어 감지됨. 프로그램 종료.")
                break

            try:
                context_node.prompting(final_command)
            except Exception as e:
                print(f"LLM 처리 중 오류 발생: {e}")

            print("다시 Wakeword 대기 중...")

# for easy Test
# def stt_loop(context_node: ContextManagerNode):
#     print("명령어를 입력해주세요. (종료하려면 '종료' 입력)")

#     example_commands = [
#             "옷장에 뭐있어?",
#             "냉장고에 뭐있어?",
#             "사과 가져다줘.",
#             "바나나 가져다줘.",
#             "골뱅이 가져다줘.",
#             "견과류 가져다줘.",
#             "밥 가져다줘.",
#             "반팔 가져다줘.",
#             "반바지 가져다줘.",
#             "긴바지 가져다줘.",
#             "가디건 가져다줘.",
#             "바람막이 가져다줘.",
#             "오늘 점심에 햄버거랑 감자튀김 먹었는데 저녁 먹을 식단 추천해줘.",
#             "옷 추천해줘"
#         ]

#     while True:
#         print("\n[예시 명령어 목록]")
#         for i, command in enumerate(example_commands, 1):
#             print(f"({i}) {command}")

#         final_command = input("문장 번호 입력 >> ")

#         if final_command.strip().lower() == "종료":
#             print("종료 명령어 감지됨. 프로그램 종료.")
#             break

#         try:
#             choice = int(final_command.strip())
#             if 1 <= choice <= len(example_commands):
#                 selected_command = example_commands[choice - 1]
#                 print(f"\n선택한 명령어: {selected_command}")
#                 context_node.prompting(selected_command)
#             else:
#                 print("잘못된 번호입니다. 다시 시도해주세요.")
#         except ValueError:
#             print("유효한 번호를 입력해주세요.")  # 번호 외의 입력이 들어올 경우 예외 처리

#         print("다시 명령어를 입력해주세요.")
    
def main():
    """ROS2 노드를 초기화하고 실행"""
    rclpy.init()
    cm = ContextManagerNode()

    try:
        stt_loop(cm)
    except KeyboardInterrupt:
        pass

    cm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
