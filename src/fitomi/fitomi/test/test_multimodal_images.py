import os
import base64
from langchain_openai import ChatOpenAI
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnableLambda
from langchain_core.messages import HumanMessage, SystemMessage

from config import keys
os.environ["OPENAI_API_KEY"] = keys.get("OPENAI_API_KEY")


def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")


def generate_matching_prompt(base64_images):
    return [
        SystemMessage(content="당신은 이미지 비교 전문가입니다."),
        HumanMessage(content=[
            {"type": "text", "text": "1번 이미지를 기준으로 2~5번 이미지 중 가장 유사한 옷을 골라주세요. "
            " 날씨와 어울리는지 알려주세요. 오늘 날씨는 최저 기온 19도, 최고 기온 30도로 맑습니다. 오늘 날씨와 안어울린다면 가장 어울리는 옷도 골라줘."
            "혹시 비슷한 옷이 없으면 추천하기 어렵습니다.를 출력해줘."},
            *[
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}
                for b64 in base64_images
            ]
        ])
    ]


def build_matching_chain():
    model = ChatOpenAI(model="gpt-4o", temperature=0.2)
    return RunnableLambda(lambda messages: model.invoke(messages)) | StrOutputParser()


def run_image_matching(image_paths):
    base64_images = [encode_image(p) for p in image_paths]
    messages = generate_matching_prompt(base64_images)
    chain = build_matching_chain()
    response = chain.invoke(messages)
    print("\n💡 GPT-4o 응답:")
    print(response)

# 사용 예시
if __name__ == "__main__":
    image_paths = [
        "/home/choi/ros2_ws/sample3.jpg",  # 기준 이미지 (1번)
        "/home/choi/ros2_ws/sample4.jpg",       # 후보 (2~5번)
        "/home/choi/ros2_ws/sample5.jpg",
        "/home/choi/ros2_ws/sample6.jpg",
        "/home/choi/ros2_ws/sample7.jpg",
    ]
    run_image_matching(image_paths)
