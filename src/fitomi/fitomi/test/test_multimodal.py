import os
import base64
import requests
import json

from config import keys

from langchain.chat_models import ChatOpenAI
from langchain_core.output_parsers import StrOutputParser

from langchain.prompts import PromptTemplate
from langchain_core.output_parsers import StrOutputParser
from langchain.chat_models import ChatOpenAI
from langchain.vectorstores import Chroma
from langchain.embeddings import OpenAIEmbeddings
from langchain.chains import LLMChain
from langchain_core.messages import HumanMessage, SystemMessage

import requests
import xml.etree.ElementTree as ET

os.environ["OPENAI_API_KEY"] = keys.get('OPENAI_API_KEY')
current_dir = os.path.dirname(os.path.abspath(__file__))
clothes_path = os.path.join(current_dir, "dummy_clothes.json")

# Function to generate a prompt based on given parameters
def gen_prompt(param_dict):

    # Define the system message content
    system_message = "당신은 사용자가 제공한 이미지를 친절하게 설명하고, 질문에 답하는 유용한 도우미입니다."

    # Define the human messages content
    human_messages = [
        {
            "type" : "text",
            "text" : f"{param_dict['question']}",
        },
        {
            "type" : "image_url",
            "image_url" : {
                "url" : f"{param_dict['image_url']}",
            }
        }

    ]

    return [SystemMessage(content=system_message), HumanMessage(content=human_messages)]

def generate_caption_from_detection(detections):
    descriptions = []
    for obj in detections:
        color = obj.get("color", "")
        label = obj["label"]
        desc = f"{color} {label}".strip()
        descriptions.append(desc)
    return f"현재 옷장에는 {', '.join(descriptions)}가(이) 있습니다."

def load_json(filepath):
    with open(filepath, "r") as f:
        return json.load(f)
    
# # Download URL of the image
# url = "http://tyritarot.github.io/warehouse/2024/2024-4-7-shining_in_the_cherry_blossoms_and_just_me_title.jpg"
# response = requests.get(url)

# # Save the image content to a file
# with open("sample.jpg", "wb") as file:
#     file.write(response.content)
# print("Image downloaded and saved as sample.jpg")

# Function to encode the image
def encode_image(image_path):
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')
print("Image downloaded and saved as sample.jpg")

# Path to your image
image_path = "/home/choi/ros2_ws/sample3.jpg"

# Getting the base64 string
base64_image = encode_image(image_path)

chat_model = ChatOpenAI(model="gpt-4o")

# Create a chain by combining the prompt generation, chat model, and output parser
chain = gen_prompt | chat_model | StrOutputParser()

# Invoke the chain with the provided question and image URL
# response = chain.invoke({
#     "question":"이 사람이 입고 있는 옷이랑 비슷한 옷 추천해줘",
#     "image_url": "http://tyritarot.github.io/warehouse/2024/2024-4-7-shining_in_the_cherry_blossoms_and_just_me_title.jpg"
#     }
# )

closet = load_json(clothes_path)
closet_caption = generate_caption_from_detection(closet)
print(closet_caption)

response = chain.invoke(
   {"question":"{closet_caption} 내 옷장에서 이 옷이랑 가장 유사한 옷을 골라줘", 
    "image_url": f"data:image/jpeg;base64,{base64_image}"}
    )
print(response)