import os
from fitomi.config import keys

from langchain.prompts import PromptTemplate
from langchain.chat_models import ChatOpenAI
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnableLambda
from langchain_core.messages import HumanMessage, SystemMessage
import base64

os.environ["OPENAI_API_KEY"] = keys.get("OPENAI_API_KEY")


class LLMQuery:
    def __init__(self, system_prompt: str):
        self.system_prompt = system_prompt
        self.prompt = PromptTemplate.from_template(system_prompt)
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0.5)
        self.output_parser = StrOutputParser()

    # ---
    # 일반 텍스트 질의용
    def query(self, command: str, context: dict = None):
        input_vars = {"user_input": command}
        if context:
            input_vars.update(context)
        print("input_vars:", input_vars)
        return (self.prompt | self.llm | self.output_parser).invoke(input_vars)
    
    # ---
    # 이미지 + 텍스트 멀티모달 질의용
    def query_with_images(self, command: str, image_paths: list[str], context: dict = None):
        input_vars = {"user_input": command}
        if context:
            input_vars.update(context)

        print("input_vars:", input_vars)
        
        system_prompt = self.prompt.format(**input_vars)
        base64_images = [self.encode_image(p) for p in image_paths]
        
        messages = [
            # SystemMessage: 지시문(역할 부여)
            # PromptTemplate의 format 처리 -> 프롬프트 내 변tㅜ 처리 rㅏ능
            SystemMessage(content=system_prompt),
            # HumanMessage: 사람 입력(음성 명령 및 문맥)
            HumanMessage(content=[
                {"type": "text", "text": command},
                *[
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}
                    for b64 in base64_images
                ]
            ])
        ]

        chain = RunnableLambda(lambda m: self.llm.invoke(m)) | self.output_parser
        return chain.invoke(messages)

    def encode_image(self, image_path):
        try:
            with open(image_path, "rb") as f:
                return base64.b64encode(f.read()).decode("utf-8")
        except Exception as e:
            raise ValueError(f"이미지 인코딩 실패: {image_path}, {e}")
