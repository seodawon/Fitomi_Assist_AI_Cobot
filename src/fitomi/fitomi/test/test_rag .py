from langchain.vectorstores import Chroma
from langchain.embeddings import SentenceTransformerEmbeddings
from langchain.docstore.document import Document

from huggingface_hub import login

import os
import json

current_dir = os.path.dirname(os.path.abspath(__file__))
knowledge_path = os.path.join(current_dir, "outfit_knowledge.json")
weather_path = os.path.join(current_dir, "weather_sample.json")

# Dummy Detection Result


#############################################################################################
''' 지식 → ChromaDB에 저장 '''

def load_knowledge():
    with open(knowledge_path, "r") as f:
        return json.load(f)
    
    
embedding_model = SentenceTransformerEmbeddings(model_name="all-MiniLM-L6-v2")
kb = load_knowledge()

docs = []
for item in kb:
    content = f"{str(item['start_temp'])}~{str(item['end_temp'])}도, {item['weather']} 날씨에는 {item['recommend']}가 적절합니다."
    docs.append(Document(page_content=content, metadata=item))

vectorstore = Chroma.from_documents(documents=docs, embedding=embedding_model, persist_directory="./rag_db")
vectorstore.persist()

#############################################################################################
''' Retriever + LLaMA2 연동 '''

from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline
from langchain.llms import HuggingFacePipeline
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain.chains import RetrievalQA
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain.llms import HuggingFacePipeline
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM

def load_weather():
    with open(weather_path, "r") as f:
        return json.load(f)

# TODO: TTS + 프롬프트 엔지니어링 필요
def load_prompt():
    prompt = PromptTemplate(
        input_variables=["location", "temp", "weather", "knowledge"],
        template=(
            "오늘은 {location}의 날씨가 {weather}이고, 기온은 {temp}도야. "
            "날씨에 적절한 옷차림 정보야: {knowledge}. "
            "위 정보를 참고해서 오늘 입으면 좋을 옷을 추천해줘. "
            "스타일이나 보온성에 대한 설명도 짧게 곁들여줘."
        )
    )
    return prompt

def generate_response(weather_info, recommendation):
    return f"{weather_info['location']}의 오늘 날씨는 {weather_info['weather']}이고 기온은 {weather_info['temp']}도입니다. {recommendation} 추천드려요!"

# DB 재로딩
vectorstore = Chroma(persist_directory="./rag_db", embedding_function=embedding_model)
retriever = vectorstore.as_retriever()

# Llamma2 로딩
login(token="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")  # HuggingFace access token
model_id = ["mistralai/Mistral-7B-Instruct-v0.1", "meta-llama/Llama-2-7b-hf"] #  "meta-llama/Llama-3.1-8B-Instruct"(pipeline 다름)
tokenizer = AutoTokenizer.from_pretrained(model_id[2])
model = AutoModelForCausalLM.from_pretrained(model_id[2], device_map="auto")
# model.to("cuda")

pipe = pipeline(
    "text-generation",
    tokenizer=tokenizer,
    max_new_tokens=256,
    temperature=0.7,
    do_sample=True,
    top_k=50,
    top_p=0.9
)
llm = HuggingFacePipeline(pipeline=pipe)

# 날씨 정보
weather_info = load_weather()

query = f"{weather_info['temp']}도 {weather_info['weather']} 날씨에는 어떤 옷이 적절할까?"
retrieved_docs = retriever.get_relevant_documents(query)
knowledge_text = "\n".join([doc.page_content for doc in retrieved_docs])

# 관련 지식 검색
retrieved_docs = retriever.get_relevant_documents(query)
knowledge_text = "\n".join([doc.page_content for doc in retrieved_docs])

prompt = load_prompt()
chain = LLMChain(llm=llm, prompt=prompt)

response = chain.run(
    location=weather_info["location"],
    temp=weather_info["temp"],
    weather=weather_info["weather"],
    knowledge=knowledge_text
)

recommendation = chain.run(**weather_info)
response = generate_response(weather_info, recommendation)
print(response)