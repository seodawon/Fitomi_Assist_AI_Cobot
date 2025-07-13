import json
import requests
import xml.etree.ElementTree as ET

from datetime import datetime, timedelta
from bs4 import BeautifulSoup
from fitomi.config import keys

from langchain.schema import Document
from langchain.vectorstores import Chroma
from langchain_community.document_loaders import CSVLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.embeddings import OpenAIEmbeddings
from functools import lru_cache
import os

embedding = OpenAIEmbeddings()
base_path = "/home/rokey/choi_ws/src/fitomi/resource"


clothes_path = f"{base_path}/dummy_clothes.json"
fridge_path = f"{base_path}/dummy_fridge_scan_result.json"
food_path = f"{base_path}/food_DB.csv"
processed_food_path = f"{base_path}/processed_food_DB.csv"

persist_dir = f"{base_path}/food"

clothing_context_dir = f"{base_path}/vectorstore/clothing"


# --------------------------------------------------------------------------------

@lru_cache()
def load_clothing_vectorstore(persist_directory: str) -> Chroma:
    if os.path.exists(os.path.join(persist_directory, "index")):
        return Chroma(persist_directory=persist_directory, embedding_function=embedding)

    # 웹에서 크롤링 후 문서 생성
    url = "https://namu.wiki/w/%EA%B8%B0%EC%98%A8%EB%B3%84%20%EC%98%B7%EC%B0%A8%EB%A6%BC"
    headers = {"User-Agent": "Mozilla/5.0"}
    
    try:
        response = requests.get(url, headers=headers)
        soup = BeautifulSoup(response.text, 'html.parser')
    except Exception as e:
        raise RuntimeError(f"웹 크롤링 실패: {str(e)}")

    docs = []
    for table in soup.find_all("table"):
        for row in table.find_all("tr")[1:]:
            cells = row.find_all(["th", "td"])
            row_text = ", ".join(cell.get_text(strip=True) for cell in cells)
            if row_text.strip():
                parts = row_text.split(",")
                if len(parts) >= 2:
                    temp = parts[0].strip()
                    clothes = ", ".join(p.strip() for p in parts[1:])
                    docs.append(Document(page_content=f"{temp}(도)일 때는 {clothes}를 입는 것이 적절합니다."))

    if not os.path.exists(persist_directory):
        os.makedirs(persist_directory)
        
    return Chroma.from_documents(documents=docs, embedding=embedding, persist_directory=persist_directory)

# ---
# 1. 날씨 정보 가져오기
def get_weather_info(nx=55, ny=127, base_time='0500',
                     url='http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getVilageFcst') -> dict:
    today = datetime.utcnow() + timedelta(hours=9)
    base_date = today.strftime('%Y%m%d')
    
    params = {
        'serviceKey': keys.get("SERVICE_KEY"),
        'pageNo': '1',
        'numOfRows': '1000',
        'dataType': 'XML',
        'base_date': base_date,
        'base_time': base_time,
        'nx': str(nx),
        'ny': str(ny)
    }
    
    try:
        response = requests.get(url, params=params)
        root = ET.fromstring(response.content)
        items = root.findall('.//item')
    except Exception as e:
        return {
            "min_temp": "오류",
            "max_temp": "오류",
            "weather_pty": f"날씨 정보를 가져오는 중 오류 발생: {str(e)}"
        }
    
    result = {'min_temp': 'N/A', 'max_temp': 'N/A', 'weather_pty': '알 수 없음'}
    for item in items:
        cat = item.find('category').text
        val = item.find('fcstValue').text
        if cat == 'TMN':
            result['min_temp'] = val
        elif cat == 'TMX':
            result['max_temp'] = val
        elif cat == 'SKY':
            sky_map = {'1': '맑음', '3': '구름많음', '4': '흐림'}
            result['weather_pty'] = sky_map.get(val, '알 수 없음')
    
    return {
        "min_temp": result["min_temp"],
        "max_temp": result["max_temp"],
        "weather_pty": result["weather_pty"]
    }

# 2. 옷장 설명 문장
def get_closet_caption(detected_list: list) -> tuple:
    try:
        with open(clothes_path, "r") as f:
            closet = json.load(f)
        
        detected_src = [c['src'] for c in closet if c['type'] in detected_list]
        caption = "현재 옷장에는 {}가(이) 있습니다.".format(
            ", ".join(f"{c['type']}".strip() for c in closet)
        )
        return detected_src, {"closet_caption": caption}
    except Exception as e:
        return [], {"closet_caption": f"옷장 정보를 불러올 수 없습니다: {str(e)}"}


# 3. 날씨 기반 옷차림 컨텍스트 정보 생성 (Vector DB)
def get_fashion_context_docs() -> dict:
    try:
        vectorstore = load_clothing_vectorstore(clothing_context_dir)
        retrieved_docs = vectorstore.as_retriever().invoke("오늘 날씨에 어울리는 옷")
        combined_result = "\n\n".join(doc.page_content for doc in retrieved_docs)
        return {"fashion_context": combined_result}
    except Exception as e:
        return {"fashion_context": f"옷차림 정보 로딩 실패: {str(e)}"}

# --------------------------------------------------------------------------------

@lru_cache()
def load_vectorstore(file_path: str, persist_directory: str) -> Chroma:
    if not os.path.exists(persist_directory):
        os.makedirs(persist_directory)
    
    if os.path.exists(os.path.join(persist_directory, "index")):
        return Chroma(persist_directory=persist_directory, embedding_function=embedding)

    loader = CSVLoader(file_path=file_path)
    docs = loader.load()
    splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=50)
    split_docs = splitter.split_documents(docs)
    
    return Chroma.from_documents(split_docs, embedding=embedding, persist_directory=persist_directory)

# ---
# 1. 냉장고 라벨 및 위치 정보
def get_fridge_caption() -> dict:
    try:
        with open(fridge_path, "r") as f:
            data = json.load(f)
        return {"food_list": data}
    except Exception as e:
        return {"food_list": f"냉장고 정보를 불러올 수 없습니다: {str(e)}"}

# 2. 식품/가공식품 기반 컨텍스트 정보 생성 (Vector DB)
def get_nutrition_context_docs(type: str, query: str = "", top_n: int = 500) -> dict:
    if type == "food":
        file_path = food_path
        persist_dir = f"{base_path}/vectorstore/food"
    elif type == "processed_food":
        file_path = processed_food_path
        persist_dir = f"{base_path}/vectorstore/processed"
    else:
        raise ValueError(f"'{type}'은 잘못된 입력입니다. 'food' 또는 'processed_food'만 허용됩니다.")

    try:
        vectorstore = load_vectorstore(file_path, persist_dir)
        retrieved_docs = vectorstore.as_retriever(search_kwargs={"k": top_n}).invoke(query)

        keywords = ["탄수화물", "단백질", "지방"]  # 추출할 키워드  // , "당류"

        result_lines = []
        for doc in retrieved_docs:
            text = doc.page_content
            lines = text.strip().split("\n")
            title = lines[0].strip() if lines else "이름없음"

            nutrition_info = {}
            for line in lines:
                for key in keywords:
                    if key in line and key not in nutrition_info:
                        parts = line.split(":")
                        if len(parts) >= 2:
                            value = parts[1].strip()
                            nutrition_info[key] = value

            line = f"{title}: " + ", ".join(
                [f"{k} {nutrition_info.get(k, '-')}" for k in keywords]
            )
            result_lines.append(line)

        combined = "\n".join(result_lines)
        return {"nutrition_context": combined}

    except Exception as e:
        return {"nutrition_context": f"영양 정보 벡터 검색 중 오류: {str(e)}"}

