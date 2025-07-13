import requests
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta

from config import keys


def get_weather_info(nx: int, ny: int, base_time='0500') -> dict:
    # 오늘 날짜 (KST)
    today = datetime.utcnow() + timedelta(hours=9)
    base_date = today.strftime('%Y%m%d')

    service_key = keys.get('SERVICE_KEY')
    url = 'http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getVilageFcst'

    params = {
        'serviceKey': service_key,
        'pageNo': '1',
        'numOfRows': '1000',
        'dataType': 'XML',
        'base_date': base_date,
        'base_time': base_time,
        'nx': str(nx),
        'ny': str(ny)
    }

    response = requests.get(url, params=params)
    root = ET.fromstring(response.content)
    items = root.findall('.//item')

    result = {
        'min_temp': None,
        'max_temp': None,
        'temperature': None,
        'sky': None,
        'precipitation_type': None
    }

    for item in items:
        category = item.find('category').text
        fcst_value = item.find('fcstValue').text
        if category == 'TMN':
            result['min_temp'] = fcst_value
        elif category == 'TMX':
            result['max_temp'] = fcst_value
        elif category == 'T1H':
            result['temperature'] = fcst_value
        elif category == 'SKY':
            result['sky'] = fcst_value
        elif category == 'PTY':
            result['precipitation_type'] = fcst_value

    return result

# 사용 예시
weather = get_weather_info(nx=55, ny=127)
print(f"하늘 상태(SKY): {weather['sky']} (1: 맑음, 3: 구름많음, 4: 흐림)")
print(f"강수 형태(PTY): {weather['precipitation_type']} (0: 없음, 1: 비, 2: 비/눈, 3: 눈, 4: 소나기)")
print(f"최저 기온: {weather['min_temp']}℃, 최고 기온: {weather['max_temp']}℃")
