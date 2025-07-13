import os
import asyncio
from openai import AsyncOpenAI
from openai.helpers import LocalAudioPlayer
from fitomi.config import keys

os.environ["OPENAI_API_KEY"] = keys.get('OPENAI_API_KEY')

openai = AsyncOpenAI()

async def speak_async(text: str) -> None:
    async with openai.audio.speech.with_streaming_response.create(
        model="gpt-4o-mini-tts",  # 또는 "gpt-4o-mini-tts" (정확한 모델명 확인 필요)
        voice="shimmer",      # coral, nova, shimmer 중 선택 가능
        input=text,
        instructions="밝고 행복한 말투로 말해주세요.",
        response_format="pcm",
    ) as response:
        await LocalAudioPlayer().play(response)

def speak(text: str) -> None:
    asyncio.run(speak_async(text))

# ex)
# speak("부르셨나요?")
# speak("오늘의 날씨는 맑고 기온은 23도입니다.")

########################################################################3
# from gtts import gTTS 
# import playsound

# def speak(text): 
# 	tts = gTTS(text=text, lang='ko') 
# 	tts.save('./hello.mp3')
# 	playsound.playsound('./hello.mp3') 

# speak("부르셨나요?")
