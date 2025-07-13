import numpy as np
import openwakeword
from openwakeword.model import Model
from scipy.signal import resample
import fitomi.MicController


MODEL_NAME = "/home/rokey/choi_ws/src/fitomi/resource/hello_rokey_8332_32.tflite"


class WakeupWord:
    def __init__(self, buffer_size):
        openwakeword.utils.download_models()
        self.model = None
        self.model_name = "hello_rokey_8332_32" # MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        self.buffer_size = buffer_size

    def is_wakeup(self):
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        outputs = self.model.predict(audio_chunk, threshold=0.3)
        confidence = outputs[self.model_name]
        print("confidence: ", confidence)
        # Wakeword 탐지
        if confidence > 0.009: # 실제 데모에선 수정 필요(원래 0.3이였음)
            print("Wakeword detected!")
            return True
        return False

    def set_stream(self, stream):
        self.model = Model(wakeword_models=[MODEL_NAME])
        self.stream = stream


if __name__ == "__main__":
    Mic = MicController.MicController()
    Mic.open_stream()

    wakeup = WakeupWord(Mic.config.buffer_size)
    wakeup.set_stream(Mic.stream)
    while wakeup.is_wakeup() is False:
        pass
