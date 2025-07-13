import pyaudio

class MicController:
    def __init__(self, rate=48000, chunk=960):
        self.config = type('obj', (object,), {"rate": rate, "chunk": chunk, "buffer_size": chunk})
        self.p = pyaudio.PyAudio()
        self.stream = None

    def open_stream(self):
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.config.rate,
            input=True,
            frames_per_buffer=self.config.chunk
        )

    def close(self):
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
