import numpy as np
import pyaudio
import socket

# For suppressing alsa error messages
from contextlib import contextmanager
from ctypes import CFUNCTYPE, c_char_p, c_int, cdll


ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)


def py_error_handler(filename, line, function, err, fmt):
    pass


c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)


@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary("libasound.so")
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)


######################################################################################3

MONO = 1
STEREO = 2

RATE_48000 = 48000
RATE_44100 = 44100
RATE_16000 = 16000

FORMAT_MAP = {
    "s16le": (pyaudio.paInt16, np.int16),
}


class HardWareParams:
    """
    This class is used to store the hardware parameters of the audio devices.
    """

    def __init__(
        self,
        device_index: int = None,  # Means default device
        channels: int = MONO,
        format_str: str = "s16le",
        sampling_rate: int = RATE_44100,
    ):
        self.device_index = device_index
        self.channels = channels
        self.format_str = format_str
        self.format_details = FORMAT_MAP[self.format_str]
        self.sampling_rate = sampling_rate


class SourceStreamer:
    """
    Obtain audio data from a source (default microphone device) -- which we assume generates data continuously -- and invokes a callback function when a chunk of audio data is available.
    """

    def __init__(
        self,
        frame_chunk_dur_sec=0.2,
        source_hw_params: HardWareParams = HardWareParams(),  # This should match with the receiver
    ):
        # Input parameters
        self.source_hw_params = source_hw_params

        # Streaming parameters
        self.frame_chunk_dur_sec = frame_chunk_dur_sec
        self.frame_chunk_size = int(
            self.source_hw_params.sampling_rate * self.frame_chunk_dur_sec
        )
        print(
            f"""Chunk size to be streamed is {self.frame_chunk_size} samples per chunk."""
        )

        # Setup output callback handling
        self.output_callback = None

        # Setup streaming
        self.pa = pyaudio.PyAudio()
        self.source_stream = self.pa.open(
            input_device_index=self.source_hw_params.device_index,
            channels=self.source_hw_params.channels,
            format=self.source_hw_params.format_details[0],
            rate=self.source_hw_params.sampling_rate,
            input=True,
            frames_per_buffer=self.frame_chunk_size,
            stream_callback=self.stream_callback,
        )

    def stream_callback(self, in_data: bytes, frame_count, time_info, status):
        """
        The input data (bytes) will have the # of frames according to frames_per_buffer = self.frame_chunk_size
        But depending on the sampling format, each frame can contain *multiple bytes*. For example, if the format is
        's16le', then each frame will have 2 bytes.
        """

        print(f"Got {len(in_data)} bytes.")

        if self.output_callback is not None:
            try:
                # Invoke the callback functions with the audio data available using
                self.output_callback(in_data, time_info)
            except Exception as e:
                print(
                    f"Error in invoking the callback function: {e}. Will try again for the next chunk."
                )

        return (in_data, pyaudio.paContinue)

    def __exit__(
        self: object,
        type: object,
        value: object,
        traceback: object,
    ) -> object:
        self.source_stream.stop_stream()
        self.source_stream.close()
        self.pa.terminate()


class SocketSourceStreamer(SourceStreamer):
    """
    Obtain audio data from a source (default microphone device) -- which we assume generates data continuously -- and sends the data to a remote server.
    """

    def __init__(
        self,
        frame_chunk_dur_sec=0.2,
        source_hw_params: HardWareParams = HardWareParams(),  # This should match with the receiver
        receiver_ip: str = "localhost",
        receiver_port: int = 50007,
    ):
        super().__init__(frame_chunk_dur_sec, source_hw_params)

        # Output parameters
        self.receiver_ip = receiver_ip
        self.receiver_port = receiver_port

        # Setup socket for sending data
        self.setup_socket()

    def setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.receiver_ip, self.receiver_port))
        print(f"Connected to the receiver at {self.receiver_ip}:{self.receiver_port}.")
    
    def stream_callback(self, in_data: bytes, frame_count, time_info, status):
        try:
            # Send the audio data to the receiver
            self.sock.sendall(in_data)
        except KeyboardInterrupt:
            print("Keyboard interrupt. Closing the socket.")
            self.sock.close()
            raise KeyboardInterrupt
        except Exception as e:
            print(f"Error in sending data: {e}. Will try again for the next chunk.")
        finally:
            return (in_data, pyaudio.paContinue)
