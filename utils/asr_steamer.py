import numpy as np
import pyaudio
import socket
import threading
import queue
import logging

from contextlib import contextmanager
from ctypes import CFUNCTYPE, c_char_p, c_int, cdll

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)

# For suppressing ALSA error messages
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)


def py_error_handler(filename, line, function, err, fmt):
    pass


c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)


@contextmanager
def noalsaerr():
    try:
        asound = cdll.LoadLibrary("libasound.so")
    except OSError:
        logging.warning("libasound.so not found. ALSA errors will not be suppressed.")
        yield
        return
    asound.snd_lib_error_set_handler(c_error_handler)
    try:
        yield
    finally:
        asound.snd_lib_error_set_handler(None)


########################################################################################

MONO = 1
STEREO = 2

RATE_48000 = 48000
RATE_44100 = 44100
RATE_16000 = 16000

FORMAT_MAP = {
    "s16le": (pyaudio.paInt16, np.int16),
    "s32le": (pyaudio.paInt32, np.int32),
    "f32le": (pyaudio.paFloat32, np.float32),
    # Add more formats as needed
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
        try:
            self.format_details = FORMAT_MAP[self.format_str]
        except KeyError:
            raise ValueError(
                f"Unsupported format '{self.format_str}'. Supported formats are: {list(FORMAT_MAP.keys())}"
            )
        self.sampling_rate = sampling_rate


class SourceStreamer:
    """
    Obtain audio data from a source (default microphone device) -- which we assume generates data continuously -- and invokes a callback function when a chunk of audio data is available.
    """

    def __init__(
        self,
        frame_chunk_dur_sec=0.2,
        source_hw_params: HardWareParams = HardWareParams(),
    ):
        # Input parameters
        self.source_hw_params = source_hw_params

        # Streaming parameters
        self.frame_chunk_dur_sec = frame_chunk_dur_sec
        self.frame_chunk_size = int(
            self.source_hw_params.sampling_rate * self.frame_chunk_dur_sec
        )
        logging.info(
            f"Chunk size to be streamed is {self.frame_chunk_size} samples per chunk."
        )

        # Setup output callback handling
        self.output_callback = None

        # Setup streaming within noalsaerr context
        with noalsaerr():
            self.pa = pyaudio.PyAudio()
            # Validate device index
            if self.source_hw_params.device_index is not None:
                try:
                    device_info = self.pa.get_device_info_by_index(
                        self.source_hw_params.device_index
                    )
                except IOError:
                    raise ValueError(
                        f"Audio device with index {self.source_hw_params.device_index} not found."
                    )
            self.source_stream = self.pa.open(
                input_device_index=self.source_hw_params.device_index,
                channels=self.source_hw_params.channels,
                format=self.source_hw_params.format_details[0],
                rate=self.source_hw_params.sampling_rate,
                input=True,
                frames_per_buffer=self.frame_chunk_size,
                stream_callback=self.stream_callback,
            )
            self.source_stream.start_stream()
            logging.info("Audio stream started.")

    def stream_callback(self, in_data: bytes, frame_count, time_info, status):
        """
        The input data (bytes) will have the # of frames according to frames_per_buffer = self.frame_chunk_size
        But depending on the sampling format, each frame can contain *multiple bytes*. For example, if the format is
        's16le', then each frame will have 2 bytes.
        """

        logging.debug(f"Got {len(in_data)} bytes.")

        if self.output_callback is not None:
            try:
                # Invoke the callback functions with the audio data available using
                self.output_callback(in_data, time_info)
            except Exception as e:
                logging.error(
                    f"Error in invoking the callback function: {e}. Will try again for the next chunk."
                )

        return (in_data, pyaudio.paContinue)

    def __enter__(self):
        # Optional: If additional setup is needed on entering the context
        return self

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback,
    ):
        if self.source_stream.is_active():
            self.source_stream.stop_stream()
        self.source_stream.close()
        self.pa.terminate()
        logging.info("Audio stream terminated.")


class SocketSourceStreamer(SourceStreamer):
    """
    Obtain audio data from a source (default microphone device) -- which we assume generates data continuously -- and sends the data to a remote server.
    """

    def __init__(
        self,
        streaming_freq=31.25,
        source_hw_params: HardWareParams = HardWareParams(),
        receiver_ip: str = "localhost",
        receiver_port: int = 50007,
    ):
        super().__init__(1 / streaming_freq, source_hw_params)

        # Output parameters
        self.receiver_ip = receiver_ip
        self.receiver_port = receiver_port

        # Setup socket for sending data
        self.send_queue = queue.Queue(maxsize=100)  # Adjust maxsize as needed
        self.sock = None
        self.setup_socket()

        # Start the sending thread
        self.sending_thread = threading.Thread(target=self.send_data)
        self.sending_thread.daemon = True
        self.sending_thread.start()

    def setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.receiver_ip, self.receiver_port))
            logging.info(
                f"Connected to the receiver at {self.receiver_ip}:{self.receiver_port}."
            )
        except socket.error as e:
            logging.error(
                f"Failed to connect to receiver at {self.receiver_ip}:{self.receiver_port}: {e}"
            )
            self.sock = None
            raise

    def send_data(self):
        while True:
            data = self.send_queue.get()
            if data is None:
                break  # Exit the thread
            if self.sock:
                try:
                    self.sock.sendall(data)
                except socket.error as e:
                    logging.error(f"Socket error during send: {e}")
                    self.sock.close()
                    self.sock = None
            self.send_queue.task_done()

    def stream_callback(self, in_data: bytes, frame_count, time_info, status):
        # Enqueue data for sending
        if self.sock:
            try:
                self.send_queue.put_nowait(in_data)
            except queue.Full:
                logging.warning("Send queue is full. Dropping audio data.")
        else:
            logging.warning("Socket is not connected. Dropping audio data.")
        return (in_data, pyaudio.paContinue)

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback,
    ):
        # Signal the sending thread to exit
        if hasattr(self, "send_queue"):
            self.send_queue.put(None)
        if hasattr(self, "sending_thread"):
            self.sending_thread.join()
        # Close the socket
        if hasattr(self, "sock") and self.sock:
            try:
                self.sock.close()
                logging.info(
                    f"Socket to {self.receiver_ip}:{self.receiver_port} closed."
                )
            except Exception as e:
                logging.error(f"Error closing socket: {e}")
        # Proceed with existing cleanup
        super().__exit__(exc_type, exc_value, traceback)
