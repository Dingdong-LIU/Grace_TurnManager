import queue
import threading


class SharedData:
    def __init__(self):
        self.queue_lock = threading.Lock()
        self.shared_queue = queue.Queue() # the shared queue for current sentiment analysis
        self.previous_question = ""
        self.previous_question_lock = threading.Lock()
        self.sentiment_analysis_lock = threading.Lock()
        self.sentiment_ready = False
        self.sentiment = None

    def write_to_queue(self, data):
        with self.queue_lock:
            self.shared_queue.put(data)

    def read_from_queue(self):
        with self.queue_lock:
            if not self.shared_queue.empty():
                self.sentiment = self.shared_queue.get()
            return self.sentiment
        
    def is_ready(self) -> bool:
        """Check if the sentiment analysis is ready

        Returns:
            bool: True if sentiment analysis is ready, False otherwise
        """
        return self.sentiment_ready
    
    def change_previous_question(self, question):
        with self.previous_question_lock:
            self.previous_question = question