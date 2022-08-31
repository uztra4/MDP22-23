import abc
import threading
import queue

class BaseServer(abc.ABC):

    def __init__(self, recv_callback):
        self.send_queue = queue.Queue()
        self.recv_callback = recv_callback

    def run(self):
        threading.Thread(target=self._queue_loop).start()
        threading.Thread(target=self._server_loop).start()
        print("Started...")

    @abc.abstractmethod
    def _server_loop(self):
        pass

    def _queue_loop(self):
        while True:
            msg = self.send_queue.get()
            self._send(msg)
            
    def send(self, msg):
        self.send_queue.put(msg)

    @abc.abstractmethod
    def _send(self, msg):
        pass
