""" Latency Buffer Module """

from collections import deque
import time

class LatencyBuffer:
    """ Implements a latency buffer for robot commands """
    def __init__(self,delay_seconds=0.3):
        self.delay = delay_seconds
        self.queue = deque()

    def push(self,command):
        """ Push a new command into the buffer with timestamp """
        timestamp = time.time()
        self.queue.append((timestamp,command))

    def pop_ready(self):
        """Return the oldest command that passed delay time"""
        if not self.queue:
            return None
        
        timestamp,command = self.queue[0]

        if time.time() - timestamp >= self.delay :
            self.queue.popleft()
            return command
        return None
    