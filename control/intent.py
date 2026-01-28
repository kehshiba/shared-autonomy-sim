""" Intent Estimation Module """
from collections import deque
import numpy as np

class IntentEstimator:
    """
    Estimates user intent based on recent end-effector positions
    """

    def __init__(self,window=6):
        self.history = deque(maxlen=window)
    
    def add(self, pos):
        """
        Add a new human command position
        """
        self.history.append(np.array(pos))

    def estimate(self):
        """
        Estimate intent as direction of motion
        """
        if len(self.history) < 2:
             return None
         
        start = self.history[0]
        end = self.history[-1]

        intent = end - start
        return intent