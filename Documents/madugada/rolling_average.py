import numpy as np

class RollingAverage:
    """Class that computes rolling average over a specified number of data points."""
    def __init__(self, count):
        self.count = count
        self.data = np.array([0.] * count, dtype = "float32")
        self.populated = False # Set to True after self.data is filled with data points
        self.index = 0
    def update(self, data_point):
        self.data[self.index] = data_point
        self.index = (self.index + 1) % self.count
        if self.index == 0: # If we have completed 1 cycle of updates...
            self.populated = True
    def calculate_average(self):
        if not self.populated:
            raise Exception("Make sure populated attribute of RollingAverage " \
                            "is True before calling calculate_average!")
        return np.average(self.data)
