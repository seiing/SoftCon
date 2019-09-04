import time
import datetime

def str_datetime():
    d = datetime.datetime.today()
    return d.strftime("%Y%m%d_%H%M%S")

class TimeChecker:
    def __init__(self):
        self.start = 0.0
        self.data = []

    def begin(self):
        self.start = time.time()
        del self.data[:]

    def print_time(self, restart=True):
        print('Time elapsed: ', self.get_time(restart))

    def get_time(self, restart=True):
        t = time.time() - self.start
        if restart:
            self.begin()
        return t

    def print_data(self):
        print(self.data)

    def get_data(self):
        return self.data

    def save(self, msg=' '):
        self.data.append([self.get_time(), msg])