import datetime

class Timestamp():
    def __init__(self):
        self.prev_frame_timestamp = datetime.datetime.utcnow()

    def run(self,):
        current_timestamp = datetime.datetime.utcnow()
        delta_seconds = (current_timestamp - self.prev_frame_timestamp).total_seconds()
        self.prev_frame_timestamp = current_timestamp
        return str(current_timestamp), delta_seconds

