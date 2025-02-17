from lidar import Lidar
from multiprocessing import Process
from datetime import datetime
from typing import Deque
from collections import deque
import cv2
import numpy as np

class FrameRaw:
    def __init__(self, limu_frame, rgb_img):
        self._limu_frame = limu_frame
        self.rgb_img = rgb_img
        self.timestamp = datetime.datetime.now()
        self.xyz_rgb = np.array(limu_frame.get_xyz_rgb(), dtype=np.float32).reshape(limu_frame.n_points, 8)
        if limu_frame.dataType == 2:
            self.amplitude = np.array(limu_frame.get_amplitude_data(), dtype=np.float32)
        else:
            self.amplitude = None

class CallbackDeque(deque):
    def __init__(self, iterable=(), callback=None, maxlen=None):
        super().__init__(iterable, maxlen)
        self.callback = callback

    def append(self, item):
        super().append(item)
        if self.callback:
            self.callback(item)

    def appendleft(self, item):
        super().appendleft(item)
        if self.callback:
            self.callback(item)
    
    def setCallback(self, callback):
        self.callback = callback

class RGBCam:
    def __init__(self, cam_index: int):
        self._cam_index = cam_index
        self.latest_frame = None
        self.cam = cv2.VideoCapture(cam_index)
    
    def capture_loop(self):
        while True:
            ret, self.latest_frame = self.cam.read()

class Event:
    def __init__(self, frame_queue: CallbackDeque):
        self.frame_queue = frame_queue
        self.start_time = datetime.now()

class CaptureProcess:
    def __init__(self, queue_size):
        self.frame_queue = CallbackDeque(maxlen=queue_size)

    def add_frame_to_queue(self, raw_limu_frame):
        global rgb_cam_1
        f = FrameRaw(raw_limu_frame, rgb_cam_1.latest_frame)
        self._frame_queue.append(f)

class EventProcess:
    def __init__(self, capture_queue: CallbackDeque):
        self.current_event = None
        self.capturing: bool = False
        self.capture_queue = capture_queue
        self.event_queue: Deque['Event'] = deque()

    def startEvent(self):
        self.capturing = True
        self.current_event = Event(self.capture_queue)
        

if __name__ == '__main__':
    lidar_1 = Lidar("10.10.31.180")
    lidar_2 = None

    rgb_cam_1 = RGBCam(0)

    capture = CaptureProcess()
    lidar_1.setFrameCallback(capture.add_frame_to_queue)

    event = EventProcess(capture.frame_queue)

    cam1 = Process(target=rgb_cam_1.capture_loop)
    cam1.start()
    cam1.join()