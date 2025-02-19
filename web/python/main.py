from lidar import Lidar
from multiprocessing import Process, Manager
from datetime import datetime
import time
import queue
import cv2
import numpy as np

class FrameRaw:
    def __init__(self, limu_frame, rgb_img):
        self.rgb_img = rgb_img
        self.timestamp = datetime.now()
        self.frame_id = limu_frame.frame_id
        self.height = limu_frame.height
        self.width = limu_frame.width
        self.n_points = limu_frame.n_points
        self.data_type = limu_frame.dataType
        self.xyz_rgb = np.array(limu_frame.get_xyz_rgb(), dtype=np.float32).reshape(limu_frame.n_points, 8)
        if self.data_type == 2:
            self.amplitude = np.array(limu_frame.get_amplitude_data(), dtype=np.float32)
        else:
            self.amplitude = None

class RGBCam:
    def __init__(self, cam_index: int):
        self._cam_index = cam_index
        self.open = False
        self.should_close = False
        self.last_frame = None
        self.cam = cv2.VideoCapture(cam_index)
        if not self.cam.isOpened():
            print("Error: Could not open camera.")
            exit()
        self.open = True
    
    def close(self):
        self.should_close = True
        print(self.should_close)
    
    def capture_loop(self, dest: queue.Queue):
        while True:
            if self.should_close:
                print("releasing camera and breaking out of capture loop")
                self.cam.release()
                self.open = False
                return
            
            ret, rgb = self.cam.read()
            if dest.full():
                dest.get()

            dest.put(rgb)

class Event:
    def __init__(self, frame_queue: queue.Queue):
        self.frame_queue = frame_queue
        self.start_time = datetime.now()

class CaptureProcess:
    def __init__(self, frame_queue: queue.Queue):
        self.frame_addresses = {}
        self.frame_queue = frame_queue

    def add_frame_to_queue(self, raw_limu_frame):
        if id(raw_limu_frame) not in self.frame_addresses:
            self.frame_addresses[id(raw_limu_frame)] = raw_limu_frame
            print(f"added frame at address {raw_limu_frame}")
        print(f"Got frame id {raw_limu_frame.frame_id}")
        # global cam_1_queue
        # rgb = cam_1_queue.get()
        f = FrameRaw(raw_limu_frame, None)
        if self.frame_queue.full():
            self.frame_queue.get()
        self.frame_queue.put(f)
        return

class EventProcess:     
    def __init__(self, capture_queue: queue.Queue, event_queue: queue.Queue):
        self.current_event = None
        self.capturing: bool = False
        self.capture_queue = capture_queue
        self.event_queue = event_queue

    def startEvent(self):
        print("TBI")
        self.capturing = True
        # self.current_event = Event(self.capture_queue)
        

if __name__ == '__main__':
    raw_frame_queue = Manager().Queue(maxsize=200)
    cam_1_queue = Manager().Queue(maxsize=1)

    lidar_1 = Lidar("10.10.31.180")
    lidar_2 = None

    rgb_cam_1 = RGBCam(0)

    capture = CaptureProcess(raw_frame_queue)
    lidar_1.setFrameCallback(capture.add_frame_to_queue)

    # event = EventProcess(capture.frame_queue)

    cam1 = Process(target=rgb_cam_1.capture_loop, args=(cam_1_queue,))
    cam1.start()
    
    while True:
        command = input("Enter Command:")
        print(command)
        match command:
            case "exit":
                lidar_1.streamStop()
                rgb_cam_1.close() # this isn't working due to multiprocessing stuff, fix it.
                exit()
