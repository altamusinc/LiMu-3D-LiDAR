import limu_py
import json
import threading

class Lidar:
    def __init__(self, address: str):
        print("connecting to LIMU")
        self.tof = limu_py.ToF.tof320(address, "50660")
        self._lens_type = 0  
        self._frequency_modulation = 2
        self._channel = 0
        self._image_type = 2
        self._hdr_mode = 2
        self._integration_time_tof_1 = 50
        self._integration_time_tof_2 = 400
        self._integration_time_tof_3 = 4000
        self._integration_time_tof_Gr = 10000
        self._min_amplitude = 60
        self._lens_center_offset_x = 0
        self._lens_center_offset_y = 0
        self._roi_left_x = 0
        self._roi_right_x = 319
        self._roi_top_y = 0
        self._roi_bottom_y = 239
        self.apply_all_settings()

    @property
    def lens_type(self):
        return self._lens_type
    
    @lens_type.setter
    def lens_type(self, val):
        self._lens_type = val
        print(f"Setting Lens Type to {val}")
        self.tof.setLensType(val)

    @property
    def frequency_modulation(self):
        return self._frequency_modulation
    
    @frequency_modulation.setter
    def frequency_modulation(self, val):
        self._frequency_modulation = val
        self.apply_modulation_and_channel_settings()
    
    @property
    def channel(self):
        return self._channel

    @channel.setter
    def channel(self, val):
        self._channel = val
        self.apply_modulation_and_channel_settings()
    
    @property
    def hdr_mode(self):
        return self._hdr_mode
    
    @hdr_mode.setter
    def hdr_mode(self, val):
        self._hdr_mode = val
        print(f"Setting HDR Mode to {val}")
        self.tof.setHDRMode(val)

    @property
    def integration_time_tof_1(self):
        return self._integration_time_tof_1
    
    @integration_time_tof_1.setter
    def integration_time_tof_1(self, val):
        self._integration_time_tof_1 = val
        self.apply_integration_time_settings()

    @property
    def integration_time_tof_2(self):
        return self._integration_time_tof_2
    
    @integration_time_tof_2.setter
    def integration_time_tof_2(self, val):
        self._integration_time_tof_2 = val
        self.apply_integration_time_settings()

    @property
    def integration_time_tof_3(self):
        return self._integration_time_tof_3
    
    @integration_time_tof_3.setter
    def integration_time_tof_3(self, val):
        self._integration_time_tof_3 = val
        self.apply_integration_time_settings()
    
    @property
    def integration_time_tof_Gr(self):
        return self._integration_time_tof_Gr
    
    @integration_time_tof_Gr.setter
    def integration_time_tof_Gr(self, val):
        self._integration_time_tof_Gr = val
        self.apply_integration_time_settings()

    @property
    def min_amplitude(self):
        return self._min_amplitude
    
    @min_amplitude.setter
    def min_amplitude(self, val):
        self._min_amplitude = val
        print(f"Setting Min Amplitude to {val}")
        self.tof.setMinAmplitude(val)

    @property
    def lens_center_offset_x(self):
        return self._lens_center_offset_x

    @lens_center_offset_x.setter
    def lens_center_offset_x(self, val):
        self._lens_center_offset_x = val
        self.apply_lens_offset_settings()

    @property
    def lens_center_offset_y(self):
        return self._lens_center_offset_y

    @lens_center_offset_y.setter
    def lens_center_offset_y(self, val):
        self._lens_center_offset_y = val
        self.apply_lens_offset_settings()

    @property
    def roi_left_x(self):
        return self._roi_left_x

    @roi_left_x.setter
    def roi_left_x(self, val):
        self._roi_left_x = val
        self.apply_roi_settings()

    @property
    def roi_right_x(self):
        return self._roi_right_x

    @roi_right_x.setter
    def roi_right_x(self, val):
        self._roi_right_x = val
        self.apply_roi_settings()

    @property
    def roi_top_y(self):
        return self._roi_top_y

    @roi_top_y.setter
    def roi_top_y(self, val):
        self._roi_top_y = val
        self.apply_roi_settings()

    @property
    def roi_bottom_y(self):
        return self._roi_bottom_y

    @roi_bottom_y.setter
    def roi_bottom_y(self, val):
        self._roi_bottom_y = val
        self.apply_roi_settings()

    @property
    def image_type(self):
        return self._image_type

    @image_type.setter
    def image_type(self, val):
        self._image_type = val
        match val:
            case 0:
                print("Stopping Stream")
                self.tof.stopStream()
            case 1:
                print("Streaming Distance")
                self.tof.streamDistance()
            case 2:
                print("Streaming Distance/Amplitude")
                self.tof.streamDistanceAmplitude()

    def apply_roi_settings(self):
        self.tof.setRoi(self.roi_left_x, self.roi_top_y, self.roi_right_x, self.roi_bottom_y)

    def apply_modulation_and_channel_settings(self):
        print(f"Setting modulation to {self.frequency_modulation} and channel to {self.channel}")
        self.tof.setModulation(self.frequency_modulation, self.channel)

    def apply_integration_time_settings(self):
        print(f"Applying Integration Times: 1: {self.integration_time_tof_1} 2: {self.integration_time_tof_2} 3: {self.integration_time_tof_3} GR: {self.integration_time_tof_Gr}")
        self.tof.setIntegrationTime(self.integration_time_tof_1, self.integration_time_tof_2, self.integration_time_tof_3, self.integration_time_tof_Gr)

    def apply_lens_offset_settings(self):
        print(f"Applying Lens offset X: {self.lens_center_offset_x} Y: {self.lens_center_offset_y}")
        self.tof.setLensCenter(self.lens_center_offset_x, self.lens_center_offset_y)

    def apply_all_settings(self):
        self.apply_modulation_and_channel_settings()
        self.min_amplitude = self.min_amplitude
        self.apply_integration_time_settings()
        self.hdr_mode = self.hdr_mode
        self.apply_roi_settings()
        self.lens_type = self.lens_type
        self.apply_lens_offset_settings()
        self.tof.setFilter(0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.image_type = self.image_type

    def update_settings_from_json(self, settings_json):
        res: bool = True
        new_settings_dict: dict = json.loads(settings_json)
        for key, value in new_settings_dict.items():
            if hasattr(self, key):
                if getattr(self, key) != value:
                    setattr(self, key, value)
            else:
                print(f"I don't have an attribute named {key}")
                res = False
        return res

    def settings_as_dict(self) -> dict:
        return {"lens_type": self.lens_type,
                "frequency_modulation": self.frequency_modulation,
                "channel": self.channel,
                "image_type": self.image_type,
                "hdr_mode": self.hdr_mode,
                "integration_time_tof_1": self.integration_time_tof_1,
                "integration_time_tof_2": self.integration_time_tof_2,
                "integration_time_tof_3": self.integration_time_tof_3,
                "integration_time_tof_Gr": self.integration_time_tof_Gr,
                "min_amplitude": self.min_amplitude,
                "lens_center_offset_x": self.lens_center_offset_x,
                "lens_center_offset_y": self.lens_center_offset_y,
                "roi_left_x": self.roi_left_x,
                "roi_right_x": self.roi_right_x,
                "roi_top_y": self.roi_top_y,
                "roi_bottom_y": self.roi_bottom_y}

    def streamDistance(self):
        self.image_type = 1
        print("Starting Stream Distance")

    def streamStop(self):
        self.image_type = 0
        print("Stopping Stream")

    def setFrameCallback(self, callback):
        self.tof.subscribeFrame(callback)
