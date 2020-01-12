import cv2
import pyrealsense2 as rs
from color_tracker.utils.camera.base_camera import Camera
from builtins import super
import numpy as np


class IntelCamera(Camera):
    """
    Simple Webcamera
    """

    def __init__(self, start = False):
        """
        :param video_src (int): camera source code. It can be an integer or the name of the video file.
        """

        super().__init__()

        if start:
            self.start_camera()

    def _init_camera(self):
        super()._init_camera()
        # Create a pipeline
        pipeline = rs.pipeline()

        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)

        self._cam = pipeline

        self._ret, self._frame = self._read_from_camera()
        if not self._ret:
            raise Exception("No camera feed")
        self._frame_height, self._frame_width, c = self._frame.shape
        return self._ret

    def _read_from_camera(self):
        super()._read_from_camera()
        frames = self._cam.wait_for_frames()
        color_frame = frames.get_color_frame()

        if color_frame:
            self._frame = np.asanyarray(color_frame.get_data())
            return True, self._frame
        else:
            return False, None

    def release(self):
        super().release()
        self._cam.stop()
