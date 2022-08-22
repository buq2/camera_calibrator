import datetime
from typing import Callable

from .cam_calibration import CamCalibration
from .gui_helpers import *


class OriginUpdater:
    """ Helper for updating calibration origin """

    def __init__(self, cam: "CamCalibration", system_ts: datetime.datetime):
        self.cam = cam
        self.system_ts = system_ts
        self.click_data = None
        self.click_displayer = None
        self.scale = 0.5

    def set_next_callback(self, next_callback: Callable[[float, float], None]):
        """ Set which function should be called when image is clicked next time """
        self.click_displayer.click_callback = lambda x, y: next_callback(x, y)

    def set_title(self, title: AnyStr):
        """ Set title displayed on top of the image """
        self.click_displayer.set_title(title)

    def start(self):
        """ Start changing the origin """
        winname = self.cam.name
        img = self.cam.get_visualization_image(system_ts=self.system_ts, scale=self.scale)
        cv2.imshow(winname, img)
        self.click_data = MouseClickData()
        self.click_displayer = DisplayMouseClickLocation(img, winname, self.click_data)
        cv2.setMouseCallback(winname, window_click_event, self.click_data)

        self.set_next_callback(self.set_origin)
        self.set_title("Select new origin")

        self.origin = None
        self.positive_x = None
        self.positive_y = None

    def get_closest_real_point_to_image_point(self, xy: List) -> List[float]:
        """ Return closest world point to an image point """

        # Take scale into account
        xy = np.array(xy) / self.scale

        data, diff_s = self.cam.get_corner_data_near_timestamp(self.system_ts)
        px = data.px
        py = data.py

        img_points = np.vstack([px, py])
        img_diffs = img_points - np.array(xy).reshape([2, 1])
        diff_norms = np.linalg.norm(img_diffs, axis=0)
        closest_idx = np.argmin(diff_norms)

        closest_real_x = data.rx[closest_idx]
        closest_real_y = data.ry[closest_idx]
        return [closest_real_x, closest_real_y]

    def recalculate(self):
        """ Recalculate origin and rotation """
        current_origin = self.get_closest_real_point_to_image_point(self.origin)
        current_pos_x = self.get_closest_real_point_to_image_point(self.positive_x)
        current_pos_y = self.get_closest_real_point_to_image_point(self.positive_y)
        self.cam.set_new_real_origin_and_rotation(self.system_ts, current_origin, current_pos_x, current_pos_y)

        # Restart = redraw the image with new coordinate system
        self.start()

    def set_origin(self, x: float, y: float):
        self.origin = [x, y]
        self.set_next_callback(self.set_positive_x)
        self.set_title("Click direction of positive X")

    def set_positive_x(self, x: float, y: float):
        self.positive_x = [x, y]
        self.set_next_callback(self.set_positive_y)
        self.set_title("Click direction of positive Y")

    def set_positive_y(self, x: float, y: float):
        self.positive_y = [x, y]

        self.recalculate()
