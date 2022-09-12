from typing import AnyStr, List

import cv2
import numpy as np
from numpy.typing import DTypeLike


class MouseClickData:
    """ Holds mouse click data from OpenCV window """

    def __init__(self):
        self.click_posses = []
        self.click_callback_fun = None

    def add_click(self, x: float, y: float, button=None):
        """ Add new click location from window """
        self.click_posses.append([x, y])

        if self.click_callback_fun is not None:
            self.click_callback_fun(x, y, button=button)

    def get_last_click_position(self) -> List[float]:
        """ Return lastest click position """
        if len(self.click_posses) == 0:
            return None
        return self.click_posses[-1]


class DisplayMouseClickLocation:
    """ Helper for displaying last mouse location on opencv window/image """

    def __init__(self, img: DTypeLike, winname: AnyStr, mouse_click_data: MouseClickData = None):
        self.img = img
        self.winname = winname
        self.click_callback = None
        self.last_click = None
        self.title = ""
        if mouse_click_data is not None:
            mouse_click_data.click_callback_fun = lambda x, y, button: self.click(x, y, button)

    def click(self, x: float, y: float, button=None):
        """ Called when image is clicked """
        self.last_click = (x, y)
        self.draw()

        if self.click_callback is not None:
            self.click_callback(x, y, button)

    def draw(self):
        """ Redraw image, last click location and title of the image """
        img = np.copy(self.img)
        if self.last_click is not None:
            cv2.circle(img, self.last_click, 4, (0, 255, 255), 1)
        if self.title:
            scale = 1
            font = cv2.FONT_HERSHEY_SIMPLEX
            thickness = 1
            textsize = cv2.getTextSize(self.title, font, scale, thickness)[0]
            textX = (img.shape[1] - textsize[0]) / 2
            cv2.putText(img, self.title, (int(textX), int(textsize[1])), font, scale, (0, 255, 0), thickness,
                        cv2.LINE_AA)
        cv2.imshow(self.winname, img)

    def set_title(self, title: AnyStr):
        """ Set title that is displayed on top of the image """
        self.title = title
        self.draw()


def window_click_event(event, x: float, y: float, flags=None, userdata: MouseClickData = None):
    """ Called when opencv window is clicked """
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        userdata.add_click(x, y, button=event)

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:
        userdata.add_click(x, y, button=event)
