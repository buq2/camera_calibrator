import abc
import json
from typing import Union, List

import cv2
import pycalibrator
from numpy.typing import DTypeLike

from .calibration_helpers import *


class CamFrameCalibrationData:
    """ Holds calibration grid coordinates (rx, ry) and projections of
    the grid coordinates (px, py) """

    def __init__(self):
        self.rx: DTypeLike = np.array([])
        self.ry: DTypeLike = np.array([])
        self.px: DTypeLike = np.array([])
        self.py: DTypeLike = np.array([])


class CamCalibrationData:
    """ Holds calibration data from multiple frames for single camera """
    max_time_diff_seconds = 0.05

    def __init__(self):
        self.calibration_data = []

    def number_of_frames(self) -> int:
        """ Return number of frames held """
        return len(self.calibration_data)

    def add_frame(self, camera_ts: datetime.datetime, data: CamFrameCalibrationData):
        """ Add frame calibration data with timestamp (in cam timeframe) """
        self.calibration_data.append({'ts': camera_ts, 'data': data})

    def get_corner_data_near_timestamp(self, cam_ts: datetime.datetime) -> [Union[CamFrameCalibrationData, None],
                                                                            Union[float, None]]:
        """ Return calibration data near timestamp """
        closest_diff = np.inf
        out_data = None

        for ts_data in self.calibration_data:
            timestamp = ts_data['ts']
            diff_s = np.abs(timestamp - cam_ts).total_seconds()

            if diff_s < closest_diff:
                closest_diff = diff_s
                out_data = ts_data['data']

        if closest_diff > self.max_time_diff_seconds:
            return None, None

        return out_data, closest_diff

    def get_frame_timestamps(self) -> List[datetime.datetime]:
        """ Return all timestamps from known frames """
        return [frame['ts'] for frame in self.calibration_data]


class CamCalibration:
    """ Holds calibration data for single cam """
    max_time_diff_seconds = 0.05

    def __init__(self, name: AnyStr):
        self.name = name

        # Time offset from system timestamp to cam timestamp
        self.time_offset_seconds = 0.0
        self.K = None
        self.K_inv = None
        self.distortion_coefficients = None
        self.width = 2456
        self.height = 2058
        self.new_origin_changes = []
        self.corner_points = CamCalibrationData()

    def serialize_origin_changes(self, fname: AnyStr):
        """ Serialize origin changes to a file.
        Origin changes are modifications to original rx,ry data"""
        json.dump(self.new_origin_changes, open(fname, 'w'), default=str)

    def load_origin_changes(self, fname: AnyStr):
        """ Load serialized origin changes and apply them to calibration data. """
        new_origin_changes = json.load(open(fname, 'r'))
        for change in new_origin_changes:
            # Convert string time to datetime
            change['system_ts'] = datetime.datetime.strptime(change['system_ts'], '%Y-%m-%d %H:%M:%S.%f')

            def convert_if_str(data):
                if isinstance(data, str):
                    return np.fromstring(data[1:-1], dtype=float, sep=' ')
                return data

            change['new_origin'] = convert_if_str(change['new_origin'])
            change['pos_x'] = convert_if_str(change['pos_x'])
            change['pos_y'] = convert_if_str(change['pos_y'])

            self.set_new_real_origin_and_rotation(**change)

    def set_new_real_origin_and_rotation(self, system_ts: datetime.datetime, new_origin: List[float], pos_x: List[float],
                                         pos_y: List[float]):
        """ Set new origin and rotation (new x- and y-axis) for frame with certain timestamp """
        # Add new origin set to array for serialization
        self.new_origin_changes.append({
            'system_ts': system_ts,
            'new_origin': new_origin,
            'pos_x': pos_x,
            'pos_y': pos_y
        })

        new_origin = np.array(new_origin)
        pos_x = np.array(pos_x)
        pos_y = np.array(pos_y)

        frame_data, diff_s = self.get_corner_data_near_timestamp(system_ts)
        if diff_s > CamCalibration.max_time_diff_seconds:
            raise RuntimeError(f'Could not find calibration data for timestamp {system_ts}')

        # Calculate rotation matrix from given axis
        x_axis = pos_x - new_origin
        y_axis = pos_y - new_origin
        x_axis /= np.linalg.norm(x_axis)
        y_axis /= np.linalg.norm(y_axis)
        R = np.vstack([x_axis, y_axis]).T

        rx = frame_data.rx - new_origin[0]
        ry = frame_data.ry - new_origin[1]

        frame_data.rx = R[0, 0] * rx + R[0, 1] * ry
        frame_data.ry = R[1, 0] * rx + R[1, 1] * ry

    def get_frame_system_timestamps(self) -> List[datetime.datetime]:
        """ Get all timestamps for known calibration data """
        return [self.cam_timestamp_to_system_timestamp(ts) for ts in self.corner_points.get_frame_timestamps()]

    def system_timestamp_to_cam_timestamp(self, system_ts: datetime.datetime) -> datetime.datetime:
        """ Convert system timestamp to camera timestamp based on saved offset """
        return system_ts + datetime.timedelta(seconds=self.time_offset_seconds)

    def cam_timestamp_to_system_timestamp(self, cam_ts: datetime.datetime) -> datetime.datetime:
        """ Convert camera timestamp to system timestamp based on saved offset """
        return cam_ts - datetime.timedelta(seconds=self.time_offset_seconds)

    @abc.abstractmethod
    def get_img_fname_closest_to_system_timestamp(self, system_ts: datetime.datetime) -> [Union[AnyStr, None],
                                                                                          Union[AnyStr, None]]:
        """ Return image filename closest to system timestamp """
        return None, None

    def get_corner_data_near_timestamp(self, system_ts: datetime.datetime) -> [Union[CamFrameCalibrationData, None],
                                                                               Union[float, None]]:
        """ Return corner data near system timestamp"""
        ts = self.system_timestamp_to_cam_timestamp(system_ts)
        return self.corner_points.get_corner_data_near_timestamp(ts)

    def get_cam_position_near_timestamp(self, system_ts: datetime.datetime) -> DTypeLike:
        """ Get camera position (cam_T_world) near system timestamp """
        data, diff_s = self.get_corner_data_near_timestamp(system_ts)
        if data is None:
            return None, None

        px = data.px
        py = data.py
        rx = data.rx
        ry = data.ry

        imgp = np.vstack([px, py]).T
        realp = np.vstack([rx, ry]).T
        H = pycalibrator.EstimateHomography(realp, imgp)
        R, t = pycalibrator.RecoverExtrinsics(self.K_inv, H)
        return np.hstack([R, t.reshape([3, 1])]), diff_s

    def get_visualization_image(self, system_ts: datetime.datetime, scale: float = 1.0) -> DTypeLike:
        """ Get visualization image near system timestamp """
        img_name, _ = self.get_img_fname_closest_to_system_timestamp(system_ts)

        if not os.path.exists(img_name):
            # Maybe just the direct image name without path
            img_fname = os.path.join(self.image_directory, img_name.strip())
        else:
            img_fname = img_name

        img = cv2.imread(img_fname, -1)  # -1 for 16bit/original
        img = (img / 6).astype(np.uint8)  # to uint8 with scaling

        # To rgb for plotting
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        # Get corresponding data for this image
        cor_data, _ = self.get_corner_data_near_timestamp(system_ts)

        rx = cor_data.rx
        ry = cor_data.ry
        px = cor_data.px
        py = cor_data.py

        def draw_axis():
            """ Draw axis near origin """

            def find_index(real_point):
                """ Find index of world point """
                drx = rx - real_point[0]
                dry = ry - real_point[1]
                d = np.sqrt(drx * drx + dry * dry)
                idx = d.argmin()
                return idx

            # Try to draw axis
            grid = estimate_calibration_grid_size(cor_data)
            idx_orig = find_index((0, 0))
            idx_pos_x = find_index((grid[0], 0))
            idx_pos_y = find_index((0, grid[1]))
            orig = (int(px[idx_orig]), int(py[idx_orig]))
            pos_x = (int(px[idx_pos_x]), int(py[idx_pos_x]))
            pos_y = (int(px[idx_pos_y]), int(py[idx_pos_y]))
            thickness = 6
            cv2.line(img, orig, pos_x, (255, 255, 0), thickness)
            cv2.line(img, orig, pos_y, (255, 0, 255), thickness)

        draw_axis()

        for idx in range(len(px)):
            pos_px = int(px[idx])
            pos_py = int(py[idx])

            pos_rx = rx[idx]
            pos_ry = ry[idx]

            center = (pos_px, pos_py)
            cv2.circle(img, center, 4, (0, 0, 255), 1)
            cv2.putText(img, "{:0.0f}, {:0.0f}".format(pos_rx, pos_ry), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 0), 2,
                        cv2.LINE_AA)

        if scale != 1.0:
            img = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))

        return img

    def optimize_intrinsics(self):
        """ Optimize camera intrinsics based on calibration data """
        # Collect image and world points
        img_points = []
        world_points = []
        for data_ts in self.corner_points.calibration_data:
            data = data_ts['data']
            px = data.px
            py = data.py
            rx = data.rx
            ry = data.ry

            imgp = np.vstack([px, py])
            realp = np.vstack([rx, ry, np.zeros_like(rx)])
            img_points.append(imgp.T)
            world_points.append(realp.T)

        calibrator = pycalibrator.Calibrator(self.width, self.height)
        calibrator.ForceDistortionToConstant(4)  # 4 = radial 3
        calibrator.Estimate(img_points, world_points)

        self.K = calibrator.GetK()
        self.K_inv = np.linalg.inv(self.K)
        self.distortion_coefficients = calibrator.GetDistortion()
