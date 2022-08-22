from typing import Union, List, Tuple

import cv2
from numpy.typing import DTypeLike

from .calibration_helpers import *
from .cam_calibration import CamCalibration, CamFrameCalibrationData
from .origin_updater import OriginUpdater


class SystemCalibration:
    """ Handles calibration data of multiple cameras """

    max_cam_center_difference_for_consistency_mm = 0

    def __init__(self):
        self.cams = []

    def add_cam(self, cam: CamCalibration):
        """ Add CamCalibration object"""
        self.cams.append(cam)

    def get_cam(self, name: AnyStr):
        """ Given name of camera, return the CamCalibration object"""
        for cam in self.cams:
            if cam.name == name:
                return cam

    def check_consistency(self, cam_name1: AnyStr, cam_name2: AnyStr):
        """ Check consistency of calibration data between two cams.
        This is done by calculating rigid transformation for each of the frames between
        the cams. If the rigid transformations are inconsistent -> cam calibrations inconsistent"""

        cam1 = self.get_cam(cam_name1)
        cam2 = self.get_cam(cam_name2)
        tss = cam1.get_frame_system_timestamps()

        cam2_T_cam1 = None
        idx = 0
        next_shift = 1
        at_least_one_bad = False

        def closest_projected_point(data: CamFrameCalibrationData, real_point: Union[List, Tuple],
                                    cam_T_world: DTypeLike,
                                    K: DTypeLike):
            """ Find the closest seen calibration point to some real world point on a plane
            """

            # TODO(buq2): Distort points

            # Project real world point to image plane
            rp = np.array([real_point[0], real_point[1], 0, 1]).reshape([4, 1])
            pp = K @ cam_T_world[0:3, :] @ rp
            pp /= pp[-1]

            # Find the closest pixel value point to the projected point
            px = data.px
            py = data.py

            px = px - pp[0]
            py = py - pp[1]
            d = np.sqrt(px * px + py * py)

            idx = np.argmin(d)
            min_dist_pix = d[idx]

            rx_min_dist = data.rx[idx]
            ry_min_dist = data.ry[idx]

            # Return closest calibration point and it's distance to the real world point
            return (rx_min_dist, ry_min_dist), min_dist_pix

        def autofix(ts: datetime.datetime, cam2_T_cam1: DTypeLike):
            """ Predict where points are projected on second cam and try to fix origin and axis automatically """
            data1, _ = cam1.get_corner_data_near_timestamp(ts)
            data2, _ = cam2.get_corner_data_near_timestamp(ts)
            cam1_T_world, _ = cam1.get_cam_position_near_timestamp(ts)
            cam1_T_world = np.vstack([cam1_T_world, [0, 0, 0, 1]])
            cam2_T_world = cam2_T_cam1 @ cam1_T_world

            grid = estimate_calibration_grid_size(data2)

            # Generate grid of points to check
            # This is very lazy way to try to find three points that form origin and axis
            # It would be better to use prediction/heuristics
            [Y, X] = np.mgrid[-30:30, -30:30]
            X = X.astype(float)
            Y = Y.astype(float)
            X *= grid[0]
            Y *= grid[1]
            D = np.sqrt(X * X + Y * Y)
            idx = np.ravel(D).argsort()
            X = np.ravel(X)[idx]
            Y = np.ravel(Y)[idx]

            def try_to_fix(real_point: Union[List, Tuple]):
                closest_origin, dist_px = closest_projected_point(data2, real_point, cam2_T_world, cam2.K)
                # TODO(buq2): Use arg for max error
                if dist_px > 50:
                    return False
                closest_pos_x, dist_px = closest_projected_point(data2, (real_point[0] + grid[0], real_point[1]),
                                                                 cam2_T_world, cam2.K)
                if dist_px > 50:
                    return False
                closest_pos_y, dist_px = closest_projected_point(data2, (real_point[0], real_point[1] + grid[1]),
                                                                 cam2_T_world, cam2.K)
                if dist_px > 50:
                    return False

                # Looks good, change origin and axis
                real_point = np.array(real_point)
                closest_origin = np.array(closest_origin) - real_point
                closest_pos_x = np.array(closest_pos_x) - real_point
                closest_pos_y = np.array(closest_pos_y) - real_point
                cam2.set_new_real_origin_and_rotation(ts, closest_origin, closest_pos_x, closest_pos_y)

                return True

            for origin_idx in range(len(X)):
                x_real = X[origin_idx]
                y_real = Y[origin_idx]

                success = try_to_fix((x_real, y_real))
                if success:
                    print('Successfully fixed coordinate system (or did we?)')
                    return

        while (True):
            if idx > len(tss) and not at_least_one_bad:
                # No bad frames, prevent infinite loop
                break

            ts = tss[idx % len(tss)]
            cam2_T_world, diff2_s = cam2.get_cam_position_near_timestamp(ts)
            if cam2_T_world is None:
                # No pos for this timestamp
                idx += next_shift
                continue
            cam2_T_world = np.vstack([cam2_T_world, [0, 0, 0, 1]])

            cam1_T_world, diff1_s = cam1.get_cam_position_near_timestamp(ts)
            if cam1_T_world is None:
                # No pos for this timestamp
                idx += next_shift
                continue
            cam1_t_world = np.vstack([cam1_T_world, [0, 0, 0, 1]])

            cam1_T_cam2 = cam1_t_world @ np.linalg.pinv(cam2_T_world)

            if cam2_T_cam1 is None:
                # Set first known cam transformation
                cam2_T_cam1 = np.linalg.pinv(cam1_T_cam2)
                idx += next_shift
                continue

            cam2now_T_cam2first = cam2_T_cam1 @ cam1_T_cam2
            t_diff = cam2now_T_cam2first[0:3, -1]

            pos_change_mm = np.linalg.norm(t_diff)
            if pos_change_mm > self.max_cam_center_difference_for_consistency_mm:
                print(f'Possibly bad consistency between cams ({pos_change_mm:0.0f}mm). Please fix the origin and axis')
                orig_updater1 = OriginUpdater(cam1, ts)
                orig_updater1.start()
                orig_updater2 = OriginUpdater(cam2, ts)
                orig_updater2.start()

                print('Press:')
                print('esc - Stop processing image pairs')
                print('d - next image')
                print('a - previous image')
                print('w - autofix based on prediction')
                key = cv2.waitKey(0)
                if key == 27:
                    # esc
                    break
                elif chr(key % 255) == 'd':
                    next_shift = 1
                elif chr(key % 255) == 'a':
                    next_shift = -1
                elif chr(key % 255) == 'w':
                    autofix(ts, cam2_T_cam1)
                    idx -= next_shift  # stay at same loc

                at_least_one_bad = True
            idx += next_shift

        print('Saving modified axis')
        cam1.serialize_origin_changes(cam1.name + '_origin_changes.json')
        cam2.serialize_origin_changes(cam2.name + '_origin_changes.json')

        cv2.destroyAllWindows()

    def display_origins_between_cams(self, cam_name1: AnyStr, cam_name2: AnyStr):
        """ Visualize two images and the calibration data """

        cam1 = self.get_cam(cam_name1)
        cam2 = self.get_cam(cam_name2)
        tss = cam1.get_frame_system_timestamps()

        idx = 0
        next_shift = 1
        while (True):
            ts = tss[idx % len(tss)]
            cam2_T_world, diff2_s = cam2.get_cam_position_near_timestamp(ts)
            if cam2_T_world is None:
                # No pos for this timestamp
                idx += next_shift
                continue

            orig_updater1 = OriginUpdater(cam1, ts)
            orig_updater1.start()
            orig_updater2 = OriginUpdater(cam2, ts)
            orig_updater2.start()

            print('Press:')
            print('esc - Stop processing image pairs')
            print('d - next image')
            print('a - previous image')
            key = cv2.waitKey(0)
            if key == 27:
                # esc
                break
            elif chr(key % 255) == 'd':
                next_shift = 1
            elif chr(key % 255) == 'a':
                next_shift = -1
            idx += next_shift
            print('---')

        print('Saving modified axis')
        cam1.serialize_origin_changes(cam1.name + '_origin_changes.json')
        cam2.serialize_origin_changes(cam2.name + '_origin_changes.json')

        cv2.destroyAllWindows()

    def load_all_origin_changes(self):
        """ Load origin change data for all cams"""
        for cam in self.cams:
            try:
                cam.load_origin_changes(cam.name + '_origin_changes.json')
            except Exception as e:
                print('Failed to load origin changes for cam', cam.name, str(e))

