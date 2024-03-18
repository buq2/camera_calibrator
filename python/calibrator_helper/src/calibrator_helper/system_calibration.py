from typing import Union, List, Tuple

import cv2
import numpy as np
import pycalibrator
from numpy.typing import DTypeLike

from .calibration_helpers import *
from .cam_calibration import CamCalibration, CamFrameCalibrationData, CamCalibrationData
from .origin_updater import OriginUpdater
from scipy.spatial.transform import Rotation


class SystemCalibration:
    """ Handles calibration data of multiple cameras """

    max_cam_center_difference_for_consistency_mm = 0

    def __init__(self):
        self.cams = []
        # rig_T_cam for each cam
        self.rig_T_cam = {}

    def set_rig_T_cam(self, cam_name: AnyStr, rig_T_cam: DTypeLike):
        self.rig_T_cam[cam_name] = rig_T_cam

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

            # Project real world point to image plane
            rp = np.array([real_point[0], real_point[1], 0, 1]).reshape([4, 1])
            pp = cam_T_world[0:3, :] @ rp
            pp /= pp[-1]

            # Find the closest pixel value point to the projected point
            px = data.px_ud
            py = data.py_ud

            px = (px - pp[0]) * K[0,0]
            py = (py - pp[1]) * K[1,1]
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
                if dist_px > 20:
                    return False
                closest_pos_x, dist_px = closest_projected_point(data2, (real_point[0] + grid[0], real_point[1]),
                                                                 cam2_T_world, cam2.K)
                if dist_px > 20:
                    return False
                closest_pos_y, dist_px = closest_projected_point(data2, (real_point[0], real_point[1] + grid[1]),
                                                                 cam2_T_world, cam2.K)
                if dist_px > 20:
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

        scale_cam1 = 0.5
        scale_cam2 = 0.5
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
            cam1_T_world = np.vstack([cam1_T_world, [0, 0, 0, 1]])

            cam1_T_cam2 = cam1_T_world @ np.linalg.pinv(cam2_T_world)

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
                orig_updater1 = OriginUpdater(cam1, ts, scale=scale_cam1)
                orig_updater1.start()
                orig_updater2 = OriginUpdater(cam2, ts, orig_updater1, scale=scale_cam2)
                orig_updater2.start()
                orig_updater1.origin_updater_cam2 = orig_updater2

                print('Press:')
                print('esc - Stop processing image pairs')
                print('d - next image')
                print('a - previous image')
                print('w - autofix based on prediction')
                print('s - set as cam2_T_cam1')
                print('j - shift one grid left')
                print('l - shift one grid right')
                print('i - shift one grid up')
                print('k - shift one grid down')
                print('1 - less zoom on cam 1')
                print('2 - more zoom on cam 1')
                print('3 - less zoom on cam 2')
                print('4 - more zoom on cam 2')
                print('p - print cam positions')
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
                elif chr(key % 255) == 's':
                    cam2_T_cam1 = np.linalg.pinv(cam1_T_cam2)
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == 'j':
                    cam2.shift_grid_by_square(ts, [-1, 0])
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == 'l':
                    cam2.shift_grid_by_square(ts, [1, 0])
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == 'i':
                    cam2.shift_grid_by_square(ts, [0, 1])
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == 'k':
                    cam2.shift_grid_by_square(ts, [0, -1])
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == '1':
                    scale_cam1 *= 0.75
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == '2':
                    scale_cam1 /= 0.75
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == '3':
                    scale_cam2 *= 0.75
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == '4':
                    scale_cam2 /= 0.75
                    idx -= next_shift  # stay at same loc
                elif chr(key % 255) == 'p':
                    print('cam1_T_world',cam1.name)
                    print(cam1_T_world)
                    print('cam2_t_world',cam2.name)
                    print(cam2_T_world)
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
            print('w - print filenames')
            key = cv2.waitKey(0)
            if key == 27:
                # esc
                break
            elif chr(key % 255) == 'd':
                next_shift = 1
            elif chr(key % 255) == 'a':
                next_shift = -1
            elif chr(key % 255) == 'w':
                next_shift = 0
                img_name, _ = cam1.get_img_fname_closest_to_system_timestamp(ts)
                print(img_name)
                img_name, _ = cam2.get_img_fname_closest_to_system_timestamp(ts)
                print(img_name)


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

    def get_transformation_between_cams_direct(self, cam1_name, cam2_name):
        """ Calculate average transformation between cameras. Only uses
        frames where points can be seen between cams. """

        cam1 = self.get_cam(cam1_name)
        cam2 = self.get_cam(cam2_name)

        t_sum = np.zeros((3,1))
        Rs = []
        for ts in cam1.get_frame_system_timestamps():
            # Try to get cam positions for both cam1 and cam2
            cam2_T_world, diff2_s = cam2.get_cam_position_near_timestamp(ts)
            if cam2_T_world is None:
                continue
            cam2_T_world = np.vstack([cam2_T_world, [0, 0, 0, 1]])

            cam1_T_world, diff1_s = cam1.get_cam_position_near_timestamp(ts)
            cam1_T_world = np.vstack([cam1_T_world, [0, 0, 0, 1]])

            # Transformation between cams
            cam1_T_cam2 = cam1_T_world @ np.linalg.pinv(cam2_T_world)

            # Just take the translation and rotation vector
            t = cam1_T_cam2[0:3, -1]
            R = cam1_T_cam2[0:3, 0:3]

            # Sum translations, append rotations into a list
            t_sum += t.reshape((3,1))
            Rs.append(Rotation.from_matrix(R))

        if len(Rs) == 0:
            return None

        # Average translation is ok (or should we take the median?)
        t_avg = t_sum / len(Rs)
        # Average rotations (I think this uses slerp)
        R_avg = Rotation.mean(Rotation.concatenate(Rs)).as_matrix()

        return np.hstack([R_avg, t_avg])

    def calculate_rig_T_cam_from_anchor(self, cam1_name:AnyStr, cam2_name:AnyStr):
        rig_T_cam1 = self.rig_T_cam[cam1_name]
        cam1_T_cam2 = self.get_transformation_between_cams_direct(cam1_name, cam2_name)
        cam1_T_cam2 = np.vstack([cam1_T_cam2,[0,0,0,1]])
        rig_T_cam2 = rig_T_cam1 @ cam1_T_cam2
        self.rig_T_cam[cam2_name] = rig_T_cam2

    def calibrate_extrinsics(self):
        """ Calibrate extrinsics of all cams """

        # Gather all timestamps
        all_tss = []
        for cam in self.cams:
            tss = cam.get_frame_system_timestamps()
            all_tss.extend(tss)

        # Remove timstamps too close together
        threshold = CamCalibrationData.max_time_diff_seconds
        all_tss.sort()
        all_tss_seconds = np.array([ts.timestamp() for ts in all_tss])
        diff = np.empty(all_tss_seconds.shape)
        diff[0] = np.inf  # always retain the 1st element
        diff[1:] = np.diff(all_tss_seconds)
        mask = diff > threshold
        all_tss = np.array(all_tss)[mask]

        extcalib = pycalibrator.ExtrinsicsCalibrator()

        for idx, cam in enumerate(self.cams):
            rig_T_cam = self.rig_T_cam[cam.name]
            cam_T_rig = np.linalg.pinv(rig_T_cam)
            freeze = idx == 0
            extcalib.AddCameraTRig(cam_T_rig, freeze)

        def get_rig_T_world(ts):
            for cam in self.cams:
                cam_T_world, _ = cam.get_cam_position_near_timestamp(ts)
                if cam_T_world is None:
                    continue
                cam_T_world = np.vstack([cam_T_world,[0,0,0,1]])

                rig_T_cam = self.rig_T_cam[cam.name]
                rig_T_world = rig_T_cam @ cam_T_world

                # TODO: We could average out the transformations from all cams
                return rig_T_world

        for ts in all_tss:
            rig_T_world = get_rig_T_world(ts)
            frame_id = extcalib.AddObservationFrame(rig_T_world)

            world_point_ids = {} # tuple of world point coordinates -> id
            for cam_id, cam in enumerate(self.cams):
                data, diff_s = cam.get_corner_data_near_timestamp(ts)
                if data is None:
                    # No data from this cam in this frame
                    continue

                print(f'Frame {frame_id}, cam {cam_id} adding {len(data.rx)} points')

                # Add each of the points
                for p_idx in range(len(data.rx)):
                    rx = data.rx[p_idx]
                    ry = data.ry[p_idx]
                    world_point = (rx, ry, 0)
                    if world_point in world_point_ids:
                        world_point_id = world_point_ids[world_point]
                    else:
                        world_point_id = extcalib.AddWorldPoint(frame_id, np.array(world_point).reshape([3,1]))
                        world_point_ids[world_point] = world_point_id
                    px = data.px_ud[p_idx] # data.px[p_idx]
                    py = data.py_ud[p_idx] # data.py[p_idx]
                    extcalib.AddObservation(cam_id, world_point_id, np.array([px, py]))

        extcalib.Optimize()

        extcalib.Serialize('optimized_results.json')

        # Set optimized rig_T_cam
        for cam_id, cam in enumerate(self.cams):
            cam_T_rig = extcalib.GetCameraTRig(cam_id)
            rig_T_cam = np.linalg.pinv(cam_T_rig)
            self.rig_T_cam[cam.name] = rig_T_cam