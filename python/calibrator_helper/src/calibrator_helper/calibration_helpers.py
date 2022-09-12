import datetime
import os
import re
from typing import AnyStr

import numpy as np
from dateutil.parser import parse


def get_timestamp_from_filename(fname: AnyStr):
    """ Return datetime based on filename """
    # Remove extension
    fname = fname[:-4]
    # Remove non-numeric characters
    timestamp = re.sub("[^0-9.]+", "", fname)
    # Separete into date and time, as "parse" could not handle
    # these both at once
    date = timestamp[0:8]
    time = timestamp[8:8+13]
    # Parse
    d1 = parse(date, fuzzy=True)
    d2 = parse(time, fuzzy=True)
    # Set time
    d1 = d1.replace(hour=d2.hour, minute=d2.minute, second=d2.second, microsecond=d2.microsecond)
    return d1


def estimate_calibration_grid_size(data: "CamFrameCalibrationData", res=0.0001):
    """ Estimate calibration grid rectangle size """
    rx = np.round(data.rx/res)*res
    ry = np.round(data.ry/res)*res
    rx = np.unique(rx)
    ry = np.unique(ry)
    rx = np.sort(rx)
    ry = np.sort(ry)
    return np.min(np.diff(rx)), np.min(np.diff(ry))


def find_filename_closest_to_timestamp(fnames: AnyStr, timestamp: datetime.datetime):
    """ Find filename that has timestamp closest to given timestamp """
    closest_diff = np.inf
    closest_fname = None

    for fname in fnames:
        base = os.path.basename(fname)
        ts = get_timestamp_from_filename(base)
        diff = np.abs(timestamp - ts).total_seconds()

        if diff < closest_diff:
            closest_diff = diff
            closest_fname = fname

    return closest_fname, closest_diff
