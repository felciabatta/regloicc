"""Utility functions for CBC"""

import os
from datetime import datetime
from pathlib import Path
import xmltodict
import yaml

DIR_DEFAULT = "~/Desktop/MM-Images"


def is_image_present(dir_parent, img_name):
    full_path = os.path.join(dir_parent, img_name)
    if os.path.isfile(full_path):
        return True, full_path
    return False, None


def get_img_name(id_pos, interval_index, channel=0):
    return f"Position{id_pos:03d}--t{interval_index:02d}--C0{channel}.tif"


def get_latest_log_file(parent_dir=DIR_DEFAULT, date=None):
    """Get the latest log file for given date (YYYY-MM-DD).

    If date is None, use today's date.
    """
    date = datetime.now().strftime("%Y-%m-%d") if date is None else date
    files = os.listdir(parent_dir)
    logs = [f for f in files if f.startswith(date) and f.endswith(".csv")]
    logs.sort(reverse=True)  # latest log first
    return os.path.join(parent_dir, logs[0]) if logs else None


def get_next_log_filename(
    base_path="/mnt/c/Users/saram/OneDrive/Desktop/MM-imgs",
):
    """Generate log filename in format `YYYY-MM-DD-logXX.csv`"""
    date_str = datetime.now().strftime("%Y-%m-%d")
    for i in range(1, 100):  # Up to log99
        filename = f"{date_str}-log{i}.csv"
        full_path = os.path.join(base_path, filename)
        # CHECK EXISTING LOG FILES
        if not os.path.exists(full_path):
            return full_path
    raise Exception("Too many logs today!")


def parse_metadata(path, save_yaml=False):
    """Parse Leica LAS X .xlif (.xml) metadata files"""
    path = Path(path)
    data = xmltodict.parse(path.read_text())
    data = data["LMSDataContainerHeader"]["Element"]
    path_out = path.with_suffix(".yaml")
    yaml_str = yaml.dump(data, sort_keys=False)
    if save_yaml:
        path_out.write_text(yaml_str)
    return yaml_str
