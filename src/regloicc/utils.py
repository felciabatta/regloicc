"""Utility functions for CBC"""

import os
from datetime import datetime
from pathlib import Path
from time import sleep

import xmltodict
import yaml
from numpy import sign
from rich.progress import Progress
from serial.tools import list_ports

DIR_DEFAULT = "~/Desktop/MM-Images"


def parse_none(arg, default):
    return default if arg is None else arg


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


def get_port(com=None, description=None):
    """Get a COM port from a list of available ports or string."""
    ports = list_ports.comports()
    if not ports:
        raise ValueError("No COM ports found. Please connect the pump.")

    if com is None and description is not None:
        for port in ports:
            if description in port.description:
                return port.device
        return get_port(com=None)  # fallback to user selection
    elif com is None:
        print("Available COM ports:")
        for p, port in enumerate(ports):
            print(f"{p}  {port.device}: {port.description}")
        print("")
        p = int(input("COM port not specified. Please select a port: "))
        return ports[p].device
    elif isinstance(com, str):
        return com
    else:
        raise ValueError("Invalid COM port spec. Use None or string.")


def flux2mmmmse(flowrate, from_unit="mL/min"):
    """Convert flowrate (flux) from vol/time to mmmmse format.

    :param flowrate:
    :param from_unit:

    :rtype: tuple[int, str, int]
    :returns:
        Flow rate tuple `(mmmm, "-", exponent)`.
    """
    v_unit, t_unit = from_unit.split("/", 1)
    time_factors = {"s": 1 / 60, "min": 1, "hr": 60, "hour": 60}
    vol_factors = {"uL": 1e-3, "µL": 1e-3, "mL": 1}
    if t_unit not in time_factors:
        raise ValueError("Invalid time unit. Use 's', min' or 'hr'.")
    if v_unit not in vol_factors:
        raise ValueError("Invalid volume unit. Use 'uL' or 'mL'.")
    flow_mL_min = flowrate * vol_factors[v_unit] / time_factors[t_unit]

    s = sign(flow_mL_min - 1)
    e = 0
    flow_scaled = flow_mL_min * 10 ** (-s * e)
    while not (1 <= flow_scaled < 10) and flow_mL_min != 0:
        e += 1
        flow_scaled = flow_mL_min * 10 ** (-s * e)
    flow_scaled *= 1000
    mmmm = int(min(round(flow_scaled), 9999))

    return (mmmm, "-" if s == -1 else "+", e)


def progress_timer(
    duration: int | float,
    step_seconds=1.0,
    description="Processing...",
    interrupt_message="process cancelled by user",
):
    duration = (
        round(duration / step_seconds)
        if isinstance(duration, float)
        else duration
    )
    try:
        with Progress() as p:
            n = int(duration)
            task = p.add_task(description, total=n)
            while not p.finished:
                p.update(task, advance=1, refresh=True)
                sleep(step_seconds)
    except KeyboardInterrupt:
        print(f"\nKeyboardInterrupt: {interrupt_message}\n")
        return False
    return True
