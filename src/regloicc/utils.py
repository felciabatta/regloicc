"""Utility functions"""

from time import sleep

from rich.progress import Progress
from serial.tools import list_ports


def parse_none(arg, default):
    return default if arg is None else arg


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
