"""Extract & log data streams for live experiments."""

import csv
import logging as pylog

from .timer import Timer


class CSVLogger:
    """Write structured records to a CSV file."""

    def __init__(self, path, fieldnames):
        self.path = path
        self.fieldnames = fieldnames

        self.file = None
        self.writer: csv.DictWriter | None = None

    def __enter__(self):
        self.file = open(self.path, "x", newline="")

        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)

        self.writer.writeheader()
        self.file.flush()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @property
    def writer_active(self):
        return self.writer is not None

    @property
    def n_fields(self):
        return len(self.fieldnames)

    def write(self, *args, **kwargs):
        record = dict() if not args else args[0]
        record |= kwargs
        if len(record) == self.n_fields and self.writer_active:
            written = self.writer.writerow(record)  # type: ignore
            self.flush()
            return written
        else:
            raise ValueError(
                f"Incorrect number of fields supplied ({len(record)})"
            )

    def flush(self):
        if self.file is not None:
            return self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None
            self.writer = None


class ElapsedTimeFilter(pylog.Filter):
    def __init__(self, timer: Timer):
        super().__init__()
        self.timer = timer

    def filter(self, record):
        record.elapsed = self.timer.elapsed
        return True
