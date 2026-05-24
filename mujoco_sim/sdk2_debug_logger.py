import csv
import os
import time


class CsvLogger:
    def __init__(self, path, fieldnames):
        self.path = path
        self.fieldnames = list(fieldnames)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.file = open(path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
        self.writer.writeheader()

    def write(self, row):
        safe_row = {key: row.get(key, "") for key in self.fieldnames}
        self.writer.writerow(safe_row)
        self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()


def wall_time_ns():
    return time.time_ns()


def add_vec(row, prefix, values, count=None):
    flat = list(values)
    if count is None:
        count = len(flat)
    for i in range(count):
        row[f"{prefix}_{i}"] = float(flat[i]) if i < len(flat) else ""


def vec_fields(prefix, count):
    return [f"{prefix}_{i}" for i in range(count)]

