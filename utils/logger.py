import csv

class Logger:
    def __init__(self):
        self.data = []

    def log(self, t, z, a):
        self.data.append((t, z, a))

    def save(self, path):
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Altitude", "Acceleration"])
            writer.writerows(self.data)
