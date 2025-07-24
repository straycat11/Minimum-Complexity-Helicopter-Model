# logger.py
import csv

class Logger:
    def __init__(self, variable_names):
        """
        variable_names: List of strings, e.g., ["Time", "Altitude", "Acceleration"]
        """
        self.variable_names = variable_names
        self.data = []

    def log(self, **kwargs):
        """
        Pass keyword arguments matching variable names:
        logger.log(Time=t, Altitude=alt, Acceleration=acc)
        """
        row = [kwargs.get(name, None) for name in self.variable_names]
        self.data.append(row)

    def save(self, path):
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.variable_names)
            writer.writerows(self.data)
