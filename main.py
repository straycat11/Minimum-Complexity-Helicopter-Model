import time
from models.helicopter import Helicopter
from utils.logger import Logger
from utils.plotter import plot_altitude

heli = Helicopter()
logger = Logger()

dt = 0.1  # timestep (s)
t = 0
for i in range(200):
    z, a = heli.step(dt, CT=0.005)  # simulated thrust coefficient
    logger.log(t, z, a)
    t += dt
    time.sleep(0.01)

logger.save("data/flight_log.csv")
plot_altitude("data/flight_log.csv")
