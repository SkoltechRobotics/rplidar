'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np

PORT_NAME = '/dev/ttyUSB0'
DMAX = 6000
IMIN = 0
IMAX = 50


def update(plot, scan):
    '''Updates plot'''
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    plot.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    plot.set_array(intens)
    plt.show()
    plt.pause(0.001)


def run():
    '''Main function'''
    plt.ion()
    lidar = RPLidar(PORT_NAME)
    subplot = plt.subplot(111, projection='polar')
    plot = subplot.scatter([0, 1], [0, 1], s=5, c=[IMIN, IMAX],
                           cmap=plt.cm.Greys_r, lw=0)
    subplot.set_rmax(DMAX)
    subplot.grid(True)
    plt.show()
    for scan in lidar.iter_scans():
        update(plot, scan)
        if not plt.get_fignums():
            break
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    run()
