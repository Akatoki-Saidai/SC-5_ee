import numpy as np
import matplotlib.pyplot as plt

past_distance = np.array([63.329,23.31,29.093,33.13,39.53])
init_velocity = np.array([60.89,22.41,29.84,35.89,46.78])

def trace(_theta,_initial_velocity):
    rad = np.deg2rad(_theta)
    y = []
    x = []
    t = 0
    dt = 0.0001
    g = 9.81
    while True:
        _y = _initial_velocity*np.sin(rad)*t-0.5*g*t**2
        _x = _initial_velocity*np.cos(rad)*t-0.5
        if _y < 0:
            dis = _x
            break
        y.append(_y)
        x.append(_x)
        t += dt
    return x, y, dis

x = []
y = []

# for i in range(len(past_distance)):
#     _x, _y = trace(75,init_velocity[i])
#     plt.plot(_x,_y)
_x,_y,dis = trace(75,34)
plt.plot(_x,_y)
print(dis)
plt.show()