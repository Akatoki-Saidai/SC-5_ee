import numpy as np
import math
import matplotlib.pyplot as plt

def caculate(init_velocity, degree):
	theta = math.radians(degree)
	g = 9.80665
	t = 2*init_velocity*math.sin(theta)/g
	x = init_velocity*math.cos(theta)
	y = (init_velocity*math.sin(theta))**2/(2*g)
	print("********************")
	print("Caculate Result")
	print("init_velocity: {}".format(init_velocity))
	print("theta: {}".format(theta))
	print("Max Height: {}".format(y))
	print("Distance: {}".format(x))
	print("Time: {}".format(t))
	print("********************")
	time = np.linspace(0,t,1000)
	height = init_velocity*math.sin(theta)*time - 0.5 *g * time**2
	plt.plot(time,height)
	plt.show()