#Automated data generator for rover.
import numpy as np
import matplotlib.pyplot as plt
from math import tan
import random

num_intervals = input('Number of intervals: ')
interval_size = input('Size of intervals (how many points per interval: ')
x_step = input('x step value within each interval: ')
init_x = input('Initial x value: ')
init_y = input('Initial y value: ')
seed = input('Seed for random path: ')

random.seed(seed)

pi = 3.14159265359



# retuns a touple of lists where tup[0] is the x values and tup[1] is the y values
def simulate_rover_path(init_theta, interval_size, num_intervals, x_step, init_x, init_y):
	#THETA SHOULD ALWAYS BE GREATER THAN 0 AND LESS THAN PI

	x_data[interval_size * num_intervals]
	y_data[interval_size * num_intervals]
	y_data[0] = init_y
	x_data[0] = init_x
	next_theta = init_theta
	for interval_num in range(0, num_intervals):
		curr_theta = next_theta

		y_change = 0.0
		# y_change is constant becuase the rover will be moving in a straight line
		if curr_theta < pi/2:
			y_change = x_step / tan(curr_theta)
		else: 
			y_change = x_step / tan(pi - curr_theta)
			y_change *= -1
		# generate linear data
		for interval_idx in range(1, interval_size):
			index = interval_size * interval_num + interval_idx
			x_data[index] = x_data[index - 1] + x_step
			y_data[index] = y_data[index - 1] + y_change

		# generate a random value for the standard deviation
		stdv = random.randomuniform(0.2, 1.2)
		# generate a random new theta with normal distribution from the current theta value
		# next_theta = min(max(0.1, random.normalvariate(curr_theta, stdv), 3.0)
		# for curve_x in range(0, transitions_size):
		# 	# genereate curve points
		# 	if curr_theta < pi/2:

		# 	else:

		curr_theta = min(max(0.1, random.normalvariate(curr_theta, stdv), 3.0)


#Create a list of the x data points, the actual y data, and the y data noise.
xdata = np.linspace(xstart, xend, num = numDataPoints)
ydataActual = []
ydataNoise = []


#This portion will help print the data to a file.
file = open("data.txt","w")
file.write(str(xstart))
file.write(" ")
file.write(str(xend))
file.write("\n")
file.write(str(numDataPoints))
file.write("\n")
for x in range(0,len(xdata)):
	file.write(str(ydataNoise[x]))
	file.write(" ")
file.close()

#Plot the noise data vs the actual data.
plt.figure(1)
plt.plot(xdata, ydataActual)
plt.scatter(xdata, ydataNoise)
plt.show()








