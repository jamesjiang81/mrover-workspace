#Automated data generator for rover.
import numpy as np
import matplotlib.pyplot as plt
from math import sin

#Set starting x, ending x, and number of data points.
xstart = input("Enter xstart: ")
xend = input("Enter xend: ")
numDataPoints = input("Enter Number Of Data Points: ")

#Making sure the ending x is not smaller than the starting x.
if(xend <= xstart):
	print("xend is smaller than xstart")
	quit()

#Create a list of the x data points, the actual y data, and the y data noise.
xdata = np.linspace(xstart, xend, num = numDataPoints)
ydataActual = []
ydataNoise = []

#For all the x data points calculate the actual y data points for the function.
#For all the x data points calculate the noise y data points for the function using the np.random.normal
#with the middle value being the standard deviation(the smaller the number the close the noise data is to the actual data).
for x in range(0, len(xdata)):
	y = sin(0.1*x) #function the data is being modeled after can be changed for different types of path (possibily add a way to input this).
	ydataActual.append(y)
	noise = np.random.normal(0,0.5,1)
	y += float(noise)
	ydataNoise.append(y)
	noise = 0;

#Print the x data and the y noise data.
print(xdata)
print(ydataNoise)

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








