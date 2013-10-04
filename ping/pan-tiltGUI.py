from Tkinter import *
import tkMessageBox
from hellousb import *
import time
from time import sleep
from threading import Thread

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.pyplot as plt
import numpy as np


root = Tk()
root.title("Pan-tilt GUI")

pic = hellousb()				#initializes USB peripheral

pan = IntVar()
tilt = IntVar()
d_range = 180					#range of degrees to display with slider
k = 65535/d_range				#maps angle to 16-bit servo position

mapsize = 20					#amount of rows/cols in the depthmap
Depthmap = [[0 for i in range(mapsize)] for j in range(mapsize)]

def move_servo(a):
	''' Callback for sliders'''
	A = pan.get()			
	B = tilt.get()
	pic.set_vals(k*A,k*B)		#vendor request

def getPixelCloud():
	''' Callback for auto-measurement'''
	for col in range(mapsize):
		for row in range(mapsize):
			''' Set the new coordinates for measurement '''
			pic.set_vals((65535 / 180 * 70) + (65535 / 180 * 110) / mapsize * col, 65535 / mapsize * row)
			
			sleep(0.7)
			''' execute measurement '''
			retval = pic.get_dist()
			
			''' Save in depthmap '''
			if retval[0] > 7000:
				retval[0] = 7000
			Depthmap[col][row] = (retval[0] - 53) / 14.7
	
	''' Do something with the depthmap... '''
	# Plot it for example?
	X = range(mapsize)
	Y = range(mapsize)
	X,Y = np.meshgrid(X,Y)
	print(X)
	print(Y)
	print(Depthmap)
	
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	
	surf = ax.plot_wireframe(X, Y, Depthmap, rstride=1, cstride=1)
		
	ax.set_zlim(0, 400)

	plt.show()
	
def updateMeasure(n):
	while True:
		retval = pic.get_dist()
		measurement.set((retval[0] - 53) / 14.7)
		time.sleep(1)

pan_slider = Scale(root, orient=HORIZONTAL, from_=0, to=d_range, label='Pan Angle', variable = pan, command = move_servo)
pan_slider.pack(anchor=CENTER)

tilt_slider = Scale(root, orient=HORIZONTAL, from_=0, to=d_range, label='Tilt Angle', variable = tilt, command = move_servo)
tilt_slider.pack(anchor=CENTER)

measurement = StringVar()
measure_lbl = Label(root, textvariable=measurement)
measure_lbl.pack(anchor=CENTER)

btn_auto = Button(root, anchor=CENTER, text='Auto-measurement', command=getPixelCloud)
btn_auto.pack(anchor=CENTER)

t = Thread(target=updateMeasure, args=(.01,))
t.start()

root.mainloop()

t.stop()
