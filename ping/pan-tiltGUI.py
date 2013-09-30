from Tkinter import *
from hellousb import *

root = Tk()
root.title("Pan-tilt GUI")

pic = hellousb()				#initializes USB peripheral

pan = IntVar()
tilt = IntVar()
d_range = 180					#range of degrees to display with slider
k = 65535/d_range				#maps angle to 16-bit servo position

def move_servo(a):
	''' Callback for sliders'''
	A = pan.get()			
	B = tilt.get()
	pic.set_vals(k*A,k*B)		#vendor request

def getPixelCloud():
	''' Callback for auto-measurement'''
	for col in range(256):
		for row in range(256):
			pic.set_vals(256*col, 256*row)
			[colval, rowval, dist] = pic.get_dist()
			pixelmap[col, row] = dist
	
	


pan_slider = Scale(root, orient=HORIZONTAL, from_=0, to=d_range, label='Pan Angle', variable = pan, command = move_servo)
pan_slider.pack(anchor=CENTER)

tilt_slider = Scale(root, orient=HORIZONTAL, from_=0, to=d_range, label='Tilt Angle', variable = tilt, command = move_servo)
tilt_slider.pack(anchor=CENTER)

btn_auto = Button(root, anchor=CENTER, text='Auto-measurement', command=getPixelCloud)

root.mainloop()