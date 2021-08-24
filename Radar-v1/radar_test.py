import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

import time
import VL53L0X
import RPi.GPIO as GPIO

dist_sensor=VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
servo_pin=18

MIN_DUTY=1
MAX_DUTY = 11
CENTRE = MIN_DUTY + (MAX_DUTY - MIN_DUTY) / 2
duty_cycle=CENTRE
degrees=(np.linspace(1,11,num=180))

def init_sensor():
	global timing,pwm_servo
	dist_sensor.open()
	dist_sensor.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
	timing=dist_sensor.get_timing()  #din teste 66ms
	if timing<20000:
		timing=20000
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(servo_pin, GPIO.OUT)
	pwm_servo = GPIO.PWM(servo_pin, 50) #!!!! PWM FREQ
	pwm_servo.start(duty_cycle)
	print("Timing %d ms" % (timing/1000))
	
init_sensor()

def sensor_dist():
	distance=dist_sensor.get_distance()
	if distance>1000:
		distance=1000
	return int(distance/10)

fig=plt.figure(facecolor='k')
win=fig.canvas.manager.window #figure window
screen_res=win.wm_maxsize() #window formatting
dpi=150.0 #fig res
fig.set_dpi(dpi) #set fig res

#polar plot 
ax=fig.add_subplot(111,polar=True,facecolor='#094602')
ax.set_position([-0.05,-0.05,1.1,1.05])
r_max=100.0 #max measured distance
ax.set_ylim([0.0,r_max]) #radius from center
ax.set_xlim([0.0,np.pi]) #servo span 0-180
ax.tick_params(axis='both',colors='w')
ax.grid(color='w',alpha=0.5) #grid color
ax.set_rticks(np.linspace(0.0,r_max,5)) #show 5 different distances

ax.set_thetagrids(np.linspace(0.0,180.0,19)) #10 angles
angles=np.arange(0,181,1) #0-180 degrees
theta=angles*(np.pi/180.0) #radians
dists=np.ones((len(angles),)) #dummy dists
pols, = ax.plot([],linestyle='',marker='o',markerfacecolor = 'w',
                 markeredgecolor='#EFEFEF',markeredgewidth=1.0,
                 markersize=10.0,alpha=0.9) # dots for radar points
line1, = ax.plot([],color='w',
                  linewidth=4.0) # sweeping arm plot

#figure presentation 
fig.set_size_inches(0.96*(screen_res[0]/dpi),0.96*(screen_res[1]/dpi))
plot_res=fig.get_window_extent().bounds
win.wm_geometry('+{0:1.0f}+{1:1.0f}'.\
                format((screen_res[0]/2.0)-(plot_res[2]/2.0),
                       (screen_res[1]/2.0)-(plot_res[3]/2.0)))

fig.canvas.toolbar.pack_forget() #remove toolbar
fig.canvas.manager.set_window_title('Radar')

fig.canvas.draw()
axbackground=fig.canvas.copy_from_bbox(ax.bbox)

#button event to stop program
def stop_event(event):
	global stop_bool
	stop_bool=1
prog_stop_ax=fig.add_axes([0.85,0.025,0.125,0.05])
pstop=Button(prog_stop_ax,'Stop Program',color='#FCFCFC',hovercolor='w')
pstop.on_clicked(stop_event)

def close_event(event):
	global stop_bool,close_bool
	if stop_bool:
		plt.close('all')
	stop_bool=1
	close_bool=1
close_ax=fig.add_axes([0.025,0.025,0.125,0.05])
close_but=Button(close_ax,'Close Plot',color='#FCFCFC',hovercolor='w')
close_but.on_clicked(close_event)

fig.show()

stop_bool,close_bool=False,False
#infinite loop
while True:
	try:
		if stop_bool:
			fig.canvas.toolbar_pack_configure() #show toolbar
			if close_bool:
				plt.close('all')
			break
		for count in range(0,180):
			pwm_servo.ChangeDutyCycle(degrees[count])
			angle=count
			dist=sensor_dist()
			dists[int(angle)]=dist
			pols.set_data(theta,dists)
			fig.canvas.restore_region(axbackground)
			ax.draw_artist(pols)
			line1.set_data(np.repeat((angle*(np.pi/180.0)),2),np.linspace(0.0,r_max,2))
			ax.draw_artist(line1)
			fig.canvas.blit(ax.bbox)
			fig.canvas.flush_events()
		for count in range(179,0,-1):
			pwm_servo.ChangeDutyCycle(degrees[count])
			angle=count
			dist=sensor_dist()
			dists[int(angle)]=dist
			pols.set_data(theta,dists)
			fig.canvas.restore_region(axbackground)
			ax.draw_artist(pols)
			line1.set_data(np.repeat((angle*(np.pi/180.0)),2),np.linspace(0.0,r_max,2))
			ax.draw_artist(line1)
			fig.canvas.blit(ax.bbox)
			fig.canvas.flush_events()
			
	except KeyboardInterrupt:
		plt.close('all')
		#print('KeyboardInterrupt')
		print("CTRL-C: Terminating program.")
		break
	finally:
		print("Cleaning up GPIO...")
		pwm_servo.ChangeDutyCycle(CENTRE)
		time.sleep(0.5)
		GPIO.cleanup()
