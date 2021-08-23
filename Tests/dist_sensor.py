import time
import numpy as np
import VL53L0X
import RPi.GPIO as GPIO

dist_sensor=VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
servo_pin=4

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

def mean_distance():
	distance=0
	arr_dist=[0 for i in range(5)]
	for i in range(5):
		arr_dist[i]=dist_sensor.get_distance()
		if arr_dist[i]>1000:
			arr_dist[i]=1000
		distance=distance+arr_dist[i]
	return int(distance/5/10)
	
def sensor_dist():
	distance=dist_sensor.get_distance()
	if distance>1000:
		distance=1000
	return int(distance/10)
		
try:
	while True:
		for count in range(0,180):
			pwm_servo.ChangeDutyCycle(degrees[count])
			#time.sleep(timing/1000000.00)
			time.sleep(0.02)
			#print("Grade: "+str(count)+" ,distanta: "+str(mean_distance()))
			#print("Grade: "+str(count)+" ,distanta: "+str(dist_sensor.get_distance()))
			print("Grade: "+str(count)+" ,distanta: "+str(sensor_dist()))
		for count in range(179,0,-1):
			pwm_servo.ChangeDutyCycle(degrees[count])
			#time.sleep(timing/1000000.00)
			time.sleep(0.02)
			#print("Grade: "+str(count)+" ,distanta: "+str(mean_distance()))
			#print("Grade: "+str(count)+" ,distanta: "+str(dist_sensor.get_distance()))
			print("Grade: "+str(count)+" ,distanta: "+str(sensor_dist()))
			
	

except KeyboardInterrupt:
    print("CTRL-C: Terminating program.")
finally:
    print("Cleaning up GPIO...")
    pwm_servo.ChangeDutyCycle(CENTRE)
    time.sleep(0.5)
    GPIO.cleanup()
	
