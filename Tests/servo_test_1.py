import RPi.GPIO as GPIO
import time
import numpy as np

MIN_DUTY = 1
MAX_DUTY = 11
CENTRE = MIN_DUTY + (MAX_DUTY - MIN_DUTY) / 2

servo_pin = 4
duty_cycle = CENTRE
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
pwm_servo = GPIO.PWM(servo_pin, 50)
pwm_servo.start(duty_cycle)
degrees=(np.linspace(1,11,num=180))
#print(degrees)

try:
	while True:
		for count in range(0,180):
			pwm_servo.ChangeDutyCycle(degrees[count])
			time.sleep(0.02)
		for count in range(179,0,-1):
			pwm_servo.ChangeDutyCycle(degrees[count])
			time.sleep(0.02)
			
	

except KeyboardInterrupt:
    print("CTRL-C: Terminating program.")
finally:
    print("Cleaning up GPIO...")
    pwm_servo.ChangeDutyCycle(CENTRE)
    time.sleep(0.5)
    GPIO.cleanup()
