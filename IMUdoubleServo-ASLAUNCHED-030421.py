import logging
import sys
import math
import time
import pigpio
import RPi.GPIO as GPIO

from Adafruit_BNO055 import BNO055


#### initiate servos #####
servo1 = 18 #### servo1 is the X-servo
servo2 = 23 #### servo2 is the Y-servo
pwm = pigpio.pi()
pwm.set_mode( servo1, pigpio.OUTPUT)
pwm.set_mode( servo2, pigpio.OUTPUT)

pwm.set_PWM_frequency( servo1, 50 )
pwm.set_PWM_frequency( servo2, 50 )

# servo1 center=1160, servo2=1390 measured with laser
# servo1 moves to +x by 0.033deg/count
# servo2 moves to +y by -0.03deg/count
maxDegreesX = 5.5  #how far in degrees are servos allowed to move
maxDegreesY = 6
#servo1, Low = 1050, Mid = 1160, High = 1350 
pwm.set_servo_pulsewidth(servo1, 1160)

#servo2, Low = 1200, Mid = 1390, High = 1700 
pwm.set_servo_pulsewidth(servo2, 1475)

# define min/max/CenterVal2
minVal1 = int( 1160 - maxDegreesX/0.033)  #993
maxVal1 = int( 1160 + maxDegreesX/0.033)  #1327
centerVal1 = (maxVal1+minVal1)/2
rangeVal1 = (maxVal1-minVal1)/2
pwm.set_servo_pulsewidth(servo1, centerVal1)

minVal2 = int( 1475 - maxDegreesY/0.03)  #1275
maxVal2 = int( 1475 + maxDegreesY/0.03)  #1675
centerVal2 = (maxVal2+minVal2)/2
rangeVal2 = (maxVal2-minVal2)/2
pwm.set_servo_pulsewidth(servo2, centerVal2)
#time.sleep(99)
###### done initaiting servos #######

f = open("IMU.txt", "a")
f.write("-------------------------")
f.write(time.strftime("      %a %d-%m-%Y @ %H:%M:%S") )
f.write("\n")

g = open("IMUbackup.txt", "a")
g.write("-------")
g.write(time.strftime("      %a %d-%m-%Y @ %H:%M:%S") )
g.write("\n")


current_milli_time = lambda: int(round(time.time() * 1000))  #prints out time in milliseconds; current_milli_time()
print("current_milli_time= " + str(current_milli_time() ) )

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=6)


###### This section changes the accelerometer range to be 16G #######
### based on https://forums.adafruit.com/viewtopic.php?f=1&t=85097
### Notes:
### 0x08 = BNO055_ACC_CONFIG_ADDR
### 0x07 = BNO055_PAGE_ID_ADDR
### default accelerometer setting is 4G, last two bits of register 08 is 01
### change to last two bits being 11. Data sheet here, look for Table 3-8
### https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf 
###
time.sleep(0.1)
savePageID = bno._read_byte(0x07)
time.sleep(0.1)
bno._write_byte(0x07, 0x01)     
time.sleep(0.1)
print bno._read_byte(0x07)      
time.sleep(0.1)
print bno._read_byte(0x08)   
time.sleep(0.1)
bno._write_byte(0x08, 0xFF)       
time.sleep(0.1)
print bno._read_byte(0x08) 
time.sleep(0.1)
print "got"
bno._write_byte(0x07, savePageID & 0xFF)
print "here"
######## End section on changing accelerometer range #########


# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    f.write('Failed toinitialize BNO055')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
#f.write('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
#f.write('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
#f.write('\n')

# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
    #f.write('System error: {0}'.format(error))
    #f.write('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')

print('starting 60 seconds with no logging')
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)    #if using same script as IMU, this must be BCM. Can be Board for Pin number otherwise
GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)  #pin 38=GPIO20, MOSFET gate for ejection charge
print('Ejection Charge Set Low')

#make n circles. when viewed from below, this should trace out a counterclockwise circle
for n in range(0):
	#blah
	print("hi")
	num_points = 50
	for increment in range(num_points):
		angle = 2*3.14*increment/num_points
		servoXval = centerVal1 + rangeVal1*math.cos( angle )
		servoYval = centerVal2 + rangeVal2*math.sin( angle )
		pwm.set_servo_pulsewidth( servo1, servoXval)
		pwm.set_servo_pulsewidth( servo2, servoYval)
		time.sleep(0.03)




###  calibrate loop #########
for n in range(85):
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    		heading, roll, pitch, sys, gyro, accel, mag))

    time.sleep(1)


f.write("Latest Calibration Data\n")
f.write('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}\n'.format(
                heading, roll, pitch, sys, gyro, accel, mag))
g.write("Latest Calibration Data\n")
g.write('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}\n'.format(
                heading, roll, pitch, sys, gyro, accel, mag))

print("We should be calibrated on the IMU now, if not restart")
time.sleep(1)
print("You should be putting the rocket on the launchpad now with igniter installed")


while( True ):
	print("Confirm calibration or kill the script. Rocket should be on the launchpad at the proper angle now")
        userInput = str( input("Enter 2 to continue:  ") )
        if( userInput =='2' ):
                break

#if we want to offset any IMU angle bias this is where we would do it....
heading, roll, pitch = bno.read_euler()
#######
if( pitch > 0 ):
    pitchVal = 180 - pitch
else:
    pitchVal = -180 - pitch
desiredPitch = pitchVal      #this is where we want to point
desiredRoll = roll           #desired roll angle
print("Desired pitch = {0:0.2F} deg, and Desired roll = {1:0.2F} degrees".format(desiredPitch, desiredRoll) )

while( True ):
        print("Confirm orientation or kill the script")
        userInput = str( input("Enter 2 to continue:  ") )
        if( userInput =='2' ):
                break




print("Fire when ready - no more messages will be given.")

# wait for launch and then do control loop
maxAZ = -14      #this is not a light touch, but does require a bump up or down
#maxAZ = 20 #20 should not be used - this jsut bipasses the loop for testing purposes
gx = 0
gy = 0
gz = 0  #gyroscope values
while True:
	time.sleep(0.02)
	ax,ay,az = bno.read_accelerometer()
	f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
	#g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
	#print('Waiting. az = {0:0.2F}'.format(az) )
	if( az < maxAZ):
		# there is an event happening. to make sure its not jitter, check it again
		time.sleep(0.03)
		ax,ay,az = bno.read_accelerometer()
		f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
		#g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
		if( az < maxAZ ):
			#sustained. check it again
			time.sleep(0.03)
			ax,ay,az = bno.read_accelerometer()
			if( az<maxAZ):
				f.write('Launch Detected\n')
				f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
                                g.write('Launch Detected\n')
                                #g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, current_milli_time(), centerVal1, centerVal2) )
				
				#ok calling it a launch now
				maxAZ = az
				#print('MaxAZ = {0}'.format(maxAZ) )
				break

f.close()
f = open("IMU.txt", "a")
g.close()
g = open("IMUbackup.txt", "a")
# at this point we are 0.08-0.11s into launch ignition.
# estes peaks at 0.18s and that's when I think we should be stearing
##################### end of launch loop ########


#### main control loop
#print('starting main control loop')
time_old = current_milli_time()
time_start = time_old           #we are about 0.2seconds into the burn at this point
Kp = 45*3.14/180.
Kd = 10*3.14/180.
Ki = 0.1*3.14/180.
roll_integral = 0
roll_old = 0
servoYval = centerVal2
rollSetpoint = 0
pitch_old = 0
pitch_integral = 0
ejectionfirecheck = True      #this is a flag to see if we have fired the ejection charge
while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    # sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.

    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    #	heading, roll, pitch, sys, gyro, accel, mag))
    # Other values you can optionally read:
    # Orientation as a quaternion:
    #x,y,z,w = bno.read_quaterion()
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    #print('Temperature = {0:0.2F}'.format(temp_c) )
    # Magnetometer data (in micro-Teslas):
    #mx,my,mz = bno.read_magnetometer()
    #print('Magnetometer x,y,z = {0:0.2F}, {1:0.2F}, {2:0.2F}'.format(x,y,z) )
    # Gyroscope data (in degrees per second):
    gx,gy,gz = bno.read_gyroscope()
    #print('Gyroscope x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )
    # Accelerometer data (in meters per second squared):
    ax,ay,az = bno.read_accelerometer()
    #print('Accelerometer x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #lx,ly,lz = bno.read_linear_acceleration()
    #print('Linear Acceleration x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #grx,gry,grz = bno.read_gravity()
    #print('Gravity x,y,z = {0:0.2F}, {1:0.2F}, {2}'.format(x,y,z) )

    #f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}, {16}, {17}, {18}, {19}, {20}, {21}, {22}, {23}'.format(heading, roll, pitch, sys, gyro, accel, mag, temp_c, mx, my, mz, gx, gy, gz, ax, ay, az, lx, ly, lz, grx, gry, grz, current_milli_time() ) )
    #f.write('\n')
    # Sleep for a second until the next reading.
    
    #this loop is on the order of 25mS, measured 18-37.
    
    time_now = current_milli_time()
    deltaT = time_now-time_old
    #print( deltaT )
    
    # after 3s, the control loop does nothing so need to break
    if( (time_now - time_start)> 55000):
	break

    if( (time_now - time_start)> 4500 and ejectionfirecheck):  #optimal delay is about 1.9s after burn out, ~2.6s burn time after detecting launch
	
	ejectionfirecheck = False
	#fire!
	f.write("About to fire Ejection Charge - {0}\n".format( current_milli_time() ) )
	f.close()
        f = open("IMU.txt", "a") 
        g.write("About to fire Ejection Charge - {0}\n".format( current_milli_time() ) )
        g.close()
        g = open("IMUbackup.txt", "a") 
	###### MAKE SURE THIS IS ON FOR AN ACTUAL TEST !!!! #####
	GPIO.output(20, GPIO.HIGH)   #Firing ejection charge
	f.write("Fired Ejection Charge\n - {0}\n".format( current_milli_time() ) )
        f.close()
        f = open("IMU.txt", "a") 
        g.write("Fired Ejection Charge\n - {0}\n".format( current_milli_time() ) )
        g.close()
        g = open("IMUbackup.txt", "a") 
	#motor has burned out
    # pitch rotates about the x-axis gyroX is d(pitch)/dt
    # roll rotates about the y-axis. gyroY is d(roll)/dt
    
    # need to set servo angles now based on PID
    
    #roll_derivative = (roll-roll_old)/(deltaT)
    roll_derivative = gy*57.32       #180/pi = 57.32. convert to degrees/s
    roll_integral = roll_integral + (roll-desiredRoll)*(deltaT/1000)
    
    rollSetpoint = -Kp*(roll-desiredRoll) - Kd*(roll_derivative) - Ki*(roll_integral)
    servoYval = centerVal2 - rollSetpoint/0.03     #map desired degreees to servo counts
    #roll corresponds to rotation about y-axis per my rocket
    #servoYval = centerVal2 + roll*(maxVal2-minVal2)/2/45.
    if( servoYval > maxVal2 ):
	servoYval = maxVal2
	#print('max value for servo2 reached + {0}'.format(maxVal2))
    if( servoYval < minVal2 ):
	servoYval = minVal2
	#print('min value for servo2 reached + {0}'.format(minVal2))
    
    #pitch corresponds to rotation about x-axix on my rocket
    if( pitch > 0 ):
	pitchVal = 180 - pitch
    else:
	pitchVal = -180 - pitch
    
    #pitch_derivative = (pitchVal-pitch_old)/(deltaT)
    pitch_derivative = gx*57.32        #180/pi=57.32
    pitch_integral = pitch_integral + (pitchVal-desiredPitch)*(deltaT/1000)
    pitchSetpoint = -Kp*(pitchVal-desiredPitch) - Kd*(pitch_derivative) - Ki*pitch_integral
    servoXval = centerVal1 - pitchSetpoint/0.033
    if( servoXval > maxVal1 ):
	servoXval = maxVal1
    elif( servoXval < minVal1 ):
	servoXval = minVal1
    
    if( servoYval > maxVal2 ):
	servoYval = maxVal2
    elif( servoYval < minVal2 ):
	servoYval = minVal2
    
    if( ejectionfirecheck ):    #after ejection charge is fired, no need 
    	pwm.set_servo_pulsewidth( servo2, servoYval )
    	pwm.set_servo_pulsewidth( servo1, servoXval )
    
    time_old = time_now
    roll_old = roll
    pitch_old = pitchVal
    #print("pitchSetpoint = {0}, rollSetpoint = {1}, deltaT = {2}".format(pitchSetpoint, rollSetpoint, deltaT) )
    f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, time_old, rollSetpoint, pitchSetpoint) )
    f.close()
    f = open("IMU.txt", "a") 
    #g.write('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}\n'.format(heading, roll, pitchVal, ax, ay, az, gx, gy, gz, time_old, rollSetpoint, pitchSetpoint) )
    #g.close()
    #g = open("IMUbackup.txt", "a")
# exited the main control loop - motor is exhausted

# continue to record data for X seconds
# fire ejection charge
# record data for Y seconds
# sleep
# turn on buzzer intermittantly
# close file, possibly use completely different one for logging after motor burns for crash issues
pwm.set_servo_pulsewidth(servo1, centerVal1)
pwm.set_servo_pulsewidth(servo2, centerVal2)

f.write("-------------------------")
f.close()
g.write("-------------------------")
g.close()
