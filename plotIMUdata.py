#!/usr/bin/env python

from pylab import * 
import csv

print "hello"
close('all')


f1 = csv.reader(  open( "IMU.txt" , "rU") )
data = [ this_row for this_row in f1]

# pitch rotates about the x-axis gyroX is d(pitch)/dt
# roll rotates about the y-axis. gyroY is d(roll)/dt
roll = []
pitch = []
heading = []
time = []
timedelta = []
gyroX = []
gyroY = []
gyroZ = []
rollDerivative = []
pitchDerivative = []
ax = []
ay = []
az = []
pitchVal = []
pitchSetPoint = []
rollSetPoint = []
rollOld = 0
pitchOld = 0
rollSetPoint = []
time_offset = 18000

for index, x in enumerate(data):
	
	if( index == 0):
		start_time = x[9]
		old_time = float(start_time)-1
		
	heading.append( float(x[0]) )
	roll.append( float(x[1]) )
	rollDerivative.append( 1000*(float(x[1]) - rollOld)/(float(x[9])-old_time) ) 
	rollOld = float(x[1])
	pitch.append( float(x[2]) )
	ax.append( float(x[3]))
	ay.append( float(x[4]))
	az.append( float(x[5]))
	
    #pitch corresponds to rotation about x-axix on my rocket
	if( float(x[2]) > 0 ):
		pitchValCurrent = 180 - float(x[2])	
	else:
		pitchValCurrent = -180 - float(x[2])
		
	pitchValCurrent = float( x[2] )
	pitchVal.append( pitchValCurrent  )
	pitchDerivative.append( 1000*( pitchValCurrent-pitchOld)/(float(x[9])-old_time) )
	pitchOld = pitchValCurrent
	

	currentPitchVal = float( x[11] )
	currentRollVal = float( x[10] )
	if( currentPitchVal  > 5.5 ):
		currentPitchVal = 5.5
	elif( currentPitchVal < -5.5 ):
		currentPitchVal = -5.5

	if( currentRollVal  > 6 ):
		currentRollVal = 6
	elif( currentRollVal < -6 ):
		currentRollVal = -6
			
	# pitch/roll vals are at 0 prior to ignition
	if( float(x[9])< (1614800415651-4500)):
		currentPitchVal = 0
		currentRollVal = 0

	rollSetPoint.append( currentRollVal )
	pitchSetPoint.append( currentPitchVal )
	time.append( float(x[9]) - float(start_time) )
	timedelta.append( float(x[9]) - old_time)
	old_time = float(x[9])
	gyroX.append( 180/3.14*float( x[6] ) )
	gyroY.append( 180/3.14*float( x[7] ) )
	gyroZ.append( 180/3.14*float( x[8] ))

# shifts start time from start of ignition wait sequence to just before launch
time = [(x-time_offset)/1000 for x in time]

p = plot( time, ax, 'k')
p = plot(time, ay, 'b')	
p = plot( time, az, 'g')
plt.setp(p, linewidth=2)
#plot(time, pitch, 'b')
#plot(time, heading, 'g')
grid()
xlabel('Time (s)')
xlim([-0.5, 5.3])
xlim([-0.5, 13])
ylim([-40, 36])
ylabel('Acceleration (m/s/s)')
title('X, Y, Z Accelerations')

# ejection charge fired ####
plot([  (float(1614800415651)-float(start_time)- time_offset)/1000, (float(1614800415651)-float(start_time)-time_offset)/1000], [-500, 500], 'r')
# estimated motor burnout
plot([  (float(1614800415651-1900)-float(start_time)-time_offset)/1000, (float(1614800415651-1900)-float(start_time)-time_offset)/1000], [-500, 500], 'm')
# estimated motor start
plot([  (float(1614800415651-4500)-float(start_time)-time_offset)/1000, (float(1614800415651-4500)-float(start_time)-time_offset)/1000], [-500, 500], 'g')

plt.show()