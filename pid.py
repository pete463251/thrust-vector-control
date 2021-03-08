print "eat me"
import numpy
from pylab import * 


delT = 0.02   #update rate on IMU is ~10mS. Update rate on Servo probably closer to 100mS?
Inertia = 0.0202
numSamples = 200   #total runtime = delT*numSamples
d = 0.215     #distance between engine and Cg
Force = 9  #Estes E9
theta0 = -0*pi/180.  #setpoint
Kp = 45
Kd = 10
Ki = 0.2
theta = numpy.zeros( numSamples)
t = [x*delT for x in range(numSamples)]
torque = numpy.zeros( numSamples )
setpoint = numpy.zeros( numSamples )

#angles for gimbal angle is in degrees!!!
maxangle = 5.5  
maxRotationPerStep = 20/0.1*delT  #100mS/60deg per spec sheet, this is hte max change in servo angle per step

# returns force for engine at time T, assuming launch at time 0
def getForce(t):
	#return Force in Newtons as a function of t
	if( t<0.05 ):
		return 0
	if( t<0.1 ):
		return 1
	if( t<0.15 ):
		return 5
	if( t<0.2 ):
		return 15
	if( t<0.25 ):
		return 25
	if( t<0.3  ):
		return 14
	if( t<0.35 ):
		return 12
	if( t<0.4  ):
		return 11
	if( t<2.7 ):
		return 10
	if( t<2.8 ):
		return 5
	
	return 0
		
	
	
	
# intial conditions
initialTh = -10
theta[0] = initialTh*pi/180.   #must be in radians
theta[1] = initialTh*pi/180.   #deg
runningSum = theta[0]+theta[1]
for n in range(2, numSamples):
	#update governing dynamics
	torque[n-1] = d*getForce(n*delT)*sin( pi*setpoint[n-1]/180.)
	
	# perturb the rocket at 1 second in
	#if( n == floor(1/delT) ):
	#	theta[n-1] = theta[n-1] + 5*pi/180.
	#	theta[n-2] = theta[n-2] + 5*pi/180.
		
	#using delayed torque to account for propagation time
	theta[n] = 2*theta[n-1] - theta[n-2] + torque[n-2]*delT*delT/Inertia #+ 0.0005*numpy.random.standard_normal()
	
	#PID control
	runningSum = runningSum + (theta[n]-theta0)
	#runningSum = 0
	#if( n> 50 ):
	#	for j in range(50):
	#		runningSum = runningSum + theta[n-j]

	desiredSetpoint = - Kp*(theta[n]-theta0) - Kd*(theta[n]-theta[n-1])/delT - Ki*runningSum
	if( (desiredSetpoint - setpoint[n-1])>maxRotationPerStep ):
		setpoint[n] = setpoint[n-1] + maxRotationPerStep
	elif( (setpoint[n-1]-desiredSetpoint)>maxRotationPerStep ):
		setpoint[n] = setpoint[n-1] - maxRotationPerStep
	else:
		setpoint[n] = desiredSetpoint
		 
	if( setpoint[n] > maxangle ):
		setpoint[n] = maxangle
	if( setpoint[n] <-maxangle ):
		setpoint[n] = -maxangle
		
	print "n=" + str(n) + " theta=" + str(theta[n]) + " setpoint=" + str(setpoint[n])
	

print maxRotationPerStep
p = plot(t, 180/pi*theta, 'k')
grid()
plot(t, setpoint, 'b')	
xlabel('time [s]')
plt.show()



### dynamic equaitons of motion ####
#applied torque = I * alpha = I*angular acceleration = I* d/dt*(d/dt(theta) )=I*d/dt ( theta[n]-theta[n-1] )/delT
# applied torque = I/delT*() (  theta[n]-theta[n-1] )-(theta[n-1]-theta[n-2] ) )/delT
# applied torque = I/delT/delT*(theta[n]+theta[n-2]-2*theta[n-1])