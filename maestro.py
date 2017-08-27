import serial
import time
import getch #Library to detect a keyboard
#
#---------------------------
# Maestro Servo Controller
#---------------------------
#
# Support for the Pololu Maestro line of servo controllers
#
# Steven Jacobs -- Aug 2013
# https://github.com/FRC4564/Maestro/
#
# These functions provide access to many of the Maestro's capabilities using the
# Pololu serial protocol
#
# With these following lines, we will automatically identify the name of the serial
# port where the Maestro is plugged in.
locations=['/dev/ttyACM0', '/dev/ttyACM2']
for device in locations:
   try:
      #print "Trying...",device
      serialport = serial.Serial(device, 2400, timeout = 0)
      right_dev=device
      break
   except:
      #print "Failed to connect on",device
      if device == 'end':
         print "Unable to find Serial Port, Please plug in cable or check cable connections."
         exit()


#Motion variables
right_servo = 4            #Raspberry pin for right servo
left_servo = 5             #Raspberry pin for left servo
time_to_move = 0.0000      #Time to move both servos
backward_init_value = 5800 #Init value to backward acceleration
forward_init_value = 6140  #Init value to forward acceleration
frictionMovement = 1.1     #Friction value [0.8-1.2] to calibrate movement
frictionAngle = 1.0        #Friction value [0.8-1.2] to calibrate spin an angle


class Controller():
    # When connected via USB, the Maestro creates two virtual serial ports
    # /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
    # Be sure the Maestro is configured for "USB Dual Port" serial mode.
    # "USB Chained Mode" may work as well, but hasn't been tested.
    #
    # Pololu protocol allows for multiple Maestros to be connected to a single
    # communication channel. Each connected device is then indexed by number.
    # This device number defaults to 0x0C (or 12 in decimal), which this module
    # assumes.  If two or more controllers are connected to different serial
    # ports, or you are using a Windows OS, you can provide the port name.  For
    # example, '/dev/ttyACM2' or for Windows, something like 'COM3'.
    def __init__(self, ttyStr=right_dev):
        # Open the command port
        self.usb = serial.Serial(ttyStr)
        # Command lead-in and device 12 are sent for each Pololu serial commands.
        self.PololuCmd = chr(0xaa) + chr(0xc)
        # Track target position for each servo. The function isMoving() will
        # use the Target vs Current servo position to determine if movement is
        # occuring.  Upto 24 servos on a Maestro, (0-23). Targets start at 0.
        self.Targets = [0] * 24
        # Servo minimum and maximum targets can be restricted to protect components.
        self.Mins = [0] * 24
        self.Maxs = [0] * 24
        
    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()

    # Set channels min and max value range.  Use this as a safety to protect
    # from accidentally moving outside known safe parameters. A setting of 0
    # allows unrestricted movement.
    #
    # ***Note that the Maestro itself is configured to limit the range of servo travel
    # which has precedence over these values.  Use the Maestro Control Center to configure
    # ranges that are saved to the controller.  Use setRange for software controllable ranges.
    def setRange(self, chan, min, max):
        self.Mins[chan] = min
        self.Maxs[chan] = max

    # Return Minimum channel range value
    def getMin(self, chan):
        return self.Mins[chan]

    # Return Maximum channel range value
    def getMax(self, chan):
        return self.Maxs[chan]
        
    # Set channel to a specified target value.  Servo will begin moving based
    # on Speed and Acceleration parameters previously set.
    # Target values will be constrained within Min and Max range, if set.
    # For servos, target represents the pulse width in of quarter-microseconds
    # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
    # Typcially valid servo range is 3000 to 9000 quarter-microseconds
    # If channel is configured for digital output, values < 6000 = Low ouput
    def setTarget(self, chan, target):
        # if Min is defined and Target is below, force to Min
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        # if Max is defined and Target is above, force to Max
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]
        #
        lsb = target & 0x7f #7 bits for least significant byte
        msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        # Send Pololu intro, device number, command, channel, and target lsb/msb
#	print target & 0x14f
#	print target & 0xdf
#	print lsb, msb, chr(lsb), chr(msb)
        cmd = self.PololuCmd + chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.usb.write(cmd)
        # Record Target value
        self.Targets[chan] = target
    
    # This method includes some improvements compared to setTarget. You can
    # introduce the "zero" speed, which is a value that changes in each servo,
    # you can write the speed in cm/s and you can control the rotation direction.
    def setTargetA(self, chan, cms, zero=None, rotationDirection=None):
        
    # Conversion from quarter-microseconds to cm/s
	if zero is None:
		zero=6000
	if rotationDirection is None:
		rotationDirection=1
	sp_qus=int(round((cms-1.53)/0.026))
	if rotationDirection == -1:
		sp_qus=-1*sp_qus
	target=zero+sp_qus
	# if Min is defined and Target is below, force to Min
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        # if Max is defined and Target is above, force to Max
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]
        #    
        lsb = target & 0x7f #7 bits for least significant byte
        msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        # Send Pololu intro, device number, command, channel, and target lsb/msb
        cmd = self.PololuCmd + chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.usb.write(cmd)
        # Record Target value
        self.Targets[chan] = target
        
    # Set speed of channel
    # Speed is measured as 0.25microseconds/10milliseconds
    # For the standard 1ms pulse width change to move a servo between extremes, a speed
    # of 1 will take 1 minute, and a speed of 60 would take 1 second.
    # Speed of 0 is unrestricted.
    def setSpeed(self, chan, speed):
        lsb = speed & 0x7f #7 bits for least significant byte
        msb = (speed >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        # Send Pololu intro, device number, command, channel, speed lsb, speed msb
        cmd = self.PololuCmd + chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.usb.write(cmd)

    # Set acceleration of channel
    # This provide soft starts and finishes when servo moves to target position.
    # Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
    # A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
    def setAccel(self, chan, accel):
        lsb = accel & 0x7f #7 bits for least significant byte
        msb = (accel >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        # Send Pololu intro, device number, command, channel, accel lsb, accel msb
        cmd = self.PololuCmd + chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.usb.write(cmd)
    
    # Get the current position of the device on the specified channel
    # The result is returned in a measure of quarter-microseconds, which mirrors
    # the Target parameter of setTarget.
    # This is not reading the true servo position, but the last target position sent
    # to the servo. If the Speed is set to below the top speed of the servo, then
    # the position result will align well with the acutal servo position, assuming
    # it is not stalled or slowed.
    def getPosition(self, chan):
        cmd = self.PololuCmd + chr(0x10) + chr(chan)
        self.usb.write(cmd)
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb

    # Test to see if a servo has reached its target position.  This only provides
    # useful results if the Speed parameter is set slower than the maximum speed of
    # the servo. 
    # ***Note if target position goes outside of Maestro's allowable range for the
    # channel, then the target can never be reached, so it will appear to allows be
    # moving to the target.  See setRange comment.
    def isMoving(self, chan):
        if self.Targets[chan] > 0:
            if self.getPosition(chan) <> self.Targets[chan]:
                return True
        return False
    
    # Have all servo outputs reached their targets? This is useful only if Speed and/or
    # Acceleration have been set on one or more of the channels. Returns True or False.
    def getMovingState(self):
        cmd = self.PololuCmd + chr(0x13)
        self.usb.write(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True

    # Run a Maestro Script subroutine in the currently active script. Scripts can
    # have multiple subroutines, which get numbered sequentially from 0 on up. Code your
    # Maestro subroutine to either infinitely loop, or just end (return is not valid).
    def runScriptSub(self, subNumber):
        cmd = self.PololuCmd + chr(0x27) + chr(subNumber)
        # can pass a param with command 0x28
        # cmd = self.PololuCmd + chr(0x28) + chr(subNumber) + chr(lsb) + chr(msb)
        self.usb.write(cmd)

    # Stop the current Maestro Script
    def stopScript(self):
        cmd = self.PololuCmd + chr(0x24)
        self.usb.write(cmd)


   # Stop both servos
    def stopServos(self):
	self.setTarget(left_servo,0)
	self.setTarget(right_servo,0)
	time.sleep(0.5)

   # Method to accelerate both servos during 'loop_time'. 'Back' varible indicates
   # if acceleration is backward (back=True) or forward (back=False)
    def acceleration(self, back, loop_time):
	time=0
	while (time<loop_time):
		time=time+1
		if (back):
			self.setTarget(left_servo, forward_init_value+2*time)
			self.setTarget(right_servo, backward_init_value-2*time)
		else:
			self.setTarget(left_servo, backward_init_value-2*time)
			self.setTarget(right_servo, forward_init_value+2*time)

   # Method to spin an 'angle'
    def spinAngle(self, angle):
	if (angle>0):
		v_izq=-1
		v_der=-1
	elif (angle<0):
		v_izq=1
		v_der=1
	self.setTarget(left_servo, v_izq)
	self.setTarget(right_servo, v_der)
	div=0.59 if angle<=90 else (0.61 + (angle/90*0.026))
	time_to_sleep = float(abs(angle))/90*div * frictionAngle
	#print "time: ",time_to_sleep
	time.sleep(time_to_sleep)
	self.stopServos()

   # Method to command a (positive and negative) 'distance' (cm) to move.
    def moveDistance(self, distance):
	if (distance>0):
		v_izq=1
		v_der=-1
		backward=False
	elif (distance<0):
		v_der=1
		v_izq=-1
		backward=True
	else:
		v_der=0
		v_izq=0
	time_to_move=float(abs(distance)-8)/17 * frictionMovement
	self.acceleration(backward, 400)
	self.setTarget(left_servo, v_izq)
	self.setTarget(right_servo, v_der)
	time.sleep(time_to_move)
	self.stopServos()

   # Method to command a square movement with side = 'distance'
    def square(self, distance):
	distance=distance+13
	self.moveDistance(distance)
	self.spinAngle(90)
	self.moveDistance(distance)
	self.spinAngle(90)
	self.moveDistance(distance)                      
	self.spinAngle(90)
	self.moveDistance(distance)
	self.spinAngle(90)

   # Method to command a rectangular movement with sides = 'distance_1' and
   # 'distance_2'
    def rectangle(self, distance_1, distance_2):
	distance_1=distance_1 + 13
	distance_2=distance_2 + 13
	self.moveDistance(distance_1)
	self.spinAngle(90)
	self.moveDistance(distance_2)
	self.spinAngle(90)
	self.moveDistance(distance_1)
	self.spinAngle(90)
	self.moveDistance(distance_2)
	self.spinAngle(90)

   # Method to command a circular movement
    def spinner(self):
      	x=0
	while(x<39):
		x=x+1
		self.setTarget(left_servo,5400)
		self.setTarget(right_servo,8000)
		time.sleep(0.3)
	self.stopServos()

   # Manual control allows to control JUS with keyboard.
   #
   # Keys to control:
   #                 w -> Forward
   #                 s -> Backward
   #                 a -> Spin to left
   #                 d -> Spin to right
   #                 q -> Exit from manual control
   #                 Any other key -> Stop 

   
    def manualControl(self):
	move = 1
	while(move):
		tecla = getch.getch()
		if tecla == "w":
#			print "forward"
			v_der=-1
			v_izq=1
		elif tecla == "s":
#			print "backward"
			v_der=1
			v_izq=-1
		elif tecla == "a":
#			print "left"
			v_der=1
			v_izq=1
		elif tecla == "d":
#			print "right"
			v_der=-1
			v_izq=-1
		elif tecla == "q":
#			print "exit"
			move = 0
		else:
			v_der=0
			v_izq=0

		self.setTarget(right_servo, v_der)
		self.setTarget(left_servo, v_izq)
   
