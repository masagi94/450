import RPi.GPIO as  GPIO
import time

# These are the pins we will be using.
# ENA-6, ENB-26, IN1-12, IN2-13, IN3-20, IN4-21
# CS-5, DataOut-23, Address-24, Clock-25
ENA = 6
ENB = 26
IN1 = 12
IN2 = 13
IN3 = 20
IN4 = 21
CS = 5
DataOut = 23
Address = 24
Clock = 25


numSensors = 5
gvalue = [0,0,0,0,0,0] #array to hold values of the rake sensor

#Functions to carry out movement of the robot
# 0 = GPIO.LOW  1 = GPIO.HIGH
def moveBackward():
    # print("Moving Backward...")
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)

def stop():
    # print("Stopping...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)

def moveForward():
    # print("Moving Forward...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)

def pivotRight():
    # print("Left Pivot...")
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)

def pivotLeft():
    # print("Right Pivot...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)

def turnRight():
    # print("Turn Right...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)


def turnLeft():
    # print("Turn Left...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)



# Reads in the values from the sensors, returns the values
def readAnalog():
    value = [0,0,0,0,0,0] #array to hold values of the rake sensor
    for j in range(0,6):
        GPIO.output(CS, GPIO.LOW)
        for i in range(0,4):
            #sent 4-bit Address
            if(((j) >> (3 - i)) & 0x01):
                GPIO.output(Address,GPIO.HIGH)
            else:
                GPIO.output(Address,GPIO.LOW)
            #read MSB 4-bit data
            value[j] <<= 1
            if(GPIO.input(DataOut)):
                value[j] |= 0x01
            GPIO.output(Clock,GPIO.HIGH)
            GPIO.output(Clock,GPIO.LOW)
        for i in range(0,6):
            #read LSB 8-bit data
            value[j] <<= 1
            if(GPIO.input(DataOut)):
                value[j] |= 0x01
            GPIO.output(Clock,GPIO.HIGH)
            GPIO.output(Clock,GPIO.LOW)
        #no mean ,just delay
        for i in range(0,6):
            GPIO.output(Clock,GPIO.HIGH)
            GPIO.output(Clock,GPIO.LOW)
#       time.sleep(0.0001)
        GPIO.output(CS,GPIO.HIGH)

    return value[1:]


# Sets the desired pin numbering system to BCM
GPIO.setmode(GPIO.BCM)

# Disables warnings in case the RPI.GPIO detects that a pin has been configured
# to something other than the default (input)
GPIO.setwarnings(False)

# Sets all the pins stated above as outputs
chan_list = [ENA,ENB,IN1,IN2,IN3,IN4,Address,CS,Clock]
GPIO.setup(chan_list,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

# creates objects "p1" and "p2", sets ena and enb to 50 Hz
p1 = GPIO.PWM(ENA,50)
p2 = GPIO.PWM(ENB,50)


# Used for determining which part of the track the robot is on
counter = 0
counter2 = 0
lastTurn = 0
afterLoop = 0
rightTurnCount = 0



# Launch.
# Loops through until the track is completed.
while True:
    
# Stops the robot and sleeps for some time, in order to read from the sensors easier, and
# to prevent the robot from moving out of the line due to moving too fast.
    counter += 1
    stop()
    time.sleep(.1)

# Read values from sensors, store into s1 - s5
    gvalue = readAnalog()
    s1 = gvalue[0]
    s2 = gvalue[1]
    s3 = gvalue[2]
    s4 = gvalue[3]
    s5 = gvalue[4]

# Print statements used for debugging the code and robot position.
    print("AfterLoop?: %d \n") % afterLoop
    print("Count: %d") % counter
    for i in range(1,6):
        print("Sensor %d: ") % i
        print(gvalue[i-1])
    print("\n")


    # These first 3 if statements are for controlling the behavior when the robot
    # is at the loop entrance, inside the loop, and at the fork at the end.
    # The robot enters the loop if it detects more than 5 right turns in a row, but only
    # if the counter is above 70, which would put the robot past the first 2 right turns.
    if (rightTurnCount > 5 and counter > 70 and afterLoop == 0):
        p1.start(25)
        p2.start(25)
        turnRight()
        time.sleep(.75)
        stop()
        time.sleep(.1)
        afterLoop = 1
        rightTurnCount = 0

    # Once inside the loop, the robot will check if there were more than 10 right turns
    # in a row, indicating the robot is turning, and pivots instead, since the turn is too
    # tight to make by just turning.
    elif (afterLoop == 1 and rightTurnCount > 10):
        p1.start(25)
        p2.start(25)
        pivotRight()
        time.sleep(1)
        stop()
        time.sleep(.1)
        afterLoop = 2

    # Once the robot is at the end of the track, it checks for the fork, and pivots to face the 
    # direction it started in to indicate the end of the run.
    elif (s1 < 600 and s2 > 600 and s3 < 600 and s4 > 600 and afterLoop == 2):
        lastTurn = 0
        p1.start(25)
        p2.start(25)
        turnLeft()
        time.sleep(.5)
        afterLoop = 0
        break;


    # If the middle sensor detects the line, move forward.
    elif (s3 < 600):
        rightTurnCount = 0
        p1.start(25)
        p2.start(25)
        moveForward()
        time.sleep(.1)
    
    # If the line is moving out of place to the right, compensate by turning in to the left.
    elif (s2 < 600):
        lastTurn = 0
        rightTurnCount = 0
        p1.start(25)
        p2.start(25)
        turnLeft()
        time.sleep(.1)
    # Smooths out the right turns.
    elif (s1 < 600 and s2 > 600):
        lastTurn = 0
        rightTurnCount = 0
        p1.start(25)
        p2.start(25)
        turnLeft()
        time.sleep(.1)
    

    # If the line is moving out of place to the left, compensate by turning in to the right.
    elif (s4 < 600):
        lastTurn = 1
        rightTurnCount += 1
        p1.start(25)
        p2.start(25)
        turnRight()
        time.sleep(.1)
    # Smooths out the left turns
    elif (s5 < 600 and s4 > 600):
        lastTurn = 1
        rightTurnCount += 1
        p1.start(25)
        p2.start(25)
        turnRight()
        time.sleep(.1)
    

    # If the robot loses the line, it will continue doing the previous movement until it finds
    # the line again.
    elif (lastTurn == 0):
        rightTurnCount = 0
        p1.start(25)
        p2.start(25)
        turnLeft()
        time.sleep(.1)
    elif (lastTurn == 1):
        rightTurnCount += 1
        p1.start(25)
        p2.start(25)
        turnRight()
        time.sleep(.1)
    
    # Move forward if nothing else is triggered.
    else:
        rightTurnCount = 0
        p1.start(25)
        p2.start(25)
        moveForward()
        time.sleep(.1)
    

# Stops both the PWM outputs
p1.stop()
p2.stop()

# Cleans up the used resources
GPIO.cleanup()
