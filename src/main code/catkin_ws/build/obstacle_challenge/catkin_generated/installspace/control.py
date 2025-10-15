import rospy
from std_msgs.msg import Float32
from open_challenge.msg import ultraInfo


import board
import busio

import time
import math
import serial

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import RPi.GPIO as GPIO
from time import sleep

from obstacle_challenge.srv import PillarDetection, PillarDetectionRequest

IN1 = 18
IN2 = 12
left_servo = 11
right_servo = 12
front_servo = 15
ackerman_servo = 0 

delta=15
left_midd = 85+delta
right_midd = 85-delta
front_midd = 90
ackerman_midd = 125


sec=0
dir1=0
prev_dist_err=0
prev_yaw = 0
unwrapped_yaw = 0
steering = 0

GPIO.setwarnings(False)

GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
pi_pwm1 = GPIO.PWM(IN1,5000)
pi_pwm2 = GPIO.PWM(IN2,5000)
pi_pwm1.start(0)
pi_pwm2.start(0)



i2c_servo = board.I2C()  
pca = PCA9685(i2c_servo)
pca.frequency = 50
servo0 = servo.Servo(pca.channels[right_servo], min_pulse=700, max_pulse=2900)
servo1 = servo.Servo(pca.channels[left_servo], min_pulse=700, max_pulse=2900)
servo2 = servo.Servo(pca.channels[ackerman_servo], min_pulse=580, max_pulse=2350)
servo3 = servo.Servo(pca.channels[front_servo], min_pulse=700, max_pulse=2900)

dists = [0,0,0,0]

pillars_array= []

yaw = 360*3+1
r = 0
l = 1
f = 2
b = 3

max_speed = 94

def pillar_detection_client():
    rospy.wait_for_service('pillar_detection')
    try:
        global pillars
        # Create service proxy
        pillar_detection = rospy.ServiceProxy('pillar_detection', PillarDetection)
        # Make request (empty in this case)
        request = PillarDetectionRequest()
        # Get response
        response = pillar_detection(request)
        pillars_array.append(response)
        # Process response
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def clean_all():
    move_dc(0,pi_pwm1,pi_pwm2)
    GPIO.cleanup()
    

def move_dc(speed,pi_pwm1,pi_pwm2):
    speed=min(speed,max_speed)
    speed=max(speed,-max_speed)
    if speed>0:
         pi_pwm1.ChangeDutyCycle(abs(speed))
         pi_pwm2.ChangeDutyCycle(0)
    else:
         pi_pwm2.ChangeDutyCycle(abs(speed))
         pi_pwm1.ChangeDutyCycle(0)
        


def move_servos(yaw):
    global steering, delta
    
    angle0=min(right_midd+yaw,right_midd+60+delta)
    angle0=max(angle0,right_midd-60+delta)
    
    angle1=min(left_midd+yaw,left_midd+60-delta)
    angle1=max(angle1,left_midd-60-delta)
    
    angle2=min(steering,50)
    angle2=max(angle2,-110)
    
    angle3=min(front_midd+yaw,front_midd+60)
    angle3=max(angle3,front_midd-60)
    
    #rospy.loginfo('1')
    servo0.angle=angle0
    servo1.angle=angle1
    servo2.angle=round(ackerman_midd+angle2)
    servo3.angle=angle3
    #rospy.loginfo('2')    

def pid_control(kp1=-1,kp2=0.5,kd2=3,speed=max_speed,threshold=0):
    global prev_dist_err,steering
    dist_err = dists[1] - dists[0]+threshold*dir1
    angle_err=yaw+float(dir1*sec*90)
    corr=kp1*(angle_err)+(kp2*(dist_err)+kd2*(dist_err-prev_dist_err))*(abs(dist_err)>10)
    prev_dist_err=dist_err
    steering = corr
    move_dc(speed,pi_pwm1,pi_pwm2)


def callback(dist): 
    global dists
    dists = dist.nums
    

   
def callback1(data):
    global yaw
    yaw = round(unwrap_yaw(data.data))
    move_servos(yaw+float(dir1*sec*90)) 
    

def unwrap_yaw(current_yaw):
    global prev_yaw, unwrapped_yaw
    if prev_yaw is None:
        unwrapped_yaw = current_yaw
    else:
        delta = current_yaw - prev_yaw
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        unwrapped_yaw += delta
    prev_yaw = current_yaw
    return unwrapped_yaw

pretime=-100
def movement_control():
    
    rospy.init_node('control')
    rospy.on_shutdown(clean_all)
    pub = rospy.Publisher('ultra_info', ultraInfo, queue_size=10)
    sub1 = rospy.Subscriber('gyro_sensor', Float32, callback1)
    for sensor in range(4):
        sub = rospy.Subscriber('ultra_sensors', ultraInfo, callback)
    
#    sub3 = 
    rate = rospy.Rate(10)
    msg = ultraInfo()
    msg.nums = [1,1,1,1]
    pub.publish(msg)

    move_servos(0)
    time.sleep(0.2)
    prev_dist_err=0
    
    global dir1, sec, turn_flag,steering
    while yaw == 1081 or dists[r] == 0 or dists[l] == 0 or dists[f] == 0 or dists[b] == 0:
        pass
    
    while (not rospy.is_shutdown()): 
        #print(dists[r],dists[l],dists[f],dists[b])
        global pretime
        pid_control(speed= 80 if dir1==0 else max_speed,threshold= 0 if dir1==0 else 0)
        

        
        
        if (dir1==0 and (dists[r]>100 or dists[l]>100 )) or (dir1!=0 and ( dists[f]<=50 or (dists[r]>100 or dists[l]>100 ))):
            if time.time()-pretime < 2:
                print(dists[r],dists[l],dists[f],dists[b])
                move_dc(-80,pi_pwm1,pi_pwm2)
                time.sleep(0.3)
                break
            dir1 = int((dists[r]>100))-int((dists[l]>100)) if dir1==0 else dir1
            sec+=1
            move_dc(max_speed,pi_pwm1,pi_pwm2)
            while (dists[l]>70 or dists[r]>70) or (abs(yaw+float(dir1*(sec-1)*90))<70):
                pid_control(-1,0,0)
            
            pretime=time.time()
                
            
            
        if sec == 4:
            
            while dists[f]>150 or dists[b]<110:
                pid_control(speed=80)
                 
            move_dc(-80,pi_pwm1,pi_pwm2)
            time.sleep(0.3)
            break
            

try:
    movement_control()
except KeyboardInterrupt:
    GPIO.cleanup()
        

pca.deinit()
GPIO.cleanup()





