import rospy
from std_msgs.msg import Float32
from open_challenge.msg import ultraInfo

import board
import busio
import time, math, serial, copy

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO
from time import sleep

# ----------------------- Hardware Pin Setup -----------------------
IN1 = 18
IN2 = 12
left_servo = 11
right_servo = 12
front_servo = 15
ackerman_servo = 0 

# ----------------------- Servo and Steering Constants -----------------------
delta=0
front_delta=0
left_midd = 85    # Center angle for left servo
right_midd = 85   # Center angle for right servo
front_midd = 90
ackerman_midd = 90
max_steering = 38
min_steering = -76

# ----------------------- Control & State Variables -----------------------
sec=0
dir1=0
prev_dist_err=0
dist_err = 0
prev_yaw = 0
prev_yw = 0
unwrapped_yaw = 0
yaw_0 = 0
correction = 0
steering = 0

# ----------------------- GPIO Setup for Motor Control -----------------------
GPIO.setwarnings(False)
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
pi_pwm1 = GPIO.PWM(IN1,5000)
pi_pwm2 = GPIO.PWM(IN2,5000)
pi_pwm1.start(0)
pi_pwm2.start(0)

# ----------------------- PCA9685 Servo Controller Setup -----------------------
i2c_servo = board.I2C()  
pca = PCA9685(i2c_servo)
pca.frequency = 50

# Initialize servo channels
servo0 = servo.Servo(pca.channels[right_servo], min_pulse=700, max_pulse=2900)
servo1 = servo.Servo(pca.channels[left_servo], min_pulse=700, max_pulse=2900)
servo2 = servo.Servo(pca.channels[ackerman_servo], min_pulse=500, max_pulse=2550)
servo3 = servo.Servo(pca.channels[front_servo], min_pulse=700, max_pulse=2900)

# ----------------------- Ultrasonic Distance Buffers -----------------------
dists = [0,0,0,0]
prev_dists = [0,0,0,0]
curr_dists = [0,0,0,0]
filtered_dists=[0,0,0,0]
prev_filtered_dists = [0,0,0,0]
raw_dists=[0,0,0,0]
prefix_sum = [0,0,0,0]
prev_dif1 = [0,0,0,0]
cnt=[0,0,0,0]
thr = [0,0,0,0]
c=0
prev_filtered_dif1 = [0,0,0,0]

# ----------------------- Sensor Indexes -----------------------
yaw = 360*3+1
r = 0   # right sensor
l = 1   # left sensor
f = 2   # front sensor
b = 3   # back sensor

max_speed = 94

# ----------------------- Utility Functions -----------------------

def clean_all():
    """Stop all motion and clean up GPIO."""
    move_dc(0,pi_pwm1,pi_pwm2)
    GPIO.cleanup()
    

def move_dc(speed,pi_pwm1,pi_pwm2):
    """Drive the DC motor forward or backward based on speed."""
    speed=min(speed,max_speed)
    speed=max(speed,-max_speed)
    if speed>0:
         pi_pwm1.ChangeDutyCycle(abs(speed))
         pi_pwm2.ChangeDutyCycle(0)
    else:
         pi_pwm2.ChangeDutyCycle(abs(speed))
         pi_pwm1.ChangeDutyCycle(0)
        

def move_servos(yaw):
    """Set servo angles based on current yaw and steering corrections."""
    global steering, delta, front_delta, max_steering, min_steering
    
    # Clamp servo angles within safe ranges
    angle0=min(right_midd+yaw+delta,right_midd+45+delta)
    angle0=max(angle0,right_midd-45+delta)
    
    angle1=min(left_midd+yaw-delta,left_midd+45-delta)
    angle1=max(angle1,left_midd-45-delta)
    
    angle2=min(steering, max_steering)
    angle2=max(angle2,min_steering)
    
    angle3=min(front_midd+yaw+front_delta,front_midd+55)
    angle3=max(angle3,front_midd-55)
    
    # Apply to servos
    servo0.angle=angle0
    servo1.angle=angle1
    servo2.angle=round(ackerman_midd+angle2)
    servo3.angle=angle3

def steering_angle(corr):
    """Compute Ackermann steering angle based on correction input."""
    l = 135
    w = 125
    if corr != 0:
        r = l/math.tan(math.radians(corr))
        phiin = math.degrees(l/(r-w/2))
    else:
        phiin = 0
    phiin = math.radians(phiin+23.578)
    xg = 50 - 25*math.tan(phiin)/(math.sqrt(1+(math.tan(phiin))**2))
    a = (1114 - xg**2 + (xg-50)**2 + 10*math.sqrt(625-(xg-50)**2))
    A = math.sqrt(900*xg**2+180**2)
    phi = math.degrees(math.atan(xg/6))-180
    ans = math.degrees(math.asin(a/A)) - phi - 90
    return -1*ans

def pid_control(kp1=0.5,kp2=-0.45,kd2=0,speed=max_speed,threshold=0,dists=[30]*4):
    """PID control loop to adjust steering and maintain distance balance."""
    global prev_dist_err,steering, dist_err
    dist_err = dists[l] - dists[r]+threshold*dir1
    angle_err=yaw+float(dir1*sec*90)
    corr=kp1*(angle_err)+(kp2*(dist_err)+kd2*(dist_err-prev_dist_err))*(abs(dist_err)>0)
    
    # Limit correction to safe range
    corr = max(min(corr,25),-25)
    
    steering = steering_angle(corr)
    print(corr, steering)
    move_dc(speed,pi_pwm1,pi_pwm2)

def factor():
    """Helper factor based on previous yaw changes."""
    if abs(prev_yw+float(dir1*sec*90))<2:
        return 0
    else:
        return (abs(yaw+float(dir1*sec*90))/abs(prev_yw+float(dir1*sec*90)))

def dists_filter(sens):
    """Filter noisy ultrasonic sensor data."""
    global dists, prev_dists, prev_dif1,prefix_sum,raw_dists,cnt, prev_filtered_dists, prev_filtered_dif1,filtered_dists, prev_yw, thr,prev_dist_err
    dif1 = raw_dists[sens]-prev_dists[sens]
    dif2 = dif1-prev_dif1[sens]
    thr[sens] = dif2*0.5 if cnt[sens]==0 else thr[sens]
    prefix_sum[sens]+=dif2

    # Apply a differential filter with thresholds
    if abs(prefix_sum[sens]) > abs(thr[sens]) and abs(dif2)>3:
        filtered_dists[sens]=copy.deepcopy(prev_filtered_dists[sens]+prev_filtered_dif1[sens]*factor())
        dists[sens] = copy.deepcopy(filtered_dists[sens]) if cnt[sens] == 0 else copy.deepcopy(raw_dists[sens])
        cnt[sens]+=1
    else:
        prefix_sum[sens] = 0
        cnt[sens] = 0
        dists[sens]=copy.deepcopy(raw_dists[sens])
        filtered_dists[sens] = copy.deepcopy(dists[sens])
    prev_yw = yaw
    prev_dif1[sens] = dif1
    prev_filtered_dif1[sens] = filtered_dists[sens] - prev_filtered_dists[sens]
    if abs(prev_filtered_dif1[sens])>7:
        prev_filtered_dif1[sens]=0

def callback(dist):
    """ROS callback for ultrasonic distance data."""
    global dists, prev_dists, prev_dif1,raw_dists, prev_filtered_dists, filtered_dists, c, f, dist_err, prev_dist_err
    prev_dists=copy.deepcopy(raw_dists)
    prev_filtered_dists=copy.deepcopy(filtered_dists)
    raw_dists = list(dist.nums)
    if c == 2:
        dists_filter(r)
        dists_filter(l)
        dists_filter(f)
        dists_filter(b)
    else:
        dists=copy.deepcopy(raw_dists)
        filtered_dists = copy.deepcopy(dists)
        c+=1
    prev_dist_err=dist_err

def callback1(data):
    """ROS callback for gyro sensor (yaw)."""
    global yaw,prev_yw, yaw_0, correction
    yaw = round(unwrap_yaw(data.data))-correction - yaw_0
    move_servos(yaw+float(dir1*sec*90)) 

def unwrap_yaw(current_yaw):
    """Unwrap yaw readings to avoid 0–360 degree wraparound."""
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

def stop_dc(delay):
    """Emergency stop for DC motor."""
    pi_pwm2.ChangeDutyCycle(100)
    pi_pwm1.ChangeDutyCycle(100)
    time.sleep(delay)

# ----------------------- Main Control Loop -----------------------
pretime=-100
def movement_control():
    """Main movement and navigation control logic."""
    rospy.init_node('control')
    rospy.on_shutdown(clean_all)
    pub = rospy.Publisher('ultra_info', ultraInfo, queue_size=10)
    sub1 = rospy.Subscriber('gyro_sensor', Float32, callback1)
    sub = rospy.Subscriber('ultra_sensors', ultraInfo, callback)
    rate = rospy.Rate(10)

    # Initialize communication
    msg = ultraInfo()
    msg.nums = [0.017,1,1,1]
    pub.publish(msg)

    move_servos(0)
    time.sleep(0.2)
    prev_dist_err=0
    
    global dir1, sec, turn_flag,steering
    while yaw == 1081 or dists[r] == 0 or dists[l] == 0 or dists[f] == 0 or dists[b] == 0:
        pass

    while (not rospy.is_shutdown()):
        global pretime
        if sec==1:
            msg.nums = [0.012,1,1,1]
            pub.publish(msg)

        # Compute PID and control motion
        s = 80 if dir1==0 else max_speed
        dis = copy.deepcopy(dists)
        pid_control(speed= s,threshold= 0,dists = dis)

        # Detect open path and turn if needed
        if (dir1==0 and (dists[r]>100 or dists[l]>100 )) or (dir1!=0 and ((dists[r]>80 or dists[l]>80 ))):
            dir1 = int((dists[r]>100))-int((dists[l]>100)) if dir1==0 else dir1
            fr=0
            while dists[f]<40:
                pid_control(kp1=-0.5, kp2=0, speed = -1*s)
                fr=1
            if fr==1:
                stop_dc(0.2)
            sec+=1
            while (abs(yaw+float(dir1*sec*90)) > 10):
                pid_control(kp1 = 0.5,kp2 = 0,kd2 = 0)
            while (dists[l]>70 or dists[r]>70):
                pid_control(kp1 = 0.5,kp2 = 0,kd2 = 0)
            pretime=time.time()
                
        # Stop after completing full routine
        if sec == 12:
            msg.nums = [0.017,1,1,1]
            pub.publish(msg)
            dis = copy.deepcopy(dists)
            while dists[f]>150 or dists[b]<110:
                pid_control(speed = 80,dists = dis)
                dis = copy.deepcopy(dists)
            stop_dc(0.3)
            break

# ----------------------- Run Control -----------------------
try:
    T=time.time()
    movement_control()
    print(f'time =  {time.time()-T}')
except KeyboardInterrupt:
    GPIO.cleanup()
        
# Clean up hardware
pca.deinit()
GPIO.cleanup()
