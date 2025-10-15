import rospy
from std_msgs.msg import Float32
from open_challenge.msg import ultraInfo

import rospy
from obstacle_challenge.srv import PillarDetection, PillarDetectionResponse
from obstacle_challenge.msg import Pillar
from picamera2 import Picamera2
import cv2
import numpy as np
import math 

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
import copy

# === GPIO / servo / steering configuration ===
IN1 = 18
IN2 = 12
left_servo = 11
right_servo = 12
front_servo = 15
ackerman_servo = 0 

delta = 0
front_delta = 0
left_midd = 85     # neutral or “straight” position for left servo
right_midd = 85   # same for right servo
front_midd = 90
ackerman_midd = 90        # the “center” angle for steering servo
max_steering = 38
min_steering = -76

# some state / section counters
sec = 0
dir1 = 1  # direction indicator: +1 or -1 for turning orientation

first_section = 1
first_section_color = 1
check_pillar = 1

prev_dist_err = 0
dist_err = 0

yaw_0 = 0
prev_yaw = 0
prev_yw = 0
unwrapped_yaw = 0
steering = 0

thr = [0, 0, 0, 0]

# sensor indices: r, l, f, b (right, left, front, back)
yaw = 360 * 3 + 1
r = 0
l = 1
f = 2
b = 3

outer = l if dir1 == 1 else r
inner = r if dir1 == 1 else l

# Setup GPIO for motor drive (DC)
GPIO.setwarnings(False)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
pi_pwm1 = GPIO.PWM(IN1, 5000)
pi_pwm2 = GPIO.PWM(IN2, 5000)
pi_pwm1.start(0)
pi_pwm2.start(0)

# Setup I2C and servos (via PCA9685)
i2c_servo = board.I2C()  
pca = PCA9685(i2c_servo)
pca.frequency = 50
servo0 = servo.Servo(pca.channels[right_servo], min_pulse=700, max_pulse=2900)
servo1 = servo.Servo(pca.channels[left_servo], min_pulse=700, max_pulse=2900)
servo2 = servo.Servo(pca.channels[ackerman_servo], min_pulse=500, max_pulse=2550)
servo3 = servo.Servo(pca.channels[front_servo], min_pulse=700, max_pulse=2900)

# distance arrays and filtering buffers
dists = [0, 0, 0, 0]               # filtered/usable distances for r, l, f, b
prev_dists = [0, 0, 0, 0]
curr_dists = [0, 0, 0, 0]
filtered_dists = [0, 0, 0, 0]
prev_filtered_dists = [0, 0, 0, 0]
raw_dists = [0, 0, 0, 0]
prefix_sum = [0, 0, 0, 0]
prev_dif1 = [0, 0, 0, 0]
cnt = [0, 0, 0, 0]
c = 0
prev_filtered_dif1 = [0, 0, 0, 0]

# To store pillars detected in each section
pillars_array = []
pillars_positions = []

max_speed = 80
value = 125
far_cw_pillar = 0
correction = 0

# === Pillar detection client helper ===
def pillar_detection_client():
    """
    Call the 'pillar_detection' ROS service to detect pillars.
    Based on first section or later logic, filter or store the pillars list
    into pillars_array for further navigation decisions.
    """
    global check_pillar, first_section, first_section_color, pillars_array, far_cw_pillar
    if first_section == 0:
        # after the first section, wait and call the service
        rospy.wait_for_service('pillar_detection')
        try:
            pillar_detection = rospy.ServiceProxy('pillar_detection', PillarDetection)
            request = PillarDetectionRequest()
            response = pillar_detection(request)
            temp = []
            n = len(response.pillars)
            print(f'n: {n}')
            # Some filtering logic: if too many pillars or disparity is large or with background distance, prune
            if (n > 2) or ((response.pillars[n-1].distance - response.pillars[0].distance > value)
                           and response.pillars[0].distance + dists[b] < 90) \
               or (response.pillars[0].distance + dists[b] >= 90 and n == 2):
                m = 1 if n == 2 else 2
                for i in range(m):
                    temp.append(response.pillars[i])
            else:
                for i in range(n):
                    temp.append(response.pillars[i])
            pillars_array.append(temp)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    else:
        # first section: simpler logic to decide if a pillar is present
        pillar_detection = rospy.ServiceProxy('pillar_detection', PillarDetection)
        request = PillarDetectionRequest()
        response = pillar_detection(request)
        # threshold distance depending on dir1
        di = 40 if dir1 == -1 else 90 
        if len(response.pillars) > 0:
            # if pillar is “far enough”, set check_pillar = 0 else 1
            check_pillar = 0 if response.pillars[0].distance > di else 1
            if di == 90 and response.pillars[0].distance > 40:
                far_cw_pillar = 1
            first_section = 0
            # record the color of the first pillar if in range
            first_section_color = response.pillars[0].color if response.pillars[0].distance < di else -1
        else:
            check_pillar = 0
            first_section = 0
        pillars_array.append([])

def clean_all():
    """ Stop motors and clean up GPIO on shutdown """
    move_dc(0, pi_pwm1, pi_pwm2)
    GPIO.cleanup()

def move_dc(speed, pi_pwm1, pi_pwm2):
    """
    Drive DC motors via PWM.
    Positive speed => forward, negative => reverse.
    Clip to max_speed.
    """
    speed = min(speed, max_speed)
    speed = max(speed, -max_speed)
    if speed > 0:
        pi_pwm1.ChangeDutyCycle(abs(speed))
        pi_pwm2.ChangeDutyCycle(0)
    else:
        pi_pwm2.ChangeDutyCycle(abs(speed))
        pi_pwm1.ChangeDutyCycle(0)

def move_servos(yaw):
    """
    Set servo angles based on yaw and steering (ackermann) and servo offsets.
    This sets left, right, front, and steering servos.
    """
    global steering, delta, front_delta, max_steering, min_steering
    
    angle0 = min(right_midd + yaw + delta, right_midd + 45 + delta)
    angle0 = max(angle0, right_midd - 45 + delta)
    
    angle1 = min(left_midd + yaw - delta, left_midd + 45 - delta)
    angle1 = max(angle1, left_midd - 45 - delta)
    
    angle2 = min(steering, max_steering)
    angle2 = max(angle2, min_steering)
    
    angle3 = min(front_midd + yaw + front_delta, front_midd + 55)
    angle3 = max(angle3, front_midd - 55)
    
    servo0.angle = angle0
    servo1.angle = angle1
    servo2.angle = round(ackerman_midd + angle2)
    servo3.angle = angle3

def steering_angle(corr):
    """
    Convert a “correction” (a desired offset) into an Ackermann steering angle.
    Uses some geometric approximations.
    """
    l = 135    #wheelbase of the Ackermann
    w = 125    #trackwidth of the Ackermann
    if corr != 0:
        r = l / math.tan(math.radians(corr))
        phiin = math.degrees(l / (r - w/2))
    else:
        phiin = 0
    phiin = math.radians(phiin + 23.578)
    xg = 50 - 25 * math.tan(phiin) / (math.sqrt(1 + (math.tan(phiin))**2))
    a = (1114 - xg**2 + (xg - 50)**2
         + 10 * math.sqrt(625 - (xg - 50)**2))
    A = math.sqrt(900 * xg**2 + 180**2)
    phi = math.degrees(math.atan(xg/6)) - 180
    ans = math.degrees(math.asin(a/A)) - phi - 90
    return -1 * ans

def pid_control(kp1=0.5, kp2=-0.75, kd2=0, speed=max_speed,
                threshold_dist=15, threshold_angle=0,
                sens=outer, dists=[15,15,15,15]):
    """
    PID-like control combining angular and distance corrections.
    - sens: which sensor (outer, inner, etc.) to use for lateral distance error.
    - threshold_dist: target distance from that sensor side.
    - threshold_angle: desired yaw offset.
    - kp1 handles angular (yaw) error, kp2 handles distance error.
    Sets steering by converting corr → steering angle, and moves DC motor.
    """
    global prev_dist_err, steering, dist_err
    # compute lateral distance error (positive or negative depending on side)
    dist_err = (dists[sens] - threshold_dist) if sens == l else (-dists[sens] + threshold_dist)
    
    angle_err = yaw + float(dir1 * sec * 90) - threshold_angle
    # if yaw error too large, we suppress correction gain
    if abs(yaw + float(dir1 * sec * 90)) > 60:
        kp2 = 0
    
    corr = (kp1 * (angle_err)
            + (kp2 * (dist_err) + kd2 * (dist_err - prev_dist_err)) * (abs(dist_err) > 0))
    # bound corr
    if corr > 25:
        corr = 25
    elif corr < -25:
        corr = -25
    steering = steering_angle(corr)
    move_dc(speed, pi_pwm1, pi_pwm2)

def factor():
    """
    Helper used in filtering: relative change between successive yaws
    returns 0 if nearly constant yaw, else ratio.
    """
    if abs(prev_yw + float(dir1 * sec * 90)) < 2:
        return 0
    else:
        return (abs(yaw + float(dir1 * sec * 90))
                / abs(prev_yw + float(dir1 * sec * 90)))

def dists_filter(sens):
    """
    Filter raw distance data to smooth sudden spikes or noise.
    Uses difference-based thresholding and prefix sums to detect anomalies.
    Updates dists[sens] (filtered) from raw_dists.
    """
    global dists, prev_dists, prev_dif1, prefix_sum, raw_dists, cnt
    global prev_filtered_dists, prev_filtered_dif1, filtered_dists, prev_yw, thr
    
    dif1 = raw_dists[sens] - prev_dists[sens]
    dif2 = dif1 - prev_dif1[sens]
    thr[sens] = dif2 * 0.5 if cnt[sens] == 0 else thr[sens]
    prefix_sum[sens] += dif2
    if abs(prefix_sum[sens]) > abs(thr[sens]) and abs(dif2) > 3:
        # anomaly detected: use filtered smoothing
        filtered_dists[sens] = prev_filtered_dists[sens] + prev_filtered_dif1[sens] * factor()
        dists[sens] = filtered_dists[sens] if cnt[sens] == 0 else raw_dists[sens]
        cnt[sens] += 1
    else:
        # no anomaly: accept raw reading
        prefix_sum[sens] = 0
        cnt[sens] = 0
        dists[sens] = raw_dists[sens]
        filtered_dists[sens] = dists[sens]
    prev_yw = yaw
    prev_dif1[sens] = dif1
    
    prev_filtered_dif1[sens] = filtered_dists[sens] - prev_filtered_dists[sens]
    if abs(prev_filtered_dif1[sens]) > 7:
        prev_filtered_dif1[sens] = 0

def callback(dist):
    """
    ROS subscriber callback for ultra sensor readings (ultraInfo).
    Stores raw data, updates filters, and shifts prev values.
    """
    global dists, prev_dists, raw_dists, filtered_dists
    global c, prev_dist_err
    
    prev_dists = copy.deepcopy(raw_dists)
    prev_filtered_dists = copy.deepcopy(filtered_dists)
    raw_dists = list(dist.nums)
    if c == 2:
        # After warm-up, do filtering on each sensor
        dists_filter(f)
        dists_filter(b)
        dists_filter(outer)
        dists_filter(inner)
    else:
        # initialization phase: directly accept raw
        dists = copy.deepcopy(raw_dists)
        filtered_dists = copy.deepcopy(dists)
        c += 1
    prev_dist_err = dist_err

def callback1(data):
    """
    ROS subscriber callback for gyro / yaw sensor (Float32).
    Unwraps yaw to full continuous angle and updates servo positions.
    """
    global yaw, prev_yw, yaw_0, correction
    yaw = round(unwrap_yaw(data.data)) - correction - yaw_0
    move_servos(yaw + float(dir1 * sec * 90))

def unwrap_yaw(current_yaw):
    """
    Convert a raw yaw reading (0–360) to a continuous, unwrapped yaw (cumulative).
    Handles wrap-around at 0/360 transitions.
    """
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

prevTime = 0

def stop_dc(spd, delay):
    """ Send a small burst, then stop DC drive, with delay """
    move_dc(spd, pi_pwm1, pi_pwm2)
    time.sleep(0.2)
    move_dc(0, pi_pwm1, pi_pwm2)
    time.sleep(delay)

last_pillar = -1

def pass_section():
    """
    Navigate through a section bounded by pillars.
    Uses first pillar(s) data in that section to choose path, then 
    uses pid_control to follow desired spacing while passing.
    """
    global anti_pillar_sens, dir1, front_delta, last_pillar, pillar_sens
    # set front servo offset (turning slightly toward opposite side of pillar)
    front_delta = -15 if pillars_array[sec%4][-1].color == 1 else 15
    pillar_sens = r if pillars_array[sec%4][0].color == 0 else l
    pass_pillar(pillars_array[sec%4][0].color, pillars_array[sec%4][0].distance + dists[b])
    if len(pillars_array[sec%4]) > 1:
        pillar_sens = r if pillars_array[sec%4][1].color == 0 else l
        prevTime = time.time()
        th = 12 if last_pillar == pillars_array[sec%4][1].color else 72
        dis = copy.deepcopy(dists)
        while time.time() - prevTime < 0.75:
            pid_control(threshold_dist=th, threshold_angle=0, sens=anti_pillar_sens, dists=dis)
            dis = copy.deepcopy(dists)
        pass_pillar(pillars_array[sec%4][1].color, pillars_array[sec%4][1].distance)
    else:
        x = 12
        y = 72
        thresh = (y + x)/2 - dir1 * (y - x)/2 if last_pillar == 0 else (y + x)/2 + dir1 * (y - x)/2
        dis = copy.deepcopy(dists)
        prevTime = time.time()
        while time.time() - prevTime < 0.5 and dists[inner] < 100:
            pid_control(threshold_dist=13, threshold_angle=0, sens=anti_pillar_sens, dists=dis)
            dis = copy.deepcopy(dists)
        dis = copy.deepcopy(dists)
        while dists[f] > 100 and dists[inner] < 100:
            pid_control(threshold_dist=thresh, threshold_angle=0, sens=outer, dists=dis)
            dis = copy.deepcopy(dists)
        print(dists[f], dists[inner])

def pass_pillar(color, distance):
    """
    Approach and pass a single pillar of a given color.
    Adjust lateral threshold, track until aligned, then drive past.
    """
    global last_pillar, dists, pillar_sens, anti_pillar_sens, max_speed
    anti_pillar_sens = l if color == 0 else r
    x = 12
    y = 72
    thresh = (y + x)/2 - dir1 * (y - x)/2 if color == 0 else (y + x)/2 + dir1 * (y - x)/2
    dis = copy.deepcopy(dists)
    # approach until outer side is roughly at threshold
    while abs(dists[outer] - thresh) > 10:
        pid_control(threshold_dist=thresh, threshold_angle=0, sens=outer, dists=dis)
        dis = copy.deepcopy(dists)
    dis = copy.deepcopy(dists)
    # then approach until pillar sensor side is close
    while dists[pillar_sens] > 40:
        pid_control(threshold_dist=thresh, threshold_angle=0, sens=outer, dists=dis)
        dis = copy.deepcopy(dists)
    # optionally follow upstream logic when in corner
    while thresh == y and dists[inner] > x + 7.5 and distance < 90:
        pid_control(kp2=0, kd2=0)
    print(f'color={color}')
    last_pillar = color

def pass_section_pink():
    """
    Similar to the pass_section but in the parking section
    """
    global anti_pillar_sens, dir1, front_delta, last_pillar, p
    pass_pillar_pink(pillars_array[sec%4][0].color, pillars_array[sec%4][0].distance + dists[b])
    p = 1
    if len(pillars_array[sec%4]) > 1:
        pink_thresh = 10.5
        x = 12
        y = 54
        thresh = (y + x)/2 - dir1 * (y - x)/2 if pillars_array[sec%4][1].color == 1 else (y + x)/2 + dir1 * (y - x)/2
        angle = 0 + 45 * ((last_pillar == 1) - (last_pillar == 0)) * (last_pillar != pillars_array[sec%4][1].color)
        prevTime = time.time()
        while time.time() - prevTime < 0.5:
            pid_control(kp2=0, kd2=0, speed=70, threshold_angle=angle)
        pass_pillar_pink(pillars_array[sec%4][1].color, pillars_array[sec%4][1].distance)
        front_delta = -15 if last_pillar else 15
    else:
        x = 30
        y = 72
        pink_thresh = 25
        front_delta = -15 if last_pillar else 15
        thresh = (y + x)/2 - dir1 * (y - x)/2 if last_pillar == 0 else (y + x)/2 + dir1 * (y - x)/2
        prevTime = time.time()
        while time.time() - prevTime < 0.5 and dists[inner] < 100:
            pid_control(kp2=0, kd2=0, speed=70, threshold_angle=0)
        dis = copy.deepcopy(dists)
        while dists[f] > 100 and dists[inner] < 100:
            dis[outer] = copy.deepcopy(thresh) if (82 - (dis[r] + dis[l])) > 3 else dis[outer]
            pid_control(kp2=-0.75, speed=70, threshold_dist=thresh, threshold_angle=0, sens=outer, dists=dis)
            dis = copy.deepcopy(dists)
    p = 0

def pass_pillar_pink(color, distance):
    """
    Similar to pass_pillar but in the parking section
    """
    global last_pillar, dists, pillar_sens, anti_pillar_sens, p
    sensor = outer if p == 0 else inner
    x = 30 if p == 0 else 54
    y = 72 if p == 0 else 12
    thresh = (y + x)/2 - dir1 * (y - x)/2 if color == 0 else (y + x)/2 + dir1 * (y - x)/2
    pillar_sens = r if color == 0 else l
    anti_pillar_sens = l if color == 0 else r
    dis = copy.deepcopy(dists)
    while abs(dists[sensor] - thresh) > 10:
        dis[sensor] = copy.deepcopy(thresh) if ((82 - (dis[r] + dis[l])) > 3 and sensor == outer) else dis[sensor]
        pid_control(kp2=-0.75, speed=70, threshold_dist=thresh, threshold_angle=0, sens=sensor, dists=dis)
        dis = copy.deepcopy(dists)
    dis = copy.deepcopy(dists)
    while dists[pillar_sens] > 20:
        dis[sensor] = copy.deepcopy(thresh) if (82 - (dis[r] + dis[l]) > 3 and sensor == outer) else dis[sensor]
        pid_control(kp2=-0.75, speed=70, threshold_dist=thresh, threshold_angle=0, sens=sensor, dists=dis)
        dis = copy.deepcopy(dists)
    while thresh == y and dists[inner] > x + 7.5 and distance < 90 and p == 0:
        pid_control(kp2=0, kd2=0)
    print(f'color={color}')
    last_pillar = color

def pass_corner():
    """
    After passing a section, perform turning or corner maneuver.
    Adjust steering and motor until orientation and distances align for next segment.
    """
    global dists, sec, last_pillar, pillar_sens, delta, front_delta, correction
    minspd = 40
    if pillar_sens == outer:
        delta = -15
        move_servos(yaw + float(dir1 * sec * 90))
        while abs(yaw + float(dir1 * sec * 90)) < 15:
            pid_control(kp2=0, speed=60, threshold_angle=dir1 * 50)
        while dists[f] > 17:
            pid_control(kp2=0, speed=minspd - 15 + dists[f], threshold_angle=0)
        while abs(yaw + float(dir1 * sec * 90)) < 85:
            pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60 + abs(yaw + float(dir1 * sec * 90)) - 85, threshold_angle=-90 * dir1)
        while dists[b] > 40:
            pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60, threshold_angle=-90 * dir1)
        stop_dc(0, 0.1)
        while dists[b] < 20:
            pid_control(kp1=0.5, kp2=0, kd2=0, speed=60, threshold_angle=-90 * dir1)
        delta = 0
    else:
        # similar logic for the other turn orientation
        if sec % 4 != 0:
            while dists[f] > 80:
                pid_control(kp1=0.5, kp2=0, kd2=0)
            while abs(yaw + float(dir1 * sec * 90)) < 75 and dists[f] > 15:
                pid_control(kp1=0.5, kp2=0, kd2=0, speed=minspd - abs(yaw + float(dir1 * sec * 90)) + 75, threshold_angle=-90 * dir1)
            while abs(yaw + float(dir1 * sec * 90)) < 85:
                pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60 + abs(yaw + float(dir1 * sec * 90)) - 85, threshold_angle=-90 * dir1)
            while dists[b] > 40:
                pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60, threshold_angle=-90 * dir1)
        else:
            move_servos(yaw + float(dir1 * sec * 90))
            while abs(yaw + float(dir1 * sec * 90)) < 15:
                pid_control(kp2=0, speed=60, threshold_angle=-dir1*50)
            while dists[f] > 17:
                pid_control(kp2=0, speed=minspd - 15 + dists[f], threshold_angle=0)
            while abs(yaw + float(dir1 * sec * 90)) < 85:
                pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60 + abs(yaw + float(dir1 * sec * 90)) - 85, threshold_angle=-90 * dir1)
            while dists[b] > 40:
                pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60, threshold_angle=-90 * dir1)
            stop_dc(0, 0.1)
            while dists[b] < 20:
                pid_control(kp1=0.5, kp2=0, kd2=0, speed=60, threshold_angle=-90 * dir1)
    front_delta = 0
    sec += 1
    correction = (dir1 * sec / 4)
    stop_dc(0, 0.3)


def set_point_before_parking():
    """
    Move into a position just before entering the final parking slot.
    Align lateral distance and yaw before reversing into the spot.
    """
    global front_delta
    dis = copy.deepcopy(dists)
    while (abs(dists[outer] - 23) > 2 or abs(yaw + float(dir1 * sec * 90)) > 3) and dists[f] > 10:
        dis[outer] = 23 if dis[outer] < 15 else dis[outer]
        pid_control(speed=50, threshold_dist=23, threshold_angle=0, sens=outer, dists=dis)
        dis = copy.deepcopy(dists)
    while dists[f] > 10:
        dis[outer] = 23 if dis[outer] < 15 else dis[outer]
        pid_control(speed=50, threshold_dist=23, threshold_angle=0, sens=outer, dists=dis)
        dis = copy.deepcopy(dists)
    front_delta = 0
    stop_dc(-50, 0.3)
    dis = copy.deepcopy(dists)
    while dists[f] < 73:
        dis[outer] = 23 if dis[outer] < 15 else dis[outer]
        pid_control(kp1=-0.5, speed=-50, threshold_dist=23, threshold_angle=0, sens=outer, dists=dis)
        dis = copy.deepcopy(dists)
    stop_dc(50, 0.3)

def pre_parking():
    """
    Prepares the robot to enter the parking maneuver.
    May reorient, reassign direction, and call pass_section logic in reverse or adjusted path.
    """
    global anti_pillar_sens, pillar_sens, dir1, front_delta, last_pillar, p
    global sec, dir1, yaw_0, outer, inner, f, pillars_array, correction
    
    if dir1 == 1:
        if len(pillars_array[sec%4]) > 1:
            pillars_array[sec%4][1].color = 1
            pass_section_pink()
        else:
            if check_pillar == 1:
                pass_pillar_pink(1, pillars_array[sec%4][0].distance + dists[b])
            else:
                pass_pillar_pink(pillars_array[sec%4][0].color, pillars_array[sec%4][0].distance + dists[b])
            front_delta = -15 if last_pillar else 15
            prevTime = time.time()
            while time.time() - prevTime < 0.5 and dists[inner] < 100:
                pid_control(kp2=0, kd2=0, speed=70, threshold_angle=0)
            
            dis = copy.deepcopy(dists)
            while (dists[inner] - 11.5) > 5 and dists[inner] < 100:
                pid_control(speed=70, threshold_dist=11.5, threshold_angle=0, sens=inner, dists=dis)
                dis = copy.deepcopy(dists)
            dis = copy.deepcopy(dists)
            while dists[f] > 80 and dists[inner] < 100:
                pid_control(speed=70, threshold_dist=11.5, threshold_angle=0, sens=inner, dists=dis)
                dis = copy.deepcopy(dists)
        while abs(yaw + float(dir1 * sec * 90)) < 87:
            pid_control(kp2=0, kd2=0, speed=70, threshold_angle=90)
        while abs(yaw + float(dir1 * sec * 90)) < 175 and dists[b] > 20:
            pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-70, threshold_angle=180)
        stop_dc(80, 0.3)
        
        sec = 0
        correction = 3
        dir1 = -1
        outer = r
        inner = l
        yaw_0 = round(yaw / 90) * 90
        stop_dc(-80, 0.3)
        pillars_array[sec%4][0].color = 1
        if len(pillars_array[sec%4]) > 1:
            pillars_array[sec%4][1].color = 1
        pass_section_pink()
        set_point_before_parking()
    else:
        # symmetric logic if dir1 == -1
        if len(pillars_array[sec%4]) > 1:
            pillars_array[sec%4][1].color = 1
            pass_section_pink()
        else:
            if check_pillar == 1:
                pillars_array[sec%4][0].color = 1
                pass_section_pink()
            else:
                pass_pillar_pink(pillars_array[sec%4][0].color, pillars_array[sec%4][0].distance + dists[b])
                k = 2.5
                front_delta = -15 if last_pillar else 15
                th = 25 if (anti_pillar_sens == outer) else 50
                dis = copy.deepcopy(dists)
                prevTime = time.time()
                while time.time() - prevTime < 0.5 and dists[inner] < 100:
                    dis[outer] = th if dis[outer] < th - 10 else dis[outer]
                    pid_control(kp2=0, kd2=0, speed=60, threshold_dist=th, threshold_angle=0, sens=anti_pillar_sens, dists=dis)
                    dis = copy.deepcopy(dists)
        set_point_before_parking()

def parking():
    """
    The final parking maneuver. Reverse, steer, and align in the parking slot.
    """
    global steering
    steering = min_steering * (dir1 == -1) + max_steering * (dir1 == 1)
    while abs(yaw + float(dir1 * sec * 90)) < 40:
        move_dc(-60, pi_pwm1, pi_pwm2)
    steering = 0
    stop_dc(0, 0.3)
    while dists[inner] < 68:
        move_dc(-60, pi_pwm1, pi_pwm2)
    steering = max_steering * (dir1 == -1) + min_steering * (dir1 == 1)
    stop_dc(0, 0.3)
    while dists[outer] > 5 and dists[b] > 10:
        pid_control(kp1=-3, kp2=0, kd2=0, speed=-60, threshold_dist=3, threshold_angle=0)
    d = dists[outer]
    while dists[inner] < 82 or abs(yaw + float(dir1 * sec * 90)) > 3:
        prevTime = time.time()
        while dists[f] < 15 and (dists[inner] < 82 or abs(yaw + float(dir1 * sec * 90)) > 3) and time.time() - prevTime < 0.25:
            move_dc(-50, pi_pwm1, pi_pwm2)
            steering = min_steering * (dir1 == -1) + max_steering * (dir1 == 1)
        prevTime = time.time()
        while dists[f] < 15 and (dists[inner] < 82 or abs(yaw + float(dir1 * sec * 90)) > 3):
            move_dc(-50, pi_pwm1, pi_pwm2)
            steering = max_steering * (dir1 == -1) + min_steering * (dir1 == 1)
        thresh = dists[inner] + 0.5
        dis = copy.deepcopy(dists)
        while dists[f] > 7 and (dists[inner] < 82 or abs(yaw + float(dir1 * sec * 90)) > 3):
            pid_control(kp1=2, kp2=0, speed=50)
            dis = copy.deepcopy(dists)
    while dists[f] < 12:
        pid_control(kp1=-1.5, kp2=0, kd2=0, speed=-50, threshold_angle=0)
        flag = 1
    while dists[f] > 12:
        pid_control(kp1=1.5, kp2=0, kd2=0, speed=50, threshold_angle=0)
        flag = -1
    stop_dc(flag * 50, 0.1)
    steering = 0

def starting_section():
    """
    The logic executed at the beginning of navigation.
    Decide turning direction, drive forward, detect pillars, and begin first pass or corner.
    """
    global steering, outer, inner, front_delta, p, dir1, sec, delta
    if dists[r] < dists[l]:
        dir1 = -1
    else:
        dir1 = 1
    outer = l if dir1 == 1 else r
    inner = r if dir1 == 1 else l
    steering = max_steering * (dir1 == -1) + min_steering * (dir1 == 1)
    stop_dc(0, 0.1)
    ang = 40 if dir1 == -1 else 30
    while abs(yaw + float(dir1 * sec * 90)) < ang:
        move_dc(70, pi_pwm1, pi_pwm2)
    steering = 0
    stop_dc(0, 0.1)
    while dists[inner] < 66 and abs(yaw + float(dir1 * sec * 90)) < 90:
        move_dc(-70, pi_pwm1, pi_pwm2)
    stop_dc(0, 0.1)
    pillar_detection_client()
    if check_pillar == 1:
        if far_cw_pillar == 0 or dir1 == -1:
            front_delta = float(dir1 * 90)
            while abs(yaw + float(dir1 * sec * 90)) < 70 and dists[b] > 6:
                pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60, threshold_angle=-float(dir1 * 90))
            while dists[f] > 10:
                pid_control(kp2=0, kd2=0, speed=60, threshold_angle=-float(dir1 * 100))
            stop_dc(-80, 0.1)
            while abs(yaw + float(dir1 * sec * 90)) > 3 or (dir1 == 1) * (dists[b] > 80):
                pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-70, threshold_angle=0)
            front_delta = 0
        if far_cw_pillar == 1 and dir1 == 1:
            while dists[inner] > 60 and abs(yaw + float(dir1 * sec * 90)) < 90:
                move_dc(70, pi_pwm1, pi_pwm2)
            p = 1
        if dir1 == -1:
            p = 1
        pass_pillar_pink(first_section_color, 10)
        p = 0
        pass_corner()
    else:
        steering = 0
        stop_dc(0, 0.1)
        while dists[inner] > 60 and abs(yaw + float(dir1 * sec * 90)) < 90:
            move_dc(70, pi_pwm1, pi_pwm2)
        minspd = 40
        delta = -15
        dis = copy.deepcopy(dists)
        while abs(dists[outer] - 60) > 7:
            if (dists[outer] < 55 and sec % 4 != 0) or dists[inner] + dists[outer] < 80:
                dis[outer] = max(dis[outer], 60)
            pid_control(speed=minspd - 15 + dists[f], threshold_dist=60, sens=outer, dists=dis)
            dis = copy.deepcopy(dists)
        dis = copy.deepcopy(dists)
        while dists[f] > 17:
            if (dists[outer] < 55 and sec % 4 != 0) or dists[inner] + dists[outer] < 80:
                dis[outer] = max(dis[outer], 60)
            pid_control(speed=minspd - 15 + dists[f], threshold_dist=60, sens=outer, dists=dis)
            dis = copy.deepcopy(dists)
        while abs(yaw + float(dir1 * sec * 90)) < 85 and dists[b] > 20:
            pid_control(kp1=-0.5, kp2=0, kd2=0, speed=-60 + abs(yaw + float(dir1 * sec * 90)) - 85, threshold_angle=-90 * dir1)
        delta = 0
        sec += 1
        correction = (dir1 * sec / 4)
    stop_dc(0, 0.2)

pretime = -100

def movement_control():
    """
    Main control loop:
    - initialize ROS node/subscribers/publishers,
    - wait for sensors to stabilize,
    - sequentially call starting, passing, cornering, and finally parking logic.
    """
    rospy.init_node('control')
    rospy.on_shutdown(clean_all)
    pub = rospy.Publisher('ultra_info', ultraInfo, queue_size=10)
    sub1 = rospy.Subscriber('gyro_sensor', Float32, callback1)
    sub = rospy.Subscriber('ultra_sensors', ultraInfo, callback)
    rate = rospy.Rate(10)
    msg = ultraInfo()
    msg.nums = [0.017, 1, 1, 1]
    pub.publish(msg)

    move_servos(0)
    time.sleep(0.2)
    prev_dist_err = 0
    
    global dir1, sec, turn_flag, steering
    # wait until yaw & distances are ready
    while yaw == 1081 or dists[r] == 0 or dists[l] == 0 or dists[f] == 0 or dists[b] == 0:
        pass
    while not rospy.is_shutdown():
        starting_section()
        msg.nums = [0.012, 1, 1, 1]
        pub.publish(msg)
        while sec < 12:
            print(sec)
            if sec < 5:
                pillar_detection_client()
                print(len(pillars_array[sec]))
                print(pillars_array[sec])
            if sec == 4:
                pillars_array[0] = pillars_array[4]
            if sec % 4 != 0:
                pass_section()
            else:
                msg.nums = [0.017, 1, 1, 1]
                pub.publish(msg)
                pass_section_pink()
                msg.nums = [0.012, 1, 1, 1]
                pub.publish(msg)
            pass_corner()
        pillar_detection_client()
        pillars_array[0] = pillars_array[sec % 4]
        msg.nums = [0.017, 1, 1, 1]
        pub.publish(msg)
        pre_parking()
        parking()
        stop_dc(0, 1)
        break

try:
    T = time.time()
    time.sleep(1)
    movement_control()
    print(f'time= {time.time() - T}')
except KeyboardInterrupt:
    GPIO.cleanup()

pca.deinit()
GPIO.cleanup()
