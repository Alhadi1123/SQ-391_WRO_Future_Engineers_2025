import rospy
from std_msgs.msg import Float32
from open_challenge.msg import ultraInfo
import RPi.GPIO as GPIO
import time

maxTime = 0.04
ultras = [17,25,27,22]
#[right,left,front,back]
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
rospy.init_node(f"ultra_sensors_0")
   
rate = rospy.Rate(10)
num = [1,1,1,1]
def get_distance(TRIG_PIN):
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.0001)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    GPIO.setup(TRIG_PIN, GPIO.IN)
    pulse_start = time.time()
    timeout= pulse_start + maxTime
    while GPIO.input(TRIG_PIN) == 0 and pulse_start < timeout:
        pulse_start = time.time()
    pulse_end = time.time()
    timeout= pulse_end + maxTime
    while GPIO.input(TRIG_PIN) == 1 and pulse_end < timeout:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * 34300) / 2
    return round(distance,2)

def callback(data):
    global num
    num = data.nums

def ultrasonic_readings():
    # 0: right, 1:left, 2:front, 3:back
    pub = rospy.Publisher('ultra_sensors', ultraInfo , queue_size=10)
    sub = rospy.Subscriber('ultra_info',ultraInfo,callback) 
    msg = ultraInfo()
    li=[0,0,0,0]
    while not rospy.is_shutdown():
        dist1 = get_distance(17)
        time.sleep(0.017)
        dist2 = get_distance(25)
        time.sleep(0.017)
        dist3 = get_distance(27)
        time.sleep(0.017)
        dist4 = get_distance(22)
        msg.nums = [dist1,dist2,dist3,dist4]
        pub.publish(msg)
        time.sleep(0.017)
        

try:
    ultrasonic_readings()
except KeyboardInterrupt:
    GPIO.cleanup()
