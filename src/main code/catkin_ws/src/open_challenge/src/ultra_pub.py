import rospy
from std_msgs.msg import Float32
from open_challenge.msg import ultraInfo  # Custom message with a list of 4 distances
import RPi.GPIO as GPIO
import time

# Maximum time to wait for echo pulse (in seconds)
maxTime = 0.04

# GPIO pins connected to ultrasonic sensors
# [right, left, front, back] — order matters for consumers of the message
ultras = [17, 25, 27, 22]

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize ROS node
rospy.init_node(f"ultra_sensors_0")

# Set ROS loop rate (10 Hz)
rate = rospy.Rate(10)

# Global list to hold timing intervals for sleeping between readings
# num[0] = delay between readings (default 0.017s)
# The rest (1,2,3) are unused in the main loop but could represent number of samples per sensor
num = [0.017, 1, 1, 1]


# Function to read a single distance from an ultrasonic sensor
def get_distance(TRIG_PIN):
    # Set pin as output and send trigger pulse
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.0001)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # Set pin to input mode to listen for echo
    GPIO.setup(TRIG_PIN, GPIO.IN)

    # Wait for echo to start
    pulse_start = time.time()
    timeout = pulse_start + maxTime
    while GPIO.input(TRIG_PIN) == 0 and pulse_start < timeout:
        pulse_start = time.time()

    # Wait for echo to end
    pulse_end = time.time()
    timeout = pulse_end + maxTime
    while GPIO.input(TRIG_PIN) == 1 and pulse_end < timeout:
        pulse_end = time.time()

    # Calculate distance using duration of pulse
    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * 34300) / 2  # Speed of sound: 343 m/s

    return round(distance, 2)


# Callback function to update timing settings from another topic
def callback(data):
    global num
    num = data.nums  # List of timing/parameters sent from another ROS node


# Main function to continuously read distances and publish them
def ultrasonic_readings():
    # 0: right, 1: left, 2: front, 3: back
    pub = rospy.Publisher('ultra_sensors', ultraInfo, queue_size=10)
    sub = rospy.Subscriber('ultra_info', ultraInfo, callback)

    msg = ultraInfo()
    li = [0, 0, 0, 0]  # Placeholder list if needed later

    while not rospy.is_shutdown():
        # Get current delay time between readings
        t = num[0]

        # Read from each sensor with delays in between
        dist1 = get_distance(17)  # right
        time.sleep(t)
        dist3 = get_distance(27)  # front
        time.sleep(t)
        dist2 = get_distance(25)  # left
        time.sleep(t)
        dist4 = get_distance(22)  # back
        time.sleep(t)

        # Pack into message in correct order
        msg.nums = [dist1, dist2, dist3, dist4]

        # Publish the message
        pub.publish(msg)

        # Debugging print (disabled by default)
        # rospy.loginfo(f'distances: {msg.nums}')

        '''
        # Optional mode: take multiple samples per sensor and use the minimum (noise filtering)
        for i in range(4):
            ar = []
            for j in range(num[i]):
                ar.append(get_distance(ultras[i]))
                time.sleep(0.02)
            ar.sort()
            li[i] = ar[0]  # Use smallest (closest) reading
        msg.nums = li
        pub.publish(msg)
        '''
    # rospy.loginfo("Loop exited")  # For debugging outside the loop


# Run the ultrasonic reader until shutdown or Ctrl+C
try:
    ultrasonic_readings()
except KeyboardInterrupt:
    GPIO.cleanup()  # Cleanup GPIO pins when the program is stopped
