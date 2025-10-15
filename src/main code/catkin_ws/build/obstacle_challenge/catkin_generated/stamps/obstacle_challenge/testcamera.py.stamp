from picamera2 import Picamera2, Preview
import time
import rospy
from std_msgs.msg import Int8
# Initialize the camera
picam2 = Picamera2()

# Configure the camera for preview

config = picam2.create_preview_configuration(main={'size':(2048,1536)})

config['main']['format'] = 'RGB888'
#config['controls']['FrameDurationLimits'] = (66666, 66666)
config['controls']['ExposureTime'] = 10000
config['controls']['AnalogueGain'] = 4.0
picam2.configure(config)
# Start the camera preview
#picam2.start_preview(Preview.QT)

# Start the camera
picam2.start()

rospy.init_node('camera1')
pub=rospy.Publisher('camera_pub',Int8,queue_size=10)
rate=rospy.Rate(60)

try:
    # Keep the script running to display the preview
    while True:
        frames=picam2.capture_array()
        pub.publish(1)
        rate.sleep()
        # Keep the script alive
except KeyboardInterrupt:
    # Stop the camera and preview when the user presses Ctrl+C
    print("Stopping the camera...")
finally:
    picam2.stop_preview()
    picam2.stop()
