import cv2
import numpy as np
import time
import serial
from picamera2 import Picamera2

def setup_camera():
    # Initialize the camera and configure settings
    camera = Picamera2()
    config = camera.create_preview_configuration()
    config['main']['format'] = 'RGB888'
    config['controls']['FrameDurationLimits'] = (66666, 66666)
    config['controls']['ExposureTime'] = 10000
    config['controls']['AnalogueGain'] = 4.0
    camera.configure(config)
    camera.start()
    return camera

def setup_serial_connection(port='/dev/ttyACM0', baud_rate=115200):
    # Setup the serial connection with the given port and baud rate
    connection = serial.Serial(port, baud_rate, timeout=1)
    connection.flush()
    connection.write("1#".encode('ascii'))
    time.sleep(1)
    return connection

def detect_pillars(frame, red_bounds_1, red_bounds_2, green_bounds):
    # Detect red and green pillars in the given frame
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    red_mask_1 = cv2.inRange(hsv_frame, *red_bounds_1)
    red_mask_2 = cv2.inRange(hsv_frame, *red_bounds_2)
    red_mask = red_mask_1 ^ red_mask_2
    green_mask = cv2.inRange(hsv_frame, *green_bounds)
    
    red_pillars = find_contours(red_mask, min_area=200, color='R')
    green_pillars = find_contours(green_mask, min_area=300, color='G')
    
    return red_pillars, green_pillars

def find_contours(mask, min_area, color):
    # Find contours in the mask and return information about detected pillars
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pillars = []
    for contour in contours:
        if cv2.contourArea(contour) > min_area:
            moments = cv2.moments(contour)
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            area = cv2.contourArea(contour)
            distance = calculate_distance(area, color)
            pillars.append([cx, cy, color, area, distance])
    return pillars

def calculate_distance(area, color):
    # Calculate distance based on the area and color of the pillar
    if color == 'R':
        return 4532.2 * pow(area, -0.532)
    elif color == 'G':
        return 6165.4 * pow(area, -0.562)

def determine_pillar_data(red_pillars, green_pillars, clockwise):
    # Determine pillar data from the detected red and green pillars
    all_pillars = red_pillars + green_pillars
    all_pillars.sort(key=lambda p: p[0], reverse=clockwise)
    return all_pillars

def send_pillar_data(connection, pillars):
    # Send pillar data over the serial connection
    for pillar in pillars:
        data = f"{pillar[0]},{pillar[1]},{pillar[2]},{pillar[3]:.2f},{pillar[4]:.2f}#"
        connection.write(data.encode('ascii'))
        time.sleep(0.1)

def main():
    # Main function to run the vision system
    camera = setup_camera()
    connection = setup_serial_connection()
    
    red_bounds_1 = (0, 100, 100), (10, 255, 255)
    red_bounds_2 = (160, 100, 100), (180, 255, 255)
    green_bounds = (50, 100, 100), (70, 255, 255)
    
    try:
        while True:
            frame = camera.capture_array()
            red_pillars, green_pillars = detect_pillars(frame, red_bounds_1, red_bounds_2, green_bounds)
            all_pillars = determine_pillar_data(red_pillars, green_pillars, clockwise=True)
            send_pillar_data(connection, all_pillars)
            time.sleep(0.1)
    except KeyboardInterrupt:
        camera.stop()
        connection.close()

if __name__ == "__main__":
    main()