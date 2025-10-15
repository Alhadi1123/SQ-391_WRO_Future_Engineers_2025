import rospy
from obstacle_challenge.srv import PillarDetection, PillarDetectionResponse
from obstacle_challenge.msg import Pillar
from picamera2 import Picamera2
import time
import cv2
import numpy as np

class PillarDetectionServer:
    def __init__(self):
        self.processing_width = 800
        self.processing_height = int(self.processing_width * 3*7/6/6)
        # Initialize camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={'size': (3072, 1792)})
        config['main']['format'] = 'RGB888'
        config['controls']['ExposureTime'] = 20000
        config['controls']['AnalogueGain'] = 7
        self.picam2.configure(config)
        self.picam2.start()
        
        # Create ROS service
        self.service = rospy.Service('pillar_detection', PillarDetection, self.handle_detection_request)
        
    def detect_and_sort_pillars(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        # Define HSV ranges for red (two ranges) and green
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        lower_green = np.array([40, 80, 75])
        upper_green = np.array([80, 255, 255])
        
        # Create masks for red and green
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        # Apply morphological operations to clean the masks
        kernel = np.ones((5, 5), np.uint8)
        cleaned_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        cleaned_red = cv2.morphologyEx(cleaned_red, cv2.MORPH_CLOSE, kernel)
        cleaned_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        cleaned_green = cv2.morphologyEx(cleaned_green, cv2.MORPH_CLOSE, kernel)
        
        # Find contours for red and green separately
        red_contours, _ = cv2.findContours(cleaned_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(cleaned_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        pillars = []
        min_area = 100
        
        # Process red pillars
        for contour in red_contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
                
            x, y, w, h = cv2.boundingRect(contour)
            if w == 0:
                continue
                
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            distance = np.sqrt((centroid_x - image.shape[1] // 2)**2 + (centroid_y - image.shape[0] // 2)**2) / w
            distance = distance*16.1454 + 9.4854
            pillars.append((centroid_x, centroid_y, distance, 'red'))
        
        # Process green pillars
        for contour in green_contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
                
            x, y, w, h = cv2.boundingRect(contour)
            if w == 0:
                continue
                
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            distance = np.sqrt((centroid_x - image.shape[1] // 2)**2 + (centroid_y - image.shape[0] // 2)**2) / w
            distance = distance*16.1454 + 9.4854
            pillars.append((centroid_x, centroid_y, distance, 'green'))
        
        # Sort pillars by distance (ascending: closer pillars first)
        pillars_sorted = sorted(pillars, key=lambda x: x[2])
        
        return pillars_sorted
    
    def handle_detection_request(self, req):
        try:
            # Capture and process image
            frame = self.picam2.capture_array()
            processed_frame = cv2.resize(frame, (self.processing_width, self.processing_height))
            frame_bgr = cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)
            
            # Detect pillars
            pillars_sorted = self.detect_and_sort_pillars(frame_bgr)
            
            # Create response
            response = PillarDetectionResponse()
            
            # Populate response with pillar data
            for centroid_x, centroid_y, distance, color in pillars_sorted:
                pillar = Pillar()
                pillar.centroid_x = centroid_x
                pillar.centroid_y = centroid_y
                pillar.distance = distance
                pillar.color = 1 if color=='red' else 0
                response.pillars.append(pillar)

            return response
            
        except Exception as e:
            rospy.logerr(f"Error in pillar detection: {str(e)}")
            return PillarDetectionResponse()

    def shutdown(self):
        self.picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('pillar_detection_server')
    server = PillarDetectionServer()
    rospy.on_shutdown(server.shutdown)
    rospy.spin()
