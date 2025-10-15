import rospy
from obstacle_challenge.srv import PillarDetection, PillarDetectionResponse
from obstacle_challenge.msg import Pillar
from picamera2 import Picamera2
import time
import cv2
import numpy as np

sec = 0  # Global counter for saving images (if enabled)

class PillarDetectionServer:
    def __init__(self):
        # Define image size for processing
        self.processing_width = 800
        self.processing_height = int(self.processing_width * 3*7/6/6)  # Maintain aspect ratio

        # Initialize PiCamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={'size': (3072, 1792)})
        config['main']['format'] = 'RGB888'  # Use RGB color format
        config['controls']['ExposureTime'] = 20000  # Manual exposure
        config['controls']['AnalogueGain'] = 7      # Manual gain
        self.picam2.configure(config)
        self.picam2.start()

        # Initialize ROS service
        self.service = rospy.Service('pillar_detection', PillarDetection, self.handle_detection_request)

    def detect_and_sort_pillars(self, image):
        # Convert image to HSV for easier color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Define HSV color ranges for red, green, and pink
        lower_red2 = np.array([116, 61, 75])
        upper_red2 = np.array([172, 255, 255])

        lower_green = np.array([27, 61, 55])
        upper_green = np.array([68, 255, 255])

        lower_pink = np.array([126, 98, 44])
        upper_pink = np.array([134, 255, 255])

        # Create masks for each color
        mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)
        mask_red = cv2.inRange(hsv, lower_red2, upper_red2)

        # Remove pink regions from red mask (to avoid overlap/misclassification)
        mask_red = cv2.bitwise_xor(mask_pink, mask_red)

        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Clean masks using morphological operations (remove noise)
        kernel = np.ones((5, 5), np.uint8)
        cleaned_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        cleaned_red = cv2.morphologyEx(cleaned_red, cv2.MORPH_CLOSE, kernel)

        cleaned_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        cleaned_green = cv2.morphologyEx(cleaned_green, cv2.MORPH_CLOSE, kernel)

        # Find contours in the cleaned masks
        red_contours, _ = cv2.findContours(cleaned_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(cleaned_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pillars = []  # List to store detected pillar data
        min_area = 100  # Ignore small contours (likely noise)

        # --- RED PILLAR DETECTION ---
        for contour in red_contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w == 0 or w > h:
                continue  # Ignore wide or flat contours

            # Calculate centroid and distance
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            distance = np.sqrt((centroid_x - image.shape[1] // 2) ** 2 +
                               (centroid_y - image.shape[0] // 2) ** 2) / w
            distance = distance * 16.1454 + 9.4854  # Calibrated distance formula

            # Heuristic checks for shape and position
            if (abs(h / w - 2) > 0.3 and distance < 100 and abs(centroid_x - image.shape[1] // 2) < 40) or \
               (abs(area / (w * h)) < 0.75 and abs(centroid_x - image.shape[1] // 2) < 70):
                # Try a slightly shifted centroid_y to improve detection
                centroid_y += 10
                distance = np.sqrt((centroid_x - image.shape[1] // 2) ** 2 +
                                   (centroid_y - image.shape[0] // 2) ** 2) / w
                distance = distance * 16.1454 + 9.4854
                pillars.append((centroid_x, centroid_y, distance, 'red'))
                centroid_y -= 20  # Try another shift in the opposite direction

            # Final pass with original centroid
            distance = np.sqrt((centroid_x - image.shape[1] // 2) ** 2 +
                               (centroid_y - image.shape[0] // 2) ** 2) / w
            distance = distance * 16.1454 + 9.4854
            pillars.append((centroid_x, centroid_y, distance, 'red'))

        # --- GREEN PILLAR DETECTION (same logic as above) ---
        for contour in green_contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w == 0:
                continue

            centroid_x = x + w // 2
            centroid_y = y + h // 2
            distance = np.sqrt((centroid_x - image.shape[1] // 2) ** 2 +
                               (centroid_y - image.shape[0] // 2) ** 2) / w
            distance = distance * 16.1454 + 9.4854

            if (abs(h / w - 2) > 0.3 and distance < 100 and abs(centroid_x - image.shape[1] // 2) < 40) or \
               (abs(area / (w * h)) < 0.75 and abs(centroid_x - image.shape[1] // 2) < 70):
                centroid_y += 10
                distance = np.sqrt((centroid_x - image.shape[1] // 2) ** 2 +
                                   (centroid_y - image.shape[0] // 2) ** 2) / w
                distance = distance * 16.1454 + 9.4854
                pillars.append((centroid_x, centroid_y, distance, 'green'))
                centroid_y -= 20

            distance = np.sqrt((centroid_x - image.shape[1] // 2) ** 2 +
                               (centroid_y - image.shape[0] // 2) ** 2) / w
            distance = distance * 16.1454 + 9.4854
            pillars.append((centroid_x, centroid_y, distance, 'green'))

        # Sort by distance (closer pillars first)
        pillars_sorted = sorted(pillars, key=lambda x: x[2])

        return pillars_sorted

    def handle_detection_request(self, req):
        try:
            global sec
            sec += 1

            # Capture image from camera
            frame = self.picam2.capture_array()

            # Resize for processing
            frame_bgr = cv2.resize(frame, (self.processing_width, self.processing_height))

            # Crop region of interest (ROI)
            start_y = 15
            start_x = 0
            width = self.processing_width
            height = self.processing_height - start_y
            frame_bgr = frame_bgr[start_y:start_y + height, start_x:start_x + width]

            # Detect pillars in the image
            pillars_sorted = self.detect_and_sort_pillars(frame_bgr)

            # Prepare response message
            response = PillarDetectionResponse()

            for centroid_x, centroid_y, distance, color in pillars_sorted:
                pillar = Pillar()
                pillar.centroid_x = centroid_x
                pillar.centroid_y = centroid_y
                pillar.distance = distance
                pillar.color = 1 if color == 'red' else 0
                response.pillars.append(pillar)

                # (Optional) Debug image visualization
                '''
                w = 10
                x = centroid_x - w // 2
                y = centroid_y - w // 2

                box_color = (0, 0, 255) if color == 'red' else (0, 255, 0)

                cv2.rectangle(frame_bgr, (x, y), (x + w, y + w), box_color, 2)
                cv2.putText(frame_bgr, f"{distance:.2f}", (x, y + w + 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                cv2.putText(frame_bgr, color, (x, y + w + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                # cv2.imwrite(f'/home/wro25/Desktop/Output_images/image{sec}.png', frame_rgb)
                '''
            return response

        except Exception as e:
            rospy.logerr(f"Error in pillar detection: {str(e)}")
            return PillarDetectionResponse()  # Return empty response on failure

    def shutdown(self):
        # Gracefully stop camera and close OpenCV windows
        self.picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('pillar_detection_server')

    # Create and run detection server
    server = PillarDetectionServer()

    # Ensure shutdown is clean on exit
    rospy.on_shutdown(server.shutdown)

    # Keep node running
    rospy.spin()
