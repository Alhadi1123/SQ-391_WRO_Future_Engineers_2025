
import cv2
import numpy as np

def trackbar_callback(x):
    # Dummy callback function for trackbars
    pass

# Initialize webcam
webcam = cv2.VideoCapture(0)
cv2.namedWindow("Color Trackbars")

# Create trackbars for color range selection
cv2.createTrackbar("Lower Hue", "Color Trackbars", 0, 179, trackbar_callback)
cv2.createTrackbar("Lower Saturation", "Color Trackbars", 0, 255, trackbar_callback)
cv2.createTrackbar("Lower Value", "Color Trackbars", 0, 255, trackbar_callback)
cv2.createTrackbar("Upper Hue", "Color Trackbars", 179, 179, trackbar_callback)
cv2.createTrackbar("Upper Saturation", "Color Trackbars", 255, 255, trackbar_callback)
cv2.createTrackbar("Upper Value", "Color Trackbars", 255, 255, trackbar_callback)

while True:
    # Capture frame from webcam
    ret, frame = webcam.read()
    if not ret:
        break
    
    # Convert frame from BGR to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Get current positions of the trackbars
    lower_hue = cv2.getTrackbarPos("Lower Hue", "Color Trackbars")
    lower_saturation = cv2.getTrackbarPos("Lower Saturation", "Color Trackbars")
    lower_value = cv2.getTrackbarPos("Lower Value", "Color Trackbars")
    upper_hue = cv2.getTrackbarPos("Upper Hue", "Color Trackbars")
    upper_saturation = cv2.getTrackbarPos("Upper Saturation", "Color Trackbars")
    upper_value = cv2.getTrackbarPos("Upper Value", "Color Trackbars")
    
    # Define the lower and upper bounds for the color mask
    lower_bound = np.array([lower_hue, lower_saturation, lower_value])
    upper_bound = np.array([upper_hue, upper_saturation, upper_value])
    
    # Create mask to isolate the color within the bounds
    color_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    
    # Apply mask to the original frame
    masked_frame = cv2.bitwise_and(frame, frame, mask=color_mask)
    
    # Display the original frame, mask, and the result
    cv2.imshow("Webcam Frame", frame)
    cv2.imshow("Color Mask", color_mask)
    cv2.imshow("Masked Frame", masked_frame)
    
    # Break the loop when 'ESC' key is pressed
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release the webcam and destroy all OpenCV windows
webcam.release()
cv2.destroyAllWindows()
