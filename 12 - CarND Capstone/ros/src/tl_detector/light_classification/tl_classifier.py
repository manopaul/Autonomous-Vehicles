from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #Reference:
           # Detecting Circles in Images using OpenCV and Hough Circles
           # --- https://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-Circles
           # Detect red circls in an image using OpenCV
           # --- https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-OpenCV

        traffic_light_state = TrafficLight.UNKNOWN

        img_copy = image.copy()
        # Convert RGB image to HSV image as we can use just the hue value to detect colors
        # Red color has lower and higher hue values
        # OpenCV H has values from 0 to 180, S and V values range from 0 to 255.
        # Red color in OpenCV has hue values in range of 0 to 10 (lower_red_hue_range)
        # and 160 to 180 (higher_red_hue_range)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        LOWER_RED_HUE_MIN = np.array([0,120,120], np.uint8)
        LOWER_RED_HUE_MAX = np.array([10,255,255], np.uint8)
        HIGHER_RED_HUE_MIN = np.array([160,50,50], np.uint8)
        HIGHER_RED_HUE_MAX = np.array([179,255,255], np.uint8)

        LOWER_YELLOW_HUE_MIN = np.array([30,120,120], np.uint8)
        LOWER_YELLOW_HUE_MAX = np.array([50,255,255], np.uint8)

        LOWER_GREEN_HUE_MIN = np.array([65,120,120], np.uint8)
        LOWER_GREEN_HUE_MAX = np.array([100,255,255], np.uint8)

        lower_red_hue_image = cv2.inRange(hsv_image, LOWER_RED_HUE_MIN, LOWER_RED_HUE_MAX)
        higher_red_hue_image = cv2.inRange(hsv_image, HIGHER_RED_HUE_MIN, HIGHER_RED_HUE_MAX)
        # Create a combined red image with lower and higher red thresholds
        # having only red pixels
        combined_red_hue_image = cv2.addWeighted(lower_red_hue_image, 1.0, higher_red_hue_image, 1.0, 0.0)

        lower_yellow_hue_image = cv2.inRange(hsv_image, LOWER_YELLOW_HUE_MIN, LOWER_YELLOW_HUE_MAX)
        lower_green_hue_image = cv2.inRange(hsv_image, LOWER_GREEN_HUE_MIN, LOWER_GREEN_HUE_MAX)

        #number_of_red_pixels = cv2.countNonZero(lower_red_hue_image)
        number_of_red_pixels = cv2.countNonZero(combined_red_hue_image)
        #print("Red Pixels Number {}".format(number_of_red_pixels))

        number_of_yellow_pixels = cv2.countNonZero(lower_yellow_hue_image)
        number_of_green_pixels = cv2.countNonZero(lower_green_hue_image)

        if number_of_red_pixels > 50:
            traffic_light_state = TrafficLight.RED
        if number_of_yellow_pixels > 50:
            traffic_light_state = TrafficLight.YELLOW
        if number_of_green_pixels > 50:
            traffic_light_state = TrafficLight.GREEN

        #cv2.imshow("Traffic Light Image", combined_red_hue_image)
        #cv2.waitKey(0)

        #print("Traffic Light State {}".format(traffic_light_state))
        return traffic_light_state

        '''
        # Slightly blurring the image to avoid false positives
        blurred_red_img = cv2.GaussianBlur(combined_red_hue_image, (15,15), 0)

        #cv2.imshow("Blurred Red Image", blurred_red_img)
        #cv2.waitKey(0)

        PARAM_1 = 70
        PARAM_2 = 30
        HOUGH_CIRCLE_MIN_RADIUS = 5
        HOUGH_CIRCLE_MAX_RADIUS = 150

        # Using Hough Circles to detect traffic light Circles
        circles = cv2.HoughCircles(blurred_red_img, cv2.HOUGH_GRADIENT, 0.5, 40,
                                    HOUGH_CIRCLE_MIN_RADIUS, HOUGH_CIRCLE_MAX_RADIUS)

        #circles = cv2.HoughCircles(blurred_red_img, cv2.HOUGH_GRADIENT, 1.2, 100)

        red_light_detected = False
        print(red_light_detected)

        if circles is not None:
            # convert the (x,y) coordinates and radius of the circles to integers
            circles = np,round(circles[0,:]).astype("int")

            # loop over the (x,y) coordinates and radius of the Circles
            for (x,y,r) in circles:
                #draw the circle in the output image
                cv2.circle(img_copy, (x,y),r, (0,255,0), 4)

            red_light_detected = True
            cv2.imshow("Red Light Detected", img_copy)
            cv2.waitKey(0)

            traffic_light_state = TrafficLight.RED
            print('Red Light Detected')

        return traffic_light_state
        '''
