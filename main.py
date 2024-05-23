import cv2
import numpy as np
from picamera2 import Picamera2
import time
import spidev
import RPi.GPIO as GPIO

spi = spidev.SpiDev()
spi.open(0, 0) # Open bus 0, device 0
spi.max_speed_hz = 500000 # Set SPI speed
GPIO.setmode(GPIO.BCM)

def picam():
    global frame
    piCam = Picamera2()
    piCam.preview_configuration.main.size=(640,480)
    piCam.preview_configuration.main.format='RGB888'
    piCam.preview_configuration.controls.FrameRate=60
    piCam.preview_configuration.align()
    piCam.configure('preview')
    piCam.start()
    frame = piCam.capture_array()

def canny():
    global canny_edge
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    equalized_gray = cv2.equalizeHist(gray)
    blur = cv2.GaussianBlur(equalized_gray, (11,11), 0)

    thresh_low = 150
    thresh_high = 200
    canny_edge = cv2.Canny(blur, thresh_low, thresh_high)

def warpImg():
    global imgWarp, frameFinal
    h,w,c = frame.shape
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(canny_edge,matrix,(w,h))
    frameFinal = imgWarp

def nothing(a):
    pass

def initializeTrackbars(intialTracbarVals,wT=640, hT=480):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0],wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2],wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)

def valTrackbars(wT=640, hT=480):
    global points
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])

def drawPoints():
    for x in range(4):
        cv2.circle(frame, (int(points[x][0]), int(points[x][1])), 15, (0,0,255), cv2.FILLED)

def find_left_right_points():
    global left_point, right_point
    """Find left and right points of lane
    """

    im_height, im_width = imgWarp.shape[:2]

    # Consider the position 70% from the top of the image
    interested_line_y = int(im_height * 0.9)
    cv2.line(frame, (0, interested_line_y),
                (im_width, interested_line_y), (0, 0, 255), 2)
    interested_line = imgWarp[interested_line_y, :]

    # Detect left/right points
    left_point = -1
    right_point = -1
    lane_width = 496
    center = im_width // 2

    # Traverse the two sides, find the first non-zero value pixels, and
    # consider them as the position of the left and right lines
    for x in range(center, 0, -1):
        if interested_line[x] > 0:
            left_point = x
            break
    for x in range(center + 1, im_width):
        if interested_line[x] > 0:
            right_point = x
            break

    # Predict right point when only see the left point
    if left_point != -1 and right_point == -1:
        right_point = left_point + lane_width

    # Predict left point when only see the right point
    if right_point != -1 and left_point == -1:
        left_point = right_point - lane_width

    # Draw two points on the image
    if left_point != -1:
        cv2.circle(frame, (left_point, interested_line_y), 10, (255, 255, 0), -1)
    if right_point != -1:
        cv2.circle(frame, (right_point, interested_line_y), 10, (0, 255, 0), -1)

def calculate_control_signal():
    """Calculate speed and steering angle
    """
    global center_diff, im_center, center_point
    center_diff = 0
    center_point = 0
    #draw_img = draw if draw is not None else img.copy()
    # Find left/right points

    # Calculate speed and steering angle
    # The speed is fixed to 50% of the max speed
    # You can try to calculate speed from turning angle
    im_center = frame.shape[1] // 2 #Đặt tâm ảnh

    if left_point != -1 and right_point != -1 :

        # Tính toán độ lệch giữa điểm giữa xe và làn đường
        center_point = (right_point + left_point) // 2
        center_diff =  im_center - center_point
    cv2.putText(frame, f"Result: {center_diff}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    # Vẽ đường thẳng màu xanh lá cây cho trung tâm của làn
    cv2.line(frameFinal, (center_point, 250), (center_point, 350), (0, 0, 255), 3)
    # Vẽ đường thẳng màu xanh dương cho trung tâm của khung hình
    cv2.line(frameFinal, (im_center, 250), (im_center, 350), (255, 0, 0), 3)

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Gain for proportional control
        self.ki = ki  # Gain for integral control
        self.kd = kd  # Gain for derivative control

        self.cte_previous = 0  # Previous cross track error
        self.integral = 0      # Integral of CTE

    def update(self, cte):
        # Calculate the derivative of the CTE
        derivative = cte - self.cte_previous

        # Update integral of CTE
        self.integral += cte

        # Calculate steering angle
        steering_angle = int(-(
            self.kp * cte +                  # Proportional term
            self.ki * self.integral +        # Integral term
            self.kd * derivative             # Derivative term
        ))

        # Update previous CTE and time
        self.cte_previous = cte

        return steering_angle

def main():
    intialTracbarVals = [55, 206, 19, 324]
    initializeTrackbars(intialTracbarVals)
    while True:
        picam()
        valTrackbars()
        drawPoints()
        canny()
        warpImg()
        find_left_right_points()
        calculate_control_signal()

        # Create a PID controller object
        pid_controller = PID(kp, ki, kd)
        # Example usage of the PID controller
        cte = float(center_diff * 0.
        1)
        steering_angle = pid_controller.update(cte)
        print(f"Steering Angle: {steering_angle}")
        cv2.putText(frame, f"Steering Angle: {steering_angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.circle(frame, (center_point, 250),10, (0, 0, 255), 3)
        cv2.line(frame, (im_center, 180), (im_center, 320), (255, 0, 0), 3)
        cv2.imshow("Original", frame)
        cv2.imshow("Perspective", imgWarp)
        cv2.imshow("Final", frameFinal)

        if cv2.waitKey(1) & 0xFF == ord('q'):
             break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
