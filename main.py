import cv2
import numpy as np
from picamera2 import Picamera2
import time
import spidev
import RPi.GPIO as GPIO
#-------------------------------------------------------------------------------
# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0) # Open bus 0, device 0
spi.max_speed_hz = 500000 # thiết lập tần số hoạt động
GPIO.setmode(GPIO.BCM)

#Thiết lập thông số ban đầu của Pi camera
piCam = Picamera2()
piCam.preview_configuration.main.size=(640,480) #Độ phân giải 640x480
piCam.preview_configuration.main.format='RGB888'
piCam.preview_configuration.controls.FrameRate=60 #Tốc độ khung hình
piCam.preview_configuration.align()
piCam.configure('preview')
piCam.start()
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#Hàm lấy hình ảnh từ camera và lưu vào biến "frame"
def picam():
    global frame
    frame = piCam.capture_array()
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#Hàm phát hiện cạnh bằng thuật toán Canny
def canny():
    global canny_edge
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) #chuyển qua ảnh xám

    blur = cv2.GaussianBlur(gray, (11,11), 0)

    thresh_low = 150
    thresh_high = 200
    canny_edge = cv2.Canny(blur, thresh_low, thresh_high) #phát hiện cạnh
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
def warpImg():
    global imgWarp, frameFinal
    h,w,c = frame.shape
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    matrix = cv2.getPerspectiveTransform(pts1,pts2) #Tạo ma trận 3x3
    imgWarp = cv2.warpPerspective(canny_edge,matrix,(w,h)) #Ánh xạ 4 điểm trên ảnh gốc (canny_edge) ra 1 ảnh mới
    frameFinal = imgWarp

def nothing(a):
    pass
#Tạo một thanh trackbar để điều khiển vị trí 4 điểm
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
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#Tìm hai điểm nằm trên cạnh đã phát hiện trước đó
def find_left_right_points():
    global left_point, right_point
    """Find left and right points of lane
    """

    im_height, im_width = imgWarp.shape[:2] #lấy chiều cao và chiều rộng của khung hình sau khi biến đổi góc nhìn

    interested_line_y = int(im_height * 0.8) #Lấy vị trí xử lý hai điểm
    cv2.line(frame, (0, interested_line_y),
                (im_width, interested_line_y), (0, 0, 255), 2) #Vẽ đường thẳng ngang dựa trên vị trí xử lý hai điểm
    interested_line = imgWarp[interested_line_y, :] #Lấy tất cả giá trị của tọa độ x nằm trên đường thẳng xử lý hai điểm

    left_point = -1
    right_point = -1
    lane_width = 550 #Độ rộng làn đường sẽ phải chỉnh sửa nếu đường rộng hay hẹp
    center = im_width // 2 #điểm giữa của khung hình

    # Xét tọa độ di chuyển qua hai bên, nếu phát hiện pixel nào khác 0 đầu tiên thì sẽ là vị trí của làn đường trái hoặc phải
    for x in range(center, 0, -1):
        if interested_line[x] > 0:
            left_point = x
            break
    for x in range(center + 1, im_width):
        if interested_line[x] > 0:
            right_point = x
            break

    # Tính điểm bên phải khi chỉ phát hiện được điểm bên trái bằng cách lấy tọa độ x của left_point + lane_width (độ rộng làn)
    if left_point != -1 and right_point == -1:
        right_point = left_point + lane_width

    ''' Tính điểm bên trái khi chỉ phát hiện được điểm bên phải bằng cách lấy tọa độ x của right_point - lane_width (độ rộng làn)
    vì tọa độ x của right_point lớn hơn giá trị lane_width'''
    if right_point != -1 and left_point == -1:
        left_point = right_point - lane_width

    # Vẽ hai điểm của làn trái và phải
    if left_point != -1:
        cv2.circle(frame, (left_point, interested_line_y), 10, (255, 255, 0), -1)
    if right_point != -1:
        cv2.circle(frame, (right_point, interested_line_y), 10, (0, 255, 0), -1)
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#Tính toán độ lệch giữa điểm giữa khung hình và trung điểm của hai điểm trái, phải
def calculate_control_signal():
    """Calculate speed and steering angle
    """
    global center_diff, im_center, center_point
    center_diff = 0
    center_point = 0

    im_center = frame.shape[1] // 2 #Đặt tâm ảnh

    if left_point != -1 and right_point != -1 :
        # Tính toán độ lệch giữa điểm giữa xe và làn đường
        center_point = (right_point + left_point) // 2
        center_diff =  im_center - center_point

    # Vẽ đường thẳng màu xanh lá cây cho trung tâm của làn
    cv2.line(frameFinal, (center_point, 300), (center_point, 400), (0, 0, 255), 3)
    # Vẽ đường thẳng màu xanh dương cho trung tâm của khung hình
    cv2.line(frameFinal, (im_center, 300), (im_center, 400), (255, 0, 0), 3)
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.cte_previous = 0
        self.integral = 0

    def update(self, cte):
        sampling_time = 0.005
        inv_sampling_time = 1 / sampling_time

        derivative = (cte - self.cte_previous) * inv_sampling_time
        self.integral += cte * sampling_time

        steering_angle = int((
            self.kp * cte +
            self.ki * self.integral +
            self.kd * derivative
        ))

        self.cte_previous = cte
        steering = steering_angle + 90 #Vì setup góc ở giữa của servo là 90 nên cộng với 90 trước khi gửi về arduino

        return steering_angle, steering
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
def main():
    intialTracbarVals = [73, 271, 21, 340]
    initializeTrackbars(intialTracbarVals)

    #Hiệu chỉnh 3 thông số kp,ki,kd theo phương pháp ziegler–nichols
    ku = 0.145
    pu = 125

    kp = 0.60 * ku
    ki = 2 * kp / pu
    kd = kp * pu / 8
    while True:
        picam()
        valTrackbars()
        drawPoints()
        canny()
        warpImg()
        find_left_right_points()
        calculate_control_signal()

        pid_controller = PID(kp, ki, kd)
        cte = float(center_diff * 0.001)
        steering_angle, steering = pid_controller.update(cte)

        spi.xfer([steering])

        cv2.putText(frame, f"Steering Angle: {cte_1}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if (steering_angle == 0):
            cv2.putText(frame, "Tien", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
        elif (steering_angle >= 1):
            cv2.putText(frame, "Trai", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
        else:
            cv2.putText(frame, "Phai", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)


        cv2.circle(frame, (center_point, 350),10, (0, 0, 255), 3)
        cv2.line(frame, (im_center, 300), (im_center, 400), (255, 0, 0), 3)
        cv2.imshow("Original", frame)
        cv2.imshow("Perspective", imgWarp)

        if cv2.waitKey(1) & 0xFF == ord('q'):
             break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
#-------------------------------------------------------------------------------
