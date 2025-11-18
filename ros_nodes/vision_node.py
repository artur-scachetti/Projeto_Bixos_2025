import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point


class VisionNode():

    def __init__(self):

        rospy.init_node('vision_node', anonymous=True)

        self.pub_min_angle = rospy.Publisher('/vision/min_angle', Float32, queue_size=10)
        self.pub_intersection = rospy.Publisher('/vision/intersection', Point, queue_size=10)

        self.rate = rospy.Rate(10)

    def open_webcam(self, device):

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if not cap.isOpened():
            rospy.logwarn("Não foi possível abrir a webcam")
            exit()

        return cap
    
    def color_mask(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 200, 200])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 250])
        upper_white = np.array([179, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        r_channel = frame[:, :, 2]
        _, r_mask = cv2.threshold(r_channel, 200, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5,5), np.uint8)

        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        mask = cv2.bitwise_or(yellow_mask, white_mask)
        mask = cv2.bitwise_or(mask, r_mask)

        masked = cv2.bitwise_and(frame, frame, mask=mask)

        return masked

    def sobel_edges(self, frame, ksize=3, thresh=(50, 255)):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=ksize)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=ksize)
        mag = np.sqrt(sobelx**2 + sobely**2)
        mag = np.uint8(255 * mag / np.max(mag))

        _, binary = cv2.threshold(mag, thresh[0], thresh[1], cv2.THRESH_BINARY)
        return binary

    def region_of_interest(self, img):

        h, w = img.shape[:2]
        
        vertices = np.array([[
            (int(0.1*w), h),
            (int(0.1*w), int(0.3*h)),
            (int(0.9*w), int(0.3*h)),
            (int(0.9*w), h)
        ]], dtype=np.int32)

        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        masked = cv2.bitwise_and(img, mask)
        return masked


    def line_intersection(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        determ = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
        if determ == 0:
            return None
        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / determ
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / determ
        return int(px), int(py)


    def run(self):

        cap = self.open_webcam(2)

        while not rospy.is_shutdown():

            ret, frame = cap.read()

            if not ret:
                    
                rospy.logwarn("Não foi possível capturar frame")
                break

            frame, area_left, area_right, closest_inter, min_angle, h = image_processing(frame)

            pub_area_l.publish(area_left)
            pub_area_r.publish(area_right)
            pub_min_angle.publish(min_angle)
            
            intersec_msg = Point()
            if closest_inter is not None:
                intersec_msg.x = float(closest_inter[0])
                intersec_msg.y = float(closest_inter[1])
                intersec_msg.z = 0.0
            
            else:
                intersec_msg.x = intersec_msg.y = intersec_msg.z = 0.0
            
            pub_intersection.publish(intersec_msg)


            cv2.imshow('Linhas', frame)

            if cv2.waitKey(1) == ord('q'):
                break

            rate.sleep()

        cap.release()
        cv2.destroyAllWindows()
    

if __name__ == '__main__':

    try:
        v = VisionNode()
        v.run()
    
    except rospy.ROSInterruptException:
        pass