#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class CVPublisher:
    def __init__(self):
        rospy.init_node('cv_publisher_node', anonymous=True)
        self.bridge = CvBridge()
        self.pub_img = rospy.Publisher('/cv/image_annotated', Image, queue_size=1)
        self.pub_det = rospy.Publisher('/cv/detections', String, queue_size=1)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        if not self.cap.isOpened():
            rospy.logerr("Kamera gagal dibuka!")
            exit()

        self.mode = "all"
        rospy.loginfo("Node Publisher aktif. Tekan 'q' untuk keluar, 's' untuk simpan frame.")
        self.run()

    def detect_shapes(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        count = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 400:
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                shape = "Unknown"
                if len(approx) == 3:
                    shape = "Triangle"
                elif len(approx) == 4:
                    shape = "Rectangle"
                elif len(approx) > 6:
                    shape = "Circle"

                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                cv2.putText(frame, shape, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                count += 1
        return count

    def detect_colors(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([15, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        cnt_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        cnt_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnt_orange:
            if cv2.contourArea(c) > 300:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 140, 255), 2)
                cv2.putText(frame, "Orange", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)

        for c in cnt_white:
            if cv2.contourArea(c) > 300:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
                cv2.putText(frame, "White", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return len(cnt_orange) + len(cnt_white)

    def run(self):
        prev_time = rospy.get_time()
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)
            shapes = self.detect_shapes(frame)
            colors = self.detect_colors(frame)

            new_time = rospy.get_time()
            fps = 1.0 / (new_time - prev_time + 1e-5)
            prev_time = new_time
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_img.publish(img_msg)

            data = {"shapes_detected": shapes, "colors_tracked": colors, "fps": round(fps, 2)}
            self.pub_det.publish(json.dumps(data))

            cv2.imshow("CV Publisher", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                cv2.imwrite("/tmp/frame_capture.jpg", frame)
                rospy.loginfo("Frame disimpan ke /tmp/frame_capture.jpg")
            elif key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    CVPublisher()
