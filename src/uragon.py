#! /usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist

# ストリーム(Color/Depth)の設定
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# ストリーミング開始
pipeline = rs.pipeline()
profile = pipeline.start(config)

cv2.namedWindow("OpenCV Window")

    # トラックバーのコールバック関数は何もしない空の関数
def nothing(x):
    pass


try:
    while True:
        # フレーム待ち
        frames = pipeline.wait_for_frames()

        #RGB
        RGB_frame = frames.get_color_frame()
        RGB_image = np.asanyarray(RGB_frame.get_data())

        img = RGB_image
        #グレー化
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        
        # 閾値の設定
        threshold = 70
    
        # 二値化(閾値を超えた画素を255にする。)
        ret, img_thresh = cv2.threshold(img_gray, threshold, 255, cv2.THRESH_BINARY)

        # 色範囲によるマスク生成
        img_mask = cv2.inRange(img_thresh, 0, 10)
        
        #輪郭作成
        contours, hierarchy = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # 小さい輪郭は誤検出として削除
        contours = list(filter(lambda x: cv2.contourArea(x) > 100, contours))

        #輪郭線を描く
        img_contour = cv2.drawContours(img, contours, -1, (120, 255, 0), 2)
        
        
        try:
               rows, cols = img_contour.shape[:2]
               [vx, vy, x, y] = cv2.fitLine(contours[0], cv2.DIST_L2, 0, 0.01, 0.01)
               lefty = int((-x*vy/vx)+y)
               righty = int(((cols-x)*vy/vx)+y)
               img_contour = cv2.line(img, (cols-1, righty), (0, lefty), (0, 0, 255), 2)

               uragon = 320 - x[0]
        
        except:
               pass
        
        rospy.init_node("uragon")
        pub = rospy.Publisher("uragon", Twist, queue_size=50)
        
        uraran = Twist()
        uraran.linear.x = -0.1
        uraran.linear.y = 0.0
        uraran.linear.z = 0.0
        uraran.angular.x = 0.0
        uraran.angular.y = 0.0
        uraran.angular.z = uragon/300
        
        pub.publish(uraran)
        
        print(uragon)


        # 表示
        images = np.hstack((img_contour))
        cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color_image', img_contour)
        if cv2.waitKey(1) & 0xff == 27:#ESCで終了
            cv2.destroyAllWindows()
            break

        


finally:
    # ストリーミング停止
    pipeline.stop()
