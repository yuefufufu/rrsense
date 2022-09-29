#! /usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# ストリーム(Color/Depth)の設定
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# ストリーミング開始
pipeline = rs.pipeline()
profile = pipeline.start(config)


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
        img_contour = cv2.drawContours(img, contours, -1, (0, 0, 0), 2)
        
        
        try:
               rows, cols = img_contour.shape[:2]
               [vx, vy, x, y] = cv2.fitLine(contours[0], cv2.DIST_L2, 0, 0.01, 0.01)
               lefty = int((-x*vy/vx)+y)
               righty = int(((cols-x)*vy/vx)+y)
               img_contour = cv2.line(img, (cols-1, righty), (0, lefty), (0, 0, 0), 2)

               ang = 320 - x[0]
        
        except:
               pass

        #RGB値
        R_min = 83
        R_max = 255
        G_min = 0
        G_max = 50
        B_min = 0
        B_max = 100

        bgrLower = np.array([B_min, G_min, R_min])    # 抽出する色の下限(BGR)
        bgrUpper = np.array([B_max, G_max, R_max])    # 抽出する色の上限(BGR)
        img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
        img_mask1 = cv2.bitwise_and(img,img, mask=img_mask)

        #輪郭作成
        contours, hierarchy = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # 小さい輪郭は誤検出として削除
        contours = list(filter(lambda x: cv2.contourArea(x) > 100, contours))
        #contours = list(map(lambda x: cv2.approxPolyDP(x, 3, True), contours))

        #輪郭線を描く
        img_contour = cv2.drawContours(img, contours, -1, (0, 0, 0), 2)
        
        area_r: float = 0

        try:
               #輪郭の面積を計算
               area_r = cv2.contourArea(contours[0])
        except:
               pass

        #RGB値
        R_min = 144
        R_max = 224
        G_min = 104
        G_max = 212
        B_min = 16
        B_max = 82

        bgrLower = np.array([B_min, G_min, R_min])    # 抽出する色の下限(BGR)
        bgrUpper = np.array([B_max, G_max, R_max])    # 抽出する色の上限(BGR)
        img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
        img_mask1 = cv2.bitwise_and(img,img, mask=img_mask)

        #輪郭作成
        contours, hierarchy = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # 小さい輪郭は誤検出として削除
        contours = list(filter(lambda x: cv2.contourArea(x) > 100, contours))
        #contours = list(map(lambda x: cv2.approxPolyDP(x, 3, True), contours))

                #輪郭線を描く
        img_contour = cv2.drawContours(img, contours, -1, (0, 0, 0), 2)
        
        area_y: float = 0

        try:
               #輪郭の面積を計算
               area_y = cv2.contourArea(contours[0])
        except:
               pass


        rospy.init_node("uragon")
        pub = rospy.Publisher("uragon", Twist, queue_size=50)
        
        uragon = Twist()
        uragon.linear.x = -0.1
        uragon.linear.y = area_r
        uragon.linear.z = area_y
        uragon.angular.x = 0.0
        uragon.angular.y = 0.0
        uragon.angular.z = ang/300
        
        pub.publish(uragon)

        print(uragon)
        


finally:
    # ストリーミング停止
    pipeline.stop()
