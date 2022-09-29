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
        img_contour = cv2.drawContours(img, contours, -1, (120, 255, 0), 2)
        
        area: float = 0

        try:
               #輪郭の面積を計算
               area = cv2.contourArea(contours[0])
        except:
               pass
               
        #print(area)
        
        rospy.init_node("red_ura")
        pub = rospy.Publisher("red", Float64, queue_size=50)
        pub.publish(area)

        # フレーム待ち
        frames = pipeline.wait_for_frames()

        #RGB
        RGB_frame = frames.get_color_frame()
        RGB_image = np.asanyarray(RGB_frame.get_data())

        img = RGB_image

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
        img_contour = cv2.drawContours(img, contours, -1, (120, 255, 0), 2)
        
        areaa: float = 0

        try:
               #輪郭の面積を計算
               areaa = cv2.contourArea(contours[0])
        except:
               pass
               
        print(area,areaa)
        pub = rospy.Publisher("yellow", Float64, queue_size=50)
        pub.publish(areaa)






finally:
    # ストリーミング停止
    pipeline.stop()
