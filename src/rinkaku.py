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

    # トラックバーの生成
cv2.createTrackbar("R_min", "OpenCV Window", 0, 255, nothing)
cv2.createTrackbar("R_max", "OpenCV Window", 120, 255, nothing)
cv2.createTrackbar("G_min", "OpenCV Window", 0, 255, nothing)
cv2.createTrackbar("G_max", "OpenCV Window", 255, 255, nothing)
cv2.createTrackbar("B_min", "OpenCV Window", 0, 255, nothing)
cv2.createTrackbar("B_max", "OpenCV Window", 255, 255, nothing)

try:
    while True:
        # フレーム待ち
        frames = pipeline.wait_for_frames()

        #RGB
        RGB_frame = frames.get_color_frame()
        RGB_image = np.asanyarray(RGB_frame.get_data())

        img = RGB_image

        #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #トラックバーの設定
        R_min = cv2.getTrackbarPos("R_min", "OpenCV Window")
        R_max = cv2.getTrackbarPos("R_max", "OpenCV Window")
        G_min = cv2.getTrackbarPos("G_min", "OpenCV Window")
        G_max = cv2.getTrackbarPos("G_max", "OpenCV Window")
        B_min = cv2.getTrackbarPos("B_min", "OpenCV Window")
        B_max = cv2.getTrackbarPos("B_max", "OpenCV Window")

        bgrLower = np.array([B_min, G_min, R_min])    # 抽出する色の下限(BGR)
        bgrUpper = np.array([B_max, G_max, R_max])    # 抽出する色の上限(BGR)
        img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
        img_mask1 = cv2.bitwise_and(img,img, mask=img_mask)

        #輪郭作成
        contours, hierarchy = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # 小さい輪郭は誤検出として削除
        #contours = list(filter(lambda x: cv2.contourArea(x) > 100, contours))
        #contours = list(map(lambda x: cv2.approxPolyDP(x, 3, True), contours))

        #輪郭線を描く
        img_contour = cv2.drawContours(img, contours, -1, (200, 255, 0), 2)

        area: float = 0

        try:
               #輪郭の面積を計算
               area = cv2.contourArea(contours[0])
        except:
               pass
               
        print(area)

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