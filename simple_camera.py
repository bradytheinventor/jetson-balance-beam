# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2
import numpy as np

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


minDist = 10
param1 = 100 # canny upper threshold. lower threshold is param1/2
param2 = 10 # hough accumulator something. smaller = more forgiving
minRadius = 10
maxRadius = 20

# TODO: camera supports 1280x720 @ 120fps - build cv2 with CUDA
skip_frames = True
div_ratio = 2

def show_camera():
    window_title = "CSI Camera"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2,framerate=60,capture_width=640,capture_height=360,display_width=640,display_height=360), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

            history = []

            drop_frame_counter = 0
            while True:
                ret_val, frame = video_capture.read()

                #process every 2nd frame: 60fps -> 30fps
                if(skip_frames):
                    if(drop_frame_counter < div_ratio-1):
                        drop_frame_counter = drop_frame_counter + 1
                        continue
                    drop_frame_counter = 0

                # actually BGR, but we want to swap blue/red so the red hue is away from 0/360 transition
                # makes masking easier
                hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

                # boost saturation. kind of boosts sensor noise too. not great
                """
                hsv = np.float32(hsv)
                hsv[:,:,1] = np.clip(hsv[:,:,1] * 1, 0, 255)
                hsv = np.uint8(hsv)
                """

                # hsv filter out anything that's not red
                # TODO: robust against lighting changes?
                lower = np.array([110,150,0])
                upper = np.array([130,255,255])
                mask = cv2.inRange(hsv, lower, upper)
                hsv_masked = cv2.bitwise_and(hsv, hsv, mask=mask)
                masked = cv2.cvtColor(hsv_masked, cv2.COLOR_HSV2RGB)

                gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
                #gray = cv2.equalizeHist(gray)

                blur = cv2.medianBlur(gray, 5)
                #blur = cv2.bilateralFilter(gray, 5, 100, 100)

                #canny = cv2.Canny(blur, param1, param1/2)
                circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1.3, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

                #display_img = cv2.cvtColor(canny, cv2.COLOR_GRAY2RGB)
                #display_img = masked
                display_img = frame

                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    for i,c in enumerate(circles[0,:]):
                        if i == 0:
                            history.append((c[0], c[1]))
                            if len(history) > 20:
                                history.pop(0)

                        color = (0,255,0) if i == 0 else (255,0,0)
                        cv2.circle(display_img, (c[0], c[1]), c[2], color, 2)

                for p in history:
                    cv2.circle(display_img, p, 0, (0,0,255), 2)

                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, display_img)
                else:
                    break 
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()
