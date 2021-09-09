# -*- coding: utf-8 -*-
import time
import ctypes
import cv2 as cv
import mss
import numpy as np
import pyautogui
import math
from time import sleep
from pynput import keyboard
#from simple_pid import PID

import win32api
import win32gui

MONITOR_DEFAULTTONEAREST = 0x2

ACTION_ZONE_CROP = (1, 50+1, 1, 1) # left, top, right, bot
WINDOW_SET_SIZE = (729, 607)
HELI_ASPECT_RATIO = 2.275
WALL_SIZE = (42, 101)

#with mss.mss() as sct:
#    sct.grab()

dwmapi = ctypes.WinDLL("dwmapi")

RECT = ctypes.c_int  * 4

def detectHelicopter(screenie, sliceX, sliceWidth):
    # cv.rectangle(img,
    #              (sliceX, 20),
    #              (sliceX + sliceWidth, img.shape[0] - 20),
    #              (255 ,255, 255))
    bChannel = screenie[:,:,0]
    helicopterSlice = bChannel[0:bChannel.shape[0], sliceX:sliceX+sliceWidth]
    ret, thresh = cv.threshold(helicopterSlice, 102, 255, cv.THRESH_BINARY)
    
    kernel = np.ones((5,5), np.uint8)
    dilated = cv.dilate(thresh, kernel, iterations=1)
    
    contours, hierarchy = cv.findContours(dilated,
                                          cv.RETR_EXTERNAL,
                                          cv.CHAIN_APPROX_SIMPLE)
    return contours
    
def detectWalls(screenie):
    gChannel = screenie[:,:,1]
    
    blurred = cv.medianBlur(gChannel, 3)
    
    ret, thresh = cv.threshold(blurred, 250, 255, cv.THRESH_BINARY)
    
    dilated = cv.dilate(thresh,
                         np.ones((5,5), np.uint8),
                         iterations=1)
    
    contours, hierarchy = cv.findContours(dilated,
                                          cv.RETR_EXTERNAL,
                                          cv.CHAIN_APPROX_SIMPLE)
    
    return contours, dilated

def cursorOutsideWindow(hwnd):
    left, top, right, bottom = win32gui.GetWindowRect(hwnd)
    # crop away drop shadow from Win10 UI
    left += 7 # left drop shadow
    right -= 7 # right drop shadow
    bottom -= 7 # bottom drop shadow
    
    # crop to interactive game area within window
    left += ACTION_ZONE_CROP[0]
    top += ACTION_ZONE_CROP[1]
    right -= ACTION_ZONE_CROP[2]
    bottom -= ACTION_ZONE_CROP[3]
    (x, y) = win32gui.GetCursorPos()
    return x <= left or x >= right or y <= top or y >= bottom

def main():
    hwnd = findAdobeFlashProjectorWindow()
    
    left, top, right, bottom = win32gui.GetWindowRect(hwnd)
    
    HWND_TOPMOST = -1
    SWP_NOMOVE = 0x2 # Retains the current position (ignores X and Y parameters).
    
    width, height = WINDOW_SET_SIZE
    print('window set size', WINDOW_SET_SIZE)
    win32gui.SetWindowPos(hwnd,
                          HWND_TOPMOST,
                          0,
                          0,
                          width,
                          height,
                          SWP_NOMOVE)
    
    isMinimizedNoticePrinted = False
    
    pyautogui.FAILSAFE = False
    pyautogui.PAUSE = 0
    last_input = time.perf_counter() * 1000
    
    with mss.mss() as sct:
        try:
            while "Screen capturing":
                #last_time = time.time_ns()
                
                if not isWindowNormalOrMaximized(hwnd):
                    if isMinimizedNoticePrinted:
                        # still minimized
                        cv.waitKey(16)
                        continue
                    else:
                        # first time?
                        print("Window is minimized, please unminimize window.")
                        isMinimizedNoticePrinted = True
                    pass
                elif isMinimizedNoticePrinted:
                    # woo! unminimized
                    isMinimizedNoticePrinted = False
                
                coords = getWindowExtendedRect(hwnd)
              
                screenie = np.array(sct.grab((coords[0], coords[1], coords[2], coords[3])))
                screenie = cv.cvtColor(screenie, cv.COLOR_BGRA2BGR)
                # crop 1px Windows border and 50px top bar
                h,w,d = screenie.shape
                screenie = screenie[ACTION_ZONE_CROP[1]:h-ACTION_ZONE_CROP[3], ACTION_ZONE_CROP[0]:w-ACTION_ZONE_CROP[2]]
                
                cv.rectangle(screenie,
                             (0, 55),
                             (711, 547-55),
                             (0, 0, 255),
                             2)
                
                cv.rectangle(screenie,
                             (190, 50),
                             (190+95, screenie.shape[0]-50),
                             (0, 255, 255))
                sliceX = 190
                heliContours = detectHelicopter(screenie, sliceX, 95)
                
                heliRect = None
                for cnt in heliContours:
                    rect = cv.boundingRect(cnt)
                    x,y,w,h = rect
                    if abs(w/h - HELI_ASPECT_RATIO) > 0.5:
                        continue
                    heliRect = (x+sliceX, y, w, h)
                    cv.rectangle(screenie, (x+sliceX, y), (x+sliceX+w, y+h), (0, 0, 255), 3)
                    break
                    
                    
                flyTargets = []
                wallContours, wallMask = detectWalls(screenie)
                for cnt in wallContours:
                    x,y,w,h = cv.boundingRect(cnt)
                    if abs(w-WALL_SIZE[0]) > 25 or abs(h-WALL_SIZE[1]) > 25:
                        continue
                    cv.rectangle(screenie, (x, y), (x+w, y+h), (64, 128, 255), 4)
                    
                    topDist = -1
                    a = False
                    b = False
                    for i in range(y-1, 0, -1):
                        if a and b:
                            break
                        if wallMask[i,x] == 255:
                            topDist = max(topDist, i)
                            a = True
                        if wallMask[i,x+w-1] == 255:
                            topDist = max(topDist, i)
                            b = True
                            
                    botDist = 999999
                    a = False
                    b = False

                    botStartRange = min(y+h+1, screenie.shape[0] - 1)
                    for i in range(botStartRange, screenie.shape[0] - 1):
                        if a and b:
                            break
                        if wallMask[i,x] == 255:
                            botDist = min(botDist, i)
                            a = True
                        if wallMask[i,x+w-1] == 255:
                            botDist = min(botDist, i)
                            b = True
                    
                    if topDist != -1 and botDist != 999999:
                        cv.circle(screenie,
                                  (round(x+w/2), topDist),
                                  10,
                                  (0, 0, 255),
                                  2)
                        cv.circle(screenie,
                                  (round(x+w/2), botDist),
                                  10,
                                  (0, 0, 255),
                                  2)
                        
                        topBlocked = False                    
                        if heliRect and y - topDist < heliRect[3]:
                            topBlocked = True
                            cv.line(screenie,
                                    (x, topDist),
                                    (x+w, y),
                                    (0, 0, 255),
                                    3)
                            cv.line(screenie,
                                    (x+w, topDist),
                                    (x, y),
                                    (0, 0, 255),
                                    3)
                        botBlocked = False
                        if heliRect and botDist - (y+h) < heliRect[3]:
                            botBlocked = True
                            cv.line(screenie,
                                    (x, y+h),
                                    (x+w, botDist),
                                    (0, 0, 255),
                                    3)
                            cv.line(screenie,
                                    (x+w, y+h),
                                    (x, botDist),
                                    (0, 0, 255),
                                    3)
                        if heliRect:
                            if topBlocked and botBlocked:
                                # nothing
                                pass
                            elif not topBlocked and botBlocked:
                                # top target
                                flyTargets.append((x, round((y+topDist)/2)))
                                flyTargets.append((x+w, round((y+topDist)/2)))
                            elif topBlocked and not botBlocked:
                                # bot target
                                flyTargets.append((x, round(((y+h)+botDist)/2)))
                                flyTargets.append((x+w, round(((y+h)+botDist)/2)))
                            else:
                                # target whichever has more space?
                                topSpace = y - topDist
                                botSpace = botDist - (y+h)
                                if topSpace > botSpace:
                                    # top target
                                    flyTargets.append((x, round((y+topDist)/2)))
                                    flyTargets.append((x+w, round((y+topDist)/2)))
                                else:
                                    # bot target
                                    flyTargets.append((x, round(((y+h)+botDist)/2)))
                                    flyTargets.append((x+w, round(((y+h)+botDist)/2)))
                    
                if heliRect:                   
                    flyTargets = sorted(flyTargets,
                           key=lambda x: x[0])
                    flyTargets = list(filter(lambda x:x[0] >= heliRect[0], flyTargets))
                    
                    if len(flyTargets) < 1:
                        flyTargets.append((round(screenie.shape[1] / 2),
                                           round(screenie.shape[0] / 2)))
                    
                    # for (x, y) in flyTargets:
                    #     cv.circle(screenie,
                    #               (x,y),
                    #               15,
                    #               (0,0,255),
                    #               5)
                    #x,y,w,h = cv.boundingRect(cnt)
                    closestX, closestY = flyTargets[0]
                    cv.circle(screenie, flyTargets[0], 20, (0, 0, 255), 5)
                    now = time.perf_counter() * 1000
                    
                    heliY = heliRect[1]
                    heliHeight = heliRect[3]

                    cv.line(screenie,
                            (100, round((heliY + heliHeight / 2))),
                            (150, round((heliY + heliHeight / 2))),
                            (0, 0, 255))
                    
                    if not cursorOutsideWindow(hwnd):
                        heliCenterY = round((heliY + heliHeight / 2))
                        if heliCenterY > closestY:
                            if now - last_input > 16:
                                pyautogui.mouseDown()
                                last_input = now
                        else:
                            if now - last_input > 16:
                                pyautogui.mouseUp()
                                last_input = now
                
                out = screenie

                # blue = screenie[:,:,0]
                # blue = cv.medianBlur(blue, 3)
                # ret, blue = cv.threshold(blue, 200, 255, cv.THRESH_BINARY);
                # blue = cv.dilate(blue, np.ones((10, 10), np.uint8))
                
                # blueSample = blue[235:270, 325:575]
                
                cv.imshow("OpenCV/Numpy normal", screenie)
                
                key = cv.waitKey(16) & 0xFF
                if (key == ord("q")):
                    cv.destroyAllWindows()
                    break
                if (key == ord(" ")):
                    pyautogui.click()
                    continue
        finally:
            cv.destroyAllWindows()
            
    #rint("ns: {}".format(time.time_ns() - last_time))

def roundup(x):
    return int(math.ceil(x / 10.0)) * 10

def isWindowNormalOrMaximized(hwnd):
    placementRes = win32gui.GetWindowPlacement(hwnd)
    return placementRes[1] == 1 or placementRes[1] == 3

def getWindowExtendedRect(hwnd):
    rect = RECT(0, 0, 0, 0)
    dwmapi.DwmGetWindowAttribute(hwnd, 9, ctypes.byref(rect), ctypes.sizeof(rect))
    return rect[0:4]

def findAdobeFlashProjectorWindow():
    title = "Adobe Flash Player 32"
    projectorWindows = findTopWindows(title)
    if len(projectorWindows) < 1:
        raise RuntimeError(f"Didn't find any windows with title '{title}'!")
    if len(projectorWindows) > 1:
        raise RuntimeError("Found multiple windows with title '{title}'!")
    return projectorWindows[0]

def findTopWindows(wantedText=None):
    results = []
    topWindows = []
    
    win32gui.EnumWindows(_windowEnumerationHandler, topWindows)
    
    for hwnd, windowText, windowClass in topWindows:
        if wantedText and wantedText in windowText:
            results.append(hwnd)
            
    return results

def _windowEnumerationHandler(hwnd, resultList):
    '''Pass to win32gui.EnumWindows() to generate list of window handle,
    window text, window class tuples.'''
    resultList.append((hwnd,
                       win32gui.GetWindowText(hwnd),
                       win32gui.GetClassName(hwnd)))
    

if __name__ == "__main__":
    main()