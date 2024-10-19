import os
import sys
import cv2 as cv
import numpy as np
import yaml
import math

class Detector:
    def __init__(self, detect_color: str, yaml_path: str):
        self.detect_color = detect_color
        with open(yaml_path, 'r') as file:
            parameters = yaml.safe_load(file)
        self.min_red = parameters['min_red']
        self.max_green = parameters['max_green']
        self.max_blue = parameters['max_blue']
        #Add more parameters here
    def detect(self, img: np.ndarray):
        #-----Image Preprocessing-----#
        #This is optional.
        #-----Color Detection-----#
        #Modify your parameters so that the mask include as much of the ring as possible
        #while excluding as much of the environment as possible.
        img1 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        if self.detect_color == "red":
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            mask = cv.inRange(img1, lower_red, upper_red) + cv.inRange(img, lower_red2, upper_red2)
        elif self.detect_color == "yellow":
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            mask = cv.inRange(img1, lower_yellow, upper_yellow)
        else:
            raise ValueError("Invalid color")
        #Optional: Add mask processing here

        #-----Mask Visualization: for Debug, Comment this before autotesting/submission!-----#
        cv.imshow("mask", mask)
        cv.waitKey(0)
        cv.destroyAllWindows()
        #-----Find countours-----#
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        centers=[]
        #-----Find Objectives-----#
        objectives = []
        for contour in contours:
            if cv.contourArea(contour)<50:
                continue
            (x,y),radius=cv.minEnclosingCircle(contour)
            center=(int(x),int(y))
            diameter=radius * 2
            areas = []
            if 21<=diameter<=26:
                area = pi * diameter ^ 2 / 4
                print(f'({center}, {area})')
        return
            #Hint: There are more than 1 ways to find circles, try them out and compare the results.
            #Your code here
        
        #-----Objectives Postprocessing-----#
        #Hint: objectives are RINGS. How to determine if overlapping circles are representing the same ring?
        #Your code here
        return objectives #each objective should be ((x,y), area). Since area is not required, if you did not include it in your code, you can return ((x,y), 0) instead.

if __name__ == '__main__':
    #File input subject to change if not using Linux
    filename = input("Enter the path of the image file: ")
    detect_color = input("Enter the color to detect (e.g., red): ")
    detector = Detector(detect_color, "parameters.yaml")
    image = cv.imread(filename)
    if image is None:
        print(f"Error loading image at {filename}. Check the file path.")
        exit()
    objectives = detector.detect(image)
    for ring in objectives:
        print(ring[0][0], ring[0][1])
    #---------Visualization---------#
    VisualizeResult= True #Set to False when autotesting/before submission!
    if not VisualizeResult:
        exit()
    for ring in objectives:
        cv.circle(image, ring[0], 5, (0, 255, 0), -1)
        cv.circle(image, ring[0], int(math.sqrt(ring[1]/3.14)), (255, 0, 0), 2)
    cv.imshow("image", image)
    cv.waitKey(0)
    cv.destroyAllWindows()


    
