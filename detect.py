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
        self.min_red = np.array(parameters["min_red"])
        self.max_red = np.array(parameters["max_red"])
        self.min_yellow = np.array(parameters["min_yellow"])
        self.max_yellow = np.array(parameters["max_yellow"])

    def detect(self, img: np.ndarray):
        #-----Image Preprocessing-----#
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

        #-----Color Detection-----#
        #Modify your parameters so that the mask include as much of the ring as possible
        #while excluding as much of the environment as possible.
        if self.detect_color == "red":
            mask = cv.inRange(rgb, self.min_red, self.max_red)
        elif self.detect_color == "yellow":
            mask = cv.inRange(hsv, self.min_yellow, self.max_yellow)
        else:
            raise ValueError("Invalid color")
        #Optional: Add mask processing here

        #-----Mask Visualization: for Debug, Comment this before autotesting/submission!-----#
        # cv.imshow("mask", mask)
        # cv.waitKey(0)
        # cv.destroyAllWindows()
        #-----Find countours-----#
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #-----Find Objectives-----#
        objectives = []
        for contour in contours:
            #Hint: There are more than 1 ways to find circles, try them out and compare the results.
            area = cv.contourArea(contour)
            if area > 100:
                centroid = int(cv.moments(contour)["m10"] / cv.moments(contour)["m00"]), int(cv.moments(contour)["m01"] / cv.moments(contour)["m00"])
                objectives.append((centroid, area))

        #-----Objectives Postprocessing-----#
        #Hint: objectives are RINGS. How to determine if overlapping circles are representing the same ring?
        # remove overlapping circles
        for i in range(len(objectives)):
            for j in range(i+1, len(objectives)):
                if math.sqrt((objectives[i][0][0]-objectives[j][0][0])**2 + (objectives[i][0][1]-objectives[j][0][1])**2) < 20:
                    if objectives[i][1] > objectives[j][1]:
                        objectives.pop(j)
                    else:
                        objectives.pop(i)
                    break


        return objectives #each objective should be ((x,y), area). Since area is not required, if you did not include it in your code, you can return ((x,y), 0) instead.

if __name__ == '__main__':
    #File input subject to change if not using Linux
    args = sys.argv
    if len(args) > 1:
        filename = args[1]
    else:
        filename = "images/Image1.png" 
    if len(args) > 2:
        detect_color = args[2]
    else:
        detect_color = "red"
    detector = Detector(detect_color, "parameters.yaml")
    image = cv.imread(filename)
    objectives = detector.detect(image)
    for ring in objectives:
        print(ring[0][0], ring[0][1], ring[1])
    #---------Visualization---------#
    VisualizeResult= False #Set to False when autotesting/before submission!
    if not VisualizeResult:
        exit()
    for ring in objectives:
        cv.circle(image, ring[0], 5, (0, 255, 0), -1)
        cv.circle(image, ring[0], int(math.sqrt(ring[1]/3.14)), (255, 0, 0), 2)
    cv.imshow("image", image)
    cv.waitKey(0)
    cv.destroyAllWindows()
    