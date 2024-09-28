#import os
import sys
import cv2 as cv
import numpy as np
import yaml
#import math

class Detector:
    def __init__(self, detect_color: str, yaml_path: str):
        self.detect_color = detect_color
        with open(yaml_path, 'r') as file:
            parameters = yaml.safe_load(file)
        self.min_red_for_red1 = parameters['min_red_for_red1']
        self.min_green_for_red1 = parameters['min_green_for_red1']
        self.min_blue_for_red1 = parameters['min_blue_for_red1']

        self.max_red_for_red1 = parameters['max_red_for_red1']
        self.max_green_for_red1 = parameters['max_green_for_red1']
        self.max_blue_for_red1 = parameters['max_blue_for_red1']

        self.min_red_for_red2 = parameters['min_red_for_red2']
        self.min_green_for_red2 = parameters['min_green_for_red2']
        self.min_blue_for_red2 = parameters['min_blue_for_red2']

        self.max_red_for_red2 = parameters['max_red_for_red2']
        self.max_green_for_red2 = parameters['max_green_for_red2']
        self.max_blue_for_red2 = parameters['max_blue_for_red2']

        self.min_red_for_yellow = parameters['min_red_for_yellow']
        self.min_green_for_yellow = parameters['min_green_for_yellow']
        self.min_blue_for_yellow = parameters['min_blue_for_yellow']

        self.max_red_for_yellow = parameters['max_red_for_yellow']
        self.max_green_for_yellow = parameters['max_green_for_yellow']
        self.max_blue_for_yellow = parameters['max_blue_for_yellow']
        #Add more parameters here
    def detect(self, img: np.ndarray):
        #-----Image Preprocessing-----#
        #img = img_original
        img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        #This is optional.
        #-----Color Detection-----#
        #Modify your parameters so that the mask include as much of the ring as possible
        #while excluding as much of the environment as possible.
        if self.detect_color == "red":
            upper_red1 = np.array([self.max_blue_for_red1,self.max_green_for_red1,self.max_red_for_red1])
            lower_red1 = np.array([self.min_blue_for_red1,self.min_green_for_red1,self.min_red_for_red1])
            upper_red2 = np.array([self.max_blue_for_red2,self.max_green_for_red2,self.max_red_for_red2])
            lower_red2 = np.array([self.min_blue_for_red2,self.min_green_for_red2,self.min_red_for_red2])
            mask1 = cv.inRange(img, lower_red1, upper_red1)
            mask2 = cv.inRange(img, lower_red2, upper_red2)
            mask = mask1 + mask2
            _, img = cv.threshold(mask, 1, 255, cv.THRESH_BINARY)#Your code here
        elif self.detect_color == "yellow":
            upper_yellow = np.array([self.max_blue_for_yellow,self.max_green_for_yellow,self.max_red_for_yellow])
            lower_yellow = np.array([self.min_blue_for_yellow,self.min_green_for_yellow,self.min_red_for_yellow])
            mask = cv.inRange(img, lower_yellow, upper_yellow)
            _, img = cv.threshold(mask, 1, 255, cv.THRESH_BINARY)#Your code here
        else:
            raise ValueError("Invalid color")
        #Optional: Add mask processing here
        #-----Mask Visualization: for Debug, Comment this before autotesting/submission!-----#
        cv.imshow("binary_img", img)
        cv.waitKey(0)
        cv.destroyAllWindows()
        #-----Find countours-----#
        contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #-----Find Objectives-----#
        objectives = []
        for contour in contours:
            #Hint: There are more than 1 ways to find circles, try them out and compare the results.
            #Your code here
            if len(contour) >= 20:  # Need at least 5 points to fit an ellipse
                ellipse = cv.fitEllipse(contour)
                try:
                    center = (int(ellipse[0][0]), int(ellipse[0][1]))
                except:
                    continue
                objectives.append(center)
        #-----Objectives Postprocessing-----#
        #Hint: objectives are RINGS. How to determine if overlapping circles are representing the same ring?
        #Your code here
        #print(objectives)
        '''
        to_remove = []
        for objective in objectives:
            for other_objective in objectives:
                if objective == other_objective:
                    continue
                if math.sqrt((objective[0][0] - other_objective[0][0])**2 + (objective[0][1] - other_objective[0][1])**2) < abs(objective[1] - other_objective[1]):
                    to_remove.append(other_objective)
        for item in to_remove:
            if item in objectives:
                objectives.remove(item)
        '''
        #print(objectives)
        return objectives #each objective should be ((x,y), area). Since area is not required, if you did not include it in your code, you can return ((x,y), 0) instead.

if __name__ == '__main__':
    #File input subject to change if not using Linux
    args = sys.argv
    if len(args) > 1:
        filename = args[1]
    else:
        filename = "images/Image_2023-10-08_13:39:27.png" 
    if len(args) > 2:
        detect_color = args[2]
    else:
        detect_color = "red"
    detector = Detector(detect_color, "parameters.yaml")
    image = cv.imread(filename)
    objectives = detector.detect(image)
    for ring in objectives:
        print(ring)
    #---------Visualization---------#
    VisualizeResult= False #Set to False when autotesting/before submission!
    if not VisualizeResult:
        exit()
    for ring in objectives:
        cv.circle(image, ring, 5, (0, 255, 0), -1)
        #cv.circle(image, ring[0], int(math.sqrt(ring[1]/3.14)), (255, 0, 0), 2)
    cv.imshow("image", image)
    cv.waitKey(0)
    cv.destroyAllWindows()
    