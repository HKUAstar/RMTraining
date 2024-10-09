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
        self.red_lower1 = np.array(parameters['red_lower1'])
        self.red_upper1 = np.array(parameters['red_upper1'])
        self.red_lower2 = np.array(parameters['red_lower2'])
        self.red_upper2 = np.array(parameters['red_upper2'])
        self.yellow_lower = np.array(parameters['yellow_lower'])
        self.yellow_upper = np.array(parameters['yellow_upper'])

        #Add more parameters here
    def detect(self, img: np.ndarray):
        #-----Image Preprocessing-----#
        #This is optional.
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        #-----Color Detection-----#
        #Modify your parameters so that the mask include as much of the ring as possible
        #while excluding as much of the environment as possible.
        if self.detect_color == "red":
            mask1 = cv.inRange(hsv, self.red_lower1, self.red_upper1)
            mask2 = cv.inRange(hsv, self.red_lower2, self.red_upper2)
            mask = mask1 + mask2 #Your code here
        elif self.detect_color == "yellow":
            mask = cv.inRange(hsv, self.yellow_lower, self.yellow_upper)#Your code here
        else:
            raise ValueError("Invalid color")
        #Optional: Add mask processing here
        # kernel = np.ones((5, 5), np.uint8)
        # mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        #-----Mask Visualization: for Debug, Comment this before autotesting/submission!-----#
        # cv.imshow("mask", mask)
        # cv.waitKey(0)
        # cv.destroyAllWindows()
        #-----Find countours-----#
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #-----Find Objectives-----#
        objectives = []
        for contour in contours:
            area = cv.contourArea(contour)
            if area > 5:  # Filter based on minimum area to avoid small noise
                # Fit a minimum enclosing circle around the contour
                (x, y), radius = cv.minEnclosingCircle(contour)
                if radius > 1:  # Filter out very small circles
                    objectives.append(((int(x), int(y)), int(radius ** 2 * 3.14)))  # Circle with (center, area)

            #Hint: There are more than 1 ways to find circles, try them out and compare the results.
            #Your code here
        
        #-----Objectives Postprocessing-----#
        merged_objectives = self.merge_overlapping_circles(objectives)
        #Hint: objectives are RINGS. How to determine if overlapping circles are representing the same ring?
        #Your code here
        return merged_objectives
         #each objective should be ((x,y), area). Since area is not required, if you did not include it in your code, you can return ((x,y), 0) instead.

    def merge_overlapping_circles(self, circles):
        merged = []
        while circles:
            (x, y), radius = circles.pop(0)
            to_merge = [(x, y, radius)]
            for ((x2, y2), radius2) in circles[:]:
                distance = math.sqrt((x - x2) ** 2 + (y - y2) ** 2)
                if distance < (radius + radius2) / 2:
                    to_merge.append((x2, y2, radius2))
                    circles.remove(((x2, y2), radius2))
            if to_merge:
                total_weight = sum([c[2] for c in to_merge])
                avg_x = int(sum([c[0] * c[2] for c in to_merge]) / total_weight)
                avg_y = int(sum([c[1] * c[2] for c in to_merge]) / total_weight)
                avg_radius = sum([c[2] for c in to_merge]) / len(to_merge)
                merged.append(((avg_x, avg_y), avg_radius))
        return merged

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
        print(ring[0][0], ring[0][1])
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
    