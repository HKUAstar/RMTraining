import os
import sys
import cv2 as cv
import numpy as np
import yaml
import math

"""
low_hue_red_lower: [0, 100, 100] #色相0-10的红色
low_hue_red_upper: [10, 255, 255]

high_hue_red_lower: [170, 100, 100] #色相170-179的红色
high_hue_red_upper: [179, 255, 255]

hue_yellow_lower: [20, 100, 100] #色相20-30的黄色
hue_yellow_upper: [30, 255, 255]
"""



class Detector:
    def __init__(self, detect_color: str, yaml_path: str):
        self.detect_color = detect_color
        with open(yaml_path, 'r') as file:
            parameters = yaml.safe_load(file)
        self.low_hue_red_lower = np.array(parameters["low_hue_red_lower"])
        self.low_hue_red_upper = np.array(parameters["low_hue_red_upper"])
        self.high_hue_red_lower = np.array(parameters["high_hue_red_lower"])
        self.high_hue_red_upper = np.array(parameters["high_hue_red_upper"])
        self.hue_yellow_lower = np.array(parameters["hue_yellow_lower"])
        self.hue_yellow_upper = np.array(parameters["hue_yellow_upper"])
        self.min_area = parameters["min_area"]

    def detect(self, img: np.ndarray):
        #-----Image Preprocessing-----#
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        #-----Color Detection-----#
        #Modify your parameters so that the mask include as much of the ring as possible while excluding as much of the environment as possible.
        if self.detect_color == "red":
            mask_low_red = cv.inRange(hsv_img, self.low_hue_red_lower, self.low_hue_red_upper) #masks are binary images
            mask_high_red = cv.inRange(hsv_img, self.high_hue_red_lower, self.high_hue_red_upper)
            mask = mask_low_red + mask_high_red
        elif self.detect_color == "yellow":
            mask = cv.inRange(hsv_img, self.hue_yellow_lower, self.hue_yellow_upper)
        else:
            raise ValueError("Invalid color")
        
        #-----Mask Visualization: for Debug, Comment this before autotesting/submission!-----#
        #cv.imshow("mask", mask)
        #cv.waitKey(0)
        #cv.destroyAllWindows()


        
        #-----Find countours-----#
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)


        #-----Find Objectives-----#
        objectives = []
        wanted_contours = []
        for contour in contours:
            con_area = cv.contourArea(contour)
            if con_area < self.min_area:
                continue
            wanted_contours.append(contour)
            (x, y), radius = cv.minEnclosingCircle(contour)
            objectives.append(((int(x), int(y)), con_area))

        return self.remove_overlapping(objectives)
    
    def remove_overlapping(self, objectives):
        copy = objectives
        processed = []
        result = []
        for ring in objectives:
            for ring2 in copy:
                if ring == ring2:
                    continue
                if ring in processed:
                    continue
                if (math.sqrt((ring[0][0] - ring2[0][0])**2 + (ring[0][1] - ring2[0][1])**2) < (math.sqrt(ring[1]/math.pi) + math.sqrt(ring2[1]/math.pi))):
                    avg_x = (ring[0][0] + ring2[0][0]) / 2
                    avg_y = (ring[0][1] + ring2[0][1]) / 2
                    area = ring[1] if ring[1] > ring2[1] else ring2[1]
                    result.append(((int(avg_x), int(avg_y)), area))
                    processed.append(ring2)
                    processed.append(ring)
                    break
            
                
        return result

 

        """
        img_copy = img.copy()
        print(len(wanted_contours))
        cv.drawContours(img_copy, wanted_contours, -1, (0, 255, 0), 3)
        cv.imshow("contours", img_copy)
        cv.waitKey(0)
        cv.destroyAllWindows()
        """


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
    #print(objectives)
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
    

    



