import os
import sys
import cv2 as cv
import numpy as np
import yaml
import math

VisualizeResult = False

class Detector:
    def __init__(self, detect_color: str, yaml_path: str):
        self.detect_color = detect_color
        with open(yaml_path, 'r') as file:
            parameters = yaml.safe_load(file)
        
        if detect_color == "red":
            self.min_color1 = np.array(parameters['red']['min1'])
            self.max_color1 = np.array(parameters['red']['max1'])
            self.min_color2 = np.array(parameters['red']['min2'])
            self.max_color2 = np.array(parameters['red']['max2'])
        elif detect_color == "yellow":
            self.min_color = np.array(parameters['yellow']['min'])
            self.max_color = np.array(parameters['yellow']['max'])
        else:
            raise ValueError("Invalid color")
    
    def detect(self, img: np.ndarray):
        #-----Image Preprocessing-----#
        # Convert the image to the HSV color space
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        #-----Color Detection-----#
        if self.detect_color == "red":
            mask1 = cv.inRange(hsv_img, self.min_color1, self.max_color1)
            mask2 = cv.inRange(hsv_img, self.min_color2, self.max_color2)
            mask = cv.bitwise_or(mask1, mask2)
        elif self.detect_color == "yellow":
            mask = cv.inRange(hsv_img, self.min_color, self.max_color)
        else:
            raise ValueError("Invalid color")
        
        #-----Morphological Closing to Eliminate Isolated Zeros-----#
        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        #-----Mask Visualization-----#
        if VisualizeResult == True:
            cv.imshow("mask", mask)
            cv.waitKey(0)
            cv.destroyAllWindows()
        
        #-----Find countours-----#
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        #-----Visualize contours-----#
        if VisualizeResult == True:
            img_with_contours = img.copy()
            cv.drawContours(img_with_contours, contours, -1, (0, 255, 0), 2)
            cv.imshow("Contours", img_with_contours)
            cv.waitKey(0)
            cv.destroyAllWindows()
        
        #-----Find Objectives-----#
        objectives = []
        for contour in contours:
            ((x, y), radius) = cv.minEnclosingCircle(contour)
            if int(radius) >= 5:
                objectives.append(((int(x), int(y)), int(radius)))
                
        #-----Visualize Centers of Circles Before Merging-----#
        if VisualizeResult == True:
            img_with_centers = img.copy()
            for (center, radius) in objectives:
                cv.circle(img_with_centers, center, 3, (255, 0, 0), -1)  # Draw small circle at center
                cv.circle(img_with_centers, center, radius, (0, 255, 0), 2)  # Draw circle with radius
            cv.imshow("Centers and Radii Before Merging", img_with_centers)
            cv.waitKey(0)
            cv.destroyAllWindows()
        
        #-----Objectives Postprocessing-----#
        #print(objectives)
        merged_objectives = self.merge_overlapping_circles(objectives)
        
        #print(merged_objectives)
        merged_objectives = [circle for circle in merged_objectives if circle[1] >= 10]
        merged_objectives = sorted(merged_objectives, key=lambda x: x[1], reverse=True)
        
        return merged_objectives

    def merge_overlapping_circles(self, circles):
        merged = []
        while circles:
            circle = circles.pop(0)
            x1, y1, r1 = circle[0][0], circle[0][1], circle[1]
            for other in circles[:]:
                x2, y2, r2 = other[0][0], other[0][1], other[1]
                distance = math.dist((x1, y1), (x2, y2))
                if distance < min(r1, r2):
                    # Merge circles
                    if r1 < r2:
                        new_x, new_y, new_r = x1, y1, r1
                    else:
                        new_x, new_y, new_r = x2, y2, r2
                    circle = ((new_x, new_y), new_r)
                    circles.remove(other)
            merged.append(circle)
        return merged

if __name__ == '__main__':
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

    if not VisualizeResult:
        exit()
    for ring in objectives:
        cv.circle(image, ring[0], 5, (0, 255, 0), -1)
        cv.circle(image, ring[0], int(math.sqrt(ring[1]/3.14)), (255, 0, 0), 2)
    cv.imshow("image", image)
    cv.waitKey(0)
    cv.destroyAllWindows()
    