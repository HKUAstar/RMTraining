import cv2 as cv
import os
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict, Tuple

# Get data
import requests
from pathlib import Path
import zipfile

url = "https://github.com/WXH-Olina/dataTransfer/raw/refs/heads/Opencv_tutorial_images/images.zip"

data_path = Path("data/")
images_path = data_path / "images"

if images_path.is_dir():
  print("Directory exists.")
  pass
else:
  print("Making directory ...")
  images_path.mkdir(parents=True, exist_ok=True)

# Download the zip file
response = requests.get(url)
if response.status_code == 200:
    with open(images_path / "images.zip", "wb") as f:
        print("Writing zip ...")
        f.write(response.content)
else:
    print(f"Failed to download file: {response.status_code}")
with zipfile.ZipFile(images_path / "images.zip", "r") as zip_ref:
  print("Unzipping ...")
  zip_ref.extractall(images_path)
  print("Done")

# Write a function here to process the center_points & the major_axis, to detect a pair of circles as a whole ring
# Can be integrated into the above function, so this is an updated_draw_contours
import math

image_path = images_path / "red"
color = "red"

def updated_draw_contours(image_folder_path: str, color: str = None, min_area: int = 100, min_d: int=15) -> List:
  """
  Take in the path of the folder storing images & the color of the circle to detect
  Without input color, the prog will detect red and yellow circles

  Return a list: [List: source_rgb_images, List: processed_rgb_images, List: List-(center: (x,y), diameter/major axis)]
  """
  imgs = []
  processed_imgs = []
  ellipses = []
  # For testing
  count = -1
  for filename in os.listdir(image_folder_path):
    this_ellipses = []
    if filename.endswith((".jpeg", ".jpg", ".png")):
      #For testing
      count+=1
      # Get source images
      image_path = os.path.join(image_folder_path, filename)
      img = cv.imread(image_path)
      imgs.append(cv.cvtColor(img, cv.COLOR_BGR2RGB))

      # Process the image
      # 1. blur & To hsv image
      blur = cv.blur(img, (5,5))
      hsv_img = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

      # 2. Make mask - red/yellow
      if color == "red" or color == None:
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv.inRange(hsv_img, lower_red_1, upper_red_1)
        mask2 = cv.inRange(hsv_img, lower_red_2, upper_red_2)
        mask = cv.bitwise_or(mask1, mask2)
        ret, binary_mask = cv.threshold(mask, 1, 255, cv.THRESH_BINARY)

        contours, _ = cv.findContours(binary_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
          if cv.contourArea(cnt) >= min_area:
            # Fit an ellipse to the contour
            if len(cnt) >= 5:  # fitEllipse requires at least 5 points
              ellipse = cv.fitEllipse(cnt)
              this_ellipses.append(ellipse)

        # 'Merge' some center_points in the ellipse
        sorted(this_ellipses, key=lambda _: _[0][0])
        merged_ellipses = []
        x_counter = -min_d -1
        y_counter = -min_d -1
        for ellipse in this_ellipses:
          (cx, cy), (a,b), angle = ellipse
          if abs(cx - x_counter) > min_d and abs(cy - y_counter) > min_d:
            merged_ellipses.append(ellipse)
          x_counter = cx
          y_counter = cy
        
        masked_img = cv.bitwise_and(img, img, mask=binary_mask)
        # Draw centers here
        for ellipse in merged_ellipses:
          cv.ellipse(masked_img, ellipse, color=(0,225,0), thickness=2)
          (cx, cy), (a, b), angle = ellipse
          center = (int(cx), int(cy))
          cv.circle(masked_img, center, color=(0, 225, 0), radius=3, thickness=-1)

        processed_imgs.append(cv.cvtColor(masked_img, cv.COLOR_BGR2RGB))
        ellipses.append(merged_ellipses)


      elif color == "yellow" or color == None:
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])

        mask = cv.inRange(hsv_img, lower_yellow, upper_yellow)
        ret, binary_mask = cv.threshold(mask, 1, 225, cv.THRESH_BINARY)

        contours, _ = cv.findContours(binary_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
          if cv.contourArea(cnt) >= min_area:
            # Fit an ellipse to the contour
            if len(cnt) >= 5:  # fitEllipse requires at least 5 points
              ellipse = cv.fitEllipse(cnt)
              this_ellipses.append(ellipse)

        # 'Merge' some center_points in the ellipse
        sorted(this_ellipses, key=lambda _: _[0][0])
        merged_ellipses = []
        x_counter = -min_d -1
        y_counter = -min_d -1
        for ellipse in this_ellipses:
          (cx, cy), (a,b), angle = ellipse
          if abs(cx - x_counter) > min_d and abs(cy - y_counter) > min_d:
            merged_ellipses.append(ellipse)
          x_counter = cx
          y_counter = cy
        
        masked_img = cv.bitwise_and(img, img, mask=binary_mask)
        # Draw centers here
        for ellipse in merged_ellipses:
          cv.ellipse(masked_img, ellipse, color=(0,225,0), thickness=2)
          (cx, cy), (a, b), angle = ellipse
          center = (int(cx), int(cy))
          cv.circle(masked_img, center, color=(0, 225, 0), radius=3, thickness=-1)

        processed_imgs.append(cv.cvtColor(masked_img, cv.COLOR_BGR2RGB))
        ellipses.append(merged_ellipses)

  return [imgs, processed_imgs, ellipses]


# Get the color of circle to detect
color = input("Enter the color of the circle to detect: ")
color = "red"
imgs, processed_imgs, ellipses = updated_draw_contours(image_path, color, 166, min_d=20)

plt.figure(figsize=(20,30))
l = len(imgs)
save_path = Path(f"{color}_circle_detection")
if save_path.is_dir():
  print("Directory exists.")
  pass
else:
  print("Making directory ...")
  save_path.mkdir(parents=True, exist_ok=True)
for i in range(l):
  img = imgs[i]
  processed_img = processed_imgs[i]
  plt.subplot(l, 2, 2*i+1)
  plt.imshow(img)
  plt.title("src_img")
  plt.axis('off')
  plt.subplot(l, 2,2*i +2)
  plt.imshow(processed_img)
  title = f"processed_img, {len(ellipses[i])} circle(s) detected."
  plt.title(title)
  plt.axis('off')
  plt.savefig(f"{color}_circle_detection/{i}.png")
plt.show()
