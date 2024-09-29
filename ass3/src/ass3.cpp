#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

class Solution {
public:
    std::vector<std::pair<int, int>> detectRing(std::string name, std::string color) {
        std::vector<std::pair<int, int>> ret;

        cv::Mat img = cv::imread(name);
        if(img.empty()){
            std::cerr << "invalid input" << std::endl;
            return {};
        }
        /*
            BAD!!!
            binary process 
        */ 

        // cv::Mat binary;
        // cv::cvtColor(img, binary, cv::COLOR_BGR2GRAY);
        // cv::imwrite("test.png", binary);

        /*
            hsv process 
        */
        cv::Mat img_hsv, binary, thresh, filtered;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
        // cv::imwrite("a1.png", img_hsv);

        if(color == "red"){
            cv::Scalar lowerRed1(0, 100, 100);
            cv::Scalar upperRed1(10, 255, 255); 
            cv::Scalar lowerRed2(160, 100, 100);
            cv::Scalar upperRed2(180, 255, 255);

            cv::Mat maskRed1, maskRed2, maskRed;
            cv::inRange(img_hsv, lowerRed1, upperRed1, maskRed1);
            cv::inRange(img_hsv, lowerRed2, upperRed2, maskRed2);

            filtered = maskRed1 | maskRed2;
            // cv::imwrite("a4.png", filtered);
        } else if (color == "yellow") {
            cv::Scalar lowerYellow(20, 100, 100);
            cv::Scalar upperYellow(30, 255, 255);
            cv::inRange(img_hsv, lowerYellow, upperYellow, filtered);
            // cv::imwrite("a4.png", filtered);
        }
        
        std::vector<std::vector<cv::Point>> contours, filteredContours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(filtered, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        for(int i = 0; i < contours.size(); ++i) {
            cv::Rect box = cv::boundingRect(contours[i]);
            if(hierarchy[i][2] != -1) {
                double a = cv::contourArea(contours[i]);
                double p1 = cv::arcLength(contours[i], true);
                double p2 = cv::arcLength(contours[hierarchy[i][2]], true);
                double ratio = p1 / p2;
                if(a > CONF_AREA || ratio > 0.75 * RATIO && ratio < 1.25 * RATIO && a >= MIN_AREA){
                    filteredContours.push_back(contours[i]);
                    
                }
            }
        }

        cv::Mat output = img.clone();
        std::sort(filteredContours.begin(), filteredContours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2){
            return cv::arcLength(c1, true) > cv::arcLength(c2, true);
        });
        for(auto& contour: filteredContours) {
            cv::Rect box = cv::boundingRect(contour);
            cv::rectangle(output, box, cv::Scalar(0, 255, 0), 1);
            int x = (box.x + box.width) >> 1;
            int y = (box.y + box.height) >> 1;
            ret.push_back(std::make_pair(x, y));
        }
        // cv::imwrite("output.png", output);
        
        return ret;
    }
    
private:
    const double CONF_AREA = 10000.0; 
    const double MIN_AREA = 100.0;
    const double RATIO = 1.30;
};

int speedUp = [] {
    std::ios::sync_with_stdio(0);
    std::cin.tie(0);
    return 0;
}();

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <image_path> <color>" << std::endl;
        return 1;
    }
    std::string name(argv[1]);
    std::string color(argv[2]);

    Solution s;
    std::vector<std::pair<int, int>> rings = s.detectRing(name, color);
    for(auto& ring: rings) {
        std::cout << ring.first << " " << ring.second << std::endl;
    }
    // s.detectRing("images/Image_2023-10-08_14_20_58.png", "yellow");
}
