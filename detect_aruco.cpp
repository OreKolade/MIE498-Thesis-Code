#include <opencv2/aruco.hpp>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc,char **argv){
   cv::Mat image ;
   cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
   //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
   cout << "Get here 1 ?";
   if (argc != 2 )
   { cout<<"Usage: inimage"<<endl;
     return -1;
   }
   //char* imageName = argv[1];
   
   
   image= imread(argv[1],1);
   
   //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
   
   
   std::vector<int> markerIds;
   std::vector<std::vector<cv::Point2f>> markerCorners;
   //cv::Ptr<cv::aruco::DetectorParameters> parameters;IF nothing is assigned to it , it causes a segmentation fault
   //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	   
   //detect
   cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

   //print info to console
   if (markerIds.size() > 0)
      cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
	   
   cv::namedWindow("image",WINDOW_NORMAL);
   cv::resizeWindow("image",600,600);
   cv::imshow("image",image);
   cv::waitKey(0);

}

