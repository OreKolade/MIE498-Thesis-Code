#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x*(180/M_PI), y*(180/M_PI), z*(180/M_PI));
   
}

int main(int argc,char **argv){
   cv::Mat image ;
   cv::Mat cameraMatrix, distCoeffs;
   int tvec_origin;
   int marker_origin;
   cv::Mat rot_mat = Mat::zeros(3,3,CV_64F);
   #define M_PI  3.14159265358979323846 /* pi*/
   //double R[3][3]; 
   //double Q[3][3];
   //struct S;
   

   //readCameraParameters(cameraMatrix,distCoeffs)
   FileStorage fs("charucooutfilehighest.xml",FileStorage::READ);//read out_camera_data.xml for camera matix and distortions
   fs["camera_matrix"]>>cameraMatrix;
   fs["distortion_coefficients"]>>distCoeffs;
   cout << "camera matrix: " << cameraMatrix << endl
     << "distortion coeffs: " << distCoeffs << endl;

   cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
   if (argc != 2 )
   { cout<<"Usage: inimage"<<endl;
     return -1;
   }
   
   image= imread(argv[1],1);
   
   std::vector<int> markerIds;
   std::vector<std::vector<cv::Point2f>> markerCorners;
   //cv::Ptr<cv::aruco::DetectorParameters> parameters;IF nothing is assigned to it , it causes a segmentation fault
   //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	   
   //detect
   cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
   //std::cout << "This is the Array of detected markers"<< markerIds<<std::endl;
   //std::cout << "This is the Array of detected corners"<< markerCorners<<std::endl;
   for(int i=0; i<markerIds.size(); ++i) {//Helperrrrrr
    	 std::cout <<"A detected Marker: "<<markerIds[i]<<std::endl;
	}

   //print info to console
   if (markerIds.size() > 0){
      cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
      std::vector<cv::Vec3d> rvecs, tvecs;//new
      cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.022, cameraMatrix, distCoeffs, rvecs, tvecs);//30 is the size of the marker side in meters or in any other unit(I used mm, so 30mm=0.03meters hard coded for now). Changed to 22mm
      // draw axis for each marker
      for(int i=0; i<markerIds.size(); i++)
          cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

  
      for(int i=0; i<tvecs.size(); ++i) {
         cv::Rodrigues(rvecs[i],rot_mat);
         //[R,Q,S]= cv::RQDecomp3x3(rot_mat);
    	 std::cout <<"For "<<markerIds[i]<<" Marker: ""rvecs is: "<< rvecs[i] <<std::endl;
         //std::cout <<" And it's Rotational matrix is:"<<rot_mat<<std::endl;//Don't need rotational vector now
         //std::cout <<"For "<<markerIds[i]<<" Marker: ""Euler angles is: "<< cv::RQDecomp3x3(rot_mat)<<std::endl;

         std::cout <<"For "<<markerIds[i]<<" Marker: Eulerangles(deg) is: "<< rotationMatrixToEulerAngles(rot_mat)<<std::endl;
         std::cout <<"For "<<markerIds[i]<<" Marker: ""tvecs is: "<< tvecs[i] <<std::endl;
         std::cout <<"     "<<std::endl;
	}
      for(int i=0; i<markerIds.size(); ++i) {
         std::cout <<"For "<<markerIds[i]<<" Marker: ""tvecs is: "<< tvecs[i] <<std::endl;
         std::cout <<"     "<<std::endl;
         marker_origin=i;
         tvec_origin=i; //catches the tvec position of a marker to act as an origin
            for(int j=0; j<tvecs.size(); ++j) {
                std::cout <<"     "<<std::endl;
                if (j != tvec_origin)
                        
         		std::cout <<"For "<<markerIds[marker_origin]<<" Marker. The tvec pose between "<<markerIds[marker_origin] <<" and "<< markerIds[j] <<" with respect to"<< markerIds[marker_origin] <<" Marker is: "<< tvecs[j] - tvecs[tvec_origin] <<std::endl;
	      }
	}
      //posebetweenmarkers(markerIds, tvecs);
   }	 

   cv::namedWindow("image",WINDOW_NORMAL);
   cv::resizeWindow("image",600,600);
   cv::imshow("image",image);
   cv::waitKey(0);
}

