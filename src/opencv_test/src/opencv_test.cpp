#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;

ros::Publisher PositionPublisher;

 Mat GetAvrageFilterImage(Mat cameraFeed, Mat &output)
{

    Size filterSize(4, 4);
    Mat adpOut;
    if (cameraFeed.empty())return cameraFeed;
    blur(cameraFeed, output, filterSize);
    cvtColor(output, output, CV_BGR2GRAY);

    //threshold(output, output, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    adaptiveThreshold(output, output, 255.0, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2.0);
    //adaptiveThreshold(output, adpOut, 255.0, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2.0);
//bitwise_or(output,adpOut,output);
    return cameraFeed;
}

 vector<Rect>  FindBlobs(Mat thresholdImage, Mat &cameraFeed,uint max,uint min)
 {
     Mat src_copy = cameraFeed.clone();
vector<Rect> res;
        Rect foundedRect = Rect(0,0,0,0);
     vector<vector<Point> > contours,outcontours;
     vector<Vec4i> hierarchy;

     findContours( thresholdImage,  contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
Mat drawing = Mat::zeros( thresholdImage.size(), CV_8UC3 );
     for( uint i = 0; i < contours.size(); i++ )
     {
     drawContours(drawing, contours, i, Scalar(255,255,255), FILLED);
        }


     //imshow( "Hull demo", drawing );
        cvtColor(drawing, thresholdImage, CV_BGR2GRAY);
     /// Find contours
     findContours( thresholdImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );



     //for( uint i = 0; i < contours.size(); i++ )
     //{

        //if(contours.at(i).size()<min)continue;
        //if(contours.at(i).size()>max)continue;

       //      outcontours.push_back(contours.at(i));
     //}

//     vector<vector<Point> >hull( outcontours.size() );
//     for( size_t i = 0; i < outcontours.size(); i++ )
//     {
//         convexHull( outcontours[i], hull[i] );
//     }
     /// Draw contours + hull results



     for( uint i = 0; i< contours.size(); i++ )
        {
          Scalar color = Scalar(255,100,0 );
            foundedRect=boundingRect(contours[i]);

            //if(foundedRect.width)continue;
            //if(foundedRect.height<40)continue;
            //std::cout << " Area:"<<i<<" ="  <<foundedRect.height-foundedRect.width<<std::endl;
            //std::cout << " Area: " << contourArea(contours[i]) << " c="<<i<<std::endl;
            if(abs(foundedRect.height-foundedRect.width)>30)continue;
            if(contourArea(contours[i])<1000)continue;
            if(contourArea(contours[i])>10000)continue;
            foundedRect.x+=350;//this shift is because of image crop
            rectangle(cameraFeed,foundedRect,Scalar(30,100,255 ));
            res.push_back(foundedRect);
         //drawContours(drawing, outcontours, i, Scalar(255,255,255), FILLED);
          //drawContours( drawing, outcontours, i, color, 1, CV_FILLED, vector<Vec4i>(), 0, Point() );
          //drawContours( cameraFeed,outcontours, i, color, 1, 1, vector<Vec4i>(), 0, Point() );
        }

//vector <Point >res;
     return  res;
     //cameraFeed=drawing;
     //namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
     //imshow( "Hull demo", drawing );

 }

 Mat  FilterBlob(Mat thresholdImage, Mat &cameraFeed,uint max,uint min)
 {
     Mat src_copy = cameraFeed.clone();

     vector<vector<Point> > contours,outcontours;
     vector<Vec4i> hierarchy;


     /// Find contours
     findContours( thresholdImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );



     for( uint i = 0; i < contours.size(); i++ )
     {

         if(contours.at(i).size()<min)continue;
         if(contours.at(i).size()>max)continue;

             outcontours.push_back(contours.at(i));
     }

     vector<vector<Point> >hull( outcontours.size() );
     for( size_t i = 0; i < outcontours.size(); i++ )
     {
         convexHull( outcontours[i], hull[i] );
     }
     /// Draw contours + hull results

     Mat drawing = Mat::zeros( thresholdImage.size(), CV_8UC3 );

     for( uint i = 0; i< outcontours.size(); i++ )
        {
          Scalar color = Scalar(255,0,0 );

          drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          drawContours( cameraFeed,hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        }


     return  drawing;
     //cameraFeed=drawing;
     //namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
     //imshow( "Hull demo", drawing );

 }
Mat  DoConvex(Mat thresholdImage, Mat &cameraFeed)
{
    Mat src_copy = cameraFeed.clone();
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    //threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );

    /// Find contours
    findContours( thresholdImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Find the convex hull object for each contour
    vector<vector<Point> >hull( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
       {  convexHull( Mat(contours[i]), hull[i], false ); }

    /// Draw contours + hull results

    Mat drawing = Mat::zeros( thresholdImage.size(), CV_8UC3 );

    for( int i = 0; i< contours.size(); i++ )
       {
         Scalar color = Scalar(255,0,0 );
        // drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
         drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       }


    return  drawing;
    //cameraFeed=drawing;
    //namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
    //imshow( "Hull demo", drawing );

}

Mat  ShapeDetect(Mat thresholdImage, Mat &cameraFeed)
{
    Mat src_copy = cameraFeed.clone();
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    //threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );

    /// Find contours
    findContours( thresholdImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point> > approx;

        approx.resize(contours.size());


for (int i = 0; i < contours.size(); i++)

{
  //  approxPolyDP(Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

    // Skip small or non-convex objects
    if (std::fabs(contourArea(contours[i])) < 100 || isContourConvex(approx))
            continue;

    if (approx.size() == 3)
    {
            ROS_INFO("TRI");                //setLabel(dst, "TRI", contours[i]);    // Triangles
    }
}
        //approxPolyDP(Mat(contours[k]), approx[k], 3, true);

    //return  drawing;
    //cameraFeed=drawing;
    //namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
    //imshow( "Hull demo", drawing );
return thresholdImage;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat output;
        Mat cameraFeed=cv_bridge::toCvShare(msg, "bgr8")->image;
        GetAvrageFilterImage(cameraFeed,output);
        //imshow("filtered", output);

        cv::Rect myROI(350, 0, 800-350,800);

        // Crop the full image to that image contained by the rectangle myROI
        // Note that this doesn't copy the data
      output= output(myROI);

      vector<Rect>result= FindBlobs(output,cameraFeed,300,20);
      geometry_msgs::PoseArray PositionList;
      for ( uint i=0 ;i<result.size();i++) {
          geometry_msgs::Pose p;
          p.position.x=result[i].x+result[i].width/2;
          p.position.y=result[i].y+result[i].height/2;

          PositionList.poses.push_back(p);
      }
  PositionPublisher.publish(PositionList);


     circle(cameraFeed, Point(400,400),20, Scalar(255,0,0),2, 8,0);
imshow("view1", cameraFeed);

imshow("view", output);
    waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  PositionPublisher = nh.advertise<geometry_msgs::PoseArray>("/detected_objects", 100);
  image_transport::Subscriber sub = it.subscribe("rrbot/camera1/image_raw", 1, imageCallback);

ros::spin();
  while(ros::ok())
  {
//      PositionPublisher.publish(PositionList);
   ros::spinOnce();

  }
//   //Mat cameraFeed=cv_bridge::toCvShare(msg, "bgr8")->image;
//   ///QString   CurrentDirectory="/home/amin/1.png";
//   Mat output;
//   Mat cameraFeed=imread("/home/amin/bug2.png");
//   //Mat cameraFeed=imread("/home/amin/robot_project/shadow.png");
//   GetAvrageFilterImage(cameraFeed,output);
//   // Setup a rectangle to define your region of interest
//   cv::Rect myROI(350, 0, 800-350,800);
//   // Crop the full image to that image contained by the rectangle myROI
//   // Note that this doesn't copy the data
//    output= output(myROI);
//    //imshow("filtered", output);
//    FindBlobs(output,cameraFeed,300,20);
//    //imshow("blob", FindBlobs(output,cameraFeed,100,40));
//    imshow("view", cameraFeed);
//    imshow("last", output);

//    ros::spinOnce();
//    waitKey(30);


  cv::destroyWindow("view");
}
