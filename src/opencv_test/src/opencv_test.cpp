#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;

Rect objectBoundingRectangle = Rect(0,0,0,0);
 int obj_x[1000];
 int obj_y[1000];
 int obj_r[1000];
Mat GetAvrageFilterImage(Mat cameraFeed, Mat &output)
{

    Size filterSize(4, 4);


    if (cameraFeed.empty())return cameraFeed;


    blur(cameraFeed, output, filterSize);

    cvtColor(output, output, CV_BGR2GRAY);
    adaptiveThreshold(output, output, 255.0, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2.0);


    return cameraFeed;

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
    findContours( thresholdImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

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


int FindSibleProperties(Mat thresholdImage, Mat &cameraFeed, vector<Point> &ElipsCountor, vector<RotatedRect> &Elipses, int min_w_thr, int min_h_thr, int max_w_thr, int max_h_thr)
{
    bool objectDetected = false;

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;


    Mat temp;
    Elipses.clear();
    ElipsCountor.clear();

    thresholdImage.copyTo(temp);

    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // retrieves external contours

    //if contours vector is not empty, we have found some objects
    if (contours.size() > 0)objectDetected = true;
    else objectDetected = false;


    int cnt = 0;

    Scalar color = Scalar(0, 255, 0);
    Scalar color2 = Scalar(0, 0, 255);
    if (objectDetected) {

        for (int i = 0; i < contours.size(); i++) {
            //if(i>)break;

            vector< vector<Point> > largestContourVec, myobj;
            largestContourVec.push_back(contours.at(contours.size() - (i + 1)));

            objectBoundingRectangle = boundingRect(largestContourVec.at(0));
max_w_thr=10;
max_h_thr=10;
min_w_thr=3;
min_h_thr=3;
            if (objectBoundingRectangle.width < min_w_thr)continue;
            if (objectBoundingRectangle.width > max_w_thr)continue;
            if (objectBoundingRectangle.height < min_h_thr)continue;
            if (objectBoundingRectangle.height > max_h_thr)continue;
            if (cnt == 0)ElipsCountor = largestContourVec.at(0);
////////////////new !!
 //approxPolyDP(i,0.01*arcLength(i,true),true);

              ///////////////new
 if(largestContourVec.size()>0)
 {
    max_h_thr++;
    //fitEllipse(largestContourVec.at(0));
 }

//RotatedRect Elips =  fitEllipse(largestContourVec.at(0));
            //Point2f center;
            //center.y = cameraFeed.rows / 2;
            //center.x = cameraFeed.cols / 2;



            //if (norm(Elips.center - center) > 100)continue;
            //Elipses.push_back(Elips);
//             ellipse( cameraFeed,Elips, color, 2, 8 );


//            vector<Point>c1 = contours.at(contours.size() - (i + 1));

//            obj_x[cnt] = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
//            obj_y[cnt] = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
//            obj_r[cnt] = objectBoundingRectangle.width / 2;
            cnt++;
            if (cnt > 1000)break;

        }
        ROS_INFO("contors=%d",cnt);

    }

//    for (int i = 0; i< contours.size(); i++)
//       {
//           Scalar color = Scalar(255, 0, 0);
//           drawContours(cameraFeed, contours, i, color, 2, 8, hierarchy, 0, Point());
//       }

    return 0;

//    QVector<int>circles;

//    int tmp_r = 0;
//    for (int i = 0; i < cnt; i++) {
//        if (abs(obj_r[i] - tmp_r) > 5) {tmp_r = obj_r[i]; circles.push_back(tmp_r);}
//        qDebug() << "r" << i << obj_r[i];
//    }
//    for (int i = 0; i < circles.length(); i++) {

//        qDebug() << "R" << i << circles[i];
//    }

    return cnt;
}
void DrawElipses(Mat &cameraFeed, vector <RotatedRect> &elipses)
{

    for (int i = 0; i < elipses.size(); i++) {
        ellipse(cameraFeed, elipses[i], Scalar(0, 255, 0), 2, 8);
    }
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat output;
        Mat cameraFeed=cv_bridge::toCvShare(msg, "bgr8")->image;
     ///QString   CurrentDirectory="/home/amin/1.png";
      ///Mat image=imread("/home/amin/1.png");
     //cameraFeed=imread("/home/amin/1.png");
     cameraFeed=imread("/home/amin/shapes.png");
        GetAvrageFilterImage(cameraFeed,output);
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
  image_transport::Subscriber sub = it.subscribe("rrbot/camera1/image_raw", 1, imageCallback);
  //ros::spin();
  while(ros::ok())
  {
      //Mat cameraFeed=cv_bridge::toCvShare(msg, "bgr8")->image;
   ///QString   CurrentDirectory="/home/amin/1.png";
   Mat output;
   Mat cameraFeed=imread("/home/amin/shapes.png");
   //Mat cameraFeed=imread("/home/amin/1.jpg");

   GetAvrageFilterImage(cameraFeed,output);
   imshow("filtered", output);
   output=ShapeDetect(output,cameraFeed);
   //output=DoConvex(output,cameraFeed);
//   vector<Point> elspcnt;
//   vector<RotatedRect> elips;
//   FindSibleProperties(output,cameraFeed,elspcnt,elips,0,0,100,100);
//   DrawElipses(cameraFeed,elips);

imshow("view", cameraFeed);
imshow("last", output);

  ros::spinOnce();
      waitKey(30);


  }
  cv::destroyWindow("view");
}
