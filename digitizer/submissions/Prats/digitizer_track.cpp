#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <bits/stdc++.h>
using namespace cv;
using namespace std;
RNG rng(12345);
Mat src,dst,canny_frame,src_gray,morph_frame,mst;
int canny_low_thresh=0;
int canny_ratio=3;
int canny_kernel_size=3;
int morph_operator=0;
int morph_elem=0;
int morph_size=1;
int dilation_type = 0;
int dilation_elem = 0;
int dilation_size = 1;
Mat morph_op(Mat frame)
{
  // Since MORPH_X : 2,3,4,5 and 6
  int operation = morph_operator + 2;
  Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  morphologyEx( frame, mst, operation, element );
  return mst;
}


int main(int argc,char **argv){

src=imread("/home/pratyush/Desktop/final_track.jpg",IMREAD_COLOR);
cvtColor( src, src_gray, COLOR_BGR2GRAY );
threshold( src_gray, dst, 200, 255, 1);
morph_frame=morph_op(dst);
Mat element = getStructuringElement( dilation_type,
                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       Point( dilation_size, dilation_size ) );

cv::Canny(morph_frame,canny_frame,canny_low_thresh,canny_ratio,canny_kernel_size);
dilate(canny_frame, canny_frame, element );
    //imshow("canny_image",canny_frame);
//------------find contours -------------------------
    vector<vector<Point> > contours;
    findContours( canny_frame, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<vector<Point> >hull( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    vector<vector<Point> > store;
Mat drawing = Mat::zeros( canny_frame.size(), CV_8UC3 );
Mat sk_drawing = Mat::zeros( canny_frame.size(), CV_8UC3 );
Mat drawing1 = Mat::zeros( canny_frame.size(), CV_8UC3 );
Mat sk_drawing1 = Mat::zeros( canny_frame.size(), CV_8UC3 );
Mat drawing2 = Mat::zeros( canny_frame.size(), CV_8UC3 );
Mat sk_drawing2 = Mat::zeros( canny_frame.size(), CV_8UC3 );
//cout<<contours.size()<<endl;
//double peri= cv::arcLength(contours,true);	
   for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 2, true );
        convexHull( contours[i], hull[i] );
        boundRect[i] = boundingRect( contours_poly[i] );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }

   for( size_t i = 0; i< contours.size(); i++ )
    {   std::string str = std::to_string(i);
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        if(i==38||i==44||i==50||i==54)//way1
        {drawContours( sk_drawing, contours_poly, (int)i,Scalar(255,255,255) );
         vector<Point> tmp = contours[i];
        const Point* elementPoints[2]= { &tmp[0] };
        int numberOfPoints = (int)tmp.size();
        fillPoly (drawing, elementPoints, &numberOfPoints, 1, Scalar (255, 255, 255), 8);
         //store.push_back(contours_poly[i]);
         //cv::putText(drawing, str, centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.5,CV_RGB(118, 185, 0), 2);  
        }
        if(i==42||i==45||i==50||i==54)//way2
        {//drawContours( drawing1, contours_poly, (int)i, color );
         drawContours( sk_drawing1, contours_poly, (int)i,Scalar(255,255,255) );
         vector<Point> tmp = contours[i];
        const Point* elementPoints[2]= { &tmp[0] };
        int numberOfPoints = (int)tmp.size();
        fillPoly (drawing1, elementPoints, &numberOfPoints, 1, Scalar (255, 255, 255), 8);
         //store.push_back(contours_poly[i]);
         //cv::putText(drawing1, str, centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.5,CV_RGB(118, 185, 0), 2);  
        }
        if(i==42||i==48||i==54)//way3
        {//drawContours( drawing2, contours_poly, (int)i, color );
         drawContours( sk_drawing2, contours_poly, (int)i,Scalar(255,255,255) );
         vector<Point> tmp = contours[i];
        const Point* elementPoints[2]= { &tmp[0] };
        int numberOfPoints = (int)tmp.size();
        fillPoly (drawing2, elementPoints, &numberOfPoints, 1, Scalar (255, 255, 255), 8);
         //store.push_back(contours_poly[i]);
         //cv::putText(drawing2, str, centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.5,CV_RGB(118, 185, 0), 2);  
        }
        
        //drawContours( drawing, contours_poly, (int)i, Scalar(255,255,255) );
        //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
        //circle( drawing, centers[i], (int)radius[i], color, 2 );
         
        // cv.putText(src, str(i+1), (int(result[0].corners[i][0]),int(result[0].corners[i][1])), cv.FONT_HERSHEY_SIMPLEX,0.4, (0, 0, 0), 2)
        //cv::putText(drawing, str, centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.5,color, 2);        
    }
    
   //cout<<store.size()<<endl;
   imwrite("/home/pratyush/Desktop/way.jpg",drawing);
   imwrite("/home/pratyush/Desktop/sk_way.jpg",sk_drawing);
   imwrite("/home/pratyush/Desktop/way1.jpg",drawing1);
   imwrite("/home/pratyush/Desktop/sk_way1.jpg",sk_drawing1);
   imwrite("/home/pratyush/Desktop/way2.jpg",drawing2);
   imwrite("/home/pratyush/Desktop/sk_way2.jpg",sk_drawing2);

   imshow("canny_image",canny_frame);
   imshow("contour",drawing);
   imshow("contour1",drawing1);
   imshow("contour2",drawing2);
   imshow("sk_contour",sk_drawing);
   imshow("sk_contour1",sk_drawing1);
   imshow("sk_contour2",sk_drawing2);
   cout<<drawing.rows<<" "<<drawing.cols<<endl;
   cv::waitKey();
//---------------------------------------------------------
}
