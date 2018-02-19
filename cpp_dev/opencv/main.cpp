#include "main.h"

using namespace cv;
using namespace std;
//function blurrs and erodes/dilliates the input image to remove high frequncy nose in the image
void image_optimizations(Mat *imgThresholded)
{
  //Blurr image
  GaussianBlur(*imgThresholded,*imgThresholded, Size(9.0,9.0), 3.0, 3.0);
  //morphological opening (remove small objects from the foreground)
  erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //morphological closing (fill small holes in the foreground)
  //dilate(&imgThresholded, &imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  //erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
}
//function takes in HSV image and color boundaries for detection
void detect_color(Mat *imageHSV,vector<int> *thresholds,Mat *imgThresholded)
{
  //Thresholding for 2 element boundaries
  if((*thresholds).size() == 2)
  {
    inRange(*imageHSV, Scalar((*thresholds)[0], 0, 0), Scalar((*thresholds)[1], 255, 255), *imgThresholded); //Threshold the image
  }
  //threshold for four element boundaries
  if((*thresholds).size() == 4)
  {
    Mat mask1;
    inRange(*imageHSV, Scalar((*thresholds)[0], 0, 0), Scalar((*thresholds)[1], 255, 255), mask1); //Threshold the image
    Mat mask2;
    inRange(*imageHSV, Scalar((*thresholds)[2], 0, 0), Scalar((*thresholds)[3], 255, 255), mask2); //Threshold the image
    addWeighted( mask1, 1.0, mask2, 1.0, 0.0, *imgThresholded);
  }
}
// function returns center point of color contour drived from input mask
 void detectContour(Mat *mask,Point2f *center)
 {
  //create vector for savng found contour points
  vector<vector<Point> >contours;
  //fnd contour pints in mask
  findContours(*mask,contours ,CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
  int largest_area=0;
  int largest_contour_index=0;
  // iterate through each contour.
  for( size_t i = 0; i< contours.size(); i++ )
    {
        float area = contourArea( contours[i] );
        //Find the area of contour
        if( area > largest_area )
        {
            largest_area = area;
            //Store the index of largest contour
            largest_contour_index = i;
        }
    }
  //calculate minimum rotated Bounding rect
  RotatedRect minRect = minAreaRect(Mat(contours[largest_contour_index]));
  //calculate center point of boundnig rect
  *center = minRect.center;
   }

 int main( int argc, char** argv )
 {
    //TODO Read frame from raspberry pi camera  https://www.uco.es/investiga/grupos/ava/node/40
    VideoCapture cap(0); //capture the video from web cam
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    //Read out frames from camera
    while (true)
    {
      // Record start time

        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        //image readout 10-20ms
         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
  auto start = std::chrono::high_resolution_clock::now();
  Mat imgHSV;
  //1-3 ms
  cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  //9-11 ms
  image_optimizations(&imgHSV);
  //thresh green
  vector<int> thresh {36,49};
  //TODO Add other thresholds
  Mat imgThresholded;
  //0.4-0.9 ms
  detect_color(&imgHSV, &thresh, &imgThresholded);
  //1-3 ms
  image_optimizations(&imgThresholded);
  Point2f contourCenter;

  //0.9-1.4 ms
  detectContour(&imgThresholded, &contourCenter);
  //cout<<contourCenter.x<<"|"<<contourCenter.y<<endl;



  //TODO threshold for all given colors
//  imshow("thresh Image", imgThresholded); //show the thresholded image
  //imshow("HS Image", imgHSV); //show the thresholded image
  //imshow("Original", imgOriginal); //show the original image
  //imshow("Contour Image",contourImage);
 auto finish = std::chrono::high_resolution_clock::now();
 std::chrono::duration<double> elapsed = finish - start;
 std::cout << "Elapsed time: " << elapsed.count() << " s\n";
 // Record end time
// auto finish = std::chrono::high_resolution_clock::now();
// std::chrono::duration<double> elapsed = finish - start;
// std::cout << "Elapsed time: " << elapsed.count() << " s\n";
    }

   return 0;

}
/*
 int iLowH = 0;
 int iHighH = 179;

 int iLowS = 0;
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;

 //Create trackbars in "Control" window
 cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "Control", &iHighV, 255);
*/
