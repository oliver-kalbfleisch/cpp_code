#include "main.h"
using namespace cv;
using namespace std;

const int numThreads=4;
mutex detector_mutex;
int number_of_tasks;
boost::mutex task_count_mutex;
boost::condition_variable task_cv;

void TaskComplete()
{
	boost::unique_lock<boost::mutex> lock(task_count_mutex);
	--number_of_tasks;
	task_cv.notify_one();
	}
void wait_for_pool()
{
	boost::unique_lock<boost::mutex> lock(task_count_mutex);
	while(number_of_tasks > 0)
	{
		cout<<number_of_tasks<<endl;
		task_cv.wait(lock);
	}
	}


//function blurrs and erodes/dilliates the input image to remove high frequncy nose in the image
void image_optimizations(Mat *imgThresholded)
{
	//Blurr image
	//GaussianBlur(*imgThresholded,*imgThresholded, Size(9.0,9.0), 3.0, 3.0);
	//morphological opening (remove small objects from the foreground)
	erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	//morphological closing (fill small holes in the foreground)
	dilate(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
}
//function takes in HSV image and color boundaries for detection
void detect_color(Mat *image,vector<int> *thresholds,Mat *imgThresholded)
{ 
inRange(*image, Scalar((*thresholds)[4],(*thresholds)[2], (*thresholds)[0]), Scalar((*thresholds)[5], (*thresholds)[3], (*thresholds)[1]), *imgThresholded); //Threshold the image
}

// function returns center point of color contour drived from input mask
void detectContour(Mat *mask,Point2f *center)
{
	vector<vector<Point> > contourPoints;
	//find contour pints in mask
	findContours(*mask,contourPoints ,CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
	int largest_area=0;
	int largest_contour_index=-1;
	
	for( unsigned int i = 0; i< contourPoints.size(); i++ )
	{
		float area = contourArea( contourPoints[i] );
			//Find the area of contour
			if( area > largest_area )
			{
				largest_area = area;
				//Store the index of largest contour
				largest_contour_index = i;
			}
	}
	if(largest_contour_index >= 0)
	{
			//calculate minimum rotated Bounding rect
	RotatedRect minRect = minAreaRect(Mat(contourPoints[largest_contour_index]));
	*center= minRect.center;	
		}
	else
	{
		*center= Point(0,0);
		}
}

void getCameraFrame(circular_buffer<Mat> *buff,raspicam::RaspiCam_Cv *Camera )
{while(true){
	bool grabbed=Camera->grab();
	if(!grabbed)
	{
		cout<<"could not grab frame from camera"<<endl;
	}
	Mat imgOriginal;
	Camera->retrieve(imgOriginal);
	buff->put(imgOriginal);   
}
}
void color_detectThread(vector<int> *color_boundary, Mat *imgOriginal, Point2f *contourCenter,const int id)
{
	//TODO CHECK INPUT VARS
	Mat imgThresholded;
	//0.4-0.9 ms
	detect_color(imgOriginal, color_boundary, &imgThresholded);
	//1-3 ms
	image_optimizations(&imgThresholded);
	//0.9-1.4 ms
	detectContour(&imgThresholded, contourCenter);
	cout<<id<<"|"<<contourCenter->x<<"|"<<contourCenter->y<<endl;
	TaskComplete();
}

int main()
{
	//preallocate Images for originaln and converted frames
	Mat imgOriginal;
	Mat imgHSV;
	vector<Point2f> *contourCenters=new vector<Point2f>(5);
	boost::asio::io_service udp_io_service;
	const string host="192.168.1.10";
	const string port= "8888"; 
	UDPClient client(udp_io_service,host,port);
	

	vector<vector<int> > colorThreshold(5);
	//vector structure RL,RH,GL,GH,BL,BH
	//blue
	colorThreshold[0]={68,122,136,165,118,255};
	//orange
	colorThreshold[1]={0,255,152,174,45,87};
	//green
	colorThreshold[2]={130,172,163,177,38,140};
	//pink
	colorThreshold[3]={0,255,117,130,144,180};
	//yellow
	colorThreshold[4]={181,255,161,255,40,165};

	raspicam::RaspiCam_Cv *Camera = new raspicam::RaspiCam_Cv;
	Camera->set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera->set(CV_CAP_PROP_FRAME_WIDTH,640);
	Camera->set(CV_CAP_PROP_FRAME_HEIGHT,480);
	circular_buffer<Mat> *img_buffer= new circular_buffer<Mat>(3);
	
	
	if ( !Camera->open()) 
	{
		cout << "Cannot open the web cam" << endl;
	}
	cout<<"camera warmup phase started"<<endl;
	sleep(5); 
	cout<<"warmup finished"<<endl;
	
	//auto start= chrono::high_resolution_clock::now();
	//auto finish= chrono::high_resolution_clock::now();
	//chrono::duration<double> elapsed= finish -start;
	//cout<<elapsed.count()<<endl;

	//setup thread for camera frame read
	boost::thread cameraThread(getCameraFrame,img_buffer,Camera);
	cout<<"waiting for buffer to fill"<<endl;
	sleep(5); 
	//cameraThread.detach();
	//setup threadpool
	boost::asio::io_service ioService;
	boost::thread_group threadpool;
	//start processing
	boost::asio::io_service::work work(ioService);
	//create  worker threads
	for(unsigned int i=0;i<4;i++)
	{
		threadpool.create_thread(boost::bind(&boost::asio::io_service::run,&ioService));
	}
	
	while(true){
	//read frame from cyclic buffer
	imgOriginal= img_buffer->get();
	cout<<imgOriginal<<endl;
	//Check if buffer contains frames
	cout<<imgOriginal.cols<<"|"<<imgOriginal.rows<<endl;
	if(imgOriginal.cols>0 && imgOriginal.rows >0){
		imshow( "Display window", imgOriginal );  
		for(unsigned int k=0;k < colorThreshold.size(); k++)
	{
		ioService.post(boost::bind(color_detectThread,(&colorThreshold[k]),&imgOriginal,(&(*contourCenters).at(k)),k));
		number_of_tasks++;
	}
	cout<<"wait"<<endl;
	wait_for_pool();
	cout<<"pool finished"<<endl;
	//
	stringstream ss;
	ss<<"hello world "<<"123456 "<<"moep";
	const string s=ss.str();
	client.send(s);
	cout<<s<<endl;
	namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
	
	}
	else
	{
		cout<<"waiting for buffer"<<endl;
	}
	// Wait for a keystroke in the window
	if((char)27==(char)waitKey(1)){
		Camera->release();
		delete Camera;
		delete img_buffer;
		ioService.stop();
		threadpool.join_all();
		return 0;} 
	//TODO wait for key inpout when no images are shown
	//Camera.release();
	//ioService.stop();
	//threadpool.join_all();
}
}



