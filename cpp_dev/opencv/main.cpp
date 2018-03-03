#include "main.h"
using namespace cv;
using namespace std;

const int numThreads=4;
const unsigned short frame_width=640;
const unsigned short frame_height=480;
const unsigned int x_offset= 30;
const unsigned int y_offset= 30;
mutex detector_mutex;
int number_of_tasks;
boost::mutex task_count_mutex;
boost::condition_variable task_cv;
clock_t t;

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
		task_cv.wait(lock);
	}
	}


//function blurrs and erodes/dilliates the input image to remove high frequncy nose in the image
void image_optimizations(Mat *imgThresholded)
{
	//morphological opening (remove small objects from the foreground)
	erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)) );
	dilate(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)) );
}
//function takes in HSV image and color boundaries for detection
void detect_color(Mat *image,vector<int> *thresholds,Mat *imgThresholded)
{ 
inRange(*image, Scalar((*thresholds)[4],(*thresholds)[2], (*thresholds)[0]), Scalar((*thresholds)[5], (*thresholds)[3], (*thresholds)[1]), *imgThresholded); //Threshold the image
}

// function returns center point of color contour drived from input mask
void detectContour(Mat *mask,Point2i *center, Rect *roi)
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
	Rect bRect=minRect.boundingRect();
	//TODO generate corner points for roi rect
	cout<<"prev roi :"<<*roi<<endl;
	//TODO Calculate Rect
	/*
	 * # tl------tl+w
    #  +       +
    #  +       +
    #  +       +
    #  tl+h-----br
	 */
	unsigned int x0=boost::algorithm::clamp((bRect.tl().x-x_offset),0,frame_width);
	unsigned int y0=boost::algorithm::clamp((bRect.tl().y-y_offset),0,frame_height);
	
	//cout<<"bRect size "<<bRect.size()<<endl;
	cout<<"center "<<minRect.center<<endl;
	//TODO limit roi area to frame boundaries
	unsigned int corr=0;
	unsigned int pos_right=x0+bRect.size().width+(2*(x_offset));
	cout<<"pos right :"<< pos_right<<endl;
	if(pos_right > frame_width)
	{
		
		corr=pos_right-frame_width;
		cout<<"Xcorr: "<<corr<<endl;
	}
	unsigned int width_clamped=boost::algorithm::clamp(bRect.size().width+(2*(x_offset))-corr,0,frame_width);
	//cout<<"width clamped "<<width_clamped<<endl;
	unsigned int pos_bottom=y0+bRect.size().height+(2*(y_offset));
	cout<<"pos bottom "<<pos_bottom<<endl;
	if(pos_right > frame_height)
	{
		
		corr=pos_bottom-frame_height;
		cout<<"Ycorr: "<<corr<<endl;
	}
	//cout<<"width clamped:"<<width_clamped<<"|"<<"brect width"<<bRect.size().width<<endl;
	unsigned int height_clamped=boost::algorithm::clamp(bRect.size().height+(2*(y_offset))-corr,0,frame_height);
	//cout<<"height clamped:"<<height_clamped<<"|"<<"brect width"<<bRect.size().height<<endl;
	//*roi=
	cout<<"calulated roi:"<<Rect(x0,y0,width_clamped,height_clamped)<<endl;
	*center= minRect.center;	
		}
	else
	{
		*center= Point(0,0);
		}
}

void getCameraFrame(circular_buffer<Mat> *buff,raspicam::RaspiCam_Cv *Camera )
{
	Mat imgOriginal;
	while(true){
	bool grabbed=Camera->grab();
	if(!grabbed)
	{
		cout<<"could not grab frame from camera"<<endl;
	}
	Camera->retrieve(imgOriginal);
	buff->put(imgOriginal);   
}
}
void color_detectThread(vector<int> color_boundary, Mat imgOriginal, Point2i contourCenter,Rect *roi)
{
	//Thread run 110ms
	//0,01ms
	Mat imgThresholded;
	//8-12 ms
	detect_color(&imgOriginal, &color_boundary, &imgThresholded);
	//4-8 ms
	detectContour(&imgThresholded, &contourCenter,roi);
	//0,02 ms
	TaskComplete();	
}

int main()
{
	//preallocate Images for original and converted frames
	Mat *imgOriginal= new Mat();
	Mat imgHSV;
	vector<Point2i> *contourCenters=new vector<Point2i>(5);
	boost::asio::io_service udp_io_service;
	const string host="192.168.1.10";
	const string port= "8888"; 
	UDPClient client(udp_io_service,host,port);
	vector<vector<int> > *colorThreshold=new vector<vector<int> >(5)  ;
	//vector structure RL,RH,GL,GH,BL,BH
	//blue
	(*colorThreshold)[0]={68,122,136,165,118,255};
	//orange
	(*colorThreshold)[1]={0,255,152,174,45,87};
	//green
	(*colorThreshold)[2]={130,172,163,177,38,140};
	//pink
	(*colorThreshold)[3]={0,255,117,130,144,180};
	//yellow
	(*colorThreshold)[4]={181,255,161,255,40,165};
	vector<Rect> *imageROIS =new vector<Rect>(5);
	for(unsigned int i=0;i<imageROIS->size();i++)
	{
		(*imageROIS)[i]= Rect(0,0,640,480);
	}
	raspicam::RaspiCam_Cv *Camera = new raspicam::RaspiCam_Cv;
	Camera->set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera->set(CV_CAP_PROP_FRAME_WIDTH,frame_width);
	Camera->set(CV_CAP_PROP_FRAME_HEIGHT,frame_height);
	circular_buffer<Mat> *img_buffer= new circular_buffer<Mat>(5);
	
	
	if ( !Camera->open()) 
	{
		cout << "Cannot open the web cam" << endl;
	}
	cout<<"camera warmup phase started"<<endl;
	sleep(5); 
	cout<<"warmup finished"<<endl;
	//setup thread for camera frame read
	boost::thread cameraThread(getCameraFrame,img_buffer,Camera);
	cout<<"waiting for buffer to fill"<<endl;
	sleep(5); 
	//TODO deamonize camera thread
	//cameraThread.detach();
	//setup threadpool
	boost::asio::io_service ioService;
	boost::thread_group threadpool;
	//start processingimageROIS
	boost::asio::io_service::work work(ioService);
	//create  worker threads
	for(unsigned int i=0;i<4;i++)
	{
		threadpool.create_thread(boost::bind(&boost::asio::io_service::run,&ioService));
	}
	
	while(true){
    
   
	//read frame from cyclic buffer
	//timimg ~0,3 ms
	*imgOriginal= img_buffer->get();
	//Check if buffer contains frames
	if(imgOriginal->cols>0 && imgOriginal->rows >0)
		{
			 
			 //time <=0,1 ms
				for(unsigned int k=0;k <1; k++)
			{
				Mat cropped(*imgOriginal,imageROIS->at(k));
				ioService.post(boost::bind(color_detectThread,(*colorThreshold)[k],*imgOriginal,(*contourCenters)[k],(&(*imageROIS)[k])));
				number_of_tasks++;
			}
			t=clock();
			wait_for_pool();
			t=clock()-t;
			printf("%f seconds\n",((float)t)/CLOCKS_PER_SEC);
			//sequential single thread is 80 ms
			//multithread is 150 ms 
			stringstream ss;
			for(unsigned int i=0;i < (*contourCenters).size();i++)
			{
				ss<<(*contourCenters).at(0)<<";";
			}
			
			
			const string s=ss.str();
			client.send(s);
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
	
}
}



