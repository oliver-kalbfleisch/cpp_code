#include "main.h"
using namespace cv;
using namespace std;

const int numThreads=4;
const unsigned short frame_width=640;
const unsigned short frame_height=480;
const unsigned int x_offset= 30;
const unsigned int y_offset= 30;
//TODO solve via console input
// Set correct value for camera -Left:8888 -Right :9999
const string host="192.168.1.10";
const string port= "9999";
const string multicast_port="6666";
mutex detector_mutex;
int number_of_tasks;
boost::mutex task_count_mutex;
boost::condition_variable task_cv;
enum {max_length=1024};
char data_[max_length];
clock_t t;
stringstream ss;


//color calibration values
int low_r=30;
int low_g=30;
int low_b=30;

int high_r = 100;
int high_g = 100;
int high_b = 100;

void TaskComplete()
{
    boost::unique_lock<boost::mutex> lock(task_count_mutex);
    --number_of_tasks;
    if(number_of_tasks==0) {
        task_cv.notify_one();
    }
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
    GaussianBlur(*imgThresholded,*imgThresholded,Size(3,3),0,0);
    erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)) );
    dilate(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)) );//morphological closing (fill small holes in the foreground)

    dilate( *imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)) );
    erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)) );
}
//function takes in HSV image and color boundaries for detection
void detect_color(Mat image,vector<int> *thresholds,Mat *imgThresholded)
{
    //FOR DEGUB USE ONLY, REMOVE FOR RELEASE
    //imshow("threshold image",image);
    //--------------------------------------
    try {
        inRange(image, Scalar((*thresholds)[4],(*thresholds)[2], (*thresholds)[0]), Scalar((*thresholds)[5], (*thresholds)[3], (*thresholds)[1]), *imgThresholded);
    } //Threshold the image
    catch(...)
    {
        cout<<"exception in inRange"<<endl;
    }


}

// function returns center point of color contour drived from input mask
void detectContour(Mat *mask,Point2i *center, Rect *roi,Point2i *offset)
{
    //TODO CATCH CASE OF NO COLOR TO TRACK PRESENT
    vector<vector<Point> > contourPoints;
    //find contour pints in mask
    findContours(*mask,contourPoints,CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
    unsigned int largest_area=0;
    int largest_contour_index=-1;

    for( unsigned int i = 0; i< contourPoints.size(); i++ )
    {   //Find the area of contour
        float area = contourArea( contourPoints[i] );

        if( area > largest_area )
        {
            largest_area = area;
            //Store the index of largest contour
            largest_contour_index = i;
        }
    }
    if(largest_contour_index >= 0)
    {
        //calculate minimum rotated bounding rect
        RotatedRect minRect = minAreaRect(Mat(contourPoints[largest_contour_index]));
        Rect bRect=minRect.boundingRect();

        /* ROI AREA CALULATION
        # tl------tl+w
        #  +       +
        #  +       +
        #  +       +
        #  tl+h-----br
        */
        unsigned int x0=boost::algorithm::clamp((offset->x+bRect.tl().x-x_offset),0,frame_width);
        unsigned int y0=boost::algorithm::clamp((offset->y+bRect.tl().y-y_offset),0,frame_height);

        unsigned int corr=0;
        unsigned int pos_right=x0+bRect.size().width+(2*(x_offset));
        if(pos_right > frame_width)
        {
            corr=pos_right-frame_width;
        }
        unsigned int width_clamped=boost::algorithm::clamp(bRect.size().width+(2*(x_offset))-corr,0,frame_width);
        unsigned int pos_bottom=y0+bRect.size().height+(2*(y_offset));
        if(pos_bottom > frame_height)
        {
            corr=pos_bottom-frame_height;
        }
        unsigned int height_clamped=boost::algorithm::clamp(bRect.size().height+(2*(y_offset))-corr,0,frame_height);

        *roi=Rect(x0,y0,width_clamped,height_clamped);
        *offset=Point2i(x0,y0);
        *center= minRect.center;

    }
    else
    {   //TODO DEBUG !!!
        *center= *center+*offset;
        *offset=Point2i(0,0);
        *roi=Rect(0,0,frame_width,frame_height);
    }
}

void getCameraFrame(circular_buffer<Mat> *buff,raspicam::RaspiCam_Cv *Camera )
{
    Mat imgOriginal;
    while(true) {
        bool grabbed=Camera->grab();
        if(!grabbed)
        {
            cout<<"could not grab frame from camera"<<endl;
        }
        Camera->retrieve(imgOriginal);
        buff->put(imgOriginal);
    }
}
void color_detectThread(vector<int> &color_boundary, Mat imgOriginal, Point2i &contourCenter,Rect &roi, Point2i &offset)
{
    Mat imgThresholded;
    detect_color(imgOriginal,&color_boundary, &imgThresholded);
    image_optimizations(&imgThresholded);
    detectContour(&imgThresholded, &contourCenter,&roi,&offset);
    //MULTITHREAD IMPLEMENTAION
    //TaskComplete();
}
void low_r_thresh(int, void *)
{
    low_r=min(high_r-1,low_r);
    setTrackbarPos("Low R","ObjectDetection",low_r);
}
void high_r_thresh(int,void *)
{
    high_r=max(high_r,low_r+1);
    setTrackbarPos("High R","ObjectDetection",high_r);
}
void low_g_thresh(int, void *)
{
    low_g=min(high_g-1,low_g);
    setTrackbarPos("Low G","ObjectDetection",low_g);
}
void high_g_thresh(int,void *)
{
    high_g=max(high_g,low_g+1);
    setTrackbarPos("High G","ObjectDetection",high_g);
}
void low_b_thresh(int, void *)
{
    low_b=min(high_b-1,low_b);
    setTrackbarPos("Low B","ObjectDetection",low_b);
}
void high_b_thresh(int,void *)
{
    high_b=max(high_b,low_b+1);
    setTrackbarPos("High B","ObjectDetection",high_b);
}
void callbackButton(int state,void* userdata)
{
    cout<<"button pressed!"<<state<<userdata<<endl;
}
void setupColorCalibration()
{
    namedWindow("ObjectDetection",WINDOW_NORMAL);
    //createButton("Set Value",callbackButton);
    createTrackbar("Low R","ObjectDetection",&low_r,255,low_r_thresh);
    createTrackbar("High R","ObjectDetection",&high_r,255,high_r_thresh);
    createTrackbar("Low G","ObjectDetection",&low_g,255,low_g_thresh);
    createTrackbar("High G","ObjectDetection",&high_g,255,high_g_thresh);
    createTrackbar("Low B","ObjectDetection",&low_b,255,low_b_thresh);
    createTrackbar("High B","ObjectDetection",&high_b,255,high_b_thresh);
}
int main()
{
    cout<<"****RHOT Realtim Hand and Object Tracker v0.1****"<<endl;
    cout<<"*"<<endl;
    cout<<"*System is setting up starting values..."<<endl;
    cout<<"*"<<endl;
    //preallocate Images for original and converted frames
    Mat *imgOriginal (new Mat());
    vector<Point2i> *contourCenters=new vector<Point2i>(5);
    boost::asio::io_service udp_io_service;
    boost::asio::io_service mcService;
    UDPClient client(udp_io_service,host,port);
    vector<Rect> *imageROIS =new vector<Rect>(5);
    vector<Point2i>*offsets= new vector<Point2i>(5);
    for(unsigned int i=0; i<imageROIS->size(); i++)
    {
        imageROIS-> at(i)= Rect(0,0,640,480);
        offsets  -> at(i)= Point2i(0,0);
    }

    //Setup Camera
    raspicam::RaspiCam_Cv *Camera = new raspicam::RaspiCam_Cv;
    Camera->set(CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera->set(CV_CAP_PROP_FRAME_WIDTH,frame_width);
    Camera->set(CV_CAP_PROP_FRAME_HEIGHT,frame_height);
    //Camera->set(CV_CAP_PROP_EXPOSURE,100);
    Camera->set(CV_CAP_PROP_GAIN,50);
    //Camera->set(CV_CAP_PROP_WHITE_BALANCE_RED_V,1);
    //Camera->set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U,1);


    //circular_buffer<Mat> *img_buffer= new circular_buffer<Mat>(5);
    vector<vector<int> > *colorThreshold=new vector<vector<int> >(5);
	cout<<"T*he system has a set of predefined values for tracking colors.\n You can also switch to color calibration mode which does require a monitor to be conected."<<endl;
	cout<<"*"<<endl;
    cout<<"*Do you want to continue with the default color values for 5 markers? (y/n)  ";
    char response;
    cin>>response;
    switch (response) {
    case 'y': {
        // Initialize default color Threshold values
        //vector structure RL,RH,GL,GH,BL,BH
        //blue
        (*colorThreshold)[0]= {91,138,160,178,166,193};
        //orange
        (*colorThreshold)[1]= {203,221,152,178,82,108};
        //green
        (*colorThreshold)[2]= {163,208,176,216,0,116};
        //pink
        (*colorThreshold)[3]= {214,255,138,176,162,195};
        //yellow
        (*colorThreshold)[4]= {205,255,195,255,29,109};
        cout<<"*"<<endl;
        cout<<"*System will utilize default values."<<endl;
        cout<<"*"<<endl;
        break;
    }
    case 'n': {
		cout<<"*"<<endl;
        cout<<"*Caution: The color calibration mode cannot be run in headless mode\n and requires a monitor to be attached to the Raspberry"<<endl;
        cout<<"*"<<endl;
        cout<<"*Do you want to continue? (y/n) ";
        char resp;
        cin>>resp;
           switch(resp)
        {
        case 'y': {
            //TODO Add Qt library support
            cout<<"*"<<endl;
            cout<<"*System will now switch to color calibration mode."<<endl;
            setupColorCalibration();
            if ( !Camera->open())
            {
                cout<<"*"<<endl;
                cout << "Cannot open the web cam" << endl;
                return -1;
            }
            bool calibrating=true;
            while(calibrating)
            {

                bool grabbed=Camera->grab();
                if(grabbed)
                {
                    try {
                        Camera->retrieve(*imgOriginal);
                    } catch(...)
                    {
                        cout<<"*"<<endl;
                        cout<<"*Camera retrieve error"<<endl;
                    }
                }
                Mat res;
                inRange(*imgOriginal,Scalar(low_b,low_g,low_r),Scalar(high_b,high_g,high_r),res);
                imshow("ObjectDetection",res);
                if((char)27==(char)waitKey(1)) {
                    Camera->release();
                    destroyAllWindows();
                    break;
                }
            }
            break;
        }
        case 'n': {
            cout<<"*Aborting..."<<endl;
            return 0;
        }
        }

    }
    }

    if ( !Camera->open())
    {
        cout<<"*"<<endl;
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    //setup thread for camera frame read
    //TODO deamonize camera thread
    //cameraThread.detach();
    //setup threadpool
    //boost::asio::io_service ioService;
    //boost::thread_group threadpool;
    //start processingimageROIS
    // boost::asio::io_service::work work(ioService);
    /*for(unsigned int i=0; i<3; i++)
        {
            threadpool.create_thread(boost::bind(&boost::asio::io_service::run,&ioService));
        }
        * */
    cout<<"*"<<endl;
    cout<<"*Camera warmup phase started"<<endl;
    sleep(5);
    cout<<"*"<<endl;
    cout<<"*Warmup finished"<<endl;
    //CAMERA THREAD
    //boost::thread cameraThread(getCameraFrame,img_buffer,Camera);
    //cout<<"waiting for buffer to fill"<<endl;
    //sleep(5);
    cout<<"*"<<endl;
    cout<<"*--->Starting processing"<<endl;
    cout<<"*"<<endl;
    cout<<"*System will be sending UDP Data to destination address: "<<host<<" to Port "<<port<<endl;
    //Setup wait for start signal
    cout<<"*"<<endl;
    cout<<"*System currently waiting for Master start signal..."<<endl;
    MulticastReciever mcRecv(mcService);
    cout<<"*"<<endl;
    mcService.run();
    mcService.stop();

    while(true) {
        //read frame from cyclic buffer
        //timimg ~0,3 ms
        t=clock();
        bool grabbed=Camera->grab();
        if(grabbed)
        {
            try {
                Camera->retrieve(*imgOriginal);
            } catch(...)
            {
                cout<<"*"<<endl;
                cout<<"*Camera retrieve error"<<endl;
            }
            //CAMERATHREAD + IMG_BUFFER
            //if(img_buffer->empty()==false)
            //{
            //Mat test=img_buffer->get().clone();
            //imgOriginal= &test;


            //Check if buffer contains frames
            //if(imgOriginal->cols>0 && imgOriginal->rows >0)
            //{
            //



            //MULTITHREAD IMPLEMENTAION
            //wait_for_pool();
            t=clock();
            for(unsigned int k=0; k<5; k++)
            {
                //TODO calculation of correct roi coordinates
                Mat cropped;
                try {
                    Mat cropped (*imgOriginal,imageROIS->at(k));
                    color_detectThread((*colorThreshold)[k],cropped,(*contourCenters)[k],imageROIS->at(k),offsets->at(k));
                }
                catch(Exception)
                {
                    cout<<"*"<<endl;
                    cout<<"*Could not retrieve roi from sourceimage"<<endl;
                    imageROIS->at(k)=Rect(0,0,frame_width,frame_height);
                }

                //MULTITHREAD IMPLEMENTAION //ioService.post(boost::bind(color_detectThread,boost::ref((*colorThreshold)[k]),cropped,boost::ref((*contourCenters)[k]),boost::ref(imageROIS->at(k)),boost::ref(offsets->at(k))));
                //number_of_tasks++;
            }
            //MULTITHREAD IMPLEMENTAION
            //cout<<"waiting for pool"<<endl;
            //
            for(unsigned int i=0; i<(*contourCenters).size(); i++)
            {
                //ADD IN FOR UDP
                ss<<((*contourCenters).at(i)+offsets->at(i))<<";";
                //DEBUG
                //cout<<(*contourCenters).at(i)<<"|"<<offsets->at(i)<<endl;
                circle(*imgOriginal,((*contourCenters).at(i)+offsets->at(i)),5,Scalar(0,0,255),-1);
                rectangle(*imgOriginal,imageROIS->at(i),Scalar(255,0,0));
                //DEBUG
            }
            //UDP
            client.send(ss.str());
            ss.str(string());
            //UDP
            t=clock()-t;
            printf("%f seconds\n",((float)t)/CLOCKS_PER_SEC);
            //DEBUG
            //imshow("image",*imgOriginal);
            //DEBUG
            //usleep(30);
        }
        else
        {
            cout<<"could not grab frame from camera"<<endl;

            //usleep(300);
        }
        //Wait for a keystroke in the window
        if((char)27==(char)waitKey(1)) {
            //delete img_buffer;
            //ioService.stop();
            //threadpool.join_all();
            destroyAllWindows();
            Camera->release();
            delete Camera;
            delete contourCenters;
            delete colorThreshold;
            delete imageROIS;
            delete offsets;

            return 0;
        }

    }
    Camera->release();
    //delete img_buffer;
    //ioService.stop();
    //threadpool.join_all();
    destroyAllWindows();
    delete Camera;
    delete contourCenters;
    delete colorThreshold;
    delete imageROIS;
    delete offsets;

    return 0;
}



