#include "calibrationListener.hpp"
using namespace cv;
using namespace std;

const unsigned short frame_width=640;
const unsigned short frame_height=480;
const string multicast_port="6666";

int initialize_Camera(raspicam::RaspiCam_Cv *Camera)
{
    //Set color and image Format
    Camera->set(CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera->set(CV_CAP_PROP_FRAME_WIDTH,frame_width);
    Camera->set(CV_CAP_PROP_FRAME_HEIGHT,frame_height);

    if ( !Camera->open())
    {
        cout<<"*"<<endl;
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    return 0;
}

int main()
{
    int counter=1;
    string side="right";
    raspicam::RaspiCam_Cv *Camera = new raspicam::RaspiCam_Cv;
    boost::asio::io_service mcService;
    Mat *imgOriginal (new Mat());

    //TODO Cout
    initialize_Camera(Camera);
    boost::shared_ptr<boost::asio::io_service::work> work(new boost::asio::io_service::work(mcService));
 
    MulticastReciever mcRecv(mcService,&counter,side,imgOriginal);
    boost::thread t(boost::bind(&boost::asio::io_service::run,&mcService));
	//t.detach();
    while(true) {
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
            imshow("image",*imgOriginal);
            if((char)27==(char)waitKey(1)) {
                destroyAllWindows();
                Camera->release();
                delete Camera;
                mcService.stop();
                return 0;
            }


        }
    }
}
