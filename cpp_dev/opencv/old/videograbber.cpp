#include "videograbber.h"

VideoGrabber::VideoGrabber(){
}
VideoGrabber::VideoGrabber(int pBuffLen,VideoCapture *pCap){
capture=pCap;
buffLen=pBuffLen;
frameBuffer.resize(buffLen);
frameLen=200;
}
bool VideoGrabber.putFrameInBuffer(Mat &frame)
{
//calulate buffer positin for new frame
pos =idx%buffLen
//save copy of frame to calculated buffer position 
frameBuffer[pos]=f.clone();
//update frame count
idx++;
}

void VideoGrabber::grabFrameFrom Cam(){
//TODO switch to raspberry pi camera for pi compilation
//init framecounter
idx=0
//create memory for frame
Mat f;

//read out frames
while(true)
{
//get frame from cam
capture->read(f);
putFrameInBuffer(f);
}
vector<Mat> VideoGrabber::getFrameFromBuffer(){
int end=pos;
int st=pos-frameLen;
if(st<0)
   st=0;
//extract a lice from the buffer of size frameLen
vector<Mat> ret(frameBuffer.begin()+st,frameBuffer.begin()+end);
return ret;
}
}


