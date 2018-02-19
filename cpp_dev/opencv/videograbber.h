#ifndef __IOSTREAM__H__
#define __IOSTREAM_H__
#include <iostream>
#endif
#ifndef __THREAD_H__
#define __THREAD_H__
#include <thread>
#endif
#include "opencvincludes.h"


class VideoGrabber{
Mat Buffer;
vector<Mat> frameBuffer;
int buffLen, frameLen;
int pos;
int idx;
bool putFrameInBuffer(Mat &frame)

public:
VideoGrabber();
VideoGrabber(int buffLen,VideoCapture *cap);
virtual vector<Mat> getFrameFromBuffer();
}

