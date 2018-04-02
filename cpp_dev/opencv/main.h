#ifndef MAIN_H
#define MAIN_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "trackingCore/cyclicBuffer.h"
#include "trackingCore/UDP_Client.hpp"
#include "trackingCore/MulticastReciever.h"

#include <raspicam/raspicam_cv.h>
#include <unistd.h>
#include <time.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>
#include <chrono>
#include <ctime> 
#include <string>
#endif
