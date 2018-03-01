#include "cameraThread.h"


class CameraThread
{
std::thread & camera_thread
public :
	CameraThread(std::thread & threadObj):  camera_thread()threadobj)
	{

	}
	~CameraThread()
	{
	if(camera_thread.joinable())
		camera_thread.detach();
	}
}
void read_frames(
