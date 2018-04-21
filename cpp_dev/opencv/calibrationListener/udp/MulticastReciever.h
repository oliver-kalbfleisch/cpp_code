#ifndef MULTICAST_RECIEVER_H
#define MULTICAST_RECIEVER_H
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
class MulticastReciever
{
	private:
	boost::asio::ip::udp::socket socket_;
	boost::asio::ip::udp::endpoint sender_endpoint_;
	enum { max_length=1024};
	char data_[max_length];
	int* counter_;
	std::string side_;
	cv::Mat *imgOriginal_;
	
	
	public:
	MulticastReciever(boost::asio::io_service& io_service,int* counter,std::string side,cv::Mat *imgOriginal);
	void handle_recieve_from(const boost::system::error_code& error, size_t bytes_recvd);
	};
#endif
