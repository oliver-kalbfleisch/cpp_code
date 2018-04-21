#ifndef MULTICAST_RECIEVER_H
#define MULTICAST_RECIEVER_H
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
class MulticastReciever
{
	private:
	boost::asio::ip::udp::socket socket_;
	boost::asio::ip::udp::endpoint sender_endpoint_;
	enum { max_length=1024};
	char data_[max_length];
	
	public:
	MulticastReciever(boost::asio::io_service& io_service);
	void handle_recieve_from(const boost::system::error_code& error, size_t bytes_recvd);
	};
#endif
