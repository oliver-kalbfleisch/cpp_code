#include "MulticastReciever.h"

//Class constructor
MulticastReciever::MulticastReciever(boost::asio::io_service& io_service,int* counter,std::string side,cv::Mat *imgOriginal): socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),6666)),counter_(counter),side_(side),imgOriginal_(imgOriginal)
{
    socket_.async_receive_from(
        boost::asio::buffer(data_,max_length),sender_endpoint_,boost::bind(&MulticastReciever::handle_recieve_from,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
};
void MulticastReciever::handle_recieve_from(const boost::system::error_code& error, size_t bytes_recvd)
{
    if(!error)
    {
        std::cout<<"*"<<std::endl;
        std::cout<<"Signal";
        std::cout.write(data_,bytes_recvd);
        std::cout<<" from Master recieved, starting tracking!"<<std::endl;
		std::string text = "img_";
		text += std::to_string(*counter_);
		text +="_";
		text +=side_;
		text +=".jpg";
        cv::imwrite( text, *imgOriginal_);
        *counter_=*counter_+1;
        socket_.async_receive_from(	boost::asio::buffer(data_,max_length),sender_endpoint_,boost::bind(&MulticastReciever::handle_recieve_from,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
    }
}
