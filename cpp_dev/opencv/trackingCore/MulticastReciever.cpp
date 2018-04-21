#include "MulticastReciever.h"

//Class constructor
MulticastReciever::MulticastReciever(boost::asio::io_service& io_service): socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),6666))
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

        //socket_.async_receive_from(	//boost::asio::buffer(data_,max_length),sender_endpoint_,boost::bind(&MulticastReciever::handle_recieve_from,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
    }
}
