// Generated by gencpp from file opencv_work/ImageSrv.msg
// DO NOT EDIT!


#ifndef OPENCV_WORK_MESSAGE_IMAGESRV_H
#define OPENCV_WORK_MESSAGE_IMAGESRV_H

#include <ros/service_traits.h>


#include <opencv_work/ImageSrvRequest.h>
#include <opencv_work/ImageSrvResponse.h>


namespace opencv_work
{

struct ImageSrv
{

typedef ImageSrvRequest Request;
typedef ImageSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ImageSrv
} // namespace opencv_work


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::opencv_work::ImageSrv > {
  static const char* value()
  {
    return "ba55116f263d40ea8759822097ad63d4";
  }

  static const char* value(const ::opencv_work::ImageSrv&) { return value(); }
};

template<>
struct DataType< ::opencv_work::ImageSrv > {
  static const char* value()
  {
    return "opencv_work/ImageSrv";
  }

  static const char* value(const ::opencv_work::ImageSrv&) { return value(); }
};


// service_traits::MD5Sum< ::opencv_work::ImageSrvRequest> should match 
// service_traits::MD5Sum< ::opencv_work::ImageSrv > 
template<>
struct MD5Sum< ::opencv_work::ImageSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::opencv_work::ImageSrv >::value();
  }
  static const char* value(const ::opencv_work::ImageSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::opencv_work::ImageSrvRequest> should match 
// service_traits::DataType< ::opencv_work::ImageSrv > 
template<>
struct DataType< ::opencv_work::ImageSrvRequest>
{
  static const char* value()
  {
    return DataType< ::opencv_work::ImageSrv >::value();
  }
  static const char* value(const ::opencv_work::ImageSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::opencv_work::ImageSrvResponse> should match 
// service_traits::MD5Sum< ::opencv_work::ImageSrv > 
template<>
struct MD5Sum< ::opencv_work::ImageSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::opencv_work::ImageSrv >::value();
  }
  static const char* value(const ::opencv_work::ImageSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::opencv_work::ImageSrvResponse> should match 
// service_traits::DataType< ::opencv_work::ImageSrv > 
template<>
struct DataType< ::opencv_work::ImageSrvResponse>
{
  static const char* value()
  {
    return DataType< ::opencv_work::ImageSrv >::value();
  }
  static const char* value(const ::opencv_work::ImageSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPENCV_WORK_MESSAGE_IMAGESRV_H
