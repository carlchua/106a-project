// Generated by gencpp from file lab4_cam/ImageSrv.msg
// DO NOT EDIT!


#ifndef LAB4_CAM_MESSAGE_IMAGESRV_H
#define LAB4_CAM_MESSAGE_IMAGESRV_H

#include <ros/service_traits.h>


#include <lab4_cam/ImageSrvRequest.h>
#include <lab4_cam/ImageSrvResponse.h>


namespace lab4_cam
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
} // namespace lab4_cam


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::lab4_cam::ImageSrv > {
  static const char* value()
  {
    return "ba55116f263d40ea8759822097ad63d4";
  }

  static const char* value(const ::lab4_cam::ImageSrv&) { return value(); }
};

template<>
struct DataType< ::lab4_cam::ImageSrv > {
  static const char* value()
  {
    return "lab4_cam/ImageSrv";
  }

  static const char* value(const ::lab4_cam::ImageSrv&) { return value(); }
};


// service_traits::MD5Sum< ::lab4_cam::ImageSrvRequest> should match
// service_traits::MD5Sum< ::lab4_cam::ImageSrv >
template<>
struct MD5Sum< ::lab4_cam::ImageSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::lab4_cam::ImageSrv >::value();
  }
  static const char* value(const ::lab4_cam::ImageSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::lab4_cam::ImageSrvRequest> should match
// service_traits::DataType< ::lab4_cam::ImageSrv >
template<>
struct DataType< ::lab4_cam::ImageSrvRequest>
{
  static const char* value()
  {
    return DataType< ::lab4_cam::ImageSrv >::value();
  }
  static const char* value(const ::lab4_cam::ImageSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::lab4_cam::ImageSrvResponse> should match
// service_traits::MD5Sum< ::lab4_cam::ImageSrv >
template<>
struct MD5Sum< ::lab4_cam::ImageSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::lab4_cam::ImageSrv >::value();
  }
  static const char* value(const ::lab4_cam::ImageSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::lab4_cam::ImageSrvResponse> should match
// service_traits::DataType< ::lab4_cam::ImageSrv >
template<>
struct DataType< ::lab4_cam::ImageSrvResponse>
{
  static const char* value()
  {
    return DataType< ::lab4_cam::ImageSrv >::value();
  }
  static const char* value(const ::lab4_cam::ImageSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LAB4_CAM_MESSAGE_IMAGESRV_H
