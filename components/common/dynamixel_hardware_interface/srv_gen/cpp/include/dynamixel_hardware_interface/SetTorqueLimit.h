/* Auto-generated by genmsg_cpp for file /home/billy/robotics/maggie/components/common/dynamixel_hardware_interface/srv/SetTorqueLimit.srv */
#ifndef DYNAMIXEL_HARDWARE_INTERFACE_SERVICE_SETTORQUELIMIT_H
#define DYNAMIXEL_HARDWARE_INTERFACE_SERVICE_SETTORQUELIMIT_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace dynamixel_hardware_interface
{
template <class ContainerAllocator>
struct SetTorqueLimitRequest_ {
  typedef SetTorqueLimitRequest_<ContainerAllocator> Type;

  SetTorqueLimitRequest_()
  : torque_limit(0.0)
  {
  }

  SetTorqueLimitRequest_(const ContainerAllocator& _alloc)
  : torque_limit(0.0)
  {
  }

  typedef double _torque_limit_type;
  double torque_limit;


  typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTorqueLimitRequest
typedef  ::dynamixel_hardware_interface::SetTorqueLimitRequest_<std::allocator<void> > SetTorqueLimitRequest;

typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitRequest> SetTorqueLimitRequestPtr;
typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitRequest const> SetTorqueLimitRequestConstPtr;


template <class ContainerAllocator>
struct SetTorqueLimitResponse_ {
  typedef SetTorqueLimitResponse_<ContainerAllocator> Type;

  SetTorqueLimitResponse_()
  {
  }

  SetTorqueLimitResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTorqueLimitResponse
typedef  ::dynamixel_hardware_interface::SetTorqueLimitResponse_<std::allocator<void> > SetTorqueLimitResponse;

typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitResponse> SetTorqueLimitResponsePtr;
typedef boost::shared_ptr< ::dynamixel_hardware_interface::SetTorqueLimitResponse const> SetTorqueLimitResponseConstPtr;

struct SetTorqueLimit
{

typedef SetTorqueLimitRequest Request;
typedef SetTorqueLimitResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetTorqueLimit
} // namespace dynamixel_hardware_interface

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7ac67170532bb79d95db2a425915bbd6";
  }

  static const char* value(const  ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x7ac67170532bb79dULL;
  static const uint64_t static_value2 = 0x95db2a425915bbd6ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamixel_hardware_interface/SetTorqueLimitRequest";
  }

  static const char* value(const  ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
float64 torque_limit\n\
\n\
";
  }

  static const char* value(const  ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamixel_hardware_interface/SetTorqueLimitResponse";
  }

  static const char* value(const  ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
";
  }

  static const char* value(const  ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.torque_limit);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTorqueLimitRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTorqueLimitResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<dynamixel_hardware_interface::SetTorqueLimit> {
  static const char* value() 
  {
    return "7ac67170532bb79d95db2a425915bbd6";
  }

  static const char* value(const dynamixel_hardware_interface::SetTorqueLimit&) { return value(); } 
};

template<>
struct DataType<dynamixel_hardware_interface::SetTorqueLimit> {
  static const char* value() 
  {
    return "dynamixel_hardware_interface/SetTorqueLimit";
  }

  static const char* value(const dynamixel_hardware_interface::SetTorqueLimit&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7ac67170532bb79d95db2a425915bbd6";
  }

  static const char* value(const dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamixel_hardware_interface/SetTorqueLimit";
  }

  static const char* value(const dynamixel_hardware_interface::SetTorqueLimitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7ac67170532bb79d95db2a425915bbd6";
  }

  static const char* value(const dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamixel_hardware_interface/SetTorqueLimit";
  }

  static const char* value(const dynamixel_hardware_interface::SetTorqueLimitResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMIXEL_HARDWARE_INTERFACE_SERVICE_SETTORQUELIMIT_H
