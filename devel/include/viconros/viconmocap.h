// Generated by gencpp from file viconros/viconmocap.msg
// DO NOT EDIT!


#ifndef VICONROS_MESSAGE_VICONMOCAP_H
#define VICONROS_MESSAGE_VICONMOCAP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace viconros
{
template <class ContainerAllocator>
struct viconmocap_
{
  typedef viconmocap_<ContainerAllocator> Type;

  viconmocap_()
    : header()
    , time(0.0)
    , position()
    , velocity()  {
    }
  viconmocap_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time(0.0)
    , position(_alloc)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _time_type;
  _time_type time;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::viconros::viconmocap_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::viconros::viconmocap_<ContainerAllocator> const> ConstPtr;

}; // struct viconmocap_

typedef ::viconros::viconmocap_<std::allocator<void> > viconmocap;

typedef boost::shared_ptr< ::viconros::viconmocap > viconmocapPtr;
typedef boost::shared_ptr< ::viconros::viconmocap const> viconmocapConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::viconros::viconmocap_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::viconros::viconmocap_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace viconros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'viconros': ['/home/uav/lzy_ws/src/viconros/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::viconros::viconmocap_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::viconros::viconmocap_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::viconros::viconmocap_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::viconros::viconmocap_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::viconros::viconmocap_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::viconros::viconmocap_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::viconros::viconmocap_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6a740ba07c044cc42dccfe8fbd04f4e5";
  }

  static const char* value(const ::viconros::viconmocap_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6a740ba07c044cc4ULL;
  static const uint64_t static_value2 = 0x2dccfe8fbd04f4e5ULL;
};

template<class ContainerAllocator>
struct DataType< ::viconros::viconmocap_<ContainerAllocator> >
{
  static const char* value()
  {
    return "viconros/viconmocap";
  }

  static const char* value(const ::viconros::viconmocap_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::viconros::viconmocap_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64 time\n"
"geometry_msgs/Vector3 position\n"
"geometry_msgs/Vector3 velocity\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::viconros::viconmocap_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::viconros::viconmocap_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time);
      stream.next(m.position);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct viconmocap_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::viconros::viconmocap_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::viconros::viconmocap_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VICONROS_MESSAGE_VICONMOCAP_H
