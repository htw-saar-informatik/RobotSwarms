// Generated by gencpp from file beginner_tutorials/Mission.msg
// DO NOT EDIT!


#ifndef BEGINNER_TUTORIALS_MESSAGE_MISSION_H
#define BEGINNER_TUTORIALS_MESSAGE_MISSION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beginner_tutorials
{
template <class ContainerAllocator>
struct Mission_
{
  typedef Mission_<ContainerAllocator> Type;

  Mission_()
    : index_from(0)
    , index_to(0)  {
    }
  Mission_(const ContainerAllocator& _alloc)
    : index_from(0)
    , index_to(0)  {
  (void)_alloc;
    }



   typedef uint8_t _index_from_type;
  _index_from_type index_from;

   typedef uint8_t _index_to_type;
  _index_to_type index_to;





  typedef boost::shared_ptr< ::beginner_tutorials::Mission_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beginner_tutorials::Mission_<ContainerAllocator> const> ConstPtr;

}; // struct Mission_

typedef ::beginner_tutorials::Mission_<std::allocator<void> > Mission;

typedef boost::shared_ptr< ::beginner_tutorials::Mission > MissionPtr;
typedef boost::shared_ptr< ::beginner_tutorials::Mission const> MissionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::beginner_tutorials::Mission_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::beginner_tutorials::Mission_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace beginner_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'beginner_tutorials': ['/home/smanier/catkin_ws/src/beginner_tutorials/msg'], 'std_msgs': ['/opt/ros/lunar/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::Mission_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::Mission_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::Mission_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::Mission_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::Mission_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::Mission_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::beginner_tutorials::Mission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8fd39787742067dcab8231230367297f";
  }

  static const char* value(const ::beginner_tutorials::Mission_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8fd39787742067dcULL;
  static const uint64_t static_value2 = 0xab8231230367297fULL;
};

template<class ContainerAllocator>
struct DataType< ::beginner_tutorials::Mission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "beginner_tutorials/Mission";
  }

  static const char* value(const ::beginner_tutorials::Mission_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::beginner_tutorials::Mission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 index_from\n\
uint8 index_to\n\
";
  }

  static const char* value(const ::beginner_tutorials::Mission_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::Mission_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index_from);
      stream.next(m.index_to);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Mission_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beginner_tutorials::Mission_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beginner_tutorials::Mission_<ContainerAllocator>& v)
  {
    s << indent << "index_from: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.index_from);
    s << indent << "index_to: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.index_to);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_MISSION_H
