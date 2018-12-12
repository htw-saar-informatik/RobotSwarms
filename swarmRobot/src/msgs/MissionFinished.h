// Generated by gencpp from file swarmRobot/MissionFinished.msg
// DO NOT EDIT!


#ifndef SWARMROBOT_MESSAGE_MISSIONFINISHED_H
#define SWARMROBOT_MESSAGE_MISSIONFINISHED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace swarmRobot
{
template <class ContainerAllocator>
struct MissionFinished_
{
  typedef MissionFinished_<ContainerAllocator> Type;

  MissionFinished_()
    : index(0)  {
    }
  MissionFinished_(const ContainerAllocator& _alloc)
    : index(0)  {
  (void)_alloc;
    }



   typedef uint8_t _index_type;
  _index_type index;





  typedef boost::shared_ptr< ::swarmRobot::MissionFinished_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::swarmRobot::MissionFinished_<ContainerAllocator> const> ConstPtr;

}; // struct MissionFinished_

typedef ::swarmRobot::MissionFinished_<std::allocator<void> > MissionFinished;

typedef boost::shared_ptr< ::swarmRobot::MissionFinished > MissionFinishedPtr;
typedef boost::shared_ptr< ::swarmRobot::MissionFinished const> MissionFinishedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::swarmRobot::MissionFinished_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::swarmRobot::MissionFinished_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace swarmRobot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/lunar/share/std_msgs/cmake/../msg'], 'swarmRobot': ['/home/smanier/catkin_ws/src/swarmRobot/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::swarmRobot::MissionFinished_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::swarmRobot::MissionFinished_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swarmRobot::MissionFinished_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swarmRobot::MissionFinished_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarmRobot::MissionFinished_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarmRobot::MissionFinished_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::swarmRobot::MissionFinished_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9865c521c6f40dd504cfcb9a4dfb1268";
  }

  static const char* value(const ::swarmRobot::MissionFinished_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9865c521c6f40dd5ULL;
  static const uint64_t static_value2 = 0x04cfcb9a4dfb1268ULL;
};

template<class ContainerAllocator>
struct DataType< ::swarmRobot::MissionFinished_<ContainerAllocator> >
{
  static const char* value()
  {
    return "swarmRobot/MissionFinished";
  }

  static const char* value(const ::swarmRobot::MissionFinished_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::swarmRobot::MissionFinished_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 index\n\
";
  }

  static const char* value(const ::swarmRobot::MissionFinished_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::swarmRobot::MissionFinished_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MissionFinished_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::swarmRobot::MissionFinished_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::swarmRobot::MissionFinished_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.index);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SWARMROBOT_MESSAGE_MISSIONFINISHED_H
