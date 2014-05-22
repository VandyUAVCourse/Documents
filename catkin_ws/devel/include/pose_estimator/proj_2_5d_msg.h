/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/UAVTeam/catkin_ws/src/pose_estimator/msg/proj_2_5d_msg.msg
 *
 */


#ifndef POSE_ESTIMATOR_MESSAGE_PROJ_2_5D_MSG_H
#define POSE_ESTIMATOR_MESSAGE_PROJ_2_5D_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pose_estimator/point2d_t.h>
#include <pose_estimator/point2d_t.h>

namespace pose_estimator
{
template <class ContainerAllocator>
struct proj_2_5d_msg_
{
  typedef proj_2_5d_msg_<ContainerAllocator> Type;

  proj_2_5d_msg_()
    : refPoints()
    , targetPoints()  {
    }
  proj_2_5d_msg_(const ContainerAllocator& _alloc)
    : refPoints(_alloc)
    , targetPoints(_alloc)  {
    }



   typedef std::vector< ::pose_estimator::point2d_t_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pose_estimator::point2d_t_<ContainerAllocator> >::other >  _refPoints_type;
  _refPoints_type refPoints;

   typedef std::vector< ::pose_estimator::point2d_t_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pose_estimator::point2d_t_<ContainerAllocator> >::other >  _targetPoints_type;
  _targetPoints_type targetPoints;




  typedef boost::shared_ptr< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct proj_2_5d_msg_

typedef ::pose_estimator::proj_2_5d_msg_<std::allocator<void> > proj_2_5d_msg;

typedef boost::shared_ptr< ::pose_estimator::proj_2_5d_msg > proj_2_5d_msgPtr;
typedef boost::shared_ptr< ::pose_estimator::proj_2_5d_msg const> proj_2_5d_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pose_estimator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'pose_estimator': ['/home/UAVTeam/catkin_ws/src/pose_estimator/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "79c33411ebcb4d92753eedd76f986950";
  }

  static const char* value(const ::pose_estimator::proj_2_5d_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x79c33411ebcb4d92ULL;
  static const uint64_t static_value2 = 0x753eedd76f986950ULL;
};

template<class ContainerAllocator>
struct DataType< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pose_estimator/proj_2_5d_msg";
  }

  static const char* value(const ::pose_estimator::proj_2_5d_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Reference points\n\
point2d_t[] refPoints\n\
\n\
# Target points\n\
point2d_t[] targetPoints\n\
\n\
================================================================================\n\
MSG: pose_estimator/point2d_t\n\
float32 x\n\
float32 y\n\
";
  }

  static const char* value(const ::pose_estimator::proj_2_5d_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.refPoints);
      stream.next(m.targetPoints);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct proj_2_5d_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pose_estimator::proj_2_5d_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pose_estimator::proj_2_5d_msg_<ContainerAllocator>& v)
  {
    s << indent << "refPoints[]" << std::endl;
    for (size_t i = 0; i < v.refPoints.size(); ++i)
    {
      s << indent << "  refPoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pose_estimator::point2d_t_<ContainerAllocator> >::stream(s, indent + "    ", v.refPoints[i]);
    }
    s << indent << "targetPoints[]" << std::endl;
    for (size_t i = 0; i < v.targetPoints.size(); ++i)
    {
      s << indent << "  targetPoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pose_estimator::point2d_t_<ContainerAllocator> >::stream(s, indent + "    ", v.targetPoints[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // POSE_ESTIMATOR_MESSAGE_PROJ_2_5D_MSG_H