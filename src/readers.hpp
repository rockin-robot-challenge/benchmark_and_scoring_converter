/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoCKIn Converter.
 *
 * RoCKIn Converter is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoCKIn Converter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoCKIn Converter.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ROCKIN_CONVERTER__READERS_HPP__
#define __ROCKIN_CONVERTER__READERS_HPP__

#include <string>
#include <boost/lexical_cast.hpp>

#include <audio_common_msgs/AudioData.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include "parser.hpp"
#include "base64.hpp"



using namespace std;



template <typename T> struct read_expect {
  static const yaml_event_type_e TYPE = YAML_SCALAR_EVENT;
};
template <> struct read_expect<audio_common_msgs::AudioData> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<geometry_msgs::Point> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<geometry_msgs::Pose2D> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<geometry_msgs::Pose> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<geometry_msgs::PoseStamped> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<geometry_msgs::Quaternion> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<nav_msgs::Path> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<ros::Time> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<sensor_msgs::Image> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<sensor_msgs::PointCloud2> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<sensor_msgs::PointField> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<std_msgs::Header> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};
template <> struct read_expect<std_msgs::String> {
  static const yaml_event_type_e TYPE = YAML_MAPPING_START_EVENT;
};



template <typename T> void read_sequence_tail (Parser& in, vector<T>& out)
{
  out.clear();
  out.reserve (10 * 1024 * 1024);
  while (in.expect (read_expect<T>::TYPE, YAML_SEQUENCE_END_EVENT)) {
    T elem;
    read_tail (in, elem);
    out.push_back (elem);
  }
}



template <typename T> void read (Parser& in, string const& key, T& out)
{
  in.expect (key);

  in.expect (read_expect<T>::TYPE);
  read_tail (in, out);
}



template <> void read (Parser& in, string const& key, vector<uint8_t>& out)
{
  in.expect (key);

  if (in.expect (YAML_SCALAR_EVENT, YAML_SEQUENCE_START_EVENT)) {
    base64_decode (in.scalar_value(), out);
  }
  else {
    read_sequence_tail (in, out);
  }
}



template <typename T> void read_sequence (Parser& in, string const& key, vector<T>& out)
{
  in.expect (key);

  in.expect (YAML_SEQUENCE_START_EVENT);
  read_sequence_tail (in, out);
}



template <typename T> void read_tail (Parser& in, T& out)
{
  try {
    out = boost::lexical_cast<T> (in.scalar_value());
  }
  catch (exception) {
    throw runtime_error ("At " + in.location() + ": Error converting \"" + in.scalar_value() + "\" to " + typeid (T).name());
  }
}
template <> void read_tail (Parser& in, audio_common_msgs::AudioData& out)
{
  read (in, "data", out.data);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, geometry_msgs::Point& out)
{
  read (in, "x", out.x);
  read (in, "y", out.y);
  read (in, "z", out.z);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, geometry_msgs::Pose2D& out)
{
  read (in, "x", out.x);
  read (in, "y", out.y);
  read (in, "theta", out.theta);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, geometry_msgs::Pose& out)
{
  read (in, "position", out.position);
  read (in, "orientation", out.orientation);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, geometry_msgs::PoseStamped& out)
{
  read (in, "header", out.header);
  read (in, "pose", out.pose);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, geometry_msgs::Quaternion& out)
{
  read (in, "x", out.x);
  read (in, "y", out.y);
  read (in, "z", out.z);
  read (in, "w", out.w);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, nav_msgs::Path& out)
{
  read (in, "header", out.header);
  read_sequence (in, "poses", out.poses);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, ros::Time& out)
{
  read (in, "secs", out.sec);
  read (in, "nsecs", out.nsec);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, sensor_msgs::Image& out)
{
  read (in, "header", out.header);
  read (in, "height", out.height);
  read (in, "width", out.width);
  read (in, "encoding", out.encoding);
  read (in, "is_bigendian", out.is_bigendian);
  read (in, "step", out.step);
  read (in, "data", out.data);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, sensor_msgs::PointCloud2& out)
{
  read (in, "header", out.header);
  read (in, "height", out.height);
  read (in, "width", out.width);
  read_sequence (in, "fields", out.fields);
  read (in, "is_bigendian", out.is_bigendian);
  read (in, "point_step", out.point_step);
  read (in, "row_step", out.row_step);
  read (in, "data", out.data);
  read (in, "is_dense", out.is_dense);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, sensor_msgs::PointField& out)
{
  read (in, "name", out.name);
  read (in, "offset", out.offset);
  read (in, "datatype", out.datatype);
  read (in, "count", out.count);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, std_msgs::Header& out)
{
  read (in, "seq", out.seq);
  read (in, "stamp", out.stamp);
  read (in, "frame_id", out.frame_id);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, std_msgs::String& out)
{
  read (in, "data", out.data);
  in.expect (YAML_MAPPING_END_EVENT);
}
template <> void read_tail (Parser& in, uint8_t& out)
{
  try {
    unsigned u = boost::lexical_cast<unsigned> (in.scalar_value());
    if (u > 255) {
      throw overflow_error ("At " + in.location() + ": Error converting \"" + in.scalar_value() + "\" to uint8_t");
    }
    out = u;
  }
  catch (exception) {
    throw runtime_error ("At " + in.location() + ": Error converting \"" + in.scalar_value() + "\" to uint8_t");
  }
}

#endif
