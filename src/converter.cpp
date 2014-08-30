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

#include <string>
#include <memory>
#include <iostream>
#include <map>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>

#include <audio_common_msgs/AudioData.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>



using namespace std;
using namespace ros;
using boost::lexical_cast;



void parse_options (int argc,
                    char** argv,
                    vector<string>& input_files)
{
  using namespace boost::program_options;

  options_description desc ("Allowed options");
  desc.add_options()
  ("help,h", "produce help message")
  ;

  options_description hidden ("Hidden options");
  hidden.add (desc);
  hidden.add_options()
  ("input,i", value< vector<string> > (&input_files), "input file")
  ;

  positional_options_description p;
  p.add ("input", -1);

  variables_map vm;
  store (command_line_parser (argc, argv).options (hidden).positional (p).run(), vm);
  notify (vm);

  if (vm.count ("help") || input_files.empty()) {
    cout << "Usage: rockin_converter [options] input_files" << endl;
    cout << desc << endl;
    exit (0);
  }
}



string
output_file_name (string const& input_file)
{
  using namespace boost;

  if (ends_with (input_file, ".yaml")) {
    return replace_tail_copy (input_file, 4, "bag");
  }
  else {
    return input_file + ".bag";
  }
}



// Note that these get functions are used instead of yaml-cpp own mechanism.
// This is because of error reporting when something is missing.



template <typename T>
void
get (YAML::Node const& node,
     string const& key,
     T& out,
     size_t message_number);



template <typename T>
void
getSeq (YAML::Node const& node,
        string const& key,
        T& out,
        size_t message_number);



template <typename T>
void
get (YAML::Node const& node,
     T& out,
     size_t message_number)
{
  out = node.as<T>();
}



template <>
void
get (YAML::Node const& node,
     uint8_t& out,
     size_t message_number)
{
  out = node.as<unsigned>();
}



template <>
void
get (YAML::Node const& node,
     audio_common_msgs::AudioData& out,
     size_t message_number)
{
  get (node, "data", out.data, message_number);
}



template <>
void
get (YAML::Node const& node,
     vector<uint8_t>& out,
     size_t message_number)
{
  if (node.IsScalar()) {
    out = YAML::DecodeBase64 (node.Scalar());
  }
  else if (node.IsSequence()) {
    out.resize (node.size());
    for (size_t i = 0; i < node.size(); ++i) {
      get (node[i], out[i], message_number);
    }
    // cout << YAML::EncodeBase64(&out[0], out.size()) << endl;
  }
  else {
    cerr << "Data should be a sequence or base 64 binary in message number " << lexical_cast<string> (message_number) << endl;
    exit (3);
  }
}



template <>
void
get (YAML::Node const& node,
     geometry_msgs::Pose2D& out,
     size_t message_number)
{
  get (node, "x", out.x, message_number);
  get (node, "y", out.y, message_number);
  get (node, "theta", out.theta, message_number);
}



template <>
void
get (YAML::Node const& node,
     std_msgs::String& out,
     size_t message_number)
{
  get (node, "data", out.data, message_number);
}



template <>
void
get (YAML::Node const& node,
     sensor_msgs::Image& out,
     size_t message_number)
{
  get (node, "header", out.header, message_number);
  get (node, "height", out.height, message_number);
  get (node, "width", out.width, message_number);
  get (node, "encoding", out.encoding, message_number);
  get (node, "is_bigendian", out.is_bigendian, message_number);
  get (node, "step", out.step, message_number);
  get (node, "data", out.data, message_number);
}



template <>
void
get (YAML::Node const& node,
     std_msgs::Header& out,
     size_t message_number)
{
  get (node, "seq", out.seq, message_number);
  get (node, "stamp", out.stamp, message_number);
  get (node, "frame_id", out.frame_id, message_number);
}



template <>
void
get (YAML::Node const& node,
     Time& out,
     size_t message_number)
{
  get (node, "secs", out.sec, message_number);
  get (node, "nsecs", out.nsec, message_number);
}



template <>
void
get (YAML::Node const& node,
     geometry_msgs::Pose& out,
     size_t message_number)
{
  get (node, "position", out.position, message_number);
  get (node, "orientation", out.orientation, message_number);
}



template <>
void
get (YAML::Node const& node,
     geometry_msgs::Point& out,
     size_t message_number)
{
  get (node, "x", out.x, message_number);
  get (node, "y", out.y, message_number);
  get (node, "z", out.z, message_number);
}



template <>
void
get (YAML::Node const& node,
     geometry_msgs::Quaternion& out,
     size_t message_number)
{
  get (node, "x", out.x, message_number);
  get (node, "y", out.y, message_number);
  get (node, "z", out.z, message_number);
  get (node, "w", out.w, message_number);
}



template <>
void
get (YAML::Node const& node,
     nav_msgs::Path& out,
     size_t message_number)
{
  get (node, "header", out.header, message_number);
  getSeq (node, "poses", out.poses, message_number);
}



template <>
void
get (YAML::Node const& node,
     geometry_msgs::PoseStamped& out,
     size_t message_number)
{
  get (node, "header", out.header, message_number);
  get (node, "pose", out.pose, message_number);
}



template <>
void
get (YAML::Node const& node,
     sensor_msgs::PointCloud2& out,
     size_t message_number)
{
  get (node, "header", out.header, message_number);
  get (node, "height", out.height, message_number);
  get (node, "width", out.width, message_number);
  getSeq (node, "fields", out.fields, message_number);
  get (node, "is_bigendian", out.is_bigendian, message_number);
  get (node, "point_step", out.point_step, message_number);
  get (node, "row_step", out.row_step, message_number);
  get (node, "data", out.data, message_number);
  get (node, "is_dense", out.is_dense, message_number);
}



template <>
void
get (YAML::Node const& node,
     sensor_msgs::PointField& out,
     size_t message_number)
{
  get (node, "name", out.name, message_number);
  get (node, "offset", out.offset, message_number);
  get (node, "datatype", out.datatype, message_number);
  get (node, "count", out.count, message_number);
}



template <typename T>
void
get (YAML::Node const& node,
     string const& key,
     T& out,
     size_t message_number)
{
  if (! node[key]) {
    cerr << "Could not find \"" << key << "\" in message number " << lexical_cast<string> (message_number) << endl;
    exit (2);
  }
  get (node[key], out, message_number);
}



template <typename T>
void
getSeq (YAML::Node const& node,
        string const& key,
        T& out,
        size_t message_number)
{
  if (! node[key]) {
    cerr << "Could not find \"" << key << "\" in message number " << lexical_cast<string> (message_number) << endl;
    exit (2);
  }
  YAML::Node const& in = node[key];
  if (! in.IsSequence()) {
    cerr << "Should be a sequence: \"" << key << "\" in message number " << lexical_cast<string> (message_number) << endl;
    exit (3);
  }
  out.resize (in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    get (in[i], out[i], message_number);
  }
}



class TopicTypes
{
    struct TopicTypeBase {
      virtual void
      write (rosbag::Bag& bag,
             string const& topic,
             Time const& time,
             YAML::Node const& node,
             size_t message_number) = 0;
    };

    template <typename T>
    struct TopicType :
        public TopicTypeBase {
      void
      write (rosbag::Bag& bag,
             string const& topic,
             Time const& time,
             YAML::Node const& node,
             size_t message_number)
      {
        T msg;
        get (node, "message", msg, message_number);
        bag.write (topic, time, msg);
      }
    };

    map<string, shared_ptr<TopicTypeBase> > types_;

  public:
    TopicTypes()
    {
      set<audio_common_msgs::AudioData> ("audio");
      set<std_msgs::String> ("command");
      set<std_msgs::String> ("condition_after");
      set<std_msgs::String> ("condition_rcv");
      set<std_msgs::String> ("container");
      set<sensor_msgs::Image> ("image");
      set<std_msgs::String> ("info");
      set<std_msgs::String> ("initial_plan");
      set<std_msgs::String> ("new_plan");
      set<std_msgs::String> ("notification");
      set<std_msgs::String> ("object");
      set<geometry_msgs::Pose> ("object_pose");
      set<nav_msgs::Path> ("path");
      set<std_msgs::String> ("plan");
      set<sensor_msgs::PointCloud2> ("pointcloud");
      set<geometry_msgs::Pose2D> ("pose2d");
      set<geometry_msgs::Pose> ("pose");
      set<geometry_msgs::Pose> ("position");
      set<std_msgs::String> ("transcriptions");
      set<std_msgs::String> ("tray");
      set<std_msgs::String> ("visitor");
    }

    template <typename T>
    void
    set (string const& topic)
    {
      types_[topic] = make_shared<TopicType<T> >();
    }

    void
    write (rosbag::Bag& bag,
           string const& topic,
           Time const& time,
           YAML::Node const& node,
           size_t message_number)
    {
      auto const& i = types_.find (topic);

      if (i == types_.end()) {
        cerr << "Unknown topic \"" << topic << "\" in message number " << lexical_cast<string> (message_number) << endl;
        exit (3);
      }

      i->second->write (bag, topic, time, node, message_number);
    }
};



int
main (int argc,
      char** argv)
{
  vector<string> input_files;
  parse_options (argc, argv, input_files);

  TopicTypes types;

  for (auto const& input_file : input_files) {
    string output_file = output_file_name (input_file);
    cerr << "Processing file " << input_file << " -> " << output_file << endl << flush;

    YAML::Node in = YAML::LoadFile (input_file);
    rosbag::Bag bag (output_file, rosbag::bagmode::Write);

    if (! in.IsSequence()) {
      cerr << "Input file should be a sequence of messages" << endl;
      exit (1);
    }

    for (size_t i = 0; i < in.size(); ++i) {
      YAML::Node const& node = in[i];

      string topic;
      get (node, "topic", topic, i);
      Time time;
      get (node, "secs", time.sec, i);
      get (node, "nsecs", time.nsec, i);

      types.write (bag, topic, time, node, i);

      cerr << "." << flush;
    }

    cerr << endl;

    bag.close();
  }

  return 0;
}
