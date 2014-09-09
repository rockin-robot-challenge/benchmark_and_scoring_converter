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

#ifndef __ROCKIN_CONVERTER__TYPES_HPP__
#define __ROCKIN_CONVERTER__TYPES_HPP__

#include <memory>
#include <map>

#include <rosbag/bag.h>

#include <audio_common_msgs/AudioData.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include "parser.hpp"



using namespace std;
using namespace ros;



class TopicTypes
{
    struct TopicTypeBase {
      virtual void
      write (rosbag::Bag& bag,
             string const& topic,
             Time const& time,
             Parser& in) = 0;
    };

    template <typename T>
    struct TopicType :
        public TopicTypeBase {
      void
      write (rosbag::Bag& bag,
             string const& topic,
             Time const& time,
             Parser& in)
      {
        T msg;
        read (in, "message", msg);
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
           Parser& in)
    {
      auto const& i = types_.find (topic);

      if (i == types_.end()) {
        cerr << "Unknown topic \"" << topic << "\" at " << in.location() << endl;
        exit (3);
      }

      i->second->write (bag, topic, time, in);
    }
};

#endif
