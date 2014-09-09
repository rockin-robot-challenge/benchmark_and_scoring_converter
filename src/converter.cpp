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
#include <iostream>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <rosbag/bag.h>

#include "parser.hpp"
#include "types.hpp"
#include "readers.hpp"



using namespace std;
using namespace ros;



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



int
main (int argc,
      char** argv)
{
  vector<string> input_files;
  parse_options (argc, argv, input_files);

  TopicTypes types;

  for (string const& input_file : input_files) {
    string output_file = output_file_name (input_file);
    cerr << "Processing file " << input_file << " -> " << output_file << endl << flush;

    // Parser (input_file).debug_dump();

    Parser in (input_file);
    rosbag::Bag bag (output_file, rosbag::bagmode::Write);

    in.expect (YAML_STREAM_START_EVENT);
    in.expect (YAML_DOCUMENT_START_EVENT);
    in.expect (YAML_SEQUENCE_START_EVENT, "document should contain a sequence of messages");
    while (in.expect (YAML_MAPPING_START_EVENT, YAML_SEQUENCE_END_EVENT, "each message should be a mapping of the required elements")) {
      string topic;
      read (in, "topic", topic);
      Time time;
      read (in, "secs", time.sec);
      read (in, "nsecs", time.nsec);

      types.write (bag, topic, time, in);

      in.expect (YAML_MAPPING_END_EVENT);
      cerr << "." << flush;
    }
    cerr << endl;
    in.expect (YAML_DOCUMENT_END_EVENT);
    in.expect (YAML_STREAM_END_EVENT, "the file should contain a single document");

    bag.close();
  }

  return 0;
}
