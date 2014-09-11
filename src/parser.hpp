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

#ifndef __ROCKIN_CONVERTER__PARSER_HPP__
#define __ROCKIN_CONVERTER__PARSER_HPP__

#include <cstdio>
#include <iostream>
#include <cerrno>
#include <system_error>

#include <boost/noncopyable.hpp>
#include <boost/lexical_cast.hpp>

#include <yaml.h>



using namespace std;



class Parser : boost::noncopyable
{
    FILE* input_;
    yaml_parser_t parser_;
    bool first_event_;
    yaml_event_t event_;

  public:
    Parser (const string& input_file) :
      first_event_ (true)
    {
      input_ = fopen (input_file.c_str(), "rb");
      if (! input_) {
        throw system_error (errno, system_category(), "Failed to open input file " + input_file + " for reading");
      }

      if (! yaml_parser_initialize (&parser_)) {
        throw runtime_error ("Failed to initialize libyaml parser");
      }

      yaml_parser_set_input_file (&parser_, input_);
    }

    ~Parser()
    {
      if (! first_event_) {
        yaml_event_delete (&event_);
      }
      yaml_parser_delete (&parser_);
      fclose (input_);
    }

    bool new_event()
    {
      if (! first_event_) {
        yaml_event_delete (&event_);
      }
      else {
        first_event_ = false;
      }

      if (! yaml_parser_parse (&parser_, &event_)) {
        throw runtime_error ("Failed to parse the input with libyaml parser");
      }

      return event_.type != YAML_STREAM_END_EVENT;
    }

    yaml_event_t const& event()
    {
      if (first_event_) {
        throw logic_error ("Cannot access events before new_event is called");
      }
      return event_;
    }

    string scalar_value()
    {
      if (event_.data.scalar.value) {
        return string ( (char*) event_.data.scalar.value);
      }
      else {
        return "";
      }
    }

    string event_name (yaml_event_type_t type)
    {
      switch (type) {
        case YAML_NO_EVENT:
          return "YAML_NO_EVENT";
          break;
        case YAML_STREAM_START_EVENT:
          return "YAML_STREAM_START_EVENT";
          break;
        case YAML_STREAM_END_EVENT:
          return "YAML_STREAM_END_EVENT";
          break;
        case YAML_DOCUMENT_START_EVENT:
          return "YAML_DOCUMENT_START_EVENT";
          break;
        case YAML_DOCUMENT_END_EVENT:
          return "YAML_DOCUMENT_END_EVENT";
          break;
        case YAML_ALIAS_EVENT:
          return "YAML_ALIAS_EVENT";
          break;
        case YAML_SCALAR_EVENT:
          return "YAML_SCALAR_EVENT";
          break;
        case YAML_SEQUENCE_START_EVENT:
          return "YAML_SEQUENCE_START_EVENT";
          break;
        case YAML_SEQUENCE_END_EVENT:
          return "YAML_SEQUENCE_END_EVENT";
          break;
        case YAML_MAPPING_START_EVENT:
          return "YAML_MAPPING_START_EVENT";
          break;
        case YAML_MAPPING_END_EVENT:
          return "YAML_MAPPING_END_EVENT";
          break;
      }
    }

    string location()
    {
      return boost::lexical_cast<string> (event_.start_mark.line + 1) + ":" + boost::lexical_cast<string> (event_.start_mark.column);
    }

    void expect (yaml_event_type_t expected, string const& reason = "")
    {
      new_event();
      if (event_.type != expected) {
        throw runtime_error ("At " + location() + ": Was expecting " + event_name (expected) + " but found " + event_name (event_.type) + (reason.empty() ? string() : (": " + reason)));
      }
    }

    bool expect (yaml_event_type_t expected0, yaml_event_type_t expected1, string const& reason = "")
    {
      new_event();
      if ( (event_.type != expected0) && (event_.type != expected1)) {
        throw runtime_error ("At " + location() + ": Was expecting " + event_name (expected0) + " or " + event_name (expected1) + " but found " + event_name (event_.type) + (reason.empty() ? string() : (": " + reason)));
      }
      return event_.type == expected0;
    }

    void expect (string const& expected)
    {
      expect (YAML_SCALAR_EVENT, "should be \"" + expected + "\"");
      if ( (! event_.data.scalar.value) || (scalar_value() != expected)) {
        throw runtime_error ("At " + location() + ": Was expecting \"" + expected + "\" but found " + (event_.data.scalar.value ? ("\"" + scalar_value() + "\"") : string ("nothing")));
      }
    }

    void debug_dump()
    {
      using namespace std;

      size_t indentation = 0;

      bool done = false;
      while (! done) {
        new_event();

        switch (event_.type) {
          case YAML_STREAM_END_EVENT:
            done = true;
          case YAML_DOCUMENT_END_EVENT:
          case YAML_SEQUENCE_END_EVENT:
          case YAML_MAPPING_END_EVENT:
            indentation -= 2;
            break;
          default:
            break;
        }

        cout << string (indentation, ' ') << location() << " " << event_name (event_.type) << endl;
        if (event_.type == YAML_SCALAR_EVENT) {
          if (event_.data.scalar.anchor) {
            cout << string (indentation, ' ') << "  anchor: " << event_.data.scalar.anchor << endl;
          }
          if (event_.data.scalar.tag) {
            cout << string (indentation, ' ') << "  tag: " << event_.data.scalar.tag << endl;
          }
          if (event_.data.scalar.value) {
            cout << string (indentation, ' ') << "  value: " << event_.data.scalar.value << endl;
          }
          cout << string (indentation, ' ') << "  length: " << event_.data.scalar.length << endl;
        }

        switch (event_.type) {
          case YAML_STREAM_START_EVENT:
          case YAML_DOCUMENT_START_EVENT:
          case YAML_SEQUENCE_START_EVENT:
          case YAML_MAPPING_START_EVENT:
            indentation += 2;
            break;
          default:
            break;
        }
      }
    }
};

#endif
