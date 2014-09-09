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

#ifndef __ROCKIN_CONVERTER__BASE64_HPP__
#define __ROCKIN_CONVERTER__BASE64_HPP__

#include <string>
#include <sstream>
#include <vector>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/insert_linebreaks.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/archive/iterators/remove_whitespace.hpp>



void
base64_encode (std::vector<uint8_t> const& in,
               std::string& out)
{
  using namespace std;
  using namespace boost::archive::iterators;

  typedef const uint8_t* i0;
  typedef transform_width<i0, 6, 8> i1;
  typedef base64_from_binary<i1> i2;
  // typedef insert_linebreaks<i2, 72> i3; // Doesn't respect YAML indentation
  typedef i2 b64;

  ostringstream os;
  copy (b64 (&in[0]), b64 (&in[0] + in.size()), ostream_iterator<char> (os));
  const string base64_padding[] = {"", "==", "="};
  os << base64_padding[in.size() % 3];
  out = os.str();
}



void
base64_decode (std::string const& in,
               std::vector<uint8_t>& out)
{
  using namespace std;
  using namespace boost::archive::iterators;

  typedef const char* i0;
  typedef remove_whitespace<i0> i1;
  typedef binary_from_base64<i1> i2;
  typedef transform_width<i2, 8, 6> i3;
  typedef i3 b64;

  size_t size = in.size();
  while (size && (isspace (in[size - 1]) || (in[size - 1] == '='))) {
    size--;
  }

  out.clear();
  out.reserve (in.size());
  for (auto i = b64 (in.data()); i != b64 (in.data() + size); ++i) {
    out.push_back (*i);
  }
}

#endif
