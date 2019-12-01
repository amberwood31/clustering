/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// This file included a few utility functions from the utilite library.
// Created by amber on 2019-10-23.
//

#ifndef SLAM_PLUS_PLUS_UTILITIES_H
#define SLAM_PLUS_PLUS_UTILITIES_H
#include <list>



/**
 * Split a string into multiple string around the specified separator.
 * Example:
 * @code
 * 		std::list<std::string> v = split("Hello the world!", ' ');
 * @endcode
 * The list v will contain {"Hello", "the", "world!"}
 * @param str the string
 * @param separator the separator character
 * @return the list of strings
 */
inline std::list<std::string> uSplit(const std::string & str, char separator)
{
    std::list<std::string> v;
    std::string buf;
    for(unsigned int i=0; i<str.size(); ++i)
    {
        if(str[i] != separator)
        {
            buf += str[i];
        }
        else if(buf.size())
        {
            v.push_back(buf);
            buf = "";
        }
    }
    if(buf.size())
    {
        v.push_back(buf);
    }
    return v;
}

/**
 * Convert a std::list to a std::vector.
 * @param list the list
 * @return the vector
 */
template<class V>
inline std::vector<V> uListToVector(const std::list<V> & list)
{
    return std::vector<V>(list.begin(), list.end());
}

/**
 * Replace old characters in a string to new ones.
 * Example :
 * @code
 * std::string str = "Hello";
 * uReplaceChar(str, 'l', 'p');
 * // The results is str = "Heppo";
 * @endcode
 *
 * @param str the string
 * @param before the character to be replaced by the new one
 * @param after the new character replacing the old one
 * @return the modified string
 */
inline std::string uReplaceChar(const std::string & str, char before, char after)
{
    std::string result = str;
    for(unsigned int i=0; i<result.size(); ++i)
    {
        if(result[i] == before)
        {
            result[i] = after;
        }
    }
    return result;
}

/**
 * Convert a string to a double independent of the locale (comma/dot).
 * @param the string
 * @return the number
 */
inline double uStr2Double(const std::string & str)
{
    double value = 0.0;
    std::istringstream istr(uReplaceChar(str, ',', '.').c_str());
    istr.imbue(std::locale("C"));
    istr >> value;
    return value;
}


#endif //SLAM_PLUS_PLUS_UTILITIES_H
