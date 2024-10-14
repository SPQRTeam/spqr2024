/*
 * PTracking - Distributed real-time multiple object tracking library.
 * Copyright (c) 2014, Fabio Previtali. All rights reserved.
 * 
 * This file is part of PTracking.
 * 
 * PTracking is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * PTracking is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with PTracking. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Written by Fabio Previtali.
 * 
 * Please, report any suggestion/comment/bug to fabio.previtali@gmail.com.
 */

#include "ConfigFile.h"
#include <ctype.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

namespace GMapping
{
	AutoVal::AutoVal(const string& value) : m_value(value) {;}
	
	AutoVal::AutoVal(const char* c) : m_value(c) {;}
	
	AutoVal::AutoVal(float d)
	{
		stringstream s;
		
		s << d;
		
		m_value = s.str();
	}
	
	AutoVal::AutoVal(bool d)
	{
		stringstream s;
		
		if (d) s << "on";
		else s << "off";
		
		m_value = s.str();
	}
	
	AutoVal::AutoVal(int i)
	{
		stringstream s;
		
		s << i;
		
		m_value = s.str();
	}
	
	AutoVal::AutoVal(unsigned int i)
	{
		stringstream s;
		
		s << i;
		
		m_value = s.str();
	}
	
	AutoVal::AutoVal(const AutoVal& other) : m_value(other.m_value) {;}
	
	AutoVal& AutoVal::operator= (const AutoVal& other)
	{
		m_value = other.m_value;
		
		return *this;
	}
	
	AutoVal& AutoVal::operator= (float d)
	{
		stringstream s;
		
		s << d;
		
		m_value = s.str();
		
		return *this;
	}
	
	AutoVal& AutoVal::operator= (bool d)
	{
		stringstream s;
		
		if (d) s << "on";
		else s << "off";
		
		m_value = s.str();
		
		return *this;
	}
	
	AutoVal& AutoVal::operator= (int i)
	{
		stringstream s;
		
		s << i;
		
		m_value = s.str();
		
		return *this;
	}
	
	AutoVal& AutoVal::operator= (unsigned int i)
	{
		stringstream s;
		
		s << i;
		
		m_value = s.str();
		
		return *this;
	}
	
	AutoVal& AutoVal::operator= (const string& s)
	{
		m_value = s;
		
		return *this;
	}
	
	AutoVal::operator string() const
	{
		return m_value;
	}
	
	AutoVal::operator float() const
	{
		return atof(m_value.c_str());
	}
	
	AutoVal::operator int() const
	{
		return atoi(m_value.c_str());
	}
	
	AutoVal::operator unsigned int() const
	{
		return (unsigned int) atoi(m_value.c_str());
	}
	
	AutoVal::operator bool() const
	{
		if (toLower(m_value) == "on" || atoi(m_value.c_str()) == 1) return true;
		
		return false;
	}
	
	string AutoVal::toLower(const string& source) const
	{
		string result(source);
		
		char c = '\0';
		
		for (unsigned int i = 0; i < result.length(); ++i)
		{
			c = result[i];
			c = ::tolower(c);
			result[i] = c;
		}
		
		return result;
	}
	
	ConfigFile::ConfigFile() {;}
	
	ConfigFile::ConfigFile(const string& configFile)
	{
		read(configFile);
	}
	
	ConfigFile::ConfigFile(const char* configFile)
	{
		read(configFile);
	}
	
	void ConfigFile::dumpValues(ostream& out)
	{
		for (map<string,AutoVal>::const_iterator it = m_content.begin(); it != m_content.end(); ++it)
		{
			out << (string) it->first << " " << (string)it->second << endl;
		}
	}
	
	void ConfigFile::insertValue(const string& section, const string& entry, const string& thevalue)
	{
		m_content[toLower(section) + '/' + toLower(entry)] = AutoVal(thevalue);
	}
	
	bool ConfigFile::read(const char* configFile)
	{
		return read(string(configFile));
	}
	
	bool ConfigFile::read(const string& configFile)
	{
		ifstream file(configFile.c_str());
		
		if (!file || file.eof()) return false;
		
		string line;
		string name;
		string val;
		string inSection;
		
		while (getline(file,line))
		{
			if (!line.length()) continue;
			if (line[0] == '#') continue;
			
			line = truncate(line,"#");
			line = trim(line);
			
			if (!line.length()) continue;
			if (line[0] == '[')
			{
				inSection = trim(line.substr(1,line.find(']') - 1));
				
				continue;
			}
			
			istringstream lineStream(line);
			
			lineStream >> name;
			lineStream >> val;
			
			insertValue(inSection,name,val);
		}
		
		return true;
	}
	
	string ConfigFile::toLower(const string& source) const
	{
		string result(source);
		
		char c = '\0';
		
		for (unsigned int i = 0; i < result.length(); ++i)
		{
			c = result[i];
			c = ::tolower(c);
			result[i] = c;
		}
		
		return result;
	}
	
	string ConfigFile::trim(const string& source, char const* delims) const
	{
		string result(source);
		string::size_type index = result.find_last_not_of(delims);
		
		if (index != string::npos) result.erase(++index);
		
		index = result.find_first_not_of(delims);
		
		if (index != string::npos) result.erase(0,index);
		else result.erase();
		
		return result;
	}
	
	string ConfigFile::truncate(const string& source, const char* atChar) const
	{
		string::size_type index = source.find_first_of(atChar);
		
		if (index == 0)  return "";
		else if (index == string::npos) return source;
		
		return source.substr(0,index);
	}
	
	const AutoVal& ConfigFile::value(const string& section, const string& entry) const
	{
		map<string,AutoVal>::const_iterator ci = m_content.find(toLower(section) + '/' + toLower(entry));
		
		if (ci == m_content.end()) throw "Entry does not exist.";
		
		return ci->second;
	}
	
	const AutoVal& ConfigFile::value(const string& section, const string& entry, float def)
	{
		try
		{
			return value(section,entry);
		}
		catch(const char *)
		{
			return m_content.insert(make_pair(section + '/' + entry,AutoVal(def))).first->second;
		}
	}
	
	const AutoVal& ConfigFile::value(const string& section, const string& entry, bool def)
	{
		try
		{
			return value(section,entry);
		}
		catch(const char *)
		{
			return m_content.insert(make_pair(section + '/' + entry,AutoVal(def))).first->second;
		}
	}
	
	const AutoVal& ConfigFile::value(const string& section, const string& entry, int def)
	{
		try
		{
			return value(section,entry);
		}
		catch(const char *)
		{
			return m_content.insert(make_pair(section + '/' + entry,AutoVal(def))).first->second;
		}
	}
	
	const AutoVal& ConfigFile::value(const string& section, const string& entry, unsigned int def)
	{
		try
		{
			return value(section,entry);
		}
		catch(const char *)
		{
			return m_content.insert(make_pair(section + '/' + entry,AutoVal(def))).first->second;
		}
	}
	
	const AutoVal& ConfigFile::value(const string& section, const string& entry, const string& def)
	{
		try
		{
			return value(section,entry);
		}
		catch(const char *)
		{
			return m_content.insert(make_pair(section + '/' + entry,AutoVal(def))).first->second;
		}
	}
	
	const AutoVal& ConfigFile::value(const string& section, const string& entry, const char* def)
	{
		try
		{
			return value(section,entry);
		}
		catch(const char *)
		{
			return m_content.insert(make_pair(section + '/' + entry,AutoVal(def))).first->second;
		}
	}
}
