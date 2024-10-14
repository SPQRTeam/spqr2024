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

#pragma once

#include <map>
#include <string>


namespace GMapping
{
	class test_{};

	class AutoVal
	{
		private:
			::std::string m_value;
			
		protected:
			::std::string toLower(const ::std::string& source) const;
			
		public:
			AutoVal() {;}
			
			AutoVal(const ::std::string&);
			AutoVal(float);
			AutoVal(int);
			AutoVal(unsigned int);
			AutoVal(bool);
			AutoVal(const char*);
			AutoVal(const AutoVal&);
			AutoVal& operator=(const AutoVal&);
			
			AutoVal& operator= (float);
			AutoVal& operator= (int);
			AutoVal& operator= (unsigned int);
			AutoVal& operator= (bool);
			AutoVal& operator= (const ::std::string&);
			
			operator ::std::string() const;
			operator float() const;
			operator int() const;
			operator unsigned int() const;
			operator bool() const;
	};
	
	class ConfigFile
	{
		::std::map<::std::string,AutoVal> m_content;
		
		protected:
			void insertValue(const ::std::string& section,const ::std::string& entry,const ::std::string& thevalue);
			::std::string toLower(const ::std::string& source) const;
			::std::string trim(const ::std::string& source,char const* delims = " \t\r\n") const;
			::std::string truncate(const ::std::string& source,const char* atChar) const;
			
		public:
			ConfigFile();
			ConfigFile(const ::std::string& configFile);
			ConfigFile(const char* configFile);
			
			void dumpValues(::std::ostream& out);
			
			bool read(const ::std::string& configFile);
			bool read(const char* configFile);
			
			const AutoVal& value(const ::std::string& section,const ::std::string& entry) const;
			const AutoVal& value(const ::std::string& section,const ::std::string& entry,float def);
			const AutoVal& value(const ::std::string& section,const ::std::string& entry,const char* def);
			const AutoVal& value(const ::std::string& section,const ::std::string& entry,bool def);
			const AutoVal& value(const ::std::string& section,const ::std::string& entry,int def);
			const AutoVal& value(const ::std::string& section,const ::std::string& entry,unsigned int def);
			const AutoVal& value(const ::std::string& section,const ::std::string& entry,const ::std::string& def);
	};
}
