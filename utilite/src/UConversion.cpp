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

#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UStl.h"

#include <sstream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#endif

std::string uReplaceChar(const std::string & str, char before, char after)
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

std::string uReplaceChar(const std::string & str, char before, const std::string & after)
{
	std::string s;
	for(unsigned int i=0; i<str.size(); ++i)
	{
		if(str.at(i) != before)
		{
			s.push_back(str.at(i));
		}
		else
		{
			s.append(after);
		}
	}
	return s;
}

std::string uToUpperCase(const std::string & str)
{
	std::string result = str;
	for(unsigned int i=0; i<result.size(); ++i)
	{
		// only change case of ascii characters ('a' to 'z')
		if(result[i] >= 'a' && result[i]<='z')
		{
			result[i] = result[i] - 'a' + 'A';
		}
	}
	return result;
}

std::string uToLowerCase(const std::string & str)
{
	std::string result = str;
	for(unsigned int i=0; i<result.size(); ++i)
	{
		// only change case of ascii characters ('A' to 'Z')
		if(result[i] >= 'A' && result[i]<='Z')
		{
			result[i] = result[i] - 'A' + 'a';
		}
	}
	return result;
}

std::string uNumber2Str(unsigned int number)
{
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uNumber2Str(int number)
{
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uNumber2Str(float number)
{
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uNumber2Str(double number)
{
	std::stringstream s;
	s << number;
	return s.str();
}

int uStr2Int(const std::string & str)
{
	return atoi(str.c_str());
}

float uStr2Float(const std::string & str)
{
	float value = 0.0f;
	std::istringstream istr(uReplaceChar(str, ',', '.').c_str());
	istr.imbue(std::locale("C"));
	istr >> value;
	return value;
}

double uStr2Double(const std::string & str)
{
	double value = 0.0;
	std::istringstream istr(uReplaceChar(str, ',', '.').c_str());
	istr.imbue(std::locale("C"));
	istr >> value;
	return value;
}

std::string uBool2Str(bool boolean)
{
	std::string s;
	if(boolean)
	{
		s = "true";
	}
	else
	{
		s = "false";
	}
	return s;
}

bool uStr2Bool(const char * str)
{
	return !(str && (uStrContains(str, "false") || uStrContains(str, "FALSE") || strcmp(str, "0") == 0));
}

bool uStr2Bool(const std::string & str)
{
	return !(uStrContains(str, "false") || uStrContains(str, "FALSE") || str.compare("0") == 0);
}

std::vector<unsigned char> uStr2Bytes(const std::string & str)
{
	std::vector<unsigned char> bytes(str.size()+1);
	memcpy(bytes.data(), str.data(), str.size());
	bytes[bytes.size()-1] = '\0'; // null character
	return bytes;
}

std::string uBytes2Str(const std::vector<unsigned char> & bytes)
{
	if(bytes.size())
	{
		if(bytes[bytes.size()-1] != '\0')
		{
			std::vector<unsigned char> tmp = bytes;
			tmp.push_back('\0');
			return std::string((const char *)tmp.data());
		}
		return std::string((const char *)bytes.data());
	}
	return std::string();
}

std::string uBytes2Hex(const char * bytes, unsigned int bytesLen)
{
	std::string hex;
	if(!bytes || bytesLen == 0)
	{
		return hex;
	}
	const unsigned char * bytes_u = (const unsigned char*)(bytes);

	hex.resize(bytesLen*2);
	char * pHex = &hex[0];
	const unsigned char * pEnd = (bytes_u + bytesLen);
	for(const unsigned char * pChar = bytes_u; pChar != pEnd; ++pChar, pHex += 2)
	{
		pHex[0] = uHex2Ascii(*pChar, 0);
		pHex[1] = uHex2Ascii(*pChar, 1);
	}
	return hex;
}

std::vector<char> uHex2Bytes(const std::string & hex)
{
	return uHex2Bytes(&hex[0], (int)hex.length());
}

std::vector<char> uHex2Bytes(const char * hex, int hexLen)
{
	std::vector<char> bytes;
	if(!hex || hexLen % 2 || hexLen == 0)
	{
		return bytes; // must be pair
	}

	unsigned int bytesLen = hexLen / 2;
	bytes.resize(bytesLen);
	unsigned char * pBytes = (unsigned char *)&bytes[0];
	const unsigned char * pHex = (const unsigned char *)hex;

	unsigned char * pEnd = (pBytes + bytesLen);
	for(unsigned char * pChar = pBytes; pChar != pEnd; pChar++, pHex += 2)
	{
		*pChar = (uAscii2Hex(pHex[0]) << 4) | uAscii2Hex(pHex[1]);
	}
	return bytes;
}

// The hex str MUST not contains any null values (0x00)
std::string uHex2Str(const std::string & hex)
{
	std::vector<char> bytes = uHex2Bytes(hex);
	return std::string(&bytes[0], bytes.size());
}

static const char HEX2ASCII[256][2] =
{
	{'0','0'},{'0','1'},{'0','2'},{'0','3'},{'0','4'},{'0','5'},{'0','6'},{'0','7'},{'0','8'},{'0','9'},{'0','A'},{'0','B'},{'0','C'},{'0','D'},{'0','E'},{'0','F'},
	{'1','0'},{'1','1'},{'1','2'},{'1','3'},{'1','4'},{'1','5'},{'1','6'},{'1','7'},{'1','8'},{'1','9'},{'1','A'},{'1','B'},{'1','C'},{'1','D'},{'1','E'},{'1','F'},
	{'2','0'},{'2','1'},{'2','2'},{'2','3'},{'2','4'},{'2','5'},{'2','6'},{'2','7'},{'2','8'},{'2','9'},{'2','A'},{'2','B'},{'2','C'},{'2','D'},{'2','E'},{'2','F'},
	{'3','0'},{'3','1'},{'3','2'},{'3','3'},{'3','4'},{'3','5'},{'3','6'},{'3','7'},{'3','8'},{'3','9'},{'3','A'},{'3','B'},{'3','C'},{'3','D'},{'3','E'},{'3','F'},
	{'4','0'},{'4','1'},{'4','2'},{'4','3'},{'4','4'},{'4','5'},{'4','6'},{'4','7'},{'4','8'},{'4','9'},{'4','A'},{'4','B'},{'4','C'},{'4','D'},{'4','E'},{'4','F'},
	{'5','0'},{'5','1'},{'5','2'},{'5','3'},{'5','4'},{'5','5'},{'5','6'},{'5','7'},{'5','8'},{'5','9'},{'5','A'},{'5','B'},{'5','C'},{'5','D'},{'5','E'},{'5','F'},
	{'6','0'},{'6','1'},{'6','2'},{'6','3'},{'6','4'},{'6','5'},{'6','6'},{'6','7'},{'6','8'},{'6','9'},{'6','A'},{'6','B'},{'6','C'},{'6','D'},{'6','E'},{'6','F'},
	{'7','0'},{'7','1'},{'7','2'},{'7','3'},{'7','4'},{'7','5'},{'7','6'},{'7','7'},{'7','8'},{'7','9'},{'7','A'},{'7','B'},{'7','C'},{'7','D'},{'7','E'},{'7','F'},
	{'8','0'},{'8','1'},{'8','2'},{'8','3'},{'8','4'},{'8','5'},{'8','6'},{'8','7'},{'8','8'},{'8','9'},{'8','A'},{'8','B'},{'8','C'},{'8','D'},{'8','E'},{'8','F'},
	{'9','0'},{'9','1'},{'9','2'},{'9','3'},{'9','4'},{'9','5'},{'9','6'},{'9','7'},{'9','8'},{'9','9'},{'9','A'},{'9','B'},{'9','C'},{'9','D'},{'9','E'},{'9','F'},
	{'A','0'},{'A','1'},{'A','2'},{'A','3'},{'A','4'},{'A','5'},{'A','6'},{'A','7'},{'A','8'},{'A','9'},{'A','A'},{'A','B'},{'A','C'},{'A','D'},{'A','E'},{'A','F'},
	{'B','0'},{'B','1'},{'B','2'},{'B','3'},{'B','4'},{'B','5'},{'B','6'},{'B','7'},{'B','8'},{'B','9'},{'B','A'},{'B','B'},{'B','C'},{'B','D'},{'B','E'},{'B','F'},
	{'C','0'},{'C','1'},{'C','2'},{'C','3'},{'C','4'},{'C','5'},{'C','6'},{'C','7'},{'C','8'},{'C','9'},{'C','A'},{'C','B'},{'C','C'},{'C','D'},{'C','E'},{'C','F'},
	{'D','0'},{'D','1'},{'D','2'},{'D','3'},{'D','4'},{'D','5'},{'D','6'},{'D','7'},{'D','8'},{'D','9'},{'D','A'},{'D','B'},{'D','C'},{'D','D'},{'D','E'},{'D','F'},
	{'E','0'},{'E','1'},{'E','2'},{'E','3'},{'E','4'},{'E','5'},{'E','6'},{'E','7'},{'E','8'},{'E','9'},{'E','A'},{'E','B'},{'E','C'},{'E','D'},{'E','E'},{'E','F'},
	{'F','0'},{'F','1'},{'F','2'},{'F','3'},{'F','4'},{'F','5'},{'F','6'},{'F','7'},{'F','8'},{'F','9'},{'F','A'},{'F','B'},{'F','C'},{'F','D'},{'F','E'},{'F','F'}
};

unsigned char uHex2Ascii(const unsigned char & c, bool rightPart)
{
	if(rightPart)
	{
		return HEX2ASCII[c][1];
	}
	else
	{
		return HEX2ASCII[c][0];
	}
}

unsigned char uAscii2Hex(const unsigned char & c)
{
	switch(c)
	{
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		return c-'0';
	case 'A':
	case 'B':
	case 'C':
	case 'D':
	case 'E':
	case 'F':
		return c-'A'+10;
	case 'a':
	case 'b':
	case 'c':
	case 'd':
	case 'e':
	case 'f':
		return c-'a'+10;
	default:
		return 0x00;
	}
}

std::string uFormatv (const char *fmt, va_list args)
{
    // Allocate a buffer on the stack that's big enough for us almost
    // all the time.  Be prepared to allocate dynamically if it doesn't fit.
    size_t size = 1024;
    std::vector<char> dynamicbuf(size);
    char *buf = &dynamicbuf[0];

    va_list argsTmp;

    while (1) {
#if defined(_WIN32) && !defined(__MINGW32__)
	argsTmp = args;
#else
	va_copy(argsTmp, args);
#endif

        // Try to vsnprintf into our buffer.
#ifdef _MSC_VER
    	int needed = vsnprintf_s(buf, size, size, fmt, argsTmp);
#else
    	int needed = vsnprintf (buf, size, fmt, argsTmp);
#endif
    	va_end(argsTmp);
        // NB. C99 (which modern Linux and OS X follow) says vsnprintf
        // failure returns the length it would have needed.  But older
        // glibc and current Windows return -1 for failure, i.e., not
        // telling us how much was needed.
        if (needed < (int)size-1 && needed >= 0) {
            // It fit fine so we're done.
            return std::string (buf, (size_t) needed);
        }

        // vsnprintf reported that it wanted to write more characters
        // than we allotted.  So try again using a dynamic buffer.  This
        // doesn't happen very often if we chose our initial size well.
        size = needed>=0?needed+2:size*2;
        dynamicbuf.resize (size);
        buf = &dynamicbuf[0];
    }
    return std::string(); // would not reach this, but for compiler complaints...
}

std::string uFormat (const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	std::string buf = uFormatv(fmt, args);
	va_end(args);
    return buf;
}

#ifdef _WIN32
// returned whar_t * must be deleted : delete [] wText;
wchar_t * createWCharFromChar(const char * text)
{
	DWORD length = MultiByteToWideChar (CP_ACP, 0, text, -1, NULL, 0);
	wchar_t * wText = new wchar_t[length];
	MultiByteToWideChar (CP_ACP, 0, text, -1, wText, length );
	return wText;
}

// returned char * must be deleted : delete [] text;
char * createCharFromWChar(const wchar_t * wText)
{
	DWORD length = WideCharToMultiByte (CP_ACP, 0, wText, -1, NULL, 0, NULL, NULL);
	char * text = new char[length];
	WideCharToMultiByte (CP_ACP, 0, wText, -1, text, length, NULL, NULL);
	return text;
}
#endif
