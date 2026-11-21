/*
Copyright (c) 2010, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef UPROCESSINFO_H
#define UPROCESSINFO_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

/**
 * \file UProcessInfo.h
 * \brief UProcessInfo class for getting process information
 *
 * This class provides methods to retrieve information about the current process,
 * such as memory usage.
 */

/**
 * \class UProcessInfo
 * \brief Class for getting information about the current process
 *
 * This class provides static methods to retrieve information about the current
 * process, such as memory usage. The information is retrieved from the operating
 * system.
 *
 * Example:
 * @code
 * long int memory = UProcessInfo::getMemoryUsage();
 * std::cout << "Memory usage: " << memory << " bytes" << std::endl;
 * @endcode
 */
class UTILITE_EXPORT UProcessInfo {
public:
	/**
	 * \brief Constructor
	 */
	UProcessInfo();
	
	/**
	 * \brief Virtual destructor
	 */
	virtual ~UProcessInfo();

	/**
	 * \brief Get the memory used by the current process
	 * 
	 * This method returns the amount of memory currently used by the process.
	 * The exact meaning of "memory used" may vary between operating systems.
	 * 
	 * @return the number of bytes used by the current process, or -1 on error
	 */
	static long int getMemoryUsage();
};

#endif /* UPROCESSINFO_H */
