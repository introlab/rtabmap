/*
Copyright (c) 2010-2017, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
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

#ifndef RECOVERY_H_
#define RECOVERY_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <string>

namespace rtabmap {

class ProgressState;

/**
 * Return true on success. The database is
 * renamed to "*.backup.db" before recovering.
 * @param corruptedDatabase database to recover
 * @param keepCorruptedDatabase if false and on recovery success, the backup database is removed
 * @param errorMsg error message if the function returns false
 * @param progressState A ProgressState object used to get status of the recovery process
 */
bool RTABMAP_CORE_EXPORT databaseRecovery(
		const std::string & corruptedDatabase,
		bool keepCorruptedDatabase = true,
		std::string * errorMsg = 0,
		ProgressState * progressState = 0);

}


#endif /* RECOVERY_H_ */
