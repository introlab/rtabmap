/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace rtabmap;

void showUsage()
{
	printf("\nUpdate a database with a set of changes recorded while another database was opened with\n"
		   "change tracking enabled (i.e. DBDriverSqlite3::setTrackChangesOutput()). This applies the\n"
		   "data changes only; it does NOT upgrade the database schema/version.\n"
		   "\n"
		   "Usage:\n"
		   "   rtabmap-dbupdate \"database.db\" \"changes.update\"\n"
		   "\n"
		   "The changes must be applied to the exact database state they were recorded from; otherwise\n"
		   "the operation is aborted and the database is left unchanged.\n"
		   "\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	if(argc < 3)
	{
		showUsage();
	}

	std::string databasePath = argv[argc-2];
	std::string updatePath = argv[argc-1];

	if(!UFile::exists(databasePath))
	{
		printf("Database \"%s\" does not exist.\n", databasePath.c_str());
		return 1;
	}
	if(!UFile::exists(updatePath))
	{
		printf("Update file \"%s\" does not exist.\n", updatePath.c_str());
		return 1;
	}

	printf("Updating database \"%s\" with changes from \"%s\"...\n", databasePath.c_str(), updatePath.c_str());

	std::string error;
	if(DBDriverSqlite3::applyChangesFromFile(databasePath, updatePath, &error))
	{
		printf("Done! Database \"%s\" was updated successfully.\n", databasePath.c_str());
		return 0;
	}

	printf("Error: %s\n", error.c_str());
	return 1;
}
