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
#include <cstring>

using namespace rtabmap;

void showUsage()
{
	printf("\nUpdate a database with a set of changes recorded in a \".dbu\" (db update) file, produced with\n"
		   "the --track-changes option of used by other tools. This applies the data changes only; it does NOT upgrade the database\n"
		   "schema/version.\n"
		   "\n"
		   "Usage:\n"
		   "   rtabmap-dbupdate [--rewind] \"database.db\" \"changes.dbu\"\n"
		   "\n"
		   "Options:\n"
		   "   --rewind   Undo a previously applied update instead of applying it (the changeset is\n"
		   "              inverted). The database must be in the state right after the update was applied.\n"
		   "\n"
		   "The changes must be applied to the exact database state they were recorded from (or, with\n"
		   "--rewind, the state right after they were applied); otherwise the operation is aborted and\n"
		   "the database is left unchanged.\n"
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

	bool rewind = false;
	for(int i=1; i<argc-2; ++i)
	{
		if(std::strcmp(argv[i], "--rewind") == 0)
		{
			rewind = true;
		}
		else
		{
			printf("Unknown option \"%s\".\n", argv[i]);
			showUsage();
		}
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

	printf("%s database \"%s\" with changes from \"%s\"...\n",
			rewind?"Rewinding":"Updating", databasePath.c_str(), updatePath.c_str());

	std::string error;
	if(DBDriverSqlite3::applyChangesFromFile(databasePath, updatePath, rewind, &error))
	{
		printf("Done! Database \"%s\" was %s successfully.\n",
				databasePath.c_str(), rewind?"rewound":"updated");
		return 0;
	}

	printf("Error: %s\n", error.c_str());
	return 1;
}
