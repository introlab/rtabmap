/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rtabmap/core/DBDriverFactory.h"
#include "DBDriverSqlite3.h"
#include "utilite/ULogger.h"

namespace rtabmap {

DBDriver * DBDriverFactory::createDBDriver(const std::string & dbDriverName, const ParametersMap & parameters)
{
	// TODO Do it with dynamic link libraries...
	// Find the driver...
	// Link dynamically to the driver...

	DBDriver * driver = 0;

	// Static link
	if(dbDriverName.compare("sqlite3") == 0)
	{
		driver = new DBDriverSqlite3(parameters);
	}
	else if(dbDriverName.compare("mysql") == 0)
	{
		// TODO mysql driver
		ULOGGER_ERROR("mysql driver is not implemented!");
	}
	else if(dbDriverName.compare("postgresql") == 0)
	{
		// TODO postgresql driver
		ULOGGER_ERROR("postgresql driver is not implemented!");
	}
	else if(dbDriverName.compare("oracle") == 0)
	{
		// TODO oracle driver
		ULOGGER_ERROR("oracle driver is not implemented!");
	}
	else
	{
		ULOGGER_ERROR("Unknown driver \"%s\"", dbDriverName.c_str());
	}

	return driver;
}

DBDriverFactory::DBDriverFactory() {

}

DBDriverFactory::~DBDriverFactory() {
}

}

