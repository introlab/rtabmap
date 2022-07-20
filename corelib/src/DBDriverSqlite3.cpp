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

#include "rtabmap/core/DBDriverSqlite3.h"
#include <sqlite3.h>

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Compression.h"
#include "DatabaseSchema_sql.h"
#include "DatabaseSchema_0_18_3_sql.h"
#include "DatabaseSchema_0_18_0_sql.h"
#include "DatabaseSchema_0_17_0_sql.h"
#include "DatabaseSchema_0_16_2_sql.h"
#include "DatabaseSchema_0_16_1_sql.h"
#include "DatabaseSchema_0_16_0_sql.h"


#include <set>

#include "rtabmap/utilite/UtiLite.h"

namespace rtabmap {

DBDriverSqlite3::DBDriverSqlite3(const ParametersMap & parameters) :
	DBDriver(parameters),
	_ppDb(0),
	_version("0.0.0"),
	_memoryUsedEstimate(0),
	_dbInMemory(Parameters::defaultDbSqlite3InMemory()),
	_cacheSize(Parameters::defaultDbSqlite3CacheSize()),
	_journalMode(Parameters::defaultDbSqlite3JournalMode()),
	_synchronous(Parameters::defaultDbSqlite3Synchronous()),
	_tempStore(Parameters::defaultDbSqlite3TempStore())
{
	ULOGGER_DEBUG("treadSafe=%d", sqlite3_threadsafe());
	this->parseParameters(parameters);
}

DBDriverSqlite3::~DBDriverSqlite3()
{
	this->closeConnection();
}

void DBDriverSqlite3::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kDbSqlite3CacheSize())) != parameters.end())
	{
		this->setCacheSize(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kDbSqlite3JournalMode())) != parameters.end())
	{
		this->setJournalMode(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kDbSqlite3Synchronous())) != parameters.end())
	{
		this->setSynchronous(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kDbSqlite3TempStore())) != parameters.end())
	{
		this->setTempStore(std::atoi((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kDbSqlite3InMemory())) != parameters.end())
	{
		this->setDbInMemory(uStr2Bool((*iter).second.c_str()));
	}
	DBDriver::parseParameters(parameters);
}

void DBDriverSqlite3::setCacheSize(unsigned int cacheSize)
{
	if(this->isConnected())
	{
		_cacheSize = cacheSize;
		std::string query = "PRAGMA cache_size = ";
		query += uNumber2Str(_cacheSize) + ";";
		this->executeNoResultQuery(query.c_str());
	}
}

void DBDriverSqlite3::setJournalMode(int journalMode)
{
	if(journalMode >= 0 && journalMode < 5)
	{
		_journalMode = journalMode;
		if(this->isConnected())
		{
			switch(_journalMode)
			{
			case 4:
				this->executeNoResultQuery("PRAGMA journal_mode = OFF;");
				break;
			case 3:
				this->executeNoResultQuery("PRAGMA journal_mode = MEMORY;");
				break;
			case 2:
				this->executeNoResultQuery("PRAGMA journal_mode = PERSIST;");
				break;
			case 1:
				this->executeNoResultQuery("PRAGMA journal_mode = TRUNCATE;");
				break;
			case 0:
			default:
				this->executeNoResultQuery("PRAGMA journal_mode = DELETE;");
				break;
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Wrong journal mode (%d)", journalMode);
	}
}

void DBDriverSqlite3::setSynchronous(int synchronous)
{
	if(synchronous >= 0 && synchronous < 3)
	{
		_synchronous = synchronous;
		if(this->isConnected())
		{
			switch(_synchronous)
			{
			case 0:
				this->executeNoResultQuery("PRAGMA synchronous = OFF;");
				break;
			case 1:
				this->executeNoResultQuery("PRAGMA synchronous = NORMAL;");
				break;
			case 2:
			default:
				this->executeNoResultQuery("PRAGMA synchronous = FULL;");
				break;
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Wrong synchronous value (%d)", synchronous);
	}
}

void DBDriverSqlite3::setTempStore(int tempStore)
{
	if(tempStore >= 0 && tempStore < 3)
	{
		_tempStore = tempStore;
		if(this->isConnected())
		{
			switch(_tempStore)
			{
			case 2:
				this->executeNoResultQuery("PRAGMA temp_store = MEMORY;");
				break;
			case 1:
				this->executeNoResultQuery("PRAGMA temp_store = FILE;");
				break;
			case 0:
			default:
				this->executeNoResultQuery("PRAGMA temp_store = DEFAULT;");
				break;
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Wrong tempStore value (%d)", tempStore);
	}
}

void DBDriverSqlite3::setDbInMemory(bool dbInMemory)
{
	UDEBUG("dbInMemory=%d", dbInMemory?1:0);
	if(dbInMemory != _dbInMemory)
	{
		if(this->isConnected())
		{
			// Hard reset...
			join(true);
			this->emptyTrashes();
			this->closeConnection();
			_dbInMemory = dbInMemory;
			this->openConnection(this->getUrl());
		}
		else
		{
			_dbInMemory = dbInMemory;
		}
	}
}

/*
** This function is used to load the contents of a database file on disk
** into the "main" database of open database connection pInMemory, or
** to save the current contents of the database opened by pInMemory into
** a database file on disk. pInMemory is probably an in-memory database,
** but this function will also work fine if it is not.
**
** Parameter zFilename points to a nul-terminated string containing the
** name of the database file on disk to load from or save to. If parameter
** isSave is non-zero, then the contents of the file zFilename are
** overwritten with the contents of the database opened by pInMemory. If
** parameter isSave is zero, then the contents of the database opened by
** pInMemory are replaced by data loaded from the file zFilename.
**
** If the operation is successful, SQLITE_OK is returned. Otherwise, if
** an error occurs, an SQLite error code is returned.
*/
int DBDriverSqlite3::loadOrSaveDb(sqlite3 *pInMemory, const std::string & fileName, int isSave) const
{
  int rc;                   /* Function return code */
  sqlite3 *pFile = 0;           /* Database connection opened on zFilename */
  sqlite3_backup *pBackup = 0;  /* Backup object used to copy data */
  sqlite3 *pTo = 0;             /* Database to copy to (pFile or pInMemory) */
  sqlite3 *pFrom = 0;           /* Database to copy from (pFile or pInMemory) */

  /* Open the database file identified by zFilename. Exit early if this fails
  ** for any reason. */
  rc = sqlite3_open(fileName.c_str(), &pFile);
  if( rc==SQLITE_OK ){

    /* If this is a 'load' operation (isSave==0), then data is copied
    ** from the database file just opened to database pInMemory.
    ** Otherwise, if this is a 'save' operation (isSave==1), then data
    ** is copied from pInMemory to pFile.  Set the variables pFrom and
    ** pTo accordingly. */
    pFrom = (isSave ? pInMemory : pFile);
    pTo   = (isSave ? pFile     : pInMemory);

    /* Set up the backup procedure to copy from the "main" database of
    ** connection pFile to the main database of connection pInMemory.
    ** If something goes wrong, pBackup will be set to NULL and an error
    ** code and  message left in connection pTo.
    **
    ** If the backup object is successfully created, call backup_step()
    ** to copy data from pFile to pInMemory. Then call backup_finish()
    ** to release resources associated with the pBackup object.  If an
    ** error occurred, then  an error code and message will be left in
    ** connection pTo. If no error occurred, then the error code belonging
    ** to pTo is set to SQLITE_OK.
    */
    pBackup = sqlite3_backup_init(pTo, "main", pFrom, "main");
    if( pBackup ){
      (void)sqlite3_backup_step(pBackup, -1);
      (void)sqlite3_backup_finish(pBackup);
    }
    rc = sqlite3_errcode(pTo);
  }

  /* Close the database connection opened on database file zFilename
  ** and return the result of this function. */
  (void)sqlite3_close(pFile);
  return rc;
}

bool DBDriverSqlite3::getDatabaseVersionQuery(std::string & version) const
{
	version = "0.0.0";
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT version FROM Admin;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc == SQLITE_OK)
		{
			// Process the result if one
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				version = reinterpret_cast<const char*>(sqlite3_column_text(ppStmt, 0));
				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		//else
		//{
			// old version detected
		//}
		return true;
	}
	return false;
}

bool DBDriverSqlite3::connectDatabaseQuery(const std::string & url, bool overwritten)
{
	this->disconnectDatabaseQuery();
	// Open a database connection
	_ppDb = 0;
	_memoryUsedEstimate = 0;

	int rc = SQLITE_OK;
	bool dbFileExist = false;
	if(!url.empty())
	{
		dbFileExist = UFile::exists(url.c_str());
		if(dbFileExist && overwritten)
		{
			UINFO("Deleting database %s...", url.c_str());
			UASSERT(UFile::erase(url.c_str()) == 0);
			dbFileExist = false;
		}
		else if(dbFileExist)
		{
			_memoryUsedEstimate = UFile::length(this->getUrl());
		}
	}

	if(_dbInMemory || url.empty())
	{
		if(!url.empty())
		{
			ULOGGER_INFO("Using database \"%s\" in the memory.", url.c_str());
		}
		else
		{
			ULOGGER_INFO("Using empty database in the memory.");
		}
		rc = sqlite3_open_v2(":memory:", &_ppDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);
	}
	else
	{
		ULOGGER_INFO("Using database \"%s\" from the hard drive.", url.c_str());
		rc = sqlite3_open_v2(url.c_str(), &_ppDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);
	}
	if(rc != SQLITE_OK)
	{
		UFATAL("DB error : %s (path=\"%s\"). Make sure that your user has write " 
			"permission on the target directory (you may have to change the working directory). ", sqlite3_errmsg(_ppDb), url.c_str());
		_ppDb = 0;
		return false;
	}

	if(_dbInMemory && dbFileExist)
	{
		UTimer timer;
		timer.start();
		ULOGGER_DEBUG("Loading DB ...");
		rc = loadOrSaveDb(_ppDb, url, 0); // Load memory from file
		ULOGGER_INFO("Loading DB time = %fs, (%s)", timer.ticks(), url.c_str());
		if(rc != SQLITE_OK)
		{
			UFATAL("DB error 2 : %s", sqlite3_errmsg(_ppDb));
			sqlite3_close(_ppDb);
			_ppDb = 0;
			return false;
		}
	}

	if(!dbFileExist)
	{
		if(!url.empty())
		{
			ULOGGER_INFO("Database \"%s\" doesn't exist, creating a new one...", url.c_str());
		}
		// Create the database
		std::string schema = DATABASESCHEMA_SQL;
		std::string targetVersion = this->getTargetVersion();
		if(!targetVersion.empty())
		{
			// search for schema with version <= target version
			std::vector<std::pair<std::string, std::string> > schemas;
			schemas.push_back(std::make_pair("0.16.0", DATABASESCHEMA_0_16_0_SQL));
			schemas.push_back(std::make_pair("0.16.1", DATABASESCHEMA_0_16_1_SQL));
			schemas.push_back(std::make_pair("0.16.2", DATABASESCHEMA_0_16_2_SQL));
			schemas.push_back(std::make_pair("0.17.0", DATABASESCHEMA_0_17_0_SQL));
			schemas.push_back(std::make_pair("0.18.0", DATABASESCHEMA_0_18_0_SQL));
			schemas.push_back(std::make_pair("0.18.3", DATABASESCHEMA_0_18_3_SQL));
			schemas.push_back(std::make_pair(uNumber2Str(RTABMAP_VERSION_MAJOR)+"."+uNumber2Str(RTABMAP_VERSION_MINOR), DATABASESCHEMA_SQL));
			for(size_t i=0; i<schemas.size(); ++i)
			{
				if(uStrNumCmp(targetVersion, schemas[i].first) < 0)
				{
					if(i==0)
					{
						UERROR("Cannot create database with target version \"%s\" (not implemented), using latest version.", targetVersion.c_str());
					}
					break;
				}
				else
				{
					schema = schemas[i].second;
				}
			}
		}
		schema = uHex2Str(schema);
		this->executeNoResultQuery(schema.c_str());
	}
	UASSERT(this->getDatabaseVersionQuery(_version)); // must be true!
	UINFO("Database version = %s", _version.c_str());

	// From 0.11.13, compare only with minor version (patch will be used for non-database structural changes)
	if((uStrNumCmp(_version, "0.11.12") <= 0 && uStrNumCmp(_version, RTABMAP_VERSION) > 0) ||
	   (uStrNumCmp(_version, "0.11.12") > 0 && uStrNumCmp(RTABMAP_VERSION, "0.11.12") > 0 && uStrNumCmp(_version, uFormat("%d.%d.99", RTABMAP_VERSION_MAJOR, RTABMAP_VERSION_MINOR)) > 0))
	{
			UERROR("Opened database version (%s) is more recent than rtabmap "
				   "installed version (%s). Please update rtabmap to new version!",
				   _version.c_str(), RTABMAP_VERSION);
			this->disconnectDatabaseQuery(false);
			return false;
	}

	//Set database optimizations
	this->setCacheSize(_cacheSize); // this will call the SQL
	this->setJournalMode(_journalMode); // this will call the SQL
	this->setSynchronous(_synchronous); // this will call the SQL
	this->setTempStore(_tempStore); // this will call the SQL

	return true;
}
void DBDriverSqlite3::disconnectDatabaseQuery(bool save, const std::string & outputUrl)
{
	UDEBUG("");
	if(_ppDb)
	{
		int rc = SQLITE_OK;
		// make sure that all statements are finalized
		sqlite3_stmt * pStmt;
		while( (pStmt = sqlite3_next_stmt(_ppDb, 0))!=0 )
		{
			rc = sqlite3_finalize(pStmt);
			if(rc != SQLITE_OK)
			{
				UERROR("");
			}
		}

		if(save && (_dbInMemory || this->getUrl().empty()))
		{
			UTimer timer;
			timer.start();
			std::string outputFile = this->getUrl();
			if(!outputUrl.empty())
			{
				outputFile = outputUrl;
			}
			if(outputFile.empty())
			{
				UWARN("Database was initialized with an empty url (in memory). To save it, "
						"the output url should not be empty. The database is thus closed without being saved!");
			}
			else
			{
				UINFO("Saving database to %s ...",  outputFile.c_str());
				rc = loadOrSaveDb(_ppDb, outputFile, 1); // Save memory to file
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s), could not save \"%s\": %s. Make sure that your user has write " 
					"permission on the target directory (you may have to change the working directory). ", _version.c_str(), outputFile.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				ULOGGER_DEBUG("Saving DB time = %fs", timer.ticks());
			}
		}

		// Then close (delete) the database connection
		UINFO("Disconnecting database %s...", this->getUrl().c_str());
		sqlite3_close(_ppDb);
		_ppDb = 0;

		if(save && !_dbInMemory && !outputUrl.empty() && !this->getUrl().empty() && outputUrl.compare(this->getUrl()) != 0)
		{
			UWARN("Output database path (%s) is different than the opened database "
					"path (%s). Opened database path is overwritten then renamed to output path.",
					outputUrl.c_str(), this->getUrl().c_str());
			if(UFile::rename(this->getUrl(), outputUrl) != 0)
			{
				UERROR("Failed to rename just closed db %s to %s", this->getUrl().c_str(), outputUrl.c_str());
			}
		}
	}
}

bool DBDriverSqlite3::isConnectedQuery() const
{
	return _ppDb != 0;
}

// In bytes
void DBDriverSqlite3::executeNoResultQuery(const std::string & sql) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc;
		rc = sqlite3_exec(_ppDb, sql.c_str(), 0, 0, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s, the query is %s", sqlite3_errmsg(_ppDb), sql.c_str()).c_str());
		UDEBUG("Time=%fs", timer.ticks());
	}
}

unsigned long DBDriverSqlite3::getMemoryUsedQuery() const
{
	if(_dbInMemory)
	{
		return sqlite3_memory_used();
	}
	else
	{
		return _memoryUsedEstimate;
	}
}

long DBDriverSqlite3::getNodesMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.18.0") >= 0)
		{
			query = "SELECT sum(length(id) + length(map_id) + length(weight) + length(pose) + length(stamp) + ifnull(length(label),0) + length(ground_truth_pose) + ifnull(length(velocity),0) + ifnull(length(gps),0) + ifnull(length(env_sensors),0) + length(time_enter)) from Node;";
		}
		else if(uStrNumCmp(_version, "0.14.0") >= 0)
		{
			query = "SELECT sum(length(id) + length(map_id) + length(weight) + length(pose) + length(stamp) + ifnull(length(label),0) + length(ground_truth_pose) + ifnull(length(velocity),0) + ifnull(length(gps),0) + length(time_enter)) from Node;";
		}
		else if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query = "SELECT sum(length(id) + length(map_id) + length(weight) + length(pose) + length(stamp) + ifnull(length(label),0) + length(ground_truth_pose) + ifnull(length(velocity),0) + length(time_enter)) from Node;";
		}
		else if(uStrNumCmp(_version, "0.11.1") >= 0)
		{
			query = "SELECT sum(length(id) + length(map_id) + length(weight) + length(pose) + length(stamp) + ifnull(length(label),0) + length(ground_truth_pose)+ length(time_enter)) from Node;";
		}
		else if(uStrNumCmp(_version, "0.8.5") >= 0)
		{
			query = "SELECT sum(length(id) + length(map_id) + length(weight) + length(pose) + length(stamp) + ifnull(length(label),0) + length(time_enter)) from Node;";
		}
		else
		{
			query = "SELECT sum(length(id) + length(map_id) + length(weight) + length(pose)+ length(time_enter)) from Node;";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getLinksMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query = "SELECT sum(length(type) + length(information_matrix) + length(transform) + ifnull(length(user_data),0) + length(from_id) + length(to_id)) from Link;";
		}
		else if(uStrNumCmp(_version, "0.10.10") >= 0)
		{
			query = "SELECT sum(length(type) + length(rot_variance) + length(trans_variance) + length(transform) + ifnull(length(user_data),0) + length(from_id) + length(to_id)) from Link;";
		}
		else if(uStrNumCmp(_version, "0.8.4") >= 0)
		{
			query = "SELECT sum(length(type) + length(rot_variance) + length(trans_variance) + length(transform) + length(from_id) + length(to_id)) from Link;";
		}
		else if(uStrNumCmp(_version, "0.7.4") >= 0)
		{
			query = "SELECT sum(length(type) + length(variance) + length(transform) + length(from_id) + length(to_id)) from Link;";
		}
		else
		{
			query = "SELECT sum(length(type) + length(transform) + length(from_id) + length(to_id)) from Link;";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getImagesMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.10.0") >= 0)
		{
			query = "SELECT sum(ifnull(length(image),0) + ifnull(length(time_enter),0)) from Data;";
		}
		else
		{
			query = "SELECT sum(length(data) + ifnull(length(time_enter),0)) from Image;";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getDepthImagesMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.10.0") >= 0)
		{
			query = "SELECT sum(ifnull(length(depth),0) + ifnull(length(time_enter),0)) from Data;";
		}
		else
		{
			query = "SELECT sum(length(data) + ifnull(length(time_enter),0)) from Depth;";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getCalibrationsMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;

		if(uStrNumCmp(_version, "0.10.0") >= 0)
		{
			query = "SELECT sum(length(calibration)) from Data;";
		}
		else if(uStrNumCmp(_version, "0.7.0") >= 0)
		{
			query = "SELECT sum(length(fx) + length(fy) + length(cx) + length(cy) + length(local_transform)) from Depth;";
		}
		else
		{
			query = "SELECT sum(length(constant) + length(local_transform)) from Depth;";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getGridsMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;

		if(uStrNumCmp(_version, "0.16.0") >= 0)
		{
			query = "SELECT sum(ifnull(length(ground_cells),0) + ifnull(length(obstacle_cells),0) + ifnull(length(empty_cells),0) + length(cell_size) + length(view_point_x) + length(view_point_y) + length(view_point_z)) from Data;";
		}
		else if(uStrNumCmp(_version, "0.11.10") >= 0)
		{
			query = "SELECT sum(ifnull(length(ground_cells),0) + ifnull(length(obstacle_cells),0) + length(cell_size) + length(view_point_x) + length(view_point_y) + length(view_point_z)) from Data;";
		}
		else
		{
			return size;
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getLaserScansMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;

		if(uStrNumCmp(_version, "0.11.10") >= 0)
		{
			query = "SELECT sum(ifnull(length(scan_info),0) + ifnull(length(scan),0)) from Data;";
		}
		else if(uStrNumCmp(_version, "0.10.7") >= 0)
		{
			query = "SELECT sum(length(scan_max_pts) + length(scan_max_range) + ifnull(length(scan),0)) from Data;";
		}
		else if(uStrNumCmp(_version, "0.10.0") >= 0)
		{
			query = "SELECT sum(length(scan_max_pts) + ifnull(length(scan),0)) from Data;";
		}
		else if(uStrNumCmp(_version, "0.8.11") >= 0)
		{
			query = "SELECT sum(length(data2d) + length(data2d_max_pts)) from Depth;";
		}
		else
		{
			query = "SELECT sum(length(data2d)) from Depth;";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getUserDataMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.10.1") >= 0)
		{
			query = "SELECT sum(length(user_data)) from Data;";
		}
		else if(uStrNumCmp(_version, "0.8.8") >= 0)
		{
			query = "SELECT sum(length(user_data)) from Node;";
		}
		else
		{
			return size; // no user_data
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getWordsMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query = "SELECT sum(length(id) + length(descriptor_size) + length(descriptor) + length(time_enter)) from Word;";

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getFeaturesMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query = "SELECT sum(length(node_id) + length(word_id) + length(pos_x) + length(pos_y) + length(size) + length(dir) + length(response) + length(octave) + ifnull(length(depth_x),0) + ifnull(length(depth_y),0) + ifnull(length(depth_z),0) + ifnull(length(descriptor_size),0) + ifnull(length(descriptor),0)) "
					 "FROM Feature";
		}
		else if(uStrNumCmp(_version, "0.12.0") >= 0)
		{
			query = "SELECT sum(length(node_id) + length(word_id) + length(pos_x) + length(pos_y) + length(size) + length(dir) + length(response) + length(octave) + ifnull(length(depth_x),0) + ifnull(length(depth_y),0) + ifnull(length(depth_z),0) + ifnull(length(descriptor_size),0) + ifnull(length(descriptor),0)) "
					 "FROM Map_Node_Word";
		}
		else if(uStrNumCmp(_version, "0.11.2") >= 0)
		{
			query = "SELECT sum(length(node_id) + length(word_id) + length(pos_x) + length(pos_y) + length(size) + length(dir) + length(response) + ifnull(length(depth_x),0) + ifnull(length(depth_y),0) + ifnull(length(depth_z),0) + ifnull(length(descriptor_size),0) + ifnull(length(descriptor),0)) "
					 "FROM Map_Node_Word";
		}
		else
		{
			query = "SELECT sum(length(node_id) + length(word_id) + length(pos_x) + length(pos_y) + length(size) + length(dir) + length(response) + ifnull(length(depth_x),0) + ifnull(length(depth_y),0) + ifnull(length(depth_z),0) "
					 "FROM Map_Node_Word";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
long DBDriverSqlite3::getStatisticsMemoryUsedQuery() const
{
	UDEBUG("");
	long size = 0L;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.16.2") >= 0)
		{
			query = "SELECT sum(length(id) + length(stamp) + ifnull(length(data),0) + ifnull(length(wm_state),0)) FROM Statistics;";
		}
		else if(uStrNumCmp(_version, "0.11.11") >= 0)
		{
			query = "SELECT sum(length(id) + length(stamp) + length(data)) FROM Statistics;";
		}
		else
		{
			return size;
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int64(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
int DBDriverSqlite3::getLastNodesSizeQuery() const
{
	UDEBUG("");
	int size = 0;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.11.11") >= 0)
		{
			query = "SELECT count(id) from Node WHERE time_enter >= (SELECT MAX(time_enter) FROM Info);";
		}
		else
		{
			query = "SELECT count(id) from Node WHERE time_enter >= (SELECT MAX(time_enter) FROM Statistics);";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
int DBDriverSqlite3::getLastDictionarySizeQuery() const
{
	UDEBUG("");
	int size = 0;
	if(_ppDb)
	{
		std::string query;
		if(uStrNumCmp(_version, "0.11.11") >= 0)
		{
			query = "SELECT count(id) from Word WHERE time_enter >= (SELECT MAX(time_enter) FROM Info);";
		}
		else
		{
			query = "SELECT count(id) from Word WHERE time_enter >= (SELECT MAX(time_enter) FROM Statistics);";
		}

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
int DBDriverSqlite3::getTotalNodesSizeQuery() const
{
	UDEBUG("");
	int size = 0;
	if(_ppDb)
	{
		std::string query = "SELECT count(id) from Node;";

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}
int DBDriverSqlite3::getTotalDictionarySizeQuery() const
{
	UDEBUG("");
	int size = 0;
	if(_ppDb)
	{
		std::string query = "SELECT count(id) from Word;";

		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			size = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return size;
}

ParametersMap DBDriverSqlite3::getLastParametersQuery() const
{
	UDEBUG("");
	ParametersMap parameters;
	if(_ppDb)
	{
		if(uStrNumCmp(_version, "0.11.8") >= 0)
		{
			std::string query;
			if(uStrNumCmp(_version, "0.11.11") >= 0)
			{
				query = "SELECT parameters "
						 "FROM Info "
						 "WHERE time_enter >= (SELECT MAX(time_enter) FROM Info);";
			}
			else
			{
				query = "SELECT parameters "
						 "FROM Statistics "
						 "WHERE time_enter >= (SELECT MAX(time_enter) FROM Statistics);";
			}

			int rc = SQLITE_OK;
			sqlite3_stmt * ppStmt = 0;
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				std::string text((const char *)sqlite3_column_text(ppStmt, 0));

				if(text.size())
				{
					parameters = Parameters::deserialize(text);
				}

				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}
	return parameters;
}

std::map<std::string, float> DBDriverSqlite3::getStatisticsQuery(int nodeId, double & stamp, std::vector<int> * wmState) const
{
	UDEBUG("nodeId=%d", nodeId);
	std::map<std::string, float> data;
	if(_ppDb)
	{
		if(uStrNumCmp(_version, "0.11.11") >= 0)
		{
			std::stringstream query;

			if(uStrNumCmp(_version, "0.16.2") >= 0 && wmState)
			{
				query << "SELECT stamp, data, wm_state "
					  << "FROM Statistics "
					  << "WHERE id=" << nodeId << ";";
			}
			else
			{
				query << "SELECT stamp, data "
					  << "FROM Statistics "
					  << "WHERE id=" << nodeId << ";";
			}

			int rc = SQLITE_OK;
			sqlite3_stmt * ppStmt = 0;
			rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				int index = 0;
				stamp = sqlite3_column_double(ppStmt, index++);

				std::string text;
				if(uStrNumCmp(this->getDatabaseVersion(), "0.15.0") >= 0)
				{
					const void * dataPtr = sqlite3_column_blob(ppStmt, index);
					int dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize>0 && dataPtr)
					{
						text = uncompressString(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
					}
				}
				else
				{
					text = (const char *)sqlite3_column_text(ppStmt, index++);
				}

				if(text.size())
				{
					data = Statistics::deserializeData(text);
				}

				if(uStrNumCmp(_version, "0.16.2") >= 0 && wmState)
				{
					const void * dataPtr = sqlite3_column_blob(ppStmt, index);
					int dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize>0 && dataPtr)
					{
						cv::Mat wmStateMat = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
						UASSERT(wmStateMat.type() == CV_32SC1 && wmStateMat.rows == 1);
						wmState->resize(wmStateMat.cols);
						memcpy(wmState->data(), wmStateMat.data, wmState->size()*sizeof(int));
					}
				}

				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}
	return data;
}

std::map<int, std::pair<std::map<std::string, float>, double> > DBDriverSqlite3::getAllStatisticsQuery() const
{
	UDEBUG("");
	std::map<int, std::pair<std::map<std::string, float>, double> > data;
	if(_ppDb)
	{
		if(uStrNumCmp(_version, "0.11.11") >= 0)
		{
			std::stringstream query;

			query << "SELECT id, stamp, data "
				  << "FROM Statistics;";

			int rc = SQLITE_OK;
			sqlite3_stmt * ppStmt = 0;
			rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;
				int id = sqlite3_column_int(ppStmt, index++);
				double stamp = sqlite3_column_double(ppStmt, index++);

				std::string text;
				if(uStrNumCmp(this->getDatabaseVersion(), "0.15.0") >= 0)
				{
					const void * dataPtr = 0;
					int dataSize = 0;
					dataPtr = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize>0 && dataPtr)
					{
						text = uncompressString(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
					}
				}
				else
				{
					text = (const char *)sqlite3_column_text(ppStmt, index++);
				}

				if(text.size())
				{
					data.insert(std::make_pair(id, std::make_pair(Statistics::deserializeData(text), stamp)));
				}

				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}
	UDEBUG("");
	return data;
}

std::map<int, std::vector<int> > DBDriverSqlite3::getAllStatisticsWmStatesQuery() const
{
	UDEBUG("");
	std::map<int, std::vector<int> > data;
	if(_ppDb)
	{
		if(uStrNumCmp(_version, "0.16.2") >= 0)
		{
			std::stringstream query;

			query << "SELECT id, wm_state "
				  << "FROM Statistics;";

			int rc = SQLITE_OK;
			sqlite3_stmt * ppStmt = 0;
			rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;
				int id = sqlite3_column_int(ppStmt, index++);

				std::vector<int> wmState;
				const void * dataPtr = sqlite3_column_blob(ppStmt, index);
				int dataSize = sqlite3_column_bytes(ppStmt, index++);
				if(dataSize>0 && dataPtr)
				{
					cv::Mat wmStateMat = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
					UASSERT(wmStateMat.type() == CV_32SC1 && wmStateMat.rows == 1);
					wmState.resize(wmStateMat.cols);
					memcpy(wmState.data(), wmStateMat.data, wmState.size()*sizeof(int));
				}

				if(!wmState.empty())
				{
					data.insert(std::make_pair(id, wmState));
				}

				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}
	UDEBUG("");
	return data;
}

void DBDriverSqlite3::loadNodeDataQuery(std::list<Signature *> & signatures, bool images, bool scan, bool userData, bool occupancyGrid) const
{
	UDEBUG("load data for %d signatures images=%d scan=%d userData=%d, grid=%d",
			(int)signatures.size(), images?1:0, scan?1:0, userData?1:0, occupancyGrid?1:0);

	if(!images && !scan && !userData && !occupancyGrid)
	{
		UWARN("All requested data fields are false! Nothing loaded...");
		return;
	}

	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.11.10") >= 0)
		{
			std::stringstream fields;

			if(images)
			{
				fields << "image, depth, calibration";
				if(scan || userData || occupancyGrid)
				{
					fields << ", ";
				}
			}
			if(scan)
			{
				fields << "scan_info, scan";
				if(userData || occupancyGrid)
				{
					fields << ", ";
				}
			}
			if(userData)
			{
				fields << "user_data";
				if(occupancyGrid)
				{
					fields << ", ";
				}
			}
			if(occupancyGrid)
			{
				if(uStrNumCmp(_version, "0.16.0") >= 0)
				{
					fields << "ground_cells, obstacle_cells, empty_cells, cell_size, view_point_x, view_point_y, view_point_z";
				}
				else
				{
					fields << "ground_cells, obstacle_cells, cell_size, view_point_x, view_point_y, view_point_z";
				}
			}

			query << "SELECT " << fields.str().c_str() << " "
				  << "FROM Data "
				  << "WHERE id = ?"
				  <<";";
		}
		else if(uStrNumCmp(_version, "0.10.7") >= 0)
		{
			query << "SELECT image, depth, calibration, scan_max_pts, scan_max_range, scan, user_data "
				  << "FROM Data "
				  << "WHERE id = ?"
				  <<";";
		}
		else if(uStrNumCmp(_version, "0.10.1") >= 0)
		{
			query << "SELECT image, depth, calibration, scan_max_pts, scan, user_data "
				  << "FROM Data "
				  << "WHERE id = ?"
				  <<";";
		}
		else if(uStrNumCmp(_version, "0.10.0") >= 0)
		{
			query << "SELECT Data.image, Data.depth, Data.calibration, Data.scan_max_pts, Data.scan, Node.user_data "
				  << "FROM Data "
				  << "INNER JOIN Node "
				  << "ON Data.id = Node.id "
				  << "WHERE Data.id = ?"
				  <<";";
		}
		else if(uStrNumCmp(_version, "0.8.11") >= 0)
		{
			query << "SELECT Image.data, "
					 "Depth.data, Depth.local_transform, Depth.fx, Depth.fy, Depth.cx, Depth.cy, Depth.data2d_max_pts, Depth.data2d, Node.user_data "
				  << "FROM Image "
				  << "INNER JOIN Node "
				  << "on Image.id = Node.id "
				  << "LEFT OUTER JOIN Depth " // returns all images even if there are no metric data
				  << "ON Image.id = Depth.id "
				  << "WHERE Image.id = ?"
				  <<";";
		}
		else if(uStrNumCmp(_version, "0.8.8") >= 0)
		{
			query << "SELECT Image.data, "
					 "Depth.data, Depth.local_transform, Depth.fx, Depth.fy, Depth.cx, Depth.cy, Depth.data2d, Node.user_data "
				  << "FROM Image "
				  << "INNER JOIN Node "
				  << "on Image.id = Node.id "
				  << "LEFT OUTER JOIN Depth " // returns all images even if there are no metric data
				  << "ON Image.id = Depth.id "
				  << "WHERE Image.id = ?"
				  <<";";
		}
		else if(uStrNumCmp(_version, "0.7.0") >= 0)
		{
			query << "SELECT Image.data, "
					 "Depth.data, Depth.local_transform, Depth.fx, Depth.fy, Depth.cx, Depth.cy, Depth.data2d "
				  << "FROM Image "
				  << "LEFT OUTER JOIN Depth " // returns all images even if there are no metric data
				  << "ON Image.id = Depth.id "
				  << "WHERE Image.id = ?"
				  <<";";
		}
		else
		{
			query << "SELECT Image.data, "
					 "Depth.data, Depth.local_transform, Depth.constant, Depth.data2d "
				  << "FROM Image "
				  << "LEFT OUTER JOIN Depth " // returns all images even if there are no metric data
				  << "ON Image.id = Depth.id "
				  << "WHERE Image.id = ?"
				  <<";";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		const void * data = 0;
		int dataSize = 0;
		int index = 0;

		for(std::list<Signature*>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
		{
			UASSERT(*iter != 0);

			ULOGGER_DEBUG("Loading data for %d...", (*iter)->id());
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				index = 0;

				cv::Mat imageCompressed;
				cv::Mat depthOrRightCompressed;
				std::vector<CameraModel> models;
				std::vector<StereoCameraModel> stereoModels;
				Transform localTransform = Transform::getIdentity();
				cv::Mat scanCompressed;
				cv::Mat userDataCompressed;

				if(uStrNumCmp(_version, "0.11.10") < 0 || images)
				{
					//Create the image
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize>4 && data)
					{
						imageCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone();
					}

					//Create the depth image
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize>4 && data)
					{
						depthOrRightCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone();
					}

					if(uStrNumCmp(_version, "0.10.0") < 0)
					{
						data = sqlite3_column_blob(ppStmt, index); // local transform
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						if((unsigned int)dataSize == localTransform.size()*sizeof(float) && data)
						{
							memcpy(localTransform.data(), data, dataSize);
							if(uStrNumCmp(_version, "0.15.2") < 0)
							{
								localTransform.normalizeRotation();
							}
						}
					}

					// calibration
					if(uStrNumCmp(_version, "0.10.0") >= 0)
					{
						data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						// multi-cameras [fx,fy,cx,cy,[width,height],local_transform, ... ,fx,fy,cx,cy,[width,height],local_transform] (4or6+12)*float * numCameras
						// stereo [fx, fy, cx, cy, baseline, local_transform] (5+12)*float
						if(dataSize > 0 && data)
						{
							if(uStrNumCmp(_version, "0.18.0") >= 0)
							{
								if(dataSize >= int(sizeof(int)*4))
								{
									const int * dataInt = (const int *)data;
									int type = dataInt[3];
									if(type == 0) // mono
									{
										CameraModel model;
										int bytesReadTotal = 0;
										unsigned int bytesRead = 0;
										while(bytesReadTotal < dataSize &&
											  (bytesRead=model.deserialize((const unsigned char *)data+bytesReadTotal, dataSize-bytesReadTotal))!=0)
										{
											bytesReadTotal+=bytesRead;
											models.push_back(model);
										}
										UASSERT(bytesReadTotal == dataSize);
									}
									else if(type == 1) // stereo
									{
										StereoCameraModel model;
										int bytesReadTotal = 0;
										unsigned int bytesRead = 0;
										while(bytesReadTotal < dataSize &&
											  (bytesRead=model.deserialize((const unsigned char *)data+bytesReadTotal, dataSize-bytesReadTotal))!=0)
										{
											bytesReadTotal+=bytesRead;
											stereoModels.push_back(model);
										}
										UASSERT(bytesReadTotal == dataSize);
									}
									else
									{
										UFATAL("Unknown calibration type %d", type);
									}
								}
								else
								{
									UFATAL("Wrong format of the Data.calibration field (size=%d bytes)", dataSize);
								}
							}
							else
							{
								float * dataFloat = (float*)data;
								if(uStrNumCmp(_version, "0.11.2") >= 0 &&
								   (unsigned int)dataSize % (6+localTransform.size())*sizeof(float) == 0)
								{
									int cameraCount = dataSize / ((6+localTransform.size())*sizeof(float));
									UDEBUG("Loading calibration for %d cameras (%d bytes)", cameraCount, dataSize);
									int max = cameraCount*(6+localTransform.size());
									for(int i=0; i<max; i+=6+localTransform.size())
									{
										// Reinitialize to a new Transform, to avoid copying in the same memory than the previous one
										localTransform = Transform::getIdentity();
										memcpy(localTransform.data(), dataFloat+i+6, localTransform.size()*sizeof(float));
										if(uStrNumCmp(_version, "0.15.2") < 0)
										{
											localTransform.normalizeRotation();
										}
										models.push_back(CameraModel(
												(double)dataFloat[i],
												(double)dataFloat[i+1],
												(double)dataFloat[i+2],
												(double)dataFloat[i+3],
												localTransform));
										models.back().setImageSize(cv::Size(dataFloat[i+4], dataFloat[i+5]));
										UDEBUG("%f %f %f %f %f %f %s", dataFloat[i], dataFloat[i+1], dataFloat[i+2],
												dataFloat[i+3], dataFloat[i+4], dataFloat[i+5],
												localTransform.prettyPrint().c_str());
									}
								}
								else if(uStrNumCmp(_version, "0.11.2") < 0 &&
										(unsigned int)dataSize % (4+localTransform.size())*sizeof(float) == 0)
								{
									int cameraCount = dataSize / ((4+localTransform.size())*sizeof(float));
									UDEBUG("Loading calibration for %d cameras (%d bytes)", cameraCount, dataSize);
									int max = cameraCount*(4+localTransform.size());
									for(int i=0; i<max; i+=4+localTransform.size())
									{
										// Reinitialize to a new Transform, to avoid copying in the same memory than the previous one
										localTransform = Transform::getIdentity();
										memcpy(localTransform.data(), dataFloat+i+4, localTransform.size()*sizeof(float));
										if(uStrNumCmp(_version, "0.15.2") < 0)
										{
											localTransform.normalizeRotation();
										}
										models.push_back(CameraModel(
												(double)dataFloat[i],
												(double)dataFloat[i+1],
												(double)dataFloat[i+2],
												(double)dataFloat[i+3],
												localTransform));
									}
								}
								else if((unsigned int)dataSize == (7+localTransform.size())*sizeof(float))
								{
									UDEBUG("Loading calibration of a stereo camera");
									memcpy(localTransform.data(), dataFloat+7, localTransform.size()*sizeof(float));
									if(uStrNumCmp(_version, "0.15.2") < 0)
									{
										localTransform.normalizeRotation();
									}
									stereoModels.push_back(StereoCameraModel(
											dataFloat[0],  // fx
											dataFloat[1],  // fy
											dataFloat[2],  // cx
											dataFloat[3],  // cy
											dataFloat[4], // baseline
											localTransform,
											cv::Size(dataFloat[5],dataFloat[6])));
								}
								else if((unsigned int)dataSize == (5+localTransform.size())*sizeof(float))
								{
									UDEBUG("Loading calibration of a stereo camera");
									memcpy(localTransform.data(), dataFloat+5, localTransform.size()*sizeof(float));
									if(uStrNumCmp(_version, "0.15.2") < 0)
									{
										localTransform.normalizeRotation();
									}
									stereoModels.push_back(StereoCameraModel(
											dataFloat[0],  // fx
											dataFloat[1],  // fy
											dataFloat[2],  // cx
											dataFloat[3],  // cy
											dataFloat[4], // baseline
											localTransform));
								}
								else
								{
									UFATAL("Wrong format of the Data.calibration field (size=%d bytes)", dataSize);
								}
							}
						}

					}
					else if(uStrNumCmp(_version, "0.7.0") >= 0)
					{
						UDEBUG("Loading calibration version >= 0.7.0");
						double fx = sqlite3_column_double(ppStmt, index++);
						double fyOrBaseline = sqlite3_column_double(ppStmt, index++);
						double cx = sqlite3_column_double(ppStmt, index++);
						double cy = sqlite3_column_double(ppStmt, index++);
						if(fyOrBaseline < 1.0)
						{
							//it is a baseline
							stereoModels.push_back(StereoCameraModel(fx,fx,cx,cy,fyOrBaseline, localTransform));
						}
						else
						{
							models.push_back(CameraModel(fx, fyOrBaseline, cx, cy, localTransform));
						}
					}
					else
					{
						UDEBUG("Loading calibration version < 0.7.0");
						float depthConstant = sqlite3_column_double(ppStmt, index++);
						float fx = 1.0f/depthConstant;
						float fy = 1.0f/depthConstant;
						float cx = 0.0f;
						float cy = 0.0f;
						models.push_back(CameraModel(fx, fy, cx, cy, localTransform));
					}
				}

				int laserScanMaxPts = 0;
				float laserScanMinRange = 0.0f;
				float laserScanMaxRange = 0.0f;
				int laserScanFormat = 0;
				float laserScanAngleMin = 0.0f;
				float laserScanAngleMax = 0.0f;
				float laserScanAngleInc = 0.0f;
				Transform scanLocalTransform = Transform::getIdentity();
				if(uStrNumCmp(_version, "0.11.10") < 0 || scan)
				{
					// scan_info
					if(uStrNumCmp(_version, "0.11.10") >= 0)
					{
						data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);

						if(dataSize > 0 && data)
						{
							float * dataFloat = (float*)data;

							if(uStrNumCmp(_version, "0.18.0") >= 0)
							{
								UASSERT(dataSize == (int)((scanLocalTransform.size()+7)*sizeof(float)));
								laserScanFormat = (int)dataFloat[0];
								laserScanMinRange = dataFloat[1];
								laserScanMaxRange = dataFloat[2];
								laserScanAngleMin = dataFloat[3];
								laserScanAngleMax = dataFloat[4];
								laserScanAngleInc = dataFloat[5];
								laserScanMaxPts = (int)dataFloat[6];
								memcpy(scanLocalTransform.data(), dataFloat+7, scanLocalTransform.size()*sizeof(float));
							}
							else
							{
								if(uStrNumCmp(_version, "0.16.1") >= 0 && dataSize == (int)((scanLocalTransform.size()+3)*sizeof(float)))
								{
									// new in 0.16.1
									laserScanFormat = (int)dataFloat[2];
									memcpy(scanLocalTransform.data(), dataFloat+3, scanLocalTransform.size()*sizeof(float));
								}
								else if(dataSize == (int)((scanLocalTransform.size()+2)*sizeof(float)))
								{
									memcpy(scanLocalTransform.data(), dataFloat+2, scanLocalTransform.size()*sizeof(float));
								}
								else
								{
									UFATAL("Unexpected size %d for laser scan info!", dataSize);
								}

								if(uStrNumCmp(_version, "0.15.2") < 0)
								{
									scanLocalTransform.normalizeRotation();
								}
								laserScanMaxPts = (int)dataFloat[0];
								laserScanMaxRange = dataFloat[1];
							}
						}
					}
					else
					{
						if(uStrNumCmp(_version, "0.8.11") >= 0)
						{
							laserScanMaxPts = sqlite3_column_int(ppStmt, index++);
						}

						if(uStrNumCmp(_version, "0.10.7") >= 0)
						{
							laserScanMaxRange = sqlite3_column_int(ppStmt, index++);
						}
					}

					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					//Create the laserScan
					if(dataSize>4 && data)
					{
						scanCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone(); // depth2d
					}
				}

				if(uStrNumCmp(_version, "0.11.10") < 0 || userData)
				{
					if(uStrNumCmp(_version, "0.8.8") >= 0)
					{
						data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						//Create the userData
						if(dataSize>4 && data)
						{
							if(uStrNumCmp(_version, "0.10.1") >= 0)
							{
								userDataCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone(); // userData
							}
							else
							{
								// compress data (set uncompressed data to signed to make difference with compressed type)
								userDataCompressed = compressData2(cv::Mat(1, dataSize, CV_8SC1, (void *)data));
							}
						}
					}
				}

				// Occupancy grid
				cv::Mat groundCellsCompressed;
				cv::Mat obstacleCellsCompressed;
				cv::Mat emptyCellsCompressed;
				float cellSize = 0.0f;
				cv::Point3f viewPoint;
				if(uStrNumCmp(_version, "0.11.10") >= 0 && occupancyGrid)
				{
					// ground
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize > 0 && data)
					{
						groundCellsCompressed = cv::Mat(1, dataSize, CV_8UC1);
						memcpy((void*)groundCellsCompressed.data, data, dataSize);
					}

					// obstacle
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize > 0 && data)
					{
						obstacleCellsCompressed = cv::Mat(1, dataSize, CV_8UC1);
						memcpy((void*)obstacleCellsCompressed.data, data, dataSize);
					}

					if(uStrNumCmp(_version, "0.16.0") >= 0)
					{
						// empty
						data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						if(dataSize > 0 && data)
						{
							emptyCellsCompressed = cv::Mat(1, dataSize, CV_8UC1);
							memcpy((void*)emptyCellsCompressed.data, data, dataSize);
						}
					}

					cellSize = sqlite3_column_double(ppStmt, index++);
					viewPoint.x = sqlite3_column_double(ppStmt, index++);
					viewPoint.y = sqlite3_column_double(ppStmt, index++);
					viewPoint.z = sqlite3_column_double(ppStmt, index++);
				}

				if(scan)
				{
					LaserScan laserScan;
					if(laserScanAngleMin < laserScanAngleMax && laserScanAngleInc != 0.0f)
					{
						laserScan = LaserScan(scanCompressed, (LaserScan::Format)laserScanFormat, laserScanMinRange, laserScanMaxRange, laserScanAngleMin, laserScanAngleMax, laserScanAngleInc, scanLocalTransform);
					}
					else
					{
						laserScan = LaserScan(scanCompressed, laserScanMaxPts, laserScanMaxRange, (LaserScan::Format)laserScanFormat, scanLocalTransform);
					}
					(*iter)->sensorData().setLaserScan(laserScan);
				}
				if(images)
				{
					if(models.size())
					{
						(*iter)->sensorData().setRGBDImage(imageCompressed, depthOrRightCompressed, models);
					}
					else
					{
						(*iter)->sensorData().setStereoImage(imageCompressed, depthOrRightCompressed, stereoModels);
					}
				}
				if(userData)
				{
					(*iter)->sensorData().setUserData(userDataCompressed);
				}

				if(occupancyGrid)
				{
					(*iter)->sensorData().setOccupancyGrid(groundCellsCompressed, obstacleCellsCompressed, emptyCellsCompressed, cellSize, viewPoint);
				}

				rc = sqlite3_step(ppStmt); // next result...
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			//reset
			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
	}
}

bool DBDriverSqlite3::getCalibrationQuery(
		int signatureId,
		std::vector<CameraModel> & models,
		std::vector<StereoCameraModel> & stereoModels) const
{
	bool found = false;
	if(_ppDb && signatureId)
	{
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.10.0") >= 0)
		{
			query << "SELECT calibration "
				  << "FROM Data "
				  << "WHERE id = " << signatureId
				  <<";";
		}
		else if(uStrNumCmp(_version, "0.7.0") >= 0)
		{
			query << "SELECT local_transform, fx, fy, cx, cy "
				  << "FROM Depth "
				  << "WHERE id = " << signatureId
				  <<";";
		}
		else
		{
			query << "SELECT local_transform, constant "
				  << "FROM Depth "
				  << "WHERE id = " << signatureId
				  <<";";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		const void * data = 0;
		int dataSize = 0;
		Transform localTransform = Transform::getIdentity();

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			found = true;
			int index = 0;

			// calibration
			if(uStrNumCmp(_version, "0.10.0") < 0)
			{
				data = sqlite3_column_blob(ppStmt, index); // local transform
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				if((unsigned int)dataSize == localTransform.size()*sizeof(float) && data)
				{
					memcpy(localTransform.data(), data, dataSize);
					localTransform.normalizeRotation();
				}
			}

			if(uStrNumCmp(_version, "0.10.0") >= 0)
			{
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				// multi-cameras [fx,fy,cx,cy,[width,height],local_transform, ... ,fx,fy,cx,cy,[width,height],local_transform] (4or6+12)*float * numCameras
				// stereo [fx, fy, cx, cy, baseline, local_transform] (5+12)*float
				if(dataSize > 0 && data)
				{
					if(uStrNumCmp(_version, "0.18.0") >= 0)
					{
						if(dataSize >= int(sizeof(int)*4))
						{
							const int * dataInt = (const int *)data;
							int type = dataInt[3];
							if(type == 0) // mono
							{
								CameraModel model;
								int bytesReadTotal = 0;
								unsigned int bytesRead = 0;
								while(bytesReadTotal < dataSize &&
									  (bytesRead=model.deserialize((const unsigned char *)data+bytesReadTotal, dataSize-bytesReadTotal))!=0)
								{
									bytesReadTotal+=bytesRead;
									models.push_back(model);
								}
								UASSERT(bytesReadTotal == dataSize);
							}
							else if(type == 1) // stereo
							{
								StereoCameraModel model;
								int bytesReadTotal = 0;
								unsigned int bytesRead = 0;
								while(bytesReadTotal < dataSize &&
									  (bytesRead=model.deserialize((const unsigned char *)data+bytesReadTotal, dataSize-bytesReadTotal))!=0)
								{
									bytesReadTotal+=bytesRead;
									stereoModels.push_back(model);
								}
								UASSERT(bytesReadTotal == dataSize);
							}
							else
							{
								UFATAL("Unknown calibration type %d", type);
							}
						}
						else
						{
							UFATAL("Wrong format of the Data.calibration field (size=%d bytes)", dataSize);
						}
					}
					else
					{
						float * dataFloat = (float*)data;
						if(uStrNumCmp(_version, "0.11.2") >= 0 &&
						  (unsigned int)dataSize % (6+localTransform.size())*sizeof(float) == 0)
						{
							int cameraCount = dataSize / ((6+localTransform.size())*sizeof(float));
							UDEBUG("Loading calibration for %d cameras (%d bytes)", cameraCount, dataSize);
							int max = cameraCount*(6+localTransform.size());
							for(int i=0; i<max; i+=6+localTransform.size())
							{
								// Reinitialize to a new Transform, to avoid copying in the same memory than the previous one
								localTransform = Transform::getIdentity();
								memcpy(localTransform.data(), dataFloat+i+6, localTransform.size()*sizeof(float));
								if(uStrNumCmp(_version, "0.15.2") < 0)
								{
									localTransform.normalizeRotation();
								}
								models.push_back(CameraModel(
										(double)dataFloat[i],
										(double)dataFloat[i+1],
										(double)dataFloat[i+2],
										(double)dataFloat[i+3],
										localTransform));
								models.back().setImageSize(cv::Size(dataFloat[i+4], dataFloat[i+5]));
								UDEBUG("%f %f %f %f %f %f %s", dataFloat[i], dataFloat[i+1], dataFloat[i+2],
										dataFloat[i+3], dataFloat[i+4], dataFloat[i+5],
										localTransform.prettyPrint().c_str());
							}
						}
						else if(uStrNumCmp(_version, "0.11.2") < 0 &&
								(unsigned int)dataSize % (4+localTransform.size())*sizeof(float) == 0)
						{
							int cameraCount = dataSize / ((4+localTransform.size())*sizeof(float));
							UDEBUG("Loading calibration for %d cameras (%d bytes)", cameraCount, dataSize);
							int max = cameraCount*(4+localTransform.size());
							for(int i=0; i<max; i+=4+localTransform.size())
							{
								// Reinitialize to a new Transform, to avoid copying in the same memory than the previous one
								localTransform = Transform::getIdentity();
								memcpy(localTransform.data(), dataFloat+i+4, localTransform.size()*sizeof(float));
								if(uStrNumCmp(_version, "0.15.2") < 0)
								{
									localTransform.normalizeRotation();
								}
								models.push_back(CameraModel(
										(double)dataFloat[i],
										(double)dataFloat[i+1],
										(double)dataFloat[i+2],
										(double)dataFloat[i+3],
										localTransform));
							}
						}
						else if((unsigned int)dataSize == (7+localTransform.size())*sizeof(float))
						{
							UDEBUG("Loading calibration of a stereo camera");
							memcpy(localTransform.data(), dataFloat+7, localTransform.size()*sizeof(float));
							if(uStrNumCmp(_version, "0.15.2") < 0)
							{
								localTransform.normalizeRotation();
							}
							stereoModels.push_back(StereoCameraModel(
									dataFloat[0],  // fx
									dataFloat[1],  // fy
									dataFloat[2],  // cx
									dataFloat[3],  // cy
									dataFloat[4], // baseline
									localTransform,
									cv::Size(dataFloat[5],dataFloat[6])));
						}
						else if((unsigned int)dataSize == (5+localTransform.size())*sizeof(float))
						{
							UDEBUG("Loading calibration of a stereo camera");
							memcpy(localTransform.data(), dataFloat+5, localTransform.size()*sizeof(float));
							if(uStrNumCmp(_version, "0.15.2") < 0)
							{
								localTransform.normalizeRotation();
							}
							stereoModels.push_back((StereoCameraModel(
									dataFloat[0],  // fx
									dataFloat[1],  // fy
									dataFloat[2],  // cx
									dataFloat[3],  // cy
									dataFloat[4], // baseline
									localTransform)));
						}
						else
						{
							UFATAL("Wrong format of the Data.calibration field (size=%d bytes)", dataSize);
						}
					}
				}

			}
			else if(uStrNumCmp(_version, "0.7.0") >= 0)
			{
				UDEBUG("Loading calibration version >= 0.7.0");
				double fx = sqlite3_column_double(ppStmt, index++);
				double fyOrBaseline = sqlite3_column_double(ppStmt, index++);
				double cx = sqlite3_column_double(ppStmt, index++);
				double cy = sqlite3_column_double(ppStmt, index++);
				UDEBUG("fx=%f fyOrBaseline=%f cx=%f cy=%f", fx, fyOrBaseline, cx, cy);
				if(fyOrBaseline < 1.0)
				{
					//it is a baseline
					stereoModels.push_back(StereoCameraModel(fx,fx,cx,cy,fyOrBaseline, localTransform));
				}
				else
				{
					models.push_back(CameraModel(fx, fyOrBaseline, cx, cy, localTransform));
				}
			}
			else
			{
				UDEBUG("Loading calibration version < 0.7.0");
				float depthConstant = sqlite3_column_double(ppStmt, index++);
				float fx = 1.0f/depthConstant;
				float fy = 1.0f/depthConstant;
				float cx = 0.0f;
				float cy = 0.0f;
				models.push_back(CameraModel(fx, fy, cx, cy, localTransform));
			}

			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return found;
}

bool DBDriverSqlite3::getLaserScanInfoQuery(
		int signatureId,
		LaserScan & info) const
{
	bool found = false;
	if(_ppDb && signatureId)
	{
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.11.10") >= 0)
		{
			query << "SELECT scan_info "
				  << "FROM Data "
				  << "WHERE id = " << signatureId
				  <<";";
		}
		else
		{
			return false;
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		const void * data = 0;
		int dataSize = 0;
		Transform localTransform = Transform::getIdentity();
		int format = 0;
		int maxPts = 0;
		float minRange = 0.0f;
		float maxRange = 0.0f;
		float angleMin = 0.0f;
		float angleMax = 0.0f;
		float angleInc = 0.0f;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			found = true;
			int index = 0;

			// scan_info
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);

			if(dataSize > 0 && data)
			{
				float * dataFloat = (float*)data;
				if(uStrNumCmp(_version, "0.18.0") >= 0)
				{
					UASSERT(dataSize == (int)((localTransform.size()+7)*sizeof(float)));
					format = (int)dataFloat[0];
					minRange = dataFloat[1];
					maxRange = dataFloat[2];
					angleMin = dataFloat[3];
					angleMax = dataFloat[4];
					angleInc = dataFloat[5];
					maxPts = (int)dataFloat[6];
					memcpy(localTransform.data(), dataFloat+7, localTransform.size()*sizeof(float));
				}
				else
				{
					if(uStrNumCmp(_version, "0.16.1") >= 0 && dataSize == (int)((localTransform.size()+3)*sizeof(float)))
					{
						// new in 0.16.1
						format = (int)dataFloat[2];
						memcpy(localTransform.data(), dataFloat+3, localTransform.size()*sizeof(float));
					}
					else if(dataSize == (int)((localTransform.size()+2)*sizeof(float)))
					{
						memcpy(localTransform.data(), dataFloat+2, localTransform.size()*sizeof(float));
					}
					else
					{
						UFATAL("Unexpected size %d for laser scan info!", dataSize);
					}
					if(uStrNumCmp(_version, "0.15.2") < 0)
					{
						localTransform.normalizeRotation();
					}
					maxPts = (int)dataFloat[0];
					maxRange = dataFloat[1];
				}

				if(angleInc != 0.0f && angleMin < angleMax)
				{
					info = LaserScan(cv::Mat(), (LaserScan::Format)format, minRange, maxRange, angleMin, angleMax, angleInc, localTransform);
				}
				else
				{
					info = LaserScan(cv::Mat(), maxPts, maxRange, (LaserScan::Format)format, localTransform);
				}
			}

			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return found;
}

bool DBDriverSqlite3::getNodeInfoQuery(int signatureId,
		Transform & pose,
		int & mapId,
		int & weight,
		std::string & label,
		double & stamp,
		Transform & groundTruthPose,
		std::vector<float> & velocity,
		GPS & gps,
		EnvSensors & sensors) const
{
	bool found = false;
	if(_ppDb && signatureId)
	{
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.18.0") >= 0)
		{
			query << "SELECT pose, map_id, weight, label, stamp, ground_truth_pose, velocity, gps, env_sensors "
					 "FROM Node "
					 "WHERE id = " << signatureId <<
					 ";";
		}
		else if(uStrNumCmp(_version, "0.14.0") >= 0)
		{
			query << "SELECT pose, map_id, weight, label, stamp, ground_truth_pose, velocity, gps "
					 "FROM Node "
					 "WHERE id = " << signatureId <<
					 ";";
		}
		else if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query << "SELECT pose, map_id, weight, label, stamp, ground_truth_pose, velocity "
					 "FROM Node "
					 "WHERE id = " << signatureId <<
					 ";";
		}
		else if(uStrNumCmp(_version, "0.11.1") >= 0)
		{
			query << "SELECT pose, map_id, weight, label, stamp, ground_truth_pose "
					 "FROM Node "
					 "WHERE id = " << signatureId <<
					 ";";
		}
		else if(uStrNumCmp(_version, "0.8.5") >= 0)
		{
			query << "SELECT pose, map_id, weight, label, stamp "
					 "FROM Node "
					 "WHERE id = " << signatureId <<
					 ";";
		}
		else
		{
			query << "SELECT pose, map_id, weight "
					 "FROM Node "
					 "WHERE id = " << signatureId <<
					 ";";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		const void * data = 0;
		int dataSize = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			found = true;
			int index = 0;
			data = sqlite3_column_blob(ppStmt, index); // pose
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if((unsigned int)dataSize == pose.size()*sizeof(float) && data)
			{
				memcpy(pose.data(), data, dataSize);
				if(uStrNumCmp(_version, "0.15.2") < 0)
				{
					pose.normalizeRotation();
				}
			}

			mapId = sqlite3_column_int(ppStmt, index++); // map id
			weight = sqlite3_column_int(ppStmt, index++); // weight

			if(uStrNumCmp(_version, "0.8.5") >= 0)
			{
				const unsigned char * p = sqlite3_column_text(ppStmt, index++);
				if(p)
				{
					label = reinterpret_cast<const char*>(p); // label
				}
				stamp = sqlite3_column_double(ppStmt, index++); // stamp

				if(uStrNumCmp(_version, "0.11.1") >= 0)
				{
					data = sqlite3_column_blob(ppStmt, index); // ground_truth_pose
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if((unsigned int)dataSize == groundTruthPose.size()*sizeof(float) && data)
					{
						memcpy(groundTruthPose.data(), data, dataSize);
						if(uStrNumCmp(_version, "0.15.2") < 0)
						{
							groundTruthPose.normalizeRotation();
						}
					}

					if(uStrNumCmp(_version, "0.13.0") >= 0)
					{
						velocity.resize(6,0);
						data = sqlite3_column_blob(ppStmt, index); // velocity
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						if((unsigned int)dataSize == velocity.size()*sizeof(float) && data)
						{
							memcpy(velocity.data(), data, dataSize);
						}
					}

					if(uStrNumCmp(_version, "0.14.0") >= 0)
					{
						data = sqlite3_column_blob(ppStmt, index); // gps
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						if((unsigned int)dataSize == 6*sizeof(double) && data)
						{
							const double * dataDouble = (const double *)data;
							gps = GPS(dataDouble[0], dataDouble[1], dataDouble[2], dataDouble[3], dataDouble[4], dataDouble[5]);
						}

						if(uStrNumCmp(_version, "0.18.0") >= 0)
						{
							data = sqlite3_column_blob(ppStmt, index);
							dataSize = sqlite3_column_bytes(ppStmt, index++);
							if(data)
							{
								UASSERT(dataSize%sizeof(double)==0 && (dataSize/sizeof(double))%3 == 0);
								const double * dataDouble = (const double *)data;
								int sensorsNum = (dataSize/sizeof(double))/3;
								for(int i=0; i<sensorsNum;++i)
								{
									EnvSensor::Type type = (EnvSensor::Type)(int)dataDouble[i*3];
									sensors.insert(std::make_pair(type, EnvSensor(type, dataDouble[i*3+1], dataDouble[i*3+2])));
								}
							}
						}
					}
				}
			}

			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	return found;
}

void DBDriverSqlite3::getLastNodeIdsQuery(std::set<int> & ids) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;

		if(uStrNumCmp(_version, "0.11.11") >= 0)
		{
			query = "SELECT n.id "
					 "FROM Node AS n "
					 "WHERE n.time_enter >= (SELECT MAX(time_enter) FROM Info) "
					 "ORDER BY n.id;";
		}
		else
		{
			query = "SELECT n.id "
					 "FROM Node AS n "
					 "WHERE n.time_enter >= (SELECT MAX(time_enter) FROM Statistics) "
					 "ORDER BY n.id;";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			ids.insert(ids.end(), sqlite3_column_int(ppStmt, 0)); // Signature Id
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%f ids=%d", timer.ticks(), (int)ids.size());
	}
}

void DBDriverSqlite3::getAllNodeIdsQuery(std::set<int> & ids, bool ignoreChildren, bool ignoreBadSignatures, bool ignoreIntermediateNodes) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT DISTINCT id "
			  << "FROM Node ";
		if(ignoreChildren)
		{
			query << "INNER JOIN Link ";
			query << "ON id = to_id "; // use to_id to ignore all children (which don't have link pointing on them)
			query << "WHERE from_id != to_id "; // ignore self referring links
			query << "AND weight>-9 "; //ignore invalid nodes
			if(ignoreIntermediateNodes)
			{
				query << "AND weight!=-1 "; //ignore intermediate nodes
			}
		}
		else if(ignoreIntermediateNodes)
		{
			query << "WHERE weight!=-1 "; //ignore intermediate nodes
		}

		if(ignoreBadSignatures)
		{
			if(ignoreChildren || ignoreIntermediateNodes)
			{
				query << "AND ";
			}
			else
			{
				query << "WHERE ";
			}
			if(uStrNumCmp(_version, "0.13.0") >= 0)
			{
				query << " id in (select node_id from Feature) ";
			}
			else
			{
				query << " id in (select node_id from Map_Node_Word) ";
			}
		}

		query  << "ORDER BY id";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			ids.insert(ids.end(), sqlite3_column_int(ppStmt, 0)); // Signature Id
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%f ids=%d", timer.ticks(), (int)ids.size());
	}
}

void DBDriverSqlite3::getAllLinksQuery(std::multimap<int, Link> & links, bool ignoreNullLinks, bool withLandmarks) const
{
	links.clear();
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.18.3") >= 0 && !withLandmarks)
		{
			query << "SELECT from_id, to_id, type, transform, information_matrix, user_data FROM Link WHERE type!=" << Link::kLandmark << " ORDER BY from_id, to_id";
		}
		else if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query << "SELECT from_id, to_id, type, transform, information_matrix, user_data FROM Link ORDER BY from_id, to_id";
		}
		else if(uStrNumCmp(_version, "0.10.10") >= 0)
		{
			query << "SELECT from_id, to_id, type, transform, rot_variance, trans_variance, user_data FROM Link ORDER BY from_id, to_id";
		}
		else if(uStrNumCmp(_version, "0.8.4") >= 0)
		{
			query << "SELECT from_id, to_id, type, transform, rot_variance, trans_variance FROM Link ORDER BY from_id, to_id";
		}
		else if(uStrNumCmp(_version, "0.7.4") >= 0)
		{
			query << "SELECT from_id, to_id, type, transform, variance FROM Link ORDER BY from_id, to_id";
		}
		else
		{
			query << "SELECT from_id, to_id, type, transform FROM Link ORDER BY from_id, to_id";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		int fromId = -1;
		int toId = -1;
		int type = Link::kUndef;
		const void * data = 0;
		int dataSize = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			int index = 0;

			fromId = sqlite3_column_int(ppStmt, index++);
			toId = sqlite3_column_int(ppStmt, index++);
			type = sqlite3_column_int(ppStmt, index++);

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);

			Transform transform;
			if((unsigned int)dataSize == transform.size()*sizeof(float) && data)
			{
				memcpy(transform.data(), data, dataSize);
				if(uStrNumCmp(_version, "0.15.2") < 0)
				{
					transform.normalizeRotation();
				}
			}
			else if(dataSize)
			{
				UERROR("Error while loading link transform from %d to %d! Setting to null...", fromId, toId);
			}

			if(!ignoreNullLinks || !transform.isNull())
			{
				cv::Mat informationMatrix = cv::Mat::eye(6,6,CV_64FC1);
				if(uStrNumCmp(_version, "0.8.4") >= 0)
				{
					if(uStrNumCmp(_version, "0.13.0") >= 0)
					{
						data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						UASSERT(dataSize==36*sizeof(double) && data);
						informationMatrix = cv::Mat(6, 6, CV_64FC1, (void *)data).clone(); // information_matrix
					}
					else
					{
						double rotVariance = sqlite3_column_double(ppStmt, index++);
						double transVariance = sqlite3_column_double(ppStmt, index++);
						UASSERT(rotVariance > 0.0 && transVariance>0.0);
						informationMatrix.at<double>(0,0) = 1.0/transVariance;
						informationMatrix.at<double>(1,1) = 1.0/transVariance;
						informationMatrix.at<double>(2,2) = 1.0/transVariance;
						informationMatrix.at<double>(3,3) = 1.0/rotVariance;
						informationMatrix.at<double>(4,4) = 1.0/rotVariance;
						informationMatrix.at<double>(5,5) = 1.0/rotVariance;
					}

					cv::Mat userDataCompressed;
					if(uStrNumCmp(_version, "0.10.10") >= 0)
					{
						const void * data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						//Create the userData
						if(dataSize>4 && data)
						{
							userDataCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone(); // userData
						}
					}

					links.insert(links.end(), std::make_pair(fromId, Link(fromId, toId, (Link::Type)type, transform, informationMatrix, userDataCompressed)));
				}
				else if(uStrNumCmp(_version, "0.7.4") >= 0)
				{
					double variance = sqlite3_column_double(ppStmt, index++);
					UASSERT(variance>0.0);
					informationMatrix *= 1.0/variance;
					links.insert(links.end(), std::make_pair(fromId, Link(fromId, toId, (Link::Type)type, transform, informationMatrix)));
				}
				else
				{
					// neighbor is 0, loop closures are 1 and 2 (child)
					links.insert(links.end(), std::make_pair(fromId, Link(fromId, toId, type==0?Link::kNeighbor:Link::kGlobalClosure, transform, informationMatrix)));
				}
			}

			rc = sqlite3_step(ppStmt);
		}

		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
}

void DBDriverSqlite3::getLastIdQuery(const std::string & tableName, int & id, const std::string & fieldName) const
{
	if(_ppDb)
	{
		UDEBUG("get last %s from table \"%s\"", fieldName.c_str(), tableName.c_str());
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT COALESCE(MAX(" << fieldName << "), " << id << ") " // In case the table is empty, return back input id
			  << "FROM " << tableName
			  << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); // Signature Id
			rc = sqlite3_step(ppStmt);
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			UDEBUG("No result from the DB for table %s with field %s", tableName.c_str(), fieldName.c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::getInvertedIndexNiQuery(int nodeId, int & ni) const
{
	ni = 0;
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query << "SELECT count(word_id) "
				  << "FROM Feature "
				  << "WHERE node_id=" << nodeId << ";";
		}
		else
		{
			query << "SELECT count(word_id) "
				  << "FROM Map_Node_Word "
				  << "WHERE node_id=" << nodeId << ";";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			ni = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB, node=%d",nodeId);
		}


		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::getNodesObservingLandmarkQuery(int landmarkId, std::map<int, Link> & nodes) const
{
	if(_ppDb && landmarkId < 0 && uStrNumCmp(_version, "0.18.3") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT from_id, type, information_matrix, transform, user_data FROM Link WHERE to_id=" << landmarkId <<"";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		int fromId = -1;
		int linkType = -1;
		std::list<Link> links;
		const void * data = 0;
		int dataSize = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			int index = 0;

			fromId = sqlite3_column_int(ppStmt, index++);
			linkType = sqlite3_column_int(ppStmt, index++);
			cv::Mat userDataCompressed;
			cv::Mat informationMatrix = cv::Mat::eye(6,6,CV_64FC1);

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			UASSERT(dataSize==36*sizeof(double) && data);
			informationMatrix = cv::Mat(6, 6, CV_64FC1, (void *)data).clone(); // information_matrix

			const void * data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			//Create the userData
			if(dataSize>4 && data)
			{
				userDataCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone(); // userData
			}

			//transform
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			Transform transform;
			if((unsigned int)dataSize == transform.size()*sizeof(float) && data)
			{
				memcpy(transform.data(), data, dataSize);
			}
			else if(dataSize)
			{
				UERROR("Error while loading link transform from %d to %d! Setting to null...", fromId, landmarkId);
			}

			if(linkType >= 0 && linkType != Link::kUndef)
			{
				nodes.insert(std::make_pair(fromId, Link(fromId, landmarkId, (Link::Type)linkType, transform, informationMatrix, userDataCompressed)));
			}
			else
			{
				UFATAL("Not supported link type %d ! (fromId=%d, toId=%d)",
						linkType, fromId, landmarkId);
			}

			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%f", timer.ticks());
	}
}

void DBDriverSqlite3::getNodeIdByLabelQuery(const std::string & label, int & id) const
{
	if(_ppDb && !label.empty() && uStrNumCmp(_version, "0.8.5") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		query << "SELECT id FROM Node WHERE label='" << label <<"'";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%f", timer.ticks());
	}
}

void DBDriverSqlite3::getAllLabelsQuery(std::map<int, std::string> & labels) const
{
	if(_ppDb && uStrNumCmp(_version, "0.8.5") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		query << "SELECT id,label FROM Node WHERE label IS NOT NULL";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			int index = 0;
			int id = sqlite3_column_int(ppStmt, index++);
			const unsigned char * p = sqlite3_column_text(ppStmt, index++);
			if(p)
			{
				std::string label = reinterpret_cast<const char*>(p);
				if(!label.empty())
				{
					labels.insert(std::make_pair(id, label));
				}
			}
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%f", timer.ticks());
	}
}

void DBDriverSqlite3::getWeightQuery(int nodeId, int & weight) const
{
	weight = 0;
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT weight FROM node WHERE id =  "
			  << nodeId
			  << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			weight= sqlite3_column_int(ppStmt, 0); // weight
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
}

//may be slower than the previous version but don't have a limit of words that can be loaded at the same time
void DBDriverSqlite3::loadSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & nodes) const
{
	ULOGGER_DEBUG("count=%d", (int)ids.size());
	if(_ppDb && ids.size())
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		unsigned int loaded = 0;

		// Load nodes information
		if(uStrNumCmp(_version, "0.18.0") >= 0)
		{
			query << "SELECT id, map_id, weight, pose, stamp, label, ground_truth_pose, velocity, gps, env_sensors "
				  << "FROM Node "
				  << "WHERE id=?;";
		}
		else if(uStrNumCmp(_version, "0.14.0") >= 0)
		{
			query << "SELECT id, map_id, weight, pose, stamp, label, ground_truth_pose, velocity, gps "
				  << "FROM Node "
				  << "WHERE id=?;";
		}
		else if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query << "SELECT id, map_id, weight, pose, stamp, label, ground_truth_pose, velocity "
				  << "FROM Node "
				  << "WHERE id=?;";
		}
		else if(uStrNumCmp(_version, "0.11.1") >= 0)
		{
			query << "SELECT id, map_id, weight, pose, stamp, label, ground_truth_pose "
				  << "FROM Node "
				  << "WHERE id=?;";
		}
		else if(uStrNumCmp(_version, "0.8.5") >= 0)
		{
			query << "SELECT id, map_id, weight, pose, stamp, label "
				  << "FROM Node "
				  << "WHERE id=?;";
		}
		else
		{
			query << "SELECT id, map_id, weight, pose "
				  << "FROM Node "
				  << "WHERE id=?;";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
		{
			//ULOGGER_DEBUG("Loading node %d...", *iter);
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, *iter);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			int id = 0;
			int mapId = 0;
			double stamp = 0.0;
			int weight = 0;
			Transform pose;
			Transform groundTruthPose;
			std::vector<float> velocity;
			std::vector<double> gps;
			EnvSensors sensors;
			const void * data = 0;
			int dataSize = 0;
			std::string label;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				int index = 0;
				id = sqlite3_column_int(ppStmt, index++); // Signature Id
				mapId = sqlite3_column_int(ppStmt, index++); // Map Id
				weight = sqlite3_column_int(ppStmt, index++); // weight

				data = sqlite3_column_blob(ppStmt, index); // pose
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				if((unsigned int)dataSize == pose.size()*sizeof(float) && data)
				{
					memcpy(pose.data(), data, dataSize);
					if(uStrNumCmp(_version, "0.15.2") < 0)
					{
						pose.normalizeRotation();
					}
				}

				if(uStrNumCmp(_version, "0.8.5") >= 0)
				{
					stamp = sqlite3_column_double(ppStmt, index++); // stamp
					const unsigned char * p = sqlite3_column_text(ppStmt, index++); // label
					if(p)
					{
						label = reinterpret_cast<const char*>(p);
					}

					if(uStrNumCmp(_version, "0.11.1") >= 0)
					{
						data = sqlite3_column_blob(ppStmt, index); // ground_truth_pose
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						if((unsigned int)dataSize == groundTruthPose.size()*sizeof(float) && data)
						{
							memcpy(groundTruthPose.data(), data, dataSize);
							if(uStrNumCmp(_version, "0.15.2") < 0)
							{
								groundTruthPose.normalizeRotation();
							}
						}

						if(uStrNumCmp(_version, "0.13.0") >= 0)
						{
							velocity.resize(6,0);
							data = sqlite3_column_blob(ppStmt, index); // velocity
							dataSize = sqlite3_column_bytes(ppStmt, index++);
							if((unsigned int)dataSize == velocity.size()*sizeof(float) && data)
							{
								memcpy(velocity.data(), data, dataSize);
							}
						}

						if(uStrNumCmp(_version, "0.14.0") >= 0)
						{
							gps.resize(6,0);
							data = sqlite3_column_blob(ppStmt, index); // gps
							dataSize = sqlite3_column_bytes(ppStmt, index++);
							if((unsigned int)dataSize == gps.size()*sizeof(double) && data)
							{
								memcpy(gps.data(), data, dataSize);
							}

							if(uStrNumCmp(_version, "0.18.0") >= 0)
							{
								data = sqlite3_column_blob(ppStmt, index); // env_sensors
								dataSize = sqlite3_column_bytes(ppStmt, index++);
								if(data)
								{
									UASSERT(dataSize%sizeof(double)==0 && (dataSize/sizeof(double))%3 == 0);
									const double * dataDouble = (const double *)data;
									int sensorsNum = (dataSize/sizeof(double))/3;
									for(int i=0; i<sensorsNum;++i)
									{
										EnvSensor::Type type = (EnvSensor::Type)(int)dataDouble[i*3];
										sensors.insert(std::make_pair(type, EnvSensor(type, dataDouble[i*3+1], dataDouble[i*3+2])));
									}
								}
							}
						}
					}
				}

				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			// create the node
			if(id)
			{
				ULOGGER_DEBUG("Creating %d (map=%d, pose=%s)", *iter, mapId, pose.prettyPrint().c_str());
				Signature * s = new Signature(
						id,
						mapId,
						weight,
						stamp,
						label,
						pose,
						groundTruthPose);
				if(velocity.size() == 6)
				{
					s->setVelocity(velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
				}
				if(gps.size() == 6)
				{
					s->sensorData().setGPS(GPS(gps[0], gps[1], gps[2], gps[3], gps[4], gps[5]));
				}
				s->sensorData().setEnvSensors(sensors);
				s->setSaved(true);
				nodes.push_back(s);
				++loaded;
			}
			else
			{
				UERROR("Signature %d not found in database!", *iter);
			}

			//reset
			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		// Prepare the query... Get the map from signature and visual words
		std::stringstream query2;
		if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query2 << "SELECT word_id, pos_x, pos_y, size, dir, response, octave, depth_x, depth_y, depth_z, descriptor_size, descriptor "
					 "FROM Feature "
					 "WHERE node_id = ? ";
		}
		else if(uStrNumCmp(_version, "0.12.0") >= 0)
		{
			query2 << "SELECT word_id, pos_x, pos_y, size, dir, response, octave, depth_x, depth_y, depth_z, descriptor_size, descriptor "
					 "FROM Map_Node_Word "
					 "WHERE node_id = ? ";
		}
		else if(uStrNumCmp(_version, "0.11.2") >= 0)
		{
			query2 << "SELECT word_id, pos_x, pos_y, size, dir, response, depth_x, depth_y, depth_z, descriptor_size, descriptor "
					 "FROM Map_Node_Word "
					 "WHERE node_id = ? ";
		}
		else
		{
			query2 << "SELECT word_id, pos_x, pos_y, size, dir, response, depth_x, depth_y, depth_z "
					 "FROM Map_Node_Word "
					 "WHERE node_id = ? ";
		}

		query2 << " ORDER BY word_id"; // Needed for fast insertion below
		query2 << ";";

		rc = sqlite3_prepare_v2(_ppDb, query2.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		float nanFloat = std::numeric_limits<float>::quiet_NaN ();

		for(std::list<Signature*>::const_iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
		{
			//ULOGGER_DEBUG("Loading words of %d...", (*iter)->id());
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			int visualWordId = 0;
			int descriptorSize = 0;
			const void * descriptor = 0;
			int dRealSize = 0;
			cv::KeyPoint kpt;
			std::multimap<int, int> visualWords;
			std::vector<cv::KeyPoint> visualWordsKpts;
			std::vector<cv::Point3f> visualWords3;
			cv::Mat descriptors;
			bool allWords3NaN = true;
			cv::Point3f depth(0,0,0);

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;
				visualWordId = sqlite3_column_int(ppStmt, index++);
				kpt.pt.x = sqlite3_column_double(ppStmt, index++);
				kpt.pt.y = sqlite3_column_double(ppStmt, index++);
				kpt.size = sqlite3_column_int(ppStmt, index++);
				kpt.angle = sqlite3_column_double(ppStmt, index++);
				kpt.response = sqlite3_column_double(ppStmt, index++);
				if(uStrNumCmp(_version, "0.12.0") >= 0)
				{
					kpt.octave = sqlite3_column_int(ppStmt, index++);
				}

				if(sqlite3_column_type(ppStmt, index) == SQLITE_NULL)
				{
					depth.x = nanFloat;
					++index;
				}
				else
				{
					depth.x = sqlite3_column_double(ppStmt, index++);
				}

				if(sqlite3_column_type(ppStmt, index) == SQLITE_NULL)
				{
					depth.y = nanFloat;
					++index;
				}
				else
				{
					depth.y = sqlite3_column_double(ppStmt, index++);
				}

				if(sqlite3_column_type(ppStmt, index) == SQLITE_NULL)
				{
					depth.z = nanFloat;
					++index;
				}
				else
				{
					depth.z = sqlite3_column_double(ppStmt, index++);
				}

				visualWordsKpts.push_back(kpt);
				visualWords.insert(visualWords.end(), std::make_pair(visualWordId, visualWordsKpts.size()-1));
				visualWords3.push_back(depth);

				if(allWords3NaN && util3d::isFinite(depth))
				{
					allWords3NaN = false;
				}

				if(uStrNumCmp(_version, "0.11.2") >= 0)
				{
					descriptorSize = sqlite3_column_int(ppStmt, index++); // VisualWord descriptor size
					descriptor = sqlite3_column_blob(ppStmt, index); 	// VisualWord descriptor array
					dRealSize = sqlite3_column_bytes(ppStmt, index++);

					if(descriptor && descriptorSize>0 && dRealSize>0)
					{
						cv::Mat d;
						if(dRealSize == descriptorSize)
						{
							// CV_8U binary descriptors
							d = cv::Mat(1, descriptorSize, CV_8U);
						}
						else if(dRealSize/int(sizeof(float)) == descriptorSize)
						{
							// CV_32F
							d = cv::Mat(1, descriptorSize, CV_32F);
						}
						else
						{
							UFATAL("Saved buffer size (%d bytes) is not the same as descriptor size (%d)", dRealSize, descriptorSize);
						}

						memcpy(d.data, descriptor, dRealSize);

						descriptors.push_back(d);
					}
				}

				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			if(visualWords.size()==0)
			{
				UDEBUG("Empty signature detected! (id=%d)", (*iter)->id());
			}
			else
			{
				if(allWords3NaN)
				{
					visualWords3.clear();
				}
				(*iter)->setWords(visualWords, visualWordsKpts, visualWords3, descriptors);
				ULOGGER_DEBUG("Add %d keypoints, %d 3d points and %d descriptors to node %d", (int)visualWords.size(), allWords3NaN?0:(int)visualWords3.size(), (int)descriptors.rows, (*iter)->id());
			}

			//reset
			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		this->loadLinksQuery(nodes);
		ULOGGER_DEBUG("Time load links=%fs", timer.ticks());

		for(std::list<Signature*>::iterator iter = nodes.begin(); iter!=nodes.end(); ++iter)
		{
			(*iter)->setModified(false);
		}

		// load calibrations
		if(nodes.size() && uStrNumCmp(_version, "0.10.0") >= 0)
		{
			std::stringstream query3;
			query3 << "SELECT calibration "
					 "FROM Data "
					 "WHERE id = ? ";

			rc = sqlite3_prepare_v2(_ppDb, query3.str().c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			for(std::list<Signature*>::const_iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
			{
				// bind id
				rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_step(ppStmt);
				if(rc == SQLITE_ROW)
				{
					int index=0;
					const void * data = 0;
					int dataSize = 0;
					Transform localTransform;
					std::vector<CameraModel> models;
					std::vector<StereoCameraModel> stereoModels;

					// calibration
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					// multi-cameras [fx,fy,cx,cy,[width,height],local_transform, ... ,fx,fy,cx,cy,[width,height],local_transform] (4or6+12)*float * numCameras
					// stereo [fx, fy, cx, cy, baseline, [width,height], local_transform] (5or7+12)*float
					if(dataSize > 0 && data)
					{
						if(uStrNumCmp(_version, "0.18.0") >= 0)
						{
							if(dataSize >= int(sizeof(int)*4))
							{
								const int * dataInt = (const int *)data;
								int type = dataInt[3];
								if(type == 0) // mono
								{
									CameraModel model;
									int bytesReadTotal = 0;
									unsigned int bytesRead = 0;
									while(bytesReadTotal < dataSize &&
										  (bytesRead=model.deserialize((const unsigned char *)data+bytesReadTotal, dataSize-bytesReadTotal))!=0)
									{
										bytesReadTotal+=bytesRead;
										models.push_back(model);
									}
									UASSERT(bytesReadTotal == dataSize);
								}
								else if(type == 1) // stereo
								{
									StereoCameraModel model;
									int bytesReadTotal = 0;
									unsigned int bytesRead = 0;
									while(bytesReadTotal < dataSize &&
										  (bytesRead=model.deserialize((const unsigned char *)data+bytesReadTotal, dataSize-bytesReadTotal))!=0)
									{
										bytesReadTotal+=bytesRead;
										stereoModels.push_back(model);
									}
									UASSERT(bytesReadTotal == dataSize);
								}
								else
								{
									UFATAL("Unknown calibration type %d", type);
								}
							}
							else
							{
								UFATAL("Wrong format of the Data.calibration field (size=%d bytes)", dataSize);
							}
						}
						else
						{
							float * dataFloat = (float*)data;
							if(uStrNumCmp(_version, "0.11.2") >= 0 &&
							   (unsigned int)dataSize % (6+localTransform.size())*sizeof(float) == 0)
							{
								int cameraCount = dataSize / ((6+localTransform.size())*sizeof(float));
								UDEBUG("Loading calibration for %d cameras (%d bytes)", cameraCount, dataSize);
								int max = cameraCount*(6+localTransform.size());
								for(int i=0; i<max; i+=6+localTransform.size())
								{
									// Reinitialize to a new Transform, to avoid copying in the same memory than the previous one
									localTransform = Transform::getIdentity();
									memcpy(localTransform.data(), dataFloat+i+6, localTransform.size()*sizeof(float));
									if(uStrNumCmp(_version, "0.15.2") < 0)
									{
										localTransform.normalizeRotation();
									}
									models.push_back(CameraModel(
											(double)dataFloat[i],
											(double)dataFloat[i+1],
											(double)dataFloat[i+2],
											(double)dataFloat[i+3],
											localTransform,
											0,
											cv::Size(dataFloat[i+4], dataFloat[i+5])));
									UDEBUG("%f %f %f %f %f %f %s", dataFloat[i], dataFloat[i+1], dataFloat[i+2],
											dataFloat[i+3], dataFloat[i+4], dataFloat[i+5],
											localTransform.prettyPrint().c_str());
								}
							}
							else if(uStrNumCmp(_version, "0.11.2") < 0 &&
									(unsigned int)dataSize % (4+localTransform.size())*sizeof(float) == 0)
							{
								int cameraCount = dataSize / ((4+localTransform.size())*sizeof(float));
								UDEBUG("Loading calibration for %d cameras (%d bytes)", cameraCount, dataSize);
								int max = cameraCount*(4+localTransform.size());
								for(int i=0; i<max; i+=4+localTransform.size())
								{
									// Reinitialize to a new Transform, to avoid copying in the same memory than the previous one
									localTransform = Transform::getIdentity();
									memcpy(localTransform.data(), dataFloat+i+4, localTransform.size()*sizeof(float));
									if(uStrNumCmp(_version, "0.15.2") < 0)
									{
										localTransform.normalizeRotation();
									}
									models.push_back(CameraModel(
											(double)dataFloat[i],
											(double)dataFloat[i+1],
											(double)dataFloat[i+2],
											(double)dataFloat[i+3],
											localTransform));
								}
							}
							else if((unsigned int)dataSize == (7+localTransform.size())*sizeof(float))
							{
								UDEBUG("Loading calibration of a stereo camera");
								memcpy(localTransform.data(), dataFloat+7, localTransform.size()*sizeof(float));
								if(uStrNumCmp(_version, "0.15.2") < 0)
								{
									localTransform.normalizeRotation();
								}
								stereoModels.push_back(StereoCameraModel(
										dataFloat[0],  // fx
										dataFloat[1],  // fy
										dataFloat[2],  // cx
										dataFloat[3],  // cy
										dataFloat[4], // baseline
										localTransform,
										cv::Size(dataFloat[5], dataFloat[6])));
							}
							else if((unsigned int)dataSize == (5+localTransform.size())*sizeof(float))
							{
								UDEBUG("Loading calibration of a stereo camera");
								memcpy(localTransform.data(), dataFloat+5, localTransform.size()*sizeof(float));
								if(uStrNumCmp(_version, "0.15.2") < 0)
								{
									localTransform.normalizeRotation();
								}
								stereoModels.push_back(StereoCameraModel(
										dataFloat[0],  // fx
										dataFloat[1],  // fy
										dataFloat[2],  // cx
										dataFloat[3],  // cy
										dataFloat[4], // baseline
										localTransform));
							}
							else
							{
								UFATAL("Wrong format of the Data.calibration field (size=%d bytes, db version=%s)", dataSize, _version.c_str());
							}
						}

						(*iter)->sensorData().setCameraModels(models);
						(*iter)->sensorData().setStereoCameraModels(stereoModels);
					}
					rc = sqlite3_step(ppStmt);
				}
				UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				//reset
				rc = sqlite3_reset(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			ULOGGER_DEBUG("Time load %d calibrations=%fs", (int)nodes.size(), timer.ticks());
		}

		// load global descriptors
		if(nodes.size() && uStrNumCmp(_version, "0.20.0") >= 0)
		{
			std::stringstream query3;
			query3 << "SELECT type, info, data "
					 "FROM GlobalDescriptor "
					 "WHERE node_id = ? ";

			rc = sqlite3_prepare_v2(_ppDb, query3.str().c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			for(std::list<Signature*>::const_iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
			{
				// bind id
				rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				std::vector<GlobalDescriptor> globalDescriptors;

				rc = sqlite3_step(ppStmt);
				while(rc == SQLITE_ROW)
				{
					int index=0;
					const void * data = 0;
					int dataSize = 0;
					int type = -1;
					cv::Mat info;
					cv::Mat dataMat;

					type = sqlite3_column_int(ppStmt, index++);
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize && data)
					{
						info = rtabmap::uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone());
					}
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize && data)
					{
						dataMat = rtabmap::uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone());
					}

					UASSERT(!dataMat.empty());
					globalDescriptors.push_back(GlobalDescriptor(type, dataMat, info));

					rc = sqlite3_step(ppStmt);
				}
				UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				if(!globalDescriptors.empty())
				{
					(*iter)->sensorData().setGlobalDescriptors(globalDescriptors);
					ULOGGER_DEBUG("Add %d global descriptors to node %d", (int)globalDescriptors.size(), (*iter)->id());
				}

				//reset
				rc = sqlite3_reset(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			ULOGGER_DEBUG("Time load %d global descriptors=%fs", (int)nodes.size(), timer.ticks());
		}

		if(ids.size() != loaded)
		{
			UERROR("Some signatures not found in database");
		}
	}
}

void DBDriverSqlite3::loadLastNodesQuery(std::list<Signature *> & nodes) const
{
	ULOGGER_DEBUG("");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;
		std::list<int> ids;

		if(uStrNumCmp(_version, "0.11.11") >= 0)
		{
			query = "SELECT n.id "
					 "FROM Node AS n "
					 "WHERE n.time_enter >= (SELECT MAX(time_enter) FROM Info) "
					 "ORDER BY n.id;";
		}
		else
		{
			query = "SELECT n.id "
					 "FROM Node AS n "
					 "WHERE n.time_enter >= (SELECT MAX(time_enter) FROM Statistics) "
					 "ORDER BY n.id;";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			ids.push_back(sqlite3_column_int(ppStmt, 0)); 	// Signature id
			rc = sqlite3_step(ppStmt); // next result...
		}

		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Loading %d signatures...", ids.size());
		this->loadSignaturesQuery(ids, nodes);
		ULOGGER_DEBUG("loaded=%d, Time=%fs", nodes.size(), timer.ticks());
	}
}

void DBDriverSqlite3::loadQuery(VWDictionary * dictionary, bool lastStateOnly) const
{
	ULOGGER_DEBUG("");
	if(_ppDb && dictionary)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::list<VisualWord *> visualWords;

		// Get the visual words
		query << "SELECT id, descriptor_size, descriptor FROM Word ";
		if(lastStateOnly)
		{
			if(uStrNumCmp(_version, "0.11.11") >= 0)
			{
				query << "WHERE time_enter >= (SELECT MAX(time_enter) FROM Info) ";
			}
			else
			{
				query << "WHERE time_enter >= (SELECT MAX(time_enter) FROM Statistics) ";
			}
		}
		query << "ORDER BY id;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		int id = 0;
		int descriptorSize = 0;
		const void * descriptor = 0;
		int dRealSize = 0;
		rc = sqlite3_step(ppStmt);
		int count = 0;
		while(rc == SQLITE_ROW)
		{
			int index=0;
			id = sqlite3_column_int(ppStmt, index++); 			// VisualWord Id

			descriptorSize = sqlite3_column_int(ppStmt, index++); // VisualWord descriptor size
			descriptor = sqlite3_column_blob(ppStmt, index); 	// VisualWord descriptor array
			dRealSize = sqlite3_column_bytes(ppStmt, index++);

			cv::Mat d;
			if(dRealSize == descriptorSize)
			{
				// CV_8U binary descriptors
				d = cv::Mat(1, descriptorSize, CV_8U);
			}
			else if(dRealSize/int(sizeof(float)) == descriptorSize)
			{
				// CV_32F
				d = cv::Mat(1, descriptorSize, CV_32F);
			}
			else
			{
				UFATAL("Saved buffer size (%d bytes) is not the same as descriptor size (%d)", dRealSize, descriptorSize);
			}

			memcpy(d.data, descriptor, dRealSize);
			VisualWord * vw = new VisualWord(id, d);
			vw->setSaved(true);
			dictionary->addWord(vw);

			if(++count % 5000 == 0)
			{
				ULOGGER_DEBUG("Loaded %d words...", count);
			}
			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Get Last word id
		getLastWordId(id);
		dictionary->setLastWordId(id);

		ULOGGER_DEBUG("Time=%fs", timer.ticks());
	}
}

//may be slower than the previous version but don't have a limit of words that can be loaded at the same time
void DBDriverSqlite3::loadWordsQuery(const std::set<int> & wordIds, std::list<VisualWord *> & vws) const
{
	ULOGGER_DEBUG("size=%d", wordIds.size());
	if(_ppDb && wordIds.size())
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::set<int> loaded;

		// Get the map from signature and visual words
		query << "SELECT vw.descriptor_size, vw.descriptor "
				 "FROM Word as vw "
				 "WHERE vw.id = ?;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		int descriptorSize;
		const void * descriptor;
		int dRealSize;
		unsigned long dRealSizeTotal = 0;
		for(std::set<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
		{
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, *iter);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				int index = 0;
				descriptorSize = sqlite3_column_int(ppStmt, index++); // VisualWord descriptor size
				descriptor = sqlite3_column_blob(ppStmt, index); 	// VisualWord descriptor array
				dRealSize = sqlite3_column_bytes(ppStmt, index++);

				cv::Mat d;
				if(dRealSize == descriptorSize)
				{
					// CV_8U binary descriptors
					d = cv::Mat(1, descriptorSize, CV_8U);
				}
				else if(dRealSize/int(sizeof(float)) == descriptorSize)
				{
					// CV_32F
					d = cv::Mat(1, descriptorSize, CV_32F);
				}
				else
				{
					UFATAL("Saved buffer size (%d bytes) is not the same as descriptor size (%d)", dRealSize, descriptorSize);
				}

				memcpy(d.data, descriptor, dRealSize);
				dRealSizeTotal+=dRealSize;
				VisualWord * vw = new VisualWord(*iter, d);
				if(vw)
				{
					vw->setSaved(true);
				}
				vws.push_back(vw);
				loaded.insert(loaded.end(), *iter);

				rc = sqlite3_step(ppStmt);
			}

			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs (%d words, %lu MB)", timer.ticks(), (int)vws.size(), dRealSizeTotal/1000000);

		if(wordIds.size() != loaded.size())
		{
			for(std::set<int>::const_iterator iter = wordIds.begin(); iter!=wordIds.end(); ++iter)
			{
				if(loaded.find(*iter) == loaded.end())
				{
					UDEBUG("Not found word %d", *iter);
				}
			}
			UERROR("Query (%d) doesn't match loaded words (%d)", wordIds.size(), loaded.size());
		}
	}
}

void DBDriverSqlite3::loadLinksQuery(
		int signatureId,
		std::multimap<int, Link> & links,
		Link::Type typeIn) const
{
	links.clear();
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query << "SELECT to_id, type, transform, information_matrix, user_data FROM Link ";
		}
		else if(uStrNumCmp(_version, "0.10.10") >= 0)
		{
			query << "SELECT to_id, type, transform, rot_variance, trans_variance, user_data FROM Link ";
		}
		else if(uStrNumCmp(_version, "0.8.4") >= 0)
		{
			query << "SELECT to_id, type, transform, rot_variance, trans_variance FROM Link ";
		}
		else if(uStrNumCmp(_version, "0.7.4") >= 0)
		{
			query << "SELECT to_id, type, transform, variance FROM Link ";
		}
		else
		{
			query << "SELECT to_id, type, transform FROM Link ";
		}
		query << "WHERE from_id = " << signatureId;
		if(typeIn < Link::kEnd)
		{
			if(uStrNumCmp(_version, "0.7.4") >= 0)
			{
				query << " AND type = " << typeIn;
			}
			else if(typeIn == Link::kNeighbor)
			{
				query << " AND type = 0";
			}
			else if(typeIn > Link::kNeighbor)
			{
				query << " AND type > 0";
			}
		}
		if(uStrNumCmp(_version, "0.18.3") >= 0 && (typeIn != Link::kAllWithLandmarks && typeIn != Link::kLandmark))
		{
			query << " AND type != " << Link::kLandmark;
		}

		query << " ORDER BY to_id";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		int toId = -1;
		int type = Link::kUndef;
		const void * data = 0;
		int dataSize = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			int index = 0;

			toId = sqlite3_column_int(ppStmt, index++);
			type = sqlite3_column_int(ppStmt, index++);

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);

			Transform transform;
			if((unsigned int)dataSize == transform.size()*sizeof(float) && data)
			{
				memcpy(transform.data(), data, dataSize);
				if(uStrNumCmp(_version, "0.15.2") < 0)
				{
					transform.normalizeRotation();
				}
			}
			else if(dataSize)
			{
				UERROR("Error while loading link transform from %d to %d! Setting to null...", signatureId, toId);
			}

			cv::Mat informationMatrix = cv::Mat::eye(6,6,CV_64FC1);
			if(uStrNumCmp(_version, "0.8.4") >= 0)
			{
				if(uStrNumCmp(_version, "0.13.0") >= 0)
				{
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					UASSERT(dataSize==36*sizeof(double) && data);
					informationMatrix = cv::Mat(6, 6, CV_64FC1, (void *)data).clone(); // information_matrix
				}
				else
				{
					double rotVariance = sqlite3_column_double(ppStmt, index++);
					double transVariance = sqlite3_column_double(ppStmt, index++);
					UASSERT(rotVariance > 0.0 && transVariance>0.0);
					informationMatrix.at<double>(0,0) = 1.0/transVariance;
					informationMatrix.at<double>(1,1) = 1.0/transVariance;
					informationMatrix.at<double>(2,2) = 1.0/transVariance;
					informationMatrix.at<double>(3,3) = 1.0/rotVariance;
					informationMatrix.at<double>(4,4) = 1.0/rotVariance;
					informationMatrix.at<double>(5,5) = 1.0/rotVariance;
				}

				cv::Mat userDataCompressed;
				if(uStrNumCmp(_version, "0.10.10") >= 0)
				{
					const void * data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					//Create the userData
					if(dataSize>4 && data)
					{
						userDataCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone(); // userData
					}
				}

				links.insert(links.end(), std::make_pair(toId, Link(signatureId, toId, (Link::Type)type, transform, informationMatrix, userDataCompressed)));
			}
			else if(uStrNumCmp(_version, "0.7.4") >= 0)
			{
				double variance = sqlite3_column_double(ppStmt, index++);
				UASSERT(variance>0.0);
				informationMatrix *= 1.0/variance;
				links.insert(links.end(), std::make_pair(toId, Link(signatureId, toId, (Link::Type)type, transform, informationMatrix)));
			}
			else
			{
				// neighbor is 0, loop closures are 1
				links.insert(links.end(), std::make_pair(toId, Link(signatureId, toId, type==0?Link::kNeighbor:Link::kGlobalClosure, transform, informationMatrix)));
			}

			rc = sqlite3_step(ppStmt);
		}

		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		if(links.size() == 0)
		{
			//UERROR("No links loaded from signature %d", signatureId);
		}
	}
}

void DBDriverSqlite3::loadLinksQuery(std::list<Signature *> & signatures) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		if(uStrNumCmp(_version, "0.13.0") >= 0)
		{
			query << "SELECT to_id, type, information_matrix, user_data, transform FROM Link "
				  << "WHERE from_id = ? "
				  << "ORDER BY to_id";
		}
		else if(uStrNumCmp(_version, "0.10.10") >= 0)
		{
			query << "SELECT to_id, type, rot_variance, trans_variance, user_data, transform FROM Link "
				  << "WHERE from_id = ? "
				  << "ORDER BY to_id";
		}
		else if(uStrNumCmp(_version, "0.8.4") >= 0)
		{
			query << "SELECT to_id, type, rot_variance, trans_variance, transform FROM Link "
				  << "WHERE from_id = ? "
				  << "ORDER BY to_id";
		}
		else if(uStrNumCmp(_version, "0.7.4") >= 0)
		{
			query << "SELECT to_id, type, variance, transform FROM Link "
				  << "WHERE from_id = ? "
				  << "ORDER BY to_id";
		}
		else
		{
			query << "SELECT to_id, type, transform FROM Link "
				  << "WHERE from_id = ? "
				  << "ORDER BY to_id";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<Signature*>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			int toId = -1;
			int linkType = -1;
			std::list<Link> links;
			const void * data = 0;
			int dataSize = 0;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;

				toId = sqlite3_column_int(ppStmt, index++);
				linkType = sqlite3_column_int(ppStmt, index++);
				cv::Mat userDataCompressed;
				cv::Mat informationMatrix = cv::Mat::eye(6,6,CV_64FC1);
				if(uStrNumCmp(_version, "0.8.4") >= 0)
				{
					if(uStrNumCmp(_version, "0.13.0") >= 0)
					{
						data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						UASSERT(dataSize==36*sizeof(double) && data);
						informationMatrix = cv::Mat(6, 6, CV_64FC1, (void *)data).clone(); // information_matrix
					}
					else
					{
						double rotVariance = sqlite3_column_double(ppStmt, index++);
						double transVariance = sqlite3_column_double(ppStmt, index++);
						UASSERT(rotVariance > 0.0 && transVariance>0.0);
						informationMatrix.at<double>(0,0) = 1.0/transVariance;
						informationMatrix.at<double>(1,1) = 1.0/transVariance;
						informationMatrix.at<double>(2,2) = 1.0/transVariance;
						informationMatrix.at<double>(3,3) = 1.0/rotVariance;
						informationMatrix.at<double>(4,4) = 1.0/rotVariance;
						informationMatrix.at<double>(5,5) = 1.0/rotVariance;
					}

					if(uStrNumCmp(_version, "0.10.10") >= 0)
					{
						const void * data = sqlite3_column_blob(ppStmt, index);
						dataSize = sqlite3_column_bytes(ppStmt, index++);
						//Create the userData
						if(dataSize>4 && data)
						{
							userDataCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone(); // userData
						}
					}
				}
				else if(uStrNumCmp(_version, "0.7.4") >= 0)
				{
					double variance = sqlite3_column_double(ppStmt, index++);
					UASSERT(variance>0.0);
					informationMatrix *= 1.0/variance;
				}

				//transform
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				Transform transform;
				if((unsigned int)dataSize == transform.size()*sizeof(float) && data)
				{
					memcpy(transform.data(), data, dataSize);
					if(uStrNumCmp(_version, "0.15.2") < 0)
					{
						transform.normalizeRotation();
					}
				}
				else if(dataSize)
				{
					UERROR("Error while loading link transform from %d to %d! Setting to null...", (*iter)->id(), toId);
				}

				if(linkType >= 0 && linkType != Link::kUndef)
				{
					if(linkType == Link::kLandmark)
					{
						(*iter)->addLandmark(Link((*iter)->id(), toId, (Link::Type)linkType, transform, informationMatrix, userDataCompressed));
					}
					else
					{
						if(uStrNumCmp(_version, "0.7.4") >= 0)
						{
							links.push_back(Link((*iter)->id(), toId, (Link::Type)linkType, transform, informationMatrix, userDataCompressed));
						}
						else // neighbor is 0, loop closures are 1 and 2 (child)
						{
							links.push_back(Link((*iter)->id(), toId, linkType == 0?Link::kNeighbor:Link::kGlobalClosure, transform, informationMatrix, userDataCompressed));
						}
					}
				}
				else
				{
					UFATAL("Not supported link type %d ! (fromId=%d, toId=%d)",
							linkType, (*iter)->id(), toId);
				}

				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			// add links
			(*iter)->addLinks(links);

			//reset
			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			UDEBUG("time=%fs, node=%d, links.size=%d", timer.ticks(), (*iter)->id(), links.size());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
}

void DBDriverSqlite3::updateQuery(const std::list<Signature *> & nodes, bool updateTimestamp) const
{
	UDEBUG("nodes = %d", nodes.size());
	if(_ppDb && nodes.size())
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		Signature * s = 0;

		std::string query;
		if(uStrNumCmp(_version, "0.8.5") >= 0)
		{
			if(updateTimestamp)
			{
				query = "UPDATE Node SET weight=?, label=?, time_enter = DATETIME('NOW') WHERE id=?;";
			}
			else
			{
				query = "UPDATE Node SET weight=?, label=? WHERE id=?;";
			}
		}
		else
		{
			if(updateTimestamp)
			{
				query = "UPDATE Node SET weight=?, time_enter = DATETIME('NOW') WHERE id=?;";
			}
			else
			{
				query = "UPDATE Node SET weight=? WHERE id=?;";
			}
		}
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<Signature *>::const_iterator i=nodes.begin(); i!=nodes.end(); ++i)
		{
			s = *i;
			int index = 1;
			if(s)
			{
				rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				if(uStrNumCmp(_version, "0.8.5") >= 0)
				{
					if(s->getLabel().empty())
					{
						rc = sqlite3_bind_null(ppStmt, index++);
						UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					}
					else
					{
						rc = sqlite3_bind_text(ppStmt, index++, s->getLabel().c_str(), -1, SQLITE_STATIC);
						UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					}
				}

				rc = sqlite3_bind_int(ppStmt, index++, s->id());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				//step
				rc=sqlite3_step(ppStmt);
				UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_reset(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Update Node table, Time=%fs", timer.ticks());

		// Update links part1
		if(uStrNumCmp(_version, "0.18.3") >= 0)
		{
			query = uFormat("DELETE FROM Link WHERE from_id=? and type!=%d;", (int)Link::kLandmark);
		}
		else
		{
			query = uFormat("DELETE FROM Link WHERE from_id=?;");
		}
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator j=nodes.begin(); j!=nodes.end(); ++j)
		{
			if((*j)->isLinksModified())
			{
				rc = sqlite3_bind_int(ppStmt, 1, (*j)->id());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				rc=sqlite3_step(ppStmt);
				UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_reset(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Update links part2
		query = queryStepLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator j=nodes.begin(); j!=nodes.end(); ++j)
		{
			if((*j)->isLinksModified())
			{
				// Save links
				const std::multimap<int, Link> & links = (*j)->getLinks();
				for(std::multimap<int, Link>::const_iterator i=links.begin(); i!=links.end(); ++i)
				{
					stepLink(ppStmt, i->second);
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Update Neighbors Time=%fs", timer.ticks());

		// Update word references
		query = queryStepWordsChanged();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator j=nodes.begin(); j!=nodes.end(); ++j)
		{
			if((*j)->getWordsChanged().size())
			{
				const std::map<int, int> & wordsChanged = (*j)->getWordsChanged();
				for(std::map<int, int>::const_iterator iter=wordsChanged.begin(); iter!=wordsChanged.end(); ++iter)
				{
					stepWordsChanged(ppStmt, (*j)->id(), iter->first, iter->second);
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("signatures update=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::updateQuery(const std::list<VisualWord *> & words, bool updateTimestamp) const
{
	if(_ppDb && words.size() && updateTimestamp)
	{
		// Only timestamp update is done here, so don't enter this if at all if false
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		VisualWord * w = 0;

		std::string query = "UPDATE Word SET time_enter = DATETIME('NOW') WHERE id=?;";
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<VisualWord *>::const_iterator i=words.begin(); i!=words.end(); ++i)
		{
			w = *i;
			int index = 1;
			UASSERT(w);

			rc = sqlite3_bind_int(ppStmt, index++, w->id());
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			//step
			rc=sqlite3_step(ppStmt);
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Update Word table, Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::saveQuery(const std::list<Signature *> & signatures)
{
	UDEBUG("");
	if(_ppDb && signatures.size())
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		// Signature table
		std::string query = queryStepNode();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			_memoryUsedEstimate += (*i)->getMemoryUsed();
			// raw data are not kept in database
			_memoryUsedEstimate -= (*i)->sensorData().imageRaw().total() * (*i)->sensorData().imageRaw().elemSize();
			_memoryUsedEstimate -= (*i)->sensorData().depthOrRightRaw().total() * (*i)->sensorData().depthOrRightRaw().elemSize();
			_memoryUsedEstimate -= (*i)->sensorData().laserScanRaw().data().total() * (*i)->sensorData().laserScanRaw().data().elemSize();

			stepNode(ppStmt, *i);
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());

		// Create new entries in table Link
		query = queryStepLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator jter=signatures.begin(); jter!=signatures.end(); ++jter)
		{
			// Save links
			const std::multimap<int, Link> & links = (*jter)->getLinks();
			for(std::multimap<int, Link>::const_iterator i=links.begin(); i!=links.end(); ++i)
			{
				stepLink(ppStmt, i->second);
			}
			if(uStrNumCmp(_version, "0.18.3") >= 0)
			{
				// Save landmarks
				const std::map<int, Link> & links = (*jter)->getLandmarks();
				for(std::map<int, Link>::const_iterator i=links.begin(); i!=links.end(); ++i)
				{
					stepLink(ppStmt, i->second);
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());


		// Create new entries in table Feature
		query = queryStepKeypoint();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		float nanFloat = std::numeric_limits<float>::quiet_NaN ();
		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			UASSERT((*i)->getWords().size() == (*i)->getWordsKpts().size());
			UASSERT((*i)->getWords3().empty() || (*i)->getWords().size() == (*i)->getWords3().size());
			UASSERT((*i)->getWordsDescriptors().empty() || (int)(*i)->getWords().size() == (*i)->getWordsDescriptors().rows);

			for(std::multimap<int, int>::const_iterator w=(*i)->getWords().begin(); w!=(*i)->getWords().end(); ++w)
			{
				cv::Point3f pt(nanFloat,nanFloat,nanFloat);
				if(!(*i)->getWords3().empty())
				{
					pt = (*i)->getWords3()[w->second];
				}

				cv::Mat descriptor;
				if(!(*i)->getWordsDescriptors().empty())
				{
					descriptor = (*i)->getWordsDescriptors().row(w->second);
				}

				stepKeypoint(ppStmt, (*i)->id(), w->first, (*i)->getWordsKpts()[w->second], pt, descriptor);
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		UDEBUG("Time=%fs", timer.ticks());

		if(uStrNumCmp(_version, "0.20.0") >= 0)
		{
			// Global descriptor table
			std::string query = queryStepGlobalDescriptor();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
			{
				for(size_t d=0; d<(*i)->sensorData().globalDescriptors().size(); ++d)
				{
					stepGlobalDescriptor(ppStmt, (*i)->id(), (*i)->sensorData().globalDescriptors()[d]);
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			UDEBUG("Time=%fs", timer.ticks());
		}

		if(uStrNumCmp(_version, "0.10.0") >= 0)
		{
			// Add SensorData
			query = queryStepSensorData();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			UDEBUG("Saving %d images", signatures.size());

			for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
			{
				if(!(*i)->sensorData().imageCompressed().empty() ||
				   !(*i)->sensorData().depthOrRightCompressed().empty() ||
				   !(*i)->sensorData().laserScanCompressed().isEmpty() ||
				   !(*i)->sensorData().userDataCompressed().empty() ||
				   !(*i)->sensorData().cameraModels().empty() ||
				   !(*i)->sensorData().stereoCameraModels().empty())
				{
					UASSERT((*i)->id() == (*i)->sensorData().id());
					stepSensorData(ppStmt, (*i)->sensorData());
				}
			}

			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			UDEBUG("Time=%fs", timer.ticks());
		}
		else
		{
			// Add images
			query = queryStepImage();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			UDEBUG("Saving %d images", signatures.size());

			for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
			{
				if(!(*i)->sensorData().imageCompressed().empty())
				{
					stepImage(ppStmt, (*i)->id(), (*i)->sensorData().imageCompressed());
				}
			}

			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			UDEBUG("Time=%fs", timer.ticks());

			// Add depths
			query = queryStepDepth();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
			{
				//metric
				if(!(*i)->sensorData().depthOrRightCompressed().empty() || !(*i)->sensorData().laserScanCompressed().isEmpty())
				{
					UASSERT((*i)->id() == (*i)->sensorData().id());
					stepDepth(ppStmt, (*i)->sensorData());
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		UDEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::saveQuery(const std::list<VisualWord *> & words) const
{
	UDEBUG("visualWords size=%d", words.size());
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;

		// Create new entries in table Map_SS_VW
		if(words.size()>0)
		{
			query = std::string("INSERT INTO Word(id, descriptor_size, descriptor) VALUES(?,?,?);");
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			for(std::list<VisualWord *>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
			{
				const VisualWord * w = *iter;
				UASSERT(w);
				if(!w->isSaved())
				{
					rc = sqlite3_bind_int(ppStmt, 1, w->id());
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					rc = sqlite3_bind_int(ppStmt, 2, w->getDescriptor().cols);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					UASSERT(w->getDescriptor().type() == CV_32F || w->getDescriptor().type() == CV_8U);
					if(w->getDescriptor().type() == CV_32F)
					{
						// CV_32F
						rc = sqlite3_bind_blob(ppStmt, 3, w->getDescriptor().data, w->getDescriptor().cols*sizeof(float), SQLITE_STATIC);
					}
					else
					{
						// CV_8U
						rc = sqlite3_bind_blob(ppStmt, 3, w->getDescriptor().data, w->getDescriptor().cols*sizeof(char), SQLITE_STATIC);
					}
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

					//execute query
					rc=sqlite3_step(ppStmt);
					UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s (word=%d)", _version.c_str(), sqlite3_errmsg(_ppDb), w->id()).c_str());

					rc = sqlite3_reset(ppStmt);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		UDEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::addLinkQuery(const Link & link) const
{
	UDEBUG("");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		// Create new entries in table Link
		std::string query = queryStepLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Save link
		stepLink(ppStmt, link);

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}

}

void DBDriverSqlite3::updateLinkQuery(const Link & link) const
{
	UDEBUG("");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		// Create new entries in table Link
		std::string query = queryStepLinkUpdate();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Save link
		stepLink(ppStmt, link);

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::updateOccupancyGridQuery(
		int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles,
		const cv::Mat & empty,
		float cellSize,
		const cv::Point3f & viewpoint) const
{
	UDEBUG("");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		// Create query
		std::string query = queryStepOccupancyGridUpdate();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Save occupancy grid
		stepOccupancyGridUpdate(ppStmt,
				nodeId,
				ground,
				obstacles,
				empty,
				cellSize,
				viewpoint);

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::updateDepthImageQuery(
		int nodeId,
		const cv::Mat & image) const
{
	UDEBUG("");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		// Create query
		std::string query = queryStepDepthUpdate();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Save depth
		stepDepthUpdate(ppStmt,
				nodeId,
				image);

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::updateLaserScanQuery(
		int nodeId,
		const LaserScan & scan) const
{
	UDEBUG("");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		// Create query
		std::string query = queryStepScanUpdate();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Save depth
		stepScanUpdate(ppStmt,
				nodeId,
				scan);

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::addStatisticsQuery(const Statistics & statistics, bool saveWmState) const
{
	UDEBUG("Ref ID = %d", statistics.refImageId());
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		// Create query
		if(uStrNumCmp(this->getDatabaseVersion(), "0.11.11") >= 0)
		{
			std::string param = Statistics::serializeData(statistics.data());
			if(param.size() && statistics.refImageId()>0)
			{
				std::string query;
				if(uStrNumCmp(this->getDatabaseVersion(), "0.16.2") >= 0)
				{
					query = "INSERT INTO Statistics(id, stamp, data, wm_state) values(?,?,?,?);";
				}
				else
				{
					query = "INSERT INTO Statistics(id, stamp, data) values(?,?,?);";
				}
				rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				int index = 1;
				rc = sqlite3_bind_int(ppStmt, index++, statistics.refImageId());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				rc = sqlite3_bind_double(ppStmt, index++, statistics.stamp());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				cv::Mat compressedParam;
				if(uStrNumCmp(this->getDatabaseVersion(), "0.15.0") >= 0)
				{
					compressedParam = compressString(param);
					rc = sqlite3_bind_blob(ppStmt, index++, compressedParam.data, compressedParam.cols, SQLITE_STATIC);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}
				else
				{
					rc = sqlite3_bind_text(ppStmt, index++, param.c_str(), -1, SQLITE_STATIC);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}

				cv::Mat compressedWmState;
				if(uStrNumCmp(this->getDatabaseVersion(), "0.16.2") >= 0)
				{
					if(saveWmState && !statistics.wmState().empty())
					{
						compressedWmState = compressData2(cv::Mat(1, statistics.wmState().size(), CV_32SC1, (void *)statistics.wmState().data()));
						rc = sqlite3_bind_blob(ppStmt, index++, compressedWmState.data, compressedWmState.cols, SQLITE_STATIC);
						UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					}
					else
					{
						rc = sqlite3_bind_null(ppStmt, index++);
						UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					}
				}

				//step
				rc=sqlite3_step(ppStmt);
				UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_reset(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				// Finalize (delete) the statement
				rc = sqlite3_finalize(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				UDEBUG("Time=%fs", timer.ticks());
			}
		}
	}
}

void DBDriverSqlite3::savePreviewImageQuery(const cv::Mat & image) const
{
	UDEBUG("");
	if(_ppDb && uStrNumCmp(_version, "0.12.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;

		// Update table Admin
		query = uFormat("UPDATE Admin SET preview_image=? WHERE version='%s';", _version.c_str());
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		int index = 1;
		cv::Mat compressedImage;
		if(image.empty())
		{
			rc = sqlite3_bind_null(ppStmt, index);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			// compress
			if(image.rows == 1 && image.type() == CV_8UC1)
			{
				// already compressed
				compressedImage = image;
			}
			else
			{
				compressedImage	= compressImage2(image, ".jpg");
			}
			rc = sqlite3_bind_blob(ppStmt, index++, compressedImage.data, compressedImage.cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		//execute query
		rc=sqlite3_step(ppStmt);
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}
cv::Mat DBDriverSqlite3::loadPreviewImageQuery() const
{
	UDEBUG("");
	cv::Mat image;
	if(_ppDb && uStrNumCmp(_version, "0.12.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT preview_image "
			  << "FROM Admin "
			  << "WHERE version='" << _version.c_str()
			  <<"';";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		UASSERT_MSG(rc == SQLITE_ROW, uFormat("DB error (%s): Not found first Admin row: query=\"%s\"", _version.c_str(), query.str().c_str()).c_str());
		if(rc == SQLITE_ROW)
		{
			const void * data = 0;
			int dataSize = 0;
			int index = 0;

			//opt_cloud
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if(dataSize>0 && data)
			{
				image = uncompressImage(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
			}
			UDEBUG("Image=%dx%d", image.cols, image.rows);

			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());

	}
	return image;
}

void DBDriverSqlite3::saveOptimizedPosesQuery(const std::map<int, Transform> & poses, const Transform & lastlocalizationPose) const
{
	UDEBUG("poses=%d lastlocalizationPose=%s", (int)poses.size(), lastlocalizationPose.prettyPrint().c_str());
	if(_ppDb && uStrNumCmp(_version, "0.17.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;

		// Update table Admin
		query = uFormat("UPDATE Admin SET opt_ids=?, opt_poses=?, opt_last_localization=?, time_enter = DATETIME('NOW') WHERE version='%s';", _version.c_str());
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		int index = 1;

		// opt ids and poses
		cv::Mat compressedIds;
		cv::Mat compressedPoses;
		if(poses.empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			std::vector<int> serializedIds(poses.size());
			std::vector<float> serializedPoses(poses.size()*12);
			int i=0;
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				serializedIds[i] = iter->first;
				memcpy(serializedPoses.data()+(12*i), iter->second.data(), 12*sizeof(float));
				++i;
			}

			compressedIds = compressData2(cv::Mat(1,serializedIds.size(), CV_32SC1, serializedIds.data()));
			compressedPoses = compressData2(cv::Mat(1,serializedPoses.size(), CV_32FC1, serializedPoses.data()));

			rc = sqlite3_bind_blob(ppStmt, index++, compressedIds.data, compressedIds.cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			rc = sqlite3_bind_blob(ppStmt, index++, compressedPoses.data, compressedPoses.cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		if(lastlocalizationPose.isNull())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			UDEBUG("lastlocalizationPose=%s", lastlocalizationPose.prettyPrint().c_str());
			rc = sqlite3_bind_blob(ppStmt, index++, lastlocalizationPose.data(), lastlocalizationPose.size()*sizeof(float), SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		//execute query
		rc=sqlite3_step(ppStmt);
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

std::map<int, Transform> DBDriverSqlite3::loadOptimizedPosesQuery(Transform * lastlocalizationPose) const
{
	UDEBUG("");
	std::map<int, Transform> poses;
	if(_ppDb && uStrNumCmp(_version, "0.17.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT opt_ids, opt_poses, opt_last_localization "
			  << "FROM Admin "
			  << "WHERE version='" << _version.c_str()
			  <<"';";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		UASSERT_MSG(rc == SQLITE_ROW, uFormat("DB error (%s): Not found first Admin row: query=\"%s\"", _version.c_str(), query.str().c_str()).c_str());
		if(rc == SQLITE_ROW)
		{
			const void * data = 0;
			int dataSize = 0;
			int index = 0;

			//opt_poses
			cv::Mat serializedIds;
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if(dataSize>0 && data)
			{
				serializedIds = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
				UDEBUG("serializedIds=%d", serializedIds.cols);
			}

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if(dataSize>0 && data)
			{
				cv::Mat serializedPoses = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
				UDEBUG("serializedPoses=%d", serializedPoses.cols);

				UASSERT(serializedIds.cols == serializedPoses.cols/12);
				UASSERT(serializedPoses.type() == CV_32FC1);
				UASSERT(serializedIds.type() == CV_32SC1);
				for(int i=0; i<serializedIds.cols; ++i)
				{
					Transform t(serializedPoses.at<float>(i*12), serializedPoses.at<float>(i*12+1), serializedPoses.at<float>(i*12+2), serializedPoses.at<float>(i*12+3),
							serializedPoses.at<float>(i*12+4), serializedPoses.at<float>(i*12+5), serializedPoses.at<float>(i*12+6), serializedPoses.at<float>(i*12+7),
							serializedPoses.at<float>(i*12+8), serializedPoses.at<float>(i*12+9), serializedPoses.at<float>(i*12+10), serializedPoses.at<float>(i*12+11));
					poses.insert(std::make_pair(serializedIds.at<int>(i), t));
					UDEBUG("Optimized pose %d: %s", serializedIds.at<int>(i), t.prettyPrint().c_str());
				}
			}

			data = sqlite3_column_blob(ppStmt, index); // ground_truth_pose
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if(lastlocalizationPose)
			{
				if((unsigned int)dataSize == lastlocalizationPose->size()*sizeof(float) && data)
				{
					memcpy(lastlocalizationPose->data(), data, dataSize);
				}
				UDEBUG("lastlocalizationPose=%s", lastlocalizationPose->prettyPrint().c_str());
			}

			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());

	}
	return poses;
}

void DBDriverSqlite3::save2DMapQuery(const cv::Mat & map, float xMin, float yMin, float cellSize) const
{
	UDEBUG("");
	if(_ppDb && uStrNumCmp(_version, "0.17.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;

		// Update table Admin
		query = uFormat("UPDATE Admin SET opt_map=?, opt_map_x_min=?, opt_map_y_min=?, opt_map_resolution=?, time_enter = DATETIME('NOW') WHERE version='%s';", _version.c_str());
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		int index = 1;

		// opt ids and poses
		cv::Mat compressedMap;
		if(map.empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			compressedMap = compressData2(map);

			rc = sqlite3_bind_blob(ppStmt, index++, compressedMap.data, compressedMap.cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		rc = sqlite3_bind_double(ppStmt, index++, xMin);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, yMin);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, cellSize);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		//execute query
		rc=sqlite3_step(ppStmt);
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

cv::Mat DBDriverSqlite3::load2DMapQuery(float & xMin, float & yMin, float & cellSize) const
{
	UDEBUG("");
	cv::Mat map;
	if(_ppDb && uStrNumCmp(_version, "0.17.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT  opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution "
			  << "FROM Admin "
			  << "WHERE version='" << _version.c_str()
			  <<"';";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		UASSERT_MSG(rc == SQLITE_ROW, uFormat("DB error (%s): Not found first Admin row: query=\"%s\"", _version.c_str(), query.str().c_str()).c_str());
		if(rc == SQLITE_ROW)
		{
			const void * data = 0;
			int dataSize = 0;
			int index = 0;

			//opt_map
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if(dataSize>0 && data)
			{
				map = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
				UDEBUG("map=%d/%d", map.cols, map.rows);
			}

			xMin = sqlite3_column_double(ppStmt, index++);
			UDEBUG("xMin=%f", xMin);
			yMin = sqlite3_column_double(ppStmt, index++);
			UDEBUG("yMin=%f", yMin);
			cellSize = sqlite3_column_double(ppStmt, index++);
			UDEBUG("cellSize=%f", cellSize);

			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());

	}
	return map;
}

void DBDriverSqlite3::saveOptimizedMeshQuery(
			const cv::Mat & cloud,
			const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords,
#else
			const std::vector<std::vector<Eigen::Vector2f> > & texCoords,
#endif
			const cv::Mat & textures) const
{
	UDEBUG("");
	if(_ppDb && uStrNumCmp(_version, "0.13.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;

		// Update table Admin
		query = uFormat("UPDATE Admin SET opt_cloud=?, opt_polygons_size=?, opt_polygons=?, opt_tex_coords=?, opt_tex_materials=?, time_enter = DATETIME('NOW') WHERE version='%s';", _version.c_str());
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		if(cloud.empty())
		{
			// set all fields to null
			for(int i=1; i<=5; ++i)
			{
				rc = sqlite3_bind_null(ppStmt, i);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}

			//execute query
			rc=sqlite3_step(ppStmt);
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			int index = 1;

			// compress and save cloud
			cv::Mat compressedCloud;
			if(cloud.rows == 1 && cloud.type() == CV_8UC1)
			{
				// already compressed
				compressedCloud = cloud;
			}
			else
			{
				UDEBUG("Cloud points=%d", cloud.cols);
				compressedCloud	= compressData2(cloud);
			}
			UDEBUG("Cloud compressed bytes=%d", compressedCloud.cols);
			rc = sqlite3_bind_blob(ppStmt, index++, compressedCloud.data, compressedCloud.cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			// opt ids and poses
			cv::Mat compressedPolygons;
			cv::Mat compressedTexCoords;
			cv::Mat compressedTextures;
			// polygons
			if(polygons.empty())
			{
				//polygon size
				rc = sqlite3_bind_null(ppStmt, index++);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				// polygons
				rc = sqlite3_bind_null(ppStmt, index++);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				// tex_coords
				rc = sqlite3_bind_null(ppStmt, index++);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				// materials
				rc = sqlite3_bind_null(ppStmt, index++);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}
			else
			{
				std::vector<int> serializedPolygons;
				std::vector<float> serializedTexCoords;
				int polygonSize = 0;
				int totalPolygonIndices = 0;
				UASSERT(texCoords.empty() || polygons.size() == texCoords.size());
				for(unsigned int t=0; t<polygons.size(); ++t)
				{
					UDEBUG("t=%d, polygons=%d", t, (int)polygons[t].size());
					unsigned int materialPolygonIndices = 0;
					for(unsigned int p=0; p<polygons[t].size(); ++p)
					{
						if(polygonSize == 0)
						{
							UASSERT(polygons[t][p].size());
							polygonSize = polygons[t][p].size();
						}
						else
						{
							UASSERT(polygonSize == (int)polygons[t][p].size());
						}

						materialPolygonIndices += polygons[t][p].size();
					}
					totalPolygonIndices += materialPolygonIndices;

					if(!texCoords.empty())
					{
						UASSERT(materialPolygonIndices == texCoords[t].size());
					}
				}
				UASSERT(totalPolygonIndices>0);
				serializedPolygons.resize(totalPolygonIndices+polygons.size());
				if(!texCoords.empty())
				{
					serializedTexCoords.resize(totalPolygonIndices*2+polygons.size());
				}

				int oi=0;
				int ci=0;
				for(unsigned int t=0; t<polygons.size(); ++t)
				{
					serializedPolygons[oi++] = polygons[t].size();
					if(!texCoords.empty())
					{
						serializedTexCoords[ci++] = texCoords[t].size();
					}
					for(unsigned int p=0; p<polygons[t].size(); ++p)
					{
						int texIndex = p*polygonSize;
						for(unsigned int i=0; i<polygons[t][p].size(); ++i)
						{
							serializedPolygons[oi++] = polygons[t][p][i];

							if(!texCoords.empty())
							{
								serializedTexCoords[ci++] = texCoords[t][texIndex+i][0];
								serializedTexCoords[ci++] = texCoords[t][texIndex+i][1];
							}
						}
					}
				}

				UDEBUG("serializedPolygons=%d", (int)serializedPolygons.size());
				compressedPolygons = compressData2(cv::Mat(1,serializedPolygons.size(), CV_32SC1, serializedPolygons.data()));

				// polygon size
				rc = sqlite3_bind_int(ppStmt, index++, polygonSize);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_bind_blob(ppStmt, index++, compressedPolygons.data, compressedPolygons.cols, SQLITE_STATIC);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

				// tex coords
				if(texCoords.empty())
				{
					// tex coords
					rc = sqlite3_bind_null(ppStmt, index++);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					// materials
					rc = sqlite3_bind_null(ppStmt, index++);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}
				else
				{
					UDEBUG("serializedTexCoords=%d", (int)serializedTexCoords.size());
					compressedTexCoords = compressData2(cv::Mat(1,serializedTexCoords.size(), CV_32FC1, serializedTexCoords.data()));
					rc = sqlite3_bind_blob(ppStmt, index++, compressedTexCoords.data, compressedTexCoords.cols, SQLITE_STATIC);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

					UASSERT(!textures.empty() && textures.cols % textures.rows == 0 && textures.cols/textures.rows == (int)texCoords.size());
					if(textures.rows == 1 && textures.type() == CV_8UC1)
					{
						//already compressed
						compressedTextures = textures;
					}
					else
					{
						compressedTextures = compressImage2(textures, ".jpg");
					}
					rc = sqlite3_bind_blob(ppStmt, index++, compressedTextures.data, compressedTextures.cols, SQLITE_STATIC);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}
			}

			//execute query
			rc=sqlite3_step(ppStmt);
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

cv::Mat DBDriverSqlite3::loadOptimizedMeshQuery(
			std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > * polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
			std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > * texCoords,
#else
			std::vector<std::vector<Eigen::Vector2f> > * texCoords,
#endif
			cv::Mat * textures) const
{
	UDEBUG("");
	cv::Mat cloud;
	if(_ppDb && uStrNumCmp(_version, "0.13.0") >= 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT opt_cloud, opt_polygons_size, opt_polygons, opt_tex_coords, opt_tex_materials "
			  << "FROM Admin "
			  << "WHERE version='" << _version.c_str()
			  <<"';";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		UASSERT_MSG(rc == SQLITE_ROW, uFormat("DB error (%s): Not found first Admin row: query=\"%s\"", _version.c_str(), query.str().c_str()).c_str());
		if(rc == SQLITE_ROW)
		{
			const void * data = 0;
			int dataSize = 0;
			int index = 0;

			//opt_cloud
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if(dataSize>0 && data)
			{
				cloud = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
			}
			UDEBUG("Cloud=%d points", cloud.cols);

			//opt_polygons_size
			int polygonSize = sqlite3_column_int(ppStmt, index++);
			UDEBUG("polygonSize=%d", polygonSize);

			//opt_polygons
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			if(dataSize>0 && data)
			{
				UASSERT(polygonSize > 0);
				if(polygons)
				{
					cv::Mat serializedPolygons = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
					UDEBUG("serializedPolygons=%d", serializedPolygons.cols);
					UASSERT(serializedPolygons.total());
					for(int t=0; t<serializedPolygons.cols; ++t)
					{
						UASSERT(serializedPolygons.at<int>(t) > 0);
						std::vector<std::vector<RTABMAP_PCL_INDEX> > materialPolygons(serializedPolygons.at<int>(t), std::vector<RTABMAP_PCL_INDEX>(polygonSize));
						++t;
						UASSERT(t < serializedPolygons.cols);
						UDEBUG("materialPolygons=%d", (int)materialPolygons.size());
						for(int p=0; p<(int)materialPolygons.size(); ++p)
						{
							for(int i=0; i<polygonSize; ++i)
							{
								materialPolygons[p][i] = serializedPolygons.at<int>(t + p*polygonSize + i);
							}
						}
						t+=materialPolygons.size()*polygonSize;
						polygons->push_back(materialPolygons);
					}
				}

				//opt_tex_coords
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				if(dataSize>0 && data)
				{
					if(texCoords)
					{
						cv::Mat serializedTexCoords = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
						UDEBUG("serializedTexCoords=%d", serializedTexCoords.cols);
						UASSERT(serializedTexCoords.total());
						for(int t=0; t<serializedTexCoords.cols; ++t)
						{
							UASSERT(int(serializedTexCoords.at<float>(t)) > 0);
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
							std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > materialtexCoords(int(serializedTexCoords.at<float>(t)));
#else
							std::vector<Eigen::Vector2f> materialtexCoords(int(serializedTexCoords.at<float>(t)));
#endif
							++t;
							UASSERT(t < serializedTexCoords.cols);
							UDEBUG("materialtexCoords=%d", (int)materialtexCoords.size());
							for(int p=0; p<(int)materialtexCoords.size(); ++p)
							{
								materialtexCoords[p][0] = serializedTexCoords.at<float>(t + p*2);
								materialtexCoords[p][1] = serializedTexCoords.at<float>(t + p*2 + 1);
							}
							t+=materialtexCoords.size()*2;
							texCoords->push_back(materialtexCoords);
						}
					}

					//opt_tex_materials
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize>0 && data)
					{
						if(textures)
						{
							*textures = uncompressImage(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
							UDEBUG("textures=%dx%d", textures->cols, textures->rows);
						}
					}
				}
			}

			rc = sqlite3_step(ppStmt); // next result...
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());

	}
	return cloud;
}

std::string DBDriverSqlite3::queryStepNode() const
{
	if(uStrNumCmp(_version, "0.18.0") >= 0)
	{
		return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, ground_truth_pose, velocity, gps, env_sensors) VALUES(?,?,?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.14.0") >= 0)
	{
		return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, ground_truth_pose, velocity, gps) VALUES(?,?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.13.0") >= 0)
	{
		return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, ground_truth_pose, velocity) VALUES(?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.11.1") >= 0)
	{
		return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, ground_truth_pose) VALUES(?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.10.1") >= 0)
	{
		return "INSERT INTO Node(id, map_id, weight, pose, stamp, label) VALUES(?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.8.8") >= 0)
	{
		return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, user_data) VALUES(?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.8.5") >= 0)
	{
		return "INSERT INTO Node(id, map_id, weight, pose, stamp, label) VALUES(?,?,?,?,?,?);";
	}
	return "INSERT INTO Node(id, map_id, weight, pose) VALUES(?,?,?,?);";
}
void DBDriverSqlite3::stepNode(sqlite3_stmt * ppStmt, const Signature * s) const
{
	UDEBUG("Save node %d", s->id());
	if(!ppStmt || !s)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;

	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, s->id());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, s->mapId());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_blob(ppStmt, index++, s->getPose().data(), s->getPose().size()*sizeof(float), SQLITE_STATIC);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(uStrNumCmp(_version, "0.8.5") >= 0)
	{
		rc = sqlite3_bind_double(ppStmt, index++, s->getStamp());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		if(s->getLabel().empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			rc = sqlite3_bind_text(ppStmt, index++, s->getLabel().c_str(), -1, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}

	std::vector<double> gps;
	std::vector<double> envSensors;
	if(uStrNumCmp(_version, "0.10.1") >= 0)
	{
		// ignore user_data

		if(uStrNumCmp(_version, "0.11.1") >= 0)
		{
			rc = sqlite3_bind_blob(ppStmt, index++, s->getGroundTruthPose().data(), s->getGroundTruthPose().size()*sizeof(float), SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

			if(uStrNumCmp(_version, "0.13.0") >= 0)
			{
				if(s->getVelocity().empty())
				{
					rc = sqlite3_bind_null(ppStmt, index++);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}
				else
				{
					rc = sqlite3_bind_blob(ppStmt, index++, s->getVelocity().data(), s->getVelocity().size()*sizeof(float), SQLITE_STATIC);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}
			}

			if(uStrNumCmp(_version, "0.14.0") >= 0)
			{
				if(s->sensorData().gps().stamp() <= 0.0)
				{
					rc = sqlite3_bind_null(ppStmt, index++);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}
				else
				{
					gps.resize(6,0.0);
					gps[0] = s->sensorData().gps().stamp();
					gps[1] = s->sensorData().gps().longitude();
					gps[2] = s->sensorData().gps().latitude();
					gps[3] = s->sensorData().gps().altitude();
					gps[4] = s->sensorData().gps().error();
					gps[5] = s->sensorData().gps().bearing();
					rc = sqlite3_bind_blob(ppStmt, index++, gps.data(), gps.size()*sizeof(double), SQLITE_STATIC);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
				}

				if(uStrNumCmp(_version, "0.18.0") >= 0)
				{
					const EnvSensors & sensors = s->sensorData().envSensors();
					if(sensors.size() == 0)
					{
						rc = sqlite3_bind_null(ppStmt, index++);
						UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					}
					else
					{

						envSensors.resize(sensors.size()*3,0.0);
						int j=0;
						for(std::map<EnvSensor::Type, EnvSensor>::const_iterator iter=sensors.begin(); iter!=sensors.end(); ++iter, j+=3)
						{
							envSensors[j] = (double)iter->second.type();
							envSensors[j+1] = iter->second.value();
							envSensors[j+2] = iter->second.stamp();
						}
						rc = sqlite3_bind_blob(ppStmt, index++, envSensors.data(), envSensors.size()*sizeof(double), SQLITE_STATIC);
						UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
					}
				}
			}
		}
	}
	else if(uStrNumCmp(_version, "0.8.8") >= 0)
	{
		if(s->sensorData().userDataCompressed().empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			rc = sqlite3_bind_blob(ppStmt, index++, s->sensorData().userDataCompressed().data, (int)s->sensorData().userDataCompressed().cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepImage() const
{
	UASSERT(uStrNumCmp(_version, "0.10.0") < 0);
	return "INSERT INTO Image(id, data) VALUES(?,?);";
}
void DBDriverSqlite3::stepImage(sqlite3_stmt * ppStmt,
		int id,
		const cv::Mat & imageBytes) const
{
	UASSERT(uStrNumCmp(_version, "0.10.0") < 0);
	UDEBUG("Save image %d (size=%d)", id, (int)imageBytes.cols);
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	int index = 1;

	rc = sqlite3_bind_int(ppStmt, index++, id);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(!imageBytes.empty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, imageBytes.data, (int)imageBytes.cols, SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepDepth() const
{
	UASSERT(uStrNumCmp(_version, "0.10.0") < 0);
	if(uStrNumCmp(_version, "0.8.11") >= 0)
	{
		return "INSERT INTO Depth(id, data, fx, fy, cx, cy, local_transform, data2d, data2d_max_pts) VALUES(?,?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.7.0") >= 0)
	{
		return "INSERT INTO Depth(id, data, fx, fy, cx, cy, local_transform, data2d) VALUES(?,?,?,?,?,?,?,?);";
	}
	else
	{
		return "INSERT INTO Depth(id, data, constant, local_transform, data2d) VALUES(?,?,?,?,?);";
	}
}
void DBDriverSqlite3::stepDepth(sqlite3_stmt * ppStmt, const SensorData & sensorData) const
{
	UASSERT(uStrNumCmp(_version, "0.10.0") < 0);
	UDEBUG("Save depth %d (size=%d) depth2d = %d",
			sensorData.id(),
			(int)sensorData.depthOrRightCompressed().cols,
			sensorData.laserScanCompressed().size());
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	int index = 1;

	rc = sqlite3_bind_int(ppStmt, index++, sensorData.id());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(!sensorData.depthOrRightCompressed().empty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, sensorData.depthOrRightCompressed().data, (int)sensorData.depthOrRightCompressed().cols, SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	float fx=0, fyOrBaseline=0, cx=0, cy=0;
	Transform localTransform = Transform::getIdentity();
	if(sensorData.cameraModels().size())
	{
		UASSERT_MSG(sensorData.cameraModels().size() == 1,
				uFormat("Database version %s doesn't support multi-camera!", _version.c_str()).c_str());

		fx = sensorData.cameraModels()[0].fx();
		fyOrBaseline = sensorData.cameraModels()[0].fy();
		cx = sensorData.cameraModels()[0].cx();
		cy = sensorData.cameraModels()[0].cy();
		localTransform = sensorData.cameraModels()[0].localTransform();
	}
	else if(sensorData.stereoCameraModels().size())
	{
		UASSERT_MSG(sensorData.stereoCameraModels().size() == 1,
				uFormat("Database version %s doesn't support multi-camera!", _version.c_str()).c_str());
		fx = sensorData.stereoCameraModels()[0].left().fx();
		fyOrBaseline = sensorData.stereoCameraModels()[0].baseline();
		cx = sensorData.stereoCameraModels()[0].left().cx();
		cy = sensorData.stereoCameraModels()[0].left().cy();
		localTransform = sensorData.stereoCameraModels()[0].left().localTransform();
	}

	if(uStrNumCmp(_version, "0.7.0") >= 0)
	{
		rc = sqlite3_bind_double(ppStmt, index++, fx);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, fyOrBaseline);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, cx);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, cy);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else
	{
		rc = sqlite3_bind_double(ppStmt, index++, 1.0f/fx);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	rc = sqlite3_bind_blob(ppStmt, index++, localTransform.data(), localTransform.size()*sizeof(float), SQLITE_STATIC);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(!sensorData.laserScanCompressed().isEmpty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, sensorData.laserScanCompressed().data().data, (int)sensorData.laserScanCompressed().size(), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(uStrNumCmp(_version, "0.8.11") >= 0)
	{
		rc = sqlite3_bind_int(ppStmt, index++, sensorData.laserScanCompressed().maxPoints());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepDepthUpdate() const
{
	if(uStrNumCmp(_version, "0.10.0") < 0)
	{
		return "UPDATE Depth SET data=? WHERE id=?;";
	}
	else
	{
		return "UPDATE Data SET depth=? WHERE id=?;";
	}
}
void DBDriverSqlite3::stepDepthUpdate(sqlite3_stmt * ppStmt, int nodeId, const cv::Mat & image) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	int index = 1;

	cv::Mat imageCompressed;
	if(!image.empty() && (image.type()!=CV_8UC1 || image.rows > 1))
	{
		// compress
		imageCompressed = compressImage2(image, ".png");
	}
	else
	{
		imageCompressed = image;
	}
	if(!imageCompressed.empty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, imageCompressed.data, (int)imageCompressed.cols, SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//id
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepScanUpdate() const
{
	UASSERT(uStrNumCmp(_version, "0.10.0") >= 0);
	if(uStrNumCmp(_version, "0.11.10") >= 0)
	{
		return "UPDATE Data SET scan_info=?, scan=? WHERE id=?;";
	}
	else if(uStrNumCmp(_version, "0.10.7") >= 0)
	{
		return "UPDATE Data SET scan_max_pts=?, scan_max_range=?, scan=? WHERE id=?;";
	}
	else
	{
		return "UPDATE Data SET scan_max_pts=? scan=? WHERE id=?;";
	}
}
void DBDriverSqlite3::stepScanUpdate(sqlite3_stmt * ppStmt, int nodeId, const LaserScan & scan) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	int index = 1;

	std::vector<float> scanInfo;
	if(uStrNumCmp(_version, "0.11.10") >= 0)
	{
		if(scan.maxPoints() > 0 ||
			scan.rangeMax() > 0 ||
			(uStrNumCmp(_version, "0.16.1")>=0 && scan.format() != LaserScan::kUnknown) ||
			(!scan.localTransform().isNull() && !scan.localTransform().isIdentity()))
		{
			if(uStrNumCmp(_version, "0.16.1") >=0)
			{
				if(uStrNumCmp(_version, "0.18.0") >=0)
				{
					scanInfo.resize(7 + Transform().size());
					scanInfo[0] = scan.format();
					scanInfo[1] = scan.rangeMin();
					scanInfo[2] = scan.rangeMax();
					scanInfo[3] = scan.angleMin();
					scanInfo[4] = scan.angleMax();
					scanInfo[5] = scan.angleIncrement();
					scanInfo[6] = scan.maxPoints(); // only for backward compatibility
					const Transform & localTransform = scan.localTransform();
					memcpy(scanInfo.data()+7, localTransform.data(), localTransform.size()*sizeof(float));
				}
				else
				{
					scanInfo.resize(3 + Transform().size());
					scanInfo[0] = scan.maxPoints();
					scanInfo[1] = scan.rangeMax();
					scanInfo[2] = scan.format();
					const Transform & localTransform = scan.localTransform();
					memcpy(scanInfo.data()+3, localTransform.data(), localTransform.size()*sizeof(float));
				}
			}
			else
			{
				scanInfo.resize(2 + Transform().size());
				scanInfo[0] = scan.maxPoints();
				scanInfo[1] = scan.rangeMax();
				const Transform & localTransform = scan.localTransform();
				memcpy(scanInfo.data()+2, localTransform.data(), localTransform.size()*sizeof(float));
			}
		}

		if(scanInfo.size())
		{
			rc = sqlite3_bind_blob(ppStmt, index++, scanInfo.data(), scanInfo.size()*sizeof(float), SQLITE_STATIC);
		}
		else
		{
			rc = sqlite3_bind_null(ppStmt, index++);
		}
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	}
	else
	{
		// scan_max_pts
		rc = sqlite3_bind_int(ppStmt, index++, scan.maxPoints());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// scan_max_range
		if(uStrNumCmp(_version, "0.10.7") >= 0)
		{
			rc = sqlite3_bind_double(ppStmt, index++, scan.rangeMax());
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}

	// scan
	cv::Mat scanCompressed;
	if(scan.isCompressed())
	{
		scanCompressed = scan.data();
	}
	else
	{
		scanCompressed = compressData2(scan.data());
	}
	if(!scanCompressed.empty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, scanCompressed.data, scanCompressed.total(), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//id
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepSensorData() const
{
	UASSERT(uStrNumCmp(_version, "0.10.0") >= 0);
	if(uStrNumCmp(_version, "0.16.0") >= 0)
	{
		return "INSERT INTO Data(id, image, depth, calibration, scan_info, scan, user_data, ground_cells, obstacle_cells, empty_cells, cell_size, view_point_x, view_point_y, view_point_z) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.11.10") >= 0)
	{
		return "INSERT INTO Data(id, image, depth, calibration, scan_info, scan, user_data, ground_cells, obstacle_cells, cell_size, view_point_x, view_point_y, view_point_z) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.10.7") >= 0)
	{
		return "INSERT INTO Data(id, image, depth, calibration, scan_max_pts, scan_max_range, scan, user_data) VALUES(?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.10.1") >= 0)
	{
		return "INSERT INTO Data(id, image, depth, calibration, scan_max_pts, scan, user_data) VALUES(?,?,?,?,?,?,?);";
	}
	else
	{
		return "INSERT INTO Data(id, image, depth, calibration, scan_max_pts, scan) VALUES(?,?,?,?,?,?);";
	}
}
void DBDriverSqlite3::stepSensorData(sqlite3_stmt * ppStmt,
		const SensorData & sensorData) const
{
	UASSERT(uStrNumCmp(_version, "0.10.0") >= 0);
	UDEBUG("Save sensor data %d (image=%d depth=%d) depth2d = %d",
			sensorData.id(),
			(int)sensorData.imageCompressed().cols,
			(int)sensorData.depthOrRightCompressed().cols,
			sensorData.laserScanCompressed().size());
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	int index = 1;

	// id
	rc = sqlite3_bind_int(ppStmt, index++, sensorData.id());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	// image
	if(!sensorData.imageCompressed().empty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, sensorData.imageCompressed().data, (int)sensorData.imageCompressed().cols, SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	// depth or right image
	if(!sensorData.depthOrRightCompressed().empty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, sensorData.depthOrRightCompressed().data, (int)sensorData.depthOrRightCompressed().cols, SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	// calibration
	std::vector<unsigned char> calibrationData;
	std::vector<float> calibration;
	// multi-cameras [fx,fy,cx,cy,width,height,local_transform, ... ,fx,fy,cx,cy,width,height,local_transform] (6+12)*float * numCameras
	// stereo [fx, fy, cx, cy, baseline, local_transform] (5+12)*float
	if(sensorData.cameraModels().size() && sensorData.cameraModels()[0].isValidForProjection())
	{
		if(uStrNumCmp(_version, "0.18.0") >= 0)
		{
			for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
			{
				UASSERT(sensorData.cameraModels()[i].isValidForProjection());
				std::vector<unsigned char> data = sensorData.cameraModels()[i].serialize();
				UASSERT(!data.empty());
				unsigned int oldSize = calibrationData.size();
				calibrationData.resize(calibrationData.size() + data.size());
				memcpy(calibrationData.data()+oldSize, data.data(), data.size());
			}
		}
		else if(uStrNumCmp(_version, "0.11.2") >= 0)
		{
			calibration.resize(sensorData.cameraModels().size() * (6+Transform().size()));
			for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
			{
				UASSERT(sensorData.cameraModels()[i].isValidForProjection());
				const Transform & localTransform = sensorData.cameraModels()[i].localTransform();
				calibration[i*(6+localTransform.size())] = sensorData.cameraModels()[i].fx();
				calibration[i*(6+localTransform.size())+1] = sensorData.cameraModels()[i].fy();
				calibration[i*(6+localTransform.size())+2] = sensorData.cameraModels()[i].cx();
				calibration[i*(6+localTransform.size())+3] = sensorData.cameraModels()[i].cy();
				calibration[i*(6+localTransform.size())+4] = sensorData.cameraModels()[i].imageWidth();
				calibration[i*(6+localTransform.size())+5] = sensorData.cameraModels()[i].imageHeight();
				memcpy(calibration.data()+i*(6+localTransform.size())+6, localTransform.data(), localTransform.size()*sizeof(float));
			}
		}
		else
		{
			calibration.resize(sensorData.cameraModels().size() * (4+Transform().size()));
			for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
			{
				UASSERT(sensorData.cameraModels()[i].isValidForProjection());
				const Transform & localTransform = sensorData.cameraModels()[i].localTransform();
				calibration[i*(4+localTransform.size())] = sensorData.cameraModels()[i].fx();
				calibration[i*(4+localTransform.size())+1] = sensorData.cameraModels()[i].fy();
				calibration[i*(4+localTransform.size())+2] = sensorData.cameraModels()[i].cx();
				calibration[i*(4+localTransform.size())+3] = sensorData.cameraModels()[i].cy();
				memcpy(calibration.data()+i*(4+localTransform.size())+4, localTransform.data(), localTransform.size()*sizeof(float));
			}
		}
	}
	else if(sensorData.stereoCameraModels().size() && sensorData.stereoCameraModels()[0].isValidForProjection())
	{
		if(uStrNumCmp(_version, "0.18.0") >= 0)
		{
			for(unsigned int i=0; i<sensorData.stereoCameraModels().size(); ++i)
			{
				UASSERT(sensorData.stereoCameraModels()[i].isValidForProjection());
				std::vector<unsigned char> data = sensorData.stereoCameraModels()[i].serialize();
				UASSERT(!data.empty());
				unsigned int oldSize = calibrationData.size();
				calibrationData.resize(calibrationData.size() + data.size());
				memcpy(calibrationData.data()+oldSize, data.data(), data.size());
			}
		}
		else
		{
			UASSERT_MSG(sensorData.stereoCameraModels().size()==1, uFormat("Database version (%s) is too old for saving multiple stereo cameras", _version.c_str()).c_str());
			const Transform & localTransform = sensorData.stereoCameraModels()[0].left().localTransform();
			calibration.resize(7+localTransform.size());
			calibration[0] = sensorData.stereoCameraModels()[0].left().fx();
			calibration[1] = sensorData.stereoCameraModels()[0].left().fy();
			calibration[2] = sensorData.stereoCameraModels()[0].left().cx();
			calibration[3] = sensorData.stereoCameraModels()[0].left().cy();
			calibration[4] = sensorData.stereoCameraModels()[0].baseline();
			calibration[5] = sensorData.stereoCameraModels()[0].left().imageWidth();
			calibration[6] = sensorData.stereoCameraModels()[0].left().imageHeight();
			memcpy(calibration.data()+7, localTransform.data(), localTransform.size()*sizeof(float));
		}
	}

	if(calibrationData.size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, calibrationData.data(), calibrationData.size(), SQLITE_STATIC);
	}
	else if(calibration.size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, calibration.data(), calibration.size()*sizeof(float), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	std::vector<float> scanInfo;
	if(uStrNumCmp(_version, "0.11.10") >= 0)
	{
		if(sensorData.laserScanCompressed().maxPoints() > 0 ||
			sensorData.laserScanCompressed().rangeMax() > 0 ||
			(uStrNumCmp(_version, "0.16.1")>=0 && sensorData.laserScanCompressed().format() != LaserScan::kUnknown) ||
			(!sensorData.laserScanCompressed().localTransform().isNull() && !sensorData.laserScanCompressed().localTransform().isIdentity()))
		{
			if(uStrNumCmp(_version, "0.16.1") >=0)
			{
				if(uStrNumCmp(_version, "0.18.0") >=0)
				{
					scanInfo.resize(7 + Transform().size());
					scanInfo[0] = sensorData.laserScanCompressed().format();
					scanInfo[1] = sensorData.laserScanCompressed().rangeMin();
					scanInfo[2] = sensorData.laserScanCompressed().rangeMax();
					scanInfo[3] = sensorData.laserScanCompressed().angleMin();
					scanInfo[4] = sensorData.laserScanCompressed().angleMax();
					scanInfo[5] = sensorData.laserScanCompressed().angleIncrement();
					scanInfo[6] = sensorData.laserScanCompressed().maxPoints(); // only for backward compatibility
					const Transform & localTransform = sensorData.laserScanCompressed().localTransform();
					memcpy(scanInfo.data()+7, localTransform.data(), localTransform.size()*sizeof(float));
				}
				else
				{
					scanInfo.resize(3 + Transform().size());
					scanInfo[0] = sensorData.laserScanCompressed().maxPoints();
					scanInfo[1] = sensorData.laserScanCompressed().rangeMax();
					scanInfo[2] = sensorData.laserScanCompressed().format();
					const Transform & localTransform = sensorData.laserScanCompressed().localTransform();
					memcpy(scanInfo.data()+3, localTransform.data(), localTransform.size()*sizeof(float));
				}
			}
			else
			{
				scanInfo.resize(2 + Transform().size());
				scanInfo[0] = sensorData.laserScanCompressed().maxPoints();
				scanInfo[1] = sensorData.laserScanCompressed().rangeMax();
				const Transform & localTransform = sensorData.laserScanCompressed().localTransform();
				memcpy(scanInfo.data()+2, localTransform.data(), localTransform.size()*sizeof(float));
			}
		}

		if(scanInfo.size())
		{
			rc = sqlite3_bind_blob(ppStmt, index++, scanInfo.data(), scanInfo.size()*sizeof(float), SQLITE_STATIC);
		}
		else
		{
			rc = sqlite3_bind_null(ppStmt, index++);
		}
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	}
	else
	{
		// scan_max_pts
		rc = sqlite3_bind_int(ppStmt, index++, sensorData.laserScanCompressed().maxPoints());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		// scan_max_range
		if(uStrNumCmp(_version, "0.10.7") >= 0)
		{
			rc = sqlite3_bind_double(ppStmt, index++, sensorData.laserScanCompressed().rangeMax());
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}

	// scan
	if(!sensorData.laserScanCompressed().isEmpty())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, sensorData.laserScanCompressed().data().data, sensorData.laserScanCompressed().size(), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(uStrNumCmp(_version, "0.10.1") >= 0)
	{
		// user_data
		if(!sensorData.userDataCompressed().empty())
		{
			rc = sqlite3_bind_blob(ppStmt, index++, sensorData.userDataCompressed().data, (int)sensorData.userDataCompressed().cols, SQLITE_STATIC);
		}
		else
		{
			rc = sqlite3_bind_null(ppStmt, index++);
		}
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	if(uStrNumCmp(_version, "0.11.10") >= 0)
	{
		//ground_cells
		if(sensorData.gridGroundCellsCompressed().empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			// compress
			rc = sqlite3_bind_blob(ppStmt, index++, sensorData.gridGroundCellsCompressed().data, (int)sensorData.gridGroundCellsCompressed().cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		//obstacle_cells
		if(sensorData.gridObstacleCellsCompressed().empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			rc = sqlite3_bind_blob(ppStmt, index++, sensorData.gridObstacleCellsCompressed().data, (int)sensorData.gridObstacleCellsCompressed().cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}

		if(uStrNumCmp(_version, "0.16.0") >= 0)
		{
			//empty_cells
			if(sensorData.gridEmptyCellsCompressed().empty())
			{
				rc = sqlite3_bind_null(ppStmt, index++);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}
			else
			{
				rc = sqlite3_bind_blob(ppStmt, index++, sensorData.gridEmptyCellsCompressed().data, (int)sensorData.gridEmptyCellsCompressed().cols, SQLITE_STATIC);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
			}
		}

		//cell_size
		rc = sqlite3_bind_double(ppStmt, index++, sensorData.gridCellSize());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

		//view_point
		rc = sqlite3_bind_double(ppStmt, index++, sensorData.gridViewPoint().x);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, sensorData.gridViewPoint().y);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, sensorData.gridViewPoint().z);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepLinkUpdate() const
{
	if(uStrNumCmp(_version, "0.13.0") >= 0)
	{
		return "UPDATE Link SET type=?, information_matrix=?, transform=?, user_data=? WHERE from_id=? AND to_id = ?;";
	}
	else if(uStrNumCmp(_version, "0.10.10") >= 0)
	{
		return "UPDATE Link SET type=?, rot_variance=?, trans_variance=?, transform=?, user_data=? WHERE from_id=? AND to_id = ?;";
	}
	else if(uStrNumCmp(_version, "0.8.4") >= 0)
	{
		return "UPDATE Link SET type=?, rot_variance=?, trans_variance=?, transform=? WHERE from_id=? AND to_id = ?;";
	}
	else if(uStrNumCmp(_version, "0.7.4") >= 0)
	{
		return "UPDATE Link SET type=?, variance=?, transform=? WHERE from_id=? AND to_id = ?;";
	}
	else
	{
		return "UPDATE Link SET type=?, transform=? WHERE from_id=? AND to_id = ?;";
	}
}
std::string DBDriverSqlite3::queryStepLink() const
{
	// from_id, to_id are at the end to match the update query above
	if(uStrNumCmp(_version, "0.13.0") >= 0)
	{
		return "INSERT INTO Link(type, information_matrix, transform, user_data, from_id, to_id) VALUES(?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.10.10") >= 0)
	{
		return "INSERT INTO Link(type, rot_variance, trans_variance, transform, user_data, from_id, to_id) VALUES(?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.8.4") >= 0)
	{
		return "INSERT INTO Link(type, rot_variance, trans_variance, transform, from_id, to_id) VALUES(?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.7.4") >= 0)
	{
		return "INSERT INTO Link(type, variance, transform, from_id, to_id) VALUES(?,?,?,?,?);";
	}
	else
	{
		return "INSERT INTO Link(type, transform, from_id, to_id) VALUES(?,?,?,?);";
	}
}
void DBDriverSqlite3::stepLink(
		sqlite3_stmt * ppStmt,
		const Link & link) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	UDEBUG("Save link from %d to %d, type=%d", link.from(), link.to(), link.type());

	// Don't save virtual links
	if(link.type()==Link::kVirtualClosure)
	{
		UDEBUG("Virtual link ignored....");
		return;
	}

	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, link.type());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(uStrNumCmp(_version, "0.13.0") >= 0)
	{
		// information_matrix
		rc = sqlite3_bind_blob(ppStmt, index++, link.infMatrix().data, (int)link.infMatrix().total()*sizeof(double), SQLITE_STATIC);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else if(uStrNumCmp(_version, "0.8.4") >= 0)
	{
		rc = sqlite3_bind_double(ppStmt, index++, link.rotVariance());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		rc = sqlite3_bind_double(ppStmt, index++, link.transVariance());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else if(uStrNumCmp(_version, "0.7.4") >= 0)
	{
		rc = sqlite3_bind_double(ppStmt, index++, link.rotVariance()<link.transVariance()?link.rotVariance():link.transVariance());
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	rc = sqlite3_bind_blob(ppStmt, index++, link.transform().data(), link.transform().size()*sizeof(float), SQLITE_STATIC);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	if(uStrNumCmp(_version, "0.10.10") >= 0)
	{
		// user_data
		if(!link.userDataCompressed().empty())
		{
			rc = sqlite3_bind_blob(ppStmt, index++, link.userDataCompressed().data, (int)link.userDataCompressed().cols, SQLITE_STATIC);
		}
		else
		{
			rc = sqlite3_bind_null(ppStmt, index++);
		}
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	rc = sqlite3_bind_int(ppStmt, index++, link.from());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, link.to());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepWordsChanged() const
{
	if(uStrNumCmp(_version, "0.13.0") >= 0)
	{
		return "UPDATE Feature SET word_id = ? WHERE word_id = ? AND node_id = ?;";
	}
	else
	{
		return "UPDATE Map_Node_Word SET word_id = ? WHERE word_id = ? AND node_id = ?;";
	}
}
void DBDriverSqlite3::stepWordsChanged(sqlite3_stmt * ppStmt, int nodeId, int oldWordId, int newWordId) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, newWordId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, oldWordId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepKeypoint() const
{
	if(uStrNumCmp(_version, "0.13.0") >= 0)
	{
		return "INSERT INTO Feature(node_id, word_id, pos_x, pos_y, size, dir, response, octave, depth_x, depth_y, depth_z, descriptor_size, descriptor) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.12.0") >= 0)
	{
		return "INSERT INTO Map_Node_Word(node_id, word_id, pos_x, pos_y, size, dir, response, octave, depth_x, depth_y, depth_z, descriptor_size, descriptor) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?);";
	}
	else if(uStrNumCmp(_version, "0.11.2") >= 0)
	{
		return "INSERT INTO Map_Node_Word(node_id, word_id, pos_x, pos_y, size, dir, response, depth_x, depth_y, depth_z, descriptor_size, descriptor) VALUES(?,?,?,?,?,?,?,?,?,?,?,?);";
	}
	return "INSERT INTO Map_Node_Word(node_id, word_id, pos_x, pos_y, size, dir, response, depth_x, depth_y, depth_z) VALUES(?,?,?,?,?,?,?,?,?,?);";
}
void DBDriverSqlite3::stepKeypoint(sqlite3_stmt * ppStmt,
		int nodeId,
		int wordId,
		const cv::KeyPoint & kp,
		const cv::Point3f & pt,
		const cv::Mat & descriptor) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, wordId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.pt.x);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.pt.y);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, kp.size);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.angle);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.response);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	if(uStrNumCmp(_version, "0.12.0") >= 0)
	{
		rc = sqlite3_bind_int(ppStmt, index++, kp.octave);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	if(uIsFinite(pt.x))
	{
		rc = sqlite3_bind_double(ppStmt, index++, pt.x);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	if(uIsFinite(pt.y))
	{
		rc = sqlite3_bind_double(ppStmt, index++, pt.y);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	if(uIsFinite(pt.z))
	{
		rc = sqlite3_bind_double(ppStmt, index++, pt.z);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else
	{
		rc = sqlite3_bind_null(ppStmt, index++);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	//descriptor
	if(uStrNumCmp(_version, "0.11.2") >= 0)
	{
		rc = sqlite3_bind_int(ppStmt, index++, descriptor.cols);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		UASSERT(descriptor.empty() || descriptor.type() == CV_32F || descriptor.type() == CV_8U);
		if(descriptor.empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
		}
		else
		{
			if(descriptor.type() == CV_32F)
			{
				// CV_32F
				rc = sqlite3_bind_blob(ppStmt, index++, descriptor.data, descriptor.cols*sizeof(float), SQLITE_STATIC);
			}
			else
			{
				// CV_8U
				rc = sqlite3_bind_blob(ppStmt, index++, descriptor.data, descriptor.cols*sizeof(char), SQLITE_STATIC);
			}
		}
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepGlobalDescriptor() const
{
	UASSERT(uStrNumCmp(_version, "0.20.0") >= 0);
	return "INSERT INTO GlobalDescriptor(node_id, type, info, data) VALUES(?,?,?,?);";
}
void DBDriverSqlite3::stepGlobalDescriptor(sqlite3_stmt * ppStmt,
		int nodeId,
		const GlobalDescriptor & descriptor) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;

	//node_if
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//type
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//info
	std::vector<unsigned char> infoBytes = rtabmap::compressData(descriptor.info());
	if(infoBytes.empty())
	{
		rc = sqlite3_bind_null(ppStmt, index++);
	}
	else
	{
		rc = sqlite3_bind_blob(ppStmt, index++, infoBytes.data(), infoBytes.size(), SQLITE_STATIC);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//data
	std::vector<unsigned char> dataBytes = rtabmap::compressData(descriptor.data());
	if(infoBytes.empty())
	{
		rc = sqlite3_bind_null(ppStmt, index++);
	}
	else
	{
		rc = sqlite3_bind_blob(ppStmt, index++, dataBytes.data(), dataBytes.size(), SQLITE_STATIC);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepOccupancyGridUpdate() const
{
	UASSERT(uStrNumCmp(_version, "0.11.10") >= 0);
	if(uStrNumCmp(_version, "0.16.0") >= 0)
	{
		return "UPDATE Data SET ground_cells=?, obstacle_cells=?, empty_cells=?, cell_size=?, view_point_x=?, view_point_y=?, view_point_z=? WHERE id=?;";
	}
	return "UPDATE Data SET ground_cells=?, obstacle_cells=?, cell_size=?, view_point_x=?, view_point_y=?, view_point_z=? WHERE id=?;";
}
void DBDriverSqlite3::stepOccupancyGridUpdate(sqlite3_stmt * ppStmt,
		int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles,
		const cv::Mat & empty,
		float cellSize,
		const cv::Point3f & viewpoint) const
{
	UASSERT(uStrNumCmp(_version, "0.11.10") >= 0);
	UASSERT(ground.empty() || ground.type() == CV_8UC1); // compressed
	UASSERT(obstacles.empty() || obstacles.type() == CV_8UC1); // compressed
	UASSERT(empty.empty() || empty.type() == CV_8UC1); // compressed
	UDEBUG("Update occupancy grid %d: ground=%d obstacles=%d empty=%d cell=%f viewpoint=(%f,%f,%f)",
			nodeId,
			ground.cols,
			obstacles.cols,
			empty.cols,
			cellSize,
			viewpoint.x,
			viewpoint.y,
			viewpoint.z);
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	int index = 1;

	//ground_cells
	if(ground.empty())
	{
		rc = sqlite3_bind_null(ppStmt, index++);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else
	{
		// compress
		rc = sqlite3_bind_blob(ppStmt, index++, ground.data, ground.cols, SQLITE_STATIC);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	//obstacle_cells
	if(obstacles.empty())
	{
		rc = sqlite3_bind_null(ppStmt, index++);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}
	else
	{
		rc = sqlite3_bind_blob(ppStmt, index++, obstacles.data, obstacles.cols, SQLITE_STATIC);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	}

	if(uStrNumCmp(_version, "0.16.0") >= 0)
	{
		//empty_cells
		if(empty.empty())
		{
			rc = sqlite3_bind_null(ppStmt, index++);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			rc = sqlite3_bind_blob(ppStmt, index++, empty.data, empty.cols, SQLITE_STATIC);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
		}
	}

	//cell_size
	rc = sqlite3_bind_double(ppStmt, index++, cellSize);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	//view_point
	rc = sqlite3_bind_double(ppStmt, index++, viewpoint.x);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, viewpoint.y);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, viewpoint.z);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	// id
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());


	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error (%s): %s", _version.c_str(), sqlite3_errmsg(_ppDb)).c_str());
}

} // namespace rtabmap
