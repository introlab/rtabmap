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

#include "DBDriverSqlite3.h"

#include "rtabmap/core/Signature.h"
#include "VisualWord.h"
#include "VWDictionary.h"
#include "DatabaseSchema_sql.h"
#include <set>

#include "utilite/UtiLite.h"

namespace rtabmap {

DBDriverSqlite3::DBDriverSqlite3(const ParametersMap & parameters) :
	DBDriver(parameters),
	_ppDb(0),
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
			this->start();
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


bool DBDriverSqlite3::connectDatabaseQuery(const std::string & url, bool overwritten)
{
	// Open a database connection
	_ppDb = 0;
	int rc = SQLITE_OK;
	bool dbFileExist = UFile::exists(url.c_str());
	if(dbFileExist && overwritten)
	{
		UFile::erase(url.c_str());
		dbFileExist = false;
	}

	if(_dbInMemory)
	{
		ULOGGER_INFO("Using database \"%s\" in the memory.", url.c_str());
		rc = sqlite3_open_v2(":memory:", &_ppDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);
	}
	else
	{
		ULOGGER_INFO("Using database \"%s\" from the hard drive.", url.c_str());
		rc = sqlite3_open_v2(url.c_str(), &_ppDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);
	}
	if(rc != SQLITE_OK)
	{
		ULOGGER_ERROR("DB error : %s", sqlite3_errmsg(_ppDb));
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
			ULOGGER_ERROR("DB error 2 : %s", sqlite3_errmsg(_ppDb));
			sqlite3_close(_ppDb);
			_ppDb = 0;
			return false;
		}
	}

	if(!dbFileExist)
	{
		ULOGGER_INFO("Database \"%s\" doesn't exist, creating a new one...", url.c_str());
		// Create the database
		std::string schema = DATABASESCHEMA_SQL;
		schema = uHex2Str(schema);
		if(this->executeNoResultQuery(schema.c_str()))
		{
			ULOGGER_DEBUG("Database schema created.");
		}
		else
		{
			ULOGGER_ERROR("Database creation failed!");
			return false;
		}
	}

	//Set database optimizations
	this->setCacheSize(_cacheSize); // this will call the SQL
	this->setJournalMode(_journalMode); // this will call the SQL

	return true;
}
void DBDriverSqlite3::disconnectDatabaseQuery()
{
	if(_ppDb)
	{
		int rc = SQLITE_OK;
		// make sure that all statements are finalized
		sqlite3_stmt * pStmt;
		while( (pStmt = sqlite3_next_stmt(_ppDb, 0))!=0 )
		{
			sqlite3_finalize(pStmt);
		}

		if(_dbInMemory)
		{
			UTimer timer;
			timer.start();
			ULOGGER_DEBUG("Saving DB ...");
			rc = loadOrSaveDb(_ppDb, this->getUrl(), 1); // Save memory to file
			ULOGGER_DEBUG("Saving DB time = %fs", timer.ticks());
		}

		// Then close (delete) the database connection
		sqlite3_close(_ppDb);
		_ppDb = 0;
	}
}

bool DBDriverSqlite3::isConnectedQuery() const
{
	return _ppDb;
}

// In bytes
bool DBDriverSqlite3::executeNoResultQuery(const std::string & sql) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc;
		rc = sqlite3_exec(_ppDb, sql.c_str(), 0, 0, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1 (rc=%d): %s", rc, sqlite3_errmsg(_ppDb));
			ULOGGER_ERROR("The query is: %s", sql.c_str());
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

long DBDriverSqlite3::getMemoryUsedQuery() const
{
	if(_dbInMemory)
	{
		return sqlite3_memory_used();
	}
	else
	{
		return UFile::length(this->getUrl());
	}
}

bool DBDriverSqlite3::getImageQuery(int id, IplImage ** image) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		*image = 0;

		query << "SELECT width, height, channels, compressed, data "
			  << "FROM Image "
			  << "WHERE id = " << id <<";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int width;
		int height;
		int channels;
		int compressed;
		const void * data = 0;
		int dataSize = 0;
		int index = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			width = sqlite3_column_int(ppStmt, index++);
			height = sqlite3_column_int(ppStmt, index++);
			channels = sqlite3_column_int(ppStmt, index++);
			compressed = sqlite3_column_int(ppStmt, index++);

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);

			//Create the image
			if(dataSize>4 && data)
			{
				if(compressed)
				{
					CvMat * compressed = cvCreateMat(1, dataSize, CV_8UC1);
					memcpy(compressed->data.ptr, data, dataSize);
					// old method, assuming compressed
					*image = cvDecodeImage(compressed, CV_LOAD_IMAGE_ANYCOLOR);
				}
				else
				{
					UDEBUG("width=%d,height=%d channels=%d", width, height, channels);
					*image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, channels);
					UDEBUG("");
					memcpy((*image)->imageData, data, dataSize);
				}
			}

			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to received only 1 result... ");
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_WARN("No result !?! from the DB");
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::getAllSignatureIdsQuery(std::set<int> & ids) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT id "
			  << "FROM Signature "
			  << "ORDER BY id";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			ids.insert(ids.end(), sqlite3_column_int(ppStmt, 0)); // Signature Id
			rc = sqlite3_step(ppStmt);
		}
		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%f", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::getLastSignatureIdQuery(int & id) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		//query.append("BEGIN TRANSACTION;");

		// Create a new entry in table KeypointSignature
		query << "SELECT max(id) "
			  << "FROM Signature;";

		//query.append("COMMIT;");

		//ULOGGER_DEBUG("DBDriverSqlite3::getLastSignatureId() Execute query : %s", query.toStdString().c_str());

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); // Signature Id
			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to received only 1 result... ");
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB");
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::getLastVisualWordIdQuery(int & id) const
{
	if(_ppDb && id > 0)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		//query.append("BEGIN TRANSACTION;");

		// Create a new entry in table KeypointSignature
		query << "SELECT max(id) "
			  << "FROM VisualWord;";

		//query.append("COMMIT;");

		//ULOGGER_DEBUG("DBDriverSqlite3::getLastSignatureId() Execute query : %s", query.toStdString().c_str());

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); // VisualWord Id
			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to received only 1 result... ");
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB");
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::getSurfNiQuery(int signatureId, int & ni) const
{
	ni = 0;
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		// Create a new entry in table KeypointSignature
		query << "SELECT count(visualWordId) "
			  << "FROM Map_SS_VW "
			  << "WHERE signatureId=" << signatureId << ";";

		//query.append("COMMIT;");

		//ULOGGER_DEBUG("DBDriverSqlite3::getSurfNi() Execute query : %s", query.toStdString().c_str());

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			ni = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to received only 1 result... signature=%d", signatureId);
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB, signature=%d",signatureId);
		}


		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

// default onlyWithActions = false
bool DBDriverSqlite3::getNeighborIdsQuery(int signatureId, std::list<int> & neighbors, bool onlyWithActions) const
{
	neighbors.clear();
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT DISTINCT nid FROM Neighbor "
			  << "WHERE sid = " << signatureId;

		if(onlyWithActions)
		{
			query << " AND actions IS NOT NULL";
		}

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			neighbors.push_back(sqlite3_column_int(ppStmt, 0)); // nid
			rc = sqlite3_step(ppStmt);
		}
		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		if(neighbors.size() == 0)
		{
			UERROR("No neighbors loaded form signature %d", signatureId);
		}
		return true;
	}
	return false;
}

bool DBDriverSqlite3::getWeightQuery(int signatureId, int & weight) const
{
	weight = 0;
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT weight FROM signature WHERE id =  "
			  << signatureId
			  << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			weight= sqlite3_column_int(ppStmt, 0); // weight
			rc = sqlite3_step(ppStmt);
		}
		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		return true;
	}
	return false;
}

bool DBDriverSqlite3::getLoopClosureIdsQuery(int signatureId, std::set<int> & loopIds, std::set<int> & childIds) const
{
	loopIds.clear();
	childIds.clear();
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT loopClosureIds, childLoopClosureIds FROM signature WHERE id =  "
			  << signatureId
			  << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			int index = 0;
			int dataSize = sqlite3_column_bytes(ppStmt, index); // loopClosureIds
			if(dataSize)
			{
				std::vector<int> vecLoopIds(dataSize/sizeof(int));
				memcpy(vecLoopIds.data(), sqlite3_column_blob(ppStmt, index++), dataSize);
				loopIds.insert(vecLoopIds.begin(), vecLoopIds.end());
				loopIds.erase(0);
			}
			dataSize = sqlite3_column_bytes(ppStmt, index); // childLoopClosureIds
			if(dataSize)
			{
				std::vector<int> vecLoopIds(dataSize/sizeof(int));
				memcpy(vecLoopIds.data(), sqlite3_column_blob(ppStmt, index++), dataSize);
				childIds.insert(vecLoopIds.begin(), vecLoopIds.end());
				childIds.erase(0);
			}
			rc = sqlite3_step(ppStmt);
		}
		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		return true;
	}
	return false;
}

//return <weight,id>
bool DBDriverSqlite3::getHighestWeightedSignaturesQuery(unsigned int count, std::multimap<int,int> & ids) const
{
	if(_ppDb && count)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT weight,id "
			  << "FROM Signature "
			  /*<< "WHERE loopClosureIds = 0 "*/
			  << "ORDER BY weight DESC "
			  << "LIMIT " << count << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		int weight = 0;
		int id = 0;
		while(rc == SQLITE_ROW)
		{
			weight = sqlite3_column_int(ppStmt, 0); // Weight
			id = sqlite3_column_int(ppStmt, 1); // Signature Id
			if(ids.size() < count ||
			   (ids.size() && ids.begin()->first < weight))
			{
				ids.insert(std::pair<int,int>(weight, id));
			}
			if(ids.size() >= count)
			{
				ids.erase(ids.begin());
			}
			rc = sqlite3_step(ppStmt);
		}
		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%f", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::loadQuery(int signatureId, Signature ** s) const
{
	ULOGGER_DEBUG("Signature");
	*s = 0;
	if(_ppDb)
	{
		std::string type;
		int weight = 0;
		std::set<int> loopClosureIds;
		std::set<int> childLoopClosureIds;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT type, weight, loopClosureIds, childLoopClosureIds "
			  << "FROM Signature "
			  << "WHERE id=" << signatureId << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			int index = 0;
			type = std::string((const char *)sqlite3_column_text(ppStmt, index++)); // Signature type
			weight = sqlite3_column_int(ppStmt, index++); // weight
			int dataSize = sqlite3_column_bytes(ppStmt, index); // loopClosureIds
			if(dataSize)
			{
				std::vector<int> vecLoopIds(dataSize/sizeof(int));
				memcpy(vecLoopIds.data(), sqlite3_column_blob(ppStmt, index++), dataSize);
				loopClosureIds.insert(vecLoopIds.begin(), vecLoopIds.end());
				loopClosureIds.erase(0);
			}
			dataSize = sqlite3_column_bytes(ppStmt, index); // childLoopClosureIds
			if(dataSize)
			{
				std::vector<int> vecLoopIds(dataSize/sizeof(int));
				memcpy(vecLoopIds.data(), sqlite3_column_blob(ppStmt, index++), dataSize);
				childLoopClosureIds.insert(vecLoopIds.begin(), vecLoopIds.end());
				childLoopClosureIds.erase(0);
			}
			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to received only 1 result... signature=%d", signatureId);
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB, signature=%d",signatureId);
		}


		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		//get neighbors
		NeighborsMultiMap neighbors;
		this->loadNeighborsQuery(signatureId, neighbors);

		if(type.compare("KeypointSignature") == 0)
		{
			*s = new KeypointSignature(signatureId);
			if(*s)
			{
				(*s)->setWeight(weight);
				(*s)->setLoopClosureIds(loopClosureIds);
				(*s)->setChildLoopClosureIds(childLoopClosureIds);
				(*s)->addNeighbors(neighbors);
				if(this->loadQuery(signatureId, (KeypointSignature*)*s))
				{
					(*s)->setSaved(true);
					(*s)->setModified(false);
					return true;
				}
				else if(*s)
				{
					delete *s;
					*s = 0;
				}
			}
			else
			{
				UFATAL("Cannot allocate memory !!!");
			}
		}
		else if(type.compare("SMSignature") == 0)
		{
			*s = new SMSignature(signatureId);
			if(*s)
			{
				(*s)->setWeight(weight);
				(*s)->setLoopClosureIds(loopClosureIds);
				(*s)->setChildLoopClosureIds(childLoopClosureIds);
				(*s)->addNeighbors(neighbors);
				if(this->loadQuery(signatureId, (SMSignature*)*s))
				{
					(*s)->setSaved(true);
					(*s)->setModified(false);
					return true;
				}
				else if(*s)
				{
					delete *s;
					*s = 0;
				}
			}
			else
			{
				UFATAL("Cannot allocate memory !!!");
			}
		}

		return false; // signature not created
	}
	return false;
}

bool DBDriverSqlite3::loadQuery(int signatureId, KeypointSignature * ss) const
{
	UDEBUG("KeypointSignature %d", signatureId);
	if(_ppDb && ss)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::multimap<int, cv::KeyPoint> visualWords;

		// Get the map from signature and visual words
		query << "SELECT visualWordId, pos_x, pos_y, laplacian, size, dir, hessian "
			  << "FROM Map_SS_VW "
			  << "WHERE signatureId=" << signatureId
			  << " ORDER BY visualWordId;"; // used for fast insertion below

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		int visualWordId = 0;
		float pos_x = 0;
		float pos_y = 0;
		int laplacian = 0;
		int size = 0;
		float dir = 0;
		float hessian = 0;
		while(rc == SQLITE_ROW)
		{
			int index=0;
			visualWordId = sqlite3_column_int(ppStmt, index++);
			pos_x = sqlite3_column_double(ppStmt, index++);
			pos_y = sqlite3_column_double(ppStmt, index++);
			laplacian = sqlite3_column_int(ppStmt, index++);
			size = sqlite3_column_int(ppStmt, index++);
			dir = sqlite3_column_double(ppStmt, index++);
			hessian = sqlite3_column_double(ppStmt, index++);
			visualWords.insert(visualWords.end(), std::pair<int, cv::KeyPoint>(visualWordId, cv::KeyPoint(cv::Point2f(pos_x,pos_y),size,dir,hessian, 0, -1)));
			rc = sqlite3_step(ppStmt);
		}
		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ss->setWords(visualWords);

		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::loadQuery(int signatureId, SMSignature * sm) const
{
	ULOGGER_DEBUG("SMSignature");
	if(_ppDb && sm)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::vector<int> sensors;
		std::vector<unsigned char> motionMask;

		// Get the map from signature
		query << "SELECT sensors, motionMask "
			  << "FROM SMState "
			  << "WHERE id=" << signatureId;

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		const void * data = 0;
		int dataSize = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			int index = 0;
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			//Create the sensors
			if(dataSize>4 && data)
			{
				UDEBUG("dataSize=%d", dataSize);
				sensors = std::vector<int>(dataSize/sizeof(int));
				memcpy(&(sensors[0]), data, dataSize);
			}

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			//Create the motion mask
			if(dataSize>4 && data)
			{
				UDEBUG("dataSize=%d", dataSize);
				motionMask = std::vector<unsigned char>(dataSize/sizeof(unsigned char));
				memcpy(&(motionMask[0]), data, dataSize);
			}

			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to received only 1 result... ");
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_WARN("No result !?! from the DB");
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		sm->setSensors(sensors);
		sm->setMotionMask(motionMask);

		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}


/*
bool DBDriverSqlite3::loadKeypointSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures, bool onlyParents) const
{
	ULOGGER_DEBUG("count=%d", ids.size());
	if(_ppDb && ids.size())
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::multimap<int, cv::KeyPoint> visualWords;
		unsigned int loaded = 0;

		// Prepare the query... Get the map from signature and visual words
		query << "SELECT id, weight, loopClosureIds, imgWidth, imgHeight, visualWordId, pos_x, pos_y, laplacian, size, dir, hessian "
				 "FROM Map_SS_VW "
				 "INNER JOIN Signature "
				 "ON Signature.id = signatureId "
				 "WHERE type = 'KeypointSignature' ";

		if(onlyParents)
		{
			query << "AND loopClosureIds = 0 ";
		}
		query << "AND (signatureId=";
		for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end();)
		{
			UDEBUG("Loading %d", *iter);
			query << *iter;
			if(++iter != ids.end())
			{
				query << " or signatureId=";
			}
		}
		query << ") ";
		query << "ORDER BY signatureId AND visualWordId"; // Needed for fast insertion below
		query << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int id = 0;
		int weight = 0;
		int loopClosureIds = 0;
		int imgWidth = 0;
		int imgHeight = 0;
		int visualWordId = 0;
		float pos_x = 0;
		float pos_y = 0;
		int laplacian = 0;
		int size = 0;
		float dir = 0;
		float hessian = 0;

		int lastId = 0;
		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); // Signature Id
			if(id!=lastId && lastId > 0)
			{
				ULOGGER_DEBUG("Creating %d with %d keypoints", lastId, visualWords.size());
				++loaded;
				KeypointSignature * ss = new KeypointSignature(visualWords, lastId);
				if(ss)
				{
					ss->setWeight(weight);
					ss->setLoopClosureId(loopClosureIds);
					ss->setWidth(imgWidth);
					ss->setHeight(imgHeight);
					ss->setSaved(true);
					signatures.push_back(ss);
				}
				visualWords.clear();

				weight = sqlite3_column_int(ppStmt, 1); // weight
				loopClosureIds = sqlite3_column_int(ppStmt, 2); // loopClosureIds
				imgWidth = sqlite3_column_int(ppStmt, 3); // imgWidth
				imgHeight = sqlite3_column_int(ppStmt, 4); // imgheight
			}
			else if(lastId == 0)
			{
				weight = sqlite3_column_int(ppStmt, 1); // weight
				loopClosureIds = sqlite3_column_int(ppStmt, 2); // loopClosureIds
				imgWidth = sqlite3_column_int(ppStmt, 3); // imgWidth
				imgHeight = sqlite3_column_int(ppStmt, 4); // imgheight
			}
			lastId = id;
			visualWordId = sqlite3_column_int(ppStmt, 5);
			pos_x = sqlite3_column_double(ppStmt, 6);
			pos_y = sqlite3_column_double(ppStmt, 7);
			laplacian = sqlite3_column_int(ppStmt, 8);
			size = sqlite3_column_int(ppStmt, 9);
			dir = sqlite3_column_double(ppStmt, 10);
			hessian = sqlite3_column_double(ppStmt, 11);

			visualWords.insert(visualWords.end(), std::pair<int, cv::KeyPoint>(visualWordId, cv::KeyPoint(cv::Point2f(pos_x,pos_y),size,dir,hessian, 0, -1)));
			rc = sqlite3_step(ppStmt);
		}

		// create the last signature
		if(lastId)
		{
			++loaded;
			ULOGGER_DEBUG("Creating %d with %d keypoints", lastId, visualWords.size());
			KeypointSignature * ss = new KeypointSignature(visualWords, lastId);
			if(ss)
			{
				ss->setWeight(weight);
				ss->setLoopClosureId(loopClosureIds);
				ss->setWidth(imgWidth);
				ss->setHeight(imgHeight);
				ss->setSaved(true);
				signatures.push_back(ss);
			}
		}

		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2.2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		for(std::list<Signature*>::iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			NeighborsMap neighbors;
			this->loadNeighborsQuery((*i)->id(), neighbors);
			(*i)->addNeighbors(neighbors);
		}
		ULOGGER_DEBUG("Time load neighbors=%fs", timer.ticks());

		if(ids.size() != loaded && !onlyParents)
		{
			UERROR("Some signatures not found in database");
		}

		return true;
	}
	return false;
}*/

//may be slower than the previous version but don't have a limit of words that can be loaded at the same time
bool DBDriverSqlite3::loadKeypointSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const
{
	ULOGGER_DEBUG("count=%d", ids.size());
	if(_ppDb && ids.size())
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::multimap<int, cv::KeyPoint> visualWords;
		unsigned int loaded = 0;

		// Prepare the query... Get the map from signature and visual words
		query << "SELECT id, weight, loopClosureIds, childLoopClosureIds, visualWordId, pos_x, pos_y, laplacian, size, dir, hessian "
				 "FROM Map_SS_VW "
				 "INNER JOIN Signature "
				 "ON Signature.id = signatureId "
				 "WHERE type = 'KeypointSignature' AND signatureId = ? ";

		/*if(onlyParents)
		{
			query << "AND loopClosureIds = 0";
		}*/
		query << " ORDER BY visualWordId"; // Needed for fast insertion below
		query << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int id = 0;
		int weight = 0;
		std::set<int> loopClosureIds;
		std::set<int> childLoopClosureIds;
		int visualWordId = 0;
		float pos_x = 0;
		float pos_y = 0;
		int laplacian = 0;
		int size = 0;
		float dir = 0;
		float hessian = 0;

		for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
		{
			ULOGGER_DEBUG("Loading %d...", *iter);
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, *iter);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			id = 0;
			weight = 0;
			loopClosureIds.clear();
			childLoopClosureIds.clear();
			visualWordId = 0;
			pos_x = 0;
			pos_y = 0;
			laplacian = 0;
			size = 0;
			dir = 0;
			hessian = 0;
			visualWords.clear();
			const void * data = 0;
			int dataSize = 0;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;
				if(id==0)
				{
					id = sqlite3_column_int(ppStmt, index++); // Signature Id
					weight = sqlite3_column_int(ppStmt, index++); // weight

					// loopClosureIds
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize)
					{
						std::vector<int> vecLoopIds(dataSize/sizeof(int));
						memcpy(vecLoopIds.data(), data, dataSize);
						loopClosureIds.insert(vecLoopIds.begin(), vecLoopIds.end());
						loopClosureIds.erase(0);
					}

					// childLoopClosureIds
					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);
					if(dataSize)
					{
						std::vector<int> vecLoopIds(dataSize/sizeof(int));
						memcpy(vecLoopIds.data(), data, dataSize);
						childLoopClosureIds.insert(vecLoopIds.begin(), vecLoopIds.end());
						childLoopClosureIds.erase(0);
					}
				}
				else
				{
					index = 4;
				}
				visualWordId = sqlite3_column_int(ppStmt, index++);
				pos_x = sqlite3_column_double(ppStmt, index++);
				pos_y = sqlite3_column_double(ppStmt, index++);
				laplacian = sqlite3_column_int(ppStmt, index++);
				size = sqlite3_column_int(ppStmt, index++);
				dir = sqlite3_column_double(ppStmt, index++);
				hessian = sqlite3_column_double(ppStmt, index++);
				visualWords.insert(visualWords.end(), std::pair<int, cv::KeyPoint>(visualWordId, cv::KeyPoint(cv::Point2f(pos_x,pos_y),size,dir,hessian, 0, -1)));
				rc = sqlite3_step(ppStmt);
			}
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			// create the signature
			if(id)
			{
				ULOGGER_DEBUG("Creating %d with %d keypoints", *iter, visualWords.size());
				KeypointSignature * ss = new KeypointSignature(visualWords, id);
				if(ss)
				{
					ss->setWeight(weight);
					ss->setLoopClosureIds(loopClosureIds);
					ss->setChildLoopClosureIds(childLoopClosureIds);
					ss->setSaved(true);
					signatures.push_back(ss);
				}
				else
				{
					UFATAL("?Cannot allocate memory !!!");
				}
				++loaded;
			}
			else
			{
				ULOGGER_ERROR("Signature %d not found in database", *iter);
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			//reset
			rc = sqlite3_reset(ppStmt);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		this->loadNeighborsQuery(signatures);
		for(std::list<Signature*>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
		{
			(*iter)->setModified(false);
		}
		ULOGGER_DEBUG("Time load neighbors=%fs", timer.ticks());

		if(ids.size() != loaded)
		{
			UERROR("Some signatures not found in database");
		}

		return true;
	}
	return false;
}

bool DBDriverSqlite3::loadSMSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & signatures) const
{
	ULOGGER_DEBUG("count=%d", ids.size());
	if(_ppDb && ids.size())
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		unsigned int loaded = 0;

		// Prepare the query...
		query << "SELECT weight, loopClosureIds, childLoopClosureIds, sensors, motionMask "
			  << "FROM SMState "
			  << "INNER JOIN Signature "
			  << "ON Signature.id = SMState.id "
			  << "WHERE type = 'SMSignature' AND Signature.id=? ";
		query << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
		{
			ULOGGER_DEBUG("Loading %d...", *iter);
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, *iter);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			int weight = 0;
			std::set<int> loopClosureIds;
			std::set<int> childLoopClosureIds;
			std::vector<int> sensors;
			std::vector<unsigned char> motionMask;
			const void * data = 0;
			int dataSize = 0;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				int index = 0;

				// weight
				weight = sqlite3_column_int(ppStmt, index++);

				// loopClosureIds
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				if(dataSize)
				{
					std::vector<int> vecLoopIds(dataSize/sizeof(int));
					memcpy(vecLoopIds.data(), data, dataSize);
					loopClosureIds.insert(vecLoopIds.begin(), vecLoopIds.end());
					loopClosureIds.erase(0);
				}

				// childLoopClosureIds
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				if(dataSize)
				{
					std::vector<int> vecLoopIds(dataSize/sizeof(int));
					memcpy(vecLoopIds.data(), data, dataSize);
					childLoopClosureIds.insert(vecLoopIds.begin(), vecLoopIds.end());
					childLoopClosureIds.erase(0);
				}

				//sensors
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				//Create the sensors
				if(dataSize>4 && data)
				{
					sensors = std::vector<int>(dataSize/sizeof(int));
					memcpy(&(sensors[0]), data, dataSize);
				}

				// motionMask
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				//Create the motion mask
				if(dataSize>4 && data)
				{
					motionMask = std::vector<unsigned char>(dataSize/sizeof(unsigned char));
					memcpy(&(motionMask[0]), data, dataSize);
				}

				rc = sqlite3_step(ppStmt);
				if(rc != SQLITE_DONE)
				{
					ULOGGER_ERROR("Supposed to received only 1 result... signature=%d", *iter);
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				ULOGGER_DEBUG("Creating %d", *iter);
				SMSignature * sm = new SMSignature(sensors, motionMask, *iter);
				if(sm)
				{
					sm->setWeight(weight);
					sm->setLoopClosureIds(loopClosureIds);
					sm->setChildLoopClosureIds(childLoopClosureIds);
					sm->setSaved(true);
					signatures.push_back(sm);
				}
				else
				{
					UFATAL("Cannot allocate memory !!!");
				}
				++loaded;
			}
			else
			{
				ULOGGER_WARN("No result !?! from the DB for location %d", *iter);
			}

			//reset
			rc = sqlite3_reset(ppStmt);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		this->loadNeighborsQuery(signatures);
		for(std::list<Signature*>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
		{
			(*iter)->setModified(false);
		}
		ULOGGER_DEBUG("Time load neighbors=%fs", timer.ticks());

		if(ids.size() != loaded)
		{
			UERROR("Some signatures not found in database");
		}

		return true;
	}
	return false;
}


bool DBDriverSqlite3::loadLastSignaturesQuery(std::list<Signature *> & signatures) const
{
	ULOGGER_DEBUG("");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::list<int> ids;

		// Get the map from signature and visual words
		query << "SELECT s.id "
				 "FROM Signature AS s "
				 "WHERE s.timeEnter >= (SELECT MAX(timeEnter) FROM StatisticsAfterRun);"
				 /*"AND s.parentId IS NULL;"*/;

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);

		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			ids.push_back(sqlite3_column_int(ppStmt, 0)); 	// Signature id
			rc = sqlite3_step(ppStmt); // next result...
		}

		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ULOGGER_DEBUG("Loading %d signatures...", ids.size());
		Signature * s = 0;
		int count = 0;
		for(std::list<int>::iterator i=ids.begin(); i!=ids.end(); ++i)
		{
			this->loadQuery(*i, &s);
			if(s)
			{
				++count;
				signatures.push_back(s);
			}
		}

		ULOGGER_DEBUG("loaded=%d, Time=%fs", count, timer.ticks());

		return true;
	}
	return false;
}

// TODO DO IT IN A MORE EFFICiENT WAY !!!!
bool DBDriverSqlite3::loadQuery(VWDictionary * dictionary) const
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

		// Get the map from signature and visual words
		query << "SELECT vw.id, vw.descriptorSize, vw.descriptor, m.signatureId "
				 "FROM VisualWord as vw "
				 "INNER JOIN Map_SS_VW as m "
				 "ON vw.id=m.visualWordId "
				 "INNER JOIN Signature as s "
				 "ON s.id=m.signatureId "
				 "WHERE s.timeEnter >= (SELECT MAX(timeEnter) FROM StatisticsAfterRun) "
				 "ORDER BY vw.id;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Process the result if one
		int id = 0;
		int lastId = 0;
		int descriptorSize = 0;
		const void * descriptor = 0;
		int dRealSize;
		int signatureId;
		rc = sqlite3_step(ppStmt);
		int count = 0;
		while(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); 			// VisualWord Id
			if(id>0)
			{
				if(id != lastId)
				{
					descriptorSize = sqlite3_column_int(ppStmt, 1); // VisualWord descriptor size
					descriptor = sqlite3_column_blob(ppStmt, 2); 	// VisualWord descriptor array
					dRealSize = sqlite3_column_bytes(ppStmt, 2);
				}
				lastId = id;
				signatureId = sqlite3_column_int(ppStmt, 3); 	// Signature ref Id
				if(dictionary->getWord(id) != 0)
				{
					// Use VWDictionary::addWordRef instead of VisualWord::addRef()
					// This will increment _totalActiveReferences in the dictionary
					dictionary->addWordRef(id, signatureId);
				}
				else
				{
					VisualWord * vw = new VisualWord(id, &((const float *)descriptor)[0], descriptorSize, signatureId);
					if(vw)
					{
						vw->setSaved(true);
						dictionary->addWord(vw);
					}
					else
					{
						ULOGGER_ERROR("Couldn't create a Visual word!?");
					}
				}
			}
			else
			{
				ULOGGER_ERROR("Wrong word id ?!? (%d)", id);
			}
			if(++count % 5000 == 0)
			{
				ULOGGER_DEBUG("Loaded %d word references...", count);
			}
			rc = sqlite3_step(ppStmt); // next result...
		}

		getLastVisualWordId(id);
		dictionary->setLastWordId(id);

		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}
/*
bool DBDriverSqlite3::loadWordsQuery(const std::list<int> & wordIds, std::list<VisualWord *> & vws) const
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
		query << "SELECT vw.id, vw.laplacian, vw.descriptorSize, vw.descriptor "
				 "FROM VisualWord as vw "
				 "WHERE vw.id = ";

		for(std::list<int>::const_iterator i=wordIds.begin(); i != wordIds.end();)
		{
			query << *i << " ";

			if(++i != wordIds.end())
			{
				query << "or vw.id = ";
			}
		}

		query << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Process the result if one
		int id=0;
		int laplacian;
		int descriptorSize;
		const void * descriptor;
		int dRealSize;
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); 			// VisualWord Id
			laplacian = sqlite3_column_int(ppStmt, 1); 		// VisualWord laplacian
			descriptorSize = sqlite3_column_int(ppStmt, 2); // VisualWord descriptor size
			descriptor = sqlite3_column_blob(ppStmt, 3); 	// VisualWord descriptor array
			dRealSize = sqlite3_column_bytes(ppStmt, 3);

			VisualWord * vw = new VisualWord(id, &((const float *)descriptor)[0], descriptorSize, laplacian);
			if(vw)
			{
				vw->setSaved(true);
			}
			vws.push_back(vw);
			loaded.insert(loaded.end(), id);

			rc = sqlite3_step(ppStmt); // next result...
		}

		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		if(wordIds.size() != loaded.size())
		{
			UERROR("Query (%d) doesn't match loaded words (%d)", wordIds.size(), loaded.size());
			UDEBUG("Query = %s", query.str().c_str());
			for(std::list<int>::const_iterator iter = wordIds.begin(); iter!=wordIds.end(); ++iter)
			{
				if(loaded.find(*iter) == loaded.end())
				{
					UDEBUG("Not found word %d", *iter);
				}
			}

		}

		return true;
	}
	return false;
}*/

//may be slower than the previous version but don't have a limit of words that can be loaded at the same time
bool DBDriverSqlite3::loadWordsQuery(const std::list<int> & wordIds, std::list<VisualWord *> & vws) const
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
		query << "SELECT vw.id, vw.descriptorSize, vw.descriptor "
				 "FROM VisualWord as vw "
				 "WHERE vw.id = ?;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int id=0;
		int descriptorSize;
		const void * descriptor;
		int dRealSize;
		for(std::list<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
		{
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, *iter);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 2.1: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				id = sqlite3_column_int(ppStmt, 0); 			// VisualWord Id
				descriptorSize = sqlite3_column_int(ppStmt, 1); // VisualWord descriptor size
				descriptor = sqlite3_column_blob(ppStmt, 2); 	// VisualWord descriptor array
				dRealSize = sqlite3_column_bytes(ppStmt, 2);

				VisualWord * vw = new VisualWord(id, &((const float *)descriptor)[0], descriptorSize);
				if(vw)
				{
					vw->setSaved(true);
				}
				vws.push_back(vw);
				loaded.insert(loaded.end(), id);

				rc = sqlite3_step(ppStmt); // next result...
			}

			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("DB error 2.2: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			rc = sqlite3_reset(ppStmt);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 2.3: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		if(wordIds.size() != loaded.size())
		{
			for(std::list<int>::const_iterator iter = wordIds.begin(); iter!=wordIds.end(); ++iter)
			{
				if(loaded.find(*iter) == loaded.end())
				{
					UDEBUG("Not found word %d", *iter);
				}
			}
			UERROR("Query (%d) doesn't match loaded words (%d)", wordIds.size(), loaded.size());
		}

		return true;
	}
	return false;
}

bool DBDriverSqlite3::loadQuery(int wordId, VisualWord ** vw) const
{
	*vw = 0;
	//ULOGGER_DEBUG("DBDriverSqlite3::load(int, VisualWord **)");
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		// Get the map from signature and visual words
		query << "SELECT vw.id, vw.descriptorSize, vw.descriptor "
				 "FROM VisualWord AS vw "
				 "WHERE vw.id = " << wordId << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Process the result if one
		int id=0;
		int descriptorSize;
		const void * descriptor;
		int dRealSize;

		rc = sqlite3_step(ppStmt);

		if(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); 			// VisualWord Id
			descriptorSize = sqlite3_column_int(ppStmt, 1); // VisualWord descriptor size
			descriptor = sqlite3_column_blob(ppStmt, 2); 	// VisualWord descriptor array
			dRealSize = sqlite3_column_bytes(ppStmt, 2);

			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to received only 1 result... wordId=%d", wordId);
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			//Create the word
			*vw = new VisualWord(id, &((const float *)descriptor)[0], descriptorSize);
			if(*vw)
			{
				(*vw)->setSaved(true);
			}
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB, wordId=%d", wordId);
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		//ULOGGER_DEBUG("DBDriverSqlite3::load(int, VisualWord **) Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::loadNeighborsQuery(int signatureId, NeighborsMultiMap & neighbors) const
{
	neighbors.clear();
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT nid, actionSize, actions, baseIds FROM Neighbor "
			  << "WHERE sid = " << signatureId;

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int nid;
		int actionSize;
		int dataSize;
		std::list<std::vector<float> > actions;
		std::vector<int> baseIds;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			int index = 0;
			actions.clear();
			nid = sqlite3_column_int(ppStmt, index++);
			actionSize = sqlite3_column_int(ppStmt, index++);
			const float * data = (const float *)sqlite3_column_blob(ppStmt, index); 	// actions array
			dataSize = sqlite3_column_bytes(ppStmt, index++)/sizeof(float);

			if(actionSize)
			{
				//UDEBUG("%d=%s", nid, uBytes2Hex((char *)data, dataSize*sizeof(float)).c_str());
				//UDEBUG("actionSize=%d, dataSize=%d", actionSize, dataSize);

				//std::stringstream stream;
				//stream << nid << "=";
				for(int i=0; i<dataSize; i+=actionSize)
				{
					//stream << "[";
					std::vector<float> action(actionSize);
					for(int j=0; j<actionSize; ++j)
					{
						action[j] = data[i + j];
						//stream << action[j] << ",";
					}
					actions.push_back(action);
					//stream << "],";
				}
				//UDEBUG("%s", stream.str().c_str());
			}

			dataSize = sqlite3_column_bytes(ppStmt, index);
			const void * dataVoid = sqlite3_column_blob(ppStmt, index++);
			if(dataSize>4)
			{
				baseIds = std::vector<int>(dataSize/sizeof(int));
				memcpy(baseIds.data(), dataVoid, dataSize);
			}

			neighbors.insert(neighbors.end(), std::pair<int, NeighborLink>(nid, NeighborLink(nid, actions, baseIds))); // nid
			rc = sqlite3_step(ppStmt);
		}
		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		if(neighbors.size() == 0)
		{
			UERROR("No neighbors loaded form signature %d", signatureId);
		}

		return true;
	}
	return false;
}

bool DBDriverSqlite3::loadNeighborsQuery(std::list<Signature *> & signatures) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT nid, actionSize, actions, baseIds FROM Neighbor "
			  << "WHERE sid = ? "
			  << "ORDER BY nid;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		for(std::list<Signature*>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			int nid;
			int actionSize;
			int dataSize;
			std::list<std::vector<float> > actions;
			std::vector<int> baseIds;
			NeighborsMultiMap neighbors;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				actions.clear();
				int index = 0;
				nid = sqlite3_column_int(ppStmt, index++);
				actionSize = sqlite3_column_int(ppStmt, index++);
				const float * data = (const float *)sqlite3_column_blob(ppStmt, index); 	// actions array
				dataSize = sqlite3_column_bytes(ppStmt, index++)/sizeof(float);

				if(actionSize)
				{
					//UDEBUG("%d=%s", nid, uBytes2Hex((char *)data, dataSize*sizeof(float)).c_str());
					//UDEBUG("actionSize=%d, dataSize=%d", actionSize, dataSize);

					//std::stringstream stream;
					//stream << nid << "=";
					for(int i=0; i<dataSize; i+=actionSize)
					{
						//stream << "[";
						std::vector<float> action(actionSize);
						for(int j=0; j<actionSize; ++j)
						{
							action[j] = data[i + j];
							//stream << action[j] << ",";
						}
						actions.push_back(action);
						//stream << "],";
					}
					//UDEBUG("%s", stream.str().c_str());
				}

				dataSize = sqlite3_column_bytes(ppStmt, index);
				const void * dataVoid = sqlite3_column_blob(ppStmt, index++);
				if(dataSize>4)
				{
					baseIds = std::vector<int>(dataSize/sizeof(int));
					memcpy(baseIds.data(), dataVoid, dataSize);
				}

				neighbors.insert(neighbors.end(), std::pair<int, NeighborLink>(nid, NeighborLink(nid, actions, baseIds))); // nid
				rc = sqlite3_step(ppStmt);
			}
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			// add neighbors
			(*iter)->addNeighbors(neighbors);

			//reset
			rc = sqlite3_reset(ppStmt);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		return true;
	}
	return false;
}


bool DBDriverSqlite3::updateQuery(const std::list<Signature *> & signatures) const
{
	if(_ppDb && signatures.size())
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		Signature * s = 0;
		std::list<KeypointSignature*> kpSignatures;
		std::list<SMSignature*> smSignatures;

		std::string query = "UPDATE Signature SET weight=?, loopClosureIds=?, childLoopClosureIds=?, timeEnter = DATETIME('NOW') WHERE id=?;";
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			s = *i;
			int index = 1;
			if(s)
			{
				if(s->signatureType().compare("KeypointSignature") == 0)
				{
					kpSignatures.push_back((KeypointSignature *)s);
				}
				else if(s->signatureType().compare("SMSignature") == 0)
				{
					smSignatures.push_back((SMSignature *)s);
				}
				else
				{
					UFATAL("type not implemented !");
				}

				rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
				if (rc != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				std::vector<int> loopIds(s->getLoopClosureIds().begin(), s->getLoopClosureIds().end());
				if(loopIds.size())
				{
					rc = sqlite3_bind_blob(ppStmt, index++, loopIds.data(), loopIds.size()*sizeof(int), SQLITE_STATIC);
				}
				else
				{
					rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
				}
				if (rc != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				std::vector<int> childIds(s->getChildLoopClosureIds().begin(), s->getChildLoopClosureIds().end());
				if(childIds.size())
				{
					rc = sqlite3_bind_blob(ppStmt, index++, childIds.data(), childIds.size()*sizeof(int), SQLITE_STATIC);
				}
				else
				{
					rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
				}
				if (rc != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				rc = sqlite3_bind_int(ppStmt, index++, s->id());
				if (rc != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				//step
				rc=sqlite3_step(ppStmt);
				if (rc != SQLITE_DONE)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				rc = sqlite3_reset(ppStmt);
				if (rc != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1.12: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		ULOGGER_DEBUG("Update Signature table, Time=%fs", timer.ticks());

		// Update neighbors part1
		query = "DELETE FROM Neighbor WHERE sid=?;";
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}
		for(std::list<Signature *>::const_iterator j=signatures.begin(); j!=signatures.end(); ++j)
		{
			if((*j)->isNeighborsModified())
			{
				rc = sqlite3_bind_int(ppStmt, 1, (*j)->id());
				if (rc != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				rc=sqlite3_step(ppStmt);
				if (rc != SQLITE_DONE)
				{
					ULOGGER_ERROR("DB error sid=%d: %s", (*j)->id(), sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}

				rc = sqlite3_reset(ppStmt);
				if (rc != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error, sid=%d: %s", (*j)->id(), sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		// Update neighbors part2
		query = queryStepNeighborLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}
		for(std::list<Signature *>::const_iterator j=signatures.begin(); j!=signatures.end(); ++j)
		{
			if((*j)->isNeighborsModified())
			{
				// Save neighbor references
				const NeighborsMultiMap & neighbors = (*j)->getNeighbors();
				for(NeighborsMultiMap::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
				{
					if(stepNeighborLink(ppStmt, (*j)->id(), i->second) != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Update Neighbors Time=%fs", timer.ticks());

		if(kpSignatures.size())
		{
			// Update word references
			query = queryStepWordsChanged();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			for(std::list<KeypointSignature *>::const_iterator j=kpSignatures.begin(); j!=kpSignatures.end(); ++j)
			{
				if((*j)->getWordsChanged().size())
				{
					const std::map<int, int> & wordsChanged = (*j)->getWordsChanged();
					for(std::map<int, int>::const_iterator iter=wordsChanged.begin(); iter!=wordsChanged.end(); ++iter)
					{
						if(stepWordsChanged(ppStmt, (*j)->id(), iter->first, iter->second) != SQLITE_OK)
						{
							ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
							rc = sqlite3_finalize(ppStmt);
							return false;
						}
					}
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				return false;
			}
		}
		ULOGGER_DEBUG("KpSignatures update=%fs", timer.ticks());

		if(smSignatures.size())
		{
			// Is something to update ???
		}
		ULOGGER_DEBUG("SMSignatures update=%fs", timer.ticks());


		return true;
	}
	return false;
}

// <oldWordId, activeWordId>
// TODO DO IT IN A MORE EFFICiENT WAY !!!! (if it is possible...)
bool DBDriverSqlite3::changeWordsRefQuery(const std::map<int, int> & refsToChange) const
{
	ULOGGER_DEBUG("Changing %d references...", refsToChange.size());
	if(_ppDb && refsToChange.size())
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		const char * query = "UPDATE Map_SS_VW SET visualWordId = ? WHERE visualWordId = ?;";

		rc = sqlite3_prepare_v2(_ppDb, query, -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int count=0;
		for(std::map<int, int>::const_iterator i=refsToChange.begin(); i!=refsToChange.end(); ++i)
		{
			rc = sqlite3_bind_int(ppStmt, 1, (*i).second);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 1.1: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			rc = sqlite3_bind_int(ppStmt, 2, (*i).first);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 1.2: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			//step
			rc=sqlite3_step(ppStmt);
			if (rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("DB error 1.10: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			rc = sqlite3_reset(ppStmt);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 1.11: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			if(++count % 5000 == 0)
			{
				ULOGGER_DEBUG("Changed %d references...", count);
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 5: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::deleteWordsQuery(const std::vector<int> & ids) const
{
	if(_ppDb && ids.size())
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;

		const char * query = "DELETE FROM VisualWord WHERE id = ?;";

		rc = sqlite3_prepare_v2(_ppDb, query, -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		for(unsigned int i=0; i<ids.size(); ++i)
		{
			rc = sqlite3_bind_int(ppStmt, 1, ids[i]);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 1.1: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			//step
			rc=sqlite3_step(ppStmt);
			if (rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("DB error 1.10: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			rc = sqlite3_reset(ppStmt);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 1.11: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 5: %s", sqlite3_errmsg(_ppDb));
			return false;
		}
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::saveQuery(const std::list<Signature *> & signatures) const
{
	UDEBUG("");
	if(_ppDb && signatures.size())
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::list<KeypointSignature*> kpSignatures;
		std::list<SMSignature*> smSignatures;

		// Signature table
		std::string query = queryStepSignature();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		std::list<Signature *> signaturesWithImage;
		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			if(stepSignature(ppStmt, *i) != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			if((*i)->getImage())
			{
				signaturesWithImage.push_back(*i);
			}

			if((*i)->signatureType().compare("KeypointSignature") == 0)
			{
				kpSignatures.push_back((KeypointSignature *)(*i));
			}
			else if((*i)->signatureType().compare("SMSignature") == 0)
			{
				smSignatures.push_back((SMSignature *)(*i));
			}
			else
			{
				UFATAL("type not implemented !");
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		// Create new entries in table Image
		if(signaturesWithImage.size())
		{
			query = queryStepImage();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			for(std::list<Signature *>::iterator i=signaturesWithImage.begin(); i!=signaturesWithImage.end(); ++i)
			{
				if(stepImage(ppStmt, (*i)->id(), (*i)->getImage()) != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				return false;
			}
		}

		// Create new entries in table Neighbor
		query = queryStepNeighborLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}
		for(std::list<Signature *>::const_iterator jter=signatures.begin(); jter!=signatures.end(); ++jter)
		{
			// Save neighbor references
			const NeighborsMultiMap & neighbors = (*jter)->getNeighbors();
			for(NeighborsMultiMap::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				if(stepNeighborLink(ppStmt, (*jter)->id(), iter->second) != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3.5: %s", sqlite3_errmsg(_ppDb));
			return false;
		}

		if(kpSignatures.size())
		{
			// Create new entries in table Map_SS_VW
			query = queryStepKeypoint();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 2: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			for(std::list<KeypointSignature *>::const_iterator i=kpSignatures.begin(); i!=kpSignatures.end(); ++i)
			{
				for(std::multimap<int, cv::KeyPoint>::const_iterator w=(*i)->getWords().begin(); w!=(*i)->getWords().end(); ++w)
				{
					if(stepKeypoint(ppStmt, (*i)->id(), w->first, w->second) != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 2.11: %s", sqlite3_errmsg(_ppDb));
				return false;
			}
		}

		if(smSignatures.size())
		{
			// Create new entries in table SMState
			query = queryStepSensors();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			for(std::list<SMSignature *>::const_iterator i=smSignatures.begin(); i!=smSignatures.end(); ++i)
			{
				if(stepSensors(ppStmt, *i) != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				return false;
			}
		}

		UDEBUG("Time=%fs", timer.ticks());

		return true;
	}
	return false;
}

/* BUG : there is a problem with the code at line indicated below... So it is commented.*/
bool DBDriverSqlite3::saveQuery(const std::vector<VisualWord *> & visualWords) const
{
	ULOGGER_DEBUG("visualWords size=%d", visualWords.size());
	if(_ppDb)
	{
		std::string type;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::string query;

		// Create new entries in table Map_SS_VW
		if(visualWords.size()>0)
		{
			query = std::string("INSERT INTO VisualWord(id, descriptorSize, descriptor) VALUES(?,?,?);");
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			for(std::vector<VisualWord *>::const_iterator iter=visualWords.begin(); iter!=visualWords.end(); ++iter)
			{
				const VisualWord * w = *iter;
				if(w && !w->isSaved())
				{
					rc = sqlite3_bind_int(ppStmt, 1, w->id());
					if (rc != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error 2.1: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}
					rc = sqlite3_bind_int(ppStmt, 2, w->getDim());
					if (rc != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error 2.3: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}
					rc = sqlite3_bind_blob(ppStmt, 3, w->getDescriptor(), w->getDim()*sizeof(float), SQLITE_STATIC);
					if (rc != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error 2.4: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}

					//execute query
					rc=sqlite3_step(ppStmt);
					if (rc != SQLITE_DONE)
					{
						ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}

					rc = sqlite3_reset(ppStmt);
					if (rc != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error 4: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 5: %s", sqlite3_errmsg(_ppDb));
				return false;
			}
		}
		else
		{
			//ULOGGER_DEBUG("DBDriverSqlite3::save(std::list<VisualWord*>) Warning : Empty words ref : size(words)=%d", words.size());
		}


		ULOGGER_DEBUG("Time=%fs", timer.ticks());

		return true;
	}
	return false;
}

std::string DBDriverSqlite3::queryStepSignature() const
{
	return "INSERT INTO Signature(id, type, weight, loopClosureIds, childLoopClosureIds) VALUES(?,?,?,?,?);";
}
int DBDriverSqlite3::stepSignature(sqlite3_stmt * ppStmt, const Signature * s) const
{
	if(!ppStmt || !s)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;

	int index = 1;
	std::string type = s->signatureType();
	rc = sqlite3_bind_int(ppStmt, index++, s->id());
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_text(ppStmt, index++, type.c_str(), -1, SQLITE_STATIC);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	std::vector<int> loopIds(s->getLoopClosureIds().begin(), s->getLoopClosureIds().end());
	if(loopIds.size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, loopIds.data(), loopIds.size()*sizeof(int), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}

	std::vector<int> childIds(s->getChildLoopClosureIds().begin(), s->getChildLoopClosureIds().end());
	if(childIds.size())
	{
		for(unsigned int i=0; i<childIds.size(); ++i)
		{
			UDEBUG("id=%d child=%d", s->id(), childIds[i]);
		}
		rc = sqlite3_bind_blob(ppStmt, index++, childIds.data(), childIds.size()*sizeof(int), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}

	//step
	rc=sqlite3_step(ppStmt);
	if (rc != SQLITE_DONE)
	{
		return rc;
	}

	return sqlite3_reset(ppStmt);
}

std::string DBDriverSqlite3::queryStepImage() const
{
	return "INSERT INTO Image(id, width, height, channels, compressed, data) VALUES(?,?,?,?,?,?);";
}
int DBDriverSqlite3::stepImage(sqlite3_stmt * ppStmt, int id, const IplImage * img) const
{
	if(!ppStmt || !img)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	CvMat * compressed = 0;
	int index = 1;

	rc = sqlite3_bind_int(ppStmt, index++, id);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, img->width);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, img->height);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, img->nChannels);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, this->isImagesCompressed()?1:0);
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	if(this->isImagesCompressed())
	{
		compressed = Signature::compressImage(img);
		if(compressed)
		{
			rc = sqlite3_bind_blob(ppStmt, index++, compressed->data.ptr, compressed->width*sizeof(uchar), SQLITE_STATIC);
		}
		else
		{
			UERROR("Compression failed!");
			rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
		}
	}
	else
	{
		rc = sqlite3_bind_blob(ppStmt, index++, img->imageData, img->imageSize*sizeof(uchar), SQLITE_STATIC);
	}

	if (rc != SQLITE_OK)
	{
		if(compressed)
		{
			cvReleaseMat(&compressed);
			compressed = 0;
		}
		return rc;
	}

	//step
	rc=sqlite3_step(ppStmt);
	if(compressed) // Release it before any return false
	{
		cvReleaseMat(&compressed);
		compressed = 0;
	}
	if (rc != SQLITE_DONE)
	{
		return rc;
	}

	return sqlite3_reset(ppStmt);
}

std::string DBDriverSqlite3::queryStepNeighborLink() const
{
	return "INSERT INTO Neighbor(sid, nid, actionSize, actions, baseIds) VALUES(?,?,?,?,?);";
}
int DBDriverSqlite3::stepNeighborLink(sqlite3_stmt * ppStmt, int signatureId, const NeighborLink & n) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, signatureId);
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, n.id());
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}

	unsigned int actionSize = n.actions().size()>0?n.actions().front().size():0;
	rc = sqlite3_bind_int(ppStmt, index++, actionSize);
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}
	std::vector<float> actions;
	if(actionSize)
	{
		//Concatenate actions
		for(std::list<std::vector<float> >::const_iterator iter=n.actions().begin(); iter!=n.actions().end(); ++iter)
		{
			if(iter->size() != actionSize)
			{
				UERROR("Actuator states must be all the same size! (actionSize=%d != currentAction%d)", actionSize, iter->size());
			}
			actions.insert(actions.end(), iter->begin(), iter->end());
		}
		rc = sqlite3_bind_blob(ppStmt, index++, actions.data(), actions.size()*sizeof(float), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	if (rc != SQLITE_OK)
	{
		UERROR("%s", sqlite3_errmsg(_ppDb));
		return rc;
	}

	if(n.baseIds().size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, n.baseIds().data(), n.baseIds().size()*sizeof(int), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}

	rc=sqlite3_step(ppStmt);
	if (rc != SQLITE_DONE)
	{
		UERROR("");
		return rc;
	}

	return sqlite3_reset(ppStmt);
}

std::string DBDriverSqlite3::queryStepWordsChanged() const
{
	return "UPDATE Map_SS_VW SET visualWordId = ? WHERE visualWordId = ? AND signatureId = ?;";
}
int DBDriverSqlite3::stepWordsChanged(sqlite3_stmt * ppStmt, int signatureId, int oldWordId, int newWordId) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, newWordId);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, oldWordId);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, signatureId);
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	rc=sqlite3_step(ppStmt);
	if (rc != SQLITE_DONE)
	{
		return rc;
	}

	return sqlite3_reset(ppStmt);
}

std::string DBDriverSqlite3::queryStepKeypoint() const
{
	return "INSERT INTO Map_SS_VW(signatureId, visualWordId, pos_x, pos_y, laplacian, size, dir, hessian) VALUES(?,?,?,?,?,?,?,?);";
}
int DBDriverSqlite3::stepKeypoint(sqlite3_stmt * ppStmt, int signatureId, int wordId, const cv::KeyPoint & kp) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, signatureId);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, wordId);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_double(ppStmt, index++, kp.pt.x);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_double(ppStmt, index++, kp.pt.y);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, uSign(kp.response)); // laplacian
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, kp.size);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_double(ppStmt, index++, kp.angle);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_double(ppStmt, index++, kp.response);
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	rc=sqlite3_step(ppStmt);
	if (rc != SQLITE_DONE)
	{
		return rc;
	}

	return sqlite3_reset(ppStmt);
}

std::string DBDriverSqlite3::queryStepSensors() const
{
	return "INSERT INTO SMState(id, sensors, motionMask) VALUES(?,?,?);";
}
int DBDriverSqlite3::stepSensors(sqlite3_stmt * ppStmt, const SMSignature * s) const
{
	if(!ppStmt || !s)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, s->id());
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	const int * sensors = s->getSensors().data();
	if(s->getSensors().size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, sensors, s->getSensors().size()*sizeof(int), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	const unsigned char * mask = s->getMotionMask().data();
	if(s->getMotionMask().size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, mask, s->getMotionMask().size()*sizeof(unsigned char), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	//step
	rc=sqlite3_step(ppStmt);
	if (rc != SQLITE_DONE)
	{
		return rc;
	}

	return sqlite3_reset(ppStmt);
}

} // namespace rtabmap
