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
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/VWDictionary.h"
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
	this->disconnectDatabaseQuery();
	// Open a database connection
	_ppDb = 0;

	if(url.empty())
	{
		UERROR("url is empty...");
		return false;
	}

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
			rc = sqlite3_finalize(pStmt);
			if(rc != SQLITE_OK)
			{
				UERROR("");
			}
		}

		if(_dbInMemory)
		{
			UTimer timer;
			timer.start();
			ULOGGER_DEBUG("Saving DB ...");
			rc = loadOrSaveDb(_ppDb, this->getUrl(), 1); // Save memory to file
			if(rc != SQLITE_OK)
			{
				UERROR("");
			}
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

bool DBDriverSqlite3::getRawDataQuery(int id, std::list<Sensor> & rawData) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		rawData.clear();

		query << "SELECT num, type, raw_width, raw_height, raw_data_type, raw_compressed, raw_data "
			  << "FROM Sensor "
			  << "WHERE id = " << id
			  << " ORDER BY num"
			  <<";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int num;
		int type;
		int width;
		int height;
		int dataType;
		int compressed;
		const void * data = 0;
		int dataSize = 0;
		int index = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			index = 0;
			num = sqlite3_column_int(ppStmt, index++);
			type = sqlite3_column_int(ppStmt, index++);
			width = sqlite3_column_int(ppStmt, index++);
			height = sqlite3_column_int(ppStmt, index++);
			dataType = sqlite3_column_int(ppStmt, index++);
			compressed = sqlite3_column_int(ppStmt, index++);

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);

			//Create the image
			if(dataSize>4 && data)
			{
				if(compressed)
				{
					std::vector<unsigned char> compressed(dataSize);
					memcpy(compressed.data(), data, dataSize);
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
					cv::Mat dataMat = cv::imdecode(compressed, cv::IMREAD_UNCHANGED);
#else
					cv::Mat dataMat = cv::imdecode(compressed, -1);
#endif
					if(dataType != dataMat.type() || width != dataMat.cols || height != dataMat.rows)
					{
						UFATAL("dataType != dataMat.type() || width != dataMat.cols || height != dataMat.rows");
					}
					rawData.push_back(Sensor(dataMat, (Sensor::Type)type, num));
				}
				else
				{
					cv::Mat dataMat(height, width, dataType);
					memcpy(dataMat.data, data, dataSize);
					rawData.push_back(Sensor(dataMat, (Sensor::Type)type, num));
				}
			}
			rc = sqlite3_step(ppStmt); // next result...
		}

		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("Not done ?!? ");
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

bool DBDriverSqlite3::getActuatorDataQuery(int id, std::list<Actuator> & actuators) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		actuators.clear();

		query << "SELECT num, type, width, height, data_type, data "
			  << "FROM Actuator "
			  << "WHERE id = " << id
			  << " ORDER BY num"
			  <<";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int num;
		int type;
		int width;
		int height;
		int dataType;
		const void * data = 0;
		int dataSize = 0;
		int index = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			index = 0;
			num = sqlite3_column_int(ppStmt, index++);
			type = sqlite3_column_int(ppStmt, index++);
			width = sqlite3_column_int(ppStmt, index++);
			height = sqlite3_column_int(ppStmt, index++);
			dataType = sqlite3_column_int(ppStmt, index++);

			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);

			if(dataSize>4 && data)
			{
				cv::Mat dataMat(height, width, dataType);
				memcpy(dataMat.data, data, dataSize);
				actuators.push_back(Actuator(dataMat, (Actuator::Type)type, num));
			}
			rc = sqlite3_step(ppStmt); // next result...
		}

		if(rc != SQLITE_DONE)
		{
			ULOGGER_ERROR("Not done ?!? ");
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

bool DBDriverSqlite3::getAllNodeIdsQuery(std::set<int> & ids) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT id "
			  << "FROM Node "
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

bool DBDriverSqlite3::getLastNodeIdQuery(int & id) const
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
			  << "FROM Node;";

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

bool DBDriverSqlite3::getLastWordIdQuery(int & id) const
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
			  << "FROM Word;";

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

bool DBDriverSqlite3::getInvertedIndexNiQuery(int nodeId, int & ni) const
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
		query << "SELECT count(word_id) "
			  << "FROM Map_Node_Word "
			  << "WHERE node_id=" << nodeId << ";";

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
				ULOGGER_ERROR("Supposed to received only 1 result... node=%d", nodeId);
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB, node=%d",nodeId);
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
bool DBDriverSqlite3::getNeighborIdsQuery(int nodeId, std::set<int> & neighbors, bool onlyWithActions) const
{
	neighbors.clear();
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT DISTINCT to_id FROM Link "
			  << "WHERE from_id = " << nodeId
			  << " AND type = 0";

		if(onlyWithActions)
		{
			query << " AND actuator_id > 0";
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
			neighbors.insert(sqlite3_column_int(ppStmt, 0)); // nid
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
			UERROR("No neighbors loaded from node %d", nodeId);
		}
		return true;
	}
	return false;
}

bool DBDriverSqlite3::getWeightQuery(int nodeId, int & weight) const
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

bool DBDriverSqlite3::getLoopClosureIdsQuery(int nodeId, std::set<int> & loopIds, std::set<int> & childIds) const
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

		query << "SELECT to_id, type FROM Link WHERE from_id =  "
			  << nodeId
			  << " AND type > 0"
			  << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int toId = 0;
		int type;
		UDEBUG("time =%fs", timer.ticks());
		// Process the result if one
		rc = sqlite3_step(ppStmt);
		UDEBUG("time =%fs", timer.ticks());
		while(rc == SQLITE_ROW)
		{
			int index = 0;
			toId = sqlite3_column_int(ppStmt, index++);
			type = sqlite3_column_int(ppStmt, index++);

			if(nodeId == toId)
			{
				UERROR("Loop links cannot be auto-reference links (node=%d)", toId);
			}
			else if(type == 1)
			{
				UDEBUG("Load link from %d to %d, type=%d", nodeId, toId, 1);
				//loop id
				loopIds.insert(toId);
			}
			else if(type == 2)
			{
				UDEBUG("Load link from %d to %d, type=%d", nodeId, toId, 2);
				//loop id
				childIds.insert(toId);
			}
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

		UDEBUG("time =%fs", timer.getElapsedTime());

		return true;
	}
	return false;
}

//return <weight,id>
bool DBDriverSqlite3::getHighestWeightedNodeIdsQuery(unsigned int count, std::multimap<int,int> & ids) const
{
	if(_ppDb && count)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT weight,id "
			  << "FROM Node "
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

bool DBDriverSqlite3::loadQuery(int nodeId, Signature ** s) const
{
	ULOGGER_DEBUG("Signature");
	*s = 0;
	if(_ppDb)
	{
		int type = -1;
		int weight = 0;
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT type, weight "
			  << "FROM Node "
			  << "WHERE id=" << nodeId << ";";

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
			type = sqlite3_column_int(ppStmt, index++); // Node type
			weight = sqlite3_column_int(ppStmt, index++); // weight
			rc = sqlite3_step(ppStmt);
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Supposed to receive only 1 result... node=%d", nodeId);
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB, node=%d",nodeId);
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
		this->loadNeighborsQuery(nodeId, neighbors);

		//get loop / child links
		std::set<int> loopIds;
		std::set<int> childIds;
		this->getLoopClosureIds(nodeId, loopIds, childIds);

		if(type == 0)
		{
			*s = new KeypointSignature(nodeId);
			if(*s)
			{
				(*s)->setWeight(weight);
				(*s)->setLoopClosureIds(loopIds);
				(*s)->setChildLoopClosureIds(childIds);
				(*s)->addNeighbors(neighbors);
				if(this->loadQuery(nodeId, (KeypointSignature*)*s))
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
		else if(type == 1)
		{
			*s = new SMSignature(nodeId);
			if(*s)
			{
				(*s)->setWeight(weight);
				(*s)->setLoopClosureIds(loopIds);
				(*s)->setChildLoopClosureIds(childIds);
				(*s)->addNeighbors(neighbors);
				if(this->loadQuery(nodeId, (SMSignature*)*s))
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

bool DBDriverSqlite3::loadQuery(int nodeId, KeypointSignature * ks) const
{
	UDEBUG("KeypointSignature %d", nodeId);
	if(_ppDb && ks)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::multimap<int, cv::KeyPoint> visualWords;

		// Get the map from signature and visual words
		query << "SELECT word_id, pos_x, pos_y, size, dir, response "
			  << "FROM Map_Node_Word "
			  << "WHERE node_id=" << nodeId
			  << " ORDER BY word_id;"; // used for fast insertion below

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
		int size = 0;
		float dir = 0;
		float response = 0;
		while(rc == SQLITE_ROW)
		{
			int index=0;
			visualWordId = sqlite3_column_int(ppStmt, index++);
			pos_x = sqlite3_column_double(ppStmt, index++);
			pos_y = sqlite3_column_double(ppStmt, index++);
			size = sqlite3_column_int(ppStmt, index++);
			dir = sqlite3_column_double(ppStmt, index++);
			response = sqlite3_column_double(ppStmt, index++);
			visualWords.insert(visualWords.end(), std::pair<int, cv::KeyPoint>(visualWordId, cv::KeyPoint(cv::Point2f(pos_x,pos_y),size,dir,response, 0, -1)));
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

		ks->setWords(visualWords);

		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

bool DBDriverSqlite3::loadQuery(int nodeId, SMSignature * sm) const
{
	ULOGGER_DEBUG("SMSignature");
	if(_ppDb && sm)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;
		std::list<std::vector<int> > sensors;
		std::vector<int> sensor;

		// Get the map from signature
		query << "SELECT data "
			  << "FROM Sensor "
			  << "WHERE id=" << nodeId;

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
		while(rc == SQLITE_ROW)
		{
			int index = 0;
			data = sqlite3_column_blob(ppStmt, index);
			dataSize = sqlite3_column_bytes(ppStmt, index++);
			//Create the sensor
			if(dataSize>4 && data)
			{
				sensor = std::vector<int>(dataSize/sizeof(int));
				memcpy(&(sensor[0]), data, dataSize);
				sensors.push_back(sensor);
			}

			rc = sqlite3_step(ppStmt);
		}

		if(rc != SQLITE_DONE)
		{
			UERROR("DB error: %s", sqlite3_errmsg(_ppDb));
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

		sm->setSensors(sensors);

		ULOGGER_DEBUG("Time=%fs", timer.ticks());
		return true;
	}
	return false;
}

//may be slower than the previous version but don't have a limit of words that can be loaded at the same time
bool DBDriverSqlite3::loadKeypointSignaturesQuery(const std::list<int> & ids, std::list<Signature *> & nodes) const
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
		std::multimap<int, cv::KeyPoint> visualWords;
		unsigned int loaded = 0;

		// Prepare the query... Get the map from signature and visual words
		query << "SELECT id, weight, word_id, pos_x, pos_y, size, dir, response "
				 "FROM Map_Node_Word "
				 "INNER JOIN Node "
				 "ON Node.id = node_id "
				 "WHERE Node.type = 0 AND node_id = ? ";

		query << " ORDER BY word_id"; // Needed for fast insertion below
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
		int visualWordId = 0;
		float pos_x = 0;
		float pos_y = 0;
		int size = 0;
		float dir = 0;
		float response = 0;

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
			visualWordId = 0;
			pos_x = 0;
			pos_y = 0;
			size = 0;
			dir = 0;
			response = 0;
			visualWords.clear();

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;
				if(id==0)
				{
					id = sqlite3_column_int(ppStmt, index++); // Signature Id
					weight = sqlite3_column_int(ppStmt, index++); // weight
				}
				else
				{
					index = 2;
				}
				visualWordId = sqlite3_column_int(ppStmt, index++);
				pos_x = sqlite3_column_double(ppStmt, index++);
				pos_y = sqlite3_column_double(ppStmt, index++);
				size = sqlite3_column_int(ppStmt, index++);
				dir = sqlite3_column_double(ppStmt, index++);
				response = sqlite3_column_double(ppStmt, index++);
				visualWords.insert(visualWords.end(), std::pair<int, cv::KeyPoint>(visualWordId, cv::KeyPoint(cv::Point2f(pos_x,pos_y),size,dir,response, 0, -1)));
				rc = sqlite3_step(ppStmt);
			}
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			// create the node
			if(id)
			{
				ULOGGER_DEBUG("Creating %d with %d keypoints", *iter, visualWords.size());
				KeypointSignature * ss = new KeypointSignature(visualWords, id);
				if(ss)
				{
					ss->setWeight(weight);
					ss->setSaved(true);
					nodes.push_back(ss);
				}
				else
				{
					UFATAL("?Cannot allocate memory !!!");
				}
				++loaded;
			}
			else
			{
				ULOGGER_ERROR("Node %d not found in database", *iter);
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

		this->loadLinksQuery(nodes);
		for(std::list<Signature*>::iterator iter = nodes.begin(); iter!=nodes.end(); ++iter)
		{
			(*iter)->setModified(false);
		}
		ULOGGER_DEBUG("Time load links=%fs", timer.ticks());

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
		query << "SELECT Node.id, weight, data "
			  << "FROM Sensor "
			  << "INNER JOIN Node "
			  << "ON Node.id = Sensor.id "
			  << "WHERE Node.type = 1 AND Node.id=? ";
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

			int id = 0;
			int weight = 0;
			std::list<std::vector<int> > postData;
			const void * data = 0;
			int dataSize = 0;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;

				if(id == 0)
				{
					id = sqlite3_column_int(ppStmt, index++);
					weight = sqlite3_column_int(ppStmt, index++);
				}
				else
				{
					index = 2;
				}

				//post processed data
				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);
				//Create the data
				if(dataSize>4 && data)
				{
					std::vector<int> sensors = std::vector<int>(dataSize/sizeof(int));
					memcpy(&(sensors[0]), data, dataSize);
					postData.push_back(sensors);
				}

				rc = sqlite3_step(ppStmt);
			}

			if(id)
			{
				ULOGGER_DEBUG("Creating %d", id);
				SMSignature * sm = new SMSignature(postData, id);
				if(sm)
				{
					sm->setWeight(weight);
					sm->setSaved(true);
					signatures.push_back(sm);
				}
				else
				{
					UFATAL("Cannot allocate memory !!!");
				}
				++loaded;
			}

			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("Db error: %s", sqlite3_errmsg(_ppDb));
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

		UINFO("Time=%fs", timer.ticks());

		this->loadLinksQuery(signatures);
		for(std::list<Signature*>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
		{
			(*iter)->setModified(false);
		}
		UINFO("Time load neighbors=%fs", timer.ticks());

		if(ids.size() != loaded)
		{
			UERROR("Some signatures not found in database");
		}

		return true;
	}
	return false;
}


bool DBDriverSqlite3::loadLastNodesQuery(std::list<Signature *> & nodes) const
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
		query << "SELECT n.id "
				 "FROM Node AS n "
				 "WHERE n.time_enter >= (SELECT MAX(time_enter) FROM Statistics);"
				 /*"AND s.parentId IS NULL;"*/;

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);

		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
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
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
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
				nodes.push_back(s);
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
		query << "SELECT vw.id, vw.descriptor_size, vw.descriptor, m.node_id "
				 "FROM Word as vw "
				 "INNER JOIN Map_Node_Word as m "
				 "ON vw.id=m.word_id "
				 "INNER JOIN Node as n "
				 "ON n.id=m.node_id "
				 "WHERE n.time_enter >= (SELECT MAX(time_enter) FROM Statistics) "
				 "ORDER BY vw.id;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		// Process the result if one
		int id = 0;
		int lastId = 0;
		int descriptorSize = 0;
		const void * descriptor = 0;
		int dRealSize = 0;
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
				if(dRealSize/int(sizeof(float)) != descriptorSize)
				{
					UERROR("Saved buffer size (%d) is not the same as descriptor size (%d)", dRealSize/sizeof(float), descriptorSize);
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

		getLastWordId(id);
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
		query << "SELECT vw.id, vw.descriptor_size, vw.descriptor "
				 "FROM Word as vw "
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

				if(dRealSize/int(sizeof(float)) != descriptorSize)
				{
					UERROR("Saved buffer size (%d) is not the same as descriptor size (%d)", dRealSize/sizeof(float), descriptorSize);
				}

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
		query << "SELECT vw.id, vw.descriptor_size, vw.descriptor "
				 "FROM Word AS vw "
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

			if(dRealSize/int(sizeof(float)) != descriptorSize)
			{
				UERROR("Saved buffer size (%d) is not the same as descriptor size (%d)", dRealSize/sizeof(float), descriptorSize);
			}

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

		query << "SELECT to_id, actuator_id, num, base_ids, Actuator.type, width, height, data_type, data FROM Link "
		      << "LEFT JOIN Actuator "
		      << "ON Actuator.id = actuator_id "
		      << "WHERE from_id = " << signatureId
			  << " AND Link.type = 0"
			  << " ORDER BY to_id, actuator_id, num";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		int toId = -1;
		int actuatorId = -1;
		int actuatorNum = -1;
		int actuatorType;
		int actuatorWidth;
		int actuatorHeight;
		int actuatorDataType;
		const void * data = 0;
		int dataSize = 0;
		std::list<Actuator> actuators;
		std::vector<int> baseIds;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			int index = 0;

			int tmpToId = sqlite3_column_int(ppStmt, index++);
			int tmpActuatorId = sqlite3_column_int(ppStmt, index++);
			int tmpActuatorNum = sqlite3_column_int(ppStmt, index++);

			if((tmpToId != toId && toId >= 0) ||
				(tmpActuatorId != actuatorId && actuatorId >= 0) ||
				(tmpActuatorNum <= actuatorNum && actuatorNum >= 0))
			{
				neighbors.insert(neighbors.end(), std::pair<int, NeighborLink>(toId, NeighborLink(toId, baseIds, actuators)));
				actuators.clear();
			}
			toId = tmpToId;
			actuatorId = tmpActuatorId;
			actuatorNum = tmpActuatorNum;

			// baseIds
			dataSize = sqlite3_column_bytes(ppStmt, index);
			data = sqlite3_column_blob(ppStmt, index++);
			if(dataSize>4)
			{
				baseIds = std::vector<int>(dataSize/sizeof(int));
				memcpy(baseIds.data(), data, dataSize);
			}

			if(actuatorId)
			{
				actuatorType = sqlite3_column_int(ppStmt, index++);
				actuatorWidth = sqlite3_column_int(ppStmt, index++);
				actuatorHeight = sqlite3_column_int(ppStmt, index++);
				actuatorDataType = sqlite3_column_int(ppStmt, index++);

				data = sqlite3_column_blob(ppStmt, index);
				dataSize = sqlite3_column_bytes(ppStmt, index++);

				//Create the actuator
				if(dataSize>4 && data)
				{
					cv::Mat dataMat(actuatorHeight, actuatorWidth, actuatorDataType);
					memcpy(dataMat.data, data, dataSize);
					actuators.push_back(Actuator(dataMat, (Actuator::Type)actuatorType, actuatorNum));
				}
			}
			rc = sqlite3_step(ppStmt);
		}
		// add the last
		if(toId > 0)
		{
			neighbors.insert(neighbors.end(), std::pair<int, NeighborLink>(toId, NeighborLink(toId, baseIds, actuators)));
			actuators.clear();
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

bool DBDriverSqlite3::loadLinksQuery(std::list<Signature *> & signatures) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT to_id, Link.type, actuator_id, num, base_ids, Actuator.type, width, height, data_type, data FROM Link "
			  << "LEFT JOIN Actuator "
			  << "ON Actuator.id = actuator_id "
			  << "WHERE from_id = ? "
			  << "ORDER BY to_id, actuator_id, num;";

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

			int toId = -1;
			int linkType = -1;
			int actuatorId = -1;
			int actuatorNum = -1;
			int actuatorType;
			int actuatorWidth;
			int actuatorHeight;
			int actuatorDataType;
			const void * data = 0;
			int dataSize = 0;
			std::list<Actuator> actuators;
			std::vector<int> baseIds;
			NeighborsMultiMap neighbors;
			std::set<int> loopIds;
			std::set<int> childIds;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;

				int tmpToId = sqlite3_column_int(ppStmt, index++);
				int tmpLinkType = sqlite3_column_int(ppStmt, index++);
				int tmpActuatorId = sqlite3_column_int(ppStmt, index++);
				int tmpActuatorNum = sqlite3_column_int(ppStmt, index++);

				if((tmpToId != toId && toId >= 0) ||
					(tmpLinkType != linkType && linkType >= 0) ||
					(tmpActuatorId != actuatorId && actuatorId >= 0) ||
					(tmpActuatorNum <= actuatorNum && actuatorNum >= 0))
				{
					if(linkType == 1)
					{
						UDEBUG("Load link from %d to %d, type=%d", (*iter)->id(), toId, 1);
						loopIds.insert(toId);
					}
					else if(linkType == 2)
					{
						UDEBUG("Load link from %d to %d, type=%d", (*iter)->id(), toId, 2);
						childIds.insert(toId);
					}
					else if(linkType == 0)
					{
						UDEBUG("Load link from %d to %d, type=%d", (*iter)->id(), toId, 0);
						neighbors.insert(neighbors.end(), std::pair<int, NeighborLink>(toId, NeighborLink(toId, baseIds, actuators)));
						actuators.clear();
					}
				}
				toId = tmpToId;
				linkType = tmpLinkType;
				actuatorId = tmpActuatorId;
				actuatorNum = tmpActuatorNum;

				// baseIds
				dataSize = sqlite3_column_bytes(ppStmt, index);
				data = sqlite3_column_blob(ppStmt, index++);
				if(dataSize>4)
				{
					baseIds = std::vector<int>(dataSize/sizeof(int));
					memcpy(baseIds.data(), data, dataSize);
				}

				if(actuatorId)
				{
					actuatorType = sqlite3_column_int(ppStmt, index++);
					actuatorWidth = sqlite3_column_int(ppStmt, index++);
					actuatorHeight = sqlite3_column_int(ppStmt, index++);
					actuatorDataType = sqlite3_column_int(ppStmt, index++);

					data = sqlite3_column_blob(ppStmt, index);
					dataSize = sqlite3_column_bytes(ppStmt, index++);

					//Create the actuator
					if(dataSize>4 && data)
					{
						cv::Mat dataMat(actuatorHeight, actuatorWidth, actuatorDataType);
						memcpy(dataMat.data, data, dataSize);
						actuators.push_back(Actuator(dataMat, (Actuator::Type)actuatorType, actuatorNum));
					}
				}
				rc = sqlite3_step(ppStmt);
			}
			// add the last
			if(linkType == 1)
			{
				loopIds.insert(toId);
			}
			else if(linkType == 2)
			{
				childIds.insert(toId);
			}
			else if(linkType == 0)
			{
				neighbors.insert(neighbors.end(), std::pair<int, NeighborLink>(toId, NeighborLink(toId, baseIds, actuators)));
				actuators.clear();
			}
			else if(linkType != -1)
			{
				UERROR("Not handled link type %d", linkType);
			}
			if(rc != SQLITE_DONE)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}

			// add links
			(*iter)->addNeighbors(neighbors);
			(*iter)->setLoopClosureIds(loopIds);
			(*iter)->setChildLoopClosureIds(childIds);

			//reset
			rc = sqlite3_reset(ppStmt);
			if (rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			UINFO("time=%fs, neighbors.size=%d, loopIds=%d, childIds=%d", timer.ticks(), neighbors.size(), loopIds.size(), childIds.size());
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


bool DBDriverSqlite3::updateQuery(const std::list<Signature *> & nodes) const
{
	if(_ppDb && nodes.size())
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		Signature * s = 0;
		std::list<KeypointSignature*> kpSignatures;
		std::list<SMSignature*> smSignatures;

		std::string query = "UPDATE Node SET weight=?, time_enter = DATETIME('NOW') WHERE id=?;";
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		for(std::list<Signature *>::const_iterator i=nodes.begin(); i!=nodes.end(); ++i)
		{
			s = *i;
			int index = 1;
			if(s)
			{
				if(s->nodeType().compare("KeypointSignature") == 0)
				{
					kpSignatures.push_back((KeypointSignature *)s);
				}
				else if(s->nodeType().compare("SMSignature") == 0)
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

		ULOGGER_DEBUG("Update Node table, Time=%fs", timer.ticks());

		// Update links part1
		query = "DELETE FROM Link WHERE from_id=?;";
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}
		for(std::list<Signature *>::const_iterator j=nodes.begin(); j!=nodes.end(); ++j)
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

		// Update links part2
		query = queryStepLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}
		for(std::list<Signature *>::const_iterator j=nodes.begin(); j!=nodes.end(); ++j)
		{
			if((*j)->isNeighborsModified())
			{
				// Save neighbor links
				const NeighborsMultiMap & neighbors = (*j)->getNeighbors();
				for(NeighborsMultiMap::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
				{
					if(stepLink(ppStmt, (*j)->id(), i->first, 0, i->second.actuatorId(), i->second.baseIds()) != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}
				}
				// save loop closure links
				const std::set<int> & loopIds = (*j)->getLoopClosureIds();
				for(std::set<int>::const_iterator i=loopIds.begin(); i!=loopIds.end(); ++i)
				{
					if(stepLink(ppStmt, (*j)->id(), *i, 1, 0, std::vector<int>()) != SQLITE_OK)
					{
						ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
						rc = sqlite3_finalize(ppStmt);
						return false;
					}
				}
				const std::set<int> & childIds = (*j)->getChildLoopClosureIds();
				for(std::set<int>::const_iterator i=childIds.begin(); i!=childIds.end(); ++i)
				{
					if(stepLink(ppStmt, (*j)->id(), *i, 2, 0, std::vector<int>()) != SQLITE_OK)
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

		const char * query = "UPDATE Map_Node_Word SET word_id = ? WHERE word_id = ?;";

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

		const char * query = "DELETE FROM Word WHERE id = ?;";

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
		std::string query = queryStepNode();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}

		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			if(stepNode(ppStmt, *i) != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			if((*i)->nodeType().compare("KeypointSignature") == 0)
			{
				kpSignatures.push_back((KeypointSignature *)(*i));
			}
			else if((*i)->nodeType().compare("SMSignature") == 0)
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

		// Create new entries in table Link
		query = queryStepLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}
		for(std::list<Signature *>::const_iterator jter=signatures.begin(); jter!=signatures.end(); ++jter)
		{
			// Save neighbor links
			const NeighborsMultiMap & neighbors = (*jter)->getNeighbors();
			for(NeighborsMultiMap::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
			{
				if(stepLink(ppStmt, (*jter)->id(), i->first, 0, i->second.actuatorId(), i->second.baseIds()) != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}
			}
			// save loop closure links
			const std::set<int> & loopIds = (*jter)->getLoopClosureIds();
			for(std::set<int>::const_iterator i=loopIds.begin(); i!=loopIds.end(); ++i)
			{
				if(stepLink(ppStmt, (*jter)->id(), *i, 1, 0, std::vector<int>()) != SQLITE_OK)
				{
					ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
					rc = sqlite3_finalize(ppStmt);
					return false;
				}
			}
			const std::set<int> & childIds = (*jter)->getChildLoopClosureIds();
			for(std::set<int>::const_iterator i=childIds.begin(); i!=childIds.end(); ++i)
			{
				if(stepLink(ppStmt, (*jter)->id(), *i, 2, 0, std::vector<int>()) != SQLITE_OK)
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

		// Create new entries in table Actuator
		query = queryStepActuator();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		if(rc != SQLITE_OK)
		{
			ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
			rc = sqlite3_finalize(ppStmt);
			return false;
		}
		for(std::list<Signature *>::const_iterator jter=signatures.begin(); jter!=signatures.end(); ++jter)
		{
			// Save actuators
			const NeighborsMultiMap & neighbors = (*jter)->getNeighbors();
			for(NeighborsMultiMap::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				//only save the next actions
				if(iter->second.actuatorId() == (*jter)->id() + 1)
				{
					const std::list<Actuator> & actuators = iter->second.actuators();
					int k=0;
					for(std::list<Actuator>::const_iterator kter=actuators.begin(); kter!=actuators.end(); ++kter)
					{
						if(stepActuator(ppStmt, iter->second.actuatorId(), k++, *kter) != SQLITE_OK)
						{
							UERROR("DB error: %s", sqlite3_errmsg(_ppDb));
							rc = sqlite3_finalize(ppStmt);
							return false;
						}
					}
					break;
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
			// Create new entries in table Map_Word_Node
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

			// Add raw sensors
			query = queryStepSensor();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			for(std::list<KeypointSignature *>::const_iterator i=kpSignatures.begin(); i!=kpSignatures.end(); ++i)
			{
				int k=0;
				for(std::list<Sensor>::const_iterator rawSensor=(*i)->getRawData().begin(); rawSensor!=(*i)->getRawData().end(); ++rawSensor)
				{
					if(stepSensor(ppStmt, (*i)->id(), k++, std::vector<int>(), *rawSensor) != SQLITE_OK)
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
			// Add sensors
			query = queryStepSensor();
			rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			for(std::list<SMSignature *>::const_iterator i=smSignatures.begin(); i!=smSignatures.end(); ++i)
			{
				int k=0;
				std::list<std::vector<int> >::const_iterator postIter=(*i)->getData().begin();
				std::list<Sensor>::const_iterator rawSensor=(*i)->getRawData().begin();
				for(; postIter!=(*i)->getData().end(); ++postIter)
				{
					if(rawSensor != (*i)->getRawData().end())
					{
						if(stepSensor(ppStmt, (*i)->id(), k++, *postIter, *rawSensor) != SQLITE_OK)
						{
							ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
							rc = sqlite3_finalize(ppStmt);
							return false;
						}
						++rawSensor;
					}
					else
					{
						cv::Mat empty;
						if(stepSensor(ppStmt, (*i)->id(), k++, *postIter, Sensor(empty, Sensor::kTypeNotSpecified)) != SQLITE_OK)
						{
							ULOGGER_ERROR("DB error 3: %s", sqlite3_errmsg(_ppDb));
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
				ULOGGER_ERROR("DB error 2.11: %s", sqlite3_errmsg(_ppDb));
				return false;
			}
		}

		UDEBUG("Time=%fs", timer.ticks());

		return true;
	}
	return false;
}

/* BUG : there is a problem with the code at line indicated below... So it is commented.*/
bool DBDriverSqlite3::saveQuery(const std::vector<VisualWord *> & words) const
{
	ULOGGER_DEBUG("visualWords size=%d", words.size());
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
			if(rc != SQLITE_OK)
			{
				ULOGGER_ERROR("DB error 1: %s", sqlite3_errmsg(_ppDb));
				rc = sqlite3_finalize(ppStmt);
				return false;
			}
			for(std::vector<VisualWord *>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
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

std::string DBDriverSqlite3::queryStepNode() const
{
	return "INSERT INTO Node(id, type, weight) VALUES(?,?,?);";
}
int DBDriverSqlite3::stepNode(sqlite3_stmt * ppStmt, const Signature * s) const
{
	if(!ppStmt || !s)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;

	int index = 1;
	std::string type = s->nodeType();
	rc = sqlite3_bind_int(ppStmt, index++, s->id());
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, type.compare("KeypointSignature")==0?0:1);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
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

std::string DBDriverSqlite3::queryStepSensor() const
{
	return "INSERT INTO Sensor(id, num, type, data, raw_width, raw_height, raw_data_type, raw_compressed, raw_data) VALUES(?,?,?,?,?,?,?,?,?);";
}
int DBDriverSqlite3::stepSensor(sqlite3_stmt * ppStmt, int id, int num, const std::vector<int> & postProcessedData, const Sensor & sensor) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	std::vector<unsigned char> compressed;
	int index = 1;

	rc = sqlite3_bind_int(ppStmt, index++, id);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, num);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, sensor.type());
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	if(postProcessedData.size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, postProcessedData.data(), postProcessedData.size()*sizeof(int), SQLITE_STATIC);
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	rc = sqlite3_bind_int(ppStmt, index++, sensor.data().cols);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, sensor.data().rows);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, sensor.data().type());
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, this->isImagesCompressed() && sensor.type()==Sensor::kTypeImage?1:0);
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	if(sensor.data().total())
	{
		if(this->isImagesCompressed() && sensor.type()==Sensor::kTypeImage)
		{
			cv::imencode(".png", sensor.data(), compressed);
			if(compressed.size())
			{
				rc = sqlite3_bind_blob(ppStmt, index++, compressed.data(), compressed.size()*sizeof(uchar), SQLITE_STATIC);
			}
			else
			{
				UERROR("Compression failed!");
				rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
			}
		}
		else
		{
			if(!sensor.data().isContinuous())
			{
				UWARN("matrix is not continuous");
			}
			rc = sqlite3_bind_blob(ppStmt, index++, sensor.data().data, sensor.data().total()*sensor.data().elemSize(), SQLITE_STATIC);
		}
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

std::string DBDriverSqlite3::queryStepLink() const
{
	return "INSERT INTO Link(from_id, to_id, type, actuator_id, base_ids) VALUES(?,?,?,?,?);";
}
int DBDriverSqlite3::stepLink(sqlite3_stmt * ppStmt, int fromId, int toId, int type, int actuatorId, const std::vector<int> & baseIds) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	UDEBUG("Save link from %d to %d, type=%d, a_id=%d", fromId, toId, type, actuatorId);
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, fromId);
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, toId);
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, type);
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, actuatorId);
	if (rc != SQLITE_OK)
	{
		UERROR("");
		return rc;
	}

	if(baseIds.size())
	{
		rc = sqlite3_bind_blob(ppStmt, index++, baseIds.data(), baseIds.size()*sizeof(int), SQLITE_STATIC);
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

std::string DBDriverSqlite3::queryStepActuator() const
{
	return "INSERT INTO Actuator(id, num, type, width, height, data_type, data) VALUES(?,?,?,?,?,?,?);";
}
int DBDriverSqlite3::stepActuator(sqlite3_stmt * ppStmt, int id, int num, const Actuator & actuator) const
{
	if(!ppStmt || actuator.data().total() == 0)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	std::vector<unsigned char> compressed;
	int index = 1;

	rc = sqlite3_bind_int(ppStmt, index++, id);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, num);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, actuator.type());
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, actuator.data().cols);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, actuator.data().rows);
	if (rc != SQLITE_OK)
	{
		return rc;
	}
	rc = sqlite3_bind_int(ppStmt, index++, actuator.data().type());
	if (rc != SQLITE_OK)
	{
		return rc;
	}

	if(!actuator.data().isContinuous())
	{
		UWARN("matrix is not continuous");
	}
	rc = sqlite3_bind_blob(ppStmt, index++, actuator.data().data, actuator.data().total()*actuator.data().elemSize(), SQLITE_STATIC);

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

std::string DBDriverSqlite3::queryStepWordsChanged() const
{
	return "UPDATE Map_Node_Word SET word_id = ? WHERE word_id = ? AND node_id = ?;";
}
int DBDriverSqlite3::stepWordsChanged(sqlite3_stmt * ppStmt, int nodeId, int oldWordId, int newWordId) const
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
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
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
	return "INSERT INTO Map_Node_Word(node_id, word_id, pos_x, pos_y, size, dir, response) VALUES(?,?,?,?,?,?,?);";
}
int DBDriverSqlite3::stepKeypoint(sqlite3_stmt * ppStmt, int nodeId, int wordId, const cv::KeyPoint & kp) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
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

} // namespace rtabmap
