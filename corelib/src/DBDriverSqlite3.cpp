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

#include "Signature.h"
#include "VisualWord.h"
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
		this->executeNoResultQuery(schema.c_str());
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

void DBDriverSqlite3::getImageQuery(int nodeId, cv::Mat & image) const
{
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT raw_width, raw_height, raw_data_type, raw_compressed, raw_data "
			  << "FROM Image "
			  << "WHERE id = " << nodeId
			  <<";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		int width;
		int height;
		int dataType;
		int compressed;
		const void * data = 0;
		int dataSize = 0;
		int index = 0;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			index = 0;
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
					image = cv::imdecode(compressed, cv::IMREAD_UNCHANGED);
#else
					image = cv::imdecode(compressed, -1);
#endif
					if(dataType != image.type() || width != image.cols || height != image.rows)
					{
						UFATAL("dataType != dataMat.type() || width != dataMat.cols || height != dataMat.rows");
					}
				}
				else
				{
					image = cv::Mat(height, width, dataType);
					memcpy(image.data, data, dataSize);
				}
			}
			rc = sqlite3_step(ppStmt); // next result...
		}

		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::getAllNodeIdsQuery(std::set<int> & ids) const
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
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			ids.insert(ids.end(), sqlite3_column_int(ppStmt, 0)); // Signature Id
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%f", timer.ticks());
	}
}

void DBDriverSqlite3::getLastIdQuery(const std::string & tableName, int & id) const
{
	if(_ppDb)
	{
		UDEBUG("get last id from table \"%s\"", tableName.c_str());
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT max(id) "
			  << "FROM " << tableName
			  << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			id = sqlite3_column_int(ppStmt, 0); // Signature Id
			rc = sqlite3_step(ppStmt);
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB");
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
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

		// Create a new entry in table Signature
		query << "SELECT count(word_id) "
			  << "FROM Map_Node_Word "
			  << "WHERE node_id=" << nodeId << ";";

		//query.append("COMMIT;");

		//ULOGGER_DEBUG("DBDriverSqlite3::getSurfNi() Execute query : %s", query.toStdString().c_str());

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			ni = sqlite3_column_int(ppStmt, 0);
			rc = sqlite3_step(ppStmt);
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		}
		else
		{
			ULOGGER_ERROR("No result !?! from the DB, node=%d",nodeId);
		}


		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Time=%fs", timer.ticks());
	}
}

// default onlyWithActions = false
void DBDriverSqlite3::getNeighborIdsQuery(int nodeId, std::set<int> & neighbors, bool onlyWithActions) const
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
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			neighbors.insert(sqlite3_column_int(ppStmt, 0)); // nid
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		if(neighbors.size() == 0)
		{
			UERROR("No neighbors loaded from node %d", nodeId);
		}
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
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());


		// Process the result if one
		rc = sqlite3_step(ppStmt);
		if(rc == SQLITE_ROW)
		{
			weight= sqlite3_column_int(ppStmt, 0); // weight
			rc = sqlite3_step(ppStmt);
		}
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	}
}

void DBDriverSqlite3::getLoopClosureIdsQuery(int nodeId, std::set<int> & loopIds, std::set<int> & childIds) const
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
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		int toId = 0;
		int type;
		// Process the result if one
		rc = sqlite3_step(ppStmt);
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
		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("time =%fs", timer.getElapsedTime());
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
		std::multimap<int, cv::KeyPoint> visualWords;
		unsigned int loaded = 0;

		// Prepare the query... Get the map from signature and visual words
		query << "SELECT id, weight, word_id, pos_x, pos_y, size, dir, response "
				 "FROM Map_Node_Word "
				 "INNER JOIN Node "
				 "ON Node.id = node_id "
				 "WHERE node_id = ? ";

		query << " ORDER BY word_id"; // Needed for fast insertion below
		query << ";";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

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
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

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
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

			// create the node
			if(id)
			{
				ULOGGER_DEBUG("Creating %d with %d keypoints", *iter, visualWords.size());
				Signature * ss = new Signature(id, visualWords);
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
			}

			//reset
			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

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
		std::stringstream query;
		std::list<int> ids;

		// Get the map from signature and visual words
		query << "SELECT n.id "
				 "FROM Node AS n "
				 "WHERE n.time_enter >= (SELECT MAX(time_enter) FROM Statistics) "
				 "ORDER BY n.id;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			ids.push_back(sqlite3_column_int(ppStmt, 0)); 	// Signature id
			rc = sqlite3_step(ppStmt); // next result...
		}

		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Loading %d signatures...", ids.size());
		this->loadSignaturesQuery(ids, nodes);
		ULOGGER_DEBUG("loaded=%d, Time=%fs", nodes.size(), timer.ticks());
	}
}

// TODO DO IT IN A MORE EFFICiENT WAY !!!!
void DBDriverSqlite3::loadQuery(VWDictionary * dictionary) const
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
				 "WHERE vw.time_enter >= (SELECT MAX(time_enter) FROM Statistics) "
				 "ORDER BY vw.id;";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

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

		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

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
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		int descriptorSize;
		const void * descriptor;
		int dRealSize;
		for(std::set<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
		{
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, *iter);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			if(rc == SQLITE_ROW)
			{
				int index = 0;
				descriptorSize = sqlite3_column_int(ppStmt, index++); // VisualWord descriptor size
				descriptor = sqlite3_column_blob(ppStmt, index); 	// VisualWord descriptor array
				dRealSize = sqlite3_column_bytes(ppStmt, index++);

				if(dRealSize/int(sizeof(float)) != descriptorSize)
				{
					UERROR("Saved buffer size (%d) is not the same as descriptor size (%d)", dRealSize/sizeof(float), descriptorSize);
				}

				VisualWord * vw = new VisualWord(*iter, &((const float *)descriptor)[0], descriptorSize);
				if(vw)
				{
					vw->setSaved(true);
				}
				vws.push_back(vw);
				loaded.insert(loaded.end(), *iter);

				rc = sqlite3_step(ppStmt);
			}

			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Time=%fs", timer.ticks());

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

void DBDriverSqlite3::loadNeighborsQuery(int signatureId, std::set<int> & neighbors) const
{
	neighbors.clear();
	if(_ppDb)
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		std::stringstream query;

		query << "SELECT to_id FROM Link "
		      << "WHERE from_id = " << signatureId
			  << " ORDER BY to_id";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		int toId = -1;

		// Process the result if one
		rc = sqlite3_step(ppStmt);
		while(rc == SQLITE_ROW)
		{
			int index = 0;

			toId = sqlite3_column_int(ppStmt, index++);

			neighbors.insert(neighbors.end(), toId);
			rc = sqlite3_step(ppStmt);
		}

		UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		if(neighbors.size() == 0)
		{
			UERROR("No neighbors loaded from signature %d", signatureId);
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
		int totalLinksLoaded = 0;

		query << "SELECT to_id, type FROM Link "
			  << "WHERE from_id = ? "
			  << "ORDER BY to_id";

		rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<Signature*>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			// bind id
			rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

			int toId = -1;
			int linkType = -1;
			std::set<int> neighbors;
			std::set<int> loopIds;
			std::set<int> childIds;

			// Process the result if one
			rc = sqlite3_step(ppStmt);
			while(rc == SQLITE_ROW)
			{
				int index = 0;

				toId = sqlite3_column_int(ppStmt, index++);
				linkType = sqlite3_column_int(ppStmt, index++);

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
					neighbors.insert(neighbors.end(), toId);
				}

				++totalLinksLoaded;
				rc = sqlite3_step(ppStmt);
			}
			UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

			// add links
			(*iter)->addNeighbors(neighbors);
			(*iter)->setLoopClosureIds(loopIds);
			(*iter)->setChildLoopClosureIds(childIds);

			//reset
			rc = sqlite3_reset(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
			UINFO("time=%fs, node=%d, neighbors.size=%d, loopIds=%d, childIds=%d", (*iter)->id(), timer.ticks(), neighbors.size(), loopIds.size(), childIds.size());
		}

		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	}
}


void DBDriverSqlite3::updateQuery(const std::list<Signature *> & nodes) const
{
	if(_ppDb && nodes.size())
	{
		UTimer timer;
		timer.start();
		int rc = SQLITE_OK;
		sqlite3_stmt * ppStmt = 0;
		Signature * s = 0;

		std::string query = "UPDATE Node SET weight=?, time_enter = DATETIME('NOW') WHERE id=?;";
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<Signature *>::const_iterator i=nodes.begin(); i!=nodes.end(); ++i)
		{
			s = *i;
			int index = 1;
			if(s)
			{
				rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_bind_int(ppStmt, index++, s->id());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

				//step
				rc=sqlite3_step(ppStmt);
				UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_reset(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("Update Node table, Time=%fs", timer.ticks());

		// Update links part1
		query = "DELETE FROM Link WHERE from_id=?;";
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator j=nodes.begin(); j!=nodes.end(); ++j)
		{
			if((*j)->isNeighborsModified())
			{
				rc = sqlite3_bind_int(ppStmt, 1, (*j)->id());
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

				rc=sqlite3_step(ppStmt);
				UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

				rc = sqlite3_reset(ppStmt);
				UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		// Update links part2
		query = queryStepLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator j=nodes.begin(); j!=nodes.end(); ++j)
		{
			if((*j)->isNeighborsModified())
			{
				// Save neighbor links
				const std::set<int> & neighbors = (*j)->getNeighbors();
				for(std::set<int>::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
				{
					stepLink(ppStmt, (*j)->id(), *i, 0);
				}
				// save loop closure links
				const std::set<int> & loopIds = (*j)->getLoopClosureIds();
				for(std::set<int>::const_iterator i=loopIds.begin(); i!=loopIds.end(); ++i)
				{
					stepLink(ppStmt, (*j)->id(), *i, 1);
				}
				const std::set<int> & childIds = (*j)->getChildLoopClosureIds();
				for(std::set<int>::const_iterator i=childIds.begin(); i!=childIds.end(); ++i)
				{
					stepLink(ppStmt, (*j)->id(), *i, 2);
				}
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		ULOGGER_DEBUG("Update Neighbors Time=%fs", timer.ticks());

		// Update word references
		query = queryStepWordsChanged();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
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
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		ULOGGER_DEBUG("signatures update=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::saveQuery(const std::list<Signature *> & signatures) const
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
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			stepNode(ppStmt, *i);
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());

		// Create new entries in table Link
		query = queryStepLink();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator jter=signatures.begin(); jter!=signatures.end(); ++jter)
		{
			// Save neighbor links
			const std::set<int> & neighbors = (*jter)->getNeighbors();
			for(std::set<int>::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
			{
				stepLink(ppStmt, (*jter)->id(), *i, 0);
			}
			// save loop closure links
			const std::set<int> & loopIds = (*jter)->getLoopClosureIds();
			for(std::set<int>::const_iterator i=loopIds.begin(); i!=loopIds.end(); ++i)
			{
				stepLink(ppStmt, (*jter)->id(), *i, 1);
			}
			const std::set<int> & childIds = (*jter)->getChildLoopClosureIds();
			for(std::set<int>::const_iterator i=childIds.begin(); i!=childIds.end(); ++i)
			{
				stepLink(ppStmt, (*jter)->id(), *i, 2);
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());


		// Create new entries in table Map_Word_Node
		query = queryStepKeypoint();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			for(std::multimap<int, cv::KeyPoint>::const_iterator w=(*i)->getWords().begin(); w!=(*i)->getWords().end(); ++w)
			{
				stepKeypoint(ppStmt, (*i)->id(), w->first, w->second);
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		UDEBUG("Time=%fs", timer.ticks());

		// Add images
		query = queryStepImage();
		rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		for(std::list<Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			if(!(*i)->getImage().empty())
			{
				stepImage(ppStmt, (*i)->id(), (*i)->getImage());
			}
		}
		// Finalize (delete) the statement
		rc = sqlite3_finalize(ppStmt);
		UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

		UDEBUG("Time=%fs", timer.ticks());
	}
}

void DBDriverSqlite3::saveQuery(const std::vector<VisualWord *> & words) const
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
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
			for(std::vector<VisualWord *>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
			{
				const VisualWord * w = *iter;
				if(w && !w->isSaved())
				{
					rc = sqlite3_bind_int(ppStmt, 1, w->id());
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
					rc = sqlite3_bind_int(ppStmt, 2, w->getDim());
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
					rc = sqlite3_bind_blob(ppStmt, 3, w->getDescriptor(), w->getDim()*sizeof(float), SQLITE_STATIC);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

					//execute query
					rc=sqlite3_step(ppStmt);
					UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

					rc = sqlite3_reset(ppStmt);
					UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
				}
			}
			// Finalize (delete) the statement
			rc = sqlite3_finalize(ppStmt);
			UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
		}

		UDEBUG("Time=%fs", timer.ticks());
	}
}

std::string DBDriverSqlite3::queryStepNode() const
{
	return "INSERT INTO Node(id, weight) VALUES(?,?);";
}
void DBDriverSqlite3::stepNode(sqlite3_stmt * ppStmt, const Signature * s) const
{
	if(!ppStmt || !s)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;

	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, s->id());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepImage() const
{
	return "INSERT INTO Image(id, raw_width, raw_height, raw_data_type, raw_compressed, raw_data) VALUES(?,?,?,?,?,?);";
}
void DBDriverSqlite3::stepImage(sqlite3_stmt * ppStmt, int id, const cv::Mat & image) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	std::vector<unsigned char> compressed;
	int index = 1;

	rc = sqlite3_bind_int(ppStmt, index++, id);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_bind_int(ppStmt, index++, image.cols);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, image.rows);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, image.type());
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, this->isImagesCompressed()?1:0);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	if(image.total())
	{
		if(this->isImagesCompressed())
		{
			cv::imencode(".png", image, compressed);
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
			if(!image.isContinuous())
			{
				UWARN("matrix is not continuous");
			}
			rc = sqlite3_bind_blob(ppStmt, index++, image.data, image.total()*image.elemSize(), SQLITE_STATIC);
		}
	}
	else
	{
		rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
	}
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepNodeToSensor() const
{
	return "INSERT INTO Node_To_Sensor(node_id, sensor_id, num) VALUES(?,?,?);";
}
void DBDriverSqlite3::stepNodeToSensor(sqlite3_stmt * ppStmt, int nodeId, int sensorId, int num) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}

	int rc = SQLITE_OK;
	std::vector<unsigned char> compressed;
	int index = 1;

	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, sensorId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, num);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	//step
	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepLink() const
{
	return "INSERT INTO Link(from_id, to_id, type) VALUES(?,?,?);";
}
void DBDriverSqlite3::stepLink(sqlite3_stmt * ppStmt, int fromId, int toId, int type) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	UDEBUG("Save link from %d to %d, type=%d", fromId, toId, type);
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, fromId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, toId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, type);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepWordsChanged() const
{
	return "UPDATE Map_Node_Word SET word_id = ? WHERE word_id = ? AND node_id = ?;";
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
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, oldWordId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
}

std::string DBDriverSqlite3::queryStepKeypoint() const
{
	return "INSERT INTO Map_Node_Word(node_id, word_id, pos_x, pos_y, size, dir, response) VALUES(?,?,?,?,?,?,?);";
}
void DBDriverSqlite3::stepKeypoint(sqlite3_stmt * ppStmt, int nodeId, int wordId, const cv::KeyPoint & kp) const
{
	if(!ppStmt)
	{
		UFATAL("");
	}
	int rc = SQLITE_OK;
	int index = 1;
	rc = sqlite3_bind_int(ppStmt, index++, nodeId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, wordId);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.pt.x);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.pt.y);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_int(ppStmt, index++, kp.size);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.angle);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
	rc = sqlite3_bind_double(ppStmt, index++, kp.response);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc=sqlite3_step(ppStmt);
	UASSERT_MSG(rc == SQLITE_DONE, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());

	rc = sqlite3_reset(ppStmt);
	UASSERT_MSG(rc == SQLITE_OK, uFormat("DB error: %s", sqlite3_errmsg(_ppDb)).c_str());
}

} // namespace rtabmap
