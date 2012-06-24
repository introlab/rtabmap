-- *******************************************************************
--  construct_avpd_db: Script for creating the database
--   Usage:
--       $ sqlite3 LTM.db < DatabaseSchema.sql
--
-- *******************************************************************

-- *******************************************************************
-- CLEAN
-- *******************************************************************
/*DROP TABLE Node;
DROP TABLE Link;
DROP TABLE Sensor;
DROP TABLE Actuator;
DROP TABLE Word;
DROP TABLE Map_Node_Word;
DROP TABLE Statistics;
DROP TABLE StatisticsSurf;*/

-- *******************************************************************
-- CREATE
-- *******************************************************************
CREATE TABLE Node (
	id INTEGER NOT NULL,
	type INTEGER NOT NULL, -- 0=Keypoint, 1=Sensor
	weight INTEGER,
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Sensor (
	id INTEGER NOT NULL,
	num INTEGER NOT NULL,
	type INTEGER NOT NULL, -- kTypeImage=0, kTypeImageFeatures2d, kTypeAudio, kTypeAudioFreq, kTypeAudioFreqSqrdMagn, kTypeJointState, kTypeNotSpecified
	data BLOB, -- PostProcessed data (indexed integers)
	raw_width INTEGER NOT NULL,
	raw_height INTEGER NOT NULL,
	raw_data_type INTEGER NOT NULL,
	raw_compressed CHAR NOT NULL,
	raw_data BLOB,
	PRIMARY KEY (id, num)
);

CREATE TABLE Link (
	from_id INTEGER NOT NULL,
	to_id INTEGER NOT NULL,
	type INTEGER NOT NULL, -- neighbor=0, loop=1, child=2
	actuator_id INTEGER,
	base_ids BLOB,
	FOREIGN KEY (from_id) REFERENCES Node(id),
	FOREIGN KEY (to_id) REFERENCES Node(id)
);

CREATE TABLE Actuator (
	id INTEGER NOT NULL,
	num INTEGER NOT NULL,
	type INTEGER NOT NULL, -- kTypeTwist=0, kTypeNotSpecified
	width INTEGER NOT NULL,
	height INTEGER NOT NULL,
	data_type INTEGER NOT NULL,
	data BLOB,
	PRIMARY KEY (id, num)
);

-- 
CREATE TABLE Word (
	id INTEGER NOT NULL,
	descriptor_size INTEGER NOT NULL,
	descriptor BLOB NOT NULL,
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Map_Node_Word (
	node_id INTEGER NOT NULL,
	word_id INTEGER NOT NULL,
	pos_x FLOAT NOT NULL,
	pos_y FLOAT NOT NULL,
	size INTEGER NOT NULL,
	dir FLOAT NOT NULL,
	response FLOAT NOT NULL,
	FOREIGN KEY (node_id) REFERENCES Node(id),
	FOREIGN KEY (word_id) REFERENCES Word(id)
);

CREATE TABLE Statistics (
	STM_size INTEGER,
	last_sign_added INTEGER,
	process_mem_used INTEGER,
	database_mem_used INTEGER,
	time_enter DATE
);

CREATE TABLE StatisticsDictionary (
	dictionary_size INTEGER,
	time_enter DATE
);

-- *******************************************************************
-- TRIGGERS
-- *******************************************************************
CREATE TRIGGER insert_Map_Node_Word BEFORE INSERT ON Map_Node_Word 
WHEN NOT EXISTS (SELECT type FROM Node WHERE Node.id = NEW.node_id AND type=0)
BEGIN
 SELECT RAISE(ABORT, 'Keypoint type constraint failed');
END;

 --   Creating a trigger for time_enter
CREATE TRIGGER insert_Node_timeEnter AFTER INSERT ON Node
BEGIN
 UPDATE Node SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Word_timeEnter AFTER INSERT ON Word
BEGIN
 UPDATE Word SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Statistics_timeEnter AFTER INSERT ON Statistics
BEGIN
 UPDATE Statistics SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_StatisticsDictionary_timeEnter AFTER INSERT ON StatisticsDictionary
BEGIN
 UPDATE StatisticsDictionary SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;


-- *******************************************************************
-- INDEXES
-- *******************************************************************
CREATE INDEX IDX_Map_Node_Word_node_id on Map_Node_Word (node_id);
CREATE INDEX IDX_Sensor_Id on Sensor (id);
CREATE INDEX IDX_Link_from_id on Link (from_id);

