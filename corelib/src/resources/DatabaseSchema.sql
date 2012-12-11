-- *******************************************************************
--  DatabaseSchema: Script for creating the database
--   Usage:
--       $ sqlite3 LTM.db < DatabaseSchema.sql
--
-- *******************************************************************

-- *******************************************************************
-- CLEAN
-- *******************************************************************
/*DROP TABLE Node;*/

-- *******************************************************************
-- CREATE
-- *******************************************************************
CREATE TABLE Node (
	id INTEGER NOT NULL,
	weight INTEGER,
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Image (
	id INTEGER NOT NULL,
	raw_width INTEGER NOT NULL,
	raw_height INTEGER NOT NULL,
	raw_data_type INTEGER NOT NULL,
	raw_compressed CHAR NOT NULL,
	raw_data BLOB,
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Link (
	from_id INTEGER NOT NULL,
	to_id INTEGER NOT NULL,
	type INTEGER NOT NULL, -- neighbor=0, loop=1, child=2
	FOREIGN KEY (from_id) REFERENCES Node(id),
	FOREIGN KEY (to_id) REFERENCES Node(id)
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
WHEN NOT EXISTS (SELECT Node.id FROM Node WHERE Node.id = NEW.node_id)
BEGIN
 SELECT RAISE(ABORT, 'Foreign key constraint failed in Map_Node_Word table');
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


-- *******************************************************************
-- INDEXES
-- *******************************************************************
CREATE INDEX IDX_Map_Node_Word_node_id on Map_Node_Word (node_id);
CREATE INDEX IDX_Link_from_id on Link (from_id);

