-- *******************************************************************
--  construct_avpd_db: Script for creating the database
--   Usage:
--       $ sqlite3 AvpdDatabase.db < DatabaseSchema.sql
--
-- *******************************************************************

-- *******************************************************************
-- CLEAN
-- *******************************************************************
/*DROP TABLE Signature;
DROP TABLE SignatureType;
DROP TABLE Neighbor;
DROP TABLE VisualWord;
DROP TABLE Map_SS_VW;
DROP TABLE StatisticsAfterRun;
DROP TABLE StatisticsAfterRunSurf;*/

-- *******************************************************************
-- CREATE
-- *******************************************************************
CREATE TABLE Signature (
	id INTEGER NOT NULL,
	type VARCHAR NOT NULL,
	weight INTEGER,
	loopClosureId INTEGER,
	image BLOB,
	imgWidth INTEGER,
	imgHeight INTEGER,
	timeEnter DATE,
	PRIMARY KEY (id),
	FOREIGN KEY (type) REFERENCES SignatureType(type),
	FOREIGN KEY (loopClosureId) REFERENCES Signature(id)
);

CREATE TABLE Neighbor (
    sid INTEGER NOT NULL,
    nid INTEGER NOT NULL,
    actionSize INTEGER,
    actions BLOB,
    timeEnter DATE,
    PRIMARY KEY (sid, nid),
    FOREIGN KEY (sid) REFERENCES Signature(id),
    FOREIGN KEY (nid) REFERENCES Signature(id)
);

CREATE TABLE SignatureType (
	type VARCHAR NOT NULL,
	PRIMARY KEY (type)
);

CREATE TABLE VisualWord (
	id INTEGER NOT NULL,
	descriptorSize INTEGER NOT NULL,
	descriptor BLOB NOT NULL,
	timeEnter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Map_SS_VW (
	signatureId INTEGER NOT NULL,
	visualWordId INTEGER NOT NULL,
	pos_x FLOAT NOT NULL,
	pos_y FLOAT NOT NULL,
	laplacian INTEGER NOT NULL,
	size INTEGER NOT NULL,
	dir FLOAT NOT NULL,
	hessian FLOAT NOT NULL,
	timeEnter DATE,
	FOREIGN KEY (signatureId) REFERENCES Signature(id),
	FOREIGN KEY (visualWordId) REFERENCES VisualWord(id)
);

CREATE TABLE StatisticsAfterRun (
	stMemSize INTEGER,
	lastSignAdded INTEGER,
	processMemUsed INTEGER,
	databaseMemUsed INTEGER,
	timeEnter DATE
);

CREATE TABLE StatisticsAfterRunSurf (
	dictionarySize INTEGER,
	timeEnter DATE
);

-- *******************************************************************
-- TRIGGERS
-- *******************************************************************
CREATE TRIGGER insert_Signature BEFORE INSERT ON Signature 
WHEN NOT EXISTS (SELECT type FROM SignatureType WHERE SignatureType.type = NEW.type)
BEGIN
 SELECT RAISE(ABORT, 'Foreign key constraint failed');
END;

CREATE TRIGGER insert_Neighbor BEFORE INSERT ON Neighbor 
WHEN NOT EXISTS (SELECT id FROM Signature WHERE Signature.id = NEW.sid)
BEGIN
 SELECT RAISE(ABORT, 'Foreign key constraint failed');
END;

CREATE TRIGGER insert_Map_SS_VW BEFORE INSERT ON Map_SS_VW 
WHEN NOT EXISTS (SELECT type FROM Signature WHERE Signature.id = NEW.signatureId AND type='surf')
--OR NOT EXISTS (SELECT id FROM VisualWord WHERE VisualWord.id = NEW.visualWordId)
BEGIN
 SELECT RAISE(ABORT, 'Foreign key constraint failed');
END;

 --   Creating a trigger for timeEnter
CREATE TRIGGER insert_Signature_timeEnter AFTER INSERT ON Signature
BEGIN
 UPDATE Signature SET timeEnter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Neighbor_timeEnter AFTER INSERT ON Neighbor
BEGIN
 UPDATE Neighbor SET timeEnter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_VisualWord_timeEnter AFTER INSERT ON VisualWord
BEGIN
 UPDATE VisualWord SET timeEnter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Map_SS_VW_timeEnter AFTER INSERT ON Map_SS_VW
BEGIN
 UPDATE Map_SS_VW SET timeEnter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_StatisticsAfterRun_timeEnter AFTER INSERT ON StatisticsAfterRun
BEGIN
 UPDATE StatisticsAfterRun SET timeEnter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_StatisticsAfterRunSurf_timeEnter AFTER INSERT ON StatisticsAfterRunSurf
BEGIN
 UPDATE StatisticsAfterRunSurf SET timeEnter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;


-- *******************************************************************
-- INDEXES
-- *******************************************************************
CREATE INDEX IDX_Map_SS_VW_SignatureId on Map_SS_VW (signatureId);
CREATE INDEX IDX_Map_SS_VW_VisualWordId on Map_SS_VW (visualWordId);
CREATE INDEX IDX_Signature_Id on Signature (id);
CREATE INDEX IDX_VisualWord_Id on VisualWord (id);
CREATE INDEX IDX_Signature_TimeEnter on Signature (timeEnter);
CREATE INDEX IDX_VisualWord_TimeEnter on VisualWord (timeEnter);
CREATE INDEX IDX_Neighbor_Sid on Neighbor (sid);

-- *******************************************************************
-- Data
-- *******************************************************************
INSERT INTO SignatureType(type) VALUES ('fourier');
INSERT INTO SignatureType(type) VALUES ('surf');

-- *******************************************************************
-- TESTS
-- *******************************************************************
-- *** Data Test ***
/*
INSERT INTO Signature VALUES(1, 'surf', null, null, null);
INSERT INTO Signature VALUES(2, 'surf', null, null, null);
INSERT INTO Signature VALUES(3, 'surf', null, null, null);
INSERT INTO VisualWord VALUES (1, 1, 2,'0.213213 0.4352323', null);
INSERT INTO VisualWord VALUES (2, 1, 2,'0.213213 0.4352323', null);
INSERT INTO VisualWord VALUES (3, 3, 2,'0.213213 0.4352323', null);
INSERT INTO Map_SS_VW VALUES (1, 1, 0,0,0,0,0, null);
INSERT INTO Map_SS_VW VALUES (2, 1, 0,0,0,0,0, null);
INSERT INTO Map_SS_VW VALUES (2, 2, 0,0,0,0,0, null);
*/

/*
-- For loading words
SELECT vw.id, vw.laplacian, vw.descriptorSize, vw.descriptor, m.signatureId FROM VisualWord as vw INNER JOIN Map_SS_VW as m on vw.id=m.visualWordId ORDER BY vw.id;
*/


-- Refreshing the dictionary
/*SELECT * FROM Map_SS_VW;
SELECT * FROM VisualWord;*/
/*
DELETE FROM VisualWord;
INSERT INTO VisualWord VALUES (1, 1, 2,'0.213213 0.4352323', null);
DELETE FROM Map_SS_VW WHERE NOT EXISTS (SELECT id FROM VisualWord WHERE id = Map_SS_VW.visualWordId);
*/
/*SELECT * FROM Map_SS_VW;
SELECT * FROM VisualWord;*/

/*
-- Loading only signatures on the last short time memory based on DATE
INSERT INTO Signature VALUES(4, 'surf', null, null, null);
INSERT INTO Signature VALUES(5, 'surf', 4, null, null);
INSERT INTO Signature VALUES(6, 'surf', null, null, null);
INSERT INTO Map_SS_VW VALUES (4, 1, 0,0,0,0,0, null);
SELECT s.id FROM Signature AS s WHERE s.timeEnter >= (SELECT vw.timeEnter FROM VisualWord AS vw LIMIT 1) AND s.loopClosureId IS NULL;
*/
