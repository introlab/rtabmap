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
	map_id INTEGER NOT NULL,
	weight INTEGER,
	stamp FLOAT,
	pose BLOB,                -- 3x4 float
	ground_truth_pose BLOB,   -- 3x4 float
	velocity BLOB,            -- 6 float (vx,vy,vz,vroll,vpitch,vyaw) m/s and rad/s
	label TEXT,
	gps BLOB,                 -- 1x6 double: stamp, longitude (DD), latitude (DD), altitude (m), accuracy (m), bearing (North 0->360 deg clockwise)
	
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Data (
	id INTEGER NOT NULL,
	image BLOB,               -- compressed image (Grayscale or RGB)
	depth BLOB,               -- compressed image (Depth or Right image)
	calibration BLOB,         -- fx, fy, cx, cy, [baseline,] width, height, local_transform
	
	scan BLOB,                -- compressed data (Laser scan)
	scan_info BLOB,           -- scan_max_pts, scan_max_range, scan_format, local_transform
	
	ground_cells BLOB,        -- compressed data (occupancy grid)
	obstacle_cells BLOB,      -- compressed data (occupancy grid)
	empty_cells BLOB,         -- compressed data (occupancy grid)
	cell_size FLOAT,
	view_point_x FLOAT,
	view_point_y FLOAT,
	view_point_z FLOAT,
	
	user_data BLOB,           -- compressed data (User data)
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Link (
	from_id INTEGER NOT NULL,
	to_id INTEGER NOT NULL,
	type INTEGER NOT NULL,    -- neighbor=0, loop=1, child=2
	information_matrix BLOB NOT NULL, -- 6x6 double (inverse covariance)
	transform BLOB,           -- 3x4 float
	user_data BLOB,           -- compressed data (User data)
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

CREATE TABLE Feature (
	node_id INTEGER NOT NULL,
	word_id INTEGER NOT NULL,
	pos_x FLOAT NOT NULL,
	pos_y FLOAT NOT NULL,
	size INTEGER NOT NULL,
	dir FLOAT NOT NULL,
	response FLOAT NOT NULL,
	octave INTEGER NOT NULL,
	depth_x FLOAT,
	depth_y FLOAT,
	depth_z FLOAT,
	descriptor_size INTEGER,
	descriptor BLOB,
	FOREIGN KEY (node_id) REFERENCES Node(id)
);

CREATE TABLE Info (
	STM_size INTEGER,
	last_sign_added INTEGER,
	process_mem_used INTEGER,
	database_mem_used INTEGER,
	dictionary_size INTEGER,
	parameters TEXT,
	time_enter DATE
);

CREATE TABLE Statistics (
	id INTEGER NOT NULL,
	stamp FLOAT,
	data BLOB,              -- compressed string
	wm_state BLOB,	        -- compressed data
	FOREIGN KEY (id) REFERENCES Node(id)
);

CREATE TABLE Admin (
	version TEXT,
	preview_image BLOB,      -- compressed image
	
	opt_cloud BLOB,          -- compressed data
	opt_ids BLOB,            -- Node ids used to generate the optimized cloud/mesh
	opt_poses BLOB,          -- compressed N*3x4 float 
	opt_last_localization BLOB, -- 3x4 float
	opt_polygons_size INTEGER, -- e.g., 3
	opt_polygons BLOB,       -- compressed data [length_v0, i0,i1,i3, length_v1, i0,i1,i3]
	opt_tex_coords BLOB,     -- compressed data [length_v0, u0,v0,u1,v1,u2,v2, length_v1, u0,v0,u1,v1,u2,v2]
	opt_tex_materials BLOB,  -- compressed image
	opt_map BLOB,            -- compressed CV_8SC1 occupancy grid
	opt_map_x_min FLOAT,
	opt_map_y_min FLOAT, 
	opt_map_resolution FLOAT, 

	time_enter DATE
);

-- *******************************************************************
-- TRIGGERS
-- *******************************************************************
CREATE TRIGGER insert_Feature BEFORE INSERT ON Feature 
WHEN NOT EXISTS (SELECT Node.id FROM Node WHERE Node.id = NEW.node_id)
BEGIN
 SELECT RAISE(ABORT, 'Foreign key constraint failed in Feature table');
END;

 --   Creating a trigger for time_enter
CREATE TRIGGER insert_Node_timeEnter AFTER INSERT ON Node
BEGIN
 UPDATE Node SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Data_timeEnter AFTER INSERT ON Data
BEGIN
 UPDATE Node SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Word_timeEnter AFTER INSERT ON Word
BEGIN
 UPDATE Word SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Info_timeEnter AFTER INSERT ON Info
BEGIN
 UPDATE Info SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

-- *******************************************************************
-- INDEXES
-- *******************************************************************
CREATE UNIQUE INDEX IDX_Node_id on Node (id);
CREATE INDEX IDX_Feature_node_id on Feature (node_id);
CREATE INDEX IDX_Link_from_id on Link (from_id);
CREATE UNIQUE INDEX IDX_node_label on Node (label);
CREATE UNIQUE INDEX IDX_Statistics_id on Statistics (id);

-- *******************************************************************
-- VERSION
-- *******************************************************************
INSERT INTO Admin(version) VALUES('0.17.0');


