-- wormholes_schema.sql
CREATE TABLE IF NOT EXISTS wormholes (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  from_map TEXT NOT NULL,
  to_map   TEXT NOT NULL,
  x REAL NOT NULL,
  y REAL NOT NULL,
  yaw REAL DEFAULT 0.0
);

INSERT INTO wormholes (from_map,to_map,x,y,yaw) VALUES ('map1','map2', 10.0, 1.0, 0.00);
INSERT INTO wormholes (from_map,to_map,x,y,yaw) VALUES ('map2','map1', 10.0, 1.5, -3.1457);
