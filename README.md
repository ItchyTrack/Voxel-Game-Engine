`cargo run --release`

### Tiles
A tile is typed data in a region.
Tiles are created, edited, and used in any way.
Tiles have paramaters, lod, and transform.
The engine's job is to create, manage, and use tiles.
Anyone can register a tile type.

Tiles creation is request from a single system.
This system has many tile sources that could generate this tile.
Every tile source can either create the whole tile or non of it.
Each tile source gives a estimation of the 'cost' and it picks the cheapest create a tile.

## \- Core

### Voxels
A voxel type defines the raw data stored in a grid.
Every grid has only one unchanging voxel type.
A voxel grid does not have a strongly typed voxel type and such must be checked when in use.
The use of voxels is to store data used to create tiles.

### Voxel Sources
Voxel sources is a way for voxels to be requested for use.
Voxel sources works in chunk coordinates.
It takes a voxel region request and splits in in to multiple voxel source implementations.
Voxel sources also defines ways of moving ownership of regions between voxel source implementations

### Voxel Assests
Voxel assets stores voxel data that has a immutable source and that is going to be used in may places.
Voxel assets are both a voxel source implementation and a tile creation source.
When tile creation is requested, it creates each tile once and returns refrences to the instance in later requests.
When voxel sources makes a requests it returns the raw data in that region.

### Voxel Streaming
Voxel streaming is a tiles creation source.
Voxel streaming manages tiles creation of tiles when their data comes from voxel sources.
Voxel streaming defines a way for others to register tile creation functions.
Each tile creation function runs async and makes requests to voxel sources till creation is done.
Voxel streaming saves this requests to build of what voxel regions each tile depends on.

### Voxel Tracking

### Presence

## \- Engine

### Tile Query

### Mass????

### Grid Sources

### Networking voxel source (Should it be a tile source?)

### Voxel Store

### Voxel Generator

### Voxel Physics

## \- Unsorted

### Gpu Stuff

### Basic Voxel

### Caches (Tiles data)

### ...
