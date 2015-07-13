# What is this all about

In a game context, Recast.js is meant to be used with a file describing a scene geometry. Using the [recastnavigation](https://github.com/memononen/recastnavigation) library, it will deduce a navigation mesh - a set of polygons on which your characters can move.

Once this navigation mesh is computed, it is possible to use built-in pathfinding operations like "find the shortest path between point A and point B", taking into account various user-defined variables like obstacles, slopes, and off-mesh connections.

If you decide to use it to animate your scene characters, it also provides a complete crowd system capable of managing all your characters movements using per-characters settings (speed, radius, ...)

# Getting started

Assuming you've got an .obj file describing your scene geometry, you can get a Recast instance with:

```js
var recast = new Recast();
recast.OBJLoader( 'path/to/geometry.obj', function () {

  // get a random navigable point A
  recast.getRandomPoint( function (x, y, z) {

    // find the shortest route from origin to point A
    // we asume 0,0,0 is a navigable point
    recast.findPath( 0, 0, 0,   x, y, z,   function ( route ) {

      console.log( "The shortest route contains", route.length, "corners" );
      console.log( "These corners are", route );
    })

  })
})
```

# Speed considerations

Computing the navigation mesh from your level can be quite cpu-intensive.
When this happens, a solution is to store the computed navmesh for later use.

## Saving

```js
var recast = new Recast();
recast.OBJLoader('nav_test.obj', function(){

  recast.buildTiled(); // only tiled mesh is supported right now

  recast.saveTileMesh('./navmesh.bin', recast.cb(function (error, blob) {

    var buffer = new Buffer(blob.length);
    for (var i = 0; i < blob.length; i++) {
        buffer.writeUInt8(blob[i], i);
    }

    fs.writeFile('./navmesh.bin', buffer, function (err) {
        // done
    });
  }));
});
```

## Loading

```js
var recast = new Recast();
recast.OBJLoader('nav_test.obj', function(){

    recast.loadTileMesh('./navmesh.bin', recast.cb(function(){
      // recast is now ready to use
    }));
});
```

# API

`new Recast()` Create a new instance

## General
* [recast.settings](API#recast.settings) - Apply new navigation mesh settings
* [recast.OBJLoader](API#recast.OBJLoader) - Load an `.obj` file
* [recast.OBJDataLoader](API#recast.OBJLoader) - Load an `.obj` content
* [recast.buildSolo](API#recast.buildSolo) - Build a single-object navigation mesh
* [recast.buildTiled](API#recast.buildTiled) - Build a tiled navigation mesh
* [recast.saveTileMesh](API#recast.saveTileMesh) - Store a tiled navigation mesh
* [recast.loadTileMesh](API#recast.loadTileMesh) - Load a tiled navigation mesh
* [recast.saveTileCache](API#recast.saveTileCache) - Store a tiled navigation tilecache
* [recast.loadTileCache](API#recast.loadTileCache) - Load a tiled navigation tilecache

## Basic navmesh operations
* [recast.getRandomPoint](API#recast.getRandomPoint) - Get a random navigable position
* [recast.findNearestPoint](API#recast.findNearestPoint) - Find the nearest navigable point
* [recast.findNearestPoly](API#recast.findNearestPoly) - Find the nearest navigable polygon
* [recast.queryPolygons](API#recast.queryPolygons) - Find nearest polygons
* [recast.setPolyFlags](API#recast.setPolyFlags) - Set flags on nearest polygons
* [recast.setPolyFlagsByRef](API#recast.setPolyFlagsByRef) - Set flags on a single polygon
* [recast.findPath](API#recast.findPath) - Find a path between two points

## Basic agents operations
* [recast.addAgent](API#recast.addAgent) - Add an agent
* [recast.updateCrowdAgentParameters](API#recast.updateCrowdAgentParameters) - Add an agent settings
* [recast.removeCrowdAgent](API#recast.removeCrowdAgent) - Remove an agent

## Crowd
* [recast.initCrowd](API#recast.initCrowd) - Initialize the crowd system
* [recast.crowdRequestMoveTarget](API#recast.crowdRequestMoveTarget) - Set an agent's destination
* [recast.crowdUpdate](API#recast.crowdUpdate) - Update the crowd system
* [recast.crowdGetActiveAgents](API#recast.crowdGetActiveAgents) - Retrieve all crowd agents
* [recast.requestMoveVelocity](API#recast.requestMoveVelocity) - Set an agent's velocity

## Temporary obstacles
* [recast.addTempObstacle](API#recast.addTempObstacle) - Add a temporary obstacle
* [recast.removeTempObstacle](API#recast.removeTempObstacle) - Remove a temporary obstacle
* [recast.getAllTempObstacles](API#recast.getAllTempObstacles) - Retrieve all temporary obstacles
* [recast.clearAllTempObstacles](API#recast.clearAllTempObstacles) - Remove all temporary obstacles

## Off-mesh connections
* [recast.addOffMeshConnection](API#recast.addOffMeshConnection) - Add an off-mesh connection

## Zones
* [recast.Zone](API#recast.Zone) - Create a new zone
* [zone.isWalkable](API#recast.Zone) - Test if a zone is walkable
* [zone.is](API#recast.Zone) - Test a specific flag
* [zone.setFlags](API#recast.Zone) - Set specified flags
* [zone.clearFlags](API#recast.Zone) - Clear specified flags
* [zone.toggleFlags](API#recast.Zone) - Toggle specified flags

# Troubleshooting

Some tips to help you getting rid of well-known traps

* test your geometry in [Recast](https://github.com/memononen/recastnavigation) - it is easily runnable on Windows, MacOS and Linux
* adjust settings wisely. Test them in Recast if you are not sure.



