## General

```js
var recast = new Recast(optionnal_path_to_worker) // Creates a new instance
```

***

### Parameters types
* coordinates and extends are specified in floats, in the geometry datum
* other values are integers
* flags and areas types are specified in bitmasks combinations of

 - recast.FLAG_WALK
 - recast.FLAG_SWIM
 - recast.FLAG_DOOR
 - recast.FLAG_JUMP
 - recast.FLAG_DISABLED
 - recast.FLAG_ALL

You can mix multiple flags with bitwise operations:

`flags = recast.FLAG_WALK | recast.FLAG_JUMP`

***

<a name="recast.settings" href="API#recast.settings">#</a> recast.<b>settings</b>(<i>options</i>)

Apply new navigation mesh settings. It is required to `build()` the navmesh to have them applied.
<br>
Have a look to [Mikko's post about settings](http://digestingduck.blogspot.fr/2009/08/recast-settings-uncovered.html) for details.
<br>
Supported options and their default value are:

```js
cellSize        :  0.3   // voxelization cell size
cellHeight      :  0.2   // voxelization cell height
agentHeight     :  2.0   // agent capsule  height
agentRadius     :  0.4   // agent capsule  radius
agentMaxClimb   :  0.9   // how high steps agents can climb, in voxels
agentMaxSlope   : 30.0   // maximum slope angle, in degrees
regionMinSize   :  8.0   // minimum isolated region size that is still kept
regionMergeSize : 20.0   // how large regions can be still merged
edgeMaxLen      : 12.0   // maximum edge length, in voxels
edgeMaxError    :  1.3   // how loosely the simplification is done
```

***

<a name="recast.OBJLoader" href="API#recast.OBJLoader">#</a> recast.<b>OBJLoader</b>(<i>path or url</i>, <i>callback</i>)

Load an `.obj` file by its path. It uses `fs.readFile()` in Node and an `XmlHttpRequest` in the browser. You must do further operations on `recast` in the callback.

***

<a name="recast.OBJDataLoader" href="API#recast.OBJDataLoader">#</a> recast.<b>OBJDataLoader</b>(<i>OBJ contents as string</i>)

Load an `OBJ` directly with its content. Useful in some cases when you want to manage geometries on your own. You must do further operations on `recast` in the callback.

***

<a name="recast.buildSolo" href="API#recast.buildSolo">#</a> recast.<b>buildSolo</b>()

Build a single-object navigation mesh. It's generally slower to generate than a tiled navmesh and faster to update, but disable some features like temporary obstacles (unlikely to change limitation).

***

<a name="recast.buildTiled" href="API#recast.buildTiled">#</a> recast.<b>buildTiled</b>()

Build a tiled navigation mesh. It's generally faster to generate than a solo navmesh and slower to update, but disable some features like off-mesh connections (current limitation, likely to be enabled in the future).

***

<a name="recast.saveTileMesh" href="API#recast.saveTileMesh">#</a> recast.<b>saveTileMesh</b>(<i>path, callback</i>)

Returns a navmesh blob buffer. You can save this blob in a file to load it later.

***

<a name="recast.loadTileMesh" href="API#recast.loadTileMesh">#</a> recast.<b>loadTileMesh</b>(<i>path, callback</i>)

Load a navmesh blob. It is significantly faster than compute the navmesh from scratch.

***

<a name="recast.saveTileCache" href="API#recast.saveTileCache">#</a> recast.<b>saveTileCache</b>(<i>path, callback</i>)

Returns a navmesh tilecache blob buffer. You can save this blob in a file to load it later.

***

<a name="recast.loadTileCache" href="API#recast.loadTileCache">#</a> recast.<b>loadTileCache</b>(<i>path, callback</i>)

Load a navmesh tilecache blob. It is significantly faster than compute the navmesh from scratch.


## Basic navmesh operations

<a name="recast.getRandomPoint" href="API#recast.getRandomPoint">#</a> recast.<b>getRandomPoint</b>(<i>callback</i>)

Get a random navigable position. `callback` is called with 3 arguments `x, y, z`.

***

<a name="recast.findNearestPoint" href="API#recast.findNearestPoint">#</a> recast.<b>findNearestPoint</b>(<i>near_x</i>, <i>near_y</i>, <i>near_z</i>, <i>extend_x</i>, <i>extend_y</i>, <i>extend_z</i>, <i>callback</i>)

Find the nearest navigable point. `callback` is called with 3 arguments `x, y, z`.

***

<a name="recast.findNearestPoly" href="API#recast.findNearestPoly">#</a> recast.<b>findNearestPoly</b>(<i>near_x</i>, <i>near_y</i>, <i>near_z</i>, <i>extend_x</i>, <i>extend_y</i>, <i>extend_z</i>, <i>callback</i>)

Find the nearest navigable polygon. `callback` is called with 1 argument `polygon`.

```js
callback(polygon) {
    polygon.ref      // the polygon_id,
    polygon.vertices // array of vertices
}
```

***

<a name="recast.queryPolygons" href="API#recast.queryPolygons">#</a> recast.<b>queryPolygons</b>(<i>near_x</i>, <i>near_y</i>, <i>near_z</i>, <i>extend_x</i>, <i>extend_y</i>, <i>extend_z</i>, <i>max</i>, <i>callback</i>)

Find all or up to `max` nearest polygons from `near` point. `callback` is called with 1 argument: an array of polygons.

```js
callback(polygons) {
    polygon[0].ref      // the polygon_id,
    polygon[0].vertices // array of vertices
}
```

***

<a name="recast.setPolyFlags" href="API#recast.setPolyFlags">#</a> recast.<b>setPolyFlags</b>(<i>near_x</i>, <i>near_y</i>, <i>near_z</i>, <i>extend_x</i>, <i>extend_y</i>, <i>extend_z</i>, <i>flags</i>)

Set flags on nearest polygons from `near` point.

***

<a name="recast.setPolyFlagsByRef" href="API#recast.setPolyFlagsByRef">#</a> recast.<b>setPolyFlagsByRef</b>(<i>polygon_id</i>, <i>flags</i>)

Set flags on a single polygon.

***

<a name="recast.findPath" href="API#recast.findPath">#</a> recast.<b>findPath</b>(<i>from_x</i>, <i>from_y</i>, <i>from_z</i>, <i>to_x</i>, <i>to_y</i>, <i>to_z</i>, <i>max</i>, <i>callback</i>)

Find all - or up to `max` - `waypoints` points between two points. `callback` is called with 1 argument: an array of waypoints.

```js
callback(waypoints) {
    waypoints[0] == { x:value, y:value, z:value }
    ...
}
```

## Basic agents operations

<a name="recast.addAgent" href="API#recast.addAgent">#</a> recast.<b>addAgent</b>(<i>options</i>)

Add an agent on the navmesh and returns its `agent_id`. Supported options are:

```js
position: { x:0, y:0, z:0 }, // initial position
radius: 0.8,                 // agent radius
height: 0.5,                 // agent height
maxAcceleration: 1.0,        // maximum acceleration factor
maxSpeed: 2.0,               // maximum speed
updateFlags: 0,              // update flags
separationWeight: 20.0       // separation factor
```

***

<a name="recast.updateCrowdAgentParameters" href="API#recast.updateCrowdAgentParameters">#</a> recast.<b>updateCrowdAgentParameters</b>(<i>agent_id</i>, <i>options</i>)

Update an agent settings.

***

<a name="recast.removeCrowdAgent" href="API#recast.removeCrowdAgent">#</a> recast.<b>removeCrowdAgent</b>(<i>agent_id</i>)

Remove an agent from the crowd


## Crowd

<a name="recast.initCrowd" href="API#recast.initCrowd">#</a> recast.<b>initCrowd</b>(<i>max</i>, <i>radius</i>)

Initialize the crowd system with a maximum of `max` agents and a radius reference of `radius`.

***

<a name="recast.crowdRequestMoveTarget" href="API#recast.crowdRequestMoveTarget">#</a> recast.<b>crowdRequestMoveTarget</b>(<i>agent_id</i>, <i>dest_x</i>, <i>dest_y</i>, <i>dest_z</i>)

Set `agent_id`'s destination to `dest` point.

***

<a name="recast.crowdUpdate" href="API#recast.crowdUpdate">#</a> recast.<b>crowdUpdate</b>(<i>time_delta</i>)

Tick the crowd system, and update all its agents positions. It internally emits an `update` event with the result of `crowdGetActiveAgents`.

***

<a name="recast.crowdGetActiveAgents" href="API#recast.crowdGetActiveAgents">#</a> recast.<b>crowdGetActiveAgents</b>(<i>callback</i>)

Retrieve all crowd agents. `callback` is called with 1 argument: an array containing all agents with their position, speed, velocity and neighbors count.

```js
callback(agents) {
    agents[0].position      // current position
    agents[0].velocity      // current velocity
    agents[0].neighbors     // number of current neighbors
    agents[0].desiredSpeed  // desired speed
    ...
}
```

***

<a name="recast.requestMoveVelocity" href="API#recast.requestMoveVelocity">#</a> recast.<b>requestMoveVelocity</b>(<i>agent_id</i>, <i>x</i>, <i>y</i>, <i>z</i>)

Set an agent's velocity. You can use this method to set an agent movements from gamepad values.


## Temporary obstacles

<a name="recast.addTempObstacle" href="API#recast.addTempObstacle">#</a> recast.<b>addTempObstacle</b>(<i>x</i>, <i>y</i>, <i>z</i>, <i>radius</i>)

Add a temporary obstacle at desired position with a specified radius.

***

<a name="recast.removeTempObstacle" href="API#recast.removeTempObstacle">#</a> recast.<b>removeTempObstacle</b>(<i>obstacle_id</i>)

Remove `obstacle_id` temporary obstacle.

***

<a name="recast.getAllTempObstacles" href="API#recast.getAllTempObstacles">#</a> recast.<b>getAllTempObstacles</b>(<i>callback</i>)

Retrieve all temporary obstacles. `callback` is called with 1 argument: an array of all current temporary obstacles with their position and radius.

***

<a name="recast.clearAllTempObstacles" href="API#recast.clearAllTempObstacles">#</a> recast.<b>clearAllTempObstacles</b>()

Remove all temporary obstacles.


## Off-mesh connections

<a name="recast.addOffMeshConnection" href="API#recast.addOffMeshConnection">#</a> recast.<b>addOffMeshConnection</b>(<i>from_x</i>, <i>from_y</i>, <i>from_z</i>, <i>to_x</i>, <i>to_y</i>, <i>to_z</i>, <i>radius</i>, <i>both_directions</i>)

Add an off-mesh connection. If `both_directions` is true, connection will available in both directions. It is required to build() the navmesh to have them applied.

## Events

`recast.vent` is an event emitter on which you should listen for `update` events, to retrieve and apply all agents positions.

```js
recast.vent.on('update', function (all_agents) {

    for (var i = 0; i < all_agents.length; i++) {

        var agent     = all_agents[i];
        var character = scene.characters[agent.idx];

        // apply new position
        character.position.copy(agent.position);

        // apply rotation from velocity
        var angle = Math.atan2(- agent.velocity.y, agent.velocity.x);
        character.rotation.y = angle;
    }
})
```

## Zones

A Recast.js zone is a collection of some navigation mesh polygons. It can be used to quickly toggle flags on some polygons. Please note that these polygons do not necessarily match the original geometry vertices.

To easily define zones from your navigation mesh, it is recommended to use the [recast.js editor](https://github.com/vincent/recastjs-editor)

***

<a name="recast.Zone" href="API#recast.Zone">#</a> new recast.<b>Zone</b>(<i>name</i>, <i>data</i>)

Create a new zone named `name` described by the `data` object. This object is required to have a `refs` attribute - an array of polygon references - and
`flags` a flags bitmask.

<a name="Zone.Zone" href="API#recast.Zone">#</a> new recast.<b>Zone</b>(<i>name</i>, <i>data</i>)

***

<a name="Zone.isWalkable" href="API#Zone.isWalkable">#</a> zone.<b>isWalkable</b>()

Returns `true` if this zone is walkable.

***

<a name="Zone.is" href="API#Zone.is">#</a> zone.<b>is</b>(<i>flag</i>)

Returns `true` if this `flag` is applied on this zone.

***

<a name="Zone.setFlags" href="API#Zone.setFlags">#</a> zone.<b>setFlags</b>(<i>flags</i>)

Set the specified `flags` on this zone.

***

<a name="Zone.clearFlags" href="API#Zone.clearFlags">#</a> zone.<b>clearFlags</b>(<i>flags</i>)

Remove the specified `flags` from this zone.

***

<a name="Zone.toggleFlags" href="API#Zone.toggleFlags">#</a> zone.<b>toggleFlags</b>(<i>flags</i>)

Toggle the specified `flags` on this zone.

