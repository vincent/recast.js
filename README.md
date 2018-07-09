# What is this all about

In a game context, Recast.js is meant to be used with a file describing a scene geometry. Using the [recastnavigation](https://github.com/recastnavigation/recastnavigation) library, it will deduce a [navigation mesh](https://en.wikipedia.org/wiki/Navigation_mesh) - a set of polygons on which your characters can move.

Once this navigation mesh is computed, it is possible to use built-in pathfinding operations like "find the shortest path between point A and point B", taking into account various variables like obstacles, slopes, and off-mesh connections.

If you decide to use it to animate your scene characters - aka "agents", it also provides a complete crowd system capable of managing all your agents movements using per-agent settings \(speed, radius, ...\)

## What can you do with it

* load any mesh in .obj format
* compute and extract its navigation mesh with options
* save this navigation mesh
* load this blob without recompute it
* find a random point guaranteed to be navigable
* find the nearest path from one point to another
* place obstacles with a radius
* add agents on the navigation mesh
* make them move with their own speed

## Is it for WebGL, Three.js or Babylon.js ?

It is designed to work along a WebGL software but it's completely library agnostic. It only manages a mesh and its properties.

Included tests and demos use [Three.js](https://github.com/mrdoob/three.js), though.



