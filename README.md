Recast.js
=========

A Javascript library to handle navigation meshes, in nodejs and the browser.

It embeds an Emscripten-compiled bundle of the [RecastDetour navigation c++ library](https://github.com/memononen/recastnavigation)

What can I do with it ?
=========

* load any mesh in .obj format
* compute and extract its navigation mesh with options (cell size, max slope, max jump)
* find a random point garanteed to be navigable
* find the nearest path from one point to another
* add agents on the navigation mesh
* make them move with their own speed

Oh, that's a 3D/WebGL/three.js/... stuff ?
=========

It is designed to work along a WebGL software but it's completely library agnostic.
It only manages a mesh and its properties.

Is there some live examples ?
=========

The library currently has no live examples, but it is used in [three-arena](https://github.com/vincent/three-arena) where one can find examples like [here](http://three-arena.com/examples/index.html)

Tests
=========

Some basic tests exist in the [tests](https://github.com/vincent/recast.js) directory.
They should pass both in node (npm test) and in the browser.
There are both simple and worker versions. 