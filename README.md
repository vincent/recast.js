Recast.js
=========

A Javascript library to handle navigation meshes, in nodejs and the browser.

It embeds an Emscripten-compiled bundle of the [RecastDetour navigation c++ library](https://github.com/memononen/recastnavigation)

[Demo](http://vincent.github.io/recast.js/tests/test.webgl.html) - [Getting started](https://github.com/vincent/recast.js/wiki) - [API](https://github.com/vincent/recast.js/wiki/API)

[![Build Status](http://ci.three-arena.com/job/Recast.js/badge/icon)](http://ci.three-arena.com/job/Recast.js/)

What can I do with it ?
=========

* load any mesh in .obj format
* compute and extract its navigation mesh with options
* save this navigation mesh
* load this blob without recompute it
* find a random point guaranteed to be navigable
* find the nearest path from one point to another
* place obstacles with a radius
* add agents on the navigation mesh
* make them move with their own speed

Oh, that's a WebGL-Three.js-Babylon.js stuff ?
=========

It is designed to work along a WebGL software but it's completely library agnostic.
It only manages a mesh and its properties.

Tests
=========

Some tests exist in the [tests](https://github.com/vincent/recast.js/tree/master/tests) directory.
They describe regular usages of the library, and should pass both in node (npm test) and in the browser.
There are both simple and worker versions.

Build
=========

It supposes you already have a working emscripten toolchain installed. You can also define your paths in the Makefile.
The Emscripten build can be ran with
`make build`

To update the build against the latest [RecastDetour](https://github.com/memononen/recastnavigation) version, use `make update-source build`

Contribute
=========

You are welcome to contribute by forking the project and send pull requests !
There are many areas where it could be improved.

* the C++ Recast part is roughly borrowed from different posts from [the google group](https://groups.google.com/forum/#!forum/recastnavigation) and [the github source](https://github.com/memononen/recastnavigation)
* the navigation mesh is currently rebuilt every time
