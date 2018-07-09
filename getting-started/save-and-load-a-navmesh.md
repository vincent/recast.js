---
description: >-
  Creating a navigation mesh can be quite long and cpu-intensive. It is advised
  to save it to a binary file, so it can be loaded later.
---

# Save and load a navmesh

## Tiled

### Save a Tiled Navmesh

In a Node environment, you can save a navigation mesh as follows

```javascript
// load the level object
recast.OBJLoader('level_mesh.obj', function(){

  // build the navigation mesh
  recast.buildTiled();

  // serialize it
  recast.saveTileMesh('./navmesh.bin', recast.cb(function (error, serialized) {

    var buffer = new Buffer(serialized.length);
    for (var i = 0; i < serialized.length; i++) {
        buffer.writeUInt8(serialized[i], i);
    }

    // write to a binary file
    fs.writeFile('./navmesh.bin', buffer, function (err) {
        if (err) throw err;
    }));
});

```

### Load a Tiled Navmesh

In a browser, or Node environment

```javascript
 recast.loadTileMesh('./navmesh.bin', recast.cb(function(){
    // the navigation mesh is ready
 }));
```

