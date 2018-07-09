---
description: >-
  You can create a navigation mesh from an .obj file describing your scene
  geometry.
---

# Create a navmesh

## TL ; DR

```javascript
recast.OBJLoader('path/to/geometry.obj', function() {

  // build the navmesh (buildSolo or buildTiled)
  recast.buildTiled();
    
  // get a random navigable point A
  recast.getRandomPoint(recast.cb(function(x, y, z) {

    // find the shortest route from origin to point A
    // we assume 0,0,0 is a navigable point
    recast.findPath(0, 0, 0, x, y, z, 1000, recast.cb(function(path) {

      console.log('The shortest route contains', path.length, 'segments');
      console.log('These segments are', path);
    }));
  }));
});
```

## Use an OBJ file

### From an async request

```javascript
recast.OBJLoader('path/to/file.obj', function(){
   // file is loaded
})
```

### From raw data

```javascript
recast.OBJDataLoader(OBJ_RAW_DATA_AS_STRING, function(){
   // file is loaded
})
```

## Choosing a data structure

Depending on your usage of Recast.js, you will choose the appropriate data structure.

### Solo

The "Solo" constructor creates a single, immutable object to represent your navigation mesh. 

```javascript
recast.buildSolo()
```

{% hint style="success" %}
Slower to generate, but faster to update
{% endhint %}

{% hint style="warning" %}
Disable some features like temporary obstacles
{% endhint %}

### Tiled

The "Tiled" constructor creates a complex object to represent your navigation mesh, on which you can add or remove "Temporary obstacles" that will alter navigation behaviors. 

```javascript
recast.buildTiled()
```

{% hint style="success" %}
Faster to generate, but slower to update
{% endhint %}

{% hint style="warning" %}
Disable some features like off-mesh connections
{% endhint %}



