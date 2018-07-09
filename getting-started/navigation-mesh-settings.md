---
description: >-
  Your navigation mesh is dependant on the settings you choose to build it,
  carefully tune them to produce the best results.
---

# Navigation mesh settings

### Read the docs

The available settings are directly picked from the original RecastDetour library. You MUST read [Mikko's post about settings](http://digestingduck.blogspot.fr/2009/08/recast-settings-uncovered.html) for details. 

### Adjust settings

```javascript
recast.settings({
    cellSize: 0.1,
    cellHeight: 0.05,
    agentHeight: 1.8,
    agentRadius: 0.4,
    agentMaxClimb: 2.0,
    agentMaxSlope: 30.0
});
```

Supported options and their default value are

```text
cellSize        :  0.3   // voxelization cell size 
cellHeight      :  0.2   // voxelization cell height
agentHeight     :  2.0   // agent capsule height
agentRadius     :  0.4   // agent capsule radius
agentMaxClimb   :  0.9   // how high steps agents can climb, in voxels
agentMaxSlope   : 30.0   // maximum slope angle, in degrees
regionMinSize   :  8.0   // minimum isolated region size
regionMergeSize : 20.0   // how large regions can still be merged
edgeMaxLen      : 12.0   // maximum edge length, in voxels
edgeMaxError    :  1.3   // how loosely the simplification is done
```



