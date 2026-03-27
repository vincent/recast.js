A `recast` binary is included for working with navmeshes from the command line:

```bash
# Build a navmesh from an OBJ file
recast build-navmesh scene.obj navmesh.bin

# Build a tile cache (supports dynamic obstacles)
recast build-tilecache scene.obj scene.tilecache

# Find a path from the closest walkable point to x=0,y=0,z=0 to the closest walkable point 10,0,10 and get JSON output
recast find-path navmesh.bin --from 0,0,0 --to 10,0,10

# Inspect a saved navmesh or tile cache
recast inspect navmesh.bin
```

Common options: 

```shell
--cellSize 0.3
```
Voxel width/depth of a cell. Smaller = higher resolution

```shell
--cellHeight 0.2
```

Voxel height
```shell
--agentHeight 2.0
```

Minimum floor-to-ceiling clearance for a walkable area
```shell
--agentRadius 0.4
```

Agent clearance radius from walls
```shell
--agentMaxClimb 0.9
```

Maximum step height an agent can climb
```shell
--agentMaxSlope 30
```

Maximum walkable slope in degrees
```shell
--regionMinSize 8
```

Region minimum size in voxels
```shell
--regionMergeSize 20
```

Region merge size in voxels.
```shell
--edgeMaxLen 12
```

Edge max length in world units
```shell
--edgeMaxError 1.3
```
