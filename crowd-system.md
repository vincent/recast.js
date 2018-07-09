---
description: >-
  Recast.js can be used to manage a set of agents movements using its crowd
  system.
---

# Crowd system

## Agents

### Add an agent

```javascript
var agentId = recast.addAgent({
    position: { x:0, y:0, z:0 }, // initial position
    radius: 0.8,                 // agent radius
    height: 0.5,                 // agent height
    maxAcceleration: 1.0,        // maximum acceleration factor
    maxSpeed: 2.0,               // maximum speed
    updateFlags: 0,              // update flags
    separationWeight: 20.0       // separation factor
})
```

### Remove an agent

```javascript
recast.removeCrowdAgent(agentId)
```

## Crowd

### Initialize the crowd system

```javascript
recast.initCrowd(maximumAgents, radiusReference)
```



