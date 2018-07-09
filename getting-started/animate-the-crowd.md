# Animate the crowd

## TL ; DR

```javascript
recast.addAgent({
    position: { x:0, y:0, z:0 }
})

recast.initCrowd(1000, 1.0);

recast.vent.on('update', function (agents) {
    agents.forEach(agent => {
        var agentRotationY = Math.atan2(- agent.velocity.z, agent.velocity.x);
        var agentPosition = {
            agent.position.x,
            agent.position.y,
            agent.position.z
        };
    })
})
```

