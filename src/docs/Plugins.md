# Extend Recast with plugins

Plugins are simple Javascript functions implementing the [RecastPlugin](/docs/recast/RecastPlugin) interface.

```js
import Recast from 'recastjs/recast.mjs';
const recast = await Recast();

recast
  .withPlugin(plugin1)
  .withPlugin(plugin2)
```

Some plugins are already bundled in Recast: 

- [FlockGroup](/docs/recast/FlockGroup) - Loose emergent group behavior using cohesion
- [Formation](/docs/recast/Formation) - Directed geometric agents formation [demo](https://vincent.github.io/recast.js/tests/browser/flock/test.flock.html)
