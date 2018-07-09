# Getting started

## Install

```text
npm install recastjs --save
```

## Link the library

### In a browser

To use it in a browser, add a &lt;script&gt; tag, or use a UMD compatible module loader.

```markup
<script src="../node_modules/recastjs/lib/recast.js"></script>
```

then get a recast instance in the same thread

```javascript
var recast = window.recast;
```

or create a recast worker instance \(include `/lib/recast.withworker.js`\)

```javascript
var recastWorker = new recast('../lib/recast.js', callback);
```

### In Node.js

get a recast instance in the same thread

```javascript
var recast = require('recastjs');
```

or create a recast worker instance

```javascript
var Recast = require('recast.withworker');
var recastWorker = new Recast('recast');
```

