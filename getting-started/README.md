# Getting started

## Install

```text
npm install recastjs --save
```

## Link the library

### In a browser

To use it in a browser, by adding a &lt;script&gt; tag or other means

```markup
<script src="../node_modules/recastjs/lib/recast.js"></script>
```

get a recast instance in the same thread

```text
var recast = window.recast;
```

create a recast worker instance in a browser \(include `/lib/recast.withworker.js`\)

```text
var recastWorker = new recast('../lib/recast.js', callback);
```

### In nodejs

get a recast instance in the same thread

```javascript
var recast = require('recastjs');
```

or create a recast worker instance in a `nodejs` environment

```text
var Recast = require('recast.withworker');
var recastWorker = new Recast('recast');
```

