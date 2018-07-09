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

```javascript
var recast = window.recast;
```

create a recast worker instance in a browser \(include `/lib/recast.withworker.js`\)

```javascript
var recastWorker = new recast('../lib/recast.js', callback);
```

### In Node.js

get a recast instance in the same thread

```javascript
var recast = require('recastjs');
```

or create a recast worker instance in a `nodejs` environment

```javascript
var Recast = require('recast.withworker');
var recastWorker = new Recast('recast');
```

