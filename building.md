# Building

Building Recast.js is not necessary to most use cases, but in case you want to contribute - pull requests are welcome ! - you will need to install compiling tools to test and produce a new build.

## Requirements

Compiling Recast.js needs the following tools to be installed

#### CMake

#### Emscripten

## Commands

Recast.js needs a patched version of [recastnavigation](https://github.com/recastnavigation/recastnavigation). To import or update it use

```bash
make update-source
```

Then compile Recast.js

```bash
npm run build
```



