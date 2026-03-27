PACKAGE = recast.js
CWD := $(shell pwd)

BUILDDIR = lib
CC = emcc

CFLAGS_RELEASE = -O3 \
	-s WASM=1 \
	-s USE_SDL=2 \
	-s NO_EXIT_RUNTIME=1 \
	-s ALLOW_MEMORY_GROWTH=1 \
	-s ALLOW_TABLE_GROWTH=1 \
	-s DISABLE_EXCEPTION_CATCHING=1 \
	-s ASSERTIONS=0 \
	-s WARN_ON_UNDEFINED_SYMBOLS=1 \
	-s MODULARIZE=1 \
	-s EXPORT_NAME='Recast' \
	-s EXPORTED_RUNTIME_METHODS='["UTF8ToString","FS"]' \
	-s ENVIRONMENT='node,web,worker' \
	--bind

CFLAGS_DEBUG = -O0 -g \
	-s WASM=1 \
	-s USE_SDL=2 \
	-s NO_EXIT_RUNTIME=1 \
	-s ALLOW_MEMORY_GROWTH=1 \
	-s DISABLE_EXCEPTION_CATCHING=1 \
	-s ASSERTIONS=2 \
	-s WARN_ON_UNDEFINED_SYMBOLS=1 \
	-s MODULARIZE=1 \
	-s EXPORT_NAME='Recast' \
	-s EXPORTED_RUNTIME_METHODS='["UTF8ToString","FS"]' \
	-s ENVIRONMENT='node,web,worker' \
	--bind \
	--source-map-base ./

DEFINES = -D NOT_GCC -D EMSCRIPTEN -D USES_UNIX_DIR
INCLUDES = -I recastnavigation/Recast/Include \
		   -I recastnavigation/Detour/Include \
		   -I recastnavigation/DetourCrowd/Include \
		   -I recastnavigation/RecastDemo/Include \
		   -I recastnavigation/DebugUtils/Include \
		   -I recastnavigation/DetourTileCache/Include \
		   -I src/zlib \
		   -I src/JavascriptInterface

FILES = recastnavigation/DebugUtils/Source/DebugDraw.cpp \
		recastnavigation/DebugUtils/Source/DetourDebugDraw.cpp \
		recastnavigation/DebugUtils/Source/RecastDebugDraw.cpp \
		recastnavigation/DebugUtils/Source/RecastDump.cpp \
		recastnavigation/Detour/Source/DetourAlloc.cpp \
		recastnavigation/Detour/Source/DetourCommon.cpp \
		recastnavigation/Detour/Source/DetourNavMesh.cpp \
		recastnavigation/Detour/Source/DetourNavMeshBuilder.cpp \
		recastnavigation/Detour/Source/DetourNavMeshQuery.cpp \
		recastnavigation/Detour/Source/DetourNode.cpp \
		recastnavigation/DetourCrowd/Source/DetourCrowd.cpp \
		recastnavigation/DetourCrowd/Source/DetourLocalBoundary.cpp \
		recastnavigation/DetourCrowd/Source/DetourObstacleAvoidance.cpp \
		recastnavigation/DetourCrowd/Source/DetourPathCorridor.cpp \
		recastnavigation/DetourCrowd/Source/DetourPathQueue.cpp \
		recastnavigation/DetourCrowd/Source/DetourProximityGrid.cpp \
		recastnavigation/DetourTileCache/Source/DetourTileCache.cpp \
		recastnavigation/DetourTileCache/Source/DetourTileCacheBuilder.cpp \
		recastnavigation/Recast/Source/Recast.cpp \
		recastnavigation/Recast/Source/RecastAlloc.cpp \
		recastnavigation/Recast/Source/RecastArea.cpp \
		recastnavigation/Recast/Source/RecastContour.cpp \
		recastnavigation/Recast/Source/RecastFilter.cpp \
		recastnavigation/Recast/Source/RecastLayers.cpp \
		recastnavigation/Recast/Source/RecastMesh.cpp \
		recastnavigation/Recast/Source/RecastMeshDetail.cpp \
		recastnavigation/Recast/Source/RecastRasterization.cpp \
		recastnavigation/Recast/Source/RecastRegion.cpp \
		recastnavigation/RecastDemo/Source/ChunkyTriMesh.cpp \
		recastnavigation/RecastDemo/Source/Filelist.cpp \
		recastnavigation/RecastDemo/Source/InputGeom.cpp \
		recastnavigation/RecastDemo/Source/MeshLoaderObj.cpp \
		recastnavigation/RecastDemo/Source/PerfTimer.cpp \
		recastnavigation/RecastDemo/Source/Sample.cpp \
		recastnavigation/RecastDemo/Source/Sample_Debug.cpp \
		recastnavigation/RecastDemo/Source/ValueHistory.cpp \
			\
		src/JavascriptInterface/JavascriptInterface.cpp \
		src/JavascriptInterface/main.cpp

FLAGS =
PRELOAD =

PREJS     = --pre-js src/pre.module.js --pre-js src/plugins/flock.js --pre-js src/plugins/formation.js
POSTJS    = --post-js src/post.module.js
LIBRARYJS = --js-library src/library_recast.js

all: clean build build-esm test

update-source:
	git submodule init; git submodule update; cd recastnavigation; patch -p1 < ../src/recastnavigation.patch

build:
	mkdir -p $(BUILDDIR)
	$(CC) $(FLAGS) $(DEFINES) $(INCLUDES) $(CFLAGS_RELEASE) $(FILES) $(LIBRARYJS) -s EXPORTED_FUNCTIONS='[]' $(PREJS) $(POSTJS) -o $(BUILDDIR)/recast.js $(PRELOAD)

build-esm:
	mkdir -p $(BUILDDIR)
	$(CC) $(FLAGS) $(DEFINES) $(INCLUDES) $(CFLAGS_RELEASE) $(FILES) $(LIBRARYJS) -s EXPORTED_FUNCTIONS='[]' $(PREJS) $(POSTJS) -s EXPORT_ES6=1 -o $(BUILDDIR)/recast.mjs $(PRELOAD)

build-debug:
	mkdir -p $(BUILDDIR)
	$(CC) $(FLAGS) $(DEFINES) $(INCLUDES) $(CFLAGS_DEBUG) $(FILES) $(LIBRARYJS) -s EXPORTED_FUNCTIONS='[]' $(PREJS) $(POSTJS) -o $(BUILDDIR)/recast.debug.js $(PRELOAD)

build-all: clean build build-esm build-debug

docs:
	npx typedoc lib/recast.d.ts --plugin typedoc-github-theme

lint:
	npm run lint

test:
	npx vitest run

clean:
	rm -rf $(BUILDDIR)

.PHONY: lint test build build-esm build-debug docs all
