PACKAGE = recast.js
NODEJS = $(if $(shell test -f /usr/bin/nodejs && echo "true"),nodejs,node)
CWD := $(shell pwd)
NODEUNIT = $(CWD)/node_modules/nodeunit/bin/nodeunit
UGLIFY = $(CWD)/node_modules/uglify-js/bin/uglifyjs
NODELINT = $(CWD)/node_modules/nodelint/nodelint

BUILDDIR = lib

CC = emcc
# FASTCOMPILER = EMCC_FAST_COMPILER=0

# PATH TO EMCC
#LLVM = ~/Workspace/emscripten-fastcomp/build/Release/bin
#CC = ~/Workspace/emscripten/emcc
# FASTCOMPILER = EMCC_FAST_COMPILER=0

CFLAGS = -O2 --closure 1 -g -s WARN_ON_UNDEFINED_SYMBOLS=0 -s VERBOSE=0 -s NO_EXIT_RUNTIME=1 -s LINKABLE=1 -s ALLOW_MEMORY_GROWTH=1 -s DISABLE_EXCEPTION_CATCHING=1 -s ASSERTIONS=0 --bind
DEFINES = -D NOT_GCC -D EMSCRIPTEN -D USES_UNIX_DIR
INCLUDES = -I recastnavigation/Recast/Include \
				 -I recastnavigation/Detour/Include \
				 -I recastnavigation/DetourCrowd/Include \
				 -I recastnavigation/RecastDemo/Include \
				 -I recastnavigation/DebugUtils/Include \
				 -I recastnavigation/DetourTileCache/Include \
				 -I src/zlib \
				 -I src/boost \
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

PREJS = --pre-js src/pre.module.js
POSTJS = --post-js src/post.module.js
LIBRARYJS = --js-library src/library_recast.js

all: clean test build

update-source:
	git submodule init; git submodule update; cd recastnavigation; git reset --hard origin/master; patch -p1 < ../src/recastnavigation.patch

build: $(wildcard  lib/*.js)
	mkdir -p $(BUILDDIR)
	$(FASTCOMPILER) $(PRE_FLAGS) $(CC) $(FLAGS) $(DEFINES) $(INCLUDES) $(CFLAGS) $(FILES) $(LIBRARYJS) -s EXPORTED_FUNCTIONS='[]' $(PREJS) $(POSTJS) -o $(BUILDDIR)/recast.js $(PRELOAD) --memory-init-file 0

test:
	$(NODEUNIT) tests

clean:
	rm -rf $(BUILDDIR)

lint:
	$(NODELINT) --config nodelint.cfg lib/recast.js

.PHONY: test build all
