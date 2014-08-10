#include <stdio.h>
#include <math.h>
#include <memory.h>
#include <time.h>
#include <sys/time.h>  
#include "fastlz.h"

#include <string>
#include <float.h>
#include <random>

#include <Recast.h>
#include <InputGeom.h>
#include <DetourNavMesh.h>
#include <DetourCommon.h>
#include <DetourTileCache.h>
#include <DetourNavMeshQuery.h>
#include <DetourNavMeshBuilder.h>
#include <DetourCrowd.h>

#include <RecastDebugDraw.h>
#include <DetourDebugDraw.h>

#include <emscripten.h>

#include <SampleInterfaces.h>

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
    SAMPLE_POLYAREA_GROUND,
    SAMPLE_POLYAREA_WATER,
    SAMPLE_POLYAREA_ROAD,
    SAMPLE_POLYAREA_DOOR,
    SAMPLE_POLYAREA_GRASS,
    SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
    SAMPLE_POLYFLAGS_WALK       = 0x01,     // Ability to walk (ground, grass, road)
    SAMPLE_POLYFLAGS_SWIM       = 0x02,     // Ability to swim (water).
    SAMPLE_POLYFLAGS_DOOR       = 0x04,     // Ability to move through doors.
    SAMPLE_POLYFLAGS_JUMP       = 0x08,     // Ability to jump.
    SAMPLE_POLYFLAGS_DISABLED   = 0x10,     // Disabled polygon
    SAMPLE_POLYFLAGS_ALL        = 0xffff    // All abilities.
};

// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static const int EXPECTED_LAYERS_PER_TILE = 4;

static const int MAX_POLYS = 256;

static const int MAX_LAYERS = 32;


extern "C" {
    extern void agentPool_clear();
    extern void agentPool_add(const int idx);
    extern void agentPool_get(const int idx,
                              const float pos_x,  const float pos_y, const float pos_z,
                              const float vel_x,  const float vel_y, const float vel_z,
                              const float radius, const int active,  const int state, const int neighbors,
                              const bool  partial,const float desiredSpeed);
    extern void flush_active_agents_callback();
    extern void invoke_vector_callback(int callback_id, const float x,  const float y, const float z);
    extern void invoke_update_callback(int callback_id);
    extern void invoke_generic_callback_string(int callback_id, const char* data);
    extern void gl_create_object(const char* objectName);
    extern void gl_draw_object(const char* objectName);
}

/////////////////////////////////

struct MeshProcess : public dtTileCacheMeshProcess
{
    InputGeom* m_geom;

    inline MeshProcess() : m_geom(0)
    {
    }

    inline void init(InputGeom* geom)
    {
        m_geom = geom;
    }
    
    virtual void process(struct dtNavMeshCreateParams* params,
                         unsigned char* polyAreas, unsigned short* polyFlags)
    {
        // Update poly flags from areas.
        for (int i = 0; i < params->polyCount; ++i)
        {
            if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
                polyAreas[i] = SAMPLE_POLYAREA_GROUND;

            if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
                polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
                polyAreas[i] == SAMPLE_POLYAREA_ROAD)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
            }
        }

        // Pass in off-mesh connections.
        if (m_geom)
        {
            params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
            params->offMeshConRad = m_geom->getOffMeshConnectionRads();
            params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
            params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
            params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
            params->offMeshConUserID = m_geom->getOffMeshConnectionId();
            params->offMeshConCount = m_geom->getOffMeshConnectionCount();  
        }
    }
};

struct FastLZCompressor : public dtTileCacheCompressor
{
    virtual int maxCompressedSize(const int bufferSize)
    {
        return (int)(bufferSize* 1.05f);
    }
    
    virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
                              unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
    {
        *compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
        return DT_SUCCESS;
    }
    
    virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
                                unsigned char* buffer, const int maxBufferSize, int* bufferSize)
    {
        *bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
        return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
    }
};

struct LinearAllocator : public dtTileCacheAlloc
{
    unsigned char* buffer;
    int capacity;
    int top;
    int high;
    
    LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
    {
        resize(cap);
    }
    
    ~LinearAllocator()
    {
        dtFree(buffer);
    }

    void resize(const int cap)
    {
        if (buffer) dtFree(buffer);
        buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
        capacity = cap;
    }
    
    virtual void reset()
    {
        high = dtMax(high, top);
        top = 0;
    }
    
    virtual void* alloc(const int size)
    {
        if (!buffer)
            return 0;
        if (top+size > capacity)
            return 0;
        unsigned char* mem = &buffer[top];
        top += size;
        return mem;
    }
    
    virtual void free(void* /*ptr*/)
    {
        // Empty
    }
};

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
    const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
    const int gridSize = gridWidth * gridHeight;
    return headerSize + gridSize*4;
}

struct TileCacheData
{
    unsigned char* data;
    int dataSize;
};

struct RasterizationContext
{
    RasterizationContext() :
        solid(0),
        triareas(0),
        lset(0),
        chf(0),
        ntiles(0)
    {
        memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
    }
    
    ~RasterizationContext()
    {
        rcFreeHeightField(solid);
        delete [] triareas;
        rcFreeHeightfieldLayerSet(lset);
        rcFreeCompactHeightfield(chf);
        for (int i = 0; i < MAX_LAYERS; ++i)
        {
            dtFree(tiles[i].data);
            tiles[i].data = 0;
        }
    }
    
    rcHeightfield* solid;
    unsigned char* triareas;
    rcHeightfieldLayerSet* lset;
    rcCompactHeightfield* chf;
    TileCacheData tiles[MAX_LAYERS];
    int ntiles;
};

static int rasterizeTileLayers(BuildContext* ctx, InputGeom* geom,
                               const int tx, const int ty,
                               const rcConfig& cfg,
                               TileCacheData* tiles,
                               const int maxTiles)
{
    if (!geom || !geom->getMesh() || !geom->getChunkyMesh())
    {
        ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
        return 0;
    }
    
    FastLZCompressor comp;
    RasterizationContext rc;
    
    const float* verts = geom->getMesh()->getVerts();
    const int nverts = geom->getMesh()->getVertCount();
    const rcChunkyTriMesh* chunkyMesh = geom->getChunkyMesh();
    
    // Tile bounds.
    const float tcs = cfg.tileSize * cfg.cs;
    
    rcConfig tcfg;
    memcpy(&tcfg, &cfg, sizeof(tcfg));

    tcfg.bmin[0] = cfg.bmin[0] + tx*tcs;
    tcfg.bmin[1] = cfg.bmin[1];
    tcfg.bmin[2] = cfg.bmin[2] + ty*tcs;
    tcfg.bmax[0] = cfg.bmin[0] + (tx+1)*tcs;
    tcfg.bmax[1] = cfg.bmax[1];
    tcfg.bmax[2] = cfg.bmin[2] + (ty+1)*tcs;
    tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
    tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;
    
    // Allocate voxel heightfield where we rasterize our input data to.
    rc.solid = rcAllocHeightfield();
    if (!rc.solid)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return 0;
    }
    if (!rcCreateHeightfield(ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
        return 0;
    }
    
    // Allocate array that can hold triangle flags.
    // If you have multiple meshes you need to process, allocate
    // and array which can hold the max number of triangles you need to process.
    rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
    if (!rc.triareas)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
        return 0;
    }
    
    float tbmin[2], tbmax[2];
    tbmin[0] = tcfg.bmin[0];
    tbmin[1] = tcfg.bmin[2];
    tbmax[0] = tcfg.bmax[0];
    tbmax[1] = tcfg.bmax[2];
    int cid[512];// TODO: Make grow when returning too many items.
    const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
    if (!ncid)
    {
        return 0; // empty
    }
    
    for (int i = 0; i < ncid; ++i)
    {
        const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
        const int* tris = &chunkyMesh->tris[node.i*3];
        const int ntris = node.n;
        
        memset(rc.triareas, 0, ntris*sizeof(unsigned char));
        rcMarkWalkableTriangles(ctx, tcfg.walkableSlopeAngle,
                                verts, nverts, tris, ntris, rc.triareas);
        
        rcRasterizeTriangles(ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb);
    }
    
    // Once all geometry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    rcFilterLowHangingWalkableObstacles(ctx, tcfg.walkableClimb, *rc.solid);
    rcFilterLedgeSpans(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
    rcFilterWalkableLowHeightSpans(ctx, tcfg.walkableHeight, *rc.solid);
    
    
    rc.chf = rcAllocCompactHeightfield();
    if (!rc.chf)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return 0;
    }
    if (!rcBuildCompactHeightfield(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
        return 0;
    }
    
    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(ctx, tcfg.walkableRadius, *rc.chf))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return 0;
    }
    
    // (Optional) Mark areas.
    const ConvexVolume* vols = geom->getConvexVolumes();
    for (int i  = 0; i < geom->getConvexVolumeCount(); ++i)
    {
        rcMarkConvexPolyArea(ctx, vols[i].verts, vols[i].nverts,
                             vols[i].hmin, vols[i].hmax,
                             (unsigned char)vols[i].area, *rc.chf);
    }
    
    rc.lset = rcAllocHeightfieldLayerSet();
    if (!rc.lset)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
        return 0;
    }
    if (!rcBuildHeightfieldLayers(ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
        return 0;
    }
    
    rc.ntiles = 0;
    for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
    {
        TileCacheData* tile = &rc.tiles[rc.ntiles++];
        const rcHeightfieldLayer* layer = &rc.lset->layers[i];
        
        // Store header
        dtTileCacheLayerHeader header;
        header.magic = DT_TILECACHE_MAGIC;
        header.version = DT_TILECACHE_VERSION;
        
        // Tile layer location in the navmesh.
        header.tx = tx;
        header.ty = ty;
        header.tlayer = i;
        dtVcopy(header.bmin, layer->bmin);
        dtVcopy(header.bmax, layer->bmax);
        
        // Tile info.
        header.width = (unsigned char)layer->width;
        header.height = (unsigned char)layer->height;
        header.minx = (unsigned char)layer->minx;
        header.maxx = (unsigned char)layer->maxx;
        header.miny = (unsigned char)layer->miny;
        header.maxy = (unsigned char)layer->maxy;
        header.hmin = (unsigned short)layer->hmin;
        header.hmax = (unsigned short)layer->hmax;

        dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
                                                &tile->data, &tile->dataSize);
        if (dtStatusFailed(status))
        {
            return 0;
        }
    }

    // Transfer ownsership of tile data from build context to the caller.
    int n = 0;
    for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
    {
        tiles[n++] = rc.tiles[i];
        rc.tiles[i].data = 0;
        rc.tiles[i].dataSize = 0;
    }
    
    return n;
}

static bool isectSegAABB(const float* sp, const float* sq,
                         const float* amin, const float* amax,
                         float& tmin, float& tmax)
{
    static const float EPS = 1e-6f;
    
    float d[3];
    rcVsub(d, sq, sp);
    tmin = 0;  // set to -FLT_MAX to get first hit on line
    tmax = FLT_MAX;     // set to max distance ray can travel (for segment)
    
    // For all three slabs
    for (int i = 0; i < 3; i++)
    {
        if (fabsf(d[i]) < EPS)
        {
            // Ray is parallel to slab. No hit if origin not within slab
            if (sp[i] < amin[i] || sp[i] > amax[i])
                return false;
        }
        else
        {
            // Compute intersection t value of ray with near and far plane of slab
            const float ood = 1.0f / d[i];
            float t1 = (amin[i] - sp[i]) * ood;
            float t2 = (amax[i] - sp[i]) * ood;
            // Make t1 be intersection with near plane, t2 with far plane
            if (t1 > t2) rcSwap(t1, t2);
            // Compute the intersection of slab intersections intervals
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            // Exit with no collision as soon as slab intersection becomes empty
            if (tmin > tmax) return false;
        }
    }
    
    return true;
}

dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq)
{
    float tmin = FLT_MAX;
    const dtTileCacheObstacle* obmin = 0;
    for (int i = 0; i < tc->getObstacleCount(); ++i)
    {
        const dtTileCacheObstacle* ob = tc->getObstacle(i);
        if (ob->state == DT_OBSTACLE_EMPTY)
            continue;
        
        float bmin[3], bmax[3], t0,t1;
        tc->getObstacleBounds(ob, bmin,bmax);
        
        if (isectSegAABB(sp,sq, bmin,bmax, t0,t1))
        {
            if (t0 < tmin)
            {
                tmin = t0;
                obmin = ob;
            }
        }
    }
    return tc->getObstacleRef(obmin);
}

////////////////////////////
// CUSTOM
//////////////
rcMeshLoaderObj* meshLoader;

////////////////////////////
// FROM SAMPLE
//////////////
InputGeom* m_geom;
dtNavMesh* m_navMesh;
dtNavMeshQuery* m_navQuery;
dtCrowd* m_crowd;

dtTileCache* m_tileCache;
float m_tileSize;
int m_cacheCompressedSize;
int m_cacheRawSize;
int m_cacheLayerCount;
int m_cacheBuildMemUsage;
LinearAllocator* m_talloc = new LinearAllocator(32000);
FastLZCompressor* m_tcomp = new FastLZCompressor;
MeshProcess* m_tmproc = new MeshProcess;

// m_talloc = new LinearAllocator(32000);
// m_tcomp  = new FastLZCompressor;
// m_tmproc = new MeshProcess;

unsigned char m_navMeshDrawFlags;

bool m_keepInterResults = false;
float m_totalBuildTimeMs;

int randModulo = 0;

//// DEFAULTS
float m_agentHeight = 2.0f;  // , 5.0f, 0.1f);
float m_agentRadius = 0.5f;  // , 5.0f, 0.1f);

float m_cellSize = m_agentRadius / 2;
float m_cellHeight = m_cellSize / 2;

float m_agentMaxClimb = 4.0f;  // , 5.0f, 0.1f);
float m_agentMaxSlope = 30.0f;  // , 90.0f, 1.0f);

float m_regionMinSize = 1.0f;  // , 150.0f, 1.0f);
float m_regionMergeSize = 1.0f;  // , 150.0f, 1.0f);
bool m_monotonePartitioning = 0;

float m_edgeMaxLen = 50.0f;  // , 50.0f, 1.0f);
float m_edgeMaxError = 1.0f;  // , 3.0f, 0.1f);
float m_vertsPerPoly = 3.0f;  // , 12.0f, 1.0f);        

float m_detailSampleDist = 0.0f;  // , 16.0f, 1.0f);
float m_detailSampleMaxError = 8.0f;  // , 16.0f, 1.0f);

unsigned char* m_triareas;
rcHeightfield* m_solid;
rcCompactHeightfield* m_chf;
rcContourSet* m_cset;
rcPolyMesh* m_pmesh;
rcConfig m_cfg; 
rcPolyMeshDetail* m_dmesh;

BuildContext* m_ctx;

DebugDrawGL* dd;

/////////////////////////////////

void debugConfig()
{
    printf("config \n");
    printf(" m_cellSize=%f \n", m_cellSize);
    printf(" m_cellHeight=%f \n", m_cellHeight);

    //// DEFAULTS
    printf(" m_agentHeight=%f \n", m_agentHeight);
    printf(" m_agentRadius=%f \n", m_agentRadius);
    printf(" m_agentMaxClimb=%f \n", m_agentMaxClimb);
    printf(" m_agentMaxSlope=%f \n", m_agentMaxSlope);

    printf(" m_monotonePartitioning=%u\n", m_monotonePartitioning);

    printf(" m_regionMinSize=%f \n", m_regionMinSize);
    printf(" m_regionMergeSize=%f \n", m_regionMergeSize);

    printf(" m_edgeMaxLen=%f \n", m_edgeMaxLen);
    printf(" m_edgeMaxError=%f \n", m_edgeMaxError);
    printf(" m_vertsPerPoly=%f \n", m_vertsPerPoly);        

    printf(" m_detailSampleDist=%f \n", m_detailSampleDist);
    printf(" m_detailSampleMaxError=%f \n", m_detailSampleMaxError);
}

void dumpConfig()
{
    char buff[1024];

    sprintf(buff, "   m_cellSize=%f ", m_cellSize);
    sprintf(buff, "%s m_cellHeight=%f ", buff, m_cellHeight);

    //// DEFAULTS
    sprintf(buff, "%s m_agentHeight=%f ", buff, m_agentHeight);
    sprintf(buff, "%s m_agentRadius=%f ", buff, m_agentRadius);
    sprintf(buff, "%s m_agentMaxClimb=%f ", buff, m_agentMaxClimb);
    sprintf(buff, "%s m_agentMaxSlope=%f ", buff, m_agentMaxSlope);

    sprintf(buff, "%s m_monotonePartitioning=%u ", buff, m_monotonePartitioning);

    sprintf(buff, "%s m_regionMinSize=%f ", buff, m_regionMinSize);
    sprintf(buff, "%s m_regionMergeSize=%f ", buff, m_regionMergeSize);

    sprintf(buff, "%s m_edgeMaxLen=%f ", buff, m_edgeMaxLen);
    sprintf(buff, "%s m_edgeMaxError=%f ", buff, m_edgeMaxError);
    sprintf(buff, "%s m_vertsPerPoly=%f ", buff, m_vertsPerPoly);        

    sprintf(buff, "%s m_detailSampleDist=%f ", buff, m_detailSampleDist);
    sprintf(buff, "%s m_detailSampleMaxError=%f ", buff, m_detailSampleMaxError);

    sprintf(buff, "console.log('%s');", buff);
    emscripten_run_script(buff);
}
void debugCreateNavMesh(unsigned char flags) {
    gl_create_object("NavMesh");
    duDebugDrawNavMesh(dd, *m_navMesh, SAMPLE_POLYAREA_GROUND);
}
void debugCreateNavMeshPortals() {
    gl_create_object("NavMeshPortals");
    duDebugDrawNavMeshPortals(dd, *m_navMesh);
}
void debugCreateRegionConnections() {
    gl_create_object("RegionConnections");
    duDebugDrawRegionConnections(dd, *m_cset, 0.5f);
}
void debugCreateRawContours() {
    gl_create_object("RawContours");
    duDebugDrawRawContours(dd, *m_cset, 0.5f);
}
void debugCreateContours() {
    gl_create_object("Contours");
    duDebugDrawContours(dd, *m_cset, 0.5f);
}
void debugCreateHeightfieldSolid() {
    gl_create_object("HeightfieldSolid");
    duDebugDrawHeightfieldSolid(dd, *m_solid);
}
void debugCreateHeightfieldWalkable() {
    gl_create_object("HeightfieldWalkable");
    duDebugDrawHeightfieldWalkable(dd, *m_solid);
}
void debugOffMeshConnections() {
    gl_create_object("HeightfieldWalkable");
    m_geom->drawOffMeshConnections(dd, true);
}

////////////////////////////

void emscripten_log(const char* string, bool escape = true)
{
    char buff[1024];
    sprintf(buff, (escape ? "console.log('%s');" : "console.log(%s);"), string);
    emscripten_run_script(buff);
    // free(buff);
}
void emscripten_debugger()
{
    emscripten_run_script("debugger");
}

////////////////////////////

void cleanup()
{
    printf("cleanup \n");
    
    delete [] m_triareas;
    m_triareas = 0;
    rcFreeHeightField(m_solid);
    m_solid = 0;
    rcFreeCompactHeightfield(m_chf);
    m_chf = 0;
    rcFreeContourSet(m_cset);
    m_cset = 0;
    rcFreePolyMesh(m_pmesh);
    m_pmesh = 0;
    rcFreePolyMeshDetail(m_dmesh);
    m_dmesh = 0;
    dtFreeNavMesh(m_navMesh);
    m_navMesh = 0;
    dtFreeTileCache(m_tileCache);
    m_tileCache = 0;
    m_tileSize = 48;
    //dtNavMeshQuery(m_navQuery);
    m_navQuery = 0;
}


void getNavMeshVertices(int callback){
    const int nvp = m_pmesh->nvp;
    const float cs = m_pmesh->cs;
    const float ch = m_pmesh->ch;
    const float* orig = m_pmesh->bmin;

    char buff[m_pmesh->npolys * 1000];

    sprintf(buff, "");

    for (int i = 0; i < m_pmesh->npolys; ++i)
    {
        if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND)
        {
            const unsigned short* p = &m_pmesh->polys[i*nvp*2];

            unsigned int color;
            if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
                color = duRGBA(0,192,255,64);
            else if (m_pmesh->areas[i] == RC_NULL_AREA)
                color = duRGBA(0,0,0,64);
            else
                color = duIntToCol(m_pmesh->areas[i], 255);

            unsigned short vi[3];
            for (int j = 2; j < nvp; ++j)
            {
                if (p[j] == RC_MESH_NULL_IDX) break;
                vi[0] = p[0];
                vi[1] = p[j-1];
                vi[2] = p[j];

                for (int k = 0; k < 3; ++k)
                {
                    const unsigned short* v = &m_pmesh->verts[vi[k]*3];
                    const float x = orig[0] + v[0]*cs;
                    const float y = orig[1] + (v[1]+1)*ch;
                    const float z = orig[2] + v[2]*cs;

                    sprintf(buff, "%s{\"x\":%f,\"y\":%f,\"z\":%f,\"col\":%u}%s", buff, x, y, z, color, (i == m_pmesh->npolys - 1 && k == 2 ? "" : ","));
                }
            }
        }
    }

    sprintf(buff, "[%s]", buff);

    invoke_generic_callback_string(callback, buff);
}


void getNavHeightfieldRegions(int callback)
{
    if (! m_chf) {
        printf("CompactHeightfield (m_chf) is not available \n");
        return;
    }

    std::string data;

    const float cs = m_chf->cs;
    const float ch = m_chf->ch;

    int height = m_chf->height;
    int width = m_chf->width;

    char buff[height * width * 1000];
    sprintf(buff, "");

    data = "[";
    // ((y == height - 1) && (x == width - 1) && (i == ni - 1) ? "" : ","

    for (int y = 0; y < height; ++y)
    {
        data += "[";

        for (int x = 0; x < width; ++x)
        {
            const float fx = m_chf->bmin[0] + x*cs;
            const float fz = m_chf->bmin[2] + y*cs;
            const rcCompactCell& c = m_chf->cells[x+y*m_chf->width];

            data += "[";
            
            for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
            {
                const rcCompactSpan& s = m_chf->spans[i];
                const float fy = m_chf->bmin[1] + (s.y)*ch;
                unsigned int color;
                if (s.reg)
                    color = duIntToCol(s.reg, 192);
                else
                    color = duRGBA(0,0,0,64);

                char localData[512];

                data += "[";

                sprintf(localData, "{\"x\":%f,\"y\":%f,\"z\":%f,\"col\":%u},",  fx,    fy, fz   , color);
                data = data + localData;

                sprintf(localData, "{\"x\":%f,\"y\":%f,\"z\":%f,\"col\":%u},",  fx,    fy, fz+cs, color);
                data = data + localData;

                sprintf(localData, "{\"x\":%f,\"y\":%f,\"z\":%f,\"col\":%u},",  fx+cs, fy, fz+cs, color);
                data = data + localData;

                sprintf(localData, "{\"x\":%f,\"y\":%f,\"z\":%f,\"col\":%u}",   fx+cs, fy, fz   , color);
                data = data + localData;

                data += "]";
            }
            data += "]";
        }
        data += "]";
    }
    data += "]";

    // char buff2[512];
    // sprintf(buff2, "cs=%f, ch=%f, height=%u, width=%u, len=%u", cs, ch, height, width, strlen(buff));
    // emscripten_log(buff2);
    
    // sprintf(buff, "[%s]", buff);

    invoke_generic_callback_string(callback, data.c_str());

    // free(buff);
}

void getNavMeshTiles(int callback){
}

// http://stackoverflow.com/questions/686353/c-random-float-number-generation
float randZeroToOne()
{
    randModulo += 1;

    srand (time(NULL) % randModulo);
    return rand() / (RAND_MAX + 1.);
}

void getRandomPoint(int callback)
{
    char buff[512];

    dtQueryFilter filter;
    filter.setIncludeFlags(3);
    filter.setExcludeFlags(0);

    dtPolyRef ref = 0;

    float randomPt[3];

    dtStatus status = m_navQuery->findRandomPoint(&filter, randZeroToOne, &ref, randomPt);

    if (dtStatusFailed(status)) {
        printf("Cannot find a random point: %u\n", status);

       invoke_vector_callback(callback, NULL, NULL, NULL);

    } else {

        invoke_vector_callback(callback, randomPt[0], randomPt[1], randomPt[2]);
    }

    // free(buff);
}

std::string recastjsPolyJSON(const dtPoly* poly, const dtMeshTile* tile, dtPolyRef ref)
{
    std::string data;
    data = "{";

    char successbuff[128 * poly->vertCount];

    sprintf(successbuff, "\"ref\":%u,\"x\":%u,\"y\":%u,\"layer\":%u,\"flags\":%u,\"area\":%u,", ref, tile->header->x, tile->header->y, tile->header->layer, poly->flags, poly->getArea());
    data += successbuff;

    // float* centroid;
    // dtCalcPolyCenter(centroid, poly->verts, poly->vertCount, tile->verts);
    // sprintf(successbuff, "\"centroid\":{\"x\":%f,\"y\":%f,\"z\":%f},", centroid[0], centroid[1], centroid[2]);
    // data += successbuff;

    data += "\"vertices\":[";
        
    for (int i = 0; i < (int)poly->vertCount; i++) {
        float* v = &tile->verts[poly->verts[i]*3];

        sprintf(successbuff, "{\"x\":%f,\"y\":%f,\"z\":%f}%s", v[0], v[1], v[2], (i == poly->vertCount - 1) ? "" : ",");
        data += successbuff;
    }

    data += "]}";

    return data;
}

void findNearestPoly(float cx, float cy, float cz,
                    float ex, float ey, float ez,
                     /*const dtQueryFilter* filter,
                     dtPolyRef* nearestRef, float* nearestPt*/
                    int callback)
{
    const float p[3] = {cx,cy,cz};
    const float ext[3] = {ex,ey,ez};
    float nearestPt[3];
    char buff[64];

    dtQueryFilter filter;
    filter.setIncludeFlags(3);
    filter.setExcludeFlags(0);

    dtPolyRef ref = 0;

    dtStatus status = m_navQuery->findNearestPoly(p, ext, &filter, &ref, 0);

    if (dtStatusFailed(status) || ref == 0) {
        printf("Cannot find nearestPoly: %u\n", status);

    } else {

        const dtMeshTile* tile = 0;
        const dtPoly* poly = 0;
        m_navMesh->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

        std::string data = recastjsPolyJSON(poly, tile, ref);

        invoke_generic_callback_string(callback, data.c_str());
        return;
    }

    invoke_generic_callback_string(callback, "null");
}

void findNearestPoint(float cx, float cy, float cz,
                    float ex, float ey, float ez,
                     /*const dtQueryFilter* filter,
                     dtPolyRef* nearestRef, float* nearestPt*/
                    int callback)
{
    const float p[3] = {cx,cy,cz};
    const float ext[3] = {ex,ey,ez};
    float nearestPt[3];

    dtQueryFilter filter;
    filter.setIncludeFlags(3);
    filter.setExcludeFlags(0);

    dtPolyRef ref = 0;
    float nearestPos[3];

    dtStatus status = m_navQuery->findNearestPoly(p, ext, &filter, &ref, nearestPos);

    if (dtStatusFailed(status)) {
        invoke_vector_callback(callback, NULL, NULL, NULL);

    } else {

        invoke_vector_callback(callback, nearestPos[0], nearestPos[1], nearestPos[2]);
    }
}

void setPolyFlagsByRef(int ref, unsigned short flags)
{
    char buff[512];
    dtStatus status;
    status = m_navMesh->setPolyFlags((dtPolyRef)ref, flags);

    if (dtStatusFailed(status)) {
        sprintf(buff, "cannot set flag %u on %u", flags, ref);
        emscripten_log(buff);
    } else {
        sprintf(buff, "found poly %u set flags %u ", ref, flags);
        emscripten_log(buff);        
    }
}

void setPolyFlags(float posX, float posY, float posZ, float extendX, float extendY, float extendZ, unsigned short flags)
{
    dtQueryFilter filter;
    filter.setIncludeFlags(3);
    filter.setExcludeFlags(0);

    char buff[512];

    const float ext[3] = {extendX, extendY, extendZ};
    float startPos[3] = { posX, posY, posZ };
    float nearestPos[3];
    dtPolyRef ref = 0;

    dtStatus status;

    status = m_navQuery->findNearestPoly(startPos, ext, &filter, &ref, nearestPos);

    if (dtStatusFailed(status)) {
        sprintf(buff, "Cannot find a poly near: %f, %f, %f ", posX, posY, posZ);
        emscripten_log(buff);

    } else {
        setPolyFlagsByRef((int)ref, flags);
    }
}

void addOffMeshConnection(float startX, float startY, float startZ,
                          float endX, float endY, float endZ,
                          const float radius, unsigned char bidir
                          /* , unsigned char area, unsigned short flags */ )
{
    const unsigned char area = SAMPLE_POLYAREA_JUMP;
    const unsigned short flags = SAMPLE_POLYFLAGS_JUMP; 

    const float spos[3] = { startX, startY, startZ };
    const float epos[3] = { endX, endY, endZ };

    m_geom->addOffMeshConnection(spos, epos, radius, bidir ? 1 : 0, area, flags);
}

void _queryPolygons(float posX, float posY, float posZ,
                    float extX, float extY, float extZ,
                    const int maxPolys, int callback)
{
    float center[3]  = { posX, posY, posZ };
    float extents[3] = { extX, extY, extZ };

    dtStatus status;

    dtPolyRef polys[maxPolys];
    int polyCount;

    dtQueryFilter filter;
    filter.setIncludeFlags(3);
    filter.setExcludeFlags(0);

    status = m_navQuery->queryPolygons(center, extents, &filter, polys, &polyCount, maxPolys);

    std::string data;
    data = "[";

    if (dtStatusFailed(status)) {
        printf("Cannot query polygons: %u\n", status);

    } else {

        for (int p = 0; p < (int)polyCount; p++) {

            const dtMeshTile* tile = 0;
            const dtPoly* poly = 0;
            m_navMesh->getTileAndPolyByRefUnsafe(polys[p], &tile, &poly);
            data += recastjsPolyJSON(poly, tile, polys[p]) + (p == polyCount - 1 ? "" : ",");
        }

        data += "]";

        invoke_generic_callback_string(callback, data.c_str());
        return;
    }

    invoke_generic_callback_string(callback, "null");
}

void findPath(float startPosX, float startPosY, float startPosZ,
                float endPosX, float endPosY, float endPosZ, int maxPath,
                int callback)
{
    emscripten_run_script("__tmp_recastjs_data = [];");
    char buff[512];

    float startPos[3] = { startPosX, startPosY, startPosZ };
    float endPos[3] = { endPosX, endPosY, endPosZ };

    const float ext[3] = {2,4,2};

    dtStatus status;

    dtPolyRef path[maxPath+1];
    int pathCount;

    dtQueryFilter filter;
    filter.setIncludeFlags(3);
    filter.setExcludeFlags(0);

    // Change costs.
    filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);

    float nearestStartPos[3];
    dtPolyRef startRef = 0;
    m_navQuery->findNearestPoly(startPos, ext, &filter, &startRef, nearestStartPos);

    float nearestEndPos[3];
    dtPolyRef endRef = 0;
    m_navQuery->findNearestPoly(endPos, ext, &filter, &endRef, nearestEndPos);

    printf("Use %u , %u as start / end polyRefs \n", startRef, endRef);

    status = m_navQuery->findPath(startRef, endRef, nearestStartPos, nearestEndPos, &filter, path, &pathCount, maxPath);

    if (dtStatusFailed(status)) {
        printf("Cannot find a path: %u\n", status);

    } else {
        printf("Found a %u polysteps path \n", pathCount);

        float straightPath[maxPath*3];
        unsigned char straightPathFlags[maxPath];
        dtPolyRef straightPathRefs[maxPath];
        int straightPathCount = 0;

        int maxStraightPath = maxPath;
        int options = 0;

        status = m_navQuery->findStraightPath(nearestStartPos, nearestEndPos, path, pathCount, straightPath,
                                    straightPathFlags, straightPathRefs, &straightPathCount, maxStraightPath, options);

        if (dtStatusFailed(status)) {
            printf("Cannot find a straight path: %u\n", status);

        } else {
            printf("Found a %u steps path \n", straightPathCount);

            for (int i = 0; i < straightPathCount; ++i) {
                const float* v = &straightPath[i*3];

                // why ?
                if (!(fabs(v[0]) < 0.0000001f && fabs(v[1]) < 0.0000001f && fabs(v[2]) < 0.0000001f)) {
                    sprintf(buff, "__tmp_recastjs_data.push({x:%f, y:%f, z:%f});", v[0], v[1], v[2]);
                    emscripten_run_script(buff);
                } else {
                    sprintf(buff, "ignore %f, %f, %f", v[0], v[1], v[2]);
                    emscripten_log(buff);                   
                }
            }
        }
    }

    sprintf(buff, "Module.__RECAST_CALLBACKS[%u](__tmp_recastjs_data);", callback);
    emscripten_run_script(buff);

    // free(buff);
}

void set_cellSize(float val){               m_cellSize = val;               }
void set_cellHeight(float val){             m_cellHeight = val;             }
void set_agentHeight(float val){            m_agentHeight = val;            }
void set_agentRadius(float val){            m_agentRadius = val;            }
void set_agentMaxClimb(float val){          m_agentMaxClimb = val;          }
void set_agentMaxSlope(float val){          m_agentMaxSlope = val;          }
void set_regionMinSize(float val){          m_regionMinSize = val;          }
void set_regionMergeSize(float val){        m_regionMergeSize = val;        }
void set_edgeMaxLen(float val){             m_edgeMaxLen = val;             }
void set_edgeMaxError(float val){           m_edgeMaxError = val;           }
void set_vertsPerPoly(float val){           m_vertsPerPoly = val;           }       
void set_detailSampleDist(float val){       m_detailSampleDist = val;       }
void set_detailSampleMaxError(float val){   m_detailSampleMaxError = val;   }
void set_monotonePartitioning(int val){     m_monotonePartitioning = !!val; }

/////////////////////////////

bool initWithFile(std::string filename)
{
    printf("loading from file");
    m_geom = new InputGeom;
    if (!m_geom || !m_geom->loadMesh(m_ctx, filename.c_str()))
    {
        printf("cannot load OBJ file \n");
        return false;       
    }
    return true;
}

bool initWithFileContent(std::string contents)
{
    printf("loading from contents \n");
    // printf(contents.c_str());

    m_geom = new InputGeom;
    if (!m_geom || !m_geom->loadMeshFromContents(m_ctx, contents.c_str()))
    {
        printf("cannot load OBJ contents \n");
        return false;       
    }
    return true;
}

bool initCrowd(const int maxAgents, const float maxAgentRadius)
{
    m_crowd->init(maxAgents, maxAgentRadius, m_navMesh);

    return true;
}

struct agentUserData {
    int idx;
};

void updateCrowdAgentParameters(const int idx, float posX, float posY, float posZ, float radius, float height, 
                                                                float maxAcceleration, float maxSpeed, unsigned char updateFlags, float separationWeight)
{
    dtCrowdAgentParams ap;
    memset(&ap, 0, sizeof(ap));
    ap.radius = radius;
    ap.height = height;
    ap.maxAcceleration = maxAcceleration;
    ap.maxSpeed = maxSpeed;
    ap.collisionQueryRange = ap.radius * 12.0f;
    ap.pathOptimizationRange = ap.radius * 300.0f;
    ap.updateFlags = updateFlags; 
    // if (m_toolParams.m_anticipateTurns)
    //  ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
    // if (m_toolParams.m_optimizeVis)
    //  ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
    // if (m_toolParams.m_optimizeTopo)
    //  ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
    // if (m_toolParams.m_obstacleAvoidance)
    //  ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
    // if (m_toolParams.m_separation)
    //  ap.updateFlags |= DT_CROWD_SEPARATION;
    ap.obstacleAvoidanceType = 3.0;
    ap.separationWeight = separationWeight;
    ap.userData = (void *)idx;

    float pos[3] = { posX, posY, posZ };

    m_crowd->updateAgentParameters(idx, &ap);
}

int addCrowdAgent(float posX, float posY, float posZ, float radius, float height, 
                                    float maxAcceleration, float maxSpeed, unsigned char updateFlags, float separationWeight)
{
    dtCrowdAgentParams ap;
    memset(&ap, 0, sizeof(ap));
    ap.radius = radius;
    ap.height = height;
    ap.maxAcceleration = maxAcceleration;
    ap.maxSpeed = maxSpeed;
    ap.collisionQueryRange = ap.radius * 5.0f;
    ap.pathOptimizationRange = ap.radius * 30.0f;
    ap.updateFlags = updateFlags;
    // if (m_toolParams.m_anticipateTurns)
    //  ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
    // if (m_toolParams.m_optimizeVis)
    //  ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
    // if (m_toolParams.m_optimizeTopo)
    //  ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
    // if (m_toolParams.m_obstacleAvoidance)
    //  ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
    // if (m_toolParams.m_separation)
    //  ap.updateFlags |= DT_CROWD_SEPARATION;
    ap.obstacleAvoidanceType = 3.0;
    ap.separationWeight = separationWeight;

    float pos[3] = { posX, posY, posZ };

    int idx = m_crowd->addAgent(pos, &ap);

    // agentUserData data;
    // memset(&data, 0, sizeof(data));
    // data.idx = 99;
    // ap.userData = (void *) &data;

    ap.userData = (void *)idx; /* FIXME: doesnt fucking work ://// */
    /* So we do this ?? */
    updateCrowdAgentParameters(idx, posX, posY, posZ, radius, height, maxAcceleration, maxSpeed, updateFlags, separationWeight);

    // char buff[512];
    // const dtCrowdAgent* ag = m_crowd->getAgent(idx);
    // const float* p = ag->npos;
    // const float r = ag->params.radius;
    // sprintf(buff, "debug('new agent', { idx:%d });", idx);
    // emscripten_run_script(buff);

    return idx;
}

void removeCrowdAgent(int idx)
{
    m_crowd->removeAgent(idx);
}

void requestMoveVelocity(int agentIdx, float velX, float velY, float velZ)
{
    float vel[3] = { velX, velY, velZ };
    m_crowd->requestMoveVelocity(agentIdx, vel);
}

bool crowdRequestMoveTarget(int agentIdx, float posX, float posY, float posZ)
{
    char buff[512];

    float pos[3] = { posX, posY, posZ };
    const float ext[3] = {2,4,2};

    dtPolyRef m_targetRef = 0;
    float m_targetPos[3];

    dtQueryFilter filter;
    filter.setIncludeFlags(SAMPLE_POLYFLAGS_JUMP | SAMPLE_POLYFLAGS_WALK);
    filter.setExcludeFlags(0);

    dtStatus status = m_navQuery->findNearestPoly(pos, ext, &filter, &m_targetRef, m_targetPos);
    if (dtStatusFailed(status)) {
        // emscripten_run_script("debug('Cannot find a poly near specified position');");
        return false;
    } else {
        // emscripten_run_script("debug('MoveTarget adjusted');");
    }

    m_crowd->requestMoveTarget(agentIdx, m_targetRef, m_targetPos); 

    return true;
}

bool crowdUpdate(float dt)
{
    dtCrowdAgentDebugInfo* m_agentDebug;

    memset(&m_agentDebug, 0, sizeof(m_agentDebug));

    m_crowd->update(dt, m_agentDebug);

    if (m_tileCache) {
        m_tileCache->update(dt, m_navMesh);
    }

    return true;
}

bool _crowdGetActiveAgents(int callback_id)
{
    int maxAgents = 1000;

    dtCrowdAgent** agents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*)*maxAgents, DT_ALLOC_PERM);
    int nagents = m_crowd->getActiveAgents(agents, maxAgents);

    for (int i = 0; i < nagents; i++) {
        dtCrowdAgent* ag = agents[i];
        const float* p = ag->npos;
        const float* v = ag->vel;
        const float r = ag->params.radius;
        int idx = (int) ag->params.userData;
        agentPool_get(idx, p[0], p[1], p[2], v[0], v[1], v[2], r, ag->active, ag->state, ag->nneis, ag->partial, ag->desiredSpeed);
    }

    // we have a specific callback to call
    if (callback_id != -1) {
       invoke_update_callback(callback_id);

    // or emit the update event
    } else {
        flush_active_agents_callback();
    }

    // free some pool slots
    for (int i = 0; i < nagents; i++) {
        agentPool_add(i);
    }
    agentPool_clear();

    return true;
}

void addTempObstacle(const float posX, const float posY, const float posZ, const float radius)
{
    if (!m_tileCache) {
        emscripten_log("tile cache is not ready");
        return;
    }
    float p[3] = { posX, posY, posZ };
    // p[1] -= 0.5f;

    dtObstacleRef* ref;
    int status = m_tileCache->addObstacle(p, radius, 2.0f, ref);

    if (dtStatusFailed(status)) {
        char buff[64];
        sprintf(buff, "cant add an obstacle: %u", status);
        emscripten_log(buff);
    }
}

void removeTempObstacle(const float spX, const float spY, const float spZ,
                        const float sqX, const float sqY, const float sqZ)
{
    if (!m_tileCache) {
        emscripten_log("tile cache is not ready");
        return;
    }
    float sp[3] = { spX, spY, spZ };
    float sq[3] = { sqX, sqY, sqZ };
    dtObstacleRef ref = hitTestObstacle(m_tileCache, sp, sq);
    m_tileCache->removeObstacle(ref);
}

void getAllTempObstacles(int callback_id)
{
    if (!m_tileCache) {
        emscripten_log("tile cache is not ready");
        return;
    }

    std::string data = "[";
    char buff[512];

    int obstacleCount = m_tileCache->getObstacleCount();
    for (int i = 0; i < obstacleCount; ++i)
    {
        const dtTileCacheObstacle* ob = m_tileCache->getObstacle(i);
        if (ob->state == DT_OBSTACLE_EMPTY) continue;

        sprintf(buff, "{ \"position\": {\"x\":%f, \"y\":%f, \"z\":%f}, \"radius\":%f, \"height\":%f, \"state\":%u }", ob->pos[0], ob->pos[1], ob->pos[2], ob->radius, ob->height, ob->state);

        data += buff;
        data += (i == obstacleCount - 1 ? "" : ",");
    }

    data += "]";

    invoke_generic_callback_string(callback_id, data.c_str());
}

void clearAllTempObstacles()
{
    if (!m_tileCache) {
        emscripten_log("tile cache is not ready");
        return;
    }
    for (int i = 0; i < m_tileCache->getObstacleCount(); ++i)
    {
        const dtTileCacheObstacle* ob = m_tileCache->getObstacle(i);
        if (ob->state == DT_OBSTACLE_EMPTY) continue;
        m_tileCache->removeObstacle(m_tileCache->getObstacleRef(ob));
    }
}
bool build()
{
    dtStatus status;
        
    char buff[1024];

    dd = new DebugDrawGL;

    if (!m_geom || !m_geom->getMesh())
    {
        printf("buildNavigation: Input mesh is not specified.");
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
        return false;
    }
    
    cleanup();

    const float* bmin  = m_geom->getMeshBoundsMin();
    const float* bmax  = m_geom->getMeshBoundsMax();
    const float* verts = m_geom->getMesh()->getVerts();
    const int nverts   = m_geom->getMesh()->getVertCount();
    const int* tris    = m_geom->getMesh()->getTris();
    const int ntris    = m_geom->getMesh()->getTriCount();
    

    // Init cache
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int ts = (int)m_tileSize;
    const int tw = (gw + ts-1) / ts;
    const int th = (gh + ts-1) / ts;

    // Max tiles and max polys affect how the tile IDs are caculated.
    // There are 22 bits available for identifying a tile and a polygon.
    int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
    if (tileBits > 14) tileBits = 14;
    int polyBits = 22 - tileBits;
    int m_maxTiles = 1 << tileBits;
    int m_maxPolysPerTile = 1 << polyBits;
    
    //
    // Step 1. Initialize build config.
    //
    
    // Init build configuration from GUI
    memset(&m_cfg, 0, sizeof(m_cfg));
    m_cfg.cs = m_cellSize;
    m_cfg.ch = m_cellHeight;
    m_cfg.walkableSlopeAngle = m_agentMaxSlope;
    m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
    m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
    m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
    m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
    m_cfg.maxSimplificationError = m_edgeMaxError;
    m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);      // Note: area = size*size
    m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);  // Note: area = size*size
    m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
    m_cfg.tileSize = (int)m_tileSize;
    m_cfg.borderSize = m_cfg.walkableRadius + 0.5; // Reserve enough padding.
    m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
    m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
    
    // Set the area where the navigation will be build.
    // Here the bounds of the input mesh are used, but the
    // area could be specified by an user defined box, etc.
    rcVcopy(m_cfg.bmin, bmin);
    rcVcopy(m_cfg.bmax, bmax);
    rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

    // Reset build times gathering.
    //m_ctx->resetTimers();

    // emscripten_log("resetTimers");

    // Start the build process. 
    //m_ctx->startTimer(RC_TIMER_TOTAL);
    
    m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
    m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
    m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts/1000.0f, ntris/1000.0f);
    
    // emscripten_log("Building navigation");

    //
    // Step 2. Rasterize input polygon soup.
    //
    
    // Allocate voxel heightfield where we rasterize our input data to.
    m_solid = rcAllocHeightfield();
    if (!m_solid)
    {
        printf("buildNavigation: Out of memory 'solid'. \n");
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return false;
    }
    if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
    {
        printf("buildNavigation: Could not create solid heightfield. \n");
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
        return false;
    }
    
    // emscripten_log("Heightfield polygon soup");

    // Allocate array that can hold triangle area types.
    // If you have multiple meshes you need to process, allocate
    // and array which can hold the max number of triangles you need to process.
    m_triareas = new unsigned char[ntris];
    if (!m_triareas)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
        return false;
    }

    // emscripten_log("rcMarkWalkableTriangles");

    // sprintf(buff, "m_ctx=%d", m_ctx);
    // emscripten_log(buff);

    // Find triangles which are walkable based on their slope and rasterize them.
    // If your input data is multiple meshes, you can transform them here, calculate
    // the are type for each of the meshes and rasterize them.
    memset(m_triareas, 0, ntris*sizeof(unsigned char));
    rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
    // printf("%u Walkable Triangles\n", sizeof(m_triareas)/sizeof(unsigned char));

    // sprintf(buff, "m_ctx=%d, WalkableTriangles=%u, verts=%d, nverts=%d, trds=%d, m_trdareas=%x, ntrds=%d, m_soldd=%d, walkableClimb=%u", m_ctx, sizeof(m_triareas)/sizeof(unsigned char), verts, nverts, tris, m_triareas, ntris, m_solid, m_cfg.walkableClimb);
    // emscripten_log(buff);
    // dumpConfig();
    
    rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb);

    if (!m_keepInterResults)
    {
        delete [] m_triareas;
        m_triareas = 0;
    }
    
    //
    // Step 3. Filter walkables surfaces.
    //
    
    // emscripten_log("rcFilterLowHangingWalkableObstacles");

    // Once all geoemtry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
    rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
    rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

    //
    // Step 4. Partition walkable surface to simple regions.
    //

    // emscripten_log("rcBuildCompactHeightfield");

    // Compact the heightfield so that it is faster to handle from now on.
    // This will result more cache coherent data as well as the neighbours
    // between walkable cells will be calculated.
    m_chf = rcAllocCompactHeightfield();
    if (!m_chf)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return false;
    }
    if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
        return false;
    }
    
    if (!m_keepInterResults)
    {
        rcFreeHeightField(m_solid);
        m_solid = 0;
    }
        
    // emscripten_log("rcErodeWalkableArea");

    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return false;
    }

    // emscripten_log("rcMarkConvexPolyArea");

    // (Optional) Mark areas.
    const ConvexVolume* vols = m_geom->getConvexVolumes();
    for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
        rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);
    
    if (m_monotonePartitioning)
    {
        // Partition the walkable surface into simple regions without holes.
        // Monotone partitioning does not need distancefield.
        if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
        {
            m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
            return false;
        }
    }
    else
    {
        // Prepare for region partitioning, by calculating distance field along the walkable surface.
        if (!rcBuildDistanceField(m_ctx, *m_chf))
        {
            m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
            return false;
        }

        // Partition the walkable surface into simple regions without holes.
        if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
        {
            m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
            return false;
        }
    }

    //
    // Step 5. Trace and simplify region contours.
    //

    // emscripten_log("rcBuildContours");
    
    // Create contours.
    m_cset = rcAllocContourSet();
    if (!m_cset)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
        return false;
    }
    if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
        return false;
    }
    
    // printf("Trace and simplify region contours: %u conts (maxSimplificationError= %f, maxEdgeLen= %u)\n", m_cset->nconts, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen);

    //
    // Step 6. Build polygons mesh from contours.
    //
    
    // Build polygon navmesh from the contours.
    m_pmesh = rcAllocPolyMesh();
    if (!m_pmesh)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
        return false;
    }
    if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
        return false;
    }
    
    // printf("Build polygons mesh from contours. \n");

    //
    // Step 7. Create detail mesh which allows to access approximate height on each polygon.
    //
    
    m_dmesh = rcAllocPolyMeshDetail();
    if (!m_dmesh)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
        return false;
    }

    if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
        return false;
    }

    if (!m_keepInterResults)
    {
        rcFreeCompactHeightfield(m_chf);
        m_chf = 0;
        rcFreeContourSet(m_cset);
        m_cset = 0;
    }

    // printf("Create detail mesh. \n");

    // At this point the navigation mesh data is ready, you can access it from m_pmesh.
    // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.
    
    //
    // (Optional) Step 8. Create Detour data from Recast poly mesh.
    //
    
    // The GUI may allow more max points per polygon than Detour can handle.
    // Only build the detour navmesh if we do not exceed the limit.
    if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
    {
        unsigned char* navData = 0;
        int navDataSize = 0;

        // Update poly flags from areas.
        for (int i = 0; i < m_pmesh->npolys; ++i)
        {
            //printf("Update poly flags from area %u \n", i);

            if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
                m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
                
            if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
                m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
                m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
            {
                m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
            }
            else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
            {
                m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
            }
            else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
            {
                m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
            }
        }


        dtNavMeshCreateParams params;
        memset(&params, 0, sizeof(params));
        params.verts = m_pmesh->verts;
        params.vertCount = m_pmesh->nverts;
        params.polys = m_pmesh->polys;
        params.polyAreas = m_pmesh->areas;
        params.polyFlags = m_pmesh->flags;
        params.polyCount = m_pmesh->npolys;
        params.nvp = m_pmesh->nvp;
        params.detailMeshes = m_dmesh->meshes;
        params.detailVerts = m_dmesh->verts;
        params.detailVertsCount = m_dmesh->nverts;
        params.detailTris = m_dmesh->tris;
        params.detailTriCount = m_dmesh->ntris;
        params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
        params.offMeshConRad = m_geom->getOffMeshConnectionRads();
        params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
        params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
        params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
        params.offMeshConUserID = m_geom->getOffMeshConnectionId();
        params.offMeshConCount = m_geom->getOffMeshConnectionCount();
        params.walkableHeight = m_agentHeight;
        params.walkableRadius = m_agentRadius;
        params.walkableClimb = m_agentMaxClimb;
        rcVcopy(params.bmin, m_pmesh->bmin);
        rcVcopy(params.bmax, m_pmesh->bmax);
        params.cs = m_cfg.cs;
        params.ch = m_cfg.ch;
        params.buildBvTree = true;

        // printf("dtNavMeshCreateParams %p \n", params);
        debugConfig();
        
        if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
        {
            m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
            return false;
        }
        
        // printf("Built Detour navdata. %p \n", navData);

        m_navMesh = dtAllocNavMesh();
        if (!m_navMesh)
        {
            dtFree(navData);
            m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
            return false;
        }

        // printf("Created Detour navmesh. %p \n", m_navMesh);

        dtNavMeshParams nmparams;
        memset(&nmparams, 0, sizeof(nmparams));
        rcVcopy(nmparams.orig, m_geom->getMeshBoundsMin());
        nmparams.tileWidth = m_tileSize*m_cellSize;
        nmparams.tileHeight = m_tileSize*m_cellSize;
        nmparams.maxTiles = m_maxTiles;
        nmparams.maxPolys = m_maxPolysPerTile;
        
        // status = m_navMesh->init(&nmparams);
        status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);

        if (dtStatusFailed(status))
        {
            dtFree(navData);
            printf("Could not init Detour navmesh (%u) \n", status);
            m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
            return false;
        }
        
        // printf("Init Detour navmesh. %p \n", navData);

        m_navQuery = dtAllocNavMeshQuery();
        status = m_navQuery->init(m_navMesh, 2048);
        if (dtStatusFailed(status))
        {
            printf("Could not init Detour navmesh query (%u) \n", status);
            m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
            return false;
        }
    }

    m_crowd = dtAllocCrowd();
    if (!m_crowd)
    {
        dtFree(m_crowd);
        m_ctx->log(RC_LOG_ERROR, "Could not create Detour Crowd");
        return false;
    }

    // printf("m_navMesh=%p \n", m_navMesh);
    
    // printf("m_navQuery=%p \n", m_navQuery);

    //m_ctx->stopTimer(RC_TIMER_TOTAL);


    //
    // Step 9. Init tilecache
    // 
    m_tmproc->init(m_geom);

    // Tile cache params.
    dtTileCacheParams tcparams;
    memset(&tcparams, 0, sizeof(tcparams));
    rcVcopy(tcparams.orig, bmin);
    tcparams.cs = m_cellSize;
    tcparams.ch = m_cellHeight;
    tcparams.width = (int)m_tileSize;
    tcparams.height = (int)m_tileSize;
    tcparams.walkableHeight = m_agentHeight;
    tcparams.walkableRadius = m_agentRadius;
    tcparams.walkableClimb = m_agentMaxClimb;
    tcparams.maxSimplificationError = m_edgeMaxError;
    tcparams.maxTiles = tw*th*EXPECTED_LAYERS_PER_TILE;
    tcparams.maxObstacles = 128;
    
    // Tile cache alocation
    m_tileCache = dtAllocTileCache();
    if (!m_tileCache)
    {
        emscripten_log("buildTiledNavigation: Could not allocate tile cache.");
        m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
        return false;
    }
    status = m_tileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
    if (dtStatusFailed(status))
    {
        emscripten_log("buildTiledNavigation: Could not allocate tile cache.");
        m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
        return false;
    }

    m_cacheLayerCount = 0;
    m_cacheCompressedSize = 0;
    m_cacheRawSize = 0;
    
    for (int y = 0; y < th; ++y)
    {
        for (int x = 0; x < tw; ++x)
        {
            TileCacheData tiles[MAX_LAYERS];
            memset(tiles, 0, sizeof(tiles));
            int ntiles = rasterizeTileLayers(m_ctx, m_geom, x, y, m_cfg, tiles, MAX_LAYERS);

            for (int i = 0; i < ntiles; ++i)
            {
                TileCacheData* tile = &tiles[i];
                status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
                if (dtStatusFailed(status))
                {
                    dtFree(tile->data);
                    tile->data = 0;
                    continue;
                }
                
                m_cacheLayerCount++;
                m_cacheCompressedSize += tile->dataSize;
                m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
            }
        }
    }

    // Build initial meshes
    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x)
            m_tileCache->buildNavMeshTilesAt(x,y, m_navMesh);
        
    const dtNavMesh* nav = m_navMesh;
    int navmeshMemUsage = 0;
    for (int i = 0; i < nav->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = nav->getTile(i);
        if (tile->header)
            navmeshMemUsage += tile->dataSize;
    }
    sprintf(buff, "navmeshTileMemUsage = %.1f kB   %u tiles over %ux%u", navmeshMemUsage/1024.0f, nav->getMaxTiles(), th, tw);
    emscripten_log(buff);


    // Show performance stats.
    //duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
    //m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);
    
    //m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;

    return true;
}



#include <emscripten/bind.h>

using namespace emscripten;

EMSCRIPTEN_BINDINGS(my_module) {
    function("dumpConfig", &dumpConfig);

    function("initWithFile", &initWithFile);
    function("initWithFileContent", &initWithFileContent);

    function("build", &build);

    function("getNavMeshVertices", &getNavMeshVertices);
    function("getNavHeightfieldRegions", &getNavHeightfieldRegions);

    function("findNearestPoly", &findNearestPoly);
    function("findNearestPoint", &findNearestPoint);
    function("findPath", &findPath);
    function("setPolyFlags", &setPolyFlags);
    function("setPolyFlagsByRef", &setPolyFlagsByRef);

    function("addOffMeshConnection", &addOffMeshConnection);

    function("getRandomPoint", &getRandomPoint);
    function("_queryPolygons", &_queryPolygons);

    function("initCrowd", &initCrowd);
    function("addCrowdAgent", &addCrowdAgent);
    function("updateCrowdAgentParameters", &updateCrowdAgentParameters);
    function("removeCrowdAgent", &removeCrowdAgent);
    function("crowdRequestMoveTarget", &crowdRequestMoveTarget);
    function("crowdUpdate", &crowdUpdate);
    function("_crowdGetActiveAgents", &_crowdGetActiveAgents);
    function("requestMoveVelocity", &requestMoveVelocity);


    function("set_cellSize", &set_cellSize);
    function("set_cellHeight", &set_cellHeight);
    function("set_agentHeight", &set_agentHeight);
    function("set_agentRadius", &set_agentRadius);
    function("set_agentMaxClimb", &set_agentMaxClimb);
    function("set_agentMaxSlope", &set_agentMaxSlope);
    function("set_regionMinSize", &set_regionMinSize);
    function("set_regionMergeSize", &set_regionMergeSize);
    function("set_edgeMaxLen", &set_edgeMaxLen);
    function("set_edgeMaxError", &set_edgeMaxError);
    function("set_vertsPerPoly", &set_vertsPerPoly);
    function("set_detailSampleDist", &set_detailSampleDist);
    function("set_detailSampleMaxError", &set_detailSampleMaxError);

    function("addTempObstacle", &addTempObstacle);
    function("removeTempObstacle", &removeTempObstacle);
    function("clearAllTempObstacles", &clearAllTempObstacles);
    function("getAllTempObstacles", &getAllTempObstacles);

    function("debugCreateNavMesh", &debugCreateNavMesh);
    function("debugCreateNavMeshPortals", &debugCreateNavMeshPortals);
    function("debugCreateRegionConnections", &debugCreateRegionConnections);
    function("debugCreateRawContours", &debugCreateRawContours);
    function("debugCreateContours", &debugCreateContours);
    function("debugCreateHeightfieldSolid", &debugCreateHeightfieldSolid);
    function("debugCreateHeightfieldWalkable", &debugCreateHeightfieldWalkable);
    function("debugOffMeshConnections", &debugOffMeshConnections);

}

