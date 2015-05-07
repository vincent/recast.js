#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#include "JavascriptInterface.h"

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "PerfTimer.h"

#include <emscripten.h>

#include <string.h>
#include <memory.h>


#ifdef WIN32
#	define snprintf _snprintf
#endif

extern "C" {
    extern void gl_add_to_object();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

BuildContext::BuildContext() :
	m_messageCount(0),
	m_textPoolSize(0)
{
	resetTimers();
}

BuildContext::~BuildContext()
{
}

// Virtual functions for custom implementations.
void BuildContext::doResetLog()
{
	m_messageCount = 0;
	m_textPoolSize = 0;
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (!len) return;
	if (m_messageCount >= MAX_MESSAGES)
		return;

	char buff[512];
	sprintf(buff, "console.log('%s');", msg);
	emscripten_run_script(buff);
}

void BuildContext::doResetTimers()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i)
		m_accTime[i] = -1;
}

void BuildContext::doStartTimer(const rcTimerLabel label)
{
}

void BuildContext::doStopTimer(const rcTimerLabel label)
{
}

int BuildContext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return 0;
}

void BuildContext::dumpLog(const char* format, ...)
{
}

int BuildContext::getLogCount() const
{
	return m_messageCount;
}

const char* BuildContext::getLogText(const int i) const
{
	return m_messages[i]+1;
}

void DebugDrawGL::depthMask(bool state)
{
	// glDepthMask(state ? GL_TRUE : GL_FALSE);
}

void DebugDrawGL::texture(bool state)
{
	// if (state)
	// {
	// 	glEnable(GL_TEXTURE_2D);
	// 	g_tex.bind();
	// }
	// else
	// {
	// 	glDisable(GL_TEXTURE_2D);
	// }
}

void DebugDrawGL::begin(duDebugDrawPrimitives prim, float size)
{
	switch (prim)
	{
		case DU_DRAW_POINTS:
			// emscripten_run_script("console.log(\"DebugDrawGL::begin POINTS\");");
			emscripten_run_script("Module.__RECAST_GLOBAL_DATA = []; Module.__RECAST_GLOBAL_DATA.mode = 1;");
			break;
		case DU_DRAW_LINES:
			// emscripten_run_script("console.log(\"DebugDrawGL::begin LINES\");");
			emscripten_run_script("Module.__RECAST_GLOBAL_DATA = []; Module.__RECAST_GLOBAL_DATA.mode = 2;");
			break;
		case DU_DRAW_TRIS:
			// emscripten_run_script("console.log(\"DebugDrawGL::begin TRIANGLES\");");
			emscripten_run_script("Module.__RECAST_GLOBAL_DATA = []; Module.__RECAST_GLOBAL_DATA.mode = 3;");
			break;
		case DU_DRAW_QUADS:
			// emscripten_run_script("console.log(\"DebugDrawGL::begin STATIC_DRAW\");");
			emscripten_run_script("Module.__RECAST_GLOBAL_DATA = []; Module.__RECAST_GLOBAL_DATA.mode = 4;");
			break;
	};
}

void DebugDrawGL::vertex(const float* pos, unsigned int color)
{
	char buff[1024];
	if ( !isnan(pos[0]) && !isnan(pos[1]) && !isnan(pos[2]) ) {
		sprintf(buff, "Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f);", pos[0], pos[1], pos[2]);
		emscripten_run_script(buff);
	}
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color)
{
	char buff[1024];
	if ( !isnan(x) && !isnan(y) && !isnan(z) ) {
		sprintf(buff, "Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f);", x, y, z);
		emscripten_run_script(buff);
		// sprintf(buff, "console.log(' %s ');", buff);
		// emscripten_run_script(buff);
	}
}

void DebugDrawGL::vertex(const float* pos, unsigned int color, const float* uv)
{
	// glTexCoord2fv(uv);

	char buff[1024];
	if ( !isnan(pos[0]) && !isnan(pos[1]) && !isnan(pos[2]) ) {
		sprintf(buff, "Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f);", pos[0], pos[1], pos[2]);
		emscripten_run_script(buff);
		// sprintf(buff, "console.log(' %s ');", buff);
		// emscripten_run_script(buff);
	}
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v)
{
	// glTexCoord2f(u,v);

	char buff[1024];
	if ( !isnan(x) && !isnan(y) && !isnan(z) ) {
		sprintf(buff, "Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f); Module.__RECAST_GLOBAL_DATA.push(%f);", x, y, z);
		emscripten_run_script(buff);
		// sprintf(buff, "console.log(' %s ');", buff);
		// emscripten_run_script(buff);
	}
}

void DebugDrawGL::end()
{
	// Instead of drawing, we register the object vertices to the current object (see gl_create_object)
	gl_add_to_object();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

FileIO::FileIO() :
	m_fp(0),
	m_mode(-1)
{
}

FileIO::~FileIO()
{
	if (m_fp) fclose(m_fp);
}

bool FileIO::openForWrite(const char* path)
{
	if (m_fp) return false;
	m_fp = fopen(path, "wb");
	if (!m_fp) return false;
	m_mode = 1;
	return true;
}

bool FileIO::openForRead(const char* path)
{
	if (m_fp) return false;
	m_fp = fopen(path, "rb");
	if (!m_fp) return false;
	m_mode = 2;
	return true;
}

bool FileIO::isWriting() const
{
	return m_mode == 1;
}

bool FileIO::isReading() const
{
	return m_mode == 2;
}

bool FileIO::write(const void* ptr, const size_t size)
{
	if (!m_fp || m_mode != 1) return false;
	fwrite(ptr, size, 1, m_fp);
	return true;
}

bool FileIO::read(void* ptr, const size_t size)
{
	if (!m_fp || m_mode != 2) return false;
	fread(ptr, size, 1, m_fp);
	return true;
}


