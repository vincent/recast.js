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

// EM_JS helper: initialise __RECAST_GLOBAL_DATA with mode and flat float array
EM_JS(void, js_set_debug_data, (const float* dataPtr, int count, int mode), {
    const arr = count > 0 ? Array.from(HEAPF32.subarray(dataPtr >> 2, (dataPtr >> 2) + count)) : [];
    arr.mode = mode;
    Module.__RECAST_GLOBAL_DATA = arr;
});

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

	printf("%s\n", msg);
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
	// stub
}

void DebugDrawGL::begin(duDebugDrawPrimitives prim, float size)
{
	m_vertexBuffer.clear();
	switch (prim)
	{
		case DU_DRAW_POINTS: m_mode = 1; break;
		case DU_DRAW_LINES:  m_mode = 2; break;
		case DU_DRAW_TRIS:   m_mode = 3; break;
		case DU_DRAW_QUADS:  m_mode = 4; break;
		default:             m_mode = 0; break;
	}
}

void DebugDrawGL::vertex(const float* pos, unsigned int color)
{
	if (!isnan(pos[0]) && !isnan(pos[1]) && !isnan(pos[2])) {
		m_vertexBuffer.push_back(pos[0]);
		m_vertexBuffer.push_back(pos[1]);
		m_vertexBuffer.push_back(pos[2]);
	}
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color)
{
	if (!isnan(x) && !isnan(y) && !isnan(z)) {
		m_vertexBuffer.push_back(x);
		m_vertexBuffer.push_back(y);
		m_vertexBuffer.push_back(z);
	}
}

void DebugDrawGL::vertex(const float* pos, unsigned int color, const float* uv)
{
	if (!isnan(pos[0]) && !isnan(pos[1]) && !isnan(pos[2])) {
		m_vertexBuffer.push_back(pos[0]);
		m_vertexBuffer.push_back(pos[1]);
		m_vertexBuffer.push_back(pos[2]);
	}
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v)
{
	if (!isnan(x) && !isnan(y) && !isnan(z)) {
		m_vertexBuffer.push_back(x);
		m_vertexBuffer.push_back(y);
		m_vertexBuffer.push_back(z);
	}
}

void DebugDrawGL::end()
{
	// Transfer the accumulated vertex buffer to JS and register with the current GL object
	js_set_debug_data(
		m_vertexBuffer.empty() ? nullptr : m_vertexBuffer.data(),
		(int)m_vertexBuffer.size(),
		m_mode
	);
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
