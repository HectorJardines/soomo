#ifndef _TRACE_H
#define _TRACE_H

#define TRACE(fmt, ...) trace("%s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#ifndef DISABLE_TRACE
void trace_init(void);
void trace(const char *format, ...);
#else
#define trace_init() ;
#define trace(fmt, ...) ;
#endif

#endif