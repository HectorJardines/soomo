#ifndef DISABLE_TRACE
#include "trace.h"
#include <stdbool.h>
#include "../external/printf/printf.h"
#include "uart.h"

static bool initialized = false;
void trace_init(void)
{
    if (initialized)
        return;
    usart_init();
    initialized = true;
}

void trace(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

#endif