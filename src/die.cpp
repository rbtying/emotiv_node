#include "die.h"

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void die(const char *str, ...) {
    va_list args;

    va_start(args, str);
    vfprintf(stderr, str, args);
    va_end(args);

    exit(1);
}

void logerr(const char *str, ...) {
    va_list args;
    
    va_start(args, str);
    vfprintf(stderr, str, args);
    va_end(args);
}

