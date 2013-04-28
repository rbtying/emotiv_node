#ifndef DIE_H_
#define DIE_H_

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

void die(const char *str, ...);

void logerr(const char *str, ...);

#endif /* DIE_H_ */
