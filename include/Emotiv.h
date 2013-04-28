#ifndef EMOTIV_H_
#define EMOTIV_H_

#include <iostream>

#include "edk.h"
#include "edkErrorCode.h"
#include "EmoStateDLL.h"

#include "data.h"

void emote(volatile bool *running, data_t *data);

#endif
