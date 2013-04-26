#ifndef SERVE_H_
#define SERVE_H_

#include "data.h"

int serve(volatile bool *running, data_t *data, int port);
std::string serialize_data(data_t *data);

#endif
