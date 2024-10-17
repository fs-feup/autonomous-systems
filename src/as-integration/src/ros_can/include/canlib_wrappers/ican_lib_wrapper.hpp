// ICanLibWrapper.hpp
#pragma once

#include <canlib.h>

class ICanLibWrapper {
public:
  virtual ~ICanLibWrapper() = default;

  virtual canStatus canWrite(canHandle hnd, long id, void *msg, unsigned int dlc,
                             unsigned int flag) = 0;

  virtual canStatus canRead(canHandle hnd, long *id, void *msg, unsigned int *dlc,
                            unsigned int *flag, unsigned long *time) = 0;
};