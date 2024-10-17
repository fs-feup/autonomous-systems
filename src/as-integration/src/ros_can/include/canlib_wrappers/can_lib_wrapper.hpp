// CanLibWrapper.hpp
#pragma once

#include "ican_lib_wrapper.hpp"

class CanLibWrapper : public ICanLibWrapper {
public:
  canStatus canWrite(canHandle hnd, long id, void *msg, unsigned int dlc,
                     unsigned int flag) override {
    return ::canWrite(hnd, id, msg, dlc, flag);
  }

  canStatus canRead(canHandle hnd, long *id, void *msg, unsigned int *dlc, unsigned int *flag,
                    unsigned long *time) override {
    return ::canRead(hnd, id, msg, dlc, flag, time);
  }
};