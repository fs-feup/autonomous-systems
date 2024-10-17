#pragma once
#include <gmock/gmock.h>

#include "../../include/canlib_wrappers/ican_lib_wrapper.hpp"

class MockCanLibWrapper : public ICanLibWrapper {
public:
  MOCK_METHOD(canStatus, canWrite,
              (canHandle hnd, long id, void* msg, unsigned int dlc, unsigned int flag), (override));
  MOCK_METHOD(canStatus, canRead,
              (canHandle hnd, long* id, void* msg, unsigned int* dlc, unsigned int* flag,
               unsigned long* time),
              (override));
};