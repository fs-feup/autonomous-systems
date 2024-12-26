#pragma once

#include "adapters/adapter.hpp"

class VENode;

class PacsimAdapter : public Adapter {
public:
  explicit PacsimAdapter(const VEParameters& parameters);
};