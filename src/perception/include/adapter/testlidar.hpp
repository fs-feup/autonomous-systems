#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_TEST_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_TEST_HPP_

#include "adapter/adapter.hpp"

class TestAdapter : public Adapter {

public:
    TestAdapter(Perception* perception);

    void init() override;
    void mission_state_callback(const std::string msg);
    void finish() override;
};

#endif