#include "adapter/pacsim.hpp"

#include "planning/planning.hpp"

PacSimAdapter::PacSimAdapter(Planning* planning) : Adapter(planning) { this->init(); }

void PacSimAdapter::init() {
  pacsim_ebs_server = this->node->create_service<std_srvs::srv::Empty>(
      "pacsim/finish_signal", std::bind(&PacSimAdapter::handle_service, this, std::placeholders::_1,
                                        std::placeholders::_2));
}
