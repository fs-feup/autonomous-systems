#include <cone_evaluator/cone_evaluator.hpp>

ConeEvaluator::ConeEvaluator(std::unordered_map<std::string, ConeValidator> cone_validators,
                             std::unordered_map<std::string, double> validator_weights)
    : cone_validators_(cone_validators), validator_weights_(validator_weights) {}