#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>


class CylinderValidator : public ConeValidator {
public:
    bool coneValidator(Cluster* cone_point_cloud) const override;
};
