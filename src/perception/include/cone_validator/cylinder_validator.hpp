#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>


class CylinderValidator : public ConeValidator {

private:

    double width;
    double height;

public:
    CylinderValidator(double width, double height);
    double getRadius() const;
    bool coneValidator(Cluster* cone_point_cloud) const override;
};
