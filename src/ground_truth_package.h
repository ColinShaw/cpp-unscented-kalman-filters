#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage {
public:
  long timestamp;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type;

  Eigen::VectorXd gt_values;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
