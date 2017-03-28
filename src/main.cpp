#include <iostream>
#include "Eigen/Dense"
#include <vector>
#include "ukf.h"
#include "measurement_package.h"
#include "ground_truth_package.h"
#include <fstream>
#include <sstream>
#include <stdlib.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {
  // Check arguments and files
  check_arguments(argc, argv);

  string in_file_name = argv[1];
  ifstream in_file(in_file_name.c_str(), ifstream::in);

  string out_file_name = argv[2];
  ofstream out_file(out_file_name.c_str(), ofstream::out);

  check_files(in_file, in_file_name, out_file, out_file_name);

  // Packings
  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  string line;

  UKF ukf;

  while (getline(in_file, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long timestamp;

    // First element from the current line
    iss >> sensor_type;

    // Lidar
    if (sensor_type.compare("L") == 0 && ukf.use_lidar == true) {
      float px;
      float py;
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      meas_package.sensor_type = MeasurementPackage::LASER;
      meas_package.raw_measurements = VectorXd(2);
      iss >> px;
      iss >> py;
      meas_package.raw_measurements << px, py;
      iss >> timestamp;
      meas_package.timestamp = timestamp;
      measurement_pack_list.push_back(meas_package);
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      gt_package.gt_values = VectorXd(4);
      gt_package.gt_values << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_package);

    // Radar
    } else if (sensor_type.compare("R") == 0 && ukf.use_radar == true) {
      float rho;
      float phi;
      float rho_dot;
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      meas_package.sensor_type = MeasurementPackage::RADAR;
      meas_package.raw_measurements = VectorXd(3);
      iss >> rho;
      iss >> phi;
      iss >> rho_dot;
      meas_package.raw_measurements << rho, phi, rho_dot;
      iss >> timestamp;
      meas_package.timestamp = timestamp;
      measurement_pack_list.push_back(meas_package);
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      gt_package.gt_values = VectorXd(4);
      gt_package.gt_values << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_package);
    }
  }

  size_t number_of_measurements = measurement_pack_list.size();

  for (size_t k = 0; k < number_of_measurements; ++k) {
    ukf.ProcessMeasurement(measurement_pack_list[k]);
    VectorXd estimate = ukf.x;

    // Output Cartesian version of output estimates
    out_file << estimate(0) << "\t";
    out_file << estimate(1) << "\t";
    out_file << estimate(2) * cos(estimate(3)) << "\t";
    out_file << estimate(2) * sin(estimate(3)) << "\t";

    // Output ground truth
    out_file << gt_pack_list[k].gt_values(0) << "\t";
    out_file << gt_pack_list[k].gt_values(1) << "\t";
    out_file << gt_pack_list[k].gt_values(2) << "\t";
    out_file << gt_pack_list[k].gt_values(3) << "\t"; 

    // Output NIS
    out_file << ukf.NIS << "\n";

    // Build estimate and ground truth vectors for RMSE
    estimations.push_back(estimate);
    ground_truth.push_back(gt_pack_list[k].gt_values);
  }

  // Display RMSE
  VectorXd rmse = ukf.CalculateRMSE(estimations, ground_truth);
  cout << "RMSE:" << endl << rmse << endl;

  // close files
  if (out_file.is_open()) {
    out_file.close();
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  return 0;
}
