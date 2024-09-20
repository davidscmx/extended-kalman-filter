#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "measurement_package.h"
#include <Eigen/Dense>



using namespace std;
using namespace Eigen;
using Eigen::VectorXd;
using namespace std;

// Function to read measurements from file
vector<MeasurementPackage> ReadMeasurements(const string& filename) {
    vector<MeasurementPackage> measurements;
    ifstream file(filename);
    string line;

    while (getline(file, line)) {
        istringstream iss(line);
        MeasurementPackage meas_package;

        char sensor_type;
        iss >> sensor_type;

        if (sensor_type == 'L') {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px, py;
            iss >> px >> py;
            meas_package.raw_measurements_ << px, py;
        } else if (sensor_type == 'R') {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float rho, phi, rho_dot;
            iss >> rho >> phi >> rho_dot;
            meas_package.raw_measurements_ << rho, phi, rho_dot;
        }

        long long timestamp;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;

        // Skip ground truth values
        float gt_px, gt_py, gt_vx, gt_vy;
        iss >> gt_px >> gt_py >> gt_vx >> gt_vy;

        measurements.push_back(meas_package);
    }

    return measurements;
}

int main() {
    // Read measurements from file
    string input_file = "../data/obj_pose-laser-radar-synthetic-input.txt";
    vector<MeasurementPackage> measurements = ReadMeasurements(input_file);

    for (const auto& meas_package : measurements) {
        cout << "Sensor Type: " << (meas_package.sensor_type_ == MeasurementPackage::LASER ? "LASER" : "RADAR") << endl;
        cout << "Timestamp: " << meas_package.timestamp_ << endl;
        cout << "Raw Measurements: ";
        for (int i = 0; i < meas_package.raw_measurements_.size(); ++i) {
            cout << meas_package.raw_measurements_(i) << " ";
        }
        cout << endl << endl;
    }

    return 0;
}
