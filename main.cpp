#include "kalman_filter.hpp"
#include "utils.hpp"

#include<vector>
#include<map>
#include<Eigen/Dense>
#include<string>
#include<iostream>
#include<memory>


using namespace std;
using namespace Eigen;

void task1() {
    int num_states = 2;        // x_pos, x_vel
    int num_measurements = 1;  // xm_pos

    string filename = "../data/cam_data1.txt";
    
    vector<double> timestamps;
    vector<double> gt_pos;
    vector<double> error;
    double stddev = 1;        // standard deviation for normal distibution noise
    double dt; 

    getData(filename, timestamps, gt_pos);
    error = errorGenerator(0, stddev, gt_pos.size());

    MatrixXd A(num_states, num_states);
    MatrixXd B(num_states, 1);
    MatrixXd H(num_measurements, num_states);
    MatrixXd Q(num_states, num_states);
    MatrixXd R(num_measurements, num_measurements);
    MatrixXd P(num_states, num_states);
    
    VectorXd x(num_states);
    VectorXd u = VectorXd::Zero(1);
    VectorXd y(num_measurements);
    vector<double> xe(gt_pos.size());

    double mse_e_gt = 0;
    double mse_p_gt = 0;

    H << 1, 0;
    Q << 0.1, 0, 0, 0.1;
    R << pow(stddev, 2);
    P << 0, 0, 0, 0;
    x << gt_pos[0], 0;
    xe[0] = gt_pos[0] + error[0];

    KalmanFilter KF_2_1(num_states, num_measurements);
    KF_2_1.setObervationMatrix(H);
    KF_2_1.setInitState(x, P);
    KF_2_1.setNoiseCovariance(Q);

    for(int i = 1; i < gt_pos.size(); i++) {
        dt = (timestamps[i] - timestamps[i-1])/1000;
        A << 1, dt, 0, 1;
        // B << 0.5*pow(dt, 2), dt; 
        B << 0, 0;
        KF_2_1.predict(A, B, u);

        y << gt_pos[i] + error[i];
        KF_2_1.update(y, R);

        x = KF_2_1.getStateEstimate();
        xe[i] = x[0];

        x = KF_2_1.getStatePredicited();
        mse_e_gt += pow(gt_pos[i] - xe[i], 2);
        mse_p_gt += pow(gt_pos[i] - x[0], 2);
    }

    mse_e_gt /= gt_pos.size() -1;
    mse_p_gt /= gt_pos.size() -1;

    cout << "KF to filter out noise from sensor data:\n";
    cout << "MSE between predicited and gt position is: " << mse_p_gt << "\n";
    cout << "MSE between estimated and gt position is: " << mse_e_gt << "\n";
    writeData("../results/output1.txt" ,timestamps, gt_pos, error, xe);
}


void task2() {
    int num_states = 2;        // x_pos, x_vel
    int num_measurements = 1;  // xm_pos

    string filename1 = "../data/cam_data2.txt";
    string filename2 = "../data/rad_data2.txt";
    
    vector<double> timestamps;
    vector<double> noise_pos;
    vector<int> sensor;
    vector<double> gt_pos;
    vector<double> gt_time;

    getData(filename1, filename2, timestamps, noise_pos, sensor);

    double stddev1 = 0.5;
    double stddev2 = 0.01;
    double dt;

    MatrixXd A(num_states, num_states);
    MatrixXd B(num_states, 1);
    MatrixXd H(num_measurements, num_states);
    MatrixXd Q(num_states, num_states);
    MatrixXd R1(num_measurements, num_measurements);
    MatrixXd R2(num_measurements, num_measurements);
    MatrixXd P(num_states, num_states);
    
    VectorXd x(num_states);
    VectorXd u = VectorXd::Zero(1);
    VectorXd y(num_measurements);
    vector<double> xe;

    double mse_e_gt = 0;
    double mse_p_gt = 0;

    H << 1, 0;
    Q << 0.06, 0.006, 0.006, 0.06;
    R1 << pow(stddev1, 2);
    R2 << pow(stddev2, 2);
    P << 0, 0, 0, 0;
    x << noise_pos[0], 0;
    xe.push_back(noise_pos[0]);

    KalmanFilter KF_2_2(num_states, num_measurements);
    KF_2_2.setObervationMatrix(H);
    KF_2_2.setInitState(x, P);
    KF_2_2.setNoiseCovariance(Q);

    double temp_pos = noise_pos[0];
    for(int i = 1; i < timestamps.size(); i++) {
        dt = (timestamps[i] - timestamps[i-1])/1000;
        A << 1, dt, 0, 1;
        // B << 0.5*pow(dt, 2), dt;
        B << 0, 0; 
        KF_2_2.predict(A, B, u);

        y << noise_pos[i];

        if(sensor[i] == 2) {
            // update KF with sensor 2 measurement
            KF_2_2.update(y, R2);
            temp_pos = noise_pos[i];
        }
        else {
            if(sensor[i] == 3) {
                // update KF with sensor 1 and 2 measurements
                KF_2_2.update(y, R1);
                i++;
                y << noise_pos[i];
                KF_2_2.update(y, R2);
                temp_pos = noise_pos[i];
            }
            else if(sensor[i] == 1) {
                // update KF with sensor 1 measurement
                KF_2_2.update(y, R1);
            }

            gt_time.push_back(timestamps[i]);
            gt_pos.push_back(temp_pos);

            x = KF_2_2.getStateEstimate();
            xe.push_back(x[0]);

            x = KF_2_2.getStatePredicited();
            mse_e_gt += pow(gt_pos.back() - xe.back(), 2);
            mse_p_gt += pow(temp_pos - x[0], 2);
        }
    }

    mse_e_gt /= (gt_pos.size() - 1);
    mse_p_gt /= (gt_pos.size() - 1);

    cout << "KF to estimate position with 2 independent sensors:\n";
    cout << "MSE between predicited and gt position is: " << mse_p_gt << "\n";
    cout << "MSE between estimated and gt position is: " << mse_e_gt << "\n";
    writeData("../results/output2.txt" , gt_time, gt_pos, xe);
}


int main() {
    // Kalman filter with 1 sensor
    task1();
    // Kalman filter with 2 intependent sensors
    task2();

    return 0;
}