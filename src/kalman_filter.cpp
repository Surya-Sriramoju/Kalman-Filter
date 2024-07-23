#include "kalman_filter.hpp"

#include<iostream>

using namespace Eigen;
using namespace std;

KalmanFilter::KalmanFilter(int num_states, int num_measurements) 
    : num_states_(num_states),
      num_measurements_(num_measurements),
      H_(num_measurements, num_states),
      Q_(num_states, num_states)
    {
        I_ = MatrixXd::Identity(num_states, num_states);
    }

void KalmanFilter::setObervationMatrix(const MatrixXd& H) {
    if(H.rows() != num_measurements_ || H.cols() != num_states_) {
        throw runtime_error("Invalid dimension for obeservation matrix.");
    }
    H_ = H;
}

void KalmanFilter::setInitState(const VectorXd& xi, const MatrixXd& Pi) {
    if(xi.size() != num_states_ || Pi.rows() != num_states_ ||
       Pi.cols() != num_states_ ) {
        throw runtime_error("Invalid dimensions for system state.");
    }
    xe_ = xi;
    Pe_ = Pi;
}

void KalmanFilter::setNoiseCovariance(const MatrixXd& Q) {
    if(Q.rows() != num_states_ || Q.cols() != num_states_) {
        throw runtime_error("Invalid dimension for noise covariance matrix.");
    }
    Q_ = Q;
}

void KalmanFilter::predict(const MatrixXd& A, const MatrixXd& B, const VectorXd& u) {
    if(A.rows() != num_states_ || A.cols() != num_states_ ||
       B.rows() != num_states_ || B.cols() != 1 || u.size() != 1) {
        throw runtime_error("Invalid dimensions for state matrices.");
       }
    
    xp_ = (A * xe_) + (B * u);
    Pp_ = (A * Pe_ * A.transpose()) + Q_; 
}

void KalmanFilter::update(const VectorXd& y, const MatrixXd& R) {
    if(R.rows() != num_measurements_ || R.cols() != num_measurements_ ||
       y.size() != num_measurements_) {
        throw runtime_error("Invalid dimension for Observation noise covariance matrix.");
       }
    K_ = Pp_ * H_.transpose() * ((H_ * Pp_ * H_.transpose()) + R).inverse();
    xe_ = xp_ + K_ * (y - (H_ * xp_));
    Pe_ = (I_ - (K_ * H_)) * Pp_;
}

VectorXd KalmanFilter::getStateEstimate() const {
    return xe_;
}

VectorXd KalmanFilter::getStatePredicited() const {
    return xp_;
}