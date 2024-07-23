#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
 private:
    int num_states_;
    int num_measurements_;

    MatrixXd H_;
    MatrixXd Q_;
    MatrixXd Pp_;
    MatrixXd Pe_;
    MatrixXd K_;
    MatrixXd I_;

    VectorXd xp_;
    VectorXd xe_;

 public:
    
    KalmanFilter(int num_states, int num_measurements);

    void setObservationMatrix(const MatrixXd& H);

    void setInitState(const VectorXd& xi, const MatrixXd& Pi);

    void setNoiseCovariance(const MatrixXd& Q);

    void predict(const MatrixXd& A, const MatrixXd& B, const VectorXd& u);

    void update(const VectorXd& y, const MatrixXd& R);
    
    VectorXd getStateEstimate() const;

    VectorXd getStatePredicited() const;

};
