# Kalman Filter
A C++ package with the implementation of Kalman Filter for sensor fusion to position based on measurements from 2 sensors with different sampling rates.

## Steps to build the package and run the test code
To Build the package:
```
cd <path_to_package>/kalman_filter
mkdir build
cd build
cmake ..
make
```

To run the test code:
```
cd build
./KalmanFilter
```

## To install the dependencies
```
sudo apt update -y
sudo apt install libeigen3-dev
```

## To plot the graphs
```
cd <path_to_package>/kalman_filter
python3 plot.py
```