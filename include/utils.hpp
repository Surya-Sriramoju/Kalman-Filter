#pragma once

#include<random>
#include<vector>
#include<map>

using namespace std;

vector<double> errorGenerator(double mean, double stddev, int len);

void getData(string filename, vector<double>& timestamps,
    vector<double>& gt_pos);

void getData(string filename1, string filename2, vector<double>& timestamps,
    vector<double>& noise_pos, vector<int>& sensor);

void writeData(string filename, vector<double>& timestamps,
    vector<double>& gt_pos, vector<double>& error, vector<double>& est_pos);

void writeData(string filename, vector<double>& timestamps,
    vector<double>& gt_pos, vector<double>& est_pos);