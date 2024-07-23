#include "utils.hpp"

#include<string>
#include<iostream>
#include<fstream>
#include<vector>
#include<sstream>


using namespace std;

vector<double> errorGenerator(double mean, double stddev, int len) {
    random_device rd;
    std::mt19937 generator(rd());
    normal_distribution<double> distribution(mean, stddev);

    vector<double> error(len);

    for(int i = 0; i < len; i++) {
        error[i] = distribution(generator);
    }

    return error;
}

void getData(string filename, vector<double>& timestamps, vector<double>& gt_pos) {
    ifstream file(filename);
    if(!file.is_open()) {
        throw runtime_error("Failed to open file: " + filename);
    }

    string line;
    while(getline(file, line)) {
        stringstream ss(line);
        string token;

        getline(ss, token, ',');
        timestamps.push_back(stod(token));

        getline(ss, token, ',');
        gt_pos.push_back(stod(token));
    }

    file.close();
}

void getData(string filename1, string filename2, vector<double>& timestamps, vector<double>& noise_pos, vector<int>& sensor) {
    ifstream file1(filename1);
    ifstream file2(filename2);

    if(!file1.is_open()) {
        throw runtime_error("Failed to open file: " + filename1);
    }

    if(!file2.is_open()) {
        throw runtime_error("Failed to open file: " + filename1);
    }

    string line1;
    string line2;
    string token;
    double ts1;
    double ts2;
    double pos1;
    double pos2;

    getline(file1, line1);
    stringstream ss1(line1);
    getline(ss1, token, ',');
    ts1 = stod(token);
    getline(ss1, token, ',');
    pos1 = stod(token);


    getline(file2, line2);
    stringstream ss2(line2);
    getline(ss2, token, ',');
    ts2 = stod(token);
    getline(ss2, token, ',');
    pos2 = stod(token);

    // merging data from 2 sensors
    while(true) {
        if(ts1 < ts2) {
            timestamps.push_back(ts1);
            noise_pos.push_back(pos1);
            sensor.push_back(1);

            if(getline(file1, line1)) {
                stringstream ss1(line1);
                getline(ss1, token, ',');
                ts1 = stod(token);
                getline(ss1, token, ',');
                pos1 = stod(token);
            }
            else
                break;
        }
        else if(ts2 < ts1) {
            timestamps.push_back(ts2);
            noise_pos.push_back(pos2);
            sensor.push_back(2);

            if(getline(file2, line2)) {
                stringstream ss2(line2);
                getline(ss2, token, ',');
                ts2 = stod(token);
                getline(ss2, token, ',');
                pos2 = stod(token);
            }
            else
                break;
        }
        else {
            timestamps.push_back(ts2);
            timestamps.push_back(ts2);
            noise_pos.push_back(pos1);
            noise_pos.push_back(pos2);
            sensor.push_back(3);
            sensor.push_back(3);

            if(getline(file1, line1) && getline(file2, line2)) {
                stringstream ss1(line1);
                getline(ss1, token, ',');
                ts1 = stod(token);
                getline(ss1, token, ',');
                pos1 = stod(token);

                stringstream ss2(line2);
                getline(ss2, token, ',');
                ts2 = stod(token);
                getline(ss2, token, ',');
                pos2 = stod(token);
            }
            else
                break;
        }
    }

    file1.close();
    file2.close();
}

void writeData(string filename, vector<double>& timestamps, vector<double>& gt_pos, vector<double>& error, vector<double>& est_pos) {
    ofstream file(filename);
    if(!file.is_open()) {
        throw runtime_error("Failed to to open file: " + filename);
    }

    for(int i = 0; i < timestamps.size(); i++) { 
        file << to_string(timestamps[i]) << ',' << gt_pos[i] << ',' << gt_pos[i] + error[i];
        file << ',' << est_pos[i] << ',' << gt_pos[i] - est_pos[i] << "\n"; 
    }

    file.close();
    cout << "File written successfully.\n"; 
}

void writeData(string filename, vector<double>& timestamps, vector<double>& gt_pos, vector<double>& est_pos) {
    ofstream file(filename);
    if(!file.is_open()) {
        throw runtime_error("Failed to to open file: " + filename);
    }

    for(int i = 0; i < timestamps.size(); i++) { 
        file << to_string(timestamps[i]) << ',' << gt_pos[i] << ',' << est_pos[i];
        file << ',' << gt_pos[i] - est_pos[i] << "\n"; 
    }

    file.close();
    cout << "File written successfully.\n";  
}