#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <chrono> //C++ standard library for time measurement used to benchmark the runtime
using namespace std;

double f(double x, double y) { return -y; } //defines the mathematical function being evaluated. It represents dy/dx=−y, returning the slope for any given coordinate

double rk4_solve(double y0, double x0, double h, int steps) {
    double y = y0;
    double x = x0;
    for (int i = 0; i < steps; ++i) {
        double k1 = f(x, y);
        double k2 = f(x + h/2.0, y + h*k1/2.0);
        double k3 = f(x + h/2.0, y + h*k2/2.0);
        double k4 = f(x + h, y + h*k3);
        y += (h/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
        x += h;
    }
    return y;
}

int main() {
    ifstream file("benchmark_inputs.csv");
    string line;
    vector<pair<double, double>> inputs;
    
    getline(file, line); // header
    while (getline(file, line)) {
        stringstream ss(line);
        string x_str, y_str;
        getline(ss, x_str, ',');
        getline(ss, y_str, ',');
        inputs.push_back({stod(x_str), stod(y_str)});
    }

    auto start = chrono::high_resolution_clock::now();
    vector<double> results;
    
    for (auto& p : inputs) {
        results.push_back(rk4_solve(p.second, p.first, 0.01, 100000));
    }
    
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double, milli> diff = end - start;

    cout << "--- C++ Standardized Benchmark ---\n";
    cout << "Processed " << inputs.size() << " equations sequentially.\n";
    cout << "Total Runtime: " << diff.count() << " ms\n";
    return 0;
}
