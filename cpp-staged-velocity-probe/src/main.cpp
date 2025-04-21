#include <iostream>
#include <csignal>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include "StagedProbe.h"
#include "CANInterface.h"

using namespace std;
using namespace std::chrono;

volatile sig_atomic_t stop = 0;
CANInterface canInterface("can0");
void handle_sigint(int sig) {
    stop = 1;
}

int main() {
    signal(SIGINT, handle_sigint);

    int divisions;
    float pause_duration, final_rho;
    cout << "Enter the number of divisions: ";
    cin >> divisions;
    cout << "Enter the hold time for each stage (seconds): ";
    cin >> pause_duration;
    cout << "Enter the final rho setpoint: ";
    cin >> final_rho;

    StagedProbe probe(divisions, 3.14, 0.6, 3.14, final_rho, pause_duration);

    try {
        probe.initializeStages();
        probe.executeStages();
        cout << "All stages complete, setting idle." << endl;
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    probe.cleanup();

    cout << "Do you want to save the data? (y/n): ";
    char save_data;
    cin >> save_data;
    if (save_data == 'y' || save_data == 'Y') {
        cout << "Enter the TEST string: ";
        string test_string;
        cin >> test_string;

        auto now = system_clock::now();
        time_t now_c = system_clock::to_time_t(now);
        stringstream ss;
        ss << put_time(localtime(&now_c), "%m%d_%H%M");
        string filename = test_string + "[" + ss.str() + "].csv";
        string file_path = "/home/traveler/Traveler_Hopper_sw-bundle/Data/STAGE/" + filename;

        cout << "Saving Data to: " + file_path << endl;
        probe.saveData(file_path);
        cout << "Save Complete" << endl;
    } else {
        cout << "Data not saved." << endl;
    }

    return 0;
}