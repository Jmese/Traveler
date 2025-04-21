#include <vector>
#include <string>
#include <chrono>
#include "PDController.h"
#include "GPIOInterface.h"
#include "utils.h"

class StagedProbe {
public:
    StagedProbe(int divisions, float start_theta, float start_rho, float final_theta, float final_rho, float pause_duration);

    void initializeStages();
    void executeStages();
    void cleanup();
    void saveData(const std::string& file_path);

private:
    int divisions;
    float start_theta;
    float start_rho;
    float final_theta;
    float final_rho;
    float pause_duration;
    float d_theta;
    float d_rho;

    PDController thetaController;
    PDController rhoController;
    GPIOInterface gpio13{13, 1.0};
    GPIOInterface gpio17{17, 1.0};

    std::vector<std::vector<double>> data_log;
    std::chrono::time_point<std::chrono::steady_clock> elapsed_start_time;
};