// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main() {

    using namespace mpcc;


//    std::cout << testSpline() << std::endl;
//    std::cout << testArcLengthSpline() << std::endl;
//
//    std::cout << testIntegrator() << std::endl;
//    std::cout << testLinModel() << std::endl;
//
//    std::cout << testAlphaConstraint() << std::endl;
//    std::cout << testTireForceConstraint() << std::endl;
//    std::cout << testTrackConstraint() << std::endl;
//
//    std::cout << testCost() << std::endl;

    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    Integrator integrator;
    Plotting plotter;

    Track track;
    // TrackPos的成员是Eigen::VectorXd，保存赛道的中心线坐标和边界坐标。
    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"]);
    mpc.setTrack(track_xy.X,track_xy.Y);// 对中心线样条插值
    State x0 = {track_xy.X(0),track_xy.Y(0),-1*M_PI/4.0,0.05,0,0,0,1.0,0,1.0}; // 10个状态量
    for(int i=0;i<jsonConfig["n_sim"];i++) // n_sim = 600
    {
        auto before_solve = std::clock();
        MPCReturn mpc_sol = mpc.runMPC(x0); // 求解
        auto after_solve = std::clock();
        std::cout << "solve time: " << (double) (after_solve - before_solve) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;
        x0 = integrator.simTimeStep(x0,mpc_sol.u0,0.02); // 推算下一个状态
        log.push_back(mpc_sol);
    }
    plotter.plotRun(log,track_xy);
    plotter.plotSim(log,track_xy);

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;
    return 0;
}


