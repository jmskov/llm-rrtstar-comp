#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <fstream>
#include <chrono>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State* state) {
    const auto* r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double x = r2state->values[0];
    double y = r2state->values[1];
    
    // Circle obstacle at (5,5) with radius 2
    double dx = x - 5.0;
    double dy = y - 5.0;
    return (dx * dx + dy * dy) > 4.0;
}

void savePath(const og::PathGeometric& path, int path_id, std::ofstream& file) {
    int point_id = 0;
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto* state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        file << path_id << "," << point_id << "," 
             << state->values[0] << "," << state->values[1] << "\n";
        point_id++;
    }
}

int main() {
    // Create 2D state space
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    
    // Set bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-2);
    bounds.setHigh(12);
    space->setBounds(bounds);
    
    // Create simple setup class
    og::SimpleSetup ss(space);
    
    // Set state validity checker
    ss.setStateValidityChecker([](const ob::State* state) { return isStateValid(state); });
    
    // Set start and goal states
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start[0] = 0.0;  // x
    start[1] = 0.0;  // y
    
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal[0] = 10.0;  // x
    goal[1] = 10.0;  // y
    
    ss.setStartAndGoalStates(start, goal);
    
    // Create and set RRT* planner
    auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    planner->setRange(0.05);  // Step size
    planner->setGoalBias(0.05);      // 10% chance of sampling goal
    planner->setKNearest(false);    // Use radius-based neighbor selection
    planner->setRewireFactor(0.1);        // Try to match paths based on LLM planner
    // Other settings I was trying to increase the runtime:
    // planner->setDelayCC(false);      // Delay collision checking
    // planner->setTreePruning(true);  // Enable tree pruning
    // planner->setPruneThreshold(1.0);
    // Set max iterations
    // planner->setMaxNumStates(10000);  // max_iter = 10000
    ss.setPlanner(planner);

    // Set optimization objective
    auto opt = std::make_shared<ob::PathLengthOptimizationObjective>(ss.getSpaceInformation());
    opt->setCostThreshold(ob::Cost(1.0));  // More aggressive optimization
    ss.setOptimizationObjective(opt);

    // Benchmark variables
    const int NUM_RUNS = 500;
    int successes = 0;
    std::chrono::duration<double> total_duration(0);
    double total_time_to_first = 0.0;
    
    // Create output files
    std::ofstream path_file("paths.csv");
    path_file << "path_id,point_id,x,y\n";
    
    double timeout = 0.01;

    // Run benchmark
    for (int i = 0; i < NUM_RUNS; i++) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(timeout));

        auto status = ss.solve(ptc);  // 1 second timeout
        
        auto end_time = std::chrono::high_resolution_clock::now();
        total_duration += end_time - start_time;
        
        if (status == ob::PlannerStatus::EXACT_SOLUTION) {
            successes++;
            savePath(ss.getSolutionPath(), successes - 1, path_file);
            // total_time_to_first += planner->getTimeToFirstSolution();
        }
        
        // Reset for next run
        planner->clear();
        ss.clear();
        ss.setStartAndGoalStates(start, goal);
    }
    
    path_file.close();
    
    // Save benchmark results
    std::ofstream results_file("benchmark_results.txt");
    results_file << "Benchmark Results:\n"
                << "Total runs: " << NUM_RUNS << "\n"
                << "Successful paths: " << successes << "\n"
                << "Average time per run: " 
                << (total_duration.count() * 1000.0 ) / NUM_RUNS << "ms\n"
                << "Average time to first solution: "
                << (successes > 0 ? total_time_to_first / successes : 0.0) << " seconds\n";
    results_file.close();
    
    return 0;
}