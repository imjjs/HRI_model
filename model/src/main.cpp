#include <despot/planner.h>
#include "rock_sample.h"
#include "simulator.h"

#define STEP_LIMIT 99999

using namespace std;
using namespace despot;

class MyPlanner: public Planner {
public:
  MyPlanner() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = NULL;
		if (options[E_PARAMS_FILE]) {
			model = new RockSample(options[E_PARAMS_FILE].arg);
		} else {
			int size = 7, number = 8;
			if (options[E_SIZE])
				size = atoi(options[E_SIZE].arg);
			else {
				cerr << "Specify map size using --size option" << endl;
				exit(0);
			}
			if (options[E_NUMBER]) {
				number = atoi(options[E_NUMBER].arg);
			} else {
				cerr << "Specify number of rocks using --number option" << endl;
				exit(0);
			}
			model = new RockSample(size, number);
		}
    return model;
  }

  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
  {
      //Create a custom world as defined and implemented by the user
      LaserTagWorld* world = new PlayerWorld();
      //Establish connection with external system
      world->Connect();
      //Initialize the state of the external system
      world->Initialize();
      //Inform despot the type of world
      world_type = "simulator";
      return world; 
  }

  void PlanningLoop(Solver*& solver, World* world, Logger* logger) {
    for (int i = 0; i <STEP_LIMIT; ++i) {
      bool terminal = RunStep(solver, world, logger);
      if (terminal)
        break;
    }
  }

  /*Customize the inner step of the planning pipeline by overloading the following function if necessary*/
  bool RunStep(Solver* solver, World* world, Logger* logger) {
    logger->CheckTargetTime();

    double step_start_t = get_time_second();

    double start_t = get_time_second();
    ACT_TYPE action = solver->Search().action;
    double end_t = get_time_second();
    double search_time = (end_t - start_t);
    logi << "[Custom RunStep] Time spent in " << typeid(*solver).name()
        << "::Search(): " << search_time << endl;

    OBS_TYPE obs;
    start_t = get_time_second();
    bool terminal = world->ExecuteAction(action, obs);
    end_t = get_time_second();
    double execute_time = (end_t - start_t);
    logi << "[Custom RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

    start_t = get_time_second();
    solver->BeliefUpdate(action, obs);
    end_t = get_time_second();
    double update_time = (end_t - start_t);
    logi << "[Custom RunStep] Time spent in Update(): " << update_time << endl;

    return logger->SummarizeStep(step_++, round_, terminal, action, obs,
        step_start_t);
  }

};
  void InitializeDefaultParameters() {
  }

  std::string ChooseSolver(){
	  return "DESPOT";
  }
};




int main(int argc, char* argv[]) {
  return MyPlanner().RunEvaluation(argc, argv);
}
