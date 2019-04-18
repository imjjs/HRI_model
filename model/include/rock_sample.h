#ifndef ROCKSAMPLE_H
#define ROCKSAMPLE_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include "base_rock_sample.h"
#include <despot/util/coord.h>
#include <despot/util/grid.h>
#include <unordered_set>

namespace despot {

/* =============================================================================
 * RockSample class
 * =============================================================================*/

class RockSample: public BaseRockSample {
public:
	RockSample(std::string map);
	RockSample(int size, int rocks, int users = 2);

	bool Step(State& state, double rand_num, ACT_TYPE action, double& reward,
		OBS_TYPE& obs) const;
	int NumActions() const;
	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
	void PrintObs(const State& state, OBS_TYPE observation,
		std::ostream& out = std::cout) const;
	ACT_TYPE GenHumanAction(const State& state, int user) const;

private:
	//helper function for Step
	bool MoveRobot(RockSampleState& rockstate, ACT_TYPE action, double& reward) const;

//Maximum entropy MDP
public:
	//(state_i, action, state_k)
	mutable std::vector<std::vector<std::vector<double> > > transition_probabilities_;
	mutable std::unordered_set<int> game_rocks_; //store corresponding game state
	mutable std::vector<double> terminal_state_reward_;
	mutable int end_state_;
	//mutable std::unordered_set<Coord> game_rocks_; // this does not work since set doesn't recognize Coord
	double discount_ = 0.1; //user wants to reach hi as soon as possible

public:
	void InitializeMaxent(const RockSampleState& rockstate, int user) const;
	double ActionProbGivenHI(const RockSampleState& rockstate, ACT_TYPE action) const;
	int NextState(int s, ACT_TYPE a) const;
	double Reward(int s, ACT_TYPE a) const;
	int NumGameStates() const;
	int NumGameActions() const;
	double Softmax(double x1, double x2) const;
};

} // namespace despot

#endif
