#ifndef BASEROCKSAMPLE_H
#define BASEROCKSAMPLE_H

#include <despot/interface/pomdp.h>
#include <despot/solver/pomcp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/grid.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>
#include <unordered_map>

namespace despot {

/* =============================================================================
 * RockSampleState class
 * =============================================================================*/

class RockSampleState: public State {
public:
	RockSampleState();
	RockSampleState(int _state_id);

	std::string text() const;
};

/* =============================================================================
 * BaseRockSample class
 * TODO LIST
 *   Optional Functions, custom belief & custom bounds
 * =============================================================================*/

class BaseRockSample: public DSPOMDP {
	/*
	public MDP,
	public BeliefMDP,
	public StateIndexer,
	public StatePolicy*/

	/*
	// OPTIONAL FUNCTIONS: Custom Bounds
	friend class RockSampleENTScenarioLowerBound;
	friend class RockSampleMMAPStateScenarioLowerBound;
	friend class RockSampleEastScenarioLowerBound;
	friend class RockSampleParticleUpperBound1;
	friend class RockSampleParticleUpperBound2;
	friend class RockSampleMDPParticleUpperBound;
	friend class RockSampleApproxParticleUpperBound;
	friend class RockSampleEastBeliefPolicy;
	friend class RockSampleMDPBeliefUpperBound;
	friend class RockSamplePOMCPPrior;
	*/

protected:
	Grid<int> grid_;
	std::vector<Coord> rock_pos_;
	int size_, num_rocks_, num_users_;
	Coord start_pos_;
	//hi_bits determines max num_rocks
	static const int hi_bits = 4, adapt_bits = 3, obs_bits = 5;
	//save human action for robot to follow
	static const int action_bits = 3;
	const int bitsPerUser;

	RockSampleState* rock_state_; //although named rock_state_, this is the state including observable and partially observable state
	mutable MemoryPool<RockSampleState> memory_pool_;

	std::vector<RockSampleState*> states_;
protected:
	void InitGeneral();
	void Init_4_4();
	void Init_5_5();
	void Init_5_7();
	void Init_7_8();
	void Init_11_11();
	void InitStates();

	/*
	std::vector<std::vector<std::vector<State> > > transition_probabilities_;
	std::vector<std::vector<double> > alpha_vectors_; // For blind policy
	mutable std::vector<ValuedAction> mdp_policy_;
	*/

public:
	//Observation enum for human actions (for one human)
	/*
	  For each user, observation is human actions {North, East, South, West, Stay}
	  Compass enum has {NORTH, EAST, SOUTH, WEST} as {0, 1, 2, 3}
	*/
	enum {
		E_STAY = 4
	};

	//Action enum for robot
	/*
      {North, East, South, West, Follow human command, HI, HA}
	  (1) Robot takes over control, highest cost
        action < E_SlAVE, {North, East, South, West}
	  (2) Tele-operation mode, robot moves based on combined human actions, no cost
	    action == E_SLAVE, {Follow human command}
	  (3) Robot influence human, human-robot collaboration, medium cost
	    (a) robot influence group, increase collaboration
		  action == E_HI, {HI}
	    (b) robot influence individual, increase engagement
		  action > E_HI, {HA0, HA1, ...} for each user, action - E_HI - 1 == user_id
	*/
	enum {
		E_SLAVE = 4,
		E_HI = 5
	};

	//Adaptability enum for human
	enum {
		E_00, //0
		E_25, //0.25
		E_50, //0.5
		E_75, //0.75
		E_100  //1
	};

public:
	BaseRockSample(std::string map);
	BaseRockSample(int size, int rocks, int users = 2);

	/* ESSENTIAL FUNCTIONS: Deterministic simulative model. */
	virtual bool Step(State& state, double rand_num, ACT_TYPE action,
		double& reward, OBS_TYPE& obs) const = 0;

	/* ESSENTIAL FUNCTIONS: Returns total number of actions. */
	virtual int NumActions() const = 0;

	/* ESSENTIAL FUNCTIONS: Functions related to beliefs and starting states. */
	virtual double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const = 0;
	
	/*
	const std::vector<State>& TransitionProbability(int s, ACT_TYPE a) const; 
	int NextState(int s, ACT_TYPE a) const;
	double Reward(int s, ACT_TYPE a) const;
	*/

	/* ESSENTIAL FUNCTIONS: Beliefs and Starting States */
	State* CreateStartState(std::string type = "DEFAULT") const;
	/*
	std::vector<State*> InitialParticleSet() const;
	std::vector<State*> NoisyInitialParticleSet() const;
	*/
	Belief* InitialBelief(const State* start, std::string type = "PARTICLE") const;

	/* ESSENTIAL FUNCTIONS: Bound-related functions. */
	inline double GetMaxReward() const {
		return 50;
	}

	/*
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;
	BeliefUpperBound* CreateBeliefUpperBound(std::string name = "DEFAULT") const;
	*/

	inline ValuedAction GetBestAction() const {
		return ValuedAction(E_SLAVE, -100);
	}

	/*
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;
	BeliefLowerBound* CreateBeliefLowerBound(std::string name = "DEFAULT") const;

	POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;
	*/

	/* ESSENTIAL FUNCTIONS: Display functions. */
	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const = 0;
	void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

	/* ESSENTIAL FUNCTIONS: Memory management. */
	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	// Helper functions.
	/*
	Belief* Tau(const Belief* belief, ACT_TYPE action, OBS_TYPE obs) const;
	void Observe(const Belief* belief, ACT_TYPE action, std::map<OBS_TYPE, double>& obss) const;
	double StepReward(const Belief* belief, ACT_TYPE action) const;
	*/

	int NumStates() const;
	const State* GetState(int index) const;
	int GetIndex(const State* state) const;

	/*
	inline int GetAction(const State& tagstate) const {
		return 0;
	}
	*/

	std::vector<double> HIProbs(const State* state) const;
	void SetRock(State* state, int rock) const;
	bool GetRock(const State* state, int rock) const;
	void TakeRock(State* state, int rock) const;
	ACT_TYPE GetHumanAction(const State* state) const;
	void SetHumanAction(State* state, ACT_TYPE action) const;
	int GetRobPosIndex(const State* state) const;
	Coord GetRobPos(const State* state) const;
	double GetAdaptability(const State* state, int user) const;
	void SetAdaptability(State* state, int user, int adapt) const;
	bool GetHA(const State* state, int user) const;
	void SetHA(State* state, int user) const;
	void UnsetHA(State* state, int user) const;
	int GetHIIndex(const State* state, int user) const;
	void SetHI(State* state, int user, int hi) const;
	Coord GetHI(const State* state, int user) const;
	int GetX(const State* state) const;
	void IncX(State* state) const;
	void DecX(State* state) const;
	int GetY(const State* state) const;
	void IncY(State* state) const;
	void DecY(State* state) const;
	double CosineSimilarity(Coord R, Coord A, Coord B) const;
	double Similarity(const State* state) const;
	int ClosestRockAmongAll(const State* state) const;
	int ClosestRockAmongCandid(const std::vector<Coord>& rocks, Coord robot, std::unordered_map<int, int> rockIdxMap) const;
	
	//Encode an array of human actions to OBS_TYPE
	OBS_TYPE HumanActionsEncode(int *human_actions) const;
	//Decode an OBS_TYPE to an array of human actions
	int* HumanActionsDecode(OBS_TYPE human_actions_obs) const;
	//Combined human actions for robot to follow based on prev_human_actions_obs
	ACT_TYPE HumanActions(OBS_TYPE human_actions_obs) const;

	inline int l1_distance(const despot::Coord& a_pos, const despot::Coord& b_pos) const {
		return abs(a_pos.x - b_pos.x) + abs(a_pos.y - b_pos.y);
	}

protected:
	void InitializeTransitions();
	Coord IndexToCoord(int pos) const;
	int CoordToIndex(Coord c) const;
	//std::vector<ValuedAction>& ComputeOptimalSamplingPolicy() const;
	//RockSampleState* MajorityRockSampleState(const std::vector<State*>& particles) const;
};

} // namespace despot

#endif
