#include "rock_sample.h"
#include "simulator.h"
#include <unordered_map>

using namespace std;

namespace despot {


RockSample::RockSample(string map) :
	BaseRockSample(map) { }

RockSample::RockSample(int size, int rocks, int users) :
	BaseRockSample(size, rocks, users) { }


bool RockSample::Step(State& state, double rand_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
	RockSampleState& rockstate = static_cast<RockSampleState&>(state);
	reward = 0;
	bool res(false);

	vector<bool> rock_exist;
	for (int rock = 0; rock < num_rocks_; ++rock)
		rock_exist.push_back(GetRock(&rockstate, rock));

	if (action < E_SLAVE) { // Move
		//additional penalty for ignoring human
		reward -= 20;

		//if human already engaged and hcl is high, robot should not move, let human play
		bool flag = true;
		for (int user = 0; user < num_users_; ++user) {
			if (!GetHA(&rockstate, user))
				flag = false;
		}
		if (Similarity(&rockstate) < 0.50)
			flag = false;
		if (flag) reward -= 100;

		res = MoveRobot(rockstate, action, reward);
	} else if (action == E_SLAVE) {
		//Determine user action then move
		ACT_TYPE human_action = GetHumanAction(&rockstate);
		if (human_action < E_STAY) { // Move
			res = MoveRobot(rockstate, human_action, reward);
		}
	} else if (action == E_HI) { 
        reward -= 10;
		for (int user = 0; user < num_users_; ++user) {
			//when HA is false for any user, robot should not address HI
			if (!GetHA(&rockstate, user)) {
				reward -= 100; 
			}
		}
		//get group hcl
		double hcl = Similarity(&rockstate); // in range [-1, 1]
		if (hcl >= 0.50) { //@@ think about this value
			// hcl is already high enough
			reward -= 100;
		} else {
			//@@ Assume when robot tries to influence HIs, it provides a specific HI
			// to let all users adapt their HI to this HI. 
			// In this specific example, I choose to use the closest rock that each user
			// intend to grab as the new HI

			// Find rocks that users are interested in
			vector<Coord> candid_rocks;
			unordered_map<int, int> rockIdxMap;
			int count (0);
			for (int user = 0; user < num_users_; ++user) {
				//@@ think of situations when HI is a rock that has just been picked up
				//Okay, now there is no extra exit state, when all rocks are picked up, end of game
				//This gaurantee that if game not end, there is at least one rock left for pick up.
				int rockID = GetHIIndex(&rockstate, user);
				if (rock_exist[rockID]) {
					candid_rocks.push_back(rock_pos_[rockID]);
					rockIdxMap[count] = rockID;
					++count;
				}
			}

			// Find closest rock as intention
			int chosen;
			if (candid_rocks.empty())
				chosen = ClosestRockAmongAll(&rockstate);
			else
				chosen = ClosestRockAmongCandid(candid_rocks, GetRobPos(&rockstate), rockIdxMap);
						
			// For each user, they switch their HI to this new HI with probability of adapt
			for (int user = 0; user < num_users_; ++user) {
				Player::player_list[user]->updating_rcf(chosen);
				if (GetHIIndex(&rockstate, user) == chosen) continue;
				double adapt = GetAdaptability(&rockstate, user);
				if (rand_num > (1 - adapt)) {
					//user switches their HI
					SetHI(&rockstate, user, chosen);
					reward += 10; //@@ think about this value.
				} // otherwise user does not change their mind.
			}
		}
	} else if (action > E_HI) {
		reward -= 10;
		int user = action - E_HI - 1; //which user
		if (GetHA(&rockstate, user)) {
			// this user is already engaged
			reward -= 100;
		} else {
			Player::player_list[user]->updating_noise();

			double adapt = GetAdaptability(&rockstate, user);
			if (rand_num > (1 - adapt)) {
				//user becomes engaged with probability adapt
				SetHA(&rockstate, user);
				reward += 10; //@@ think about this value.
			} //otherwise user stay not engaged
		}
	}

	//Get new human actions as observation
	// This is based on simulation of human behaviors
	//@@ TODO
	int p1_action = Player::player_list[0]->play(grid_, rock_pos_, GetRobPos(&rockstate), rock_exist);
	int p2_action = Player::player_list[1]->play(grid_, rock_pos_, GetRobPos(&rockstate), rock_exist);
	int human_actions[2];//get human_action from human behavior simulation code
	human_actions[0] = p1_action;
	human_actions[1] = p2_action;
	obs = HumanActionsEncode(human_actions);
	ACT_TYPE human_action = HumanActions(obs);
	SetHumanAction(&rockstate, human_action);

	return res;
}

int RockSample::NumActions() const {
	// robot controls itself actions {north, south, east, west}
	// robot follow human action {follow}
	// robot influence HI action {HI_group}
	// robot influence HA actions {HA_user0, HA_user1, ...}
	return 4 + 1 + 1 + num_users_;
}

double RockSample::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
	const RockSampleState& rockstate =
		static_cast<const RockSampleState&>(state);

	int *human_actions = HumanActionsDecode(obs);
	vector<double> probs; //store observation probability for each user
	for (int user = 0; user < num_users_; ++user) {
		if (!GetHA(&rockstate, user)) { //user is not engaged, and takes actions randomly
			double prob = static_cast<double>(1) / (E_STAY + 1);
			probs.push_back(prob);
		} else { //based on the principle of maximum entropy
			//int hi = GetHIIndex(&rockstate, user);
			InitializeMaxent(rockstate, user);
			probs.push_back(ActionProbGivenHI(rockstate, human_actions[user]));
		}
	}
	delete[] human_actions;

	//multiply probs for each user for final results
	double res = 1.0;
	for (const auto prob : probs) {
		res *= prob;
	}
	return res;
}

void RockSample::PrintObs(const State& state, OBS_TYPE observation,
	ostream& out) const {
	int *human_actions = HumanActionsDecode(observation);
	for (int user = 0; user < num_users_; ++user) {
		out << "user " << user << ": ";
		switch (human_actions[user]) {
			case Compass::EAST:
				out << "East" << endl;
				break;
			case Compass::NORTH:
				out << "North" << endl;
				break;
			case Compass::SOUTH:
				out << "South" << endl;
				break;
			case Compass::WEST:
				out << "West" << endl;
				break;
			case E_STAY:
				out << "Stay" << endl;
				break;
		}
	}
	delete[] human_actions;
}

bool RockSample::MoveRobot(RockSampleState& rockstate, ACT_TYPE action, double& reward) const {
	switch (action) {
	case Compass::EAST:
		if (GetX(&rockstate) + 1 < size_)
			IncX(&rockstate);
		else
			reward -= 100;
		break;

	case Compass::NORTH:
		if (GetY(&rockstate) + 1 < size_)
			IncY(&rockstate);
		else
			reward -= 100;
		break;

	case Compass::SOUTH:
		if (GetY(&rockstate) - 1 >= 0)
			DecY(&rockstate);
		else
			reward -= 100;
		break;

	case Compass::WEST:
		if (GetX(&rockstate) - 1 >= 0)
			DecX(&rockstate);
		else
			reward -= 100;
		break;
	}

	//if robot pos == rock position, pick up the rock
	for (int rock = 0; rock < num_rocks_; ++rock) {
		if (GetRock(&rockstate, rock) && GetX(&rockstate) == rock_pos_[rock].x &&
			GetY(&rockstate) == rock_pos_[rock].y) {
			reward += 10;
			TakeRock(&rockstate, rock);
			//check if end game
			int countRock = 0;
			for (int i = 0; i < num_rocks_; ++i) {
				if (GetRock(&rockstate, i)) ++countRock;
			}
			if (countRock == 0) {
				reward += 10;
				return true;
			}
			//update belief for each user
			for (int user = 0; user < num_users_; ++user) {
				if (GetHIIndex(&rockstate, user) == rock) {
					vector<double> probs = HIProbs(&rockstate);
					vector<double> cdf (num_rocks_ + 1, 0.0);
					for (int i = 0; i < num_rocks_; ++i)
						cdf[i + 1] = cdf[i] + probs[i];
					int rand_num = Random::RANDOM.NextInt(100);
					for (int i = 0; i < num_rocks_; ++i) {
						if (rand_num >= cdf[i] * 100 && rand_num < cdf[i + 1] * 100) {
							SetHI(&rockstate, user, i);
							break;
						}
					}
				}
			}

			break;
		}
	}
	return false;
}

void RockSample::InitializeMaxent(const RockSampleState& rockstate, int user) const {
	int num_states = NumGameStates(), num_actions = NumGameActions();

	//get end_state
	end_state_ = CoordToIndex(GetHI(&rockstate, user));
	terminal_state_reward_.resize(num_states);
	fill(terminal_state_reward_.begin(),
	          terminal_state_reward_.end(), 0);
	terminal_state_reward_[end_state_] = 100;

	//compute transition probability
	transition_probabilities_.resize(num_states);
	for (int s = 0; s < num_states; ++s) {
		transition_probabilities_[s].resize(num_actions);
		for (int a = 0; a < num_actions; ++a) {
			transition_probabilities_[s][a].resize(num_states);
			fill(transition_probabilities_[s][a].begin(), 
					  transition_probabilities_[s][a].end(), 0);
			int s_prime = NextState(s, a);
			transition_probabilities_[s][a][s_prime] = 1.0; //deterministic MDP
		}
	}

	//initialize game_rocks
	game_rocks_.clear();
	for (int rock = 0; rock < num_rocks_; ++rock) {
		if (GetRock(&rockstate, rock)) game_rocks_.insert(CoordToIndex(rock_pos_[rock]));
	}
}

double RockSample::ActionProbGivenHI(const RockSampleState& rockstate, ACT_TYPE action) const {
	//Ziebart's PhD thesis (algorithm 9.1)
	//softened version value iteration
	int num_states = NumGameStates(), num_actions = NumGameActions();
	vector<double> V (num_states, -1 * INFINITY);
	bool converge = false;
	while (!converge) {
		vector<double> new_V = terminal_state_reward_;
		for (int a = 0; a < num_actions; ++a) {
			for (int s = 0; s < num_states; ++s) {
				double sum = 0;
				for (int s_prime = 0; s_prime < num_states; ++s_prime) {
					sum += transition_probabilities_[s][a][s_prime] * V[s_prime];
				}
				new_V[s] = Softmax(new_V[s], Reward(s, a) + discount_ * sum);
			}
		}

		//@@ new_V diverge? need to z-score it?
		//Answer: after z-score, this no longer works.
		
		converge = true;
		//check convergence
		for (int s = 0; s < num_states; ++s) {
			if (abs(V[s] - new_V[s]) > 1e-4) {
				converge = false;
				break;
			}
		}
		V = new_V;
	}

	//return Q(s, a) with probability
	int cur_state = CoordToIndex(GetRobPos(&rockstate));
	vector<double> Qs (num_actions, 0.0);
	for (int a = 0; a < num_actions; ++a) {
		double sum = 0;
		for (int s = 0; s < num_states; ++s) {
			sum += transition_probabilities_[cur_state][a][s] * V[s];
		}
		Qs[a] = discount_ * sum + Reward(cur_state, a);
	}

	//softmax to get probability
	double maxQ = Qs[0];
	for (const auto& elem : Qs)
		maxQ = max(maxQ, elem);
	for (auto& elem : Qs)
		elem -= maxQ;
	double expSum (0.0);
	for (const auto& elem : Qs)
		expSum += exp(elem);
	for (auto& elem : Qs)
		elem = exp(elem) / expSum;
	
	return Qs[action];
}

int RockSample::NextState(int s, ACT_TYPE a) const {
	if (s == end_state_) return end_state_;

	Coord rob_pos = IndexToCoord(s);
	if (a < E_STAY) {
		rob_pos += Compass::DIRECTIONS[a];
		if (grid_.Inside(rob_pos)) {
			return CoordToIndex(rob_pos);
		} else {
			return s;
		}
	} 
	
	return s;
}

double RockSample::Reward(int s, ACT_TYPE a) const {
	if (s == end_state_) return 0;

	if (a < E_STAY) {
		Coord rob_pos = IndexToCoord(s);
		rob_pos += Compass::DIRECTIONS[a];
		if (grid_.Inside(rob_pos)) {
			//move to end state
			int new_s = CoordToIndex(rob_pos);
			if (new_s == end_state_) return 100;
			//move on to a rock, then pickup and get reward, else 0 reward
			if (game_rocks_.count(new_s)) {
				game_rocks_.erase(new_s);
				return 10;
			}
			return 0;
		} else {
			return -100; //move out of grid
		}
	}
	return 0;
}

int RockSample::NumGameStates() const {
	return grid_.xsize() * grid_.ysize();
}

int RockSample::NumGameActions() const {
	return E_STAY + 1;
}

double RockSample::Softmax(double x1, double x2) const {
	double maxV = max(x1, x2);
	double minV = min(x1, x2);
	return maxV + log(1 + exp(minV - maxV));
}

} // namespace despot
