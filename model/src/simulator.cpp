#include <cstdlib>
#include <cassert>
#include <chrono>
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>
#include "simulator.h"
#include "base_rock_sample.h"
#include <bitset>

#define DEFAULT_BELIEVE 50
#define INC_BEL     100
#define INC_NOISE  0.3

despot::Grid<int> PlayerWorld::grid_;
std::vector<despot::Coord> PlayerWorld::rock_pos_;
int PlayerWorld::size_;
int PlayerWorld::num_rocks_;
int PlayerWorld::num_users_;
despot::Coord PlayerWorld::current_pos_;
std::vector<bool> PlayerWorld::rock_exists_;
int PlayerWorld::player1_prev_action;
int PlayerWorld::player2_prev_action;

std::vector<Player*> Player::player_list;
int Player::rock_num;

bool PlayerWorld::Connect(){
	return true;
}

despot::OBS_TYPE PlayerWorld::HumanActionsEncode(int *human_actions){
	//human_actions is represented by observation enum
	//each human action is encoded by obs_bits = 5 bits
	//human 0 is the lower bit, if his/her action is E_STAY, it is 10000, 
	//  if his/her action is Compass::EAST, it is 00010
	const int obs_bits = 5;
	despot::OBS_TYPE obs(0);
	for (int i = num_users_-1; i >= 0; --i) {
		obs = (obs | (1 << human_actions[i])); //SetFlag for OBS_TYPE
		if (i == 0) break;
		obs = (obs << obs_bits);
	}

	return obs;
}

despot::State* PlayerWorld::Initialize(){
	//@@ Modify player property
	Player::set_rock_num(rock_pos_.size());
	Player* p1 = new Player(0.1, 0.1, 10);
	Player* p2 = new Player(0.1, 0.1, 10);
	Player::player_list.push_back(p1);

	//@@ Modify player initial target belief, range [0, 100]
	std::vector<double> p1_initial_target_belief (num_rocks_, 0.0);
	p1_initial_target_belief[2] = 50;
	p1->update_target_distribution(p1_initial_target_belief);
	std::vector<double> p2_initial_target_belief (num_rocks_, 0.0);
	p2_initial_target_belief[3] = 50;
	p2->update_target_distribution(p2_initial_target_belief);

	Player::player_list.push_back(p2);
	std::cout<<"init PlayerWorld"<<std::endl;
	player1_prev_action = 4;
	player2_prev_action = 4;
	std::cout<<"Done"<<std::endl;
	return nullptr;
}

despot::State* PlayerWorld::GetCurrentState() const {
	const int hi_bits = 4, adapt_bits = 3, action_bits = 3;
	int bitsPerUser = adapt_bits + 1 + hi_bits;
	int state = (1 << (action_bits + num_rocks_ + num_users_ * bitsPerUser)) *
			(current_pos_.y * grid_.xsize() + current_pos_.x);
	despot::State* rockstate = new despot::RockSampleState(state);

	//Set rock states
	for (int rock = 0; rock < num_rocks_; ++rock)
		if (rock_exists_[rock])
			despot::SetFlag(rockstate->state_id, rock + num_users_ * bitsPerUser);
		else
			despot::UnsetFlag(rockstate->state_id, rock + num_users_ * bitsPerUser);

	//Set human actions
	int action;
	if (player1_prev_action == player2_prev_action)
		action = player1_prev_action;
	else
		action = 4;

	std::string action_chosen = std::bitset<action_bits>(action).to_string();
	int pos = (num_rocks_ + num_users_ * bitsPerUser);
	for (int i = action_bits-1; i >= 0; --i) {
		if (action_chosen[i] == '1')
			despot::SetFlag(rockstate->state_id, pos);
		else
			despot::UnsetFlag(rockstate->state_id, pos);
		++pos;
	}

	return rockstate;
}

void PlayerWorld::move(int action){
	switch(action) {
	case despot::Compass::EAST:
		if (current_pos_.x + 1 < size_)
			current_pos_.x += 1;
		break;
	case despot::Compass::NORTH:
		if (current_pos_.y + 1 < size_)
			current_pos_.y += 1;
		break;
	case despot::Compass::SOUTH:
		if (current_pos_.y - 1 >= 0)
			current_pos_.y -= 1;
		break;
	case despot::Compass::WEST:
		if (current_pos_.x - 1 >= 0)
			current_pos_.x -= 1;
		break;
	}

	for(int i = 0; i < rock_pos_.size(); ++i){
		if(current_pos_ == rock_pos_[i] && rock_exists_[i] == true){
			rock_exists_[i] = false;
			for(auto p: Player::player_list)
				p->rock_reached(i);
		}
	}
}

bool PlayerWorld::ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE& obs){
	std::cout<<"PlayerWorld Exec"<<std::endl;
	if(despot::BaseRockSample::E_SLAVE > action){
		move(action); 
	}else if(despot::BaseRockSample::E_SLAVE == action){
		if(player1_prev_action == player2_prev_action)
			move(player1_prev_action);
	}else if(despot::BaseRockSample::E_HI == action){
		int min_dist = 999999;
		int selected_rock = -1;
		for(int i  = 0; i < rock_pos_.size(); ++i){
			if(rock_exists_[i] == false)
				continue;
			int dist = despot::Coord::ManhattanDistance(current_pos_, rock_pos_[i]);
			if(dist < min_dist){
				min_dist = dist;
				selected_rock = i;
			}
		}
		Player::player_list[0]->updating_rcf(selected_rock);
		Player::player_list[1]->updating_rcf(selected_rock);
	}else if(despot::BaseRockSample::E_HI + 2 > action)
		Player::player_list[action - despot::BaseRockSample::E_HI]->updating_noise();
	else  //should not be here
		assert(false);
	int p1_action = Player::player_list[0]->play(grid_, rock_pos_, current_pos_, rock_exists_);
	int p2_action = Player::player_list[1]->play(grid_, rock_pos_, current_pos_, rock_exists_);
	if(p1_action != p2_action)
		Player::updating_hcf();
	int human_actions[2];
	human_actions[0] = p1_action;
	human_actions[1] = p2_action;
	obs = HumanActionsEncode(human_actions);
	player1_prev_action = p1_action;
	player2_prev_action = p2_action;
	std::cout<<"PlayerWorld Exec Done"<<std::endl;

	for (int rock = 0; rock < num_rocks_; ++rock)
		if (rock_exists_[rock])
			return false;

	return true;
}

void Player::rock_reached(int idx){
	target_distribution[idx] = -1;
	norm_target_distribution();
}

Player::Player(double _hcf, double _rcf,
		double _nl):
		human_cooperative_factor(_hcf), robot_cooperative_factor(_rcf), noise_level(_nl){
		for(int i = 0; i < rock_num; ++i)
			target_distribution.push_back(DEFAULT_BELIEVE);
}

void Player::update_target_distribution(const std::vector<double>& _target_distribution){
    target_distribution = _target_distribution;
    norm_target_distribution();
}

void Player::norm_target_distribution(){
    double max = -200;
    double min = 200;
	for(int i = 0; i < target_distribution.size(); ++i){
		if(false == PlayerWorld::rock_exists_[i]){
			assert(target_distribution[i] - 1.0 < 1e-3);
			continue;
		}
		if(max < target_distribution[i])
			max = target_distribution[i];
		if(min > target_distribution[i])
			min = target_distribution[i];
	}
	if(max == min){
		for(int i = 0; i < target_distribution.size(); ++i){
			if(false == PlayerWorld::rock_exists_[i])
				continue;
			target_distribution[i] = 50;
		}
	}else{
    	for(int i = 0; i < target_distribution.size(); ++i){
			if(false == PlayerWorld::rock_exists_[i])
				continue;
        	target_distribution[i] = (target_distribution[i] - min) / (max - min) * 100;
		}
	}
}

void Player::updating_hcf(){
	std::vector<double> avg_b = avg_distribution();
	for(int j = 0; j < player_list.size(); ++j){
		for(int i = 0; i < rock_num; ++i){
			double diff = avg_b[i] - player_list[j]->target_distribution[i];
			player_list[j]->target_distribution[i] += player_list[j]->human_cooperative_factor * diff;
		}
	}
	for(int j = 0; j < player_list.size(); ++j)
		player_list[j]->norm_target_distribution();
}

void Player::updating_rcf(int hinted_rock_index = -1){
	if(hinted_rock_index != -1)
		target_distribution[hinted_rock_index] += INC_BEL * robot_cooperative_factor;
		norm_target_distribution();
}

void Player::updating_noise(){
	noise_level *= INC_NOISE;
}

void Player::set_rock_num(int _rock_num){
	rock_num = _rock_num;
}

std::vector<double> Player::avg_distribution(){
		std::vector<double> ret;
		for(int i = 0; i < rock_num; ++i){
			double tmp = 0;
			for(int j = 0; j < player_list.size(); ++j){
				tmp += player_list[j]->target_distribution[i];
			}
			tmp /= player_list.size();
			ret.push_back(tmp);
		}
		return ret;
}

int Player::play(const despot::Grid<int>& grid_, const std::vector<despot::Coord>& rock_pos_, const despot::Coord& current_pos_, const std::vector<bool>& rock_exist_){
	std::cout<<"Player start"<<std::endl;
	int x = current_pos_.x;
	int y = current_pos_.y;
	//{North, East, South, West, Stay}
	std::cout<<"current pos:("<<x<<','<<y<<')'<<std::endl;
	std::cout<<"rock pos:";
	for(auto &tmp: rock_pos_)
		std::cout<<'('<<tmp.x<<','<<tmp.y<<')';
	std::cout<<std::endl;
	std::vector<despot::Coord> pos_list {despot::Coord(x, y + 1), despot::Coord(x + 1, y),
		despot::Coord(x, y - 1), despot::Coord(x - 1, y), despot::Coord(x, y)};
	std::vector<double> value_list {0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<bool> coord_available_list {true, true, true, true, true};
    assert(value_list.size() == pos_list.size());
	for(int i = 0; i < value_list.size(); ++i){
	    if(!grid_.Inside(pos_list[i]))
	        coord_available_list[i] = false;
	}
	std::cout<<"target belief:";
	for(auto &tmp: target_distribution)
		std::cout<<tmp<<", ";
	std::cout<<std::endl;
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);

	std::normal_distribution<double> d1(0.0, noise_level);
	for(int i = 0; i < value_list.size(); ++i){
        double value = 0;
	    if(!coord_available_list[i]){
            value_list[i] = -1000;
			continue;
		}
		for(int j = 0; j < rock_num; ++j){
			int l1_dist = l1_distance(pos_list[i], rock_pos_[j]);
			//std::cout<<"i:"<<i<<",j:"<<j<<",expl1:"<< exp(-1.0 * l1_dist)<<",exist:"<<rock_exist_[j]<<",target:"<<target_distribution[j]<<
			//",result:"<<target_distribution[j] * exp(-1.0 * l1_dist) * rock_exist_[j]<<std::endl;
			value += target_distribution[j] * exp(-1.0 * l1_dist) * rock_exist_[j];  //TODO:: Adjust parameter
		}
		double a = d1(generator);
		//std::cout<<"i:"<<i<<"random:"<<a<<std::endl;
		value += a;
		value_list[i] = value;
	}
	std::cout<<"target value:";
	for(auto &tmp: value_list)
		std::cout<<tmp<<", ";
	std::cout<<"Player end"<<std::endl;
	return std::distance(value_list.begin(), std::max_element(value_list.begin(), value_list.end()));
}




int Player::l1_distance(const despot::Coord& a_pos, const despot::Coord& b_pos){
	return abs(a_pos.x - b_pos.x) + abs(a_pos.y - b_pos.y);
}
