#include <cstdlib>
#include <cassert>
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>
#include "simulator.h"
#include "base_rock_sample.h"

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
	Player* p1 = new Player(.2, .1, 1);
	Player* p2 = new Player(.3, .2, 1.1);
	Player::player_list.push_back(p1);
	Player::player_list.push_back(p2);
	std::cout<<"init PlayerWorld"<<std::endl;
	for(int i = 0; i < rock_pos_.size(); ++i)
		rock_exists_.push_back(true);
	player1_prev_action = 4;
	player2_prev_action = 4;
	std::cout<<"Done"<<std::endl;
	return nullptr;
}

void PlayerWorld::move(int direction){
	current_pos_ = current_pos_ + despot::Compass::DIRECTIONS[direction];
	for(int i = 0; i < rock_pos_.size(); ++i){
		if(current_pos_ == rock_pos_[i] && rock_exists_[i] == true)
			rock_exists_[i] = false;
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
	return true;
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
    double max = *std::max_element(target_distribution.begin(), target_distribution.end());
    double min = *std::min_element(target_distribution.begin(), target_distribution.end());
    for(int i = 0; i < target_distribution.size(); ++i)
        target_distribution[i] = (target_distribution[i] - min) / (max - min) * 100;
}

void Player::updating_hcf(){
	std::vector<double> avg_b = avg_distribution();
	for(int j = 0; j < player_list.size(); ++j){
		for(int i = 0; i < rock_num; ++i){
			double diff = avg_b[i] - player_list[j]->target_distribution[i];
			player_list[j]->target_distribution[i] += player_list[j]->human_cooperative_factor * diff;
		}
	}
}

void Player::updating_rcf(int hinted_rock_index = -1){
	if(hinted_rock_index != -1)
		target_distribution[hinted_rock_index] += INC_BEL * robot_cooperative_factor;
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
			int tmp = 0;
			for(int j = 0; j < player_list.size(); ++j)
				tmp += player_list[j]->target_distribution[j];
			tmp /= player_list.size();
			ret.push_back(tmp);
		}
		return ret;
}

int Player::play(const despot::Grid<int>& grid_, const std::vector<despot::Coord>& rock_pos_, const despot::Coord& current_pos_, const std::vector<bool>& rock_exist_){
	int x = current_pos_.x;
	int y = current_pos_.y;
	//{North, East, South, West, Stay}
	std::vector<despot::Coord> pos_list {despot::Coord(x, y + 1), despot::Coord(x + 1, y),
		despot::Coord(x, y - 1), despot::Coord(x - 1, y), despot::Coord(x, y)};
	std::vector<double> value_list {0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<bool> coord_available_list {true, true, true, true, true};
    assert(value_list.size() == pos_list.size());
	for(int i = 0; i < value_list.size(); ++i){
	    if(!grid_.Inside(pos_list[i]))
	        coord_available_list[i] = false;
	}
	std::default_random_engine generator;
	std::normal_distribution<double> d1(0.0, noise_level);
	for(int i = 0; i < value_list.size(); ++i){
        double value = 0;
	    if(!coord_available_list[i])
            continue;
		for(int j = 0; j < rock_num; ++j){
			int l1_dist = l1_distance(pos_list[i], rock_pos_[j]);
			value += target_distribution[j] * exp(-1.0 * l1_dist) * rock_exist_[j];  //TODO:: Adjust parameter
		}
		value += d1(generator);
		value_list[i] = value;
	}
	return std::distance(value_list.begin(), std::max_element(value_list.begin(), value_list.end()));
}


int Player::l1_distance(const despot::Coord& a_pos, const despot::Coord& b_pos){
	return abs(a_pos.x - b_pos.x) + abs(a_pos.y - b_pos.y);
}
