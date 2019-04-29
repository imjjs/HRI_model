#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <despot/interface/world.h>
#include <despot/util/coord.h>
#include <despot/util/grid.h>
#include <vector>



class PlayerWorld: public despot::World{
public:
	virtual bool Connect();
	virtual despot::State* Initialize();
	virtual despot::State* GetCurrentState() const; //@@
	bool ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE& obs);
	static despot::Grid<int> grid_;
	static std::vector<despot::Coord> rock_pos_;
	static int size_;
	static int num_rocks_;
	static int num_users_;
	static despot::Coord current_pos_;
	static std::vector<bool> rock_exists_;
	static int player1_prev_action;
	static int player2_prev_action;
private:
	static void move(int direction);
	static despot::OBS_TYPE HumanActionsEncode(int *human_actions);
};

class Player{
private:
	double human_cooperative_factor;
	double robot_cooperative_factor;
	double noise_level;
	std::vector<double> target_distribution;
	static int rock_num;

	static int l1_distance(const despot::Coord&, const despot::Coord&);
	static std::vector<double> avg_distribution();
    void norm_target_distribution();

public:
	Player(double, double, double);
	static void set_rock_num(int);
	static void updating_hcf();
	void updating_rcf(int);

	void updating_noise();
    void update_target_distribution(const std::vector<double>&);
	int play(const despot::Grid<int>&, const std::vector<despot::Coord>&, const despot::Coord&, const std::vector<bool>&);
	static std::vector<Player*> player_list;
};


#endif /* SIMULATOR_H_ */
