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
	bool ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE& obs);
}

class Player{
private:
	double human_cooperative_factor;
	double robot_cooperative_factor;
	double noise_level;
	std::vector<double> target_distribution;
	static std::vector<Player*> player_list;
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
};


#endif /* SIMULATOR_H_ */
