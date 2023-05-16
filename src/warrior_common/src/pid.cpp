#include "warrior_common/pid.hpp"
#include <iostream>
PID::PID(uint32_t mode,
		 uint32_t maxout,
		 uint32_t intergral_limit,
		 float kp,
		 float ki,
		 float kd)
{
	IntegralLimit_ = intergral_limit;
	MaxOutput_ = maxout;
	pid_mode_ = mode;

	P = kp;
	I = ki;
	D = kd;
}

void PID::Init(
	uint32_t mode,
	uint32_t maxout,
	uint32_t intergral_limit,
	float kp,
	float ki,
	float kd)
{
	IntegralLimit_ = intergral_limit;
	MaxOutput_ = maxout;
	pid_mode_ = mode;

	P = kp;
	I = ki;
	D = kd;
}

void PID::pid_reset(float kp,
					float ki,
					float kd)
{
	P = kp;
	I = ki;
	D = kd;
}

float PID::pid_calc(float get, float set)
{
	
	get_[NOW] = get;
	set_[NOW] = set;
	err_[NOW] = set - get; // set - measure
	if (max_err_ != 0 && fabs(err_[NOW]) > max_err_)
		return 0;
	if (deadband_ != 0 && fabs(err_[NOW]) < deadband_)
		return 0;

	if (pid_mode_ == POSITION_PID) // 位置式p
	{
		pout_ = P * err_[NOW];
		iout_ += I * err_[NOW];
		dout_ = D * (err_[NOW] - err_[LAST]);
		abs_limit(&(iout_), IntegralLimit_, 0);
		pos_out_ = pout_ + iout_ + dout_;
		abs_limit(&(pos_out_), MaxOutput_, 0);
		last_pos_out_ = pos_out_; // update last time
	}
	else if (pid_mode_ == DELTA_PID) // 增量式P
	{
		pout_ = P * (err_[NOW] - err_[LAST]);
		iout_ = I * err_[NOW];
		dout_ = D * (err_[NOW] - 2 * err_[LAST] + err_[LLAST]);

		abs_limit(&(iout_), IntegralLimit_, 0);
		delta_u_ = pout_ + iout_ + dout_;
		delta_out_ = last_delta_out_ + delta_u_;
		abs_limit(&(delta_out_), MaxOutput_, 0);
		last_delta_out_ = delta_out_; // update last time
	}
	std::cout << "Pout = " << pos_out_ << std::endl;	
	err_[LLAST] = err_[LAST];
	err_[LAST] = err_[NOW];
	get_[LLAST] = get_[LAST];
	get_[LAST] = get_[NOW];
	set_[LLAST] = set_[LAST];
	set_[LAST] = set_[NOW];
	return pid_mode_ == POSITION_PID ? pos_out_ : delta_out_;
}
