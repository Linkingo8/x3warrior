#ifndef __PID_H__
#define __PID_H__
#include <cstdint>
#include <cmath>
/*
------------------------------------------------------------------------------
~ File   : pid.h
~ Author : Fengguang Ji
~ Version: V1.1.0
~ Created: 05/15/2023 00:51:00 AM
~ Brief  :
~ Support:
		  E-Mail : jfg1765@163.com

		  Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:      CPP version

~ Attention  :

~ Changes    :
------------------------------------------------------------------------------
*/

/* PID Mode */
enum
{
	LLAST = 0,
	LAST = 1,
	NOW = 2,

	POSITION_PID,
	DELTA_PID,
};

class PID
{

private:


	float set_[3]; // 目标值,包含NOW， LAST， LLAST上上次
	float get_[3]; // 测量值
	float err_[3]; // 误差

	float pout_; // p输出
	float iout_; // i输出
	float dout_; // d输出

	float pos_out_;		 // 本次位置式输出
	float last_pos_out_; // 上次输出
	float delta_u_;		 // 本次增量值
	float delta_out_;	 // 本次增量式输出 = last_delta_out + delta_u
	float last_delta_out_;

	float max_err_;
	float deadband_; // err < deadband return
	uint32_t pid_mode_;
	uint32_t MaxOutput_;	 // 输出限幅
	uint32_t IntegralLimit_; // 积分限幅

public:

	double P;
	double I;
	double D;

	PID(uint32_t mode,
		uint32_t maxout,
		uint32_t intergral_limit,
		float kp,
		float ki,
		float kd);
	void Init(uint32_t mode,
			  uint32_t maxout,
			  uint32_t intergral_limit,
			  float kp,
			  float ki,
			  float kd);

	/*change PID parameters when the program running...*/
	void pid_reset(float kp,
				   float ki,
				   float kd);
	float pid_calc(float get, float set);
	void abs_limit(float *a, float ABS_MAX, float offset)
	{
		if (*a > ABS_MAX + offset)
			*a = ABS_MAX + offset;
		if (*a < -ABS_MAX + offset)
			*a = -ABS_MAX + offset;
	}
};
#endif
