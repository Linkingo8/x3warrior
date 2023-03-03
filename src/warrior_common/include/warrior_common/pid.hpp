#ifndef PID_H
#define PID_H

class MiniPID{
public:
	MiniPID(double, double, double);
	MiniPID(double, double, double, double);
	void setP(double);
	void setI(double);
	void setD(double);
	void setF(double);
	void setPID(double, double, double);
	void setPID(double, double, double, double);
	void setMaxIOutput(double);
	void setOutputLimits(double);
	void setOutputLimits(double,double);
	void setDirection(bool);
	void setSetpoint(double);
	void reset();
	void setOutputRampRate(double);
	void setSetpointRange(double);
	void setOutputFilter(double);
	double getOutput();
	double getOutput(double);
	double getOutput(double, double);
	double P;
	double I;
	double D;
	double F;

private:
	double clamp(double, double, double);
	bool bounded(double, double, double);
	void checkSigns();
	void init();


	double maxIOutput;
	double maxError;
	double errorSum;
	double error_now;
	double error_last;
	double maxOutput; 
	double minOutput;

	double setpoint_now;
	double setpoint;
	double actual_now;
	double lastActual;

	bool firstRun;
	bool reversed;

	double outputRampRate;
	double lastOutput;

	double outputFilter;

	double setpointRange;
};
#endif
