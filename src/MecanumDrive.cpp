#include "MecanumDrive.h"

MecanumDrive::MecanumDrive(
	int fl_fwd_pwm, int fl_bwd_pwm,
	int fr_fwd_pwm, int fr_bwd_pwm,
	int rl_fwd_pwm, int rl_bwd_pwm,
	int rr_fwd_pwm, int rr_bwd_pwm,
	float speed_limit_percentage,
	float width, float length
)
{
	_fl_fwd_pwm = fl_fwd_pwm;
	_fl_bwd_pwm = fl_bwd_pwm;
	_fr_fwd_pwm = fr_fwd_pwm;
	_fr_bwd_pwm = fr_bwd_pwm;
	_rl_fwd_pwm = rl_fwd_pwm;
	_rl_bwd_pwm = rl_bwd_pwm;
	_rr_fwd_pwm = rr_fwd_pwm;
	_rr_bwd_pwm = rr_bwd_pwm;

	pinMode(_fl_fwd_pwm, OUTPUT);
	pinMode(_fl_bwd_pwm, OUTPUT);
	pinMode(_fr_fwd_pwm, OUTPUT);
	pinMode(_fr_bwd_pwm, OUTPUT);
	pinMode(_rl_fwd_pwm, OUTPUT);
	pinMode(_rl_bwd_pwm, OUTPUT);
	pinMode(_rr_fwd_pwm, OUTPUT);
	pinMode(_rr_bwd_pwm, OUTPUT);

	_speed_limit_percentage = constrain(speed_limit_percentage, 0, 100);

	setDimensions(width, length);
}

void MecanumDrive::setDimensions(float width, float length){
	_length = length;
	_width = width;
}

void MecanumDrive::setSpeedLimit(int speed_limit_percentage){
	_speed_limit_percentage = constrain(speed_limit_percentage, 0, 100);
}


void MecanumDrive::drive(float x, float y, float omega){

	float vmagnitude = sqrt(pow(x, 2) + pow(y, 2));
	
	float vx = x;
	float vy = y;
	float vomega = omega;

	if(vmagnitude > 1){
		vx = vx/vmagnitude;
		vy = vy/vmagnitude;
	}


	// Distance term used for rotational effect
	float LpW = _length/2 + _width/2;
	
	// Calculate each wheel's velocity based on mecanum kinematics
	_v_fl = vy + vx - vomega * LpW;
	_v_fr = vy - vx + vomega * LpW;
	_v_rl = vy - vx - vomega * LpW;
	_v_rr = vy + vx + vomega * LpW;

	// Normalize velocities to ensure they're between -1 and 1
	float max_v = max(max(abs(_v_fl), abs(_v_fr)), max(abs(_v_rl), abs(_v_rr)));
	if (max_v > 1){
		_v_fl /= max_v;
		_v_fr /= max_v;
		_v_rl /= max_v;
		_v_rr /= max_v;
	}

	update();
}

void MecanumDrive::drive(float x, float y, float omega, setPWM setMethod){

	float vmagnitude = sqrt(pow(x, 2) + pow(y, 2));
	
	float vx = x;
	float vy = y;
	float vomega = omega;

	if(vmagnitude > 1){
		vx = vx/vmagnitude;
		vy = vy/vmagnitude;
	}


	// Distance term used for rotational effect
	float LpW = _length/2 + _width/2;
	
	// Calculate each wheel's velocity based on mecanum kinematics
	_v_fl = vy + vx - vomega * LpW;
	_v_fr = vy - vx + vomega * LpW;
	_v_rl = vy - vx - vomega * LpW;
	_v_rr = vy + vx + vomega * LpW;

	// Normalize velocities to ensure they're between -1 and 1
	float max_v = max(max(abs(_v_fl), abs(_v_fr)), max(abs(_v_rl), abs(_v_rr)));
	if (max_v > 1){
		_v_fl /= max_v;
		_v_fr /= max_v;
		_v_rl /= max_v;
		_v_rr /= max_v;
	}

	update(setMethod);
}

// Apply the calculated velocities to each motor
void MecanumDrive::update(){
	setMotor(_fl_fwd_pwm, _fl_bwd_pwm, _v_fl);
	setMotor(_fr_fwd_pwm, _fr_bwd_pwm, _v_fr);
	setMotor(_rl_fwd_pwm, _rl_bwd_pwm, _v_rl);
	setMotor(_rr_fwd_pwm, _rr_bwd_pwm, _v_rr);
}

void MecanumDrive::update(setPWM setMethod){
	setMotor(_fl_fwd_pwm, _fl_bwd_pwm, _v_fl, setMethod);
	setMotor(_fr_fwd_pwm, _fr_bwd_pwm, _v_fr, setMethod);
	setMotor(_rl_fwd_pwm, _rl_bwd_pwm, _v_rl, setMethod);
	setMotor(_rr_fwd_pwm, _rr_bwd_pwm, _v_rr, setMethod);
}

// Helper function to apply a velocity to one motor
void MecanumDrive::setMotor(int fwd_pwm, int bwd_pwm, float velocity){
	int pwm = int(abs(velocity) * (_speed_limit_percentage/100.0f * 255.0f));	// Scale velocity to PWM
	if(velocity >= 0){
		analogWrite(fwd_pwm, pwm);
		analogWrite(bwd_pwm, 0);
	}
	else{
		analogWrite(fwd_pwm, 0);
		analogWrite(bwd_pwm, pwm);
	}
}

// Overload for use with external PWM drivers
void MecanumDrive::setMotor(int fwd_pwm, int bwd_pwm, float velocity, setPWM setMethod){
	int pwm = int(abs(velocity) * (_speed_limit_percentage/100.0f * 255.0f));	// Scale velocity to PWM
	if(velocity >= 0){
		setMethod(fwd_pwm, pwm);
		setMethod(bwd_pwm, 0);
	}
	else{
		setMethod(fwd_pwm, 0);
		setMethod(bwd_pwm, pwm);
	}
}
