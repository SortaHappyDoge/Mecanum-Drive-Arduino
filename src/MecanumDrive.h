#ifndef MECANUM_DRIVE_H
#define MECANUM_DRIVE_H

#include <Arduino.h>

class MecanumDrive {
  public:
    /**
     * @param fl_fwd_pwm ccw pwm input for the front left motor in the drivetrain
     * @param fl_bwd_pwm cw pwm input for the front left motor in the drivetrain
     * @param fr_fwd_pwm ccw pwm input for the front right motor in the drivetrain
     * @param fr_bwd_pwm cw pwm input for the front right motor in the drivetrain
     * @param rl_fwd_pwm ccw pwm input for the rear left motor in the drivetrain
     * @param rl_bwd_pwm cw pwm input for the rear left motor in the drivetrain
     * @param rr_fwd_pwm ccw pwm input for the rear right motor in the drivetrain
     * @param rr_bwd_pwm cw pwm input for the rear right motor in the drivetrain
     */
    MecanumDrive(
		int fl_fwd_pwm, int fl_bwd_pwm,
		int fr_fwd_pwm, int fr_bwd_pwm,
		int rl_fwd_pwm, int rl_bwd_pwm,
		int rr_fwd_pwm, int rr_bwd_pwm,
		float speed_limit_percentage = 100,
		float width = 0, float length = 0
	);

    // Set the robot's physical dimensions in meters
    void setDimensions(float width, float length);

    // Set the speed limit for PWM output as percentage (from 0% to 100%)
    void setSpeedLimit(int speed_limit_percentage);

    /**
     * @param x right positive velocity multiplier
     * @param y forward positive velocity multiplier
     */
    void drive(float x, float y, float omega)

    

  private:
    // Motor pin assignments
    int _fl_fwd_pwm, _fl_bwd_pwm;
    int _fr_fwd_pwm, _fr_bwd_pwm;
    int _rl_fwd_pwm, _rl_bwd_pwm;
    int _rr_fwd_pwm, _rr_bwd_pwm;

    // Robot dimensions in meters
    float _length, _width;

    // Speed limit for PWM output as percentage (from 0% to 100%)
    int _speed_limit_percentage;

    // Individual motor velocity values (normalized)
    float _v_fl, _v_fr, _v_rl, _v_rr;

	// Compute and apply motor outputs
    void update();

	// Set motor velocity with forwards and backwards pwm
    void setMotor(int fwd_PWM, int bwd_PWM, float velocity);
};

#endif