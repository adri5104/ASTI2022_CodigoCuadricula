// Provides low-level control of a the speed and direction of a single DC motor
// by means of a motor driver such as the TB6612FNG


#include "Motor.h"
#include <Arduino.h>


// accepts three ints as parameters: 
//    the pin numbers for the direction control pins
//    and the pin number of the pwm pin
Motor::Motor(int dir1, int dir2, int pwm, int pwm_ch, int pwm_frec, int pwm_res)
{
	
	_dir1 = dir1;
	_dir2 = dir2;
	_pwm = pwm;
    pwm_ch_ = pwm_ch;
    pwm_frec_ = pwm_frec;
    pwm_res_ = pwm_res;
}

void Motor::init()
{
	pinMode(_dir1, OUTPUT);
	pinMode(_dir2, OUTPUT);
	pinMode(_pwm, OUTPUT);
    ledcSetup(pwm_ch_, pwm_frec_,pwm_res_);
    ledcAttachPin(_pwm, pwm_ch_);
}

// sets the motor's direction to forward
void Motor::setFwd()
{
	digitalWrite(_dir1, HIGH);
	digitalWrite(_dir2, LOW);
}

// sets the motor's direction to backward
void Motor::setBack()
{
	digitalWrite(_dir1, LOW);
	digitalWrite(_dir2, HIGH);
}

// sets the motor to freewheel
void Motor::setFree()
{
	digitalWrite(_dir1, LOW);
	digitalWrite(_dir2, LOW);
}

// sets the motor to brake
void Motor::setStop()
{
	digitalWrite(_dir1, HIGH);
	digitalWrite(_dir2, HIGH);
}

// accepts an int, the PWM level, as a parameter
// sets the PWM output to the motor to the given int
// level must be between 0 and 255 inclusive
// behavior is undefined if level is outside this range
void Motor::setPWM(uint8_t level)
{
	ledcWrite(pwm_ch_, level);
}