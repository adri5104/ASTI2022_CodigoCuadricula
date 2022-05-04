#ifndef Motor_h
#define Motor_h

#include "Config.h"

class Motor
{
public:
	Motor(int dir1, int dir2, int pwm, int pwm_ch, int pwm_frec, int pwm_res);
	void init();
	void setFwd();
	void setBack();
	void setFree();
	void setStop();
	void setPWM(uint8_t level);
private:
	int _dir1, _dir2, _pwm, pwm_ch_, pwm_frec_, pwm_res_; //Atributos para almacenar pines y movidas del pwm
     //Valor actual de la salida pwm en cada instante (0 - 255)
    
};

#endif