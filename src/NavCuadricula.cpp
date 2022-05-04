#include "NavCuadricula.h"

NavCuadricula::NavCuadricula(QTRSensors*  a, Motor* b, Motor* c )
{
    myQtr = a;
    MisMotores[RIGHT] = b;
    MisMotores[LEFT] = c;

    PID::PIDParameters<float> aux(1.0,0.0,0.0);
    parametros_PID = aux;
    myPID = new PID::PIDController<float>(aux);

    vel_max = 255;
    vel_base = 100;
    myPID->Setpoint = 0.0;
    myPID->SetOutputLimits(-255, 255);
    myPID->TurnOn();

    myumbral = THRESHOLD;
    
}

void NavCuadricula::parar()
{
    int i;
    for(i = 0; i< 2; i++)
    {
        MisMotores[i]->setPWM(0);
        MisMotores[i]->setStop();
    }
}

void NavCuadricula::retroceder()
{
    MisMotores[RIGHT]->setBack();
    MisMotores[LEFT]->setBack();
    MisMotores[RIGHT]->setPWM(200);
    MisMotores[LEFT]->setPWM(200) ;
}

void NavCuadricula::avanzar()
{
    MisMotores[RIGHT]->setFwd();
    MisMotores[LEFT]->setFwd();
    MisMotores[RIGHT]->setPWM(200);
    MisMotores[LEFT]->setPWM(200) ;
}

void NavCuadricula::girar(bool sentido)
{
    if(sentido == HORARIO)
    {
        MisMotores[RIGHT]->setFwd();
        MisMotores[LEFT]->setBack();
        MisMotores[RIGHT]->setPWM(200);
        MisMotores[LEFT]->setPWM(200) ;
    }else
    {
        MisMotores[RIGHT]->setBack();
        MisMotores[LEFT]->setFwd();
        MisMotores[RIGHT]->setPWM(200);
        MisMotores[LEFT]->setPWM(200) ;
    }
}

void NavCuadricula::giro90(bool sentido)
{
    if(sentido == HORARIO)
    {
        MisMotores[RIGHT]->setFwd();
        MisMotores[LEFT]->setBack();
        MisMotores[RIGHT]->setPWM(200);
        MisMotores[LEFT]->setPWM(200) ;
    }else
    {
        MisMotores[RIGHT]->setBack();
        MisMotores[LEFT]->setFwd();
        MisMotores[RIGHT]->setPWM(200);
        MisMotores[LEFT]->setPWM(200) ;
    }

    delay(MILLIS_GIRO90);
    this->parar();
}

float NavCuadricula::getOutput()
{
    return output;
    
}


void NavCuadricula::seguirLinea()
{
    float vel_a, vel_b;
    vel_a = vel_base + output;
    vel_b = vel_base - output;

    MisMotores[RIGHT] ->setFwd();
    MisMotores[LEFT]  ->setFwd();

    vel_a > vel_max? MisMotores[RIGHT]->setPWM(vel_max) : MisMotores[RIGHT]-> setPWM(vel_a);
    vel_b > vel_max? MisMotores[LEFT]->setPWM(vel_max) :MisMotores[LEFT]-> setPWM(vel_b);
}

bool NavCuadricula::sobreLineaHorizontal()
{
    int suma = 0;
    int i;
    for(i = 0; i < SENSORCOUNT; i++)
    {
        suma = suma + sensorValues[i];
    }

    return suma > myumbral? true : false;
}

void NavCuadricula::compute()
{
    posicion = myQtr->readLineBlack(sensorValues);
    myPID->Input = (posicion - center)/10;
    myPID->Update();
    output = myPID->Output;
    
}