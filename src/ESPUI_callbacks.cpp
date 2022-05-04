

#include "ESPUI_callbacks.h"
#include "Config.h"
#include "NavCuadricula.h"
#include <PIDController.hpp>

extern String kp;
extern String kd;
extern String ki;

extern volatile int inicio_;
extern volatile int final_;

extern volatile modos Estado;
extern volatile modos_display Estado_display;

extern NavCuadricula misMotores;





void display_mode_callback(Control* sender, int value)
{
    Estado_display = (modos_display) sender->value.toInt();
}


void pad_callback(Control* sender, int value)
{
    switch(value){
        case P_LEFT_DOWN:

        misMotores.girar(ANTIHORARIO);
        
        break;

        case P_LEFT_UP:

        misMotores.parar();
        
        break;

    case P_RIGHT_DOWN:
        misMotores.girar(HORARIO);
    
    break;

    case P_RIGHT_UP:
        misMotores.parar();
    
    break;

    case P_FOR_DOWN:
        misMotores.avanzar();
        break;

    case P_FOR_UP:
        misMotores.parar();
        break;

    case P_BACK_DOWN:

        misMotores.retroceder();
        
    break;

  case P_BACK_UP:

    misMotores.parar();
    
    break;

  }
}






void callback_inicio(Control* sender , int type)
{
    inicio_ = sender -> value.toInt();
}

void callback_final(Control* sender , int type)
{
    final_ = sender -> value.toInt();
}

void start_callback(Control* sender, int type)
{
    switch (type) {
    case B_DOWN:
        
    break;

    case B_UP:
        if(Estado == ESPERANDO) Estado = ORDEN_RECIBIDA;

        if(Estado == ERROR_) Estado = ESPERANDO;
    break;
    }
}

void stop_callback(Control* sender, int value)
{
    switch(value)
    {
        case B_DOWN:
            Estado = ESPERANDO;
        break;
    }
}

void callback_setkp(Control* sender, int value_)
{
      
    kp = sender->value;
}
void callback_setki(Control* sender, int value_)
{
    ki = sender->value;
}
void callback_setkd(Control* sender, int value_)
{
    kd = sender->value;
}
void callback_setbutton(Control* sender, int value)
{
    if(B_DOWN)
    {
        PID::PIDParameters<float> aux(kp.toFloat(), ki.toFloat(), kd.toFloat());
        misMotores.setPIDparam(aux);
    }
}

void callback_umbral(Control* sender, int value)
{
    if(value == N_VALUE)
    {
        misMotores.setUmbral(sender->value.toInt());
    }
}

void callback_linebutton(Control* sender, int value)
{
    switch(value)
    {
        case B_DOWN:
            misMotores.seguirLinea();
            break;
        
        case B_UP:
            misMotores.parar();
            break;
    }
    
}
