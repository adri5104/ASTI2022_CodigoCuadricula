#pragma once


#define C_RECTOI 0
#define C_RECTOD 1
#define C_RECTO 2
#define C_ZIGZAG 3
#define C_ERROR -1

#define V_MAX 6
#define V_MIN 1
#define H_MAX 13
#define H_MIN 7

//Funcion que determina en que caso de la cuadricula estamos
int determinar_caso(int ,int);
