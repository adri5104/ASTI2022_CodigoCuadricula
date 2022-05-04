#include "funcionesCuadricula.h"

int determinar_caso(int ini,int  fin)
{
    if(ini >= V_MIN && ini <= V_MAX)
    {
        if(fin >= H_MIN && fin <= H_MAX)
        {
            return C_RECTOI;
        }
    }
    else
    {
        if(fin >= V_MIN && fin <= V_MAX)
        {
            return C_RECTOD;
        } 
    }

    return ini == fin? C_RECTO : C_ZIGZAG;
}
