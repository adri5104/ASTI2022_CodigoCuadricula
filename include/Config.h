#ifndef CONFIG_
#define CONFIG_

#include "Pinout.h"
#include <DNSServer.h>
#include <ESPUI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TofSensors.h"
#include <Adafruit_PWMServoDriver.h>
#include "ESPUI_callbacks.h"
#include "funcionesSetup.h"
#include "PCF8575.h"
#include <PIDController.hpp>
#include <QTRSensors.h>




#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define TOF_ADRESS_FRONT 0x30
#define TOF_ADRESS_LEFT 0x31
#define TOF_ADRESS_RIGHT 0x32

#define FRONT 2
#define RIGHT 0
#define LEFT 1
#define LED_BUILTIN 2


// --------------- Movidas para la salida pwm --------------
#define PWM_FREC 5000 //frecuencia de la senal pwm
#define PWM_RES 8// resolucion de la senal pwm. En nuestro caso 8 bits (0 - 255)
#define PWM_CH_D 0 //canal para el pwm del motor derecho 
#define PWM_CH_I 1 //canal para el pwm del motor izquierdo

//Movidas del sensor siguelineas
#define SENSORCOUNT 6
const uint16_t THRESHOLD = 4500;
const float center = 2500.0;

#define MILLIS_GIRO90 1000
#define MILLIS_LLEGADA_A_DESTINO 3000


enum modos
{
    ESPERANDO,
    MOVIENDOSE,
    LLEGADA_A_DESTINO, //Se mueve un poco mas hacia delante y despues vuelve a pasar a estado esperando
    ORDEN_RECIBIDA,
    ERROR_,
    MODO_LABERINTO

};


#define NUM_MODOS_DISPLAY 6
enum modos_display
{
    DISPLAY_APAGADO,
    DISPLAY_MOSTRAR_TOF,
    DISPLAY_MOSTRAR_PARAMETROSPID,
    DISPLAY_MOSTRAR_LINEA,
    DISPLAY_MOSTRAR_ESTADO,
    DISPLAY_MOSTRAR_LOGO,
    DISPLAY_MOSTRAR_PARAMETROS
};


#endif