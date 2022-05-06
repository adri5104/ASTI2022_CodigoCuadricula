#ifndef ESPUI_callbacks
#define ESPUI_callbacks

#include <DNSServer.h>
#include <ESPUI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"
#include "TofSensors.h"
#include <Adafruit_PWMServoDriver.h>



void callback_umbral(Control*, int);
void callback_tiempo_giro(Control*, int);
void callback_tiempo_avance(Control*, int);
void callback_vel_base(Control*, int);


void callback_inicio(Control* , int );
void callback_final(Control* , int );
void start_callback(Control* , int );

void pad_callback(Control*, int);
void stop_callback(Control*, int);

void display_mode_callback(Control*, int);

void callback_setkp(Control*, int);
void callback_setki(Control*, int);
void callback_setkd(Control*, int);
void callback_setbutton(Control*, int);

void callback_linebutton(Control*, int);

void callback_start_laberinto(Control*, int);
void callback_t1_laberinto(Control*, int);
void callback_t2_laberinto(Control*, int);





#endif