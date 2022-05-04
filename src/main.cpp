#include <QTRSensors.h>

/* Codigo pora el robot Rumbaracha para la prueba de la cuadricula del AstiChllenge 2022
 */

#include "Pinout.h"
#include "Config.h"
#include <DNSServer.h>
#include <ESPUI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TofSensors.h"
//#include <Adafruit_PWMServoDriver.h>
#include "ESPUI_callbacks.h"
#include "funcionesSetup.h"
#include "PCF8575.h"
#include <PIDController.hpp>
#include "FuncionesDisplay.h"

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
extern DNSServer dnsServer;
#include "Motor.h"
//#include "QTRSensors_mod.h"
#include "NavCuadricula.h"
#include "funcionesCuadricula.h"
#include "Logo.h"




//------------------------- Objetos y variables globales -------------------------

// Ignorar
String kp, ki, kd;

// Estado

const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
volatile modos Estado = ESPERANDO;
volatile modos_display Estado_display = DISPLAY_MOSTRAR_ESTADO;
volatile int a  = 0;

volatile int inicio_;
int caso;
volatile int final_;
uint8_t contador_lineas_horizontales;
int lineas_a_contar = 0;
bool flag_horizontal = false;
uint8_t numero_de_veces_giradas = 0; //a


// Objeto global para el extensor de pines
PCF8575 pinExtensor = PCF8575(0x20);

// Objeto que maneja los 3 sensores tof
TofSensors MySensors(1, 2, 3, &pinExtensor);

// objeto global para el display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Objetos para los dos motores (NO control de posicion)
Motor Motor_derecho(PIN_MOTOR_D_IN1, PIN_MOTOR_D_IN2, PIN_MOTOR_D_PWM, PWM_CH_D, PWM_FREC, PWM_RES);   // Motor derecho
Motor Motor_izquierdo(PIN_MOTOR_I_IN1, PIN_MOTOR_I_IN2, PIN_MOTOR_I_PWM, PWM_CH_I, PWM_FREC, PWM_RES); // Motor izquierdo

// Objeto para el sensor siguelineas qtr8rc
QTRSensors qtr;


// Objeto que maneja los motores
NavCuadricula misMotores(&qtr, &Motor_derecho, &Motor_izquierdo);

// Movidas del multitasking
TaskHandle_t Task_CORE0; // Nucleo secundario
TaskHandle_t Task_CORE1; // Nucleo principal



//------------------------- EL codigo  -------------------------------------------

// El core 0 lo usaremos para manejar la movida web
//  y alguna tarea secundaria
void TaskCORE0code(void *pvParameters)
{ 
    
    Serial.println("Core 0 activo");
    display.println("Core 0 activo");
    // Bucle infinito
    for (;;)
    {
        if(Estado == ORDEN_RECIBIDA)
        {
            caso = determinar_caso(inicio_, final_);
            flag_horizontal = false;
            contador_lineas_horizontales = 0;
            numero_de_veces_giradas = 0;
            switch(caso)
            {
                case C_RECTO:
                    lineas_a_contar = 6;
                break;

                case C_RECTOD:  
                    lineas_a_contar = 7 - final_;
                break;

                case C_RECTOI:
                    lineas_a_contar = final_ - 6;
                break;

                case C_ZIGZAG:

                    lineas_a_contar = final_>inicio_? final_ - inicio_ : inicio_ - final_;

                break;
            }
            Estado =  MOVIENDOSE;
        }

        if(Estado == ESPERANDO || Estado == ERROR_)
        {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                
        }

        if(Estado == MOVIENDOSE)
        {
            
            misMotores.seguirLinea();

            if(misMotores.sobreLineaHorizontal())
            {
                flag_horizontal = true;
            }
            else
            {
                if(flag_horizontal == true)
                {
                    flag_horizontal = false;
                    contador_lineas_horizontales++;
                }
            }

            switch(caso)
            {
                case C_RECTO:
                    if(contador_lineas_horizontales == lineas_a_contar) 
                        Estado = LLEGADA_A_DESTINO;
                break;

                case C_RECTOD:
                    if((contador_lineas_horizontales == lineas_a_contar) && (numero_de_veces_giradas == 0))
                    {
                        contador_lineas_horizontales == 0;
                        misMotores.girar(HORARIO);
                        vTaskDelay( xDelay ); 
                        numero_de_veces_giradas++;
                        lineas_a_contar = 13 - inicio_;
                    }

                    if((contador_lineas_horizontales == lineas_a_contar) && (numero_de_veces_giradas == 1))
                    {
                        Estado = LLEGADA_A_DESTINO;
                    }
                break;

                case C_RECTOI:
                    if((contador_lineas_horizontales == lineas_a_contar) && (numero_de_veces_giradas == 0))
                    {
                        contador_lineas_horizontales == 0;
                        misMotores.girar(ANTIHORARIO); 
                        vTaskDelay( xDelay );
                        numero_de_veces_giradas++;
                        lineas_a_contar = inicio_ - 1;
                    }

                    if((contador_lineas_horizontales == lineas_a_contar)&&(numero_de_veces_giradas == 1))
                    {
                        Estado = LLEGADA_A_DESTINO;
                    }
                break;

                case C_ZIGZAG:

                    if((contador_lineas_horizontales == 1) && (numero_de_veces_giradas == 0))
                    {
                        if(final_ > inicio_) misMotores.girar(HORARIO);
                        else misMotores.girar(ANTIHORARIO);
                        vTaskDelay( xDelay );

                        contador_lineas_horizontales = 0;
                        numero_de_veces_giradas++;
                    }

                    if((contador_lineas_horizontales == lineas_a_contar) && (numero_de_veces_giradas == 1))
                    {
                        if(final_ > inicio_) misMotores.girar(ANTIHORARIO);
                        else misMotores.girar(HORARIO);
                        vTaskDelay( xDelay );
                        contador_lineas_horizontales = 0;
                        numero_de_veces_giradas++;
                        lineas_a_contar = 5;
                    }

                    if((contador_lineas_horizontales == lineas_a_contar) && (numero_de_veces_giradas == 2))
                    {
                        Estado = LLEGADA_A_DESTINO;
                    }

                break;
                
            }

        }

        if(Estado == LLEGADA_A_DESTINO)
        {
            misMotores.avanzar();
            delay(MILLIS_LLEGADA_A_DESTINO);
            misMotores.parar();
            contador_lineas_horizontales = 0;
            delay(100);
            Estado = ESPERANDO;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
        Serial.print("lineas_a_contar: ");
        Serial.println(lineas_a_contar);
        Serial.print("contador lineas horizontales: ");
        Serial.println(contador_lineas_horizontales);

        

    }
}

// El core 1 contiene cosas secundarias
void TaskCORE1code(void *pvParameters)
{
    Serial.println("Core 1 activo");
    display.println("Core 1 activo");
    for (;;)
    {
        displayThings();
        misMotores.compute();
        
        dnsServer.processNextRequest();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//------------------------- Setup  ----------------------------------------------
void setup(void)
{
    Wire.setClock(800000);
    

    // Inicializamos el serial
    Serial.begin(9600);

    // Se inicializa y configura el display
    configDisplay();
    
    display.clearDisplay();
    display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_reset, LOGO_WIDTH, LOGO_HEIGHT, 1);
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(1, 1);
    display.println("Configurando Hardware...");
    display.display();

    Motor_derecho.init();
    Motor_izquierdo.init();

    
    // Se inicializan los pines de los tof
    MySensors.init();

    // Se inicializa el extensor de pines
    pinExtensor.begin();

    // Se cambia la direccion de memoria de los sensores tof y se bootean
    MySensors.setID(TOF_ADRESS_FRONT, TOF_ADRESS_RIGHT, TOF_ADRESS_LEFT);
    display.println(" Listo");
    display.display();

    // Funcion que inicializa la interfaz web y todas sus movidas
    display.println("Configurando web..."); 
    display.display();
    configESPUI();
    display.println(" Listo");
    display.display();

    // Configuracion del sensor siguelineas
    display.println("Calibrando qtr...");
    display.display();
    qtr.setTypeRC();
    //qtr.setSensorPins((const uint8_t[]){12, 33, 23, 15, 14, 32}, SENSORCOUNT);
    qtr.setSensorPins((const uint8_t[])
    {
        PIN_QTR_L0, 
        PIN_QTR_L1, 
        PIN_QTR_L2, 
        PIN_QTR_L3, 
        PIN_QTR_L4, 
        PIN_QTR_L5
    }, SENSORCOUNT);
    qtr.setEmitterPin(PIN_QTR_LEDON);
        for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    display.println(" Listo");
    display.display();

    delay(1000);
    display.clearDisplay();

    // -------------- Creamos los dos hilos en los dos nucleos ----------------

    // Para el core 0 (secundario)
    xTaskCreatePinnedToCore(
        TaskCORE0code, // funcion de la task
        "Task_CORE0",  // nombre de la task
        10000,         // tamano del stack de la task
        NULL,          // Parametos
        1,             // task priority
        &Task_CORE0,   // callback
        0              // se lo mandamos al nucleo 0
    );

    // Para el core 1 (principal)
    xTaskCreatePinnedToCore(
        TaskCORE1code, // funcion de la task
        "Task_CORE1",  // nombre de la task
        10000,         // tamano del stack de la task
        NULL,          // Parametos
        1,             // task priority
        &Task_CORE1,   // callback
        1              // se lo mandamos al nucleo 1
    );
}



// No usado
void loop(void)
{
}