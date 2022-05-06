#include "FuncionesDisplay.h"

extern volatile modos Estado;
extern volatile modos_display Estado_display;
extern volatile int inicio_;
extern volatile int final_;
extern TofSensors MySensors;
extern Adafruit_SSD1306 display;
extern QTRSensors qtr;
extern NavCuadricula misMotores;
extern const unsigned char logo_reset [];
extern int caso;

void displayThings()
{
    display.clearDisplay();
        if(Estado_display!= DISPLAY_APAGADO)
        {
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(1, 1);
            switch(Estado_display)
            {
                case DISPLAY_MOSTRAR_ESTADO:
                    switch (Estado)
                    {
                    case ESPERANDO:
                        display.println("Estado: Esperando");
                        break;

                    case MODO_LABERINTO:
                        display.println("Estado: Laberinto");
                        break;

                    case ORDEN_RECIBIDA:
                        display.println("Estado: Orden recibida");
                        break;
                    
                    case MOVIENDOSE:
                        display.println("Estado: Ruta iniciada");
                        break;
                    default:
                        break;
                    }
                    display.println("");
                    display.print("Inicio: ");
                    display.println(inicio_);
                    display.print("Final: ");
                    display.println(final_);
                    display.print("L_Horiz: ");
                    if(misMotores.sobreLineaHorizontal()) display.println("true");
                    else display.println("false");

                    if(Estado == MOVIENDOSE)
                    {
                        switch(caso)
                        {
                            case C_RECTO: 
                                display.println("Todo recto");
                                break;

                            case C_RECTOI: 
                                display.println("Recto y a la izquierda");
                                break;

                            case C_RECTOD: 
                                display.println("Recto y a la derecha");
                                break;

                            case C_ZIGZAG: 
                                display.println("zigzag");
                                break;

                            case C_ERROR:
                                display.println("ERROR");
                                break;
                         }
                    }

                break;

                case DISPLAY_MOSTRAR_TOF:
                    display.println("Sensores laser: ");
                    display.println("");
                    display.print("Front: ");
                    display.println(MySensors.getDistance(FRONT));
                    display.println();
                    display.print("Left: ");
                    display.println(MySensors.getDistance(LEFT));
                    display.println();
                    display.print("Right: ");
                    display.println(MySensors.getDistance(RIGHT));
                    display.println();

                break;

                case DISPLAY_MOSTRAR_PARAMETROSPID:
                    display.println("Parametros PID: ");
                    display.println("");
                    display.print("Kp actual:" );
                    display.println(misMotores.getKp());
                    display.print("Ki actual:" );
                    display.println(misMotores.getKi());
                    display.print("Kd actual:" );
                    display.println(misMotores.getKd());
                    display.print("PID input:" );
                    display.println(misMotores.getPos());
                    display.print("PID output: ");
                    display.println(misMotores.getOutput());
                    
                break;

                case DISPLAY_MOSTRAR_LOGO:
                display.clearDisplay();
                display.drawBitmap(
                (display.width()  - LOGO_WIDTH ) / 2,
                (display.height() - LOGO_HEIGHT) / 2,
                logo_reset, LOGO_WIDTH, LOGO_HEIGHT, 1);
                display.display();
                break;

                case DISPLAY_MOSTRAR_PARAMETROS:
                
                    display.println("Parametros : ");
                    display.println("");
                    display.print("Threshold:" );
                    display.println(misMotores.getUmbral());
                    display.print("T_giro:" );
                    display.println(misMotores.getVgiro());
                    display.print("Vel_base:" );
                    display.println(misMotores.getVelBase());
                    
                break;

                case DISPLAY_MOSTRAR_LINEA:
                    uint8_t SensorCount = SENSORCOUNT;
                    uint16_t sensorValues[SensorCount];
                    uint16_t position = qtr.readLineBlack(sensorValues);
                    for (uint8_t i = 0; i < SensorCount; i++)
                    {
                        display.print(i);
                        display.print(": ");
                        display.print(sensorValues[i]);
                        display.print('\n');
                    }
                    display.println(position);
                    

                break;

                

            }
        }display.display();
}
