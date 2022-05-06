#include "funcionesSetup.h"

extern Adafruit_SSD1306 display;
extern TofSensors MySensors;
extern const uint16_t THRESHOLD;
const char* ssid = "eduroam";
const char* password = "espui";
const char* hostname = "Rumbaracha";
IPAddress apIP(192, 168, 4, 1);
const byte DNS_PORT = 53;
int statusLabelId;
DNSServer dnsServer;
extern const uint16_t THRESHOLD;

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

void configDisplay()
{
    // ------------------------- Movidas del display --------------------
if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }

  delay(1000); //antes 2000
display.display();

  // Clear the buffer
  display.clearDisplay();
  display.display();
  delay(1000); //antes 2000
  display.clearDisplay();


}

void configESPUI()
{
    // ------------------------- Movidas de la interfaz web --------------------
    ESPUI.setVerbosity(Verbosity::Quiet);
   

#if defined(ESP32)
    WiFi.setHostname(hostname);
#else
    WiFi.hostname(hostname);
#endif

    // try to connect to existing network
    WiFi.begin(ssid, password);
    Serial.print("\n\nTry to connect to existing network");

    {
        uint8_t timeout = 10;

        // Wait for connection, 5s timeout
        do
        {
            delay(500);
            Serial.print(".");
            timeout--;
        } while (timeout && WiFi.status() != WL_CONNECTED);

        // not connected -> create hotspot
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.print("\n\nCreating hotspot");

            WiFi.mode(WIFI_AP);
            delay(100);
            WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
#if defined(ESP32)
            uint32_t chipid = 0;
            for (int i = 0; i < 17; i = i + 8)
            {
                chipid |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
            }
#else
            uint32_t chipid = ESP.getChipId();
#endif
            char ap_ssid[25];
            snprintf(ap_ssid, 26, "ESPUI-%08X", chipid);
            WiFi.softAP(ap_ssid);

            timeout = 5;

            do
            {
                delay(500);
                Serial.print(".");
                timeout--;
            } while (timeout);
        }
    }

    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.println("\n\nWiFi parameters:");
    Serial.print("Mode: ");
    Serial.println(WiFi.getMode() == WIFI_AP ? "Station" : "Client");
    Serial.print("IP address: ");
    Serial.println(WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP());

    // --------------------------- Interfaz ---------------------

    uint16_t tab0 =  ESPUI.addControl(ControlType::Tab, "Cuadricula", "Cuadricula");
    uint16_t tab1 =  ESPUI.addControl(ControlType::Tab, "Ajustes generales", "Ajustes generales");
    uint16_t tab2 =  ESPUI.addControl(ControlType::Tab, "Ajustes PID", "Ajustes PID");
    uint16_t tab3 =  ESPUI.addControl(ControlType::Tab, "Minifabrica", "Minifabrica");
    uint16_t tab4 =  ESPUI.addControl(ControlType::Tab, "Laberinto", "Laberinto");
    ESPUI.addControl(ControlType::Number, "Inicio", "1", ControlColor::Wetasphalt, tab0,&callback_inicio);
    ESPUI.addControl(ControlType::Number, "Final", "1", ControlColor::Wetasphalt,tab0, &callback_final);
    ESPUI.addControl(ControlType::Button, "Iniciar prueba", "Start", ControlColor::Turquoise, tab0, &start_callback);
    ESPUI.addControl(ControlType::Number, "Modo display", "1", ControlColor::Wetasphalt, tab0, &display_mode_callback);
    ESPUI.addControl(ControlType::Button, "EMERGENCY STOP", "STOP", ControlColor::Alizarin, tab0, &stop_callback);
    ESPUI.addControl(ControlType::Number, "Threshold", "4500", ControlColor::Wetasphalt, tab1, &callback_umbral);
    ESPUI.addControl(ControlType::Number, "Tiempo giro", "1000", ControlColor::Wetasphalt, tab1, &callback_tiempo_giro);
    ESPUI.addControl(ControlType::Number, "Velocidad base", "150", ControlColor::Wetasphalt, tab1, &callback_vel_base);
    ESPUI.addControl(ControlType::Text, "Kp", "2.0", ControlColor::Wetasphalt, tab2, &callback_setkp );
    ESPUI.addControl(ControlType::Text, "Ki", "0.1", ControlColor::Wetasphalt, tab2, &callback_setki );
    ESPUI.addControl(ControlType::Text, "Kd", "0.01", ControlColor::Wetasphalt, tab2, &callback_setkd );
    ESPUI.addControl(ControlType::Button, "Actualizar parametros", "Update", ControlColor::Turquoise, tab2, &callback_setbutton );
    ESPUI.addControl(ControlType::Button, "Start laberinto", "press", ControlColor::Turquoise, tab4, &callback_start_laberinto );  
    ESPUI.addControl(ControlType::Number, "Avance 1", "1000", ControlColor::Wetasphalt, tab4, &callback_t1_laberinto);
    ESPUI.addControl(ControlType::Number, "Avance 2", "1000", ControlColor::Wetasphalt, tab4, &callback_t2_laberinto);
    ESPUI.addControl(ControlType::Pad, "Modo manual", "", ControlColor::Wetasphalt, tab3, &pad_callback);
    ESPUI.begin("Rumbaracha V1");
}