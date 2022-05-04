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
    ESPUI.number("Inicio", &callback_inicio, ControlColor::Alizarin, 0, 0, 10);
    ESPUI.number("Final", &callback_final, ControlColor::Alizarin,0, 0, 10);
    ESPUI.button("Start", &start_callback, ControlColor::Dark, "Press");
    ESPUI.pad("Pad without center", &pad_callback, ControlColor::Carrot);
    
    statusLabelId = ESPUI.label("Status:", ControlColor::Turquoise, "Stop");
    
    ESPUI.number("Modo display", &display_mode_callback, ControlColor::Alizarin, 0, 0, NUM_MODOS_DISPLAY);
   
    
    ESPUI.button("EMERGENCY STOP", &stop_callback, ControlColor::Carrot,"STOP" );


    ESPUI.text("Kp", &callback_setkp, ControlColor::Alizarin, "KP");
    ESPUI.text("Ki", &callback_setki, ControlColor::Alizarin, "KI");
    ESPUI.text("Kd", &callback_setkd, ControlColor::Alizarin, "KD");
    ESPUI.button("Actualizar parametros", &stop_callback, ControlColor::Carrot,"Actualizar" );


    ESPUI.sliderContinuous = false;

    ESPUI.number("Threshold", &callback_umbral, ControlColor::Alizarin, THRESHOLD, 1000, 7000);

    ESPUI.begin("ESPUI Control");
}