/******************************************************************************/
// ovenctrl

#include <Arduino.h>

/* change it with your wifi_sta_ssid-wifi_sta_password */
const char* wifi_sta_ssid = "gewifon";
const char* wifi_sta_password = "wifi-pass";

const char* wifi_ap_ssid = "Pizza Oven";
const char* wifi_ap_pass = "sWEE3hFoUGzn";

const char* dnsName = "pizza";        // Domain name for the mDNS responder
IPAddress wifi_ap_ip(192, 168, 4, 1); // The IP address of the access point

/******************************************************************************/
// ArduinoOTA

#include <ArduinoOTA.h>

void ota_setup() {
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");

    // Password can be set with it's md5 value as well
    ArduinoOTA.setPasswordHash("5a75c115aab0352a300b32ece12f9af4");

    ArduinoOTA.onStart([]() { Serial.print("D:Start OTA update\n"); });
    ArduinoOTA.onEnd([]() { Serial.print("D:End OTA update\n"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("D:OTA Progress: %u%%\n", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("D:OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.print("Auth Failed\n");
        else if (error == OTA_BEGIN_ERROR)
            Serial.print("Begin Failed\n");
        else if (error == OTA_CONNECT_ERROR)
            Serial.print("Connect Failed\n");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.print("Receive Failed\n");
        else if (error == OTA_END_ERROR)
            Serial.print("End Failed\n");
    });
    ArduinoOTA.begin();
}

void ota_poll() {
    // check for over the air updates
    ArduinoOTA.handle();
}

/******************************************************************************/
// Wifi

#include <ESPmDNS.h>
#include <WiFi.h>

void wifi_event(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED: Serial.println("WiFi connected"); break;

    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi IP address: ");
        Serial.println(WiFi.localIP());
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        break;
    }
}

void wifi_setup() {
    WiFi.mode(WIFI_AP_STA);

    // start the wifi access point
    WiFi.softAPConfig(wifi_ap_ip, wifi_ap_ip, IPAddress(255, 255, 255, 0));
    WiFi.softAP(wifi_ap_ssid, wifi_ap_pass);

    Serial.print("Access Point \"");
    Serial.print(wifi_ap_ssid);
    Serial.println("\" started\r\n");

    // start async wifi station
    WiFi.onEvent(wifi_event);

    WiFi.begin(wifi_sta_ssid, wifi_sta_password);
}

void mdns_setup() { // Start the mDNS responder
    Serial.println("Starting mDNS responder ...");
    MDNS.begin(dnsName);
    Serial.print("mDNS responder started: http://");
    Serial.print(dnsName);
    Serial.println(".local");
}

/******************************************************************************/
// MAX6675 Thermocouple

#include <MAX6675_Thermocouple.h>

const int spi_sck_pin = 18;
const int spi_so_pin = 19;
const int spi_cs0_pin = 5;
const int spi_cs1_pin = 17;

MAX6675_Thermocouple thermocouple0(spi_sck_pin, spi_cs0_pin, spi_so_pin);
MAX6675_Thermocouple thermocouple1(spi_sck_pin, spi_cs1_pin, spi_so_pin);

double temp0 = 0.0;
double temp1 = 0.0;

void temp_main(void*) {
    while (true) {
        temp0 = thermocouple0.readCelsius();
        temp1 = thermocouple1.readCelsius();

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void temp_setup() {
    xTaskCreate(&temp_main, // function
        "temp_main",        // name
        2048,               // stack
        NULL,               // void* to input parameter
        10,                 // priority
        NULL                // task handle
    );
}

/******************************************************************************/
// Buttons

const int button_up0_pin = 12;
const int button_down0_pin = 14;
const int button_up1_pin = 26;
const int button_down1_pin = 27;

volatile unsigned target_temp0 = 0;
volatile unsigned target_temp1 = 0;

volatile unsigned isr_ts = 0;
volatile unsigned isr_last_ts = 0;

const int max_temp = 900;

void IRAM_ATTR on_button_up0() {
    if (isr_last_ts != isr_ts) {
        if (target_temp0 < max_temp)
            target_temp0 += 25;
        isr_last_ts = isr_ts;
    }
}
void IRAM_ATTR on_button_down0() {
    if (isr_last_ts != isr_ts) {
        if (target_temp0 >= 25)
            target_temp0 -= 25;
        isr_last_ts = isr_ts;
    }
}

void IRAM_ATTR on_button_up1() {
    if (isr_last_ts != isr_ts) {
        if (target_temp1 < max_temp)
            target_temp1 += 25;
        isr_last_ts = isr_ts;
    }
}
void IRAM_ATTR on_button_down1() {
    if (isr_last_ts != isr_ts) {
        if (target_temp1 >= 25)
            target_temp1 -= 25;
        isr_last_ts = isr_ts;
    }
}

void buttons_main(void*) {
    while (true) {
        isr_ts = millis();

        if (digitalRead(button_up0_pin) == LOW) {
            if (target_temp0 < max_temp)
                target_temp0 += 25;
        }
        if (digitalRead(button_down0_pin) == LOW) {
            if (target_temp0 >= 25)
                target_temp0 -= 25;
        }
        if (digitalRead(button_up1_pin) == LOW) {
            if (target_temp1 < max_temp)
                target_temp1 += 25;
        }
        if (digitalRead(button_down1_pin) == LOW) {
            if (target_temp1 >= 25)
                target_temp1 -= 25;
        }

        digitalWrite(16, digitalRead(button_up0_pin));
        digitalWrite(4, digitalRead(button_up1_pin));

        // wait / yield time to other tasks
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void buttons_setup() {
    pinMode(button_up0_pin, INPUT_PULLUP);
    pinMode(button_down0_pin, INPUT_PULLUP);
    pinMode(button_up1_pin, INPUT_PULLUP);
    pinMode(button_down1_pin, INPUT_PULLUP);

    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    attachInterrupt(
        digitalPinToInterrupt(button_up0_pin), on_button_up0, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(button_down0_pin), on_button_down0, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(button_up1_pin), on_button_up1, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(button_down1_pin), on_button_down1, FALLING);

    xTaskCreate(&buttons_main, // function
        "buttons_main",        // name
        2048,                  // stack
        NULL,                  // void* to input parameter
        10,                    // priority
        NULL                   // task handle
    );
}

/******************************************************************************/
// Optocoupled Contacts

const int opto0_pin = 13;
const int opto1_pin = 25;

volatile bool opto0_state = false;
volatile bool opto1_state = false;

void IRAM_ATTR on_opto0() {
    opto0_state = digitalRead(opto0_pin);
}
void IRAM_ATTR on_opto1() {
    opto1_state = digitalRead(opto1_pin);
}

void opto_setup() {
    pinMode(opto0_pin, INPUT_PULLUP);
    pinMode(opto1_pin, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(opto0_pin), on_opto0, CHANGE);
    attachInterrupt(
        digitalPinToInterrupt(opto1_pin), on_opto1, CHANGE);
}

/******************************************************************************/
// MQTT

#include <PubSubClient.h>
#include <WiFiClientSecure.h>

const char* mqtt_ca_cert =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/"
    "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT"
    "DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow"
    "SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT"
    "GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC"
    "AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF"
    "q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8"
    "SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0"
    "Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA"
    "a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj"
    "/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T"
    "AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG"
    "CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv"
    "bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k"
    "c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw"
    "VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC"
    "ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz"
    "MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu"
    "Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF"
    "AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo"
    "uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/"
    "wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu"
    "X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG"
    "PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6"
    "KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\n"
    "-----END CERTIFICATE-----";

/* create an instance of WiFiClientSecure */
WiFiClientSecure mqtt_wifi_client;
PubSubClient mqtt_client(mqtt_wifi_client);

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received: ");
    Serial.println(topic);

    Serial.print("payload: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void mqtt_setup() {
    /* set SSL/TLS certificate */
    mqtt_wifi_client.setCACert(mqtt_ca_cert);
    /* configure the MQTT server with IP address and port */
    mqtt_client.setServer("panthema.net", 993);
    // mqtt_callback function will be invoked when subscribed topic are received
    mqtt_client.setCallback(mqtt_callback);
}

void mqtt_connect() {
    if (WiFi.status() != WL_CONNECTED)
        return;

    Serial.print("MQTT connecting ...");
    /* mqtt_client ID */
    String clientId = "pizza-control";
    /* connect now */
    if (mqtt_client.connect(clientId.c_str())) {
        Serial.println("connected");
        /* subscribe topic */
        // mqtt_client.subscribe(led_topic);
    }
    else {
        Serial.print("failed, status code =");
        Serial.print(mqtt_client.state());
        Serial.println("try again in 5 seconds");
        /* Wait 5 seconds before retrying */
        delay(5000);
    }
}

/******************************************************************************/
// LCD

#include <LiquidCrystal_I2C.h>

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void lcd_main(void*) {
    char buffer[32];

    while (true) {
        unsigned xtemp0 = temp0;
        if (xtemp0 >= 10000)
            xtemp0 = 9999;

        snprintf(buffer, sizeof(buffer), "%4u C/%3u C %c  ", xtemp0, target_temp0,
                 opto0_state ? 'C' : 'O');

        lcd.setCursor(0, 0);
        lcd.print(buffer);

        unsigned xtemp1 = temp1;
        if (xtemp1 >= 10000)
            xtemp1 = 9999;

        snprintf(buffer, sizeof(buffer), "%4u C/%3u C %c  ", xtemp1, target_temp1,
                 opto1_state ? 'C' : 'O');

        lcd.setCursor(0, 1);
        lcd.print(buffer);

        // wait / yield time to other tasks
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void lcd_setup() {
    // initialize the LCD via pins 21, 22
    lcd.begin();

    // Print a message to the LCD.
    lcd.backlight();
    lcd.clear();
    lcd.print("Initializing...");

    xTaskCreate(&lcd_main, // function
        "lcd_main",        // name
        2048,              // stack
        NULL,              // void* to input parameter
        10,                // priority
        NULL               // task handle
    );
}

/******************************************************************************/

const char* counter_topic = "pizza/temp0";
char msg[20];
int counter = 0;
long lastMsg = 0;

const char* temperature_topic = "pizza/temp1";
uint8_t temperature = 0;

void mqtt_poll() {
    /* if mqtt_client was disconnected then try to reconnect again */
    if (!mqtt_client.connected()) {
        mqtt_connect();
        return;
    }

    /* this function will listen for incoming topics */
    mqtt_client.loop();

    /* we increase counter every 3 secs
       we count until 3 secs reached to avoid blocking program if using
       delay()*/
    long now = millis();
    if (now - lastMsg > 1000) {
        lastMsg = now;
        if (counter < 100) {
            counter++;
            snprintf(msg, 20, "%d", counter);
            /* publish the message */
            mqtt_client.publish(counter_topic, msg);
        }
        else {
            counter = 0;
        }

        snprintf(msg, 20, "%d", temperature);
        /* publish the message */
        mqtt_client.publish(temperature_topic, msg);
    }
}

/******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

void task_main(void*) {
    while (true) {

        temperature = temprature_sens_read();

        // wait / yield time to other tasks
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task_setup() {
    xTaskCreate(&task_main, // function
        "task_main",        // name
        2048,               // stack
        NULL,               // void* to input parameter
        10,                 // priority
        NULL                // task handle
    );
}

/******************************************************************************/
// main

void setup() {
    Serial.begin(115200);
    Serial.println("Booting");

    lcd_setup();

    wifi_setup();
    mdns_setup();
    ota_setup();
    mqtt_setup();

    temp_setup();
    buttons_setup();
    opto_setup();

    task_setup();
}

void loop() {
    ota_poll();
    mqtt_poll();

    vTaskDelay(250 / portTICK_PERIOD_MS);
}

/******************************************************************************/
