/******************************************************************************/
// ovenctrl

#include <Arduino.h>

#include <deque>

/* change it with your wifi_sta_ssid-wifi_sta_password */
const char* wifi_sta_ssid = "gewifon";
const char* wifi_sta_password = "wifi-pass";
// const char* wifi_sta_ssid = "GPN18-insecureXXX";
// const char* wifi_sta_password = "";

const char* wifi_ap_ssid = "Pizza Control";
const char* wifi_ap_pass = "pizzaoven";

const char* dnsName = "pizza";        // Domain name for the mDNS responder
IPAddress wifi_ap_ip(192, 168, 4, 1); // The IP address of the access point

static const bool g_ofen_simulator = false;

/******************************************************************************/
// ArduinoOTA

#include <ArduinoOTA.h>

void ota_setup() {
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
    case SYSTEM_EVENT_STA_CONNECTED:
        Serial.println("WiFi connected");
        break;

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

    // enable async wifi station
    WiFi.onEvent(wifi_event);
}

void wifi_poll(unsigned long now) {
    static long ts_check = 0;

    if (ts_check < now) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.disconnect();

            // start async wifi connect
            WiFi.begin(wifi_sta_ssid, wifi_sta_password);

            Serial.print("Wifi: connecting to ");
            Serial.println(wifi_sta_ssid);
        }
        ts_check = now + 10000;
    }
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

#include <FS.h>
#include <MAX6675.h>
#include <SD.h>
#include <SPI.h>

const int spi_sck_pin = 18;
const int spi_so_pin = 19;
const int spi_tc0_cs_pin = 17;
const int spi_tc1_cs_pin = 16;

MAX6675 thermocouple0(spi_tc0_cs_pin);
MAX6675 thermocouple1(spi_tc1_cs_pin);

double temp0 = 0.0;
double temp1 = 0.0;
float temp_esp = 0.0;

#ifdef __cplusplus
extern "C" {
#endif

// ESP internal temperature in Celsius
float temperatureRead();

#ifdef __cplusplus
}
#endif

void ilog_sd_store();
void ilog_sd_push();

void spi_poll(unsigned long now) {
    static unsigned long last_check = 0;
    if (last_check > now)
        return;
    last_check = now + 500;

    if (g_ofen_simulator) {
        double temp0 = thermocouple0.readCelsius();
        double temp1 = thermocouple1.readCelsius();
    }
    else {
        temp0 = thermocouple0.readCelsius();
        temp1 = thermocouple1.readCelsius();
    }
    temp_esp = temperatureRead();

    ilog_sd_store();
    ilog_sd_push();
}

void spi_setup() { SPI.begin(); }

/******************************************************************************/
// Relais Pins

const int relay0_pin = 2;
const int relay1_pin = 4;

/******************************************************************************/
// Buttons

const int button_up0_pin = 14;
const int button_down0_pin = 12;
const int button_up1_pin = 27;
const int button_down1_pin = 26;

volatile unsigned target_temp0 = 0;
volatile unsigned target_temp1 = 0;

volatile unsigned isr_ts = 0;
volatile unsigned isr_last_ts = 0;

const int max_temp = 900;

static inline void on_target_temp_up(volatile unsigned& temp) {
    if (temp + 20 < max_temp)
        temp += 20;
    else
        temp = max_temp;
}
static inline void on_target_temp_down(volatile unsigned& temp) {
    if (temp >= 20)
        temp -= 20;
    else
        temp = 0;
}

void IRAM_ATTR on_button_up0() {
    if (isr_last_ts != isr_ts) {
        on_target_temp_up(target_temp0);
        isr_last_ts = isr_ts;
    }
}
void IRAM_ATTR on_button_down0() {
    if (isr_last_ts != isr_ts) {
        on_target_temp_down(target_temp0);
        isr_last_ts = isr_ts;
    }
}

void IRAM_ATTR on_button_up1() {
    if (isr_last_ts != isr_ts) {
        on_target_temp_up(target_temp1);
        isr_last_ts = isr_ts;
    }
}
void IRAM_ATTR on_button_down1() {
    if (isr_last_ts != isr_ts) {
        on_target_temp_down(target_temp1);
        isr_last_ts = isr_ts;
    }
}

void buttons_poll(unsigned long now) {
    static unsigned long last_check = 0;
    if (last_check > now)
        return;
    last_check = now + 500;

    if (digitalRead(button_up0_pin) == LOW) {
        on_target_temp_up(target_temp0);
    }
    if (digitalRead(button_down0_pin) == LOW) {
        on_target_temp_down(target_temp0);
    }
    if (digitalRead(button_up1_pin) == LOW) {
        on_target_temp_up(target_temp1);
    }
    if (digitalRead(button_down1_pin) == LOW) {
        on_target_temp_down(target_temp1);
    }
}

void buttons_setup() {
    pinMode(button_up0_pin, INPUT_PULLUP);
    pinMode(button_down0_pin, INPUT_PULLUP);
    pinMode(button_up1_pin, INPUT_PULLUP);
    pinMode(button_down1_pin, INPUT_PULLUP);

    pinMode(relay0_pin, OUTPUT);
    digitalWrite(relay0_pin, LOW);
    pinMode(relay1_pin, OUTPUT);
    digitalWrite(relay1_pin, LOW);

    attachInterrupt(
        digitalPinToInterrupt(button_up0_pin), on_button_up0, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(button_down0_pin), on_button_down0, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(button_up1_pin), on_button_up1, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(button_down1_pin), on_button_down1, FALLING);
}

/******************************************************************************/
// Optocoupled Contacts

const int opto0_pin = 13;
const int opto1_pin = 25;

volatile bool opto0_state = false;
volatile bool opto1_state = false;

void IRAM_ATTR on_opto0() { opto0_state = digitalRead(opto0_pin); }
void IRAM_ATTR on_opto1() { opto1_state = digitalRead(opto1_pin); }

void opto_setup() {
    pinMode(opto0_pin, INPUT_PULLUP);
    pinMode(opto1_pin, INPUT_PULLUP);

    opto0_state = digitalRead(opto0_pin);
    opto1_state = digitalRead(opto1_pin);

    attachInterrupt(digitalPinToInterrupt(opto0_pin), on_opto0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(opto1_pin), on_opto1, CHANGE);
}

/******************************************************************************/
// PID Controller

#include <PID_v1.h>

// Specify the initial tuning parameters
double pid_Kp = 2, pid_Ki = 5, pid_Kd = 1;

double pid0_input = temp0;
double pid0_target = target_temp0;
double pid0_output = 0.0;

double pid1_input = temp1;
double pid1_target = target_temp1;
double pid1_output = 0.0;

PID pid0(
    &pid0_input, &pid0_output, &pid0_target, pid_Kp, pid_Ki, pid_Kd, DIRECT);
PID pid1(
    &pid1_input, &pid1_output, &pid1_target, pid_Kp, pid_Ki, pid_Kd, DIRECT);

void pid_setup() {
    // turn the PID on
    pid0.SetMode(AUTOMATIC);
    // output range 0-1
    pid0.SetOutputLimits(0.0, 1.0);

    // turn the PID on
    pid1.SetMode(AUTOMATIC);
    // output range 0-1
    pid1.SetOutputLimits(0.0, 1.0);

    if (g_ofen_simulator) {
        target_temp0 = 200;
        target_temp1 = 200;
    }
}

const unsigned speed = 1;

void pid_poll(unsigned long now) {
    static unsigned long last_check = 0;
    if (last_check > now)
        return;
    last_check = now + 1000;

    static std::deque<bool> pid0_heat = {
        false, false, false, false, false, false};
    static std::deque<bool> pid1_heat = {
        false, false, false, false, false, false};

    if (g_ofen_simulator) {
        // ofen simulator
        temp0 += pid0_heat.front() * 0.3 * speed;
        for (size_t i = 0; i < speed; ++i) {
            temp0 *= (1.0 - 0.000052);
        }
        if (random(2200) < 8 * speed && temp0 > 30)
            temp0 -= 30;

        temp1 += pid1_heat.front() * 0.3 * speed;
        for (size_t i = 0; i < speed; ++i) {
            temp1 *= (1.0 - 0.000052);
        }
        if (random(2200) < 8 * speed && temp1 > 30)
            temp1 -= 30;
    }

    static unsigned round = 5;
    if (round-- == 0) {
        round = 5;
        if (g_ofen_simulator) {
            pid0_heat.pop_front();
            pid0_heat.push_back(pid0_output >= 0.5);

            pid1_heat.pop_front();
            pid1_heat.push_back(pid1_output >= 0.5);
        }

        pid0_input = temp0;
        pid0_target = target_temp0;
        pid0.Compute();

        pid1_input = temp1;
        pid1_target = target_temp1;
        pid1.Compute();

        Serial.print("temp0: ");
        Serial.print(temp0);
        Serial.print(" pid: ");
        Serial.print(pid0_output);
        Serial.print(" temp1: ");
        Serial.print(temp1);
        Serial.print(" pid: ");
        Serial.println(pid1_output);

        if (target_temp0 == 0)
            pid0_output = 0;
        if (target_temp1 == 0)
            pid1_output = 0;

        digitalWrite(relay0_pin, pid0_output >= 0.5 ? HIGH : LOW);
        digitalWrite(relay1_pin, pid1_output >= 0.5 ? HIGH : LOW);
    }
}

/******************************************************************************/
// MQTT

#include <PubSubClient.h>
#include <WiFiClientSecure.h>

const char* mqtt_id = "pizza_control";
const char* mqtt_user = "pizza";
const char* mqtt_passwd = "IjvQYJ6XBd5m";

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
// WiFiClientSecure mqtt_wifi_client;
WiFiClient mqtt_wifi_client;
PubSubClient mqtt_client(mqtt_wifi_client);

void mqtt_callback(char* topic, byte* payload, unsigned int length);
void mqtt_subscribe();

bool mqtt_connected = false;

void mqtt_setup() {
    /* set SSL/TLS certificate */
    // mqtt_wifi_client.setCACert(mqtt_ca_cert);
    /* configure the MQTT server with IP address and port */
    mqtt_client.setServer("panthema.net", 1883);
    // mqtt_callback function will be invoked when subscribed topic are received
    mqtt_client.setCallback(mqtt_callback);
}

void mqtt_connect(unsigned long now) {
    if (WiFi.status() != WL_CONNECTED)
        return;

    static unsigned long last_check = 0;
    if (now < last_check)
        return;
    last_check = now + 5000;

    Serial.print("MQTT connecting ...");
    /* connect now */
    if (mqtt_client.connect(mqtt_id, mqtt_user, mqtt_passwd,
            /* willTopic */ nullptr, /* willQos */ MQTTQOS2,
            /* willRetain */ true, /* willMessage */ nullptr)) {
        Serial.println("connected");
        /* subscribe topics */
        mqtt_subscribe();

        mqtt_connected = true;
    }
    else {
        Serial.print("failed, status code =");
        Serial.print(mqtt_client.state());
        Serial.println("try again in 5 seconds");

        mqtt_connected = false;
    }
}

/******************************************************************************/
// NTP RTC Client

#include <DS3231.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// RTC module
DS3231 ds_rtc;
unsigned long ds_rtc_now = 0;
unsigned long ds_rtc_millis = 0;

// By default 'time.nist.gov' is used with 60 seconds update interval and no
// offset
WiFiUDP ntp_udp;
NTPClient ntp_client(ntp_udp, "europe.pool.ntp.org", 0, 3600000);

// Switched to NTP time
bool ntp_update_done = false;

unsigned long long rtc_epoch_millis() {
    if (ntp_update_done)
        return ntp_client.getEpochMilli();
    else
        return (unsigned long long)ds_rtc_now * 1000 + millis() - ds_rtc_millis;
}

void ntp_setup() {
    // read initial date/time
    DateTime dt = RTClib::now();
    ds_rtc_millis = millis();
    ds_rtc_now = dt.unixtime();
    Serial.print("ds_rtc_now ");
    Serial.println(ds_rtc_now);

    // initialize NTP
    ntp_client.begin();
}

void ntp_poll() {

    if (WiFi.status() != WL_CONNECTED)
        return;

    ntp_client.update();

    if (!ntp_update_done) {
        Serial.println("ntp update done");

        unsigned long ts = ntp_client.getEpochTime();
        Serial.print("ntp ts ");
        Serial.println(ts);

        Serial.print("ntp rtc delta ");
        Serial.println(
            (long)((long long)ts - (long long)rtc_epoch_millis() / 1000));

        ntp_update_done = true;

        DateTime tm(ts);
        ds_rtc.setClockMode(/* 24-hour */ false);
        ds_rtc.setYear(tm.year() - 2000);
        ds_rtc.setMonth(tm.month());
        ds_rtc.setDate(tm.day());
        ds_rtc.setHour(tm.hour());
        ds_rtc.setMinute(tm.minute());
        ds_rtc.setSecond(tm.second());
    }
}

/******************************************************************************/
// Influx/SDCard Logger

struct LogEntry {
    unsigned long long ts;
    double temp0, temp1;
    double temp_esp;
    unsigned target_temp0, target_temp1;
    unsigned char opto0_state, opto1_state;
    double pid_Kp, pid_Ki, pid_Kd;
    double pid0, pid1;
};

const size_t ilog_size = 240;
static LogEntry ilog_list[ilog_size];
static size_t ilog_pos = 0;

const size_t ilog_pos_push = 2;
const size_t ilog_pos_store = 10;

bool ilog_use_sdcard = false;
bool ilog_sdcard_empty = false;

void ilog_poll(unsigned long now) {
    static unsigned long last_check = 0;
    if (last_check > now)
        return;
    last_check = now + 1000;

    long long ts = rtc_epoch_millis();
    if (ts >= 1000000) {
        // create new log entry

        if (ilog_pos < ilog_size) {
            LogEntry& e = ilog_list[ilog_pos++];
            Serial.print("filling ");
            Serial.println(ilog_pos - 1);

            e.ts = ts;
            e.temp0 = temp0, e.temp1 = temp1;
            e.temp_esp = temp_esp;
            e.target_temp0 = target_temp0, e.target_temp1 = target_temp1;
            e.opto0_state = opto0_state, e.opto1_state = opto1_state;
            e.pid_Kp = pid_Kp, e.pid_Ki = pid_Ki, e.pid_Kd = pid_Kd;
            e.pid0 = pid0_output, e.pid1 = pid1_output;
        }
    }
}

void ilog_poll_push(unsigned long now) {
    static unsigned long last_check = 0;
    if (last_check > now)
        return;
    last_check = now + 100;

    if (ilog_pos < ilog_pos_push)
        return;

    if (!mqtt_client.connected())
        return;

    static char buffer[1 * 768];
    size_t out = 0;

    size_t ilog_use;
    for (ilog_use = 0; ilog_use < ilog_pos; ++ilog_use) {
        const LogEntry& e = ilog_list[ilog_use];

        char buffer0[32], buffer1[32], buffer_esp[32];
        char buffer_pid_Kp[32], buffer_pid_Ki[32], buffer_pid_Kd[32];
        char buffer_pid0[32], buffer_pid1[32];

        if (e.temp0 != e.temp0 || e.temp1 != e.temp1)
            continue;

        dtostrf(e.temp0, 0, 2, buffer0);
        dtostrf(e.temp1, 0, 2, buffer1);
        dtostrf(e.temp_esp, 0, 2, buffer_esp);

        dtostrf(e.pid_Kp, 0, 2, buffer_pid_Kp);
        dtostrf(e.pid_Ki, 0, 2, buffer_pid_Ki);
        dtostrf(e.pid_Kd, 0, 2, buffer_pid_Kd);

        dtostrf(e.pid0, 0, 2, buffer_pid0);
        dtostrf(e.pid1, 0, 2, buffer_pid1);

        int a = snprintf(buffer + out, sizeof(buffer) - out,
            "pizza_gluehpn18 temp0=%s,temp1=%s,"
            "target_temp0=%u,target_temp1=%u,"
            "temp_esp=%s,"
            "opto0=%u,opto1=%u,"
            "pid_Kp=%s,pid_Ki=%s,pid_Kd=%s,"
            "pid0=%s,pid1=%s "
            "%0lu%06lu000000\n",
            buffer0, buffer1, e.target_temp0, e.target_temp1, buffer_esp,
            e.opto0_state, e.opto1_state, buffer_pid_Kp, buffer_pid_Ki,
            buffer_pid_Kd, buffer_pid0, buffer_pid1,
            (unsigned long)(e.ts / 1000000L), (unsigned long)(e.ts % 1000000L));

        if (a >= sizeof(buffer) - out)
            break;

        out += a;
    }

    Serial.print("logentry ");
    Serial.println(ilog_use);
    Serial.print("out ");
    Serial.println(out);

    if (!mqtt_client.publish("pizza/influx_data", (uint8_t*)buffer, out,
            /* retain */ false))
        return;

    memmove(ilog_list, ilog_list + ilog_use,
        (ilog_pos - ilog_use) * sizeof(ilog_list[0]));
    ilog_pos -= ilog_use;

    Serial.print("remain ");
    Serial.println(ilog_pos);
}

void ilog_sd_store() {
    unsigned long now = millis();

    static unsigned long last_check = 0;
    if (last_check > now)
        return;

    last_check = now + 2000;

    // to disable logging:
    // return;

    /**************************************************************************/

    if (ilog_pos < ilog_pos_store)
        return;

    uint32_t start = millis();

    if (!ilog_use_sdcard && !SD.begin(5)) {
        Serial.println("Card Mount Failed");
        return;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        ilog_use_sdcard = false;
        return;
    }

    char filename[48];
    snprintf(filename, sizeof(filename), "/log-%0lu%06lu.dat",
        (unsigned long)(ilog_list[0].ts / 1000000L),
        (unsigned long)(ilog_list[0].ts % 1000000L));

    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        ilog_use_sdcard = false;
        return;
    }

    file.write(reinterpret_cast<const uint8_t*>(ilog_list),
        sizeof(ilog_list[0]) * ilog_pos);
    file.close();

    uint32_t end = millis() - start;

    Serial.printf("%u bytes written for %u ms to %s\n",
        sizeof(ilog_list[0]) * ilog_pos, end, filename);

    ilog_pos = 0;

    ilog_use_sdcard = true;
    ilog_sdcard_empty = false;
}

void ilog_sd_push() {

    if (ilog_sdcard_empty)
        return;

    unsigned long now = millis();

    static unsigned long last_check = 0;
    if (last_check > now)
        return;

    last_check = now + 1000;

    // to disable logging:
    // return;

    /**************************************************************************/

    uint32_t start = millis();

    if (!mqtt_client.connected())
        return;

    if (!ilog_use_sdcard && !SD.begin(5)) {
        Serial.println("Card Mount Failed");
        return;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        ilog_use_sdcard = false;
        return;
    }

    File root = SD.open("/");
    if (!root) {
        Serial.println("Failed to open root directory");
        ilog_use_sdcard = false;
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("Root is not a directory");
        ilog_use_sdcard = false;
        return;
    }

    size_t count = 0;
    File file = root.openNextFile();
    while (file) {
        ++count;
        if (!file.isDirectory()) {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());

            if (strncmp(file.name(), "/log-", 5) == 0 &&
                file.size() % sizeof(LogEntry) == 0 &&
                ilog_pos + file.size() / sizeof(LogEntry) + 16 < ilog_size) {
                file.read((uint8_t*)(ilog_list + ilog_pos), file.size());
                ilog_pos += file.size() / sizeof(LogEntry);

                Serial.print("ilog_sd_push read: ");
                Serial.println(file.size() / sizeof(LogEntry));

                if (SD.remove(file.name())) {
                    Serial.println("ilog_sd_push file deleted");
                }
                else {
                    Serial.println("ilog_sd_push delete failed");
                }
                break;
            }
        }
        file = root.openNextFile();
    }

    Serial.printf("ilog_sd_push() done in %u ms\n",
        static_cast<unsigned>(millis() - start));

    ilog_use_sdcard = true;

    if (count == 0)
        ilog_sdcard_empty = true;
}

/******************************************************************************/
// LCD

#include <LiquidCrystal_I2C.h>

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void lcd_poll(unsigned long now) {
    static char buffer0[32], buffer1[32];
    static unsigned ticker = 0;

    static unsigned long last_check = 0;
    if (last_check > now)
        return;
    last_check = now + 500;

    unsigned xtemp0 = temp0;
    if (xtemp0 >= 10000)
        xtemp0 = 9999;

    snprintf(buffer0, sizeof(buffer0), "%4u/%3uC   %4d", xtemp0, target_temp0,
        ticker);

    unsigned xtemp1 = temp1;
    if (xtemp1 >= 10000)
        xtemp1 = 9999;

    snprintf(buffer1, sizeof(buffer1), "%4u/%3uC %c%c%c%c%c%c", xtemp1,
        target_temp1, WiFi.status() == WL_CONNECTED ? 'W' : 'w',
        ntp_update_done ? 'N' : 'n', mqtt_connected ? 'M' : 'm',
             ilog_use_sdcard ? (ilog_sdcard_empty ? 'E' : 'S') : 's',
             opto0_state ? '-' : 'O',
             opto1_state ? '-' : 'O');

    ticker = (ticker + 1) % 10000;

    lcd.setCursor(0, 0);
    lcd.print(buffer0);
    lcd.setCursor(0, 1);
    lcd.print(buffer1);
}

void lcd_setup() {
    // initialize the LCD via pins 21, 22
    lcd.begin();

    // Print a message to the LCD.
    lcd.backlight();
    lcd.clear();
    lcd.print("Initializing...");
}

/******************************************************************************/

const char* heartbeat_topic = "pizza/heartbeat";
unsigned mqtt_heartbeat = 0;

const char* temp0_topic = "pizza/temp0";
const char* temp1_topic = "pizza/temp1";
const char* temp_esp_topic = "pizza/temp_esp";

double temp0_last = temp0;
double temp1_last = temp1;
float temp_esp_last = temp_esp;

const char* target_temp0_topic = "pizza/target_temp0";
const char* target_temp1_topic = "pizza/target_temp1";

unsigned target_temp0_last = target_temp0;
unsigned target_temp1_last = target_temp1;

const char* opto0_state_topic = "pizza/opto0_state";
const char* opto1_state_topic = "pizza/opto1_state";

bool opto0_state_last = opto0_state;
bool opto1_state_last = opto1_state;

const char* pid_Kp_topic = "pizza/pid_Kp";
const char* pid_Ki_topic = "pizza/pid_Ki";
const char* pid_Kd_topic = "pizza/pid_Kd";

double pid_Kp_last = pid_Kp;
double pid_Ki_last = pid_Ki;
double pid_Kd_last = pid_Kd;

/*----------------------------------------------------------------------------*/

void mqtt_subscribe() {
    mqtt_client.subscribe(target_temp0_topic);
    mqtt_client.subscribe(target_temp1_topic);

    mqtt_client.subscribe(pid_Kp_topic);
    mqtt_client.subscribe(pid_Ki_topic);
    mqtt_client.subscribe(pid_Kd_topic);

    if (g_ofen_simulator) {
        mqtt_client.subscribe(temp0_topic);
        mqtt_client.subscribe(temp1_topic);
    }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received: ");
    Serial.println(topic);

    Serial.print("payload: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    if (strcmp(topic, target_temp0_topic) == 0) {
        payload[length] = 0;
        target_temp0 = atoi(reinterpret_cast<const char*>(payload));
        target_temp0_last = target_temp0;
    }
    else if (strcmp(topic, target_temp1_topic) == 0) {
        payload[length] = 0;
        target_temp1 = atoi(reinterpret_cast<const char*>(payload));
        target_temp1_last = target_temp1;
    }
    else if (strcmp(topic, pid_Kp_topic) == 0) {
        payload[length] = 0;
        pid_Kp = atof(reinterpret_cast<const char*>(payload));
        pid_Kp_last = pid_Kp;
        pid0.SetTunings(pid_Kp, pid_Ki, pid_Kd);
        pid1.SetTunings(pid_Kp, pid_Ki, pid_Kd);
    }
    else if (strcmp(topic, pid_Ki_topic) == 0) {
        payload[length] = 0;
        pid_Ki = atof(reinterpret_cast<const char*>(payload));
        pid_Ki_last = pid_Ki;
        pid0.SetTunings(pid_Kp, pid_Ki, pid_Kd);
        pid1.SetTunings(pid_Kp, pid_Ki, pid_Kd);
    }
    else if (strcmp(topic, pid_Kd_topic) == 0) {
        payload[length] = 0;
        pid_Kd = atof(reinterpret_cast<const char*>(payload));
        pid_Kd_last = pid_Kd;
        pid0.SetTunings(pid_Kp, pid_Ki, pid_Kd);
        pid1.SetTunings(pid_Kp, pid_Ki, pid_Kd);
    }
}

void mqtt_poll(unsigned long now) {
    char msg[32];

    /* if mqtt_client was disconnected then try to reconnect again */
    if (!mqtt_client.connected()) {
        mqtt_connect(now);
        return;
    }

    /* this function will listen for incoming topics */
    mqtt_client.loop();

    /* update MQTT values */
    static unsigned long last_check = 0;
    if (now < last_check)
        return;
    last_check = now + 1000;

    // send heartbeat message
    mqtt_heartbeat++;
    snprintf(msg, 20, "%d", mqtt_heartbeat);
    mqtt_client.publish(heartbeat_topic, msg);

    // update temperatures
#define MQTT_DOUBLE(X)                                                         \
    if (X != X##_last) {                                                       \
        dtostrf(X, 0, 2, msg);                                                 \
        mqtt_client.publish(X##_topic, msg, /* retained */ true);              \
        X##_last = X;                                                          \
    }
#define MQTT_UNSIGNED(X)                                                       \
    if (X != X##_last) {                                                       \
        snprintf(msg, sizeof(msg), "%u", X);                                   \
        mqtt_client.publish(X##_topic, msg, /* retained */ true);              \
        X##_last = X;                                                          \
    }

    MQTT_DOUBLE(temp0);
    MQTT_DOUBLE(temp1);
    MQTT_DOUBLE(temp_esp);

    MQTT_UNSIGNED(target_temp0);
    MQTT_UNSIGNED(target_temp1);

    MQTT_UNSIGNED(opto0_state);
    MQTT_UNSIGNED(opto1_state);

    MQTT_DOUBLE(pid_Kp);
    MQTT_DOUBLE(pid_Ki);
    MQTT_DOUBLE(pid_Kd);
}

/******************************************************************************/
// main

void setup() {
    Serial.begin(115200);
    Serial.println("Booting");

    Wire.begin();
    Wire.setTimeOut(1000);

    lcd_setup();

    wifi_setup();
    mdns_setup();
    ota_setup();
    mqtt_setup();
    ntp_setup();

    spi_setup();
    buttons_setup();
    opto_setup();
    pid_setup();
}

void loop() {
    unsigned long now = millis();

    lcd_poll(now);

    wifi_poll(now);
    ota_poll();
    mqtt_poll(now);
    ntp_poll();

    spi_poll(now);
    buttons_poll(now);
    pid_poll(now);
    ilog_poll(now);

    ilog_poll_push(now);
}

/******************************************************************************/
