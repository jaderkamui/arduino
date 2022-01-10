#include <WiFi.h>
#include <PubSubClient.h>


const char *ssid = "Lusana";
const char *password = "{=Edu5us#2bd4,ZF";
const char *id_board = "5d86665b732249f8c6965e2bf013316b";
const char *mqtt_server = "annie.systems";
const int PushButton = 15;

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon SERIAL_8N1

//#define WIFISSID "Lusana" // Enter WifiSSID here
//#define PASSWORD "{=Edu5us#2bd4,ZF" // Enter password here
// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
//#define SerialAT Serial1

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// set GSM PIN, if any
#define GSM_PIN "13"

// Your GPRS credentials, if any
const char apn[]  = "internet";     //SET TO YOUR APN
//const char gprsUser[] = "wap";
//const char gprsPass[] = "wap";

#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR      1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP       60          // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13
#define LED_PIN             12  // (D3 on NodeMCU)


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (250)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void enableGPS(void)
{
  // Set SIM7000G GPIO4 LOW ,turn on GPS power
  // CMD:AT+SGPIO=0,4,1,1
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();


}

void disableGPS(void)
{
  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

void setup_wifi()
{

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1')
  {
    //digitalWrite(BUILTIN_LED, LOW); // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  }
  else
  {
    //digitalWrite(BUILTIN_LED, HIGH); // Turn the LED off by making the voltage HIGH
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "{\"string_id_board\":\"5d86665b732249f8c6965e2bf013316b\",\"MD001\":true}");
      // ... and resubscribe
      client.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}




void enableGPS(void)
{
  // Set SIM7000G GPIO4 LOW ,turn on GPS power
  // CMD:AT+SGPIO=0,4,1,1
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();


}

void disableGPS(void)
{
  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

void modemPowerOn()
{
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);    //Datasheet Ton mintues = 1S
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff()
{
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);    //Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, HIGH);
}


void modemRestart()
{
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  delay(10);

  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  modemPowerOn();
  enableGPS();

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println("/**********************************************************/");
  Serial.println("To initialize the network test, please make sure your GPS");
  Serial.println("antenna has been connected to the GPS port on the board.");
  Serial.println("/**********************************************************/\n\n");

  delay(10000);
}

void loop()
{
  if (!modem.testAT()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    modemRestart();
    return;
  }

  Serial.println("Start positioning . Make sure to locate outdoors.");
  Serial.println("The blue indicator light flashes to indicate positioning.");

  //enableGPS();

  float lat,  lon;
  //while (1) {
  if (modem.getGPS(&lat, &lon)) {
    Serial.println("The location has been locked, the latitude and longitude are:");
    Serial.print("latitude:"); Serial.println(lat);
    Serial.print("longitude:"); Serial.println(lon);
    //      break;
  }
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(2000);
  //}

  //disableGPS();

  Serial.println("/**********************************************************/");
  Serial.println("After the network test is complete, please enter the  ");
  Serial.println("AT command in the serial terminal.");
  Serial.println("/**********************************************************/\n\n");

  //while (1) {
  while (SerialAT.available()) {
    SerialMon.write(SerialAT.read());
  }
  while (SerialMon.available()) {
    SerialAT.write(SerialMon.read());
  }

  //}

  int DI_in1_Result = digitalRead(PushButton);


  Serial.println(DI_in1_Result);


  delay(1);
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 1001)
  {
    lastMsg = now;
    ++value;
    //snprintf (msg, MSG_BUFFER_SIZE, "hello worldx #%ld", value);
    //String init_json = "{\"string_id_board\":\"5d86665b732249f8c6965e2bf013316b\",\"data\":\"";
    //String end_json = "\"}";
    //DI_in1_Result
    String fake_json = "";
    fake_json = fake_json + "{\"string_id_board\":\"5d86665b732249f8c6965e2bf013316b\",\"MD001\":true";
    fake_json = fake_json + ",\"DI0001\":\"";
    fake_json = fake_json + lat;

    fake_json = fake_json + "\"}";

    //String fake_json = init_json + AN_In1_Result  + end_json;
    //snprintf (msg, MSG_BUFFER_SIZE, fake_json);
    fake_json.toCharArray(msg, MSG_BUFFER_SIZE);
    Serial.print("Publish message: ");
    Serial.println(msg);
    Serial.println(fake_json);
    client.publish("eemml/boards/5d86665b732249f8c6965e2bf013316b", msg);
  }
}
