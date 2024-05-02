// Import required libraries
#include <Arduino.h>

//<OLED>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 myOLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int id_display_time=2000;
//</OLED>


//<WiFi>
#include <WiFi.h>
const char *ssid = "SamIsTestingHisEsp32";
const char *password = "SamIsTestingHisEsp32";
//</WiFi>

//<time>
const char* ntpServer = "stdtime.gov.hk";//天文台的互聯網時間伺服器
const long  gmtOffset_sec = 28800;//格林威治時間,一格3600,GMT+8就是8*3600=28800
const int   daylightOffset_sec = 0;
void printLocalTime(bool debug=0);
//</time>

//<MQTT>
#include <PubSubClient.h>
IPAddress mqtt_server;
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
long pMqttReconnect_millis=0;
float pt,ph,pb,ps;
bool pPUMPSTATE;
void reconnect() {
  pMqttReconnect_millis=millis();
  Serial.print("Attempting MQTT connection...");
  // Attempt to connect
  if (client.connect("NodeMCU-32s")) {
    client.subscribe("$SYS/broker/clients/total");
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
  }
}
//</MQTT>

//<HardWare>
//  <RFID>
#include <MFRC522.h> //library responsible for communicating with the module RFID-RC522
#include <SPI.h> //library responsible for communicating of SPI bus
#define SS_PIN    16
#define RST_PIN   17
#define SIZE_BUFFER     18
#define MAX_SIZE_BLOCK  16
MFRC522::MIFARE_Key key;
MFRC522::StatusCode status;
MFRC522 mfrc522(SS_PIN, RST_PIN);
//  </RFID>
//  <carPark>
//    <system>
byte carPark[5][10];
int left_spaces=5;
int identify(byte* car);
byte last_car[10];
int last_icode=2;
long last_gate_millis=0;
#define ENTERED 0
#define EXITED 1
#define FULLALREADY -1
//    </system>
//    <gate>
#include <Adafruit_PWMServoDriver.h>//adafruit/Adafruit PWM Servo Driver Library@^3.0.2
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
int angleToPulse(int ang);
void gate(bool state);
//    </gate>
//    <parking_space>
//      <74HC165>
const int dataPin = 35;   /* SER_OUT */
const int clockPin = 32;  /* CLK */
const int latchPin = 33;  /* SH/LD */
long p74HC165_Sample_millis=0;
const int numBits = 8;   /* Set to 8 * number of shift registers */
//        <QRE1113>
bool park_space_data[5];
//        </QRE1113>
//      </74HC165>
//    </parking_space>
//  </carPark>
//</HardWare>

TaskHandle_t SamplingTask;
void core0(void *pvParameters){
  long pWifiReconnect_millis=0;
  reconnect();
  for(;;){
    if(!client.connected()&&millis()-pMqttReconnect_millis>=5000)reconnect();
    // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
    if ((WiFi.status() != WL_CONNECTED) && (millis() - pWifiReconnect_millis >=30000)) {
      Serial.print(millis());
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      pWifiReconnect_millis = millis();
    }
    delay(1);
  }
}



void setup() 
{
  Serial.begin(115200);
  SPI.begin(); // Init SPI bus
  // Init MFRC522
  //<HardWare>
  //  <RFID>
  mfrc522.PCD_Init();
  mfrc522.PCD_DumpVersionToSerial();
  //  </RFID>
  //  <carPark>
  //    <system>
  //    </system>
  //    <gate>
  board1.begin();
  board1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  gate(0);
  //    </gate>
  //    <parking_space>
  //      <QRE1113>
  //      </QRE1113>
  //      <74HC165>
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  //      </74HC165>
  //    </parking_space>
  //  </carPark>
  //</HardWare>
  
  
  
  Serial.println("Approach your reader card...");
  Serial.println();
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!myOLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  myOLED.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  myOLED.clearDisplay();
  myOLED.setTextSize(0.5);
  myOLED.setTextColor(WHITE);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  myOLED.setCursor(0,0);
  myOLED.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    myOLED.print(".");
    myOLED.display();
  }
  myOLED.clearDisplay();
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  myOLED.setCursor(0,0);
    myOLED.setTextColor(WHITE);
  myOLED.println(WiFi.localIP());
  myOLED.println(WiFi.gatewayIP());
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  myOLED.display();
  myOLED.clearDisplay();
  mqtt_server=WiFi.gatewayIP();
  Serial.println(mqtt_server);
  client.setServer(mqtt_server, 1883);
  client.setCallback(
    [](char* topic, byte* message, unsigned int length) {
      Serial.print("Message arrived on topic: ");
      Serial.print(topic);
      Serial.print(". Message: ");
      String msg;
      
      for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        msg += (char)message[i];
      }
      Serial.println();
      if (String(topic)=="$SYS/broker/clients/total"){
        // only_me=(msg.toInt()<=1);
        for (int i = 0; i < 5; i++) {
          int bit = park_space_data[i];
          client.publish(("carpark/object_sensor"+String(i)).c_str(),String(bit?0:1).c_str());
        }
        client.publish("carpark/left_space",String(left_spaces).c_str());
      }
    }
  );
  xTaskCreatePinnedToCore(
      core0,     /* Task function. */
      "Task of reconnect",       /* name of task. */
      10000,         /* Stack size of task */
      NULL,          /* parameter of the task */
      1,             /* priority of the task */
      &SamplingTask, /* Task handle to keep track of created task */
      0);            /* pin task to core 0 */
}
void loop() 
{ 
  client.loop();
  myOLED.display();
  myOLED.clearDisplay();
  myOLED.setTextSize(1);
  myOLED.setTextColor(WHITE);
  myOLED.setCursor(0,0);
  printLocalTime();
  myOLED.printf("%d parking space%s remaining",left_spaces,left_spaces==1?"":"s");
  //<HardWare>
  //  <RFID>
  if (mfrc522.PICC_IsNewCardPresent()and mfrc522.PICC_ReadCardSerial()){
    Serial.println("mfrc522.PICC_IsNewCardPresent()");
    Serial.println("mfrc522.PICC_ReadCardSerial()");
    for(byte i=0;i<10;i++){
      last_car[i]=mfrc522.uid.uidByte[i];
      Serial.print(last_car[i]<16?" 0":" ");
      Serial.print(last_car[i],HEX);
    }
    last_icode=identify(last_car);
    if(last_icode==ENTERED)left_spaces-=1;
    else if(last_icode==EXITED)left_spaces+=1;
    if(last_icode!=FULLALREADY)gate(1);
    last_gate_millis=millis();
    client.publish("carpark/left_space",String(left_spaces).c_str());
    Serial.println();
  }
  //  </RFID>
  //  <carPark>
  //    <system>
  if(left_spaces==0&&millis()-last_gate_millis>id_display_time){
      myOLED.setTextSize(4);
      myOLED.println("FULL");
  }
  else if(left_spaces==0&&millis()-last_gate_millis<=id_display_time&&last_icode==FULLALREADY){
    Serial.printf("abs((millis()-last_gate_millis)%%300) = %d\r\n",abs(long(millis()-last_gate_millis)%300));
    if(abs(long(millis()-last_gate_millis)%300)<150){
      myOLED.println("");
      Serial.println("");
    }
    else{
      myOLED.setTextSize(4);
      myOLED.println("FULL");
      Serial.println("FULL");
    }
  }
  else if(millis()-last_gate_millis<=id_display_time){
    for(byte i=0;i<10;i++){
      myOLED.print(last_car[i]<16?" 0":" ");
      myOLED.print(last_car[i],HEX);
    }
    myOLED.println(last_icode==ENTERED?"\nEnter":"\nExit");
  }
  //    </system>
  //    <gate>
  if(millis()-last_gate_millis>=id_display_time)
    gate(0);
  //    </gate>
  //    <parking_space>
  //      <QRE1113>
  //      </QRE1113>
  //      <74HC165>
  if(millis()-p74HC165_Sample_millis>=1){
    p74HC165_Sample_millis=millis();
    digitalWrite(latchPin, LOW);
    digitalWrite(latchPin, HIGH);
    // Serial.print("Bits: ");
    for (int i = 0; i < numBits; i++) {
      int bit = digitalRead(dataPin);
      if(i>2&&park_space_data[i-2]!=bit)client.publish(("carpark/object_sensor"+String(i-2)).c_str(),String(bit?0:1).c_str());
      if(i>2&&park_space_data[i-2]!=bit)Serial.printf("%s%d = %d\r\n","carpark/object_sensor",i-2,bit);
      if(i>2)park_space_data[i-2]=bit;
      digitalWrite(clockPin, HIGH); // Shift out the next bit
      digitalWrite(clockPin, LOW);
    }
    // Serial.println();
  }
  //      </74HC165>
  //    </parking_space>
  //  </carPark>
  //</HardWare>
  
  
  //instructs the PICC when in the ACTIVE state to go to a "STOP" state
  mfrc522.PICC_HaltA(); 
  // "stop" the encryption of the PCD, it must be called after communication with authentication, otherwise new communications can not be initiated
  mfrc522.PCD_StopCrypto1();  
}
void printLocalTime(bool debug){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    if(debug)Serial.println("Failed to obtain time");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    return;
  }
  myOLED.println(&timeinfo, "%A,%d %B,%Y %H:%M:%S");
  if(!debug)return;
  Serial.println(&timeinfo, "%A,%d %B,%Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("TEST variables");//抓取時間資料裡的"小時"存成字串做測試,以便後續應用
  char myHour[3];
  strftime(myHour,3, "%H", &timeinfo);
  Serial.println(myHour);
  String A = String(myHour);
  Serial.println(A);

}
int identify(byte* car){
  int mt_index=-1;
  for(byte i=0;i<5;i++){
    int found_car=10;//mark of same
    int mt=10;//mark of empty
    for(byte j=0;j<10;j++){
      if(carPark[i][j]!=car[j])
        found_car-=1;
      if(carPark[i][j]!=0)mt-=1;
    }
    if(found_car==10){
      for(byte j=0;j<10;j++)
        carPark[i][j]=0;
      return EXITED;
    }
    if(mt==10&&mt_index<0)mt_index=i;
  }
  if(mt_index>=0){
    for(byte i=0;i<10;i++)
      carPark[mt_index][i]=car[i];
    return ENTERED;
  }
  return FULLALREADY;
}
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}
void gate(bool state){
  board1.setPWM(0, 0, angleToPulse(state?45:135) );
}