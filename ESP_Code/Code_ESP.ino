//MD Drone - Code for ESP12-E
//Author: Łukasz Halbiniak
//V1.0
#include <ESP8266WiFi.h>       
#include <ESP8266WebServer.h> 

//Init variables
String X_Uc = "1";
String Y_Uc = "2";
String Z_Uc = "3";
String F_Uc = "4";
String T_Uc = "5";
String P_Uc = "6";
String B_Uc = "7";
String F_Android = "0";
String T_Android = "0";
String P_Android = "0";
String H_Android = "0";
String SD_Android = "0";
String TR_Android = "0";
String M1_Android = "0";
String M2_Android = "0";
String M3_Android = "0";
String M4_Android = "0";

//HOTSPOT parameters
const char *ssid = "MDDRONE"; 
const char *password = "D12345678";  

//For LED blink
bool change =true;

//Recovery functions
IPAddress local_ip(192,168,4,4);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);
ESP8266WebServer server(80);  

//Setup ESP
void setup() {
  //Int UART
  Serial.begin(115200);
  delay(1000);
  Serial.setTimeout(100);

  //Waiting for connection
  WiFi.begin(ssid, password);   
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on (Note that LOW is the voltage level
  //Init server  
  server.on("/",HTTP_GET, handleArgs); 
  server.begin();
  //Serial.print("Uruchomiono");
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LEDas output
  
  //Toggle if everything is good
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on (Note that LOW is the voltage level
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
}

void loop() 
  {
    //Regular function
  delay(1);
  server.handleClient();
  }


void handleArgs() 
{ 
  //Serial.print("Otrzymano handlera");
  
  //Preapre message for uC
  int paraml = server.args();   
  F_Android = server.arg(0);
  T_Android = server.arg(1);
  P_Android = server.arg(2);
  H_Android = server.arg(3);
  SD_Android = server.arg(4);
  TR_Android = server.arg(5);
  M1_Android = server.arg(6);
  M2_Android = server.arg(7);
  M3_Android = server.arg(8);
  M4_Android = server.arg(9);
  String message_to_Uc = "F" + F_Android + "T" + T_Android + "P000"  + "H" + H_Android + "D" + SD_Android + "R" + TR_Android + "Z" + M1_Android + "X" + M2_Android + "C" + M3_Android + "V" + M4_Android;
  
  //Send message to uC
  Serial.print(message_to_Uc + "L");

  //Receive message from uC
  char Buff_UART[50];
  Serial.readBytesUntil('Q', Buff_UART, 50);
  
  if(strlen(Buff_UART) == 0) {
  
    //Send message for Android
    String message_to_Android = "X1Y2Z3F4T5P6B7V";
    server.send(200, "text/plain", message_to_Android);       //Response to the HTTP request
    if (change ==true)
    {
      change = false;
      digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    }else
    {
      change = true;
      digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on (Note that LOW is the voltage level
    }
   }else
   {
      String Buff_ConV_To_String = String(Buff_UART);
      int X_Pos = Buff_ConV_To_String.indexOf("X");
      int Y_Pos = Buff_ConV_To_String.indexOf("Y");
      int Z_Pos = Buff_ConV_To_String.indexOf("Z");
      int F_Pos = Buff_ConV_To_String.indexOf("F");
      int T_Pos = Buff_ConV_To_String.indexOf("T");
      int P_Pos = Buff_ConV_To_String.indexOf("P");
      int B_Pos = Buff_ConV_To_String.indexOf("B");
      int Q_Pos = Buff_ConV_To_String.indexOf("Q");
      X_Uc = Buff_ConV_To_String.substring(X_Pos+1, Y_Pos);
      Y_Uc = Buff_ConV_To_String.substring(Y_Pos+1, Z_Pos);
      Z_Uc = Buff_ConV_To_String.substring(Z_Pos+1, F_Pos);
      F_Uc = Buff_ConV_To_String.substring(F_Pos+1, T_Pos);
      T_Uc = Buff_ConV_To_String.substring(T_Pos+1, P_Pos);
      P_Uc = Buff_ConV_To_String.substring(P_Pos+1, B_Pos);
      B_Uc = Buff_ConV_To_String.substring(B_Pos+1, Q_Pos);
      
      //Send message for Android
      String message_to_Android = "X" + X_Uc + "Y" + Y_Uc + "Z" + Z_Uc + "F" + F_Uc + "T" + T_Uc + "P" + P_Uc + "B" + B_Uc + "V";
      server.send(200, "text/plain", message_to_Android);       //Response to the HTTP request
      if (change ==true)
      {
        change = false;
        digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
      }else
      {
        change = true;
        digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on (Note that LOW is the voltage level
      }
   }
}
