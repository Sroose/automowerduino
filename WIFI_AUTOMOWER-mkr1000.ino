/*
  AUTOMOWER CONTROLLER

  AM motherboard pin - MKR1000
  1 (rx) - 8 (rx)
  2 (tx) - 9 (tx) 
  7 (gnd) - 11 (gnd)
  8 (5v) - 13 (vin)
  
  (As mkr1000 needs 5V on vin, you can't use the external AM port which only gives 3.3v, hence we connect to mobo directly)

 The modi returned to Loxone are:
  2: auto
  3: man
  4: home
 The status returned are text, eg No_loop_signal. See in code.
  
*/
#include <SPI.h>
#include <Adafruit_SleepyDog.h>
#include <WiFi101.h>


//Remove // before // Serial.println for debugging
//AVOID STRINGS => memory issues

// VARS 
// For incoming serial data
uint8_t inputBuffer[5];
unsigned int lastStatus = 0;
//extra state to indicate Auto (2), Man (3) or Home (4)
unsigned int modus = 0;

//wifi api reference: https://www.arduino.cc/en/Reference/WiFi101
const char ssid[] = "";      //  your network SSID (name)
const char pass[] = "";   // your network password
int status = WL_IDLE_STATUS;     // the WiFi radio's status

//---NEEDED FOR WIFI KEEP ALIVE --//
// interval after which checking should be done When in connected state
const unsigned long CheckAfterDuration = 60*1000UL; //sec x 1000
unsigned long lastGoodConnection = 0;
const unsigned long oneMinute =  60*1000UL;
const unsigned long oneSecond =  1000UL;
static unsigned long lastSampleTime = 0;

// Loxone (or other) server for statusUpdates
const IPAddress loxone(192,168,1,2); //loxone

// remote server for testing connectivity
const IPAddress testServer(192,168,1,1); //router

// Initialize the server and client library
WiFiServer server(80);

// States of the apparatus
enum States
{
  Checking,
  Connected,
  Disconnected
};

// time at which state started
long stateStartMilliSec = 0;

// current state
enum States currentState;
//------------------------------//

// IMPORTANT!!!!
// comment out all debug lines with strings as they consume memory footprint and the program would not fit
// also avoid any other long strings and do not use variables larger and more global than needed

void setup() {
 
  // SERIAL: USB COM FOR PC
  Serial.begin(115200);      // initialize serial communication

  // SERIAL1: AM connection
  Serial1.begin(9600);      // initialize serial communication
  Serial1.setTimeout(500); // after 500ms, timeout a read. Else it waits forever

  // initialize watchdog
  const unsigned int timeout = 65000; // 65 sec timeout before resetting the arduino (max amount that fits in this int)
  int countdownMS = Watchdog.enable(timeout); // reboots arduno if hangs for this time
  // Serial.println("Enabled the watchdog with max countdown of ");
  // Serial.println(countdownMS, DEC);
  // Serial.println(" milliseconds!");
  // Serial.println();
  
  // connect to Wi-Fi
  ConnectToWiFi();

  // set initial state
  Enter_State_Checking();
  // Serial.println("After initial state checking");

  server.begin();

  // Serial.println("setup done");
  
}



void loop() {
    //  one loop only takes 2ms
   
    // Reset the watchdog with every loop to make sure the sketch keeps running.
    Watchdog.reset();
  
    unsigned long now = millis(); //ms since the program started

    // (very verbose) erial.println(now);
    
    switch(currentState) {
        case Connected:

          serverLogic();

          //only poll the automower every 5 second
          if (now - lastSampleTime >= 5 * oneSecond) {
            lastSampleTime = now;

            checkStatus();
          }
             
          Process_State_Connected();
                break;
    
        case Disconnected:
          Process_State_Disconnected();
                break;
    }
    
}

void closeClient(WiFiClient client) {
  while (client.read() != -1); // makes sure no unread data is present
  client.flush();
  client.stop();
  delay(100);
}


void Enter_State_Checking()
{

  unsigned long now = millis();
  if (now - lastGoodConnection >= oneMinute || now <= oneMinute) {
    currentState = Checking;
  
    // wifi.status() method does not return status correctly. It would return connected
    // even after wi-fi has been shut. To get around this, the method attempts to connect to the server.

    WiFiClient client;
    if(client.connect(testServer, 80))
    {
       lastGoodConnection = millis();
       Enter_State_Connected();
    }
    else
    {
       // Serial.println("WiFi Disconnected");
       Enter_State_Disconnected();
    }

    closeClient(client);
  }
}


void Enter_State_Connected()
{
  currentState = Connected;
  
  // reset the timer
  stateStartMilliSec = millis();  
}

void Process_State_Connected()
{
  
  // if time has elapsed, transition to Checking state to see if wifi is still present
  long timeNow = millis();
  if((timeNow - stateStartMilliSec) >= CheckAfterDuration)
  {
      Enter_State_Checking();
  }
  
}

void Enter_State_Disconnected()
{
  currentState = Disconnected;
  // reset the timer
  stateStartMilliSec = millis(); 

}

void Process_State_Disconnected()
{
    // RECONNECT
    // Serial.println("Reconnecting..");
    ConnectToWiFi();
    Enter_State_Checking();
    // Serial.println("After state check when reconnecting");
}

  
// common method to connect to Wi-Fi
void ConnectToWiFi()
{
  while ( status != WL_CONNECTED) {
    
    // Serial.print("Attempting to connect to WPA SSID: ");
    // Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    
    // wait 10 seconds for connection
    delay(10000);  
  }

  // next to checking status, also check if signal -100 as there is some bug it seems, that status is ok while not being connected
  // you'll in this case see ip 0.0.0.0 and signal -100 dBm
  long rssi = WiFi.RSSI();
  if(status == WL_CONNECTED && rssi == -100) {
    // Serial.println("Says connected but it seems not. Changing status");
    status = WL_IDLE_STATUS; // does this work ?
    delay(5000);
  }

  printWifiStatus(); // remove for live
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  // Serial.println("You're connected to the network");
  // Serial.println("SSID: ");
  // Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  // Serial.println("IP Address: ");
  // Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  // Serial.println("signal strength (RSSI):");
  // Serial.println(rssi);
  // Serial.println(" dBm");
  // print where to go in a browser:
  // Serial.println("To see this page in action, open a browser to http://");
  // Serial.println(ip);
}


void checkStatus() {

  // Serial.println("Checking AM status..");

  // Clear incoming buffer so we will not be reading any noise
  while(Serial1.available())
    Serial1.read();

  uint8_t requestStatus1[5] = { 0x0F, 0x01, 0xF1, 0x00, 0x00 };
  Serial1.write(requestStatus1, 5);
  Serial1.flush();
  uint8_t statusAutomower1[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // Resultat leer initialisieren
  Serial1.readBytes(statusAutomower1,5);
  unsigned int statusInt1 = statusAutomower1[4] << 8 | statusAutomower1[3];
  // ignore collision status, as it happens on each boundary hit
  if(statusInt1 == 1040) {
    return;
  }
  if(statusInt1 != lastStatus) {
    has changed
    // Serial.println("Status has changed, tell to loxone..");
    lastStatus = statusInt1;
    
    WiFiClient client;
    
    // Serial.println("New client to loxone");
    if(client.connect(loxone, 333))
    {
      // Serial.println("Connected to loxone");
      lastGoodConnection = millis();
      // Make a HTTP request:
      client.print("GET /dev/sps/io/AutomowerStatus/");
      clientPrintStatusName(client, statusInt1);
      client.println(" HTTP/1.1");
      //Base 64 encode user_name:password
      client.println("Authorization: Basic TBD"); 
      client.println("Connection: close");
      client.println();
      while (client.read() != -1); // makes sure no unread data is present
      client.flush();
      client.stop();
      delay(100);
    } else {
      // Serial.println("COULD NOT CONNECT TO LOXONE");
    }
    closeClient(client);
    // Serial.println("End of checking status.");
  }

}

void sendModusUpdate() {
  WiFiClient client;

  // Serial.println("New client to loxone");
  if(client.connect(loxone, 333))
  {
    // Serial.println("Connected to loxone");
    lastGoodConnection = millis();
    // Make a HTTP request:
    client.print("GET /dev/sps/io/AutomowerModus/");
    client.print(modus);
    client.println(" HTTP/1.1");
    //Base 64 encode user_name:password
    client.println("Authorization: Basic TBD"); 
    client.println("Connection: close");
    client.println();
    
  } else {
    // Serial.println("COULD NOT CONNECT TO LOXONE");
  }
  
  // Serial.println("End of checking status");
  closeClient(client);
}


void sendTimerUpdate(int timerStatus) {
  WiFiClient client;

  // Serial.println("New client to loxone");
  if(client.connect(loxone, 333))
  {
    // Serial.println("Connected to loxone");
    lastGoodConnection = millis();
    // Make a HTTP request:
    client.print("GET /dev/sps/io/AutomowerTimer/");
    client.print(timerStatus);
    client.println(" HTTP/1.1");
    //Base 64 encode user_name:password
    client.println("Authorization: Basic TBD"); 
    client.println("Connection: close");
    client.println();
    
  } else {
    // Serial.println("COULD NOT CONNECT TO LOXONE");
  }
  
  // Serial.println("End of checking status");
  closeClient(client);
}

void serverLogic() { 

  WiFiClient client;
  
  client = server.available();   // listen for incoming clients, available means the other side already has sent data

  if (client) {                             // if you get a client,
    // Serial.println("new client");           // print a message out the serial port
    lastGoodConnection = millis();
    String currentLine = "";                // make a String to hold incoming data from the client
    int timeoutCounter = 0;
    char action[] = "123456"; // USE 6 CHARS FOR EVERY ACTION
    
    while (client.connected()) {            // loop while the client's connected

      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //enable this debug line to see full incomign requesst
        //Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            //just some info on the wifi strength:
            long rssi = WiFi.RSSI();
            client.print("signal strength (RSSI):");
            client.print(rssi);
            client.println(" dBm<br><br>");

            // Serial.println("ACTION is ");
            // Serial.println(action);
            if(strcmp(action,"modus#")==0) {
              client.print("Modus:");
              doCommand(client, 18);
            } else if(strcmp(action,"status")==0) {
              client.print("Modus (2=auto/3=man/4=home): ");
              doCommand(client, 18);
              client.print("Status: ");
              doCommand(client, 1);
              client.print("Chargetime: ");
              doCommand(client, 21);
              client.print("Accu capacity: ");
              doCommand(client, 22);
              client.print("Accu capacity used: ");
              doCommand(client, 23);
              client.print("Accu Temp: ");
              doCommand(client, 24);
              client.print("Time since charge: ");
              doCommand(client, 25);
              client.print("Accu voltage: ");
              doCommand(client, 26);
            } else if(strcmp(action,"auto##")==0) {
              doCommand(client, 2);
            } else if(strcmp(action,"man###")==0) {
              doCommand(client, 3);
            } else if(strcmp(action,"home##")==0) {
              doCommand(client, 4);
            } else if(strcmp(action,"advanc")==0) {
              doCommand(client, 6);
            } else if(strcmp(action,"normal")==0) {
              doCommand(client, 7);
            } else if(strcmp(action,"tmrOn#")==0) {
              doCommand(client, 16);
            } else if(strcmp(action,"tmrOff")==0) {
              doCommand(client, 17);
            } else {

              client.println("No automower command given");
            }

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            //first show which line we got
            if(currentLine.startsWith("GET ")) {
              // Serial.println(currentLine);
            }
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /modus")) {
          strcpy(action, "modus#");
        }
        else if (currentLine.endsWith("GET /status")) {
          strcpy(action, "status");
        }
        else if (currentLine.endsWith("GET /auto")) {
          strcpy(action, "auto##");
        }
        else if (currentLine.endsWith("GET /man")) {
          strcpy(action, "man###");
        }
        else if (currentLine.endsWith("GET /home")) {
          strcpy(action, "home##");
        }
        else if (currentLine.endsWith("GET /advanced")) {
          strcpy(action, "advanc");
        }
        else if (currentLine.endsWith("GET /normal")) {
          strcpy(action, "normal");
        }
        else if (currentLine.endsWith("GET /timerOn")) {
          strcpy(action, "tmrOn#");
        }
        else if (currentLine.endsWith("GET /timerOff")) {
          strcpy(action, "tmrOff");
        }
      } else {
        //CLIENT CONNECTED BUT NO DATA AVAILABLE
        //MAKE SURE TO TIMEOUT WHEN TOO LONG!
        if(timeoutCounter++ > 100) {
          break; // while
        }
        delay(1);
      }
    }
  }

  closeClient(client);
}

void doCommand(WiFiClient client, int commandoID) {

  uint8_t commandAutomower[5] = { 0x0F, 0x01, 0xF1, 0x00, 0x00 }; //initialisiert Status (zur Sicherheit)
  uint8_t leer[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  uint8_t statusAutomower[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // Resultat leer initialisieren
  
  uint8_t requestStatusAutomower[5] = { 0x0F, 0x01, 0xF1, 0x00, 0x00 };
  uint8_t requestModusAutomower[5] = { 0x0F, 0x01, 0x2C, 0x00, 0x00 };
  
  uint8_t autoAutomower[5] = { 0x0F, 0x81, 0x2C, 0x00, 0x01 };
  uint8_t manuellAutomower[5] = { 0x0F, 0x81, 0x2C, 0x00, 0x00 };
  uint8_t chargeAutomower[5] = { 0x0F, 0x81, 0x2C, 0x00, 0x03 };
  uint8_t tasteYESAutomower[5] = { 0x0F, 0x80, 0x5F, 0x00, 0x12 };

  uint8_t chargingTime[5] = { 0x0F, 0x01, 0xEC, 0x00, 0x00 };
  uint8_t timeSinceChargingAutomower[5] = { 0x0F, 0x02, 0x34, 0x00, 0x00 };
  uint8_t batteryVoltageAutomower[5] = { 0x0F, 0x2E, 0xF4, 0x00, 0x00 };

  uint8_t batteryAutomower[5] = { 0x0F, 0x01, 0xEF, 0x00, 0x00 };
  uint8_t usedBatteryAutomower[5] = { 0x0F, 0x2E, 0xE0, 0x00, 0x00 };
  //uint8_t starkeAutomower[5] = { 0x0F, 0x00, 0x4D, 0x00, 0x00 };

  uint8_t tempBatteryAutomower[5] = { 0x0F, 0x02, 0x33, 0x00, 0x00 };

  uint8_t menuAdvanced[5] = { 0x0F, 0xCA, 0x54, 0x00, 0x03 };
  uint8_t menuNormal[5] = { 0x0F, 0xCA, 0x54, 0x00, 0x01 };

  uint8_t timerActiveren[5] = { 0x0F, 0xCA, 0x4E, 0x00, 0x00 };
  uint8_t timerDeactiveren[5] = { 0x0F, 0xCA, 0x4E, 0x00, 0x01 };

  //for joyriding:
  uint8_t moveSetRightWheel0[5] = { 0x0F, 0x92, 0x03, 0x00, 0x00 };
  uint8_t moveSetLeftWheel0[5]  = { 0x0F, 0x92, 0x23, 0x00, 0x00 };
  uint8_t moveSetRightWheel8000[5] = { 0x0F, 0x92, 0x03, 0x80, 0x00 };
  uint8_t moveSetLeftWheel8000[5]  = { 0x0F, 0x92, 0x23, 0x80, 0x00 };
  uint8_t moveSetMode0D[5] = { 0x0F, 0x81, 0x0D, 0x3A, 0x9D };
  uint8_t moveSetMode9A[5] = { 0x0F, 0x81, 0x9A, 0x00, 0x90 };
  uint8_t moveSetMode99[5] = { 0x0F, 0x81, 0x99, 0x00, 0x90 };
  uint8_t backKey[5] = { 0x0F, 0x80, 0x5F, 0x00, 0x0F };
  
  switch (commandoID) {
    case 1:
        memcpy(commandAutomower, requestStatusAutomower, sizeof(commandAutomower));
            break;
    case 2: //Automatik
        memcpy(commandAutomower, autoAutomower, sizeof(commandAutomower));
            break;
    case 3: //Manuell
        memcpy(commandAutomower, manuellAutomower, sizeof(commandAutomower));
            break;
    case 4: //Home (Ladestation)
        memcpy(commandAutomower, chargeAutomower, sizeof(commandAutomower));
            break;
    case 5: //Taste YES
        memcpy(commandAutomower, tasteYESAutomower, sizeof(commandAutomower));
            break;
    case 6: //Enable advanced menu (EXP)
        memcpy(commandAutomower, menuAdvanced, sizeof(commandAutomower));
            break;
    case 7: //Back to normal menu
        memcpy(commandAutomower, menuNormal, sizeof(commandAutomower));
            break;  
    case 16: //TIMER ACTIVEREN
        memcpy(commandAutomower, timerActiveren, sizeof(commandAutomower));
            break;
    case 17: //TIMER DEACTIVEREN
        memcpy(commandAutomower, timerDeactiveren, sizeof(commandAutomower));
            break;
    case 18: //Modus
        memcpy(commandAutomower, requestModusAutomower, sizeof(commandAutomower));
            break;  
            
    case 21: //Charging time
        memcpy(commandAutomower, chargingTime, sizeof(commandAutomower));
            break;
    case 22: //Akku Capaciteit
        memcpy(commandAutomower, batteryAutomower, sizeof(commandAutomower));
            break;
    case 23: //used capacity
        memcpy(commandAutomower, usedBatteryAutomower, sizeof(commandAutomower));
            break;
    case 24: //Temperatur Akku aktuell
        memcpy(commandAutomower, tempBatteryAutomower, sizeof(commandAutomower));
            break;
    case 25: //Zeit seit letztem Laden
        memcpy(commandAutomower, timeSinceChargingAutomower, sizeof(commandAutomower));
            break;
    case 26: //Batteriespannung
        memcpy(commandAutomower, batteryVoltageAutomower, sizeof(commandAutomower));
            break;      

       default: //no valid parameter
        return; // stop doCommand function
            break;
  }

  //Clear incoming buffer so we will not be reading any noise
  while(Serial1.available())
    Serial1.read();
  
  // Send
  Serial1.write(commandAutomower, 5);
  Serial1.flush(); //makes the program wait untill bytes are fully written on the stream
  client.println("BYTES TO AM: ");
  clientPrintHex8(client, commandAutomower, 5); 
  client.println("<br>");
  
  // Receive
  Serial1.readBytes(statusAutomower,5);

  client.println("BYTES FROM AM: ");
  clientPrintHex8(client, statusAutomower, 5); 
  client.println("<br>");
  
  if(memcmp(statusAutomower, leer, 5) == 0) {
    //Fehler, Automower gibt keine Antwort! Aufruf wird abgebrochen.
    client.println("No RS-232!");
    return; // stop doCommand function
  } else {

      if ((commandoID >= 2) && (commandoID <= 7)) { //commando check
        // Compare
        int antwort1 = statusAutomower[4] << 8 | statusAutomower[3]; // Bytes switched
        int commando1 = commandAutomower[3] << 8 | commandAutomower[4];
        int antwort2 = statusAutomower[0] << 8 | statusAutomower[2];
        int commando2 = commandAutomower[0] << 8 | commandAutomower[2];
        if ((commando1 == antwort1) && (commando2 == antwort2)) //commando check
        {
          client.println("commando OK");

          //got request to change modus from loxone, immediatly communicate back this change as an ack
          if ((commandoID >= 2) && (commandoID <= 4)) {
            modus = commandoID;
            sendModusUpdate();
          }
          
        }
        else
        {
          client.println("commando FAIL");
          return; // stop doCommand function
        }
      }

      if ((commandoID >= 16) && (commandoID <= 17)) { //Timer on/off
        // got request to change modus from loxone, immediatly communicate back this change as an ack
        int answer = statusAutomower[4] << 8 | statusAutomower[3];
        // attention mower gives back 0 if timer on, 1 if off. Send inverso to loxone
        answer = !answer;
        sendTimerUpdate(answer);
      }
      
      if (commandoID == 1) check
      {
        letzte zwei bytes vertauscht!
        unsigned int statusInt = statusAutomower[4] << 8 | statusAutomower[3];
        client.print("status=");
        clientPrintStatusName(client, statusInt);
        client.println("<br>");
      }

      if (commandoID == 18) //Modus check
      {
        letzte zwei bytes vertauscht!
        unsigned int modusInt = statusAutomower[4] << 8 | statusAutomower[3];
        client.print("modus=");
        clientPrintModusName(client, modusInt);
        client.println("<br>");
      }
    
    
      if ((commandoID >= 20)) //Messwerte 
      {
        received last two bytes are value
        int messwert = statusAutomower[4] << 8 | statusAutomower[3];
        client.print("com");
        client.print(commandoID);
        client.print("=");
        client.println(messwert);
        client.println("<br>");
      }
  }
}

void clientPrintHex8(WiFiClient client, uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {client.write("0");} 
         client.print(data[i],HEX); 
         client.write(" "); 
       }
}


void clientPrintModusName(WiFiClient client, int modusCode) {
  //NOT REALLY NAME BUT MY USED CODES
  /*
   * MY CODES: 2=A, 3=M, 4=H
   * 
   * CODE FROM AM: 0=M, 1=A, 3=H
   */
  switch (modusCode) {
    case 0: //MAN
      client.print("3");
      break;
    case 1: //AUTO
      client.print("2");
      break;
    case 3: //HOME
      client.print("4");
      break;

    default: //no valid parameter: send status
      client.print("Unknown_modus_");
      client.print(modusCode);
      break;
  }
}
void clientPrintStatusName(WiFiClient client, int statusCode) {
  //ATTENTION: MUST BE URL-ALLOWED STRING (no spaces etc)
  switch (statusCode) {
    case 6:
      client.print("Left_wheel_motor_blocked");
      break;
    case 12:
      client.print("No_loop_signal");
      break;
    case 18:
      client.print("Low_bat_voltage");
      break;
    case 26:
      client.print("Station_blocked");
      break;
    case 34:
      client.print("Mower_lifted");
      break;
    case 52:
      client.print("station_no_contact");
      break;
    case 54:
      client.print("Pin_expired");
      break;
    case 1000:
      client.print("leaving_station");
      break;
    case 1002:
      client.print("mowing");
      break;
    case 1006:
      client.print("Start_mowing");
      break;
    case 1008:
      client.print("mowing_started");
      break;
    case 1012:
      client.print("Start_mowing2");
      break;
    case 1014:
      client.print("charging");
      break;
    case 1016:
      client.print("waiting_timer");  
      break;
    case 1024:
      client.print("Parking_in_station");
      break;
    case 1036:
      client.print("square_mode");
      break;
    case 1038:
      client.print("stuck");
      break;
    case 1040:
      client.print("collission_or_dodge");
      break;
    case 1042:
      client.print("searching");
      break;
    case 1044:
      client.print("Stop");
      break;
    case 1048:
      client.print("Docking");
      break;
    case 1050:
      client.print("Leaving_station");
      break;   
    case 1052:
      client.print("error");
      break;
    case 1056:
      client.print("Waiting_for_use");
      break;
    case 1058:
      client.print("follow_boundary");
      break;
    case 1060:
      client.print("found_N-Signal");
      break;
    case 1062:
      client.print("stuck");
      break;
    case 1064:
      client.print("searching");
      break;
    case 1070:
      client.print("follow_guide_line");
      break;
    case 1072:
      client.print("follow_loop_wire");
      break;
    default: //no valid parameter: send status
      client.print("Unknown_status_");
      client.print(statusCode);
      break;
  }
}
