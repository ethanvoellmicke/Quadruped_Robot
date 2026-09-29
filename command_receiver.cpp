/*this recieves the commands for the robot
it can recivieve commands through the USB serial port, going from serial monitor to the arduino
it can also revcieve commmands over wifi using UDP where the nano is constantly listening on UDP port 5005


*/

#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "command_receiver.h"
#include "gait_controller.h"
#include "servo_controller.h"

namespace
{
const char WIFI_NAME[] = "iPhone";
const char WIFI_PASSWORD[] = "12345678";
constexpr unsigned int UDP_PORT = 5005;//nano ip address
constexpr size_t COMMAND_BUFFER_SIZE = 96;//the largest amount of bytes a text command can have, more than enough
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 5000;//tries to connect to wifi every 5000 millis instead of constantly

WiFiUDP udp;
char commandBuffer[COMMAND_BUFFER_SIZE];//this is the array that has the characters sent from EITHER wifi or serial
unsigned long lastWiFiAttemptMs = 0; //stores the time/millis() when wifi tried connecting before, allows to send every 5 seconds
bool udpStarted = false;//dont open UDP listener more than once
bool previousWiFiConnected = false;//switches when wifi connects or disconnects

//this function allows text with spaces infront or behind to be read by removing these spaces
void trimInPlace(char *text)
{
  if (text == nullptr) return;//checking for invalid text with pointer
  char *start = text;//maing new pointe rfor first non space character
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) ++start;//the string ends in \0, so continue if that \0 isnt reached or current character is a space or tab etc
  if (start != text) memmove(text, start, strlen(start) + 1);//move the relavent characters to beginnnign of array

  const size_t length = strlen(text);//get the new length
  if (length == 0) return;//return if zero

  size_t end = length;
  while (end > 0 && isspace(static_cast<unsigned char>(text[end - 1]))) --end;//go backwards to non space character
  text[end] = '\0'; //put \0 right after the last non spcae character
}

void lowercaseInPlace(char *text)//makes the text case insensitive
{
  for (size_t i = 0; text[i] != '\0'; ++i)
  {
    text[i] = static_cast<char>(tolower(static_cast<unsigned char>(text[i])));
  }
}

const char *modeName(GaitMode mode)
{
  switch (mode)
  {
    case GaitMode::Stand: return "stand";
    case GaitMode::Crawl: return "crawl";
    case GaitMode::Trot: return "trot";
    default: return "unknown";
  }
}

/*
 a lot of the commands outside of the gaits consist of a command and a number like "speed 4" or "turn 0.5"
 this function translates these

 command = speed 4
 prefix = speed
 value = 4
 */
bool parseFloatArgument(const char *command, const char *prefix, float &value)
{
  const size_t prefixLength = strlen(prefix);//get amount of characters in prefix
  if (strncmp(command, prefix, prefixLength) != 0) return false;//returns 0 when matched

  const char *argument = command + prefixLength;//moves pointer past the prefix word, arguement points at spcae before value
  while (*argument == ' ') ++argument;//skip the spaces
  if (*argument == '\0') return false;//if nothing comes afer arguement, theres no number

  char *endPointer = nullptr;//pointer goes to firxt character that isnt part of number
  const float parsed = strtof(argument, &endPointer);//make current value at pointed a float
  if (endPointer == argument) return false;//if theres no number, but instead nothing or a letter or something return false

  while (*endPointer != '\0' && isspace(static_cast<unsigned char>(*endPointer))) ++endPointer;//ignore space after number
  if (*endPointer != '\0') return false; 

  value = parsed;
  return true;//commant is valid
}

/*
 MAIN FUNCTION
 revieves serial and udp commands, and calls the corresponding function in gait_controller
 
 */
void processCommand(char *command)
{
  trimInPlace(command);//remove the spaces
  lowercaseInPlace(command);//case insensitive
  if (command[0] == '\0') return;//if nothing after command ignore it

//if stand or stop typed set gait mode correspondly
  if (strcmp(command, "stand") == 0 || strcmp(command, "stop") == 0)
  {
    setGaitMode(GaitMode::Stand);
    Serial.println("Command: stand");
    return;
  }
//if crawl or walk typed, move forward, set gait mode
  if (strcmp(command, "crawl") == 0 || strcmp(command, "walk") == 0)
  {
    setForwardCommand(1.0f);
    setTurnCommand(0.0f);
    setGaitMode(GaitMode::Crawl);
    Serial.println("Command: crawl forward");
    return;
  }

//if trot typed, move forward and set gait mode
  if (strcmp(command, "trot") == 0)
  {
    setForwardCommand(1.0f);
    setTurnCommand(0.0f);
    setGaitMode(GaitMode::Trot);
    Serial.println("Command: trot forward");
    return;
  }

//if straight typed, set turn to 0
  if (strcmp(command, "straight") == 0)
  {
    setTurnCommand(0.0f);
    Serial.println("Turn command reset to 0");
    return;
  }

//print status
  if (strcmp(command, "status") == 0)
  {
    printRobotStatus();
    return;
  }

  float value = 0.0f;

//now takes in prefix and value, this gives gait controller the set speed
  if (parseFloatArgument(command, "speed", value))
  {
    setGaitSpeed(value);
    Serial.print("Speed set to ");
    Serial.println(getGaitSpeed(), 3);
    return;
  }

// this sets height in gait controller
  if (parseFloatArgument(command, "height", value))
  {
    setGroundHeight(value);
    Serial.print("Height set to ");
    Serial.println(getGroundHeight(), 3);
    return;
  }

//sets forward command in gait controller
  if (parseFloatArgument(command, "forward", value))
  {
    setForwardCommand(value);
    Serial.print("Forward command set to ");
    Serial.println(getForwardCommand(), 3);
    return;
  }

//sets turning command in gait controller
  if (parseFloatArgument(command, "turn", value))
  {
    setTurnCommand(value);
    Serial.print("Turn command set to ");
    Serial.println(getTurnCommand(), 3);
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(command);
}

//reconnecting to wifi without stopping controller
void connectWiFiNonBlocking()
{
  if (WiFi.status() == WL_CONNECTED) return; // if connected, leave

  const unsigned long now = millis();
  if (now - lastWiFiAttemptMs < WIFI_RETRY_INTERVAL_MS) return;//after 5000ms, run it

  lastWiFiAttemptMs = now;
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_NAME);
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
}

/*
 Read commands from USB serial
 characters arrive one at a time, so they must be arranged into commandBuffer which is passed to processCommand()
 */
void readSerialCommands()
{
  static size_t length = 0;

  while (Serial.available() > 0)//processing every character waiting in the serial recieve buffer
  {
    const char character = static_cast<char>(Serial.read());//read one byte and make check if character
    if (character == '\r') continue;//\r means keep going

    if (character == '\n')//this means new line, so comand was finished being typed
    {
      commandBuffer[length] = '\0';//this adds a \0 to the end of the command
      processCommand(commandBuffer);
      length = 0;//resets the buffer position to index 0
      continue;
    }

    if (length + 1 < COMMAND_BUFFER_SIZE)//if theres enough room for new character plus \0
    {
      commandBuffer[length++] = character;//add the character
    }
    else
    {
      length = 0;//reset length becasue command went over the 96 byte buffer
      Serial.println("Serial command too long, discarded");
    }
  }
}

/*
 reads the commands sent over wifi from computer using UDP

 Computer sends UDP packet, arduino uses udp().parsePacket() and udp.read(), then that goes into commandBuffer then to processCommand
 */
void readUdpCommands()
{
  if (WiFi.status() != WL_CONNECTED) return;

  while (true)
  {
    const int packetSize = udp.parsePacket();
    if (packetSize <= 0) break;

    const int bytesToRead =
      packetSize < static_cast<int>(COMMAND_BUFFER_SIZE - 1)
      ? packetSize
      : static_cast<int>(COMMAND_BUFFER_SIZE - 1);

    const int bytesRead = udp.read(
      reinterpret_cast<uint8_t *>(commandBuffer),
      bytesToRead
    );

    while (udp.available() > 0) udp.read();

    if (bytesRead > 0)
    {
      commandBuffer[bytesRead] = '\0';
      processCommand(commandBuffer);
    }
  }
}
}

void initializeCommandReceiver()
{
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Wi-Fi module not detected; serial commands still work");
    return;
  }

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000UL)
  {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    udp.begin(UDP_PORT);
    udpStarted = true;
    previousWiFiConnected = true;
    Serial.print("Wi-Fi connected. Nano IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Listening for text commands on UDP port ");
    Serial.println(UDP_PORT);
  }
  else
  {
    Serial.println("Wi-Fi not connected yet; gait and serial control remain active");
    lastWiFiAttemptMs = millis();
  }
}

void updateCommandReceiver()
{
  readSerialCommands();
  connectWiFiNonBlocking();

  const bool isConnected = WiFi.status() == WL_CONNECTED;

  if (isConnected && !udpStarted)
  {
    udp.begin(UDP_PORT);
    udpStarted = true;
    Serial.print("UDP listener active. IP: ");
    Serial.println(WiFi.localIP());
  }
  else if (!isConnected && previousWiFiConnected)
  {
    udp.stop();
    udpStarted = false;
    Serial.println("Wi-Fi disconnected; local gait continues");
  }

  previousWiFiConnected = isConnected;
  readUdpCommands();
}

void printRobotStatus()
{
  Serial.print("mode=");
  Serial.print(modeName(getGaitMode()));
  Serial.print(" speed=");
  Serial.print(getGaitSpeed(), 3);
  Serial.print(" height=");
  Serial.print(getGroundHeight(), 3);
  Serial.print(" forward=");
  Serial.print(getForwardCommand(), 3);
  Serial.print(" turn=");
  Serial.print(getTurnCommand(), 3);
  Serial.print(" wifi=");
  Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
}
