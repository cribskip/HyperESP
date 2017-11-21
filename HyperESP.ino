#include <NeoPixelBus.h>

/*
  Flashchip IDs:
  ESP201      0x1340ef  #define WINBOND_NEX_W25Q40_V  0x4013  4Mbit = 512K BYTE
  ESP01-black 0x1440c8  #define GIGADEVICE_GD25Q80  0x4014   8Mbit = 1M BYTE
  ESP01-blue  0x1340c8  #define GIGADEVICE_GD25Q40  0x4013   4Mbit = 512K BYTE
            0x1640c8  #define GIGADEVICE_GD25Q32  0x4016  32Mbit = 4M BYTE
  ebay chip   0x1640ef  #define WINBOND_NEX_W25Q32_V  0x4016  32Mbit = 4M BYTE



  ASCIIPort protocol:
  Lrgb\n
  LLrrggbb\n
  LLLrrggbb\n
  Led#, red, green, blue as ascii hex
  L can be '*' for all leds
  \n can be \r or ^ to pack multiple updates in one packet
  RESET
  STATUS
  VERSION
  BLANK set all leds to 222

  RAW protocol.
  no parsing, simply send RGBRGBRGBRGB to set a string of LEDs from #0

  RAW Delta protocol

  Byte 0
  [7..5] frame type
  0 normal update
  1 set soft max leds
  2 set auto update rate

  [0.4] update sequence number

  byte 1
  [7] this is max frag
  [012] fragment number

  Byte 2
  [0..6] start led index high boys
  Byte 3
  [0..7] start led index low byte

  Byte 4.. End
  3 bytes per led in sequence RGB

*/

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <WiFiUdp.h>

#define NeoPin 2
// LG LEDs
//#define pixelCount 297

// Mobil
#define pixelCount 60

//#define pixelCount 48

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(pixelCount, NeoPin);

// seriously... arduino is supposed to magically do this for me...
void fill (byte r, byte g, byte b);
void wifi_connect();
void handle_raw_port();
void handle_raw_delta_port() ;
int find_buffer_for_seq(int seqNum) ;
void handle_ascii_port() ;
void send_to_remote(IPAddress rIP, int rPort, char * pacBuf) ;
byte ascii2hex(char asc) ;

struct GrbPixel
{
  uint8_t G;
  uint8_t R;
  uint8_t B;
};
GrbPixel* pixels = (GrbPixel*) strip.Pixels();

RgbColor color;
uint8_t pos;
bool rainbow = false;

long unsigned int temp_mill;

long unsigned int this_mill;
long unsigned int last_mill;
unsigned long long  extended_mill;
unsigned long long mill_rollover_count;

unsigned int ASCIIPort = 2390;      // local port to listen for UDP packets
unsigned int RAWPort = 2391;      // local port to listen for UDP packets
unsigned int RAWDeltaPort = 2392;      // local port to listen for UDP packets

#define PACKET_BUFFER_SIZE  3 * pixelCount + 4
char packetBuffer[PACKET_BUFFER_SIZE]; //buffer to hold incoming and outgoing packets

#define NUM_BUFFERS 4
#define NUM_SEQ 8
char buffers[NUM_BUFFERS][3 * pixelCount];
int8_t sequence_frag_state[NUM_SEQ];  // which fragments are valid for this sequence?
int8_t sequence_to_buffer[NUM_SEQ] = { -1, -1, -1, -1, -1, -1, -1, -1}; // in which of the 4 buffers is this sequence# stored in?
int8_t buffer_to_sequence[NUM_BUFFERS] = { -1, -1, -1, -1}; // which sequence does this buffer hold?

// A UDP instance to let us send and receive packets over UDP
WiFiUDP ASCIIudp;
WiFiUDP RAWudp;
WiFiUDP RAWDeltaudp;

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
RgbColor Wheel(uint8_t WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return RgbColor(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return RgbColor(0, WheelPos * 3, 255 - WheelPos * 3);
  } else
  {
    WheelPos -= 170;
    return RgbColor(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle() {
  for (uint16_t j = 0; j < 256 * 5; j++) // complete 5 cycles around the color wheel
  {
    for (uint16_t i = 0; i < pixelCount; i++)
    {
      // generate a value between 0~255 according to the position of the pixel
      // along the strip
      pos = ((i * 256 / pixelCount) + j) & 0xFF;
      // calculate the color for the ith pixel
      color = Wheel( pos );
      // set the color of the ith pixel
      strip.SetPixelColor(i, color);
    }
    strip.Show();
    delay(50);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for serial attach
  strip.Begin();

  fill(3, 3, 3);
  delay(200);
  fill(0, 0, 0);

  WiFiManager wifiManager;
  wifiManager.setRemoveDuplicateAPs(false);
  WiFiManagerParameter custom_text("<p>Scan zum Suchen, WLAN ausw&auml;hlen, Passwort eingeben. Save dr&uuml;cken und im WLAN suchen.</p>");
  wifiManager.addParameter(&custom_text);
  wifiManager.setConfigPortalTimeout(60);

  wifiManager.autoConnect("LED-Streifen 60");

  if (!MDNS.begin("LED-Streifen", WiFi.localIP())) {
    Serial.println(F("Error setting up MDNS responder!"));
  } else
    Serial.println(F("mDNS responder started"));

  fill(0, 12, 0);
  delay(500);
  fill(0, 0, 0);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
    rainbowCycle();
  } else {
    this_mill = millis();
    if (last_mill > this_mill) {  // rollover
      mill_rollover_count ++;
    }
    extended_mill = (mill_rollover_count << (8 * sizeof(this_mill))) + this_mill;
    last_mill = this_mill;

    handle_raw_port();
    handle_raw_delta_port();
    handle_ascii_port();
  }
}

void handle_raw_port() {
  if (!RAWudp) {
    Serial.println(F("RE-Starting UDP"));
    RAWudp.begin(RAWPort);
    Serial.print(F("Local RAWport: "));
    Serial.println(RAWudp.localPort());
  }
  if (!RAWudp) {
    Serial.println(F("RE-Start failed."));
    return;
  }

  int cb = RAWudp.parsePacket();
  if (!cb) {
    //    Serial.println("no packet yet");
  }
  else {
    //    Serial.print("packet received, length=");
    //    Serial.println(cb);
    // We've received a packet, read the data from it
    RAWudp.read(packetBuffer, PACKET_BUFFER_SIZE); // read the packet into the buffer

    //      Serial.println (cb);
    //      Serial.print (": ");
    int neo = 0;
    for (int i = 0; (i < cb) && (i < PACKET_BUFFER_SIZE); i += 3) {
      RgbColor t = RgbColor( packetBuffer[i], packetBuffer[i + 1], packetBuffer[i + 2]);
      if (neo < pixelCount) {
        strip.SetPixelColor(neo++, t);
      }
    }
    strip.Show();
  }
}

void handle_raw_delta_port() {
  if (!RAWDeltaudp) {
    Serial.println(F("RE-Starting UDP"));
    RAWDeltaudp.begin(RAWDeltaPort);
    Serial.print(F("Local RAWDeltaport: "));
    Serial.println(RAWDeltaudp.localPort());
  }
  if (!RAWDeltaudp) {
    Serial.println(F("RE-Start failed."));
    return;
  }

  int cb = RAWDeltaudp.parsePacket();
  if (!cb) {
  }
  else {
    //    Serial.print("packet received, length=");
    //    Serial.println(cb);
    // We've received a packet, read the data from it
    RAWDeltaudp.read(packetBuffer, PACKET_BUFFER_SIZE); // read the packet into the buffer
    //      Serial.println (cb);
    //      Serial.print (": ");


    if ( (cb >= 4) && ((packetBuffer[0] & 0xf0) == 0) ) {

      int dataLen = cb - 4;
      int seqNum = packetBuffer[0] & 0x0f;
      int fragNum = packetBuffer[0] & 0x0f;
      int fragStart = 256 * packetBuffer[2] + packetBuffer[3];


      // find the buffer for this sequence
      int bufNum = find_buffer_for_seq(seqNum);
      if (bufNum >= 0) {
        for (int i = 0; i < dataLen; i++) {
          if (fragStart + i < sizeof(buffers[0])) {
            //          buffers[bufNum][fragStart+i] = packetBuffer[4+i];
          }
        }
      }


      // original code with no reassembly
      int neo = 256 * packetBuffer[2] + packetBuffer[3];
      for (int i = 4; (i < cb) && (i < PACKET_BUFFER_SIZE); i += 3) {
        RgbColor t = RgbColor( packetBuffer[i], packetBuffer[i + 1], packetBuffer[i + 2]);
        if (neo < pixelCount) {
          strip.SetPixelColor(neo++, t);
        }
      }

      temp_mill = millis();
      strip.Show();
      temp_mill = millis() - temp_mill;
      //    Serial.print ("Show took ");
      //    Serial.println (temp_mill);

      //  #define DROP_PKTS
#ifdef DROP_PKTS
      int drop_count = 0;
      while (RAWDeltaudp.parsePacket()) {
        drop_count++;
      }
      if (drop_count) {
        Serial.print (F("drop_count is" ));
        Serial.println (drop_count);
      }
#endif
    }
  }
}

int find_buffer_for_seq(int seqNum) {

  int ret = sequence_to_buffer[seqNum];
  if (ret >= 0) {
    return ret;
  } else {
    // need to find a free slot
    return -1; // give up, cant find one

  }
}

void handle_ascii_port() {
  if (!ASCIIudp) {
    Serial.println(F("RE-Starting UDP"));
    ASCIIudp.begin(ASCIIPort);
    Serial.print(F("Local ASCII port: "));
    Serial.println(ASCIIudp.localPort());
  }
  if (!ASCIIudp) {
    Serial.println(F("RE-Start failed."));
    return;
  }
  int cb = ASCIIudp.parsePacket();
  if (!cb) {
    //    Serial.println("no packet yet");
  }
  else {
    IPAddress remoteIP = ASCIIudp.remoteIP();
    int remotePort = ASCIIudp.remotePort();
    /*
        Serial.print("packet received, length=");
        Serial.print(cb);
        Serial.print(" from port");
        Serial.print(remotePort);
        Serial.print(" from IP");
        Serial.println(remoteIP);
    */
    ASCIIudp.read(packetBuffer, PACKET_BUFFER_SIZE); // read the packet into the buffer

    if (cb < PACKET_BUFFER_SIZE) {
      packetBuffer[cb] = 0;
    }

    //      Serial.println (cb);
    //      Serial.print (": ");
    /*
        for (int i=0; (i<cb) && (i<PACKET_BUFFER_SIZE); i++) {
          Serial.write(packetBuffer[i]);
        }
        Serial.println("");
    */
    for (int i = 0; (i < cb) && (i < PACKET_BUFFER_SIZE); i++) {
      if (packetBuffer[i] == '\r') {
        packetBuffer[i] = '\n';
      }
      if (packetBuffer[i] == '^') {
        packetBuffer[i] = '\n';
      }
    }

    byte neo, red, green, blue;
    if (strncmp(packetBuffer, "RESET\n", cb) == 0) {
      ESP.reset();
    }
    if (strncmp(packetBuffer, "STATUS\n", cb) == 0) {
      long int secs = (extended_mill / 1000);
      long int mins = (secs / 60);
      long int hours = (mins / 60);
      long int days = (hours / 24);

      memset(packetBuffer, 0, PACKET_BUFFER_SIZE);
      sprintf (packetBuffer, "Uptime: %d days, %02d:%02d:%02d.%03d\r\nFreeHeap: 0x%x\r\nFlashChipId: 0x%x\r\nSignal: %d\r\n",
               days, hours % 24, mins % 60, secs % 60, (int) extended_mill % 1000, ESP.getFreeHeap(), ESP.getFlashChipId(), WiFi.RSSI() );
      send_to_remote(remoteIP, remotePort, packetBuffer);
    }
    if (strncmp(packetBuffer, "VERSION\n", cb) == 0) {
      memset(packetBuffer, 0, PACKET_BUFFER_SIZE);
      sprintf (packetBuffer, "File: %s\r\nBuild Date: %s %s\r\n", __FILE__, __DATE__, __TIME__);
      send_to_remote(remoteIP, remotePort, packetBuffer);

    }
    if (strncmp(packetBuffer, "BLANK\n", cb) == 0) {
      fill(2, 2, 2);
      strip.Show();
    }
    if (packetBuffer[4] == '\n') {
      for (int i = 0; (i < (cb - 4)) && (i < PACKET_BUFFER_SIZE); i += 5) {
        if (packetBuffer[i + 4] == '\n') {
          packetBuffer[i + 4] = 0;
          neo = ascii2hex(packetBuffer[i + 0]);
          red = 16 * ascii2hex(packetBuffer[i + 1]);
          green = 16 * ascii2hex(packetBuffer[i + 2]);
          blue = 16 * ascii2hex(packetBuffer[i + 3]);
          RgbColor t = RgbColor( red, green, blue);
          //          if ((packetBuffer[i+0] & 0b11011111) == 'F') {
          if ((packetBuffer[i + 0]) == '*') {
            fill (red, green, blue);
          } else if (neo < pixelCount) {
            strip.SetPixelColor(neo, t);
          }
        }
      }
    }
    if (packetBuffer[8] == '\n') {
      for (int i = 0; (i < (cb - 8)) && (i < PACKET_BUFFER_SIZE); i += 9) {
        if (packetBuffer[i + 8] == '\n') {
          packetBuffer[i + 8] = 0;
          neo = 16 * ascii2hex(packetBuffer[i + 0]) + ascii2hex(packetBuffer[i + 1]);
          red = 16 * ascii2hex(packetBuffer[i + 2]) + ascii2hex(packetBuffer[i + 3]);
          green = 16 * ascii2hex(packetBuffer[i + 4]) + ascii2hex(packetBuffer[i + 5]);
          blue = 16 * ascii2hex(packetBuffer[i + 6]) + ascii2hex(packetBuffer[i + 7]);
          RgbColor t = RgbColor( red, green, blue);
          //          if ((packetBuffer[i+0] & 0b11011111) == 'F') {
          if ((packetBuffer[i + 0]) == '*') {
            fill (red, green, blue);
          } else if (neo < pixelCount) {
            strip.SetPixelColor(neo, t);
          }
        }
      }
    }
    if (packetBuffer[9] == '\n') {
      for (int i = 0; (i < (cb - 9)) && (i < PACKET_BUFFER_SIZE); i += 10) {
        if (packetBuffer[i + 9] == '\n') {
          packetBuffer[i + 9] = 0;
          neo = 256 * ascii2hex(packetBuffer[i + 0]) + 16 * ascii2hex(packetBuffer[i + 1]) + ascii2hex(packetBuffer[i + 2]);
          red = 16 * ascii2hex(packetBuffer[i + 3]) + ascii2hex(packetBuffer[i + 4]);
          green = 16 * ascii2hex(packetBuffer[i + 5]) + ascii2hex(packetBuffer[i + 6]);
          blue = 16 * ascii2hex(packetBuffer[i + 7]) + ascii2hex(packetBuffer[i + 8]);
          RgbColor t = RgbColor( red, green, blue);
          if ((packetBuffer[i + 0]) == '*') {
            fill (red, green, blue);
          } else if (neo < pixelCount) {
            strip.SetPixelColor(neo, t);
          }
        }
      }
    }
    strip.Show();
  }
  //  delay(100);
}

void send_to_remote(IPAddress rIP, int rPort, char * pacBuf) {
  ASCIIudp.beginPacket(rIP, rPort);
  //    ASCIIudp.write(packetBuffer, PACKET_BUFFER_SIZE);
  ASCIIudp.write(pacBuf, strlen(pacBuf));
  ASCIIudp.endPacket();
}

byte ascii2hex(char asc) {
  if ((asc >= '0') && (asc <= '9')) {
    return asc - '0';
  }
  else if ((asc >= 'a') && (asc <= 'f')) {
    return 10 + asc - 'a';
  }
  else if ((asc >= 'A') && (asc <= 'F')) {
    return 10 + asc - 'A';
  }
  else {
    return 0;
  }
}


int splitString(char separator, String input, String results[], int numStrings) {
  int idx;
  int last_idx = 0;
  int retval = 0;
  //    message += "numStrings: ";
  //    message += numStrings;
  //    message += "\n";

  for (int i = 0; i < numStrings; i++) {
    results[i] = "";     // pre clear this
    idx = input.indexOf(separator, last_idx);
    // message += "i: " ; message += i;
    // message += " idx: " ; message += idx;
    // message += " last_idx: " ; message += last_idx;
    if ((idx == -1) && (last_idx == -1)) {
      break;
    } else {
      results[i] = input.substring(last_idx, idx);
      retval ++;
      // message += " results: "; message += results[i];
      // message += "\n";
      if (idx != -1) {
        last_idx = idx + 1;
      } else {
        last_idx = -1;
      }
    }
  }
  return retval;
}

void fill (byte r, byte g, byte b) {
  for (int neo = 0; neo < pixelCount; neo++) {
    //    strip.SetPixelColor(neo, RgbColor( r,g,b));
    pixels[neo].R = r;
    pixels[neo].G = g;
    pixels[neo].B = b;
    strip.Dirty();
  }
  strip.Show();
}
