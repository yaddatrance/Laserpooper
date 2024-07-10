#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "PinAssignments.h"
#include "Globals.h"
#include <SD.h>
#include "TeensyID.h" // https://github.com/sstaub/TeensyID

const int chipSelect = BUILTIN_SDCARD; 
Sd2Card card;
SdVolume volume;
SdFile root;
File file;

#define VERSION "Laserpooper 616 v0.2a"
#define max(a,b) ((a) > (b) ? (a) : (b))

#define USBSERIAL Serial

#define LOGLEVEL 0

// OLED CODE
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

IntervalTimer myTimer;

bool paused = false; // Flag to indicate if data transmission should be paused
int filectr = 0;

void setup() {
  Serial.begin(921600);
  USBSERIAL.setTimeout(0);
  Serial.println("Serial ready! eh");
  Wire2.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      for (;;) { // Don't proceed, loop forever
          Serial.println(F("SSD1306 no donut"));
          delay(1000);
      }
  }
  Wire2.setClock(1000000);

  display.clearDisplay();
  display.setTextSize(1); // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0); // Start at top-left corner
  display.println(VERSION);

//  snprintf(strbuf, sizeof(strbuf), "%s", teensySN());
  snprintf(strbuf, sizeof(strbuf), "Serial: %lu", teensyUsbSN());
  display.println(String(strbuf));
//  display.println(F("ShareholderValue++"));
  display.display();
  klog_flush("Eat my shorts");

  buffer0 = (Point*)malloc(MAX_POINTS_IN_FRAME * sizeof(Point));
  buffer1 = (Point*)malloc(MAX_POINTS_IN_FRAME * sizeof(Point));
  if (!buffer0 || !buffer1) {
      Serial.println("Failed to allocate memory for buffers!");
      klog_flush("Buffer memory Fail");
      return;
  }
  memset(buffer0, 0, (MAX_POINTS_IN_FRAME * sizeof(Point)));
  memset(buffer1, 0, (MAX_POINTS_IN_FRAME * sizeof(Point)));
  renderBufferIndex = 0;
  renderTotalPoints = 1;


  currentBuffer = buffer0;
  whichBuffer = 0; // points to active currentBuffer
  renderBuffer = buffer1;

  setupDAC();

  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    klog_flush("card.init Fail");
    SDPresent = 1;
  } else {
    klog_flush("card.init OK");
    SDPresent = 0;
  }    
  if (SDPresent) {
    if (SD.begin(chipSelect)) {
      klog_flush("sd.begin");
    }
  }
  lineCount = 0;
  // Callback kickpoint() every pointDelay microseconds. Default = 30
  myTimer.begin(kickpoint, pointDelay);
}


void kickpoint() {
    if (renderBufferIndex >= renderTotalPoints) {
        renderBufferIndex = 0; // go back to beginning
        if (bufferSwap) { // swap buffer at end of frame
          bufferSwap = false;
          Point* temp = currentBuffer;
          currentBuffer = renderBuffer;
          renderBuffer = temp;
          renderTotalPoints = totalPoints;
        }
    }
    Point p = renderBuffer[renderBufferIndex];              
    laserPoint(p.y, p.x, p.r, p.g, p.b);
    renderBufferIndex++;
}

int sSr = 0;
int16_t sS = 0;

void loop() {
    unsigned long currentMillis = millis();
    frameCount++;
    if (currentMillis - lastUpdate >= updateInterval) { // 0.25 second screen updates
        lastUpdate = currentMillis; // Reset the timer
        renderDisplay();
    }
/*    // Calculate FPS every  second
    if (currentMillis - lastTime >= 1000) {
      fps = frameCount / ((currentMillis - lastTime) / 1000.0);
      lastTime = currentMillis;
      frameCount = 0;
    }    */

    if (Serial.available()) {
        processIncomingData(); // Grab an array of Point from serial using StreamSend::receiveObject() and fill currentBuffer[]
    }
    

}

void renderDisplay() {
    display.clearDisplay();
    display.setCursor(0, 0); // Start at top-left corner
    display.print(VERSION);
    if (totalPoints) {
        display.println(String("b:") + totalPoints);
    } else {
        snprintf(strbuf, sizeof(strbuf), "Serial: %lu", teensyUsbSN());
      //  snprintf(strbuf, sizeof(strbuf), "%s", teensySN());
        display.println(String(strbuf));
//        display.println(F("ShareholderValue++"));
//        display.println(String("fps:") + fps);
    }
    
    display.setCursor(8 * 15, 8); 
    display.print(String(spinner[spinnerstate])), spinnerstate = (spinnerstate + 1) % MAX_SPINNERS;
    for (int i = 0; i < lineCount; i++) {
        display.setCursor(0, 16 + i * 8); // Set cursor to start of line (8 pixels per line height)
        display.println(lines[i]);
    }
    display.display();
}


static uint8_t tempBuffer[sizeof(Point)];
static size_t tempBufferIndex = 0;
void processIncomingData() {
    static uint8_t incomingBuffer[512];
    static uint8_t headerBuffer[4];
    static size_t headerIndex = 0;
    size_t i = 0;

    size_t bytesRead = Serial.readBytes(reinterpret_cast<char*>(incomingBuffer), sizeof(incomingBuffer));

    bytesReadCtr += bytesRead;

    if (LOGLEVEL) {
        snprintf(strbuf, sizeof(strbuf), "SbR=%d/%d c%d", bytesRead, bytesReadCtr, packetCtr++);
        klog(String(strbuf));
    }

    while (i < bytesRead) {
        if (currentState == ReceiveState::IDLE) {
            // Fill the header buffer if not already filled
            while (headerIndex < 4 && i < bytesRead) {
                headerBuffer[headerIndex++] = incomingBuffer[i++];
            }

            // Check if we have a complete header
            if (headerIndex == 4) {
                if ((headerBuffer[0] == 0xFF) && 
                    (headerBuffer[1] == 0x00 || headerBuffer[1] == 0x01 || headerBuffer[1] == 0x02)) {
                    if (LOGLEVEL) {
                        klog("Start");
                    }
                    currentCommand = headerBuffer[1];
                    if (currentCommand == 0) {

                    }
                    totalPoints = 256 * headerBuffer[2] + headerBuffer[3];
                    totalSize = totalPoints * sizeof(Point);
                    remainingBytes = totalSize;
                    currentState = ReceiveState::RECEIVING_DATA;
                } else {
                    snprintf(strbuf, sizeof(strbuf), "SHFT [%02x] %d", headerBuffer[0], i);
                    klog(String(strbuf));
                    // Shift buffer left and try again
                    headerBuffer[0] = headerBuffer[1];
                    headerBuffer[1] = headerBuffer[2];
                    headerBuffer[2] = headerBuffer[3];
                    headerIndex = 3;
                }
            }
        }

        if (currentState == ReceiveState::RECEIVING_DATA) {
            if (LOGLEVEL) {
                snprintf(strbuf, sizeof(strbuf), "Recv %d pts i=%d", totalPoints, i);
                klog(String(strbuf));
            }

            size_t bytesToProcess = min(bytesRead - i, remainingBytes);
            switch (currentCommand) {
                case 0x00: {
                    size_t remainingTempBytes = sizeof(Point) - tempBufferIndex;

                    if (bytesToProcess < remainingTempBytes) {
                        // Not enough bytes to complete a Point
                        memcpy(tempBuffer + tempBufferIndex, &incomingBuffer[i], bytesToProcess);
                        tempBufferIndex += bytesToProcess;
                        remainingBytes -= bytesToProcess;
                        i += bytesToProcess;
                    } else {
                        // Complete the current Point
                        memcpy(tempBuffer + tempBufferIndex, &incomingBuffer[i], remainingTempBytes);
                        memcpy(&currentBuffer[currentBufferIndex], tempBuffer, sizeof(Point));
                        currentBufferIndex++;
                        remainingBytes -= remainingTempBytes;
                        i += remainingTempBytes;
                        tempBufferIndex = 0;

                        // Process remaining bytes
                        size_t pointsToCopy = (bytesToProcess - remainingTempBytes) / sizeof(Point);
                        size_t remainingBytesAfterCopy = (bytesToProcess - remainingTempBytes) % sizeof(Point);

                        memcpy(&currentBuffer[currentBufferIndex], &incomingBuffer[i], pointsToCopy * sizeof(Point));
                        currentBufferIndex += pointsToCopy;
                        remainingBytes -= pointsToCopy * sizeof(Point);
                        i += pointsToCopy * sizeof(Point);

                        if (remainingBytesAfterCopy > 0) {
                            memcpy(tempBuffer, &incomingBuffer[i], remainingBytesAfterCopy);
                            tempBufferIndex = remainingBytesAfterCopy;
                            remainingBytes -= remainingBytesAfterCopy;
                            i += remainingBytesAfterCopy;
                        }
                    }

                    if (remainingBytes <= 0) {
                        currentBufferIndex = 0; // reset index
                        packetCtr = 0;
                        bytesReadCtr = 0;
                        currentState = ReceiveState::IDLE;
                        bufferSwap = true;
                        headerIndex = 0; // Reset header index
                        if (LOGLEVEL) {
                            klog(String("end"));
                        }
                    }
                    break;
                }
                case 0x01: {
                    klog("CFG");
                    if (remainingBytes <= bytesRead - i) {
                        pointsPerSecond = *((int*)&incomingBuffer[i]);
                        i += bytesToProcess;
                        remainingBytes = 0;
                        currentState = ReceiveState::IDLE;
                        headerIndex = 0; // Reset header index
                    }
                    break;
                }
                case 0x02: {
                    klog("QUERY");
                    //sendBufferState();
                    sendID();
                    remainingBytes -= bytesToProcess;
                    i += bytesToProcess;
                    if (remainingBytes <= 0) {
                        currentState = ReceiveState::IDLE;
                        headerIndex = 0; // Reset header index
                    }
                    break;
                }
                default: {
                    klog("WTF?");
                    i += bytesToProcess;
                    remainingBytes -= bytesToProcess;
                    if (remainingBytes <= 0) {
                        currentState = ReceiveState::IDLE;
                        headerIndex = 0; // Reset header index
                    }
                    break;
                }
            }
        }
    }
}

void sendBufferState() {
    char buffer[64];
    int filledPoints = currentBufferIndex; // Number of points currently filled
    sprintf(buffer, "BufPts: %d", filledPoints);
    Serial.println(buffer);
}

void sendID() {
    //char buffer[64];
    //snprintf(buffer, sizeof(buffer), "%s", teensySN());
    snprintf(strbuf, sizeof(strbuf), "%lu", teensyUsbSN());
    Serial.println(String(strbuf));
}


void debugnoscrollklog(String message) { // logs to screen
  if (lineCount < MAX_LINES) {
        lines[lineCount++] = message; // Add new message at the next available line
  }
}

void klog(String message) { // logs to screen
  if (SDPresent) {
    file = SD.open("logfile.msg",FILE_WRITE);
    file.seek(0);
    file.write(message.c_str());
    file.write("\n");
    file.close();
  }
  if (lineCount >= MAX_LINES) { // Shift all lines up    
        for (int i = 0; i < MAX_LINES - 1; i++) {
            lines[i] = lines[i + 1];
        }
        lines[MAX_LINES - 1] = message; // Add new message at the bottom
  } else {
        lines[lineCount++] = message; // Add new message at the next available line
  }
}


void klog_flush(String message) {
  klog(message);
  renderDisplay();
}

void laserPoint(int rawX, int rawY, int rawR, int rawG, int rawB) {
    int16_t X = rawX;
    int16_t Y = rawY;

    // Perceptual RGB to greyscale for blank channel
    int rawK = ((0.299 * (float)rawR + 0.587 * (float)rawG + 0.114 * (float)rawB) * 65535.0);

    int16_t R = 256 * rawR - 32768;
    int16_t G = 256 * rawG - 32768;
    int16_t B = 256 * rawB - 32768;
    int16_t K = 256 * rawK - 32768;

    // at 10MHz (HALFCLKns 50) bit banging takes up 3.4us for delaynanoseconds calls, plus around 100x digitalWriteFast times 
    // total appears to be about 5us 
    //--   WS=0, Right XKG
    digitalWriteFast(WS1, 0);
    digitalWriteFast(WS2, 0);
    digitalWriteFast(WS3, 0);
    delayNanoseconds(HALFCLKns);
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(CLK1, 0);
        digitalWriteFast(CLK2, 0);
        digitalWriteFast(CLK3, 0);
        digitalWriteFast(DATA1, (X >> i) & 1);
        digitalWriteFast(DATA2, (K >> i) & 1);
        digitalWriteFast(DATA3, (G >> i) & 1);
        delayNanoseconds(HALFCLKns);
        digitalWriteFast(CLK1, 1);
        digitalWriteFast(CLK2, 1);
        digitalWriteFast(CLK3, 1);
        delayNanoseconds(HALFCLKns);
    }
    //-- WS=1, LEFT YRB
    digitalWriteFast(WS1, 1);
    digitalWriteFast(WS2, 1);
    digitalWriteFast(WS3, 1);
    delayNanoseconds(HALFCLKns);
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(CLK1, 0);
        digitalWriteFast(CLK2, 0);
        digitalWriteFast(CLK3, 0);
        digitalWriteFast(DATA1, (Y >> i) & 1);
        digitalWriteFast(DATA2, (R >> i) & 1);
        digitalWriteFast(DATA3, (B >> i) & 1);
        delayNanoseconds(HALFCLKns);
        digitalWriteFast(CLK1, 1);
        digitalWriteFast(CLK2, 1);
        digitalWriteFast(CLK3, 1);
        delayNanoseconds(HALFCLKns);
    }
    digitalWriteFast(WS1, 0);
    digitalWriteFast(WS2, 0);
    digitalWriteFast(WS3, 0);
}

void setupDAC() {
    klog_flush("setupDAC");

    pinMode(CLK1, OUTPUT);
    pinMode(DATA1, OUTPUT);
    pinMode(WS1, OUTPUT);
    pinMode(CLK2, OUTPUT);
    pinMode(DATA2, OUTPUT);
    pinMode(WS2, OUTPUT);
    pinMode(CLK3, OUTPUT);
    pinMode(DATA3, OUTPUT);
    pinMode(WS3, OUTPUT);

    digitalWrite(DATA1, 0);
    digitalWrite(CLK1, 0);
    digitalWrite(WS1, 0);
    digitalWrite(DATA2, 0);
    digitalWrite(CLK2, 0);
    digitalWrite(WS2, 0);
    digitalWrite(DATA3, 0);
    digitalWrite(CLK3, 0);
    digitalWrite(WS3, 0);

    laserPoint(0, 0, 0, 0, 0); // midscale XY, zero KRGB
}

void ignoreS(char *foo); // remove errors about unused variables :P
void ignoreI(int bar);

int readILDA2(char *filename) { // Used to debug
    int i;
    int rawinput;
    u_int8_t tmpvar1, tmpvar2;
    char inputchar;
    u_int8_t formatcode;
    char framename[8];
    char companyname[8];
    int16_t framenumber;
    int ildavalid=0;
    char scanhead;
    framename[0] = 0;
    companyname[0] = 0;
    scanhead = 0;

    ignoreS(framename); // remove errors about unused variables :P
    ignoreS(companyname);
    ignoreI(scanhead);

		File fd = SD.open(filename); // Open for read
        if (!fd) {
            klog_flush("File No Open");
                //printf("\nWARNING! Unable to open file %s\n\n",filename);
                //help(myname);
        }
        for (;;) {
                ildavalid=0;
                for (;;) {
                        rawinput=fd.read();
                        if (rawinput==EOF) { // sanity check AURA doesn't
                                return(0);   // end with proper ILDA end frame
                        }
                        inputchar=(char) rawinput;
                        if (inputchar=='I') {
                                ildavalid=1;
                        } else if (inputchar=='L') {
                                if (ildavalid==1) {
                                        ildavalid=2;
                                } else ildavalid=0;
                        } else if (inputchar=='D') {
                                if (ildavalid==2) {
                                        ildavalid=3;
                                } else ildavalid=0;
                        } else if (inputchar=='A') {
                                if (ildavalid==3) {
                                        break;
                                } else ildavalid=0;
                        } else {
                                ildavalid=0;
                        }

                }
                for (i=0;i<3;i++) { // JUNK
                        inputchar=(char)fd.read();
                }
                formatcode=(char)fd.read();
                if (formatcode>2) printf ("Mental Note: This ILDA set has a nonstandard (or new) formatcode: %d\n", formatcode);
                if ((formatcode==0)||(formatcode==1)) {
                        for (i=0;i<8;i++) { // frame name
                                framename[i]=(char)fd.read();
                        }
                        framename[7]='\0';
                        for (i=0;i<8;i++) { // company name
                                companyname[i]=(char)fd.read();
                        }
                        companyname[7]='\0';
                        tmpvar1=fd.read();
                        tmpvar2=fd.read();
                        totalpointstmp=tmpvar1*256+tmpvar2;
                        if (totalpointstmp==0) {
                                fd.close();
                                return(1);
                        }

                        // ILDA Header: Frame Number
                        lastframestanding=framenumber=(u_char)fd.read()*256+(u_char)fd.read();
                        if (firstframe) {
                                firstframe=0; // no longer first frame
                        }
                        totalpoints[framenumber]=totalpointstmp;
                        //klog_flush(String(totalpointstmp));

                        // ILDA Header: Total Frames
                        totalframes=(u_char)fd.read()*256+(u_char)fd.read();
                        if ((setcurrframetoend)&&(!explicitrange)) {
                                currframe=totalframes-1;
                        }

                        // ILDA Header: Scanhead
                        scanhead=fd.read();

                        // ILDA Header: zero byte
                        inputchar=(char)fd.read();
                        if (formatcode==0) {
                                lf[framenumber]=0;
                                fpos[fiter]=liter;
                                fiter++;
                                for (i=0;i<totalpoints[framenumber];i++) {
                                        tmpvar1=fd.read(); // X
                                        tmpvar2=fd.read();
                                        ldata[liter+i*5]=tmpvar1*256+tmpvar2;

                                        tmpvar1=fd.read(); // Y
                                        tmpvar2=fd.read();
                                        ldata[liter+i*5+1]=tmpvar1*256+tmpvar2;

                                        tmpvar1=fd.read(); // Z
                                        tmpvar2=fd.read();
                                        ldata[liter+i*5+2]=tmpvar1*256+tmpvar2;
                                        if (ldata[liter+i*5+2]!=0) rotateimage=1;

                                        /* Coordinate Data: Status Code
                                         *
                                         * Bytes 39-40
                                         * bits 0-7, (lsb) color tbl lookup
                                         * bits 8-13 reserved, set to 0
                                         * bit 14, blanking
                                         * bit 15, (1=last point, 0=all others)
                                         */
                                        stat1[i]=fd.read();
                                        stat2[i]=fd.read();
                                        ldata[liter+i*5+3]=(stat1[i] & 0x40)>>6; // status blanking
                                        ldata[liter+i*5+4]= stat2[i]; // color table

                                        for (zz=0;zz<8;zz++) {
                                                bp1[zz]=(stat1[i] & bm[zz])>>zz;
                                                bp2[zz]=(stat2[i] & bm[zz])>>zz;
                                        }
                                }
                                liter=liter+totalpoints[framenumber]*5;
                        } else if (formatcode==1) {
                                lf[framenumber]=1;
                                fpos[fiter]=liter;
                                fiter++;
                                for (i=0;i<totalpoints[framenumber];i++) {
                                        tmpvar1=fd.read();
                                        tmpvar2=fd.read();
                                        ldata[liter+i*5]=tmpvar1*256+tmpvar2;
                                        tmpvar1=fd.read();
                                        tmpvar2=fd.read();
                                        ldata[liter+i*5+1]=tmpvar1*256+tmpvar2;
                                        ldata[liter+i*5+2]=0;
                                        stat1[i]=fd.read();
                                        stat2[i]=fd.read();
                                        ldata[liter+i*5+3]=(stat1[i] & 0x40)>>6;
                                        ldata[liter+i*5+4]= stat2[i]; // color table
                                        for (zz=0;zz<8;zz++) {
                                                bp1[zz]=(stat1[i] & bm[zz])>>zz;
                                                bp2[zz]=(stat2[i] & bm[zz])>>zz;
                                        }
                                }
                                liter=liter+totalpoints[framenumber]*5;
                        }
                } else {
                }
        }
        fd.close();
        return(1);
}

