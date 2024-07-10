#ifndef __GLOBALS_H__
#define __GLOBALS_H__

//#define HALFCLKns 50    // clock cycle duration is twice this 
#define HALFCLKns 30    // clock cycle duration is twice this 

//static unsigned int loopCount = 0;
static int pointDelay = 33; // microseconds                                             
int kicklaser =0;

//struct alignas(1) Point {
#pragma pack(push, 1)
struct Point {
  int16_t x, y;
  uint8_t r, g, b;
};
#pragma pack(pop)

// Definitions for state management
enum class ReceiveState {
  IDLE,
  RECEIVING_DATA
};

int SDPresent = 1;

#define MAX_SPINNERS 4
char spinner[] = {'|', '/', '-', '\\'};
int spinnerstate=0;

size_t Kremainder=0;

int renderTotalPoints;

Point *buffer0;
Point *buffer1;
Point *currentBuffer;
Point *renderBuffer;

int whichBuffer=0;

volatile bool bufferSwap = false;
volatile int pointsPerSecond = 30000; // Default to 30,000 points per second
volatile int configurationValue = 0; // Some configuration setting
volatile int currentBufferIndex = 0; // Index to track where to write new data
volatile int renderBufferIndex = 0;

int32_t fiter=0;
int32_t liter=0;

int currframe=0;
int explicitrange;
int totalpointstmp;
int lastframestanding;
int firstname;
int firstframe;
int totalframes;
int16_t totalpoints[100];
int setcurrframetoend;
int rotateimage;
int rotateinframedelay=0;
int zz;
unsigned short bp1[16];
unsigned short bp2[16];
unsigned short bm[16]={1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,32768};
int pidx=0;
//static int16_t ldata[100000];
//char stat1[1000];
//char stat2[1000];
//static char     lf[1000];  // 0=3D 1=2D
//static int fpos[1000];

// placeholder so compiler doens't bitch about undeclared variables
static int16_t ldata[10];
char stat1[10];
char stat2[10];
static char     lf[10];  // 0=3D 1=2D
static int fpos[10];

#define MAX_POINTS_IN_FRAME 20000
unsigned long lastTime = 0;
unsigned int frameCount = 0;
float fps = 0;

ReceiveState currentState = ReceiveState::IDLE;
int totalPoints = 0;
int receivedPoints = 0;

int bytesReadCtr = 0;

uint8_t currentCommand = 0xFF;  // No command
int totalSize = 0;         // Total size of the data expected for current command

unsigned int remainingBytes = 0;   // Track remaining bytes for the current command

char strbuf[50]; // Ensure the buffer is large enough to hold the entire string
char logfile[32]; // Ensure the buffer is large enough to hold the entire string

#define MAX_LINES 6 // The maximum number of lines the display can show at once
String lines[MAX_LINES] = {""}; // Buffer to store lines of text
int lineCount = 0; // How many lines of text are currently stored

int packetCtr =0;

unsigned long lastUpdate = 0; // Last update time
//const unsigned long updateInterval = 0; // Update every 1000 milliseconds
const unsigned long updateInterval = 250; // Update every 1000 milliseconds

float theta = 0.0;

// teensyID
uint8_t serial[4];
uint8_t mac[6];
uint8_t uid64[8];

#endif