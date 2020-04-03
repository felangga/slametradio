
#include <SI4735.h>

#include "Rotary.h"
#include <math.h>
#include <SPI.h>

#include <TFT_eSPI.h>                 // Include the graphics library (this includes the sprite functions)

#define FF18 &FreeSans12pt7b


// Test it with patch_init.h or patch_full.h. Do not try load both.
//#include "patch_init.h" // SSB patch for whole SSBRX initialization string
#include "patch_full.h"    // SSB patch for whole SSBRX full download

const uint16_t size_content = sizeof ssb_patch_content; // see ssb_patch_content in patch_full.h or patch_init.h

TFT_eSPI    tft = TFT_eSPI();

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

// OLED Diaplay constants
#define I2C_ADDRESS 0x3C
#define RST_PIN -1 // Define proper RST_PIN if required.

#define RESET_PIN 12

// Enconder PINs
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 4
#define ENCODER_SWITCH 36

// Buttons controllers
#define MODE_SWITCH 4      // Switch MODE (Am/LSB/USB)
#define BANDWIDTH_BUTTON 5 // Used to select the banddwith. Values: 1.2, 2.2, 3.0, 4.0, 0.5, 1.0 KHz
#define VOL_UP 6           // Volume Up
#define VOL_DOWN 7         // Volume Down
#define BAND_BUTTON_UP 8   // Next band
#define BAND_BUTTON_DOWN 9 // Previous band
#define AGC_SWITCH 11      // Switch AGC ON/OF
#define STEP_SWITCH 10     // Used to select the increment or decrement frequency step (1, 5 or 10 KHz)
#define BFO_SWITCH 13      // Used to select the enconder control (BFO or VFO)

#define MIN_ELAPSED_TIME 100
#define MIN_ELAPSED_RSSI_TIME 150

#define DEFAULT_VOLUME 50 // change it for your favorite sound volume

#define FM 0
#define LSB 1
#define USB 2
#define AM 3
#define LW 4

#define SSB 1

const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM "};
uint8_t currentMode = FM;

bool bfoOn = false;
bool disableAgc = true;
bool ssbLoaded = false;
bool fmStereo = true;

int currentBFO = 0;
int previousBFO = 0;
int step = 10;
int screenWidth = 320;
int screenHeight = 240;
int idxBand = 1;

long elapsedRSSI = millis();
long elapsedButton = millis();
long elapsedFrequency = millis();

// Encoder control variables
volatile int encoderCount = 0;

// Some variables to check the SI4735 status
uint16_t currentFrequency;
uint16_t previousFrequency;
uint8_t currentStep = 1;
uint8_t currentBFOStep = 25;

uint8_t bwIdxSSB = 2;
const char *bandwitdthSSB[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};

uint8_t bwIdxAM = 1;
const char *bandwitdthAM[] = {"6", "4", "3", "2", "1", "1.8", "2.5"};
unsigned int buttonstate;
/*
   Band data structure
*/
typedef struct
{
  uint8_t bandType;     // Band type (FM, MW or SW)
  uint16_t minimumFreq; // Minimum frequency of the band
  uint16_t maximumFreq; // maximum frequency of the band
  uint16_t currentFreq; // Default frequency or current frequency
  uint16_t currentStep; // Defeult step (increment and decrement)
} Band;

/*
   Band table
*/
Band band[] = {
  {FM_BAND_TYPE, 8400, 10800, 10390, 10},
  {LW_BAND_TYPE, 100, 510, 300, 1},
  {MW_BAND_TYPE, 520, 1720, 810, 10},
  {SW_BAND_TYPE, 1800, 3500, 1900, 1}, // 160 meters
  {SW_BAND_TYPE, 3500, 4500, 3700, 1}, // 80 meters
  {SW_BAND_TYPE, 4500, 5500, 4850, 5},
  {SW_BAND_TYPE, 5600, 6300, 6000, 5},
  {SW_BAND_TYPE, 6800, 7800, 7200, 5}, // 40 meters
  {SW_BAND_TYPE, 9200, 10000, 9600, 5},
  {SW_BAND_TYPE, 10000, 11000, 10100, 1}, // 30 meters
  {SW_BAND_TYPE, 11200, 12500, 11940, 5},
  {SW_BAND_TYPE, 13400, 13900, 13600, 5},
  {SW_BAND_TYPE, 14000, 14500, 14200, 1}, // 20 meters
  {SW_BAND_TYPE, 15000, 15900, 15300, 5},
  {SW_BAND_TYPE, 17200, 17900, 17600, 5},
  {SW_BAND_TYPE, 18000, 18300, 18100, 1},  // 17 meters
  {SW_BAND_TYPE, 21000, 21900, 21200, 1},  // 15 mters
  {SW_BAND_TYPE, 24890, 26200, 24940, 1},  // 12 meters
  {SW_BAND_TYPE, 26200, 27900, 27500, 1},  // CB band (11 meters)
  {SW_BAND_TYPE, 28000, 30000, 28400, 1}
}; // 10 meters

const int lastBand = (sizeof band / sizeof(Band)) - 1;
int bandIdx = 0;

uint8_t rssi = 0;
uint8_t snr = 0;
uint8_t stereo = 1;
uint8_t volume = DEFAULT_VOLUME;

unsigned long waktu;
unsigned long waktuTerakhir;

int tempTitik = 0;
double currentBottomFreq = 8800;
double currentTopFreq = 10800;
double currentPembagi = 100.0;
String currentSatuan = "MHz";
int currentDecimal = 2;
double geser = 0.0;
double geserSebelum = geser;
int frekuensi[13];
boolean fast = false;

// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);
SI4735 si4735;

// Use Rotary.h and  Rotary.cpp implementation to process encoder via interrupt
void IRAM_ATTR rotaryEncoder()
{ // rotary encoder events

  uint8_t encoderStatus = encoder.process();

  if (encoderStatus)
  {
    if (encoderStatus == DIR_CW)
    {
      encoderCount = 1;

    }
    else
    {
      encoderCount = -1;

    }
  }
}


void setup()
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(0x0000);


  tft.setTextColor(0xFFFF, 0x0000);
  tft.setTextWrap(false);

  Serial.begin(9600);
  // Look for the Si47XX I2C bus address

  int16_t si4735Addr = 0 ;
  while (si4735Addr == 0) {
    si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);
    if ( si4735Addr == 0 ) {
      tft.println("Radio chip not found!");
      Serial.println("Si473X not found!");

    } else {
      Serial.print("The Si473X I2C address is 0x");
      Serial.println(si4735Addr, HEX);
    }
    delay(500);
  }

  tft.fillScreen(0x0000);

  si4735.setup(RESET_PIN, FM_BAND_TYPE);

  //  // Encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_SWITCH, INPUT_PULLUP);

  attachInterrupt(ENCODER_PIN_A, rotaryEncoder, CHANGE);
  attachInterrupt(ENCODER_PIN_B, rotaryEncoder, CHANGE);

  gantiBand(1);
  delay(200);


  currentFrequency = si4735.getFrequency();
  geserSebelum = ((currentTopFreq - currentFrequency) / currentPembagi) * 30;

  si4735.setFrequency(currentFrequency);
  si4735.setVolume(volume);

  drawFreq();
  drawDial(currentBottomFreq, currentTopFreq, 100);
}

void setAM(int startFreq, int endFreq, int currentFreq, int step) {
  if (startFreq < 1700)
    si4735.setTuneFrequencyAntennaCapacitor(0);
  else
    si4735.setTuneFrequencyAntennaCapacitor(1);

  if (ssbLoaded)
  {
    si4735.setSSB(startFreq, endFreq, currentFreq, step, currentMode);
    si4735.setSSBAutomaticVolumeControl(1);
    si4735.setSsbSoftMuteMaxAttenuation(0); // Disable Soft Mute for SSB
  }
  else
  {
    currentMode = AM;
    si4735.setAM(startFreq, endFreq, currentFreq, step);
    si4735.setAutomaticGainControl(1, 0);
    si4735.setAmSoftMuteMaxAttenuation(0); // // Disable Soft Mute for AM
    bfoOn = false;
  }
}

void gantiBand(int band) {
  switch (band) {
    // FM
    case 1 :
      currentBottomFreq = 8800;
      currentTopFreq = 10800;
      currentPembagi = 100.0;
      currentFrequency = currentBottomFreq;
      currentSatuan  = "MHz";
      currentDecimal = 2;
      currentMode = FM;
      si4735.setTuneFrequencyAntennaCapacitor(0);
      si4735.setFM(currentBottomFreq, currentTopFreq, 9800, 10);
      bfoOn = ssbLoaded = false;
      break;
    // LW
    case 2 :
      currentBottomFreq = 100;
      currentTopFreq = 510;
      currentPembagi = 1;
      currentSatuan  = "KHz";
      currentDecimal = 0;
      currentFrequency = 300;
      setAM(currentBottomFreq, currentTopFreq, currentFrequency, 1);
      break;
    // MW
    case 3 :
      currentBottomFreq = 520;
      currentTopFreq = 1720;
      currentPembagi = 1;
      currentDecimal = 0;
      currentSatuan  = "KHz";
      currentFrequency = currentBottomFreq;
      setAM(currentBottomFreq, currentTopFreq, currentFrequency, 10);
      break;
    // SW
    case 4 :
      currentBottomFreq = 1800;
      currentTopFreq = 28000;
      currentPembagi = 1000.0;
      currentDecimal = 3;
      currentSatuan  = "KHz";
      currentFrequency = currentBottomFreq;
      setAM(currentBottomFreq, currentTopFreq, currentFrequency, 1);
      break;
  }

  geserSebelum = ((currentTopFreq - currentFrequency) / currentPembagi) * 30;

}

// Show current frequency
//void showFrequency()
//{
//  String freqDisplay;
//  String unit;
//  String bandMode;
//  int divider = 1;
//  int decimals = 3;
//  if (band[bandIdx].bandType == FM_BAND_TYPE)
//  {
//    divider = 100;
//    decimals = 1;
//    unit = "MHz";
//  }
//  else if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE)
//  {
//    divider = 1;
//    decimals = 0;
//    unit = "KHz";
//  }
//  else
//  {
//    divider = 1000;
//    decimals = 3;
//    unit = "KHz";
//  }
//
//  if ( !bfoOn )
//    freqDisplay = String((float)currentFrequency / divider, decimals);
//  else
//    freqDisplay = ">" + String((float)currentFrequency / divider, decimals) + "<";
//
//  //  display.setCursor(7, 0);
//  //  display.print("        ");
//  //  display.setCursor(7, 0);
//  //  display.print(freqDisplay);
//
//  if (currentFrequency < 520 )
//    bandMode = "LW  ";
//  else
//    bandMode = bandModeDesc[currentMode];
//  //
//  //  display.setCursor(0, 0);
//  //  display.print(bandMode);
//  //
//  //  display.setCursor(17, 0);
//  //  display.print(unit);
//}

/*
    Show some basic information on display
*/
//void showStatus()
//{
//
//  showFrequency();
//
//  //  display.setCursor(13, 1);
//  //  display.print("      ");
//  //  display.setCursor(13, 1);
//  //  display.print("St: ");
//  //  display.print(currentStep);
//  //
//  //  display.setCursor(0, 3);
//  //  display.print("           ");
//  ////  display.setCursor(0, 3);
//  //
//  //  if (currentMode == LSB || currentMode == USB)
//  //  {
//  //    display.print("BW:");
//  //    display.print(String(bandwitdthSSB[bwIdxSSB]));
//  //    display.print("KHz");
//  //    showBFO();
//  //  }
//  //  else if (currentMode == AM)
//  //  {
//  //    display.print("BW:");
//  //    display.print(String(bandwitdthAM[bwIdxAM]));
//  //    display.print("KHz");
//  //  }
//  //
//  //  // Show AGC Information
//  //  si4735.getAutomaticGainControl();
//  //  display.setCursor(0, 1);
//  //  display.print((si4735.isAgcEnabled()) ? "AGC ON " : "AGC OFF");
//
//  showRSSI();
//  showVolume();
//}

/* *******************************
   Shows RSSI status
*/
void showRSSI()
{

  // draw signal bar
  tft.fillRect(((rssi * 8)), 0, screenWidth, 15, 0x0000);
  tft.fillRect(0, 0, rssi * 8 , 15, 0x0E00);

  tft.fillRect(((snr * 4)), 15, screenWidth, 15, 0x0000);
  tft.fillRect(0, 15, snr * 8 , 15, 0xAA00);

  tft.setCursor(2, 4);
  tft.setTextColor(0xFFFF);
  tft.setTextWrap(false);
  tft.setTextSize(1);

  tft.print("SGNL");

  tft.setCursor(2, 19);
  tft.print("SNR");

}

/*
   Shows the volume level on LCD
*/
//void showVolume()
//{
//  //  display.setCursor(10, 3);
//  //  display.print("  ");
//  //  display.setCursor(10, 3);
//  //  display.print(si4735.getCurrentVolume());
//}

/*
   Shows the BFO current status.
   Must be called only on SSB mode (LSB or USB)
  //*/
//void showBFO()
//{
//
//  String bfo;
//
//  if (currentBFO > 0)
//    bfo = "+" + String(currentBFO);
//  else
//    bfo = String(currentBFO);
//
//  //  display.setCursor(0, 2);
//  //  display.print("         ");
//  //  display.setCursor(0, 2);
//  //  display.print("BFO:");
//  //  display.print(bfo);
//  //  display.print("Hz ");
//  //
//  //  display.setCursor(13, 2);
//  //  display.print("       ");
//  //  display.setCursor(13, 2);
//  //  display.print("St: ");
//  //  display.print(currentBFOStep);
//}

/*
   Goes to the next band (see Band table)
*/
//void bandUp()
//{
//  // save the current frequency for the band
//  band[bandIdx].currentFreq = currentFrequency;
//  band[bandIdx].currentStep = currentStep;
//
//  if (bandIdx < lastBand)
//  {
//    bandIdx++;
//  }
//  else
//  {
//    bandIdx = 0;
//  }
//  useBand();
//}
//
///*
//   Goes to the previous band (see Band table)
//*/
//void bandDown()
//{
//  // save the current frequency for the band
//  band[bandIdx].currentFreq = currentFrequency;
//  band[bandIdx].currentStep = currentStep;
//  if (bandIdx > 0)
//  {
//    bandIdx--;
//  }
//  else
//  {
//    bandIdx = lastBand;
//  }
//  useBand();
//}

/*
   This function loads the contents of the ssb_patch_content array into the CI (Si4735) and starts the radio on
   SSB mode.
*/
//void loadSSB()
//{
//
//
//  si4735.reset();
//  si4735.queryLibraryId(); // Is it really necessary here?  Just powerDown() maigh work!
//  si4735.patchPowerUp();
//  delay(50);
//  // si4735.setI2CFastMode(); // Recommended
//  si4735.setI2CFastModeCustom(500000); // It is a test and may crash.
//  si4735.downloadPatch(ssb_patch_content, size_content);
//  si4735.setI2CStandardMode(); // goes back to default (100KHz)
//
//
//  // delay(50);
//  // Parameters
//  // AUDIOBW - SSB Audio bandwidth; 0 = 1.2KHz (default); 1=2.2KHz; 2=3KHz; 3=4KHz; 4=500Hz; 5=1KHz;
//  // SBCUTFLT SSB - side band cutoff filter for band passand low pass filter ( 0 or 1)
//  // AVC_DIVIDER  - set 0 for SSB mode; set 3 for SYNC mode.
//  // AVCEN - SSB Automatic Volume Control (AVC) enable; 0=disable; 1=enable (default).
//  // SMUTESEL - SSB Soft-mute Based on RSSI or SNR (0 or 1).
//  // DSP_AFCDIS - DSP AFC Disable or enable; 0=SYNC MODE, AFC enable; 1=SSB MODE, AFC disable.
//  si4735.setSSBConfig(bwIdxSSB, 1, 0, 0, 0, 1);
//  delay(25);
//  ssbLoaded = true;
//
//}

/*
   Switch the radio to current band.
   The bandIdx variable points to the current band.
   This function change to the band referenced by bandIdx (see table band).
*/
void useBand()
{

  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    currentMode = FM;
    si4735.setTuneFrequencyAntennaCapacitor(0);
    si4735.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
    bfoOn = ssbLoaded = false;

  }
  else
  {
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE)
      si4735.setTuneFrequencyAntennaCapacitor(0);
    else
      si4735.setTuneFrequencyAntennaCapacitor(1);

    if (ssbLoaded)
    {
      si4735.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep, currentMode);
      si4735.setSSBAutomaticVolumeControl(1);
      si4735.setSsbSoftMuteMaxAttenuation(0); // Disable Soft Mute for SSB
    }
    else
    {
      currentMode = AM;
      si4735.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
      si4735.setAutomaticGainControl(1, 0);
      si4735.setAmSoftMuteMaxAttenuation(0); // // Disable Soft Mute for AM
      bfoOn = false;
    }

  }
  delay(100);
  currentFrequency = band[bandIdx].currentFreq;
  currentStep = band[bandIdx].currentStep;
  //  showStatus();
}

void drawFreq() {


  if (previousFrequency != currentFrequency) {
    char freq[8];
    String convert = " " + String(currentFrequency / currentPembagi, currentDecimal);
    convert.toCharArray(freq, 8);

    tft.setTextSize(3);
    tft.setTextColor(0xFFFF);
    tft.drawRightString(freq, screenWidth, 4, 1);
    previousFrequency = currentFrequency;
  }

}


void drawDial(double startFreq, double endFreq, double pembagi) {

  geser = ((endFreq - currentFrequency) / pembagi) * 30.0;
  Serial.println(geser);

  int r = (screenWidth / 2) + 30;
  int x = screenWidth / 2;
  int y = r + 130;

  tempTitik = 0;

  char freq[8];
  String convert = String(currentFrequency / currentPembagi, currentDecimal);
  convert.toCharArray(freq, 8);

  for (int isi = ((int) floor(geser / 360) * 12); isi < ((int) floor(geser / 360) * 12) + 12; isi ++) {
    if ((int) geser == 0) {
      if (isi >= 9) continue;
    }
    frekuensi[isi % 12] = endFreq - (isi * pembagi);
    if ((int) geser % 360 >= 300) {
      Serial.println("LEBIH 360");
      frekuensi[0] = endFreq - ((((int) floor(geser / 360) * 12) + 12 ) * pembagi) ;
      frekuensi[1] = endFreq - ((((int) floor(geser / 360) * 12) + 13 ) * pembagi) ;
      frekuensi[2] = endFreq - ((((int) floor(geser / 360) * 12) + 14 ) * pembagi) ;
    } else if ((int) geser % 360 <= 60 && geser > 0) {
      Serial.println("KURANG 60");
      frekuensi[11] = endFreq - ((((int) floor(geser / 360) * 12) - 1 ) * pembagi) ;
      frekuensi[10] = endFreq - ((((int) floor(geser / 360) * 12) - 2 ) * pembagi) ;
      frekuensi[9] =  endFreq - ((((int) floor(geser / 360) * 12) - 3 ) * pembagi) ;
    }

  }


  tft.fillRect(0, screenHeight - 30, 100, screenHeight, 0x0000);
  tft.fillRect(screenWidth - 100, screenHeight - 30, screenWidth, screenHeight, 0x0000);

  Serial.print("GESER : "); Serial.println(geser);
  Serial.print("GESER SEBELUM : "); Serial.println(geserSebelum);

  for (int i = 0; i < 360; i += 1)
  {
    int x1 = r * cos(((i) - 90) * PI / 180);
    int y1 = r * sin(((i) - 90) * PI / 180);
    tft.fillCircle(x + x1, y + y1, 2, 0x0000);
  }

  for (int i = 0; i < 360; i += 1)
  {
    int x1 = r * cos(((i - geser) - 90) * PI / 180);
    int y1 = r * sin(((i - geser) - 90) * PI / 180);

    if (i % 3 == 0) {
      tft.drawPixel(x + x1, y + y1, TFT_BLUE);
    }

    if (i % 30 == 0) {


      convert = String(frekuensi[(i / 30)] / currentPembagi, currentDecimal);
      convert.toCharArray(freq, 8);


      int x2 = r * cos(((i - geserSebelum) - 90) * PI / 180);
      int y2 = r * sin(((i - geserSebelum) - 90) * PI / 180);


      tft.fillCircle(x + x1, y + y1, 1, TFT_CYAN);

      if (x + x1 < -10) continue;
      if (x + x1 > screenWidth) continue;
      if (y + y1 > screenHeight) continue;
      if (frekuensi[(i / 30)] <= endFreq && frekuensi[(i / 30)] >= startFreq) {

        tft.setTextSize(2);

        tft.setTextColor(0x0000, 0x0000);
        tft.drawCentreString(freq, x + x2, y + y2 + 15, 1);
        tft.setTextColor(0xFFFF, 0x0000);
        tft.drawCentreString(freq, x + x1, y + y1 + 15, 1);
        tft.setTextSize(1);

        tft.drawCentreString(currentSatuan, screenWidth / 2, y - 20, 4);
        tft.drawLine(x, screenHeight, x, y - r - 10, 0xF000);
      }
    }
  }
}


void loop()
{

  if (millis() - waktuTerakhir > 700) fast = false;

  // Show fast & slow tuning text
  if (fast) {
    tft.setTextSize(1);
    tft.setTextColor(0xFFFF, 0x0000);
    tft.drawRightString("FAST", screenWidth - 3, 33, 1);
  } else {
    tft.setTextSize(1);
    tft.setTextColor(0xFFFF, 0x0000);
    tft.drawRightString("SLOW", screenWidth - 3 , 33, 1);
  }

  buttonstate = digitalRead(ENCODER_SWITCH);
  if (buttonstate == LOW) {
    idxBand++;
    if (idxBand > 4) idxBand = 1; // back to FM
    gantiBand(idxBand);
    tft.fillRect(0, 0, screenWidth, screenHeight, 0x0000);
    drawDial(currentBottomFreq, currentTopFreq, 100);
    while (digitalRead(ENCODER_SWITCH) == 0);
  }


  if (encoderCount != 0)
  {
    if (bfoOn)
    {
      currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
    }
    else
    {

      if (millis() - waktuTerakhir < 80) {
        fast = true;
      }

      if (fast) {
        step = 10;

      } else {
        step = 1;

      }

      if (encoderCount == -1) {
        if (currentFrequency + step <= currentTopFreq) currentFrequency += step; else currentFrequency = currentTopFreq;
      } else {
        if (currentFrequency - step >= currentBottomFreq)  currentFrequency -= step; else currentFrequency = currentBottomFreq;
      }

      si4735.setFrequency(currentFrequency);
      geserSebelum = geser;

      drawDial(currentBottomFreq, currentTopFreq, 100);
      showRSSI();

    }

    waktuTerakhir = millis();
    encoderCount = 0;
  }


  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 5)
  {
    si4735.getCurrentReceivedSignalQuality();
    int aux = si4735.getCurrentRSSI();
    snr = si4735.getCurrentSNR();

    if (rssi != aux)
    {
      rssi = aux;
      previousFrequency = 0;
      showRSSI();
    }
    elapsedRSSI = millis();
  }

  drawFreq();

  delay(10);
}
