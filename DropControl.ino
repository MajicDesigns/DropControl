// Water Droplet Photography Controller
// by MajicDesigns January 2022
// 
// Timer based controller to trigger the productions of a water drop through a solenoid
// valve and then trigger a camera using infrared control to photograph the drop at the 
// end of its drop.
// 
// ** Hardware **
// - Rotary encoder with built-in switch
// - tact Switch
// - Optional colored LED
// - IR LED
// - MOSFET or Transistor solenoid switch
// - Solenoid actuated water valve
// 
// ** External Dependencies **
// hd44780 at https://github.com/duinoWitchery/hd44780 or Arduino IDE Library Manager 
// MD_REncoder at https://github.com/MajicDesigns/MD_REncoder or Arduino IDE Library Manager
// MD_UISwitch at https://github.com/MajicDesigns/MD_UISwitch or Arduino IDE Library Manager
// MD_Menu at https://github.com/MajicDesigns/MD_Menu or Arduino IDE Library Manager
// MD_multiCameraControl at https://github.com/MajicDesigns/MD_MD_multicameraControl
//

#include <EEPROM.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include <MD_REncoder.h>
#include <MD_UISwitch.h>
#include <MD_Menu.h>
#include <MD_multiCameraIrControl.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

#ifndef USE_DEBUG
#define USE_DEBUG 0
#endif

#if USE_DEBUG
#define DEBUGS(s)    do { Serial.print(F(s)); } while (false)
#define DEBUG(s, v)  do { Serial.print(F(s)); Serial.print(v); } while (false)
#define DEBUGX(s, v) do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#else
#define DEBUGS(s)
#define DEBUG(s, v)
#define DEBUGX(s, v)
#endif

// Hardware definitions --------------
// -- SOLENOID 
const uint8_t PIN_SOLENOID = 4;

#define SOLENOID_ON  digitalWrite(PIN_SOLENOID, HIGH)
#define SOLENOID_OFF digitalWrite(PIN_SOLENOID, LOW)

// -- IR TRANSMITTER
const uint8_t PIN_IR = 5;       // Infrared LED

// -- LCD DISPLAY
// Using I2C expander I/O
const uint8_t LCD_ADDR = 0x0;    // I2C address; set to 0 for autodetect
const uint8_t LCD_ROWS = 4;
const uint8_t LCD_COLS = 20;

#define LCD_PRINT(r, c, v) do { lcd.setCursor(c, r); lcd.print(v); } while (false)

// -- USER INPUT AND FEEDBACK
const uint8_t PIN_SWITCH_AUX = 6;  // Additional switch
const uint8_t PIN_LED = 7;         // Status LED
const uint8_t PIN_ENCODER_A = 8;   // Encoder A output
const uint8_t PIN_ENCODER_B = 9;   // Encoder B output
const uint8_t PIN_ENCODER_SW = 10; // Encoder Switch

#define LED_ON  digitalWrite(PIN_LED, HIGH)
#define LED_OFF digitalWrite(PIN_LED, LOW)

// Global enums and data types ------
enum mode_t 
{ 
  M_PURGE, // Purge the lines for set time 
  M_RUN,   // Loop through as many times as defined (min 1) 
};

enum state_t 
{
  S_IDLE,       // doing nothing
  S_INIT,       // initialise run

  S_PURGE_START,// start the purge process
  S_PURGING,    // doing the purge process
  S_PURGE_END,  // end the purge process

  S_SEQ_START,  // set up sequence for looping/waiting time
  S_SETTLE,     // wait for settling time
  S_SOL_INIT,   // initialise the solenoid trigger
  S_SOL_ACTIVE, // activate solenoid
  S_CAMERA,     // wait for and trigger the camera
  S_END,        // end of the process, adjust for next run

  S_ABORT,      // interrupt the current everything
  S_RESET,      // reset all variables at end of run
};

const uint8_t MAX_SEQUENCE = 2;   // number of sequences per profile
const uint8_t MAX_PROFILES = 4;   // number of available profiles

struct sequence_t
{
  bool enabled;         // true when enabled
  uint16_t solenoid;    // solenoid active time in ms (0..999)
  int16_t  deltaSol;    // solenoid active time in ms (-99..+99)
  uint16_t camera;      // camera wait in ms (0..999)
  int16_t  deltaCam;    // adjustment wait time in ms (-99..+99)
};

struct profile_t
{
  uint16_t settle;   // settling time in seconds
  sequence_t  seq[MAX_SEQUENCE];
};

// Global Objects and Variables ------
state_t curState = S_IDLE;
mode_t curMode = M_RUN;
uint8_t curProfile = 0;
uint8_t curSeqIdx = 0;
uint8_t loopIdx = 0, loops = 1;
uint16_t timeSettle = 0;          // for display of settle time countdown
uint32_t curCameraTime, curSolenoidTime;  // timings for current loop in ms
bool refreshDisplay = true;       // flag to refresh the display if needed
profile_t profile[MAX_PROFILES];

hd44780_I2Cexp lcd(LCD_ADDR);
MD_REncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B);
MD_UISwitch_Digital swEnc(PIN_ENCODER_SW);
MD_UISwitch_Digital swAux(PIN_SWITCH_AUX);
Canon camera(PIN_IR);

// Config Data Management ----------
const uint16_t EEPROM_ADDR = 0;   // base address for load/save
const uint8_t SIG0 = 0xe5;        // 2 signature bytes ..
const uint8_t SIG1 = 0x5e;        // .. to detect config is invalid

void configInit(void)
// Initialise configuration defaults
{
  DEBUGS("\nConfig Init");
  for (uint8_t i = 0; i < MAX_PROFILES; i++)
  {
    profile[i].settle = 5;
    for (uint8_t j = 0; j < MAX_SEQUENCE; j++)
    {
      profile[i].seq[j].enabled = (j == 0);
      profile[i].seq[j].solenoid = 30;
      profile[i].seq[j].deltaSol = 0;
      profile[i].seq[j].camera = 20;
      profile[i].seq[j].deltaCam = 0;
    }
  }
}

void configSave(void)
// Save the current config data
{
  DEBUGS("\nConfig Save");
  EEPROM.write(EEPROM_ADDR, SIG0);
  EEPROM.write(EEPROM_ADDR + 1, SIG1);
  EEPROM.put(EEPROM_ADDR + (2 * sizeof(SIG0)), profile);
}

void dumpEEPROM(uint16_t addr)
// Dump a bit of the EEPROM to check config data
// Debugging only
{
#if USE_DEBUG
  const uint8_t STEP_SIZE = 8;

  for (uint8_t i = 0; i < 32; i += STEP_SIZE)
  {
    DEBUG("\n[", i);
    DEBUGS("]:");
    for (uint8_t j = 0; j < STEP_SIZE; j++)
      DEBUGX(" ", EEPROM[addr + i + j]);
  }
#endif
}

bool configLoad(void)
{
  uint8_t s0 = EEPROM.read(EEPROM_ADDR);
  uint8_t s1 = EEPROM.read(EEPROM_ADDR + 1);
  bool b = (s0 == SIG0 && s1 == SIG1);

  dumpEEPROM(EEPROM_ADDR);    /// debugging only

  DEBUGX("\n\nSIG expect [", SIG0); DEBUGX(",", SIG1);
  DEBUGX("] got [", s0); DEBUGX(",", s1); DEBUGS("]");
  DEBUG("\nSIG match = ", (b ? 'T' : 'F'));

  if (b)
  {
    DEBUGS("\nConfig Load");
    EEPROM.get(EEPROM_ADDR + (2 * sizeof(SIG0)), profile);
  }

  return(b);
}

// Display Management --------------
const uint8_t LABEL_SIZE = 2;
const uint8_t DATA_SIZE = 10;

// Data structure and table for static label information
struct label_t
{
  uint8_t r, c;   // lcd rown and column
  char sz[LABEL_SIZE + 1];
};

const label_t PROGMEM dispLabel[] =
{
  { 0, 0, "P:"}, { 0, 6, "T:" }, { 0, 13, "L:"}, {0, 17, "/"},
  { 1, 0, "C:"}, { 1, 3, "-"}, { 1, 7, "-"}, { 1, 13, "S:"},
  { 2, 0, "1:"}, { 2, 5, ":"}, { 2, 14, ":"},
  { 3, 0, "2:"}, { 3, 5, ":"}, { 3, 14, ":"},
};

// Define codes for custom characters
const uint8_t TLC = 0;
const uint8_t TRC = 1;
const uint8_t BLC = 2;
const uint8_t BRC = 3;
const uint8_t LBV = 4;
const uint8_t RBV = 5;
const uint8_t UBH = 6;
const uint8_t LBH = 7;

// Define canned messages for popup display
const char PROGMEM MSG_PURGE[] = "PURGING";
const char PROGMEM MSG_WELCOME[] = "DROP CONTROL";
const char PROGMEM MSG_ABORT[] = "ABORT";
const char PROGMEM MSG_SAVE[] = "SAVE";

void displayInit(uint8_t type)
// Initialise user defined characters in the LCD display
// These are used as the bounds for the popup display
{
  const uint8_t NUM_CHAR = 8;
  const uint8_t SIZE_DATA = 8;
/*
  // Single line shadow box
  static const uint8_t PROGMEM customChar1[NUM_CHAR][SIZE_DATA] =
  {
    { 0x00, 0x07, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 }, // Top left corner (TLC)
    { 0x00, 0x1C, 0x04, 0x07, 0x07, 0x07, 0x07, 0x07 }, // Top right corner (TRC)
    { 0x04, 0x04, 0x04, 0x04, 0x04, 0x07, 0x01, 0x01 }, // Bottom left corner (BLC)
    { 0x07, 0x07, 0x07, 0x07, 0x07, 0x1F, 0x1F, 0x1F }, // Bottom right corner (BRC)
    { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 }, // Left boundary vertical (LBV)
    { 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07 }, // Right boundary vertical (RBV)
    { 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // Upper boundary horizontal (UBH)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F }, // Lower boundary horizontal (LBH)
  };
*/
  // Single line box
  static const uint8_t PROGMEM customChar1[NUM_CHAR][SIZE_DATA] =
  {
    { 0x00, 0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08 }, // Top left corner (TLC)
    { 0x00, 0x1E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 }, // Top right corner (TRC)
    { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0F, 0x00 }, // Bottom left corner (BLC)
    { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x1E, 0x00 }, // Bottom right corner (BRC)
    { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08 }, // Left boundary vertical (LBV)
    { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 }, // Right boundary vertical (RBV)
    { 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // Upper boundary horizontal (UBH)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00 }, // Lower boundary horizontal (LBH)
  };

  // Double Line Box
  static const uint8_t PROGMEM customChar2[NUM_CHAR][SIZE_DATA] =
  {
    { 0x1F, 0x10, 0x17, 0x14, 0x14, 0x14, 0x14, 0x14 }, // Top left corner (TLC)
    { 0x1F, 0x01, 0x1D, 0x05, 0x05, 0x05, 0x05, 0x05 }, // Top right corner (TRC)
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x17, 0x10, 0x1F }, // Bottom left corner (BLC)
    { 0x05, 0x05, 0x05, 0x05, 0x05, 0x1D, 0x01, 0x1F }, // Bottom right corner (BRC)
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }, // Left boundary vertical (LBV)
    { 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05 }, // Right boundary vertical (RBV)
    { 0x1F, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00 }, // Upper boundary horizontal (UBH)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x1F }, // Lower boundary horizontal (LBH)
  };

  char custom[8];

  for (uint8_t i = 0; i < NUM_CHAR; i++)
  {
    memcpy_P(custom, type == 1 ? &customChar1[i] : &customChar2[i], SIZE_DATA);
    lcd.createChar(i, custom);
  }
}

void displayPopup(const char* cp)
{
  const uint8_t row = 0;
  uint8_t len = strlen_P(cp);
  uint8_t offset = (LCD_COLS - len - 2) / 2;

  // create the frame (3 lines of the display)
  lcd.setCursor(offset, row);
  lcd.write(TLC);
  for (uint8_t i = 0; i < len; i++)
    lcd.write(UBH);
  lcd.write(TRC);

  lcd.setCursor(offset, row + 1);
  lcd.write(LBV);
  for (uint8_t i = 0; i < len; i++)
    lcd.write(' ');
  lcd.write(RBV);

  lcd.setCursor(offset, row + 2);
  lcd.write(BLC);
  for (uint8_t i = 0; i < len; i++)
    lcd.write(LBH);
  lcd.write(BRC);

  // pop in the message
  char sz[LCD_COLS];
  strcpy_P(sz, cp);
  lcd.setCursor(offset + 1, row + 1);
  lcd.print(sz);
}

void displayLabels(void)
{
  lcd.clear();
  for (uint8_t i = 0; i < ARRAY_SIZE(dispLabel); i++)
  {
    label_t label;

    memcpy_P(&label, &dispLabel[i], sizeof(label_t));
    LCD_PRINT(label.r, label.c, label.sz);
  }
}

char* num2string(char* sz, int32_t n, uint8_t size, bool leadZero = false, bool sign = false)
{
  uint8_t s;
  char* p = sz + size;
  bool negative = (n < 0);

  if (n < 0) n = -n;

  // clear the string
  memset(sz, (leadZero ? '0' : ' '), size + 1);

  *p-- = '\0';

  // handle the number
  for (s = 0; s < size; s++)
  {
    *p-- = (n % 10) + '0';
    n = n / 10;
    if (n == 0) break;
  }

  // now do the sign indicator
  if (s != size && negative && !sign)
  {
    if (leadZero) p = sz;
    *p = '-';
  }
  else if (sign)
    *sz = (negative ? '-' : '+');

  return(sz);
}

void displayData(profile_t& p)
{
  const bool LZ = false;    // leading zero default

  char s[DATA_SIZE + 1];

  LCD_PRINT(0, 2, num2string(s, curProfile, 1));

  LCD_PRINT(0, 8, num2string(s, timeSettle, 3, LZ));

  LCD_PRINT(0, 15, num2string(s, loopIdx + 1, 2, LZ));
  LCD_PRINT(0, 18, num2string(s, loops, 2, LZ));

  LCD_PRINT(1, 2, num2string(s, curSeqIdx + 1, 1, LZ)); // current sequence
  LCD_PRINT(1, 4, num2string(s, curSolenoidTime, 3, true));   // current solenoid
  LCD_PRINT(1, 8, num2string(s, curCameraTime, 3, true));     // current solenoid
  LCD_PRINT(1, 15, num2string(s, p.settle, 3, LZ));       // settle convert to secs

  for (uint8_t i = 0; i < MAX_SEQUENCE; i++)
  {
    if (i == 0 || (i != 0 && p.seq[i].enabled))
    {
      LCD_PRINT(i + 2, 2, num2string(s, p.seq[i].solenoid, 3, LZ));
      LCD_PRINT(i + 2, 6, num2string(s, p.seq[i].deltaSol, 3, true, true));
      LCD_PRINT(i + 2, 11, num2string(s, p.seq[i].camera, 3, LZ));
      LCD_PRINT(i + 2, 15, num2string(s, p.seq[i].deltaCam, 3, true, true));
    }
    else
    {
      LCD_PRINT(i + 2, 2, F("   "));
      LCD_PRINT(i + 2, 6, F("   "));
      LCD_PRINT(i + 2, 11, F("   "));
      LCD_PRINT(i + 2, 15, F("    "));
    }
  }
}

// Menu Management -----------------
const uint16_t MENU_TIMEOUT = 2500; // in milliseconds
MD_Menu::value_t vBuf;        // single interface buffer for value editing

MD_Menu::userNavAction_t menuNav(uint16_t& incDelta);             // function prototype
bool menuDisplay(MD_Menu::userDisplayAction_t action, char* msg); // function prototype

MD_Menu::value_t* saveProfiles(MD_Menu::mnuId_t id, bool bGet) 
{ 
  //displayPopup(MSG_SAVE);
  configSave(); 
  //delay(1000);
  return(nullptr); 
}

MD_Menu::value_t* mnuValueRqst(MD_Menu::mnuId_t id, bool bGet)
// Value request callback for editing variables
{
  static MD_Menu::value_t* r = &vBuf;
  int8_t index = -1;

  switch (id)
  {
  case 20:  // Settle - convert to seconds on edit display
    if (bGet)
      vBuf.value = profile[curProfile].settle;
    else
    {
      profile[curProfile].settle = vBuf.value;
      DEBUG("\nSettle changed to ", profile[curProfile].settle);
    }
    break;

  case 30:  // Solenoid
    index = 0;
  case 50:
    index = (index == -1 ? 1 : index);
    if (bGet)
      vBuf.value = profile[curProfile].seq[index].solenoid;
    else
    {
      profile[curProfile].seq[index].solenoid = vBuf.value;
      DEBUG("\nSolenoid ", index);
      DEBUG(" changed to ", profile[curProfile].seq[index].solenoid);
    }
    break;

  case 31:  // SolAdj
    index = 0;
  case 51:
    index = (index == -1 ? 1 : index);
    if (bGet)
      vBuf.value = profile[curProfile].seq[index].deltaSol;
    else
    {
      profile[curProfile].seq[index].deltaSol = vBuf.value;
      DEBUG("\ndeltaSol ", index);
      DEBUG(" changed to ", profile[curProfile].seq[index].deltaSol);
    }
    break;

  case 32:  // Camera
    index = 0;
  case 52:
    index = (index == -1 ? 1 : index);
    if (bGet)
      vBuf.value = profile[curProfile].seq[index].camera;
    else
    {
      profile[curProfile].seq[index].camera = vBuf.value;
      DEBUG("\nCamera ", index);
      DEBUG(" changed to ", profile[curProfile].seq[index].camera);
    }
    break;

  case 33:  // CamAdj
    index = 0;
  case 53:
    index = (index == -1 ? 1 : index);
    if (bGet)
      vBuf.value = profile[curProfile].seq[index].deltaCam;
    else
    {
      profile[curProfile].seq[index].deltaCam = vBuf.value;
      DEBUG("\ndeltaCam ", index);
      DEBUG(" changed to ", profile[curProfile].seq[index].deltaCam);
    }
    break;

  case 40:  // S2 enable
    if (bGet)
      vBuf.value = profile[curProfile].seq[1].enabled;
    else
    {
      profile[curProfile].seq[1].enabled = vBuf.value;
      DEBUG("\nS2 enable changed to ", profile[curProfile].seq[1].enabled);
    }
    break;
  }

  return(r);
}

// Menu data structures
const PROGMEM MD_Menu::mnuHeader_t mnuHdr[] =
{
  // Starting Menu
  { 10, "EDIT PROFILE", 10, 20, 0 },
};

const PROGMEM MD_Menu::mnuItem_t mnuItm[] =
{
    // Main (Setup) menu
    { 10, "Settle",      MD_Menu::MNU_INPUT, 20 },

    { 11, "S1 Solenoid", MD_Menu::MNU_INPUT, 30 },
    { 12, "S1 SolAdj",   MD_Menu::MNU_INPUT, 31 },
    { 13, "S1 Camera",   MD_Menu::MNU_INPUT, 32 },
    { 14, "S1 CamAdj",   MD_Menu::MNU_INPUT, 33 },

    { 15, "S2 Enable",   MD_Menu::MNU_INPUT, 40 },

    { 16, "S2 Solenoid", MD_Menu::MNU_INPUT, 50 },
    { 17, "S2 SolAdj",   MD_Menu::MNU_INPUT, 51 },
    { 18, "S2 Camera",   MD_Menu::MNU_INPUT, 52 },
    { 19, "S2 CamAdj",   MD_Menu::MNU_INPUT, 53 },

    { 20, "Save",        MD_Menu::MNU_INPUT, 100 },
};

const PROGMEM MD_Menu::mnuInput_t mnuInp[] =
{
  { 20, "Set (s)", MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 999, 0, 10, nullptr },     // Settle

  { 30, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 999, 0, 10, nullptr },    // S1 Solenoid
  { 31, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, -99, 0, 99, 0, 10, nullptr },   // S1 SolAdj
  { 32, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 999, 0, 10, nullptr },    // S1 Camera
  { 33, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, -99, 0, 99, 0, 10, nullptr },   // S1 CamAdj

  { 40, "Set", MD_Menu::INP_BOOL, mnuValueRqst, 1, 0, 0, 0, 0, 10, nullptr },          // S2 Enable

  { 50, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 999, 0, 10, nullptr },    // S2 Solenoid
  { 51, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, -99, 0, 99, 0, 10, nullptr },   // S2 SolAdj
  { 52, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 999, 0, 10, nullptr },    // S2 Camera
  { 53, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 3, -99, 0, 99, 0, 10, nullptr },   // S2 CamAdj

  { 100, "Confirm", MD_Menu::INP_RUN, saveProfiles, 0, 0, 0, 0, 0, 10, nullptr },      // Save
};

// bring it all together in the global menu object
MD_Menu menu(menuNav, menuDisplay, // user navigation and display
     mnuHdr, ARRAY_SIZE(mnuHdr),   // menu header data
     mnuItm, ARRAY_SIZE(mnuItm),   // menu item data
     mnuInp, ARRAY_SIZE(mnuInp));  // menu input data

// Menu user input and display functions
bool menuDisplay(MD_Menu::userDisplayAction_t action, char* msg)
{
  const uint8_t START_ROW = 2;

  static char szLine[LCD_COLS + 1] = { '\0' };

  switch (action)
  {
  case MD_Menu::DISP_INIT:
    memset(szLine, ' ', LCD_COLS);
    break;

  case MD_Menu::DISP_CLEAR:
    lcd.setCursor(0, START_ROW - 1);
    for (uint8_t i = 0; i < LCD_COLS; i++)
      lcd.write('-');
    LCD_PRINT(START_ROW, 0, szLine);
    LCD_PRINT(START_ROW + 1, 0, szLine);
    break;

  case MD_Menu::DISP_L0:
    LCD_PRINT(START_ROW, 0, szLine);
    LCD_PRINT(START_ROW, 0, msg);
    break;

  case MD_Menu::DISP_L1:
    LCD_PRINT(START_ROW + 1, 0, szLine);
    LCD_PRINT(START_ROW + 1, 0, msg);
    break;
  }

  return(true);
}

MD_Menu::userNavAction_t menuNav(uint16_t& incDelta)
{
  uint8_t re = encoder.read();

  if (re != DIR_NONE)
  {
    incDelta = (menu.isInEdit() ? (1 << (abs(encoder.speed() >> 2))) : 1);
    return(re == DIR_CCW ? MD_Menu::NAV_DEC : MD_Menu::NAV_INC);
  }

  switch (swEnc.read())
  {
  case MD_UISwitch::KEY_PRESS:     return(MD_Menu::NAV_SEL);
  case MD_UISwitch::KEY_LONGPRESS: return(MD_Menu::NAV_ESC);
  default: return(MD_Menu::NAV_NULL);
  }

  return(MD_Menu::NAV_NULL);
}

// Handle User Input ------
void checkUI(void)
{
  // Encoder switch
  switch (swEnc.read())
  {
  case MD_UISwitch::KEY_PRESS:  // start a run sequence
    if (curState == S_IDLE)
    {
      curMode = M_RUN;
      curState = S_INIT;
    }
    break;

  case MD_UISwitch::KEY_LONGPRESS:   // abort current operation or run the menu
    if (curState != S_IDLE)
      // abort takes priority if we are currently running anything
      curState = S_ABORT;
    else
      // force the menu first run
      // won't come back to checkUI() until menu exits
      menu.runMenu(true);
    break;

  default:    // stop the compiler complaining
    break;
  }

  // Function Switch
  switch (swAux.read())
  {
  case MD_UISwitch::KEY_PRESS:      // run a purge process
    if (curState == S_IDLE)
    {
      curMode = M_PURGE;
      curState = S_INIT;
    }
    else if (curState == S_PURGING)
      curState = S_PURGE_END;
    break;

  case MD_UISwitch::KEY_LONGPRESS:  // increment the current profile
    if (curState == S_IDLE)
    {
      curProfile = (curProfile + 1) % MAX_PROFILES;
      refreshDisplay = true;
    }
    break;

  default:    // stop the compiler complaining
    break;
  }

  // Encoder rotation
  switch (encoder.read())
  {
  case DIR_CW:
    if (loops < 99) loops++;
    refreshDisplay = true;
    break;

  case DIR_CCW:
    if (loops > 1) loops--;
    refreshDisplay = true;

  default:    // stop the compiler complaining
    break;
  }
}

// Main FSM ---------------
bool dropTimerFSM(bool reset = false)
// return true when the cycle has finished
{
  static uint32_t timeMark;

  if (reset) curState = S_IDLE;

  switch (curState)
  {
  case S_IDLE:  // do nothing
    break;

  case S_INIT:
    DEBUGS("\n\n>> S_INIT");
    curState = (curMode == M_PURGE ? S_PURGE_START : S_SEQ_START);
    break;

  //=== PURGING sequence
  case S_PURGE_START:
    DEBUGS("\n>> S_PURGE_START");
    displayPopup(MSG_PURGE);
    LED_ON;
    SOLENOID_ON;
    curState = S_PURGING;
    break;

  case S_PURGING:
    // do nothing - state will be changed by a keypress
    break;

  case S_PURGE_END:
    DEBUGS("\n>> PURGE ENDED");
    SOLENOID_OFF;
    LED_OFF;
    displayLabels();
    refreshDisplay = true;
    curState = S_IDLE;
    break;
    
  //=== MAIN sequence - delay (after first loop), drip, wait, trigger camera

  case S_SEQ_START:
    DEBUG("\n>> S_SEQ_START L", loopIdx);
    DEBUG(" S", curSeqIdx);
    if (!profile[curProfile].seq[curSeqIdx].enabled)
    {
      // sequence is not enabled, skip it altogether
      curState = S_END;
    }
    else
    {
      sequence_t* sp = &profile[curProfile].seq[curSeqIdx];

      // adjust timings and keep in bounds (>=0)
      // if the delta is + then no problems
      // if the total delta
      if (sp->deltaSol >= 0 || loopIdx * abs(sp->deltaSol) <= sp->solenoid)
        curSolenoidTime = sp->solenoid + (loopIdx * sp->deltaSol);
      DEBUG(" Sol: ", curSolenoidTime);
      if (sp->deltaCam >= 0 || loopIdx * abs(sp->deltaCam) <= sp->camera)
        curCameraTime = sp->camera + (loopIdx * sp->deltaCam);
      DEBUG(" Cam: ", curCameraTime);

      refreshDisplay = true;
      timeMark = millis();
      timeSettle = 0;
      curState = (loopIdx == 0 || curSeqIdx != 0) ? S_SOL_INIT : S_SETTLE;
      if (curState == S_SETTLE) DEBUGS("\n>> S_SETTLE");
    }
    break;

  case S_SETTLE:
    {
      uint16_t sec = (millis() - timeMark) / 1000;

      LED_ON;

      // check if update display countdown needed
      if (sec != timeSettle)
      {
        timeSettle = sec;
        refreshDisplay = true;
      }

      // check if we have reached the end of timer
      if (sec >= profile[curProfile].settle)
      {
        timeSettle = 0;
        curState = S_SOL_INIT;
        DEBUGS("\n>> SETTLE ENDED");
      }
    }
    break;

  case S_SOL_INIT:
    DEBUGS("\n>> S_SOL_INIT");
    LED_OFF;
    SOLENOID_ON;
    timeMark = millis();
    curState = S_SOL_ACTIVE;
    break;

  case S_SOL_ACTIVE:
    if (millis() - timeMark >= curSolenoidTime)
    {
      DEBUGS("\n>> SOL_ACTIVE ENDED");
      SOLENOID_OFF;
      timeMark = millis();
      curState = (curCameraTime != 0) ? S_CAMERA : S_END;
    }
    break;

  case S_CAMERA:
    if (millis() - timeMark >= curCameraTime)
    {
      DEBUGS("\n>> S_CAMERA");
      camera.shutterNow();
      curState = S_END;
    }
    break;

  case S_END:
    DEBUGS("\n>> S_END");
    curSeqIdx++;
    if (curSeqIdx < MAX_SEQUENCE)
    {
      // go back and do the next sequence in this profile
      curState = S_SEQ_START;
    }
    else
    {
      // all sequences are done
      curSeqIdx = 0;    // reset the sequence number for next iteration
      loopIdx++;          // next iteration index
      curState = (loopIdx >= loops) ? S_RESET : S_SEQ_START;
    }
    break;

  case S_ABORT:   // aborted all operations - reset everything
    DEBUGS("\n>> S_ABORT");
    SOLENOID_OFF;
    LED_OFF;
    displayPopup(MSG_ABORT);
    delay(1000);
    displayLabels();
    curState = S_RESET;
    break;

  case S_RESET:   // all sequences/loops are done
    DEBUGS("\n>> S_RESET");
    loopIdx = 0;
    curSeqIdx = 0;

    curSolenoidTime = 0;
    curCameraTime = 0;
    refreshDisplay = true;
    curState = S_IDLE;
    break;

  default:       // unknown or missed state
    curState = S_IDLE;
    break;
  }

  return(curState == S_IDLE);
}

void setup(void)
{
#if USE_DEBUG
  Serial.begin(115200);
#endif
  DEBUGS("\n[DropControl Debug]");

  // load or create new default profiles
  if (!configLoad())
  {
    configInit();
    configSave();
  }

  // LCD display
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) // non zero status means it was unsuccessful
  {
    DEBUGS("\nLCD failed startup");
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the on board LED if possible
    hd44780::fatalError(status); // does not return
  }

  // Camera
  camera.begin();

  // Solenoid
  pinMode(PIN_SOLENOID, OUTPUT);

  // UI elements
  pinMode(PIN_LED, OUTPUT);
  encoder.begin();
  swEnc.begin();
  swEnc.enableRepeat(false);
  swAux.begin();
  swAux.enableRepeat(false);
  menu.begin();
  menu.setMenuWrap(true);
  menu.setTimeout(MENU_TIMEOUT);
  menuDisplay(MD_Menu::DISP_INIT, nullptr);

  dropTimerFSM(true);

  // pause to show the welcome screen
  displayInit(2);
  displayPopup(MSG_WELCOME);
  delay(2000);

  // Finally, set up the application display
  displayInit(1);
  displayLabels();
}

void loop(void)
{
  static bool wasInMenu = false;

  // update display if flagged
  if (refreshDisplay)
  {
    displayData(profile[curProfile]);
    refreshDisplay = false;
  }

  // run the drop Timer main task
  dropTimerFSM();

  // run menu or UI and detect the transition out of menu
  if (!menu.runMenu())
  {
    if (wasInMenu)
    {
      displayLabels();
      refreshDisplay = true;
    }
    checkUI();
  }
  wasInMenu = menu.isInMenu();
}
