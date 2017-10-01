

/************************************************************************/
/*                                                                      */
/*                      RGB LED IR/UART interface                       */
/*                                                                      */
/*                     Initial Author: Martin Christoph                 */
/*                                                                      */
/*                     Reworked/Githubed : Aurelien Labrosse            */
/*                                                                      */
/*                                                                      */
/************************************************************************/


/*  #define SIMULATOR 1 */
 /* #define DEBUG 1   */

/*
  #include <avr/io.h>

  #include <util/delay.h>
  #include <stdlib.h>
  #include <stdbool.h>
  #include <avr/interrupt.h>
  #include <avr/eeprom.h>
*/
#include <avr/sleep.h>

#include <IRremote.h>
#include <IRremoteInt.h>

#define COLOR_PARAM_MAX  (255)

#define RED_PIN    6
#define GREEN_PIN  9
#define BLUE_PIN   10

#define PROGRAM_PIN 14

#define COLOR_PARAM_MAX 255

// force EEPROM overwrite, active low
#define FORCE_EEPROM_ERASE_PIN 5

#define IR_PIN  2  /* PD2, IR input, active low */

#define DEBUG_LED1  7 /* PD7 */

/*
   Small structure for RGB color description
*/
struct rgbColor_t
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

/**
  Gamma correction table in flash memory (Correction for gamma 2.2)


  \see https://www.carnetdumaker.net/articles/utiliser-des-leds-rgb-avec-une-carte-arduino-genuino/

*/
static const uint8_t PROGMEM _gamma[] = {
  0,  21,  28,  34,  39,  43,  46,  50,  53,  56,  59,  61,  64,  66,  68,  70,
  72,  74,  76,  78,  80,  82,  84,  85,  87,  89,  90,  92,  93,  95,  96,  98,
  99, 101, 102, 103, 105, 106, 107, 109, 110, 111, 112, 114, 115, 116, 117, 118,
  119, 120, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135,
  136, 137, 138, 139, 140, 141, 142, 143, 144, 144, 145, 146, 147, 148, 149, 150,
  151, 151, 152, 153, 154, 155, 156, 156, 157, 158, 159, 160, 160, 161, 162, 163,
  164, 164, 165, 166, 167, 167, 168, 169, 170, 170, 171, 172, 173, 173, 174, 175,
  175, 176, 177, 178, 178, 179, 180, 180, 181, 182, 182, 183, 184, 184, 185, 186,
  186, 187, 188, 188, 189, 190, 190, 191, 192, 192, 193, 194, 194, 195, 195, 196,
  197, 197, 198, 199, 199, 200, 200, 201, 202, 202, 203, 203, 204, 205, 205, 206,
  206, 207, 207, 208, 209, 209, 210, 210, 211, 212, 212, 213, 213, 214, 214, 215,
  215, 216, 217, 217, 218, 218, 219, 219, 220, 220, 221, 221, 222, 223, 223, 224,
  224, 225, 225, 226, 226, 227, 227, 228, 228, 229, 229, 230, 230, 231, 231, 232,
  232, 233, 233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238, 239, 239, 240,
  240, 241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 247, 247, 248,
  248, 249, 249, 249, 250, 250, 251, 251, 252, 252, 253, 253, 254, 254, 255, 255
};

/**
   Apply gamma correction to the given sample.

   @param x The input 8 bits sample value.
   @return The output 8 bits sample value with gamma correction.

   \note the gamma correction factor is known at compile time
*/
inline uint8_t BSP_gamma(uint8_t x) {
  return pgm_read_byte(&_gamma[x]);
}

/* Type may be 8, 16 or 32 bits (ie 1, 2 or 4 bytes) */
typedef uint32_t CodeType;

/*!
   Command code from UART of IR
*/
CodeType color_command;

/*!
   Statuses for the code structure
*/
typedef enum {
  STATUS_EMPTY, /* Never be synced (EEPROM empty)      */
  STATUS_DIRTY, /* updated since last read drom EEPROM */
  STATUS_SYNC   /* synchronized with EEPROM            */
} Statuses;



/* Size of the code structure, expressed in bytes */
const uint8_t CODE_SIZE = sizeof(uint8_t) + 20 * sizeof(CodeType) + sizeof(uint16_t) + 2 * sizeof(uint8_t);

/* The code structure */
typedef struct {
  uint8_t status = STATUS_EMPTY;
  CodeType button0        = 0x01;
  CodeType button1        = 0x02;
  CodeType button2        = 0x03;
  CodeType button3        = 0x04;
  CodeType button4        = 0x05;
  CodeType button5        = 0x06;
  CodeType button6        = 0x07;
  CodeType button7        = 0x08;
  CodeType button8        = 0x09;
  CodeType button9        = 0x10;
  CodeType buttonRed      = 0x0a;// 10
  CodeType buttonGreen    = 0x0b;// 11
  CodeType buttonBlue     = 0x0c;// 12
  CodeType buttonYellow   = 0x0d;// 13
  CodeType buttonUp       = 0x1a;// 26
  CodeType buttonDown     = 0x1b;// 27
  CodeType buttonLeft     = 0x2a;// 42
  CodeType buttonRight    = 0x2b;// 43
  CodeType buttonOk       = 0x3a;// 58
  CodeType buttonStandby  = 0x3b;// 59
  CodeType buttonCycle    = 0x3c;// 60

  // Last hue before power-down.
  uint16_t lastHue = 0;

  // Last saturation before power-down.
  uint8_t lastSaturation = COLOR_PARAM_MAX;

  // Last value before power-down.
  uint8_t lastValue = 0;

} CodeMap;
CodeMap CODE_TABLE;

// Not working?
// CodeMap EE_CODE_TABLE EEMEM;

// at start, no color cycle
int COLOR_CYCLE = 0;

/*!
   Save current code values to EEPROM
*/
void SaveCodesToEEProm();

/*!
   Read code values from EEPROM
*/
void ReadCodesFromEEProm();

/*!
   Save current H,S,V values in EEPROM
*/
void SynchronizeHSVValuesInEEPROM();

// angle, saturation, value
void HsvToRgb(uint16_t h, uint8_t s, uint8_t v, rgbColor_t *rgbColor);
CodeType MapIrMessageToCommand(decode_results *results);
void power_down(void);

void parameter_inc8(uint8_t *p_param, uint8_t max, uint8_t delta);
void parameter_dec8(uint8_t *p_param, uint8_t max, uint8_t delta);

void parameter_inc16(uint16_t *p_param, uint8_t max, uint8_t delta);
void parameter_dec16(uint16_t *p_param, uint8_t max, uint8_t delta);

void SetLedColor(rgbColor_t *rgbColor);
void program(void);

/* Set this to true to send the device to bed in next loop() */
volatile bool go_to_sleep = false;

/*! true when the device has just awaken */
volatile bool awaken = true;

// IRemote variables
IRrecv irrecv(IR_PIN);

/* single global structure used to save RAM */
rgbColor_t RGB_COLOR;

void setup(void)
{

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  analogWrite(RED_PIN, 120);
  analogWrite(GREEN_PIN, 120);
  analogWrite(BLUE_PIN, 120);

  pinMode(PROGRAM_PIN, INPUT);
  digitalWrite(PROGRAM_PIN, 1);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  // activate BT module
  pinMode(12, OUTPUT);
  digitalWrite(12, 1);


  pinMode(FORCE_EEPROM_ERASE_PIN, INPUT);
  digitalWrite(FORCE_EEPROM_ERASE_PIN, 1);

  pinMode(DEBUG_LED1, OUTPUT);

  Serial.begin(57600);

Serial.print("EEPROM byte used for saving codes :");Serial.println(CODE_SIZE);

  //ports_enable();

  digitalWrite(DEBUG_LED1, 1);

  irrecv.enableIRIn();  // Start the receiver

  // If PROGRAME_PIN is grounded then activate programming mode.
  if (LOW == digitalRead(PROGRAM_PIN))
  {
    //DEBUG_LED_OFF(DEBUG_LED2);
    program();
  }

  if (digitalRead(FORCE_EEPROM_ERASE_PIN) == 0)
  {
    Serial.println(F("Force EEPROM overwrite with default codes"));
    SaveCodesToEEProm();
  }

  // todo method/macro
  uint8_t readStatus = eeprom_read_byte((void*)(0)) ;
  if (readStatus == STATUS_SYNC)

  {
    ReadCodesFromEEProm();
  }
  else
  {
    Serial.println(F("Code are not programmed yet"));
    Serial.println(F("Writing default values in EEPROM"));

    CODE_TABLE.lastHue = 170;
    CODE_TABLE.lastSaturation = COLOR_PARAM_MAX;
    CODE_TABLE.lastValue = 4;
  }


  Serial.print(F("Code table size : ")); Serial.print(CODE_SIZE); Serial.println(F(" bytes (2Kbytes available)"));

  go_to_sleep = false;

  CODE_TABLE.lastValue = 255;

  Serial.println(F("Ready"));
}

void loop()
{
  decode_results  results;        // Somewhere to store the results

  if (COLOR_CYCLE != 0)
  {
    if (COLOR_CYCLE == 360)
    {
      COLOR_CYCLE = -359;
    }
    if (COLOR_CYCLE == -1)
    {
      COLOR_CYCLE = 1;
    }
    else
    {
      COLOR_CYCLE++;
    }
    HsvToRgb(abs(COLOR_CYCLE), 255, 255, &RGB_COLOR);
    SetLedColor(&RGB_COLOR);
    delay(400);
  }

  if (irrecv.decode(&results)) {  // Grab an IR code
    color_command = MapIrMessageToCommand(&results);
    irrecv.resume();
  }
  if (Serial.available())
  {
    color_command = Serial.parseInt();
    if (0 != color_command)
    {
      //Serial.println(color_command, HEX);
    }
  }
  if (color_command != 0)
  {
    
    Serial.print(F("Received command : 0x")); Serial.println(color_command, HEX);
    
    if (color_command == CODE_TABLE.button0)            CODE_TABLE.lastValue = 1; // min
    else if (color_command == CODE_TABLE.button1)       CODE_TABLE.lastValue = 2;
    else if (color_command == CODE_TABLE.button2)       CODE_TABLE.lastValue = 4;
    else if (color_command == CODE_TABLE.button3)       CODE_TABLE.lastValue = 8; //7;
    else if (color_command == CODE_TABLE.button4)       CODE_TABLE.lastValue = 16; //11;
    else if (color_command == CODE_TABLE.button5)       CODE_TABLE.lastValue = 32; //16;
    else if (color_command == CODE_TABLE.button6)       CODE_TABLE.lastValue = 64; //22;
    else if (color_command == CODE_TABLE.button7)       CODE_TABLE.lastValue = 128; //32;
    else if (color_command == CODE_TABLE.button8)       CODE_TABLE.lastValue = COLOR_PARAM_MAX; // max
    else if (color_command == CODE_TABLE.button9)       CODE_TABLE.lastValue = 0; // off
    else if (color_command == CODE_TABLE.buttonStandby)
    {
      power_down();// stand-by
      awaken = true;
    }
    else if (color_command == CODE_TABLE.buttonRed) // red
    {
      CODE_TABLE.lastHue = 0;
      if (COLOR_CYCLE != 0) COLOR_CYCLE = 1;
      CODE_TABLE.lastSaturation = COLOR_PARAM_MAX;
    }
    else if (color_command == CODE_TABLE.buttonGreen) // green
    {
      CODE_TABLE.lastHue = 120;
      if (COLOR_CYCLE != 0)COLOR_CYCLE = 120;
      CODE_TABLE.lastSaturation = COLOR_PARAM_MAX;
    }
    else if (color_command == CODE_TABLE.buttonBlue) // blue
    {
      CODE_TABLE.lastHue = 240;
      if (COLOR_CYCLE != 0)COLOR_CYCLE = 240;
      CODE_TABLE.lastSaturation = COLOR_PARAM_MAX;
    }
    else if (color_command == CODE_TABLE.buttonYellow) // yellow
    {
      CODE_TABLE.lastHue = 60;
      if (COLOR_CYCLE != 0) COLOR_CYCLE = 60;
      CODE_TABLE.lastSaturation = COLOR_PARAM_MAX;
    }
    else if (color_command == CODE_TABLE.buttonUp)    parameter_inc8(&CODE_TABLE.lastSaturation, COLOR_PARAM_MAX, 2); // sat up
    else if (color_command == CODE_TABLE.buttonDown)  parameter_dec8(&CODE_TABLE.lastSaturation, COLOR_PARAM_MAX, 2); // sat down
    else if (color_command == CODE_TABLE.buttonLeft)  parameter_dec16((uint16_t)&CODE_TABLE.lastHue, 360, 1); // hue up
    else if (color_command == CODE_TABLE.buttonRight) parameter_inc16((uint16_t)&CODE_TABLE.lastHue, 360, 1); // hue down
    else if (color_command == CODE_TABLE.buttonOk) // toggle between max saturation and current value.
    {
      if (CODE_TABLE.lastSaturation > 120)
      {
        CODE_TABLE.lastSaturation = 0;
      }
      else
      {
        CODE_TABLE.lastSaturation = COLOR_PARAM_MAX;
      }
    }
    else if (color_command == CODE_TABLE.buttonCycle)
    {
      if (COLOR_CYCLE == 0)
      {
        COLOR_CYCLE = 1;
      }
      else
      {
        COLOR_CYCLE = 0;
      }
    }
    else
    {
      Serial.print(F("Unknown command : 0x")); Serial.println(color_command, HEX);
      if (true == awaken)
      {
        // just awaken, no valid command : go back to sleep
        power_down();

        // just awaken
        SetLedColor(&RGB_COLOR);
      }
    }
    if (0 != color_command)
    {
      HsvToRgb(CODE_TABLE.lastHue, CODE_TABLE.lastSaturation, CODE_TABLE.lastValue, &RGB_COLOR);
      SetLedColor(&RGB_COLOR);
    }
    color_command = 0; // Clear command
  }
  else
  {
    awaken = false;
  }
}

/*!

   \param[in] hue 0-360, degree
   \param[in] saturation 0-255
   \param[in] value 0-255
   \param[inout] rgbColor RGB structure to update
*/
void HsvToRgb(uint16_t hue, uint8_t saturation, uint8_t value, rgbColor_t *rgbColor)
{

  float H, S, V;
  H = (float)hue;
  S = (float)(saturation / 255.0);
  V = (float)(value / 255.0);

/*
#ifdef DEBUG
  Serial.print(F("H/S/V : "));
  Serial.print(hue); Serial.print("="); Serial.print(H);
  Serial.print(F("/"));
  Serial.print(saturation); Serial.print("="); Serial.print(S);
  Serial.print(F("/"));
  Serial.print(value); Serial.print("="); Serial.print(V);
#endif
*/
  double P, Q, T, fract;

  (H == 360.) ? (H = 0.) : (H /= 60.);
  fract = H - floor(H);

  P = V * (1. - S);
  Q = V * (1. - S * fract);
  T = V * (1. - S * (1. - fract));

  P *= 255;
  Q *= 255;
  T *= 255;
  V *= 255;

  if      (0. <= H && H < 1.)
  {
    rgbColor->red   = V;
    rgbColor->green = T;
    rgbColor->blue  = P;
  }
  else if (1. <= H && H < 2.)
  {
    rgbColor->red   = Q;
    rgbColor->green = V;
    rgbColor->blue  = P;
  }
  else if (2. <= H && H < 3.)
  {
    rgbColor->red   = P;
    rgbColor->green = V;
    rgbColor->blue  = T;
  }
  else if (3. <= H && H < 4.)
  {
    rgbColor->red   = P;
    rgbColor->green = Q;
    rgbColor->blue  = V;
  }
  else if (4. <= H && H < 5.)
  {
    rgbColor->red   = T;
    rgbColor->green = P;
    rgbColor->blue  = V;
  }
  else if (5. <= H && H < 6.)
  {
    rgbColor->red   = V;
    rgbColor->green = P;
    rgbColor->blue  = Q;
  }
  else
  {
    rgbColor->red   = 0;
    rgbColor->green = 0;
    rgbColor->blue  = 0;
  }
/*
#ifdef DEBUG
  Serial.print(F("=> R/G/B : "));
  Serial.print(rgbColor->red );
  Serial.print(F("/"));
  Serial.print(rgbColor->green);
  Serial.print(F("/"));
  Serial.println(rgbColor->blue);
#endif
*/
}


void program(void)
{

  Serial.println(F("Enter program mode"));
   decode_results  results; 
  uint8_t n = 0;
  CodeType newCode = 0;
  CodeType previousCode = 0;
  while (n <= 20)
  {
  
    if (irrecv.decode(&results)) {  // Grab an IR code
      color_command = MapIrMessageToCommand(&results);
      irrecv.resume();
    }
  
    HsvToRgb(85, COLOR_PARAM_MAX, 4, &RGB_COLOR);
    SetLedColor(&RGB_COLOR);
    
    newCode = color_command;
    
    if (newCode != 0 && newCode != previousCode)
    {
      // New code received, store it in EEPROM.

      // momentarily turn off LEDs if code received
      HsvToRgb(85, COLOR_PARAM_MAX, 0, &RGB_COLOR);
      SetLedColor(&RGB_COLOR);

      switch (n)
      {
        case  0: CODE_TABLE.button0 = newCode; break;
        case  1: CODE_TABLE.button1 = newCode; break;
        case  2: CODE_TABLE.button2 = newCode; break;
        case  3: CODE_TABLE.button3 = newCode; break;
        case  4: CODE_TABLE.button4 = newCode; break;
        case  5: CODE_TABLE.button5 = newCode; break;
        case  6: CODE_TABLE.button6 = newCode; break;
        case  7: CODE_TABLE.button7 = newCode; break;
        case  8: CODE_TABLE.button8 = newCode; break;
        case  9: CODE_TABLE.button9 = newCode; break;
        case 10: CODE_TABLE.buttonStandby = newCode; break;
        case 11: CODE_TABLE.buttonRed = newCode; break;
        case 12: CODE_TABLE.buttonGreen = newCode; break;
        case 13: CODE_TABLE.buttonBlue = newCode; break;
        case 14: CODE_TABLE.buttonYellow = newCode; break;
        case 15: CODE_TABLE.buttonUp = newCode; break;
        case 16: CODE_TABLE.buttonDown = newCode; break;
        case 17: CODE_TABLE.buttonLeft = newCode; break;
        case 18: CODE_TABLE.buttonRight = newCode; break;
        case 19: CODE_TABLE.buttonOk = newCode; break;
        case 20: CODE_TABLE.buttonCycle = newCode; break;
      }
      Serial.print("Stored ");Serial.print(newCode, HEX);
      Serial.print(" as hash for command #");Serial.println(n);
      delay(1000);
      n++;
      previousCode = newCode;
      newCode = 0;
    }
  }
  CODE_TABLE.status = STATUS_DIRTY;
  SaveCodesToEEProm();
}
CodeType MapIrMessageToCommand(decode_results *results)
{
  CodeType result = 0;
 
    //dumpRaw(results);

   // Serial.print("Hash : 0x");
   // Serial.println(results->value, HEX);
    result = results->value;
    
   // Serial.print(results->bits, DEC);
   // Serial.println(" bits");
 
    //result = results->value & ~(1 << 11); // Clear toggle bit.
  
    return result;
}

void SetLedColor(rgbColor_t *rgbColor)
{
  /*
#if DEBUG
  Serial.print(F("Set R/G/B : "));
  Serial.print(rgbColor->red);
  Serial.print(F("/"));
  Serial.print(rgbColor->green);
  Serial.print(F("/"));
  Serial.println(rgbColor->blue);
#endif

  //rgbColor->red   = BSP_gamma(rgbColor->red);
  //rgbColor->green = BSP_gamma(rgbColor->green);
  //rgbColor->blue  = BSP_gamma(rgbColor->blue);
#ifdef DEBUG
  Serial.print(F("Set R/G/B (gamma corrected): "));
  Serial.print(rgbColor->red);
  Serial.print(F("/"));
  Serial.print(rgbColor->green);
  Serial.print(F("/"));
  Serial.println(rgbColor->blue);
#endif

  Serial.print("*G") ;
  Serial.print(rgbColor->red);
  Serial.print(F(","));
  Serial.print(rgbColor->green);
  Serial.print(F(","));
  Serial.print(rgbColor->blue);
  Serial.println("*");
*/
#ifdef SIMULATOR
  analogWrite(RED_PIN,   rgbColor->red);
  analogWrite(GREEN_PIN, rgbColor->green);
  analogWrite(BLUE_PIN,  rgbColor->blue);
#else
  analogWrite(RED_PIN,   255 - rgbColor->red);
  analogWrite(GREEN_PIN, 255 - rgbColor->green);
  analogWrite(BLUE_PIN,  255 - rgbColor->blue);
#endif
}
void parameter_inc16(uint16_t *p_param, uint8_t max, uint8_t delta)
{
  int16_t temp = *p_param;
  temp += delta;
  if (temp > max) temp -= max;
  *p_param = (uint16_t)temp;
}


void parameter_dec16(uint16_t *p_param, uint8_t max, uint8_t delta)
{
  int16_t temp = *p_param;
  temp -= delta;
  if (temp < 0) temp += max;
  *p_param = (uint16_t)temp;
}

void parameter_inc8(uint8_t *p_param, uint8_t max, uint8_t delta)
{
  uint8_t temp = *p_param;
  temp += delta;
  if (temp > max) temp -= max;
  *p_param = (uint8_t)temp;
}


void parameter_dec8(uint8_t *p_param, uint8_t max, uint8_t delta)
{
  uint8_t temp = *p_param;
  temp -= delta;
  if (temp < 0) temp += max;
  *p_param = (uint8_t)temp;
}

void ReadCodesFromEEProm()
{
  eeprom_read_block((void*)&CODE_TABLE, 0, CODE_SIZE);
}

void SaveCodesToEEProm()
{
  eeprom_busy_wait();
  CODE_TABLE.status = STATUS_SYNC;
  eeprom_write_block((void*)&CODE_TABLE, 0, CODE_SIZE);
}

void SynchronizeHSVValuesInEEPROM()
{
  // berk
  static const uint16_t hueAdress = sizeof(uint8_t) + (20 * sizeof(CodeType));
  uint16_t saturationAdress       = hueAdress + 2; // hue is uint16_t
  uint16_t valueAddress           = saturationAdress + 1; // saturation is uint8_t

  eeprom_busy_wait();

  eeprom_write_word((void *)hueAdress,        CODE_TABLE.lastHue);
  eeprom_write_word((void *)saturationAdress, CODE_TABLE.lastSaturation);
  eeprom_write_word((void *)valueAddress,     CODE_TABLE.lastValue);

}

void power_down(void)
{
#ifdef DEBUG
  Serial.println("Go to sleep");
  Serial.flush();
#endif
  cli();
  // Prepare wake-up interrupt: a low level on INT0 must wake the MCU up.
  EICRA &= ~(_BV(ISC00) | _BV(ISC01)); // Low-level on INT0 generates interrupt.
  EIMSK |= _BV(INT0); // Enable INT0 interrupt.
  EIFR = 0; // Clear any pending interrupts.
  TIFR2 = 0;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  go_to_sleep = false;
  sleep_enable(); // Prepare for sleep.
  
  analogWrite(RED_PIN,   255);
  analogWrite(GREEN_PIN, 255);
  analogWrite(BLUE_PIN,  255);
  
  sei(); // Arm wake-up interrupt.
  sleep_cpu(); // Sleep...
  sleep_disable();
}


ISR(INT0_vect)
{
  // Wake up.
  sleep_disable();
  EIMSK &= ~_BV(INT0); // Disable INT0 interrupt
  awaken = true;
}

