// Motor monitor for the LMS 5200 mini-lathe, LMS3990 mini-mill and similar
// tools that use the XMT-DRV-500C(3) / ZM3404 BLDC motor controller PCB.
//
// The monitor board plugs into the BLDC motor hall effect sensor connector,
// which supplies 5VDC power and position pulses. From this, and knowledge
// of the belt drive ratio, we calculate spindle speed and direction.
// We also keep a count of spindle revolutions, which is useful for coil winding.
//
// Rev count is periodically stored in an I2C FRAM so that a power interruption
// (which happens due to fault or safety activation) won't zero the count.
// We store two counts, total lifetime count (absolute) and incremental.
// Incremental count is erased by holding the pushbutton for a few seconds.
// Briefly pressing pushbutton switches between absolute and incremental mode.
//
// We also monitor the motor current using an isolated current sense transformer,
// and display the motor current on the LCD.
//
// Arduino Micro R3 - ATMEGA32U4 (Upload using the "Arduino Leonardo" setting)
// https://www.arduino.cc/en/uploads/Main/arduino-micro-schematic.pdf
// https://www.tonylabs.com/wp-content/uploads/arduino-micro-pinout-diagram.png
//
// Warning: Disconnnect the board from the BLDC motor controller when connecting
// to USB for programming!  Otherwise you risk damage to your motor driver and
// computer.
//
////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 B. Kuschak <bkuschak@yahoo.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// hardware, software, and associated documentation files (the "Product"), to deal in
// the Product without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Product, and
// to permit persons to whom the Product is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice shall be included in all copies
// or substantial portions of the Product.
//
// THE PRODUCT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
// OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE PRODUCT OR THE USE OR OTHER
// DEALINGS IN THE PRODUCT.
//

#include <LiquidCrystal.h>
#include <Wire.h>

#include <avr/boot.h>

/////////////////////////////////////////////////////////////////////////
// These might be different based on your motor and drivetrain:
// Hall effect sensor
#define COUNTS_PER_MOTOR_REV  12            // hall counts per motor rev

// Drive belt ratio
#define DRIVE_RATIO           2.0           // motor revs per spindle rev

// Should be +1 or -1 depending on what constitutes 'forward' and 'reverse'
#define ROTATION_CONVENTION   -1
/////////////////////////////////////////////////////////////////////////

#define VER_STR               "Motor Mon v1.0.2"  // should be 16 characters
#define LCD_UPDATE_TIME_MSEC  850

// 3 Hall effect sensor inputs - on PCINT capable pins
#define PIN_HALL_A            8             // PB4 (board pin 8)
#define PIN_HALL_B            9             // PB5 (board pin 9)
#define PIN_HALL_C            10            // PB6 (board pin 10)

// LCD
#define PIN_LCD_RS            A0            // PF7 (board pin A0)
#define PIN_LCD_EN            A1            // PF6 (board pin A1)
#define PIN_LCD_D4            A2            // PF5 (board pin A2)
#define PIN_LCD_D5            A3            // PF4 (board pin A3)
#define PIN_LCD_D6            4             // PF3 (board pin 
#define PIN_LCD_D7            5             // PF2 (board pin 

// Differential ADC
#define PIN_CURR_SENSE_N      A4            // Analog input (ADC1)
#define PIN_CURR_SENSE_P      A5            // Analog input (ADC0)

#define LED_RED               12
#define LED_GREEN             11
#define BUTTON_1              6             // D6 (board pin 11)
#define BUTTON_2              7             // D7 (board pin 12)

// FRAM to store our recent rev count
#define FRAM_I2C_ADDR         0x54          // FIXME - schematic error on A2, A0
#define FRAM_SIGNATURE        0xACDC1234
#define FRAM_VERSION          1

#define DISPLAY_INCREMENTAL   0
#define DISPLAY_LIFETIME      1

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

volatile int32_t hall_count;
volatile uint16_t incr_abs_hall_count;
volatile uint32_t hall_count_time;
volatile uint32_t errors;
volatile int8_t last_hall_seq;
volatile uint32_t adc_overflows;
volatile uint32_t adc_count;
volatile int16_t adc_val;
volatile int32_t adc_rms;
volatile bool adc_flag;
volatile int16_t adc_sum, adc_filter;
int8_t dir;
uint8_t power_pct;
uint32_t adc_sample_rate;
float rpm;
float motor_curr;
uint8_t rev_display_mode;
int32_t rpm_update_hall_count;
uint32_t rpm_update_time;

// Note, Wire library has a limitation of <32 bytes, so this has to be smaller
struct fram_data_t
{
  uint32_t signature;
  uint32_t version;
  int64_t  lifetime_hall_count;
  int32_t  hall_count;
  uint32_t reserved1;
  uint32_t csum;
} __attribute__((aligned(8), packed)) fram_data;


// Store rev count in FRAM.  FRAM lifetime is essentially unlimited, however power loss
// during read or write can corrupt the value.  So store it in two checksummed blocks.
// At least one should survive.

int store_fram(void)
{
  uint8_t addr;
  uint8_t *ptr8;
  uint32_t *ptr32;
  uint8_t i, j;
  uint32_t csum = 0;
  uint8_t fram_addr = 0;

  fram_data.signature = FRAM_SIGNATURE;
  fram_data.version = FRAM_VERSION;
  fram_data.reserved1 = 0;
  fram_data.hall_count = hall_count;

  // Accumulate recent to lifetime total
  noInterrupts();
  fram_data.lifetime_hall_count += incr_abs_hall_count;
  incr_abs_hall_count = 0;
  interrupts();

  // compute checksum
  fram_data.csum = 0;
  csum = 0;
  ptr32 = (uint32_t*)&fram_data;
  for (i = 0; i < sizeof(fram_data) / sizeof(uint32_t); i++)
    csum += *ptr32++;
  fram_data.csum = ~csum;                       // so all zero doesn't fool us

  // write 2 copies of data to I2C FRAM.  if power fails during write, the other one should
  // remain intact.  FIXME - this assumes power fails quickly.  
  for (j = 0; j < 2; j++) {

    // We use 4Kbit FRAM: it has 8 bit address plus one page select bit
    Wire.beginTransmission(FRAM_I2C_ADDR);      // slave write, page zero
    Wire.write(fram_addr);                      // addr
    ptr8 = (uint8_t*)&fram_data;
    for (i = 0; i < sizeof(fram_data); i++)
      Wire.write(*ptr8++);
    Wire.endTransmission();

    // FIXME Verify write was successful before starting next? (in case power is going down 
    // while we're writing.) Complicated because FRAM read is destructive at the physical level.
    // So a verify may succeed, but actually the data can't be immediately rewritten.

    fram_addr += sizeof(fram_data);
  }
}

// There should be two identical copies of data in the FRAM.  Read and select the first one that
// has a valid checksum.

int load_fram(void)
{
  uint8_t addr;
  uint8_t *ptr8;
  uint8_t i, j;
  uint32_t *ptr32;
  uint32_t csum;
  uint8_t fram_addr = 0;

  for (j = 0; j < 2; j++) {
    ptr8 = (uint8_t*)&fram_data;

    // read I2C FRAM
    // We use 4Kbit FRAM: it has 8 bit address plus one page select bit
    Wire.beginTransmission(FRAM_I2C_ADDR);                  // slave write, page zero
    Wire.write(fram_addr);                                  // addr
    Wire.endTransmission();                                 // Does issue stop
    Wire.requestFrom(FRAM_I2C_ADDR, sizeof(fram_data));
    for (i = 0; i < sizeof(fram_data) && Wire.available(); i++)
      *ptr8++ = Wire.read();

    // validate checksum
    csum = 0;
    ptr32 = (uint32_t*)&fram_data;
    for (i = 0; i < sizeof(fram_data) / sizeof(uint32_t); i++)
      csum += *ptr32++;
    csum += 1;
    if (csum == 0) {
      // Make sure it's one we recognize
      if (fram_data.signature == FRAM_SIGNATURE) {
        switch (fram_data.version) {
          case FRAM_VERSION:
            // this is our native version
            hall_count = fram_data.hall_count;
            rpm_update_hall_count = hall_count;   // to avoid glitch on first RPM calc
            return 0;                             // use it
          // In future releases we might need to do some translation here..
          //case FRAM_VERSION-1:
          // translate as needed...
          default:
            // unrecognized version - ignore it.
            ;
        }
      }
    }
    fram_addr += sizeof(fram_data);
  }

  // No valid checksum found.  store a good one.  If this happens during
  // normal operation, we will lose the lifetime rev count...  Maybe we should store that differently?
  store_fram();
  return -1;
}

// Return hall sensors value as a 3 bit number
uint8_t get_hall_sensors(void)
{
  uint8_t idx;

  // FIXME - do this with port I/O as it is slow using digitalRead()
  idx = digitalRead(PIN_HALL_A) << 2 | digitalRead(PIN_HALL_B) << 1 | digitalRead(PIN_HALL_C);
  idx &= 0x7;
  return idx;
}

// interrupt on change for the hall effect sensor pins
void hall_handler(void)
{
  // hall sensors form a three bit code with 6 valid values.
  // see http://hades.mech.northwestern.edu/index.php/Brushless_DC_Motors
  // table index = hall_sensors[A,B,C]. illegal states are -1
  // two table cycles equals one motor revolution.
  static const int8_t hall_seq_table[] = { -1, 4, 2, 3, 0, 5, 1, -1 };
  int8_t seq;
  int8_t diff;

  // timestamp the pulse for more accurate RPM calculation
  hall_count_time = micros();

  seq = hall_seq_table[get_hall_sensors()];

  if (seq < 0)
    return;

  diff = seq - last_hall_seq;
  last_hall_seq = seq;

  // handle wraparound
  if (diff == -5)
    diff = 1;
  else if (diff == 5)
    diff = -1;

  // Lathe motor rotation sense is opposite (+1 count is reverse, -1 is forward)

  if (diff == (1 * ROTATION_CONVENTION)) {
    hall_count++;
    dir = 1;
    incr_abs_hall_count++;       // will accumulate this to the lifetime total in FRAM
  }
  else if (diff == (-1 * ROTATION_CONVENTION)) {
    hall_count--;
    dir = -1;
    incr_abs_hall_count++;
  }
  else {
    // we missed a count or the sensors are wired incorrectly
    errors++;
  }
}

float hall_count_to_revs(int32_t count)
{
  return (float)count / COUNTS_PER_MOTOR_REV / DRIVE_RATIO;
}

float get_revs()
{
  return hall_count_to_revs(hall_count);
}

// Pin change interrupt config
void pcintSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// Interrupt service routine for pin change interrupt on D8 to D13
ISR (PCINT0_vect)
{
  hall_handler();
}

// Interrupt service routine for the ADC completion.
// This takes some time during calculation, we should verify we're not overruning.
ISR(ADC_vect)
{
  uint8_t decimation = 4;

  // Right-justified 2s complement
  // Must read low first
  adc_val = ADCL;
  adc_val |= (((uint16_t)ADCH) << 8);
  adc_val <<= 6;
  adc_val >>= 6;

  adc_sum += adc_val;

  // Decimate and then run the RMS calculator
  if ((adc_count % decimation) == 0) {
    // HPF to remove any ADC offset
    adc_filter = high_pass_filter(adc_sum);
    adc_filter /= decimation;

    // Run the recursive RMS calculation
    adc_rms = calc_rms(adc_filter);
    adc_sum = 0;
  }
  adc_count++;
}

// ADC ISR that samples AC current and computes RMS power (assuming a certain power factor)
// Interrupt driven ADC adapted from
// http://www.glennsweeney.com/tutorials/interrupt-driven-analog-conversion-with-an-atmega328p
//
// ADC sample rate is
//  differential mode: (Fcpu / 128) / 14 => (16e6 / 128) / 14 = 8929 Hz
//  single endeded mode: (Fcpu / 128) / 14 => (16e6 / 128) / 13 = 9616 Hz
//
// in Gain = 1, ADC max input is +/-2.56 V or 5.12 Vpp or 1.81 Vrms.
// in Gain = 10, ADC max input is +/-0.256 V or 0.512 Vpp or 181 mVrms.
// The actual range will be reduced by the ADC offset error, and limited by ~5V VCC.
//
// The RMS calculation is positive only, so its range is half of the ADC range.
// ADC peak is 512 DN, max RMS measureable without clipping is 361 DN, again limited by ADC offset.
//
// Our current transformer will ouput 0.1Vrms @ 1A and 1Vrms @ 10A (into 100 ohm),
// so use Gain=1 for >10 Arms full scale.
// RMS(A) = DN / 512 / Gain * 2.56 / 0.1
//
// Typically we're using a fraction of the 1X range, but 10X is too small.
// Maybe we can use variable gain?  If the sensed value isn't clipping then switch to 10X mode?
// With the decimation and RMS averaging, it seems to be good enough for now...

void init_adc(void)
{
  // Disable Digital inputs for ADC0 and ADC1
  DIDR0 |= 0x3;

  // Configure ADC for differential mode
  // Full scale +/- VREF/Gain
  ADMUX = 0xC0;             // internal 2.56V Vref
  //ADMUX = 0x40;             // AVCC Vref

  ADMUX |= 0x10;            // MUX[5:0] = 0x10: POS = ADC0, NEG = ADC1, GAIN = 1.
  //ADMUX |= 0x09;            // MUX[5:0] = 0x09: POS = ADC1, NEG = ADC0, GAIN = 10
  //ADMUX |= 0x00;            // MUX[5:0] = 0x00: POS = ADC0, NEG = GND, GAIN = 1
  //ADMUX |= 0x01;            // MUX[5:0] = 0x01: POS = ADC1, NEG = GND, GAIN = 1

  ADCSRA = 0xA7;            // Enable auto, prescaler=128
  ADCSRB = 0x00;            // MUX[5] = 0, free-running mode
  ADCSRA |= 0x08;           // enable interrupts

  adc_flag = false;
  ADCSRA |= 0x40;           // kick off the first conversion
}


// Recursive single pole high pass filter.
// Removes any ADC offset prior to computing the AC(rms) voltage.
//
// Fc = normalized corner frequency from 0 to 0.5 (0 to Fs/2).
// We set Fc = 4 Hz.  Fs = 8928.   Fc(normalized) = 8.96e-4
// x = e^(-2*pi*Fc_norm)
// a0 = (1+x)/2
// a1 = -(1+x)/2
// b1 = x
//
#define HPF_Fc_Hz     4.0
#define ADC_Fs        8928.0
#define HPF_Fc        (HPF_Fc_Hz/(ADC_Fs/2))
#define HPF_X(fc)     (exp(-2*3.14*fc))
#define HPF_A0(x)     ((1.0+x)/2)
#define HPF_A1(x)     (-(1.0+x)/2)
#define HPF_B1(x)     (x)

int32_t high_pass_filter(int16_t x)
{
  float a0 = HPF_A0(HPF_X(HPF_Fc));
  float a1 = HPF_A1(HPF_X(HPF_Fc));
  float b1 = HPF_B1(HPF_X(HPF_Fc));
  float y;
  static float xn_1, yn_1;

  y = a0 * x + a1 * xn_1 + b1 * yn_1;
  xn_1 = x;
  yn_1 = y;
  return y;
}

// Use a recursive low pass filter to calculate the root mean squared.
// FIXME - not enough horsepower to run this floating point calc at full ADC rate.
//
#define RMS_Fc_Hz     2.0
#define RMS_Fc        (RMS_Fc_Hz/(ADC_Fs/2))
#define RMS_X(fc)     (exp(-2*3.14*fc))
#define RMS_A0(x)     (1.0-x)
#define RMS_B1(x)     (x)

int32_t calc_rms(int16_t x)
{
  float b1 = RMS_B1(RMS_X(RMS_Fc));
  float a0 = RMS_A0(RMS_X(RMS_Fc));
  float y;
  float xsq = (float)x * x;
  static float yn_1;

  // LPF the square of the signal.  This is the mean of squares.
  y = a0 * xsq + b1 * yn_1;
  yn_1 = y;

  // return root of mean of squares
  return sqrt(y);
}

// 2x16 LCD
// 0123456789012345
// ----------------
// 1400 RPM FORWARD
// 1.23 A -999999.9
// ----------------
// 1500 RPM REVERSE
// 1.23 A 9999999.9
// ----------------

void update_lcd(void)
{
  char str_revs[12];
  char str_curr[6];
  char str[21];
  static uint32_t last_errors;
  static uint8_t last_mode;

  // Testing
  //motor_curr = 9.99;
  //rpm = 1234;
  //hall_count = 80000000.0 * COUNTS_PER_MOTOR_REV * DRIVE_RATIO;
  // seems like +/-89e6 is the limit (continuous operation for 24 days)

  // First line
  // 1400 RPM FORWARD
  snprintf(str, sizeof(str), "%4u RPM %s",
           abs((int)rpm),
           ((dir > 0) ? "FORWARD" :
            ((dir < 0) ? "REVERSE" : "FORWARD" )),
           str_curr);
  str[16] = 0;
  lcd.setCursor(0, 0);
  lcd.print(str);

  // Second line
  // 3.23 A -999999.9
  // FIXME handle wraparound
  dtostrf(motor_curr, 3, 2, str_curr);    // snprintf doesn't have %f
  if (rev_display_mode == DISPLAY_LIFETIME)
    //snprintf(str_revs, sizeof(str_revs), "%9lu",
    dtostrf(fram_data.lifetime_hall_count / COUNTS_PER_MOTOR_REV / DRIVE_RATIO, 9, 0, str_revs);
  else
    dtostrf(get_revs(), 9, 1, str_revs);    // snprintf doesn't have %f
  str_curr[4] = 0;
  str_revs[9] = 0;
  if (last_mode == rev_display_mode)
    snprintf(str, sizeof(str), "%s A %s", str_curr, str_revs);
  else
    snprintf(str, sizeof(str), "%s A %s", str_curr,
             ((rev_display_mode == DISPLAY_LIFETIME) ? " LIFETIME" :
              ((rev_display_mode == DISPLAY_INCREMENTAL) ? "INCREMENT" : "")));

  // Instead of displaying revs, display some debug info:
  //snprintf(str, sizeof(str), "%6d %6d", adc_sample_rate, adc_val);
  //snprintf(str, sizeof(str), "%6d", adc_rms);

  str[16] = 0;
  lcd.setCursor(0, 1);
  lcd.print(str);


  // if new errors, flash an error message
  if (errors != last_errors) {
    lcd.setCursor(0, 0);
    lcd.print("Error   ");
  }

  last_errors = errors;
  last_mode = rev_display_mode;
}

// Update the RPM calculation every ~500 msec (while motor is running)
// Return 0 if new RPM value available, -1 otherwise.
// If the spindle has moved at all, set count_changed = TRUE.
// RPM > 0 indicates forward
// RPM < 0 indicates reverse
#if 0
int update_rpm(bool *count_changed)
{
  static uint32_t start_time, end_time;           // in microseconds
  static int32_t start_count, end_count;
  uint32_t curr_time; //, difftime;

  // Get delta T and delta count
  curr_time = micros();
  noInterrupts();
  end_count = hall_count;
  end_time = hall_count_time;
  interrupts();

  if (end_count == start_count)
    *count_changed = false;
  else
    *count_changed = true;

  // FIXME - we want to update ~500msec while motor is not running also...
  // if > 500 msec since last hall_count_time, then do it anyway.

  // If > 500 msec since last update, do it now
  //difftime = end_time - start_time;
  //if (difftime >= 500000) {

  // Handle the stall case where hall_count_time has not changed for > 500msec
  // We need an 'last_update_time' which is either based on the hall_count_time or
  // normal time.

  // Else case where motor is running
  if ((end_time - start_time) >= 500000) {
    rpm = hall_count_to_revs(end_count - start_count) / (end_time - start_time) * 1e6 * 60.0;
    start_time = end_time;
    start_count = end_count;
    return 0;
  }
  return -1;
}
#else
void update_rpm(void)
{
  //static uint32_t start_time, end_time;           // in microseconds
  //static int32_t start_count, end_count;
  uint32_t t;
  int32_t count;
  static uint32_t last_t;

  // Get delta T and delta count
  noInterrupts();
  //end_count = hall_count;
  //end_time = hall_count_time;
  count = hall_count;
  t = hall_count_time;
  interrupts();

  // Avoid divide by zero if the motor stopped.
  //if (end_time != start_time) {
  if (t != rpm_update_time) {
    //rpm = hall_count_to_revs(end_count - start_count) / (end_time - start_time) * 1e6 * 60.0;
    rpm = hall_count_to_revs(count - rpm_update_hall_count) / (t - rpm_update_time) * 1e6 * 60.0;
    //start_time = end_time;
    //start_count = end_count;
    rpm_update_time = t;
    rpm_update_hall_count = count;
  }
  // Also handle the case when the motor stops spinning
  else {
    rpm = 0;
  }
}
#endif


// Debounce the button and return:
// PRESSED - recognized immediately
// NOT_PRESSED - recognized after released for > debounce time
// SHORT_PRESS - upon release (after debounce), when held down briefly.
// LONG_PRESS - when held for longer than LONG_PRESS_TIME_MSEC. One shot, then transition to PRESSED state.
//
// SHORT_PRESS state is a one-shot. Next call returns NOT_PRESSED.
// LONG_PRESS state is a one-shot. Next call returns PRESSED.
//
// buttons should be array of size 2.

#define DEBOUNCE_TIME_MSEC        20
#define LONG_PRESS_TIME_MSEC      2500

// States used for debouncing and reporting button status
#define NOT_PRESSED               0
#define PRESSED                   1
#define LONG_PRESS                2
#define SHORT_PRESS               3
#define DEBOUNCE                  4     // internal state only
#define LONG_PRESSED              5     // internal state only
#define LONG_DEBOUNCE             6     // internal state only


void read_buttons(uint8_t *bstate)
{
  uint16_t t;
  uint8_t i, button[2];
  static uint8_t state[2];
  static uint16_t press_time[2];
  static uint16_t debounce_time[2];

  // buttons are active low
  button[0] = ((digitalRead(BUTTON_1)) ? 0 : 1);
  button[1] = ((digitalRead(BUTTON_2)) ? 0 : 1);
  t = millis();

  for (i = 0; i < 2; i++) {
    switch (state[i]) {

      // press recoginzed immediately.  release recognized after delay
      // long/short handled differently
      default:
      case NOT_PRESSED:
        if (button[i]) {
          state[i] = PRESSED;
          press_time[i] = t;
        }
        break;

      case PRESSED:
        if (!button[i]) {
          state[i] = DEBOUNCE;
          debounce_time[i] = t;
        }
        else if ((t - press_time[i]) >= LONG_PRESS_TIME_MSEC) {
          state[i] = LONG_PRESS;
        }
        break;

      case DEBOUNCE:
        // if still not pressed after debounce time, then it was really released
        if (button[i])
          state[i] = PRESSED;
        else if ((t - debounce_time[i]) >= DEBOUNCE_TIME_MSEC)
          state[i] = SHORT_PRESS;
        break;

      case SHORT_PRESS:
        state[i] = NOT_PRESSED;
        break;

      case LONG_PRESS:
        state[i] = LONG_PRESSED;
        break;

      case LONG_PRESSED:
        if (!button[i]) {
          state[i] = LONG_DEBOUNCE;
          debounce_time[i] = t;
        }
        break;

      case LONG_DEBOUNCE:
        if (button[i])
          state[i] = LONG_PRESSED;
        else if ((t - debounce_time[i]) >= DEBOUNCE_TIME_MSEC)
          state[i] = NOT_PRESSED;
        break;
    }

    // Generate outputs.  Some internal states are hidden.
    bstate[i] = state[i];
    if (state[i] == DEBOUNCE || state[i] == LONG_DEBOUNCE || state[i] == LONG_PRESSED)
      bstate[i] = PRESSED;
  }
}

void setup()
{
  char str[17];
  char str_ratio[5];

  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  // LED off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);

  // interrupts on state change of the hall effect sensors
  // do this early so we catch the first pulses from the motor
  pinMode(PIN_HALL_A, INPUT);           // no pull-up so we don't interfere with BLDC controller
  pinMode(PIN_HALL_B, INPUT);
  pinMode(PIN_HALL_C, INPUT);
  noInterrupts();     // disable ints
  pcintSetup(PIN_HALL_A);
  pcintSetup(PIN_HALL_B);
  pcintSetup(PIN_HALL_C);
  last_hall_seq = get_hall_sensors();
  interrupts();       // enable ints
  
  delay(10);

  // 16x2 line display
  lcd.begin(16, 2);
  lcd.clear();

  // Startup message
  lcd.setCursor(0, 0);
  lcd.print(VER_STR);
  dtostrf(DRIVE_RATIO, 5, 2, str_ratio);
  snprintf(str, sizeof(str), "Drv. Ratio %5s", str_ratio);
  lcd.setCursor(0, 1);
  lcd.print(str);
  delay(1000);
  lcd.clear();

#if 0
  // interrupts on state change
  pinMode(PIN_HALL_A, INPUT);           // no pull-up so we don't interfere with BLDC controller
  pinMode(PIN_HALL_B, INPUT);
  pinMode(PIN_HALL_C, INPUT);
  noInterrupts();     // disable ints
  pcintSetup(PIN_HALL_A);
  pcintSetup(PIN_HALL_B);
  pcintSetup(PIN_HALL_C);
  last_hall_seq = get_hall_sensors();
  interrupts();       // enable ints
#endif

  init_adc();

  // Initialize from I2C FRAM
  Wire.begin();
  // TWBR is 72 initially.  Must be > 10
  //TWBR = ((F_CPU / 400000L) - 16) / 2;    // Set I2C frequency to 400kHz  - FIXME causes hang!
  if (load_fram() != 0) {
    lcd.setCursor(0, 0);
    lcd.print("No FRAM data!");
    delay(1000);
    lcd.clear();
  }

#if 0
  {
    // read fuse bits
    // ff d8 cb ef
    uint8_t data1, data2, data3, data4;
    noInterrupts();
    data1 = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);       // 0xff
    data2 = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);      // 0xd8 
    data3 = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);  // 0xcb - should bx 0xf9 or 0xf8 for 3.5V, 4.3V BOD
    data4 = boot_lock_fuse_bits_get(GET_LOCK_BITS);           // 0xef
    interrupts();
    snprintf(str, sizeof(str), "%02hx %02hx %02hx %02hx", data1, data2, data3, data4);
    lcd.setCursor(0, 1);
    lcd.print(str);
    delay(2000);
    lcd.clear();
  }
#endif
}

// the loop function runs over and over again forever
void loop()
{
  static uint8_t led;
  static uint32_t lcd_update_time;
  static int32_t last_hall_count;
  int32_t count;
  float rms_volt;
  uint8_t buttons[2];
  int ret;
  uint16_t tmsec;

  // We want to run this loop fast.  This helps with button debouncing and possibly other things.
  tmsec = millis();

  digitalWrite(LED_BUILTIN, led);
  led = led ^ 1;

  // Handle the button. Short press means switch between incremental and absolute (lifetime).
  // Long press means zero incremental rev count.
  // Maybe also meanings for holding button at boot (and also for long hold > 10 seconds - factory default?)

  read_buttons(buttons);
  if (buttons[0] == LONG_PRESS) {
    // Clear rev counter
    noInterrupts();               // critical section - updated in ISR
    hall_count = 0;
    interrupts();
    lcd_update_time = 0;          // force immediate update
  }
  else if (buttons[0] == SHORT_PRESS) {
    // Toggle between incremental and absolute modes
    rev_display_mode ^= 1;
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  }

  // If the spindle moved, update the FRAM.  FRAM write are essentially limitless, so this should be
  // safe. By writing FRAM often we minimize missed counts if power cuts unexpectedly.
  noInterrupts();
  count = hall_count;
  interrupts();
  if (count != last_hall_count) {
    // FRAM write takes about 3 msec @ 100KHz.
    store_fram();
    last_hall_count = count;
  }

  // Periodically compute the RPM and AC current, and update the LCD
  if ((tmsec - lcd_update_time) >= LCD_UPDATE_TIME_MSEC) {

    update_rpm();

    // Current sense transformer RMS voltage
    // Vrms = ADC_DN / 512 / Gain * 2.56
    rms_volt = (float)adc_rms / 512 * 2.56;

    // Calculate motor current.  Sensor is slightly nonlinear.
    // Irms = coil Vrms * 10.31 at Irms = 1A
    // Irms = coil Vrms * 10.0 at Irms = 10A
    motor_curr = rms_volt * (10.0 + (1.0 - rms_volt) * 0.333);

    update_lcd();
    lcd_update_time = tmsec;

    digitalWrite(LED_RED, !digitalRead(LED_RED));
  }

  // Loop delay
  delay(10);
}

