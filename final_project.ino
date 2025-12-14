// Nicole Lacanlale and Thalia Ysassi
// Group 30
// CPE 301 Final Project

#include <DHT.h>            // humidity and temperature library
#include <LiquidCrystal.h>  // lcd library
#include <Stepper.h>        // stepper library
#include <RTClib.h>         // rtc library

// macros
#define RDA 0x80
#define TBE 0x20
#define DHTPIN 11
#define DHTTYPE DHT11

// four states of the system
enum States {
  DISABLED,  // yellow led
  IDLE,      // green led
  ERROR,     // red led
  RUNNING    // blue led
};

// GPIO registers pointers
unsigned char *ddr_b = (unsigned char *)0x24;
unsigned char *port_b = (unsigned char *)0x25;
unsigned char *pin_b = (unsigned char *)0x23;
unsigned char *ddr_h = (unsigned char *)0x101;
unsigned char *port_h = (unsigned char *)0x102;
unsigned char *pin_h = (unsigned char *)0x100;
unsigned char *ddr_e = (unsigned char *)0x2D;
unsigned char *port_e = (unsigned char *)0x2E;
unsigned char *pin_e = (unsigned char *)0x2C;
unsigned char *ddr_g = (unsigned char *)0x33;
unsigned char *port_g = (unsigned char *)0x34;
unsigned char* ddr_l  = (unsigned char*)0x10A;
unsigned char* port_l = (unsigned char*)0x10B;

// UART registers pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

// ADC registers pointers
volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

// set up lcd
const int RS = 22, EN = 24, D4 = 26, D5 = 28, D6 = 30, D7 = 32;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// set up stepper
const int stepsPerRevolution = 2048;
const int halfTurn = 1024;
Stepper myStepper(stepsPerRevolution, 8, 6, 7, 5);

// thresholds
unsigned int waterThreshold = 150;
unsigned int tempThreshold = 26;

// state tracker
States currState = DISABLED;
States prevState = DISABLED;

// real time clock and temperature/humidity sensor
DHT dht(DHTPIN, DHTTYPE);
RTC_DS1307 rtc;

// for one minute LCD updates
unsigned long lastLCDUpdate = 0;
const unsigned long lcdInterval = 60000;

// for ISR start request
volatile bool startRequest = false; 

void setup() {
  U0init(9600);  // setup the UART
  adc_init();    // setup the ADC

  // configure buttons to input
  *ddr_b &= ~(1 << 6);           // stop button
  *ddr_h &= ~(1 << 6);           // reset button
  *ddr_e &= 0xDF;                // vent control button
  *ddr_e &= ~(1 << 4);           // start button

  // configure fan pin to output
  *ddr_b |= (1 << 7);

  // attach interrupt
  attachInterrupt(digitalPinToInterrupt(2), handleStartRequest, FALLING);

  // enable internal pull up resistors for buttons
  *port_h |=  (1 << 6);
  *port_b |= (1 << 6);
  *port_e |= 0x20;
  *port_e |= (1 << 4); 

  // configure leds to output and enable pull up resistors
  *ddr_l  |=  (1 << 1) | (1 << 3) | (1 << 5) | (1 << 7);
  *port_l &= ~((1 << 1) | (1 << 3) | (1 << 5) | (1 << 7));

  dht.begin();       // set up dht
  rtc.begin();       // set up rtc
  lcd.begin(16, 2);  // set up lcd

  myStepper.setSpeed(10);  // set up stepper speed
}

void loop() {
    unsigned long currentMillis = millis();

  // monitor humidity and temperature and report to LCD screen once per minute, only if state is not disabled
  if (currState != DISABLED) {

    if (currState == ERROR) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Error!");
        lcd.setCursor(0, 1);
        lcd.print("Water Levels Low");
        lcd.clear();
    }

    // get humidity and temperature
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    // make updates every minute
    if (currentMillis - lastLCDUpdate >= lcdInterval) {
        lastLCDUpdate = currentMillis;  // reset the timer

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print(" C");

      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
    }
  
    bool pressedVentControl = !(*pin_e & (1 << 5));  // check for vent control press
    bool pressedStop = !(*pin_b & (1 << 6));         // check for stop press
  
    if (pressedStop) {
        if (!(*pin_b & (1 << 6))) {
            currState = DISABLED;
            printStateMessage(currState);
            return;
        }
    }

    if (pressedVentControl) {
      myStepper.step(halfTurn);   // rotate 180 degrees
    }

    if (currState == IDLE) {
      LEDstate(IDLE);
      fanControl(IDLE);
      unsigned int waterSensorVal = adc_read(0);

      if (waterSensorVal < waterThreshold) {
        currState = ERROR;
        printStateMessage(currState);
        return;
      } else {
        if (temperature > tempThreshold) {
          currState = RUNNING;
          printStateMessage(currState);
          return;
        }
      }
    } else if (currState == ERROR) {
      LEDstate(ERROR);
      fanControl(ERROR);
      printStateMessage(currState);
      bool pressedReset = !(*pin_h & (1 << 6));

      if (pressedReset) {
        unsigned int waterSensorVal = adc_read(0);

        if (waterSensorVal > waterThreshold) {
          currState = IDLE;
          printStateMessage(currState);
          return;
        }
      }
    } else if (currState == RUNNING) {
      LEDstate(RUNNING);
      fanControl(RUNNING);
      printStateMessage(currState);
  
      if (temperature < tempThreshold) {
        currState = IDLE;
        printStateMessage(currState);
        return;
      }

      unsigned int waterSensorVal = adc_read(0);

      if (waterSensorVal < waterThreshold) {
        currState = ERROR;
        printStateMessage(currState);
        return;
      }
    }
    
  } else { // disabled state
    LEDstate(DISABLED);
    fanControl(DISABLED);
    printStateMessage(currState);
  
    if (startRequest) {   // check for start button press
      startRequest = false;
      currState = IDLE;
      printStateMessage(currState);
      return;
    }
  }
}

// handle start button press
void handleStartRequest() {
  startRequest = true; 
}

// adc initialization
void adc_init() {
    *my_ADCSRA |= 0b10000000; 
    *my_ADCSRA &= 0b11011111; 
    *my_ADCSRA &= 0b11110111; 
    *my_ADCSRA &= 0b11111000; 
    *my_ADCSRB &= 0b11110111; 
    *my_ADCSRB &= 0b11111000;
    *my_ADMUX &= 0b01111111; 
    *my_ADMUX |= 0b01000000;
    *my_ADMUX &= 0b11011111; 
    *my_ADMUX &= 0b11100000; 
}

// read adc
unsigned int adc_read(unsigned char adc_channel_num) {
    *my_ADMUX &= 0b11100000; 
    *my_ADCSRB &= 0b11110111;
    if (adc_channel_num > 7) {
        adc_channel_num -= 8;
        *my_ADCSRB |= 0b00001000;
    }
    *my_ADMUX += adc_channel_num;
    *my_ADCSRA |= 0x40; 
    while ((*my_ADCSRA & 0x40) != 0); 
    return *my_ADC_DATA;
}

// uart functions
void U0init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

unsigned char U0kbhit() {
  return *myUCSR0A & RDA;
}

unsigned char U0getchar() {
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata) {
  while ((*myUCSR0A & TBE) == 0)
    ;
  *myUDR0 = U0pdata;
}

// light led depending on state
void LEDstate(States s) {
  *port_l &= ~((1 << 1) | (1 << 3) | (1 << 5) | (1 << 7));

  switch (s) {
    case IDLE:
      *port_l |= (1 << 1);  // green led
      break;

    case DISABLED:
      *port_l |= (1 << 3);  // yellow led
      break;

    case ERROR:
      *port_l |= (1 << 5);  // red led
      break;

    case RUNNING:
      *port_l |= (1 << 7);  // blue led
      break;
  }
}

// control fan depending on state
void fanControl(States s) {
    static bool fanState = false; // tracks current fan state

    switch (s) {
        case RUNNING:
            *port_b |= (1 << 7); // turn fan on
            if (!fanState) {     // print if fan just turned on
                fanState = true;
                printFanActivity("Fan ON");
            }
            break;

        case IDLE:
        case ERROR:
        case DISABLED:
        default:
            *port_b &= ~(1 << 7); // turn fan off
            if (fanState) {       // print if fan just turned off
                fanState = false;
                printFanActivity("Fan OFF");
            }
            break;
    }
}

// prints fan activity
void printFanActivity(const char* action) {
    DateTime now = rtc.now();      // get current time from rtc
    char buffer[32];

    snprintf(buffer, sizeof(buffer), "[%02d:%02d:%02d] ", now.hour(), now.minute(), now.second());

    // print timestamp
    U0sendString((unsigned char*)buffer);
    U0sendString((unsigned char*)action);
    U0sendString((unsigned char*)"\r\n");
}

void printStateMessage(States s) {
    static States prevState = IDLE;  // track last update

    if (s != prevState) {
        DateTime now = rtc.now(); // get current time from rtc
        char timestamp[16];
  
        snprintf(timestamp, sizeof(timestamp), "[%02d:%02d:%02d] ", now.hour(), now.minute(), now.second());

        // send timestamp to uart
        U0sendString((unsigned char*)timestamp);

        // print message depending on current state
        switch (s) {
            case ERROR:
                U0sendString((unsigned char*)"System ERROR: Water levels low!\r\n");
                break;
            case IDLE:
                U0sendString((unsigned char*)"System IDLING\r\n");
                break;
            case RUNNING:
                U0sendString((unsigned char*)"System RUNNING\r\n");
                break;
            case DISABLED:
                U0sendString((unsigned char*)"System DISABLED\r\n");
                break;
        }

        prevState = s;  // update previous state
    }
}

// helper function for printing to serial monitor
void U0sendString(const unsigned char* msg) {
    for (int i = 0; msg[i] != '\0'; i++) {
        U0putchar(msg[i]);
    }
}