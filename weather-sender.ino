#include <RCSwitch.h>
#include <DHT.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define TX_PIN 2
#define LED_PIN 0

#define DHT_POWER_PIN 3  // power for DHT sensor (to switch it OFF before going to sleep)
#define DHT_PIN 4  // Pin which is connected to the DHT sensor.

#define DHT_TYPE DHT22 // DHT 22 (AM2302)

#ifndef cbi // Clear Bit in I/O Register
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi // Set Bit in I/O Register
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

DHT dht(DHT_PIN, DHT_TYPE);

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms 6=1sec,7=2sec, 8=4sec, 9=8sec
const int sleepTimeConst = 9;  // approximately 8 seconds sleep

byte nextPacketId = 0;   // Stamp packets with their sequence

RCSwitch mySwitch = RCSwitch();

// --------------------------------------------------------------

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
//  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
//    ADMUX = _BV(MUX5) | _BV(MUX0);
//  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
//  #else
//    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

// set system into the sleep state
// system wakes up when watchdog is timed out
void system_sleep() {
  cbi(ADCSRA, ADEN);                    // switch Analog to Digitalconverter OFF
   
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
   
  sleep_mode();                        // System sleeps here
   
  sleep_disable();                     // System continues execution here when watchdog timed out
  sbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter ON
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms 6=1sec,7=2sec, 8=4sec, 9=8sec
void setup_watchdog(int ii) {
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb = ii & 7;
  if (ii > 7) bb |= (1<<5);
  bb |= (1 << WDCE);
  ww = bb;
   
  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  // nothing to do, just waking up
}

void setupDht() {
  dht.begin();
}

void setupRadio() {
  // Transmitter is connected to
  mySwitch.enableTransmit(TX_PIN);
  
  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Optional set pulse length.
  //mySwitch.setPulseLength(320);
  
  // Optional set number of transmission repetitions.
  mySwitch.setRepeatTransmit(15);
}

void blink(int timeout) {
  digitalWrite(LED_PIN, HIGH);  // let led blink
  delay(timeout);
  digitalWrite(LED_PIN, LOW);
}

void sendData() {
  long vccLong = readVcc();
  float vcc = floatMap(float(vccLong),0,6000,0,6);
  
  float t = dht.readTemperature(); // Read temperature as Celsius (the default)
  float h = dht.readHumidity();
  
  // FIXME
//  float t = -22.56;
//  float h = 74.4;
  
  unsigned long result = 0UL;
  if (!isnan(h) && !isnan(t)) {
    float rounding = t > 0 ? 0.05 : -0.05;
    int it = (int)((t + rounding) * 10 ) + 500; // rounding + offset 50C
	  int ih = (int)(h * 10);
	  int ivcc = (int)(vcc * 100);
	  result = (it*1000000UL) + (ih * 1000UL) + ivcc;
  }
  
  mySwitch.send(result, 32);
  delay(200);
  mySwitch.send(nextPacketId++, 16); // 24
  delay(200);
}

void beforeSleep() {
  digitalWrite(DHT_POWER_PIN, LOW);  // switch off DHT
  
  pinMode(DHT_POWER_PIN, INPUT); // set all used port to input to save power
  pinMode(DHT_PIN, INPUT);
  pinMode(TX_PIN, INPUT);
}

void afterWake() {
  pinMode(DHT_POWER_PIN, OUTPUT); // set ports into OUTPUT after wake
//  pinMode(DHT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(DHT_POWER_PIN, HIGH);  // switch on DHT
  
  setupRadio();
  setupDht();
}

// --------------------------------------------------------------

void setup() {
  pinMode(DHT_POWER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(DHT_POWER_PIN, HIGH);  // switch on DHT
  
  setupDht();
  setupRadio();
  setup_watchdog(sleepTimeConst);

  delay(2000); // 2 seconds delay before 1st. data send
}

void loop(void) {
  afterWake();

  blink(5);
  
  sendData();

  blink(5);

  beforeSleep();
      
  for (byte i = 1; i <= 1; i++) // multiple sleeps to reach total sleep time (8s x 112 = cca 15min)
    system_sleep();

}
