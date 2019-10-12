

#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <SPI.h>
#define kNOERROR 0
#define kPRIMARYREADERROR 1
#define kEXTENDEDREADTIMEOUTERROR 2
#define kPRIMARYWRITEERROR 3
#define kEXTENDEDWRITETIMEOUTERROR 4
#define kCRCERROR 5
#define kUNABLETOCHANGEPROCESSORSTATE 6

//190907const uint16_t LEDPin = 13;
const uint32_t WRITE = 0x40;
const uint32_t READ = 0x00;
const uint32_t COMMAND_MASK = 0xC0;
const uint32_t ADDRESS_MASK = 0x3F;
unsigned long nextTime;
bool ledOn = false;
bool includeCRC = true;

uint16_t angle;
uint16_t temperature;
uint16_t fieldStrength;

float angleDegrees;
float temperatureC;
float fieldStrengthGauss;

const int sensorPin = A1;    // select the input pin for the input voltage
float sensorVoltage = 0;
float outputVoltage = 0;

Adafruit_MCP4725 dac;

//Code below is to read Vin, which will be used later as the "reference voltage" for the DAC
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  //#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1105300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello!");

  uint16_t unused;
  uint32_t flags;
  uint16_t angle;
  uint32_t flagsAndZeroOffset;
  // Initialize SPI
  SPI.begin();
  pinMode(SS, OUTPUT);
  //Sets Unused Pins to Internal Pullup
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(15,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);
  pinMode(17,INPUT_PULLUP);

 //190907 pinMode(LEDPin, OUTPUT);

  nextTime = millis();
  //190907digitalWrite(LEDPin, LOW);
  digitalWrite(SS, HIGH);
  // Make sure all of the SPI pins are
  // ready by doing a read
  PrimaryRead(SS, 0x0, unused);
  // Unlock the device
  ExtendedWrite(SS, 0xFFFE, 0x27811F77);
  // Make sure the device is unlocked
  ExtendedRead(SS, 0x22, flags);
  if (!((flags & 0x0022) == 0x0020))
  {
  Serial.println("Device is not Unlocked");
  }

  

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
 dac.begin(0x62);
    TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz

}

void loop(void) {

  if (PrimaryRead(SS, 0x20, angle) == kNOERROR)
      {
        angleDegrees = (float)(angle & 0x0FFF) * (360.0 / 4096.0);
        if (CalculateParity(angle))
        {
       //   Serial.print("Angle = ");
      //    Serial.print(angleDegrees);
       //   Serial.println(" Degrees");
        }
        else
        {
      //    Serial.println("Parity error on Angle read");
        }
      }
      else
      {
    //    Serial.println("Unable to read Angle");
      }



// This is the lookup table I use to tell the DAC how much voltage to output
// Angle From Sensor on left // Corresponding Desired Output Voltage on right

float array[8] = {
0, 3267,
29, 4785,
287, 265,
360, 3267,

};


if (angleDegrees<= array[0])
{
  outputVoltage = dac.setNearestActualVoltage(array[1], sensorVoltage, false);
 // Serial.println("Your below the bottom end of the table");
}
else if (angleDegrees >= array[6])
{
  outputVoltage = dac.setNearestActualVoltage(sensorVoltage, sensorVoltage, false);
//  Serial.println("Your above the top end of the table");
}

int x;
  for (x= 0; x<8; x=x+2){
  if (angleDegrees >=array[x]
&& angleDegrees <=array[x+2])

        outputVoltage = dac.setNearestActualVoltage((array[x+1] + ( (array[x+3] - array[x+1]) * ( (angleDegrees - array[x]
) / (array[x+2] - array[x]
) ) )),sensorVoltage, false);

}

//Serial.println("The Output Voltage is:  ");
//Serial.println(outputVoltage);


}

uint16_t PrimaryRead(uint16_t cs, uint16_t address, uint16_t& value)
{
uint16_t command;
if (!includeCRC)
{
uint8_t crcValue;
uint8_t crcCommand;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));


// Combine the register address and the command into one byte
command = ((address & ADDRESS_MASK) | READ) << 8;
crcCommand = CalculateCRC(command);

// take the chip select low to select the device
PORTB &= ~_BV(PB2);
// send the device the register you want to read
SPI.transfer16(command);
SPI.transfer(crcCommand);
PORTB |= _BV(PB2);
PORTB &= ~_BV(PB2);
// send the command again to read the contents
value = SPI.transfer16(command);
crcValue = SPI.transfer(crcCommand);
// take the chip select high to de-select
PORTB |= _BV(PB2);
// Restore the 8 bit description

SPI.endTransaction();
// Check the CRC value
if (CalculateCRC(value) != crcValue)
{
//Serial.println("Calculated CRC = ");
//Serial.println(+ CalculateCRC(value));
//Serial.println("crcValue = ");
//Serial.println(crcValue);
return kCRCERROR;
}
}
else
{
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// Combine the register address and the command into one byte
command = ((address & ADDRESS_MASK) | READ) << 8;
// take the chip select low to select the device
PORTB &= ~_BV(PB2);
// send the device the register you want to read
SPI.transfer16(command);
PORTB |= _BV(PB2);
PORTB &= ~_BV(PB2);
// send the command again to read the contents
value = SPI.transfer16(command);
// take the chip select high to de-select
PORTB |= _BV(PB2);
SPI.endTransaction();
}
return kNOERROR;
}
/*
* PrimaryRead
*
* Read from the primary serial registers
*/

void PrimaryRead(uint16_t cs, uint16_t* addresses, uint16_t* values, uint16_t* errors, uint16_t numberOfItems)
{
uint16_t command;
if (includeCRC)
{
uint8_t crcValue;
uint8_t crcCommand;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// to send 4 bits
// Combine the register address and the command into one byte
command = ((addresses[0] & ADDRESS_MASK) | READ) << 8;
crcCommand = CalculateCRC(command);
// take the chip select low to select the device
PORTB &= ~_BV(PB2);
// send the device the register you want to read
SPI.transfer16(command);
SPI.transfer(crcCommand);
PORTB |= _BV(PB2);
for (int index = 0; index < (numberOfItems - 1); ++index)
{
// Combine the register address and the command into one byte
command = ((addresses[index + 1] & ADDRESS_MASK) | READ) << 8;
crcCommand = CalculateCRC(command);
PORTB &= ~_BV(PB2);
// send the command again to read the contents
values[index] = SPI.transfer16(command);
crcValue = SPI.transfer(crcCommand);
// take the chip select high to de-select
PORTB |= _BV(PB2);
// Check the CRC of the read value
if (CalculateCRC(values[index]) != crcValue)
{
errors[index] = kCRCERROR;
}
else
{
errors[index] = kNOERROR;
}
}
PORTB &= ~_BV(PB2);

// send the command again to read the contents
values[numberOfItems - 1] = SPI.transfer16(command);
crcValue = SPI.transfer(crcCommand);

// take the chip select high to de-select
PORTB |= _BV(PB2);
// Check the CRC value
if (CalculateCRC(values[numberOfItems - 1]) != crcValue)
{
errors[numberOfItems - 1] = kCRCERROR;
}
else
{
errors[numberOfItems - 1] = kNOERROR;
}
// Restore the 8 bit description

SPI.endTransaction();
}
else
{
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// Combine the register address and the command into one byte
command = ((addresses[0] & ADDRESS_MASK) | READ) << 8;
// take the chip select low to select the device
PORTB &= ~_BV(PB2);
// send the device the register you want to read
SPI.transfer16(command);
PORTB |= _BV(PB2);
for (int index = 0; index < (numberOfItems - 1); ++index)
{
command = ((addresses[index + 1] & ADDRESS_MASK) | READ) << 8;
PORTB &= ~_BV(PB2);
// send the command again to read the contents
values[index] = SPI.transfer16(command);
// take the chip select high to de-select
PORTB |= _BV(PB2);
errors[index] = kNOERROR;
}
PORTB &= ~_BV(PB2);
// send the command again to read the contents
values[numberOfItems - 1] = SPI.transfer16(command);
errors[numberOfItems - 1] = kNOERROR;
// take the chip select high to de-select
PORTB |= _BV(PB2);
SPI.endTransaction();

}
}
/*
* PrimaryWrite
*
* Write to the primary serial registers
*/
uint16_t PrimaryWrite(uint16_t cs, uint16_t address, uint16_t value)
{
uint16_t command;
if (includeCRC)
{
uint8_t crcCommand;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

// Combine the register address and the command into one byte
command = (((address & ADDRESS_MASK) | WRITE) << 8) | ((value >> 8) & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device:
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send most significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select:
PORTB |= _BV(PB2);
command = ((((address + 1) & ADDRESS_MASK) | WRITE) << 8 ) | (value & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device:
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send least significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select:
PORTB |= _BV(PB2);
// Restore the 8 bit description

SPI.endTransaction();
}
else
{
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// Combine the register address and the command into one byte
command = (((address & ADDRESS_MASK) | WRITE) << 8) | ((value >> 8) & 0x0FF);
// take the chip select low to select the device:
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send most significant byte of register data
// take the chip select high to de-select:
PORTB |= _BV(PB2);
command = ((((address + 1) & ADDRESS_MASK) | WRITE) << 8) | (value & 0x0FF);
// take the chip select low to select the device:
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send least significant byte of register data
// take the chip select high to de-select:
PORTB |= _BV(PB2);
SPI.endTransaction();
}
return kNOERROR;
}
/*
* PrimaryWrite
*
* Write to the primary serial registers
*/
void PrimaryWrite(uint16_t cs, uint16_t* addresses, uint16_t* values, uint16_t* errors, uint16_t numberOfItems)
{
uint16_t command;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
if (includeCRC)
{
uint8_t crcCommand;

for (int index = 0; index < numberOfItems; ++index)
{
// Combine the register address and the command into one byte
command = (((addresses[index] & ADDRESS_MASK) | WRITE) << 8) | ((values[index] >> 8) & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send most significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select
PORTB |= _BV(PB2);
command = ((((addresses[index] + 1) & ADDRESS_MASK) | WRITE) << 8 ) | (values[index] & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send least significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select
PORTB |= _BV(PB2);
errors[index] = kNOERROR;
}

}
else
{
for (int index = 0; index < numberOfItems; ++index)
{
// Combine the register address and the command into one byte
command = (((addresses[index] & ADDRESS_MASK) | WRITE) << 8) | ((values[index] >> 8) & 0x0FF);
// take the chip select low to select the device
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send most significant byte of register data
// take the chip select high to de-select
PORTB |= _BV(PB2);
command = ((((addresses[index] + 1) & ADDRESS_MASK) | WRITE) << 8 ) | (values[index] & 0x0FF);
// take the chip select low to select the device
PORTB &= ~_BV(PB2);
SPI.transfer16(command); // Send least significant byte of register data
// take the chip select high to de-select:
PORTB |= _BV(PB2);
errors[index] = kNOERROR;
}
}
SPI.endTransaction();
}
/*
* ExtendedRead
*
* Read from the SRAM, EEPROM, AUX or Extended Registers
*/
uint16_t ExtendedRead(uint16_t cs, uint16_t address, uint32_t& value)
{
uint16_t results;
uint16_t readFlags;
uint32_t timeout;
uint32_t currentTime;
// Write the address to the Extended Read Address register
results = PrimaryWrite(cs, 0x0A, address & 0xFFFF);
if (results != kNOERROR)
{
return results;
}
// Initiate the extended read
results = PrimaryWrite(cs, 0x0C, 0x8000);
if (results != kNOERROR)
{
return results;
}
timeout = millis() + 100L;
do // Wait for the read to be complete
{
results = PrimaryRead(cs, 0x0C, readFlags);
if (results != kNOERROR)
{
return results;
}
// Make sure the read is not taking too long
currentTime = millis();
if (timeout < currentTime)
{
return kEXTENDEDREADTIMEOUTERROR;
}
} while ((readFlags & 0x0001) != 0x0001);
// Read the 2 data registers
uint16_t addresses[] = {0x0E, 0x10};
uint16_t values[2];
uint16_t errors[2];
PrimaryRead(cs, addresses, values, errors, 2);
// Check all of the results of the reads
for (int index = 0; index < 2; ++index)
{
if (errors[index] != kNOERROR)
{
return errors[index];
}
}
// Combine them
value = ((uint32_t)values[0] << 16) + (uint32_t)values[1];
return results;
}

uint16_t ExtendedWrite(uint16_t cs, uint16_t address, uint32_t value)
{
uint16_t results;
uint16_t writeFlags;
uint32_t timeout;
uint16_t addresses[] = {0x02, 0x04, 0x06, 0x08};
uint16_t values[] = {address, (uint16_t)(value >> 16), (uint16_t)value, 0x8000};
uint16_t errors[4];
// Write into the extended address register
PrimaryWrite(cs, addresses, values, errors, 4);
// Check all of the results of the writes
for (int index = 0; index < 4; ++index)
{
if (errors[index] != kNOERROR)
{
return errors[index];
}
}
// If writing to the EEPROM, generate the Program
// Pulses on Vcc
if ((address >= 0x300) && (address < 0x320))
{
// Send the Program Pulses
// Need hardware which this example does not have.
}
timeout = millis() + 100;
// Wait for the write to complete
do
{
results = PrimaryRead(cs, 0x08, writeFlags);
if (results != kNOERROR)
{
return results;
}
if (timeout < millis())
{
return kEXTENDEDWRITETIMEOUTERROR;
}
} while ((writeFlags & 0x0001) != 0x0001);
return results;
}
/*
* SetProcessorStateToIdle
*
* Change the processor state to idle
* This is needed to write EEPROM, change ORATE or perform certain self tests
*/
uint16_t SetProcessorStateToIdle(uint8_t cs)
{
uint16_t results;
uint16_t value;
// Write the enter idle state command into the control register
results = PrimaryWrite(cs, 0x1F, 0x8046);
if (results == kNOERROR)
{
delay(1);
// Read the status register to see if the processor is in the idle state
results = PrimaryRead(cs, 0x22, value);
if (results == kNOERROR)
{
if ((value & 0x00FF) != 0x0010)
{
return kUNABLETOCHANGEPROCESSORSTATE;
}
}
}
return results;
}
/*
* SetProcessorStateToRun
*
* Change the processor state to run
* This is needed to process angles
*/
uint16_t SetProcessorStateToRun(uint8_t cs)
{
uint16_t results;
uint16_t value;
// Write the enter idle state command into the control register
results = PrimaryWrite(cs, 0x1F, 0xC046);
if (results == kNOERROR)
{
delay(1);
// Read the status register to see if the processor is in the idle state
results = PrimaryRead(cs, 0x22, value);
if (results == kNOERROR)
{
if ((value & 0x00FF) != 0x0011)
{
return kUNABLETOCHANGEPROCESSORSTATE;
}
}
}
return results;
}
/*
* CalculateParity
*
* From the 16 bit input, calculate the parity
*/
bool CalculateParity(uint16_t input)
{
uint16_t count = 0;
// Count up the number of 1s in the input
for (int index = 0; index < 16; ++index)
{
if ((input & 1) == 1)
{
++count;
}
input >>= 1;
}
// return true if there is an odd number of 1s
return (count & 1) != 0;
}
/*
* CalculateCRC
*
* Take the 16bit input and generate a 4bit CRC
* Polynomial = x^4 + x^1 + 1
* LFSR preset to all 1’s
*/
uint8_t CalculateCRC(uint16_t input)
{
bool CRC0 = true;
bool CRC1 = true;
bool CRC2 = true;
bool CRC3 = true;
int i;
bool DoInvert;
uint16_t mask = 0x8000;
for (i = 0; i < 16; ++i)
{
DoInvert = ((input & mask) != 0) ^ CRC3; // XOR required?
CRC3 = CRC2;
CRC2 = CRC1;
CRC1 = CRC0 ^ DoInvert;
CRC0 = DoInvert;
mask >>= 1;
}
return (CRC3 ? 8U : 0U) + (CRC2 ? 4U : 0U) + (CRC1 ? 2U : 0U) + (CRC0 ? 1U : 0U);
}
