// Pin configuration - STM32F103C8T6
/*
  SPI default pins:
  MOSI  - A7 // DIN
  MISO  - A6 // DOUT
  SCK	  - A5 // SCLK
  SS	  -	A4 // CS
  --------------------
  MOSI: Master OUT Slave IN -> DIN
  MISO: Master IN Slave OUT -> DOUT
  --------------------
  Other pins
  RST	  -	A3
  DRDY  - A2 // this is an interrupt pin
  PDWN/SYNC  - +3.3 V or A1
*/
//--------------------------------------------------------------------------------
//Clock rate
/*
	f_CLKIN = 7.68 MHz
	tau = 130.2 ns
*/
//--------------------------------------------------------------------------------

#include <SPI.h> //SPI comunication

void setup()
{
  Serial.begin(115200);
  delay(1000);
  initialize_ADS1256();
  delay(1000);
  reset_ADS1256();
  userDefaultRegisters();
  //printInstructions();
  attachInterrupt(digitalPinToInterrupt(A2), checkDReady, FALLING); //FALLING - DRDY goes low
}
//--------------------------------------------------------------------------------
//Variables

//Booleans
volatile boolean dataReady; //It is very important to have it defined as volatile!
double VREF = 2.50; //VREF for converting the raw data to human-readable voltage

//Pins
const byte CS_pin = A4;	//goes to CS on ADS1256
const byte DRDY_pin = A2;  //goes to DRDY on ADS1256
const byte RESET_pin = A3; //goes to RST on ADS1256
//const byte PDWN_PIN = A1; //Goes to the PDWN pin - Alternatively can be permanently tied to 3.3 V

//Values for registers
uint8_t registerAddress; //address of the register, both for reading and writing - selects the register
uint8_t registerValueR; //this is used to READ a register
uint8_t registerValueW; //this is used to WRITE a register
int32_t registerData; //this is used to store the data read from the register (for the AD-conversion)
uint8_t directCommand; //this is used to store the direct command for sending a command to the ADS1256
String PrintMessage; //this is used to concatenate stuff into before printing it out.

byte outputBuffer[3]; //3-byte (24-bit) buffer for the fast acquisition - Single-channel, continuous
byte differentialBuffer[12]; //4x3-byte buffer for the fast differential-channel acquisition -
byte singleBuffer[24]; //8x3-byte buffer for the fast single-ended-channel acquisition
//float StartTime; //This only serves test purposes

//--------------------------------------------------------------------------------
void loop()
{
  if (Serial.available() > 0)
  {
    char commandCharacter = Serial.read(); //we use characters (letters) for controlling the switch-case
    switch (commandCharacter) //based on the command character, we decide what to do
    {
      case 'r': //this case is used to READ the value of a register
        while (!Serial.available()); //wait for the serial
        registerAddress = Serial.parseInt(); //parse the address of the register
        /*
          //Wait for the input
          while (!Serial.available());
          Text before the print
          PrintMessage = "*Value of register " + String(registerAddress) + " is " + String(readRegister(registerAddress));
          Serial.println(PrintMessage);
          PrintMessage = ""; //resetting value
        */
        break;

      case 'w': //this case is used to WRITE the value of a register
        while (!Serial.available()); //wait for the serial
        registerAddress = Serial.parseInt(); //Store the register in registerAddress
        delay(100);
        while (!Serial.available());
        registerValueW = Serial.parseInt(); //Store the value to be written
        delay(100);
        writeRegister(registerAddress, registerValueW);
        delay(500);
        break;

      case 't': //this case is used to print a message to the serial terminal. Just to test the connection...etc.
        Serial.println("*Test message triggered by serial command");
        break;

      case 'O': //this case is used to read a single value from the AD converter
        readSingle();
        break;

      case 'R': //this does a RESET on the ADS1256
        reset_ADS1256();
        break;

      case 's': //SDATAC - Stop Reading Data Continously
        SPI.transfer(B00001111);
        break;

      case 'A': //Single channel continous reading - MUX is manual, can be single and differential too
        readSingleContinuous();
        break;

      case 'C': //Single-ended mode cycling
        cycleSingleEnded();
        break;

      case 'D': //differential mode cycling
        cycleDifferential();
        break;

      case 'd': //direct command
        while (!Serial.available());
        directCommand = Serial.parseInt();
        sendDirectCommand(directCommand);
        break;

      case 'U'://Set everything back to default
        userDefaultRegisters();
        break;
    }
  }
}
//--------------------------------------------------------------------------------
//Functions

void checkDReady() //Function for the ISR
{
  dataReady = true;
}

unsigned long readRegister(uint8_t registerAddress) //Function for READING a selected register
{
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  //SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.

  digitalWrite(CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]

  SPI.transfer(0x10 | registerAddress); //0x10 = 0001000 = RREG - OR together the two numbers (command + address)

  SPI.transfer(0x00); //2nd (empty) command byte

  delayMicroseconds(5); //see t6 in the datasheet

  registerValueR = SPI.transfer(0xFF); //read out the register value

  digitalWrite(CS_pin, HIGH);
  SPI.endTransaction();

  return registerValueR;
}

void writeRegister(uint8_t registerAddress, uint8_t registerValueW)
{
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  //SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.

  digitalWrite(CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]

  delayMicroseconds(5); //see t6 in the datasheet

  SPI.transfer(0x50 | registerAddress); // 0x50 = 01010000 = WREG

  SPI.transfer(0x00);

  SPI.transfer(registerValueW);

  digitalWrite(CS_pin, HIGH);
  SPI.endTransaction();
}

void reset_ADS1256()
{
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1

  digitalWrite(CS_pin, LOW);

  delayMicroseconds(7);

  SPI.transfer(0xFE); //Reset command

  delay(2); //Minimum 0.6ms required for Reset to finish.

  SPI.transfer(0x0F); //Issue SDATAC (any 8-bit value would complete the RESET command)

  delayMicroseconds(100);

  digitalWrite(CS_pin, HIGH);

  SPI.endTransaction();
}

void initialize_ADS1256()	//starting up the chip by making the necessary steps. This goes into the setup() later.
{
  //Chip select
  pinMode(CS_pin, OUTPUT); //Chip select is an output
  digitalWrite(CS_pin, LOW);

  //DRDY
  pinMode(DRDY_pin, INPUT);
  //Reset
  pinMode(RESET_pin, OUTPUT);
  //We do a manual chip reset on the ADS1256 - Datasheet Page 27/ RESET
  digitalWrite(RESET_pin, LOW);
  delay(100);
  digitalWrite(RESET_pin, HIGH); //RESET is set to high
  delay(500);

  SPI.begin();
}

void readSingle() //Reading a single value ONCE using the RDATA command
{
  registerData = 0; // every time we call this function, this should be 0 in the beginning!
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"

  while (dataReady == false) {} //Wait for DRDY to go LOW
  SPI.transfer(B00000001); //Issue RDATA (0000 0001) command
  delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

  //step out the data: MSB | mid-byte | LSB,
  registerData = SPI.transfer(0x0F); //MSB comes in, first 8 bit is updated
  registerData <<= 8;					//MSB gets shifted LEFT by 8 bits
  registerData |= SPI.transfer(0x0F); //MSB | Mid-byte // '|=' compound bitwise OR operator
  registerData <<= 8;					//MSB | Mid-byte gets shifted LEFT by 8 bits
  registerData |= SPI.transfer(0x0F); //(MSB | Mid-byte) | LSB - final result
  //After this, DRDY should go HIGH automatically

  Serial.print("Raw data: ");
  Serial.println(registerData); //prints the raw data (24-bit number)
  Serial.print("Voltage: ");
  convertToVoltage(registerData); //prints the converted data (PGA should be 0!)

  digitalWrite(CS_pin, HIGH); //We finished the command sequence, so we switch it back to HIGH
  SPI.endTransaction();
}

void readSingleContinuous() //Reads the recently selected channel using RDATAC
{
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"

  //These variables serve only testing purposes!
  //uint32_t loopcounter = 0;
  //StartTime = micros();
  //--------------------------------------------

  while (dataReady == false) {} //Wait until DRDY does low, then issue the command
  SPI.transfer(B00000011);  //Issue RDATAC (0000 0011) command after DRDY goes low
  delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

  while (Serial.read() != 's')
  {
    //while (GPIOA->regs->IDR & 0x0004){} //direct port access to A2 (DRDY) pin - less reliable polling alternative
    while (dataReady == false) {} //waiting for the dataReady ISR
    //Reading a single input continuously using the RDATAC
    //step out the data: MSB | mid-byte | LSB
    outputBuffer[0] = SPI.transfer(0); // MSB comes in
    outputBuffer[1] = SPI.transfer(0); // Mid-byte
    outputBuffer[2] = SPI.transfer(0); // LSB - final conversion result
    //After this, DRDY should go HIGH automatically
    Serial.write(outputBuffer, sizeof(outputBuffer)); //this buffer is [3]
    dataReady = false; //reset dataReady manually

    /*
      //These variables only serve test purposes!
      loopcounter++;
      //if(micros() - StartTime >= 5000000) //5 s
      if(loopcounter >= 150000)
      {
             Serial.print(" Loops: ");
             Serial.println(loopcounter++);
             Serial.println(micros() - StartTime);
             break; //exit the whole thing
      }
    */
  }
  SPI.transfer(B00001111); //SDATAC stops the RDATAC - the received 's' just breaks the while(), this stops the acquisition
  digitalWrite(CS_pin, HIGH); //We finished the command sequence, so we switch it back to HIGH
  SPI.endTransaction();
}

void cycleSingleEnded()
{
  int cycle = 0;
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]
  while (Serial.read() != 's')
  {
    for (cycle = 0; cycle < 8; cycle++)
    {
      //we cycle through all the 8 single-ended channels with the RDATAC
      //INFO:
      //RDATAC = B00000011
      //SYNC = B11111100
      //WAKEUP = B11111111
      //---------------------------------------------------------------------------------------------
      /*Some comments regarding the cycling:
        When we start the ADS1256, the preconfiguration already sets the MUX to [AIN0+AINCOM].
        When we start the RDATAC (this function), the default MUX ([AIN0+AINCOM]) will be included in the
        cycling which means that the first readout will be the [AIN0+AINCOM]. But, before we read the data
        from the [AIN0+AINCOM], we have to switch to the next register already, then start RDATA. This is
        demonstrated in Figure 19 on Page 21.

        Therefore, in order to get the 8 channels nicely read and formatted, we have to start the cycle
        with the 2nd input of the ADS1256 ([AIN1+AINCOM]) and finish with the first ([AIN0+AINCOM]).

         \ CH1 | CH2 CH3 CH4 CH5 CH6 CH7 CH8 \ CH1 | CH2 CH3 ...

        The switch-case is between the  two '|' characters
        The output (one line of values) is between the two '\' characters.
      */
      //-------------------------------------------------------------------------------------------
      //Steps are on Page 21 of the datasheet
      while (dataReady == false) {} //direct port access to A2 (DRDY) pin
      //Step 1. - Updating MUX
      switch (cycle)
      {
        //Channels are written manually
        case 0: //Channel 2
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B00011000);  //AIN1+AINCOM
          break;

        case 1: //Channel 3
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B00101000);  //AIN2+AINCOM
          break;

        case 2: //Channel 4
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B00111000);  //AIN3+AINCOM
          break;

        case 3: //Channel 5
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B01001000);  //AIN4+AINCOM
          break;

        case 4: //Channel 6
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B01011000);  //AIN5+AINCOM
          break;

        case 5: //Channel 7
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B01101000);  //AIN6+AINCOM
          break;

        case 6: //Channel 8
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B01111000);  //AIN7+AINCOM
          break;

        case 7: //Channel 1
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B00001000); //AIN0+AINCOM
          break;
      }
      //Step 2.
      SPI.transfer(B11111100); //SYNC
      delayMicroseconds(4); //t11 delay 24*tau = 3.125 us //delay should be larger, so we delay by 4 us
      SPI.transfer(B11111111); //WAKEUP

      //Step 3.
      //Issue RDATA (0000 0001) command
      SPI.transfer(B00000001);
      delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

      //step out the data: MSB | mid-byte | LSB
      singleBuffer[(3 * cycle)] = SPI.transfer(0x0F); //MSB comes in, first 8 bit is updated
      singleBuffer[(3 * cycle) + 1] = SPI.transfer(0x0F); //Mid-byte
      singleBuffer[(3 * cycle) + 2] = SPI.transfer(0x0F); //LSB - final result
      dataReady = false;
      //After this, DRDY should go HIGH automatically
    }
    //Dump the buffer after all 8 channels are read
    Serial.write(singleBuffer, 24);
  }
  SPI.transfer(B00001111); //SDATAC stops the RDATAC - the received 's' just breaks the while(), this stops the acquisition
  digitalWrite(CS_pin, HIGH); //We finished the command sequence, so we switch it back to HIGH
  SPI.endTransaction();
}

void cycleDifferential() //APPROVED
{
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  //Set the AIN0+AIN1 as inputs manually
  digitalWrite(CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]
  SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
  SPI.transfer(0x00);
  SPI.transfer(B00000001);  //AIN0+AIN1
  digitalWrite(CS_pin, HIGH);
  delay(50);
  digitalWrite(CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]

  while (Serial.read() != 's')
  {
    for (int cycle = 0; cycle < 4; cycle++)
    {
      //Steps are on Page21
      //Step 1. - Updating MUX
      //DRDY has to go low
      while (dataReady == false) {}

      switch (cycle)
      {
        case 0: //Channel 2
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B00100011);  //AIN2+AIN3
          break;

        case 1: //Channel 3
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B01000101); //AIN4+AIN5
          break;

        case 2: //Channel 4
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B01100111); //AIN6+AIN7
          break;

        case 3: //Channel 1
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(B00000001); //AIN0+AIN1
          break;
      }

      SPI.transfer(B11111100); //SYNC
      delayMicroseconds(4); //t11 delay 24*tau = 3.125 us //delay should be larger, so we delay by 4 us
      SPI.transfer(B11111111); //WAKEUP

      //Step 3.
      SPI.transfer(B00000001); //Issue RDATA (0000 0001) command
      delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

      differentialBuffer[(3 * cycle)] = SPI.transfer(0x0F); //MSB comes in, first 8 bit is updated // '|=' compound bitwise OR operator
      differentialBuffer[(3 * cycle) + 1] = SPI.transfer(0x0F); //Mid-byte
      differentialBuffer[(3 * cycle) + 2] = SPI.transfer(0x0F); //LSB - final result
      dataReady = false;
    }
    //Dump the buffer after all 4 channels are read
    Serial.write(differentialBuffer, 12);
  }
  SPI.transfer(B00001111); //SDATAC stops the RDATAC - the received 's' just breaks the while(), this stops the acquisition
  digitalWrite(CS_pin, HIGH); //We finished the command sequence, so we switch it back to HIGH
  SPI.endTransaction();
}

void sendDirectCommand(uint8_t directCommand)
{
  //Direct commands can be found in the datasheet Page 34, Table 24.
  SPI.beginTransaction(SPISettings(1700000, MSBFIRST, SPI_MODE1));

  digitalWrite(CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"
  delayMicroseconds(5);
  SPI.transfer(directCommand); //Send Command
  delayMicroseconds(5);
  digitalWrite(CS_pin, HIGH); //REF: P34: "CS must stay low during the entire command sequence"

  SPI.endTransaction();
}

void userDefaultRegisters()
{
  // This function is "manually" updating the values of the registers then reads them back.
  // This function can be used in the setup() after performing an initialization-reset process
  /*
  	REG   VAL     USE
  	0     54      Status Register, Everyting Is Default, Except Auto - Cal
  	1     1       Multiplexer Register, AIN0 POS, AIN1 POS
  	2     0       ADCON, Everything is OFF, PGA = 0
  	3     132      DataRate = 100 SPS
  */
  //We update the 4 registers that we are going to use

  delay(500);
  writeRegister(0x00, B00110110); //STATUS: bit1: bufen=1; bit2: acal=1; rest is not important or factory default
  delay(200);
  writeRegister(0x01, B00000001); //MUX AIN0+AIN1
  delay(200);
  writeRegister(0x02, B00000000); //ADCON - PGA = 0 (+/- 5 V)
  delay(200);
  writeRegister(0x03, B10000100); //100SPS
  delay(500);
  sendDirectCommand(B11110000); //Offset and self-gain calibration
}

void printInstructions()
{
  //This function should be in the setup() and it shows the commands - not used
  PrintMessage = "*Use the following letters to send a command to the device:" + String("\n")
                 + "*r - Read a register. Example: 'r1' - reads the register 1" + String("\n")
                 + "*w - Write a register. Example: 'w1 8' - changes the value of the 1st register to 8." + String("\n")
                 + "*O - Single readout. Example: 'O' - Returns a single value from the ADS1256." + String("\n")
                 + "*A - Single, continuous reading with manual MUX setting." + String("\n")
                 + "*C - Cycling the ADS1256 Input multiplexer in single-ended mode (8 channels). " + String("\n")
                 + "*D - Cycling the ADS1256 Input multiplexer in differential mode (4 channels). " + String("\n")
                 + "*R - Reset ADS1256. Example: 'R' - Resets the device, everything is set to default." + String("\n")
                 + "*s - SDATAC: Stop Read Data Continously." + String("\n")
                 + "*U - User Default Registers."  + String("\n")
                 + "*d - Send direct command.";

  Serial.println(PrintMessage);
  PrintMessage = ""; //Reset (empty) variable.
}

void convertToVoltage(int32_t registerData)
{
  if (registerData >> 23 == 1) //if the 24th bit (sign) is 1, the number is negative
  {
    registerData = registerData - 16777216;  //conversion for the negative sign
    //"mirroring" around zero
  }
  //This is only valid if PGA = 0 (2^0). Otherwise the voltage has to be divided by 2^(PGA)
  double voltage = ((2 * VREF) / 8388608) * registerData; //5.0 = Vref; 8388608 = 2^{23} - 1

  Serial.println(voltage, 8); //print it on serial
}