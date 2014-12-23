//Datalogger
//Light sensors and accelerometer
//Magaly Sandoval
//Compology
//December, 2014.
//V1.3

/*Hardware setup:
 MMA8452 Breakout ---------- Arduino
 3.3V ---------------------- 3.3V
 SDA -------^^(330)^^------- A4
 SCL -------^^(330)^^------- A5
 GND ---------------------- GND
 
 TSL2561 Breakout ---------- Arduino
 Vin ----------------------- 5V
 SDA -------^^(330)^^------- A4
 SCL -------^^(330)^^------- A5
 GND ---------------------- GND
  
  */
  //Libraries required
  #include <Wire.h>
  #include <SD.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_TSL2561_U.h>
  #include "RTClib.h"

  const int pwmPin = 3;
  long distance, cm;

/*   I2C Address
   ===========
   The address will be different depending on whether you leave
   the ADDR pin floating (addr 0x39), or tie it to ground or vcc. 
   The default addess is 0x39, which assumes the ADDR pin is floating
   (not connected to anything).  If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.
    
   History
   =======
   2013/JAN/31  - First version (KTOWN)
*/
   
  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 1);
  Adafruit_TSL2561_Unified tsl2 = Adafruit_TSL2561_Unified(TSL2561_ADDR_HIGH, 2);
  Adafruit_TSL2561_Unified tsl3 = Adafruit_TSL2561_Unified(TSL2561_ADDR_HIGH, 3);
  
  //Maxbotix rangeSensorPW(8, Maxbotix::PW, Maxbotix::LV);
  //Maxbotix rangeSensorTX(6, Maxbotix::TX, Maxbotix::LV);
  //Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV);


  // how many milliseconds between grabbing data and logging it. 
  #define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)
  
  // how many milliseconds before writing the logged data permanently to disk
  // set it to the LOG_INTERVAL to write each time (safest)
  // set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
  // the last 10 reads if power is lost but it uses less power and is much faster!
  #define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
  uint32_t syncTime = 0; // time of last sync()
  
  #define ECHO_TO_SERIAL   1 // echo data to serial port
  #define WAIT_TO_START    0 // Wait for serial input in setup()
  
  // the digital pins that connect to the LEDs
  #define redLEDpin 2
  #define greenLEDpin 3
  
  #define BANDGAPREF 14            // special indicator that we want to measure the bandgap
  
  #define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
  #define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off
  
  // The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
  #define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low
  
  //Define a few of the registers that we will be accessing on the MMA8452
  #define OUT_X_MSB 0x01
  #define XYZ_DATA_CFG  0x0E
  #define WHO_AM_I   0x0D
  #define CTRL_REG1  0x2A
  
  #define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
  
  RTC_DS1307 RTC; // define the Real Time Clock object
  
  // for the data logging shield, we use digital pin 10 for the SD cs line
  const int chipSelect = 10;
  
  // the logging file
  File logfile;
       
  void error(char *str)
  {
    Serial.print("error: ");
    Serial.println(str);
    
    // red LED indicates error
    digitalWrite(redLEDpin, HIGH);
  
    while(1);
  }
  
  
  void setup(void)
  {
    Serial.begin(9600);
    Serial.println();
    Wire.begin(); //Join the bus as a master
    
    initMMA8452(); //Test and intialize the MMA8452
  
    // use debugging LEDs
    pinMode(redLEDpin, OUTPUT);
    pinMode(greenLEDpin, OUTPUT);
    pinMode(pwmPin, INPUT);
    
  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START
  
    // initialize the SD card
    Serial.print("Initializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);
    
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      error("Card failed, or not present");
    }
    Serial.println("card initialized.");
    
    // create a new file
    char filename[] = "Logger00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
      }
    }
    
    if (! logfile) {
      error("couldnt create file");
    }
    
    Serial.print("Logging to: ");
    Serial.println(filename);
  
    // connect to RTC
    Wire.begin();  
    if (!RTC.begin()) {
      logfile.println("RTC failed");
  #if ECHO_TO_SERIAL
      Serial.println("RTC failed");
  #endif  //ECHO_TO_SERIAL
    }
    
    logfile.println("MILIS,DATETIME,LUX1,LUX2,LUX3,X,Y,Z,DISTANCE");    
  #if ECHO_TO_SERIAL
    Serial.println("MILIS,DATETIME,LUX1,LUX2,LUX3,X,Y,Z,DISTANCE");
  #endif //ECHO_TO_SERIAL
   
    // If you want to set the aref to something other than 5v
    analogReference(EXTERNAL);
    
    /* Initialize the Light sensor */
    
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    if(!tsl2.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561_2 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
      if(!tsl3.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561_2 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println(""); 
    
  }
  
  void loop(void)
  {
    DateTime now;
  
    // delay for the amount of time we want between readings
    delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
    
    digitalWrite(greenLEDpin, HIGH);
    
    // log milliseconds since starting
    uint32_t m = millis();
    logfile.print(m);           // milliseconds since start
    logfile.print(", ");    
  #if ECHO_TO_SERIAL
    Serial.print(m);         // milliseconds since start
    Serial.print(", ");  
  #endif
  
    // fetch the time
    now = RTC.now();
    // log time
    logfile.print('"');
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print('"');
    logfile.print(",");

    
   //Write Light Data
  //*******************************************************************************
    /* Get a new sensor event */ 
  sensors_event_t event;
  sensors_event_t event2;
  sensors_event_t event3;

  tsl.getEvent(&event);
  tsl2.getEvent(&event2);
  tsl3.getEvent(&event3);
  
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux");
    logfile.print(event.light);
    logfile.print(",");
 }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }
  if (event2.light)
  {
    Serial.print(event2.light); Serial.println(" lux2");
    logfile.print(event2.light);
    logfile.print(",");
  }
   if (event3.light)
  {
    Serial.print(event3.light); Serial.println(" lux3");
    logfile.print(event3.light);
  }
   
  delay(500);
   //Write Acceleration Data
  //*******************************************************************************
     
     int accelCount[3];  // Stores the 12-bit signed value
  
     readAccelData(accelCount);  // Read the x/y/z adc values
      
    // Now we'll calculate the acceleration value into actual g's
    float accelG[3];  // Stores the real accel value in g's
    for (int i = 0 ; i < 3 ; i++)
    {
      accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
      
    }
    
    logfile.print(", ");
    logfile.print(accelG[0]);
    logfile.print(", ");
    logfile.print(accelG[1]);
    logfile.print(", ");
    logfile.print(accelG[2]);
    
  //Distance Sensors Read
    
    distance = pulseIn(pwmPin, HIGH);
    cm = distance/58;
    Serial.print("S1");
    Serial.print(" = ");
    Serial.print(cm);
    Serial.print("cm");
    
    logfile.print(",");
    logfile.print(cm);
    
    
                                                   
  //New Line
    logfile.println();
  #if ECHO_TO_SERIAL
    Serial.println();
  #endif // ECHO_TO_SERIAL
  
    digitalWrite(greenLEDpin, LOW);
  
    // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
    // which uses a bunch of power and takes time
    if ((millis() - syncTime) < SYNC_INTERVAL) return;
    syncTime = millis();
    
    // blink LED to show we are syncing data to the card & updating FAT!
    digitalWrite(redLEDpin, HIGH);
    logfile.flush();
    digitalWrite(redLEDpin, LOW);
        
  }
  
  //************************************************************************/
  
  // ---------------------ACCELEROMETER OPERATIONS----------------------------
  
  
  void readAccelData(int *destination)
  {
    byte rawData[6];  // x/y/z accel register data stored here
  
    readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array
  
    // Loop to calculate 12-bit ADC and g value for each axis
    for(int i = 0; i < 3 ; i++)
    {
      int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
      gCount >>= 4; //The registers are left align, here we right align the 12-bit integer
  
      // If the number is negative, we have to make it so manually (no 12-bit data type)
      if (rawData[i*2] > 0x7F)
      {  
        gCount = ~gCount + 1;
        gCount *= -1;  // Transform into negative 2's complement #
      }
  
      destination[i] = gCount; //Record this gCount into the 3 int array
    }
  }
  
  // Initialize the MMA8452 registers 
  // See the many application notes for more info on setting all of these registers:
  // http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
  void initMMA8452()
  {
    byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
    if (c == 0x2A) // WHO_AM_I should always be 0x2A
    {  
      Serial.println("MMA8452Q is online...");
    }
    else
    {
      Serial.print("Could not connect to MMA8452Q: 0x");
      Serial.println(c, HEX);
      while(1) ; // Loop forever if communication doesn't happen
    }
  
    MMA8452Standby();  // Must be in standby to change registers
  
    // Set up the full scale range to 2, 4, or 8g.
    byte fsr = GSCALE;
    if(fsr > 8) fsr = 8; //Easy error check
    fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
    writeRegister(XYZ_DATA_CFG, fsr);
  
    //The default data rate is 800Hz and we don't modify it in this example code
  
    MMA8452Active();  // Set to active to start reading
  }
  
  // Sets the MMA8452 to standby mode. It must be in standby to change most register settings
  void MMA8452Standby()
  {
    byte c = readRegister(CTRL_REG1);
    writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
  }
  
  // Sets the MMA8452 to active mode. Needs to be in this mode to output data
  void MMA8452Active()
  {
    byte c = readRegister(CTRL_REG1);
    writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
  }
  
  // Read bytesToRead sequentially, starting at addressToRead into the dest byte array
  void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
  {
    Wire.beginTransmission(MMA8452_ADDRESS);
    Wire.write(addressToRead);
    Wire.endTransmission(false); //endTransmission but keep the connection active
  
    Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default
  
    while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect
  
    for(int x = 0 ; x < bytesToRead ; x++)
      dest[x] = Wire.read();    
  }
  
  // Read a single byte from addressToRead and return it as a byte
  byte readRegister(byte addressToRead)
  {
    Wire.beginTransmission(MMA8452_ADDRESS);
    Wire.write(addressToRead);
    Wire.endTransmission(false); //endTransmission but keep the connection active
  
    Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default
  
    while(!Wire.available()) ; //Wait for the data to come back
    return Wire.read(); //Return this one byte
  }
  
  // Writes a single byte (dataToWrite) into addressToWrite
  void writeRegister(byte addressToWrite, byte dataToWrite)
  {
    Wire.beginTransmission(MMA8452_ADDRESS);
    Wire.write(addressToWrite);
    Wire.write(dataToWrite);
    Wire.endTransmission(); //Stop transmitting
  }
 
   //************************************************************************/
  
  // ---------------------LIGHT OPERATIONS----------------------------
   
   void displaySensorDetails(void)
{
  sensor_t sensor;
  sensor_t sensor2;
  sensor_t sensor3;
  tsl.getSensor(&sensor);
  tsl2.getSensor(&sensor2);
  tsl3.getSensor(&sensor3);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor1:       "); Serial.println(sensor.name);
  Serial.print  ("Sensor2:       "); Serial.println(sensor2.name);  
  Serial.print  ("Sensor2:       "); Serial.println(sensor3.name);
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500); 
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  tsl2.enableAutoRange(true);
  tsl3.enableAutoRange(true);
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl2.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl3.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
}



