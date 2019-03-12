#include <Sodaq_LPS22HB.h>

#include <Arduino.h>
#include <Wire.h>
//#include <Sodaq_wdt.h>
//#include <Sodaq_nbIOT.h>
#include <Sodaq_HTS221.h>

// Include for acoustics sensor//////////////////////////////////////
#include "ADC.h"
#include <Snooze.h>
/////////////////////////////////////////////////////////////////////

//#include <Wire.h>
#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)
#define IP "172.27.131.100"
#define INTERVAL 61 //minutes to sleep

#define SPEED_OF_SOUND 343.2 //m/s
#define SAMPLE_PERIOD 23
#define SAMPLE_RATE 1.0/SAMPLE_PERIOD
#define number_samples_out 90 //2ms per chirp
#define number_samples_in (number_samples_out * 15) //5.8ms*4m=23.2+1 chirp = 24.2ms



//////////////////////////////////////////////
// global variable for sensor's code////////////////////////////////////////////////////////
// Create an timer for sleep object
SnoozeTimer timer;
SnoozeBlock config_teensyLC( timer);

// Create an IntervalTimer object
IntervalTimer myTimer;
ADC *adc = new ADC(); // adc object


uint16_t chirp[] = {
  2048, 2339, 2633, 2925, 3205, 3464, 3693, 3880, 4015, 4087,
  4086, 4006, 3840, 3588, 3254, 2845, 2379, 1878, 1370, 891,
  480, 176, 16, 31, 238, 635, 1199, 1880, 2604, 3279,
  3801, 4074, 4027, 3635, 2935, 2037, 1113, 371, 12, 170,
  853, 1911, 3045, 3877, 4073, 3496, 2305, 967, 97, 179,
  1262, 2822, 3954, 3881, 2541, 805, 0, 885, 2823, 4071,
  3313, 1189, 0, 1257, 3546, 3837, 1562, 0, 1721, 4011,
  2783, 157, 1313, 4013, 2346, 3, 2677, 3696, 299, 1863,
  3904, 286, 2333, 3334, 24, 3844, 1064, 2092, 2048, 2048
};



int16_t samples[number_samples_in];
volatile int current_sample = 0;

volatile boolean should_sample = false;
volatile boolean doneSample = false;

#define KEEP_MAXS 10
int32_t sums[KEEP_MAXS][2] = {{0}};


unsigned int sleepCount = 0;
unsigned int sampleCount = 0;
int16_t data_count = 0;
//Sodaq_nbIOT nbiot;
////////////////////////////////////////////////////////
// variable for temperature and humidity
Sodaq_HTS221 humiditySensor;
/////////////////////////////////
Sodaq_LPS22HB barometricSensor;


void setup() {

  while ((!Serial) && (millis() < 10000)) {
    // Wait for serial monitor for 10 seconds
  }

  pinMode(13, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  digitalWrite(7, HIGH);
  digitalWrite(13, LOW);
  delay(200);
  Serial.begin(9600);
  Serial1.begin(9600);
  delay (220);
  //Serial.println("in set tup");

  // initializing ADC ///////////////////////##################################################################################################

  ///////////////////////////// set ADC
  adc->setAveraging(0); // set number of averages
  adc->setResolution(12); // set bits of resolution

  /* it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
     see the documentation for more information
    adc->setConversionSpeed(ADC_ADACK_6_2); // change the conversion speed
  */
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::ADACK_6_2); // change the conversion speed

  /* it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    adc->setSamplingSpeed(ADC_VERY_HIGH_SPEED); // change the sampling speed
  */

  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  //  adc->startContinuous(0, ADC_0) ;

  analogWriteResolution(12);

  myTimer.begin(scheduledTask, SAMPLE_PERIOD);  // blinkLED to run every 0.15 seconds

  delay(100);

  //sensor power pins
  //  pinMode(14, OUTPUT); cai nay ko biet
  pinMode(21, OUTPUT); //Vdd for MIC through resistor 2.2k //cai nay dung trong mach cu
//  pinMode(17, OUTPUT); //Vdd for MIC through resistor 2.2k
  //  digitalWrite(14, LOW);cai nay ko biet
    digitalWrite(21, LOW); //cai nay dung trong mach cu
//  digitalWrite(17, LOW);
  delay(2000); //delay for Serial1 ?


  /////////////////////////////////////////////////



}









void loop() {
  pinMode(13, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(3, OUTPUT);
  unsigned int strLen;
  char sendmsgResponse[5];
  char RSSIResponse[5];
  byte message[8];
  uint16_t cursor = 0;
  int16_t depth ;
  int16_t count ;
  int16_t SignalStrength = 80;
  //  bool reSocket;
  //  int16_t Signal = -113;

  //  Serial.begin(9600);
  //  Serial1.begin(9600);
  delay(200);
  //////////////////////////////////////
  /*
    reregister and open socket

  */
digitalWrite(21, HIGH);
  if (reRegister()) {
    createSocket(7000);
  }


  delay(5000);

  ////////////initial temperature

  Wire.begin();
  //  Serial.println("Initializing Temperature + humidity sensor");

  if (humiditySensor.init()) {
    //    Serial.println("Temperature + humidity sensor initialized.");
    humiditySensor.enableSensor();
  }
  else {
    //    Serial.println("Temperature + humidity initialization failed!");
  }
  if (barometricSensor.init()) {
    //        Serial.println("Barometric sensor initialization succeeded!");
    barometricSensor.enableSensor(Sodaq_LPS22HB::OdrOneShot);
  }
  else {
    //        Serial.println("Barometric sensor initialization failed!");
  }


//  Serial.println("Done with setup!");





  ////////////////////////////////////////////////////


  /////////////////////////////////////////////////
  //  configtestNbiot();

  ////////////////////////////////////////////////////////////////////////////////////////

  //  digitalWrite(21, HIGH); //Vdd for MIC through resistor 2.2k, turn on MIC
  digitalWrite(17, HIGH); //thay cho chan 21 tren board adapter sparkfun
  delay(1000); //hoi cao va lap lai ??????????????????????
  ///////////////////////////////////////////////////////////////////////////////////////////


  ///code from loop send sensor data///////////////////////////////////////////////////////////////////////////

  if (!doneSample && !should_sample)
  {
    //    Serial.println("Starting");
    adc->startContinuous(0, ADC_0); // should enable from begining?
    current_sample = 0;
    should_sample = true;
    //    digitalWrite(ledPin, HIGH);
  }

  if (doneSample)
  {
    doneSample = false;
    adc->stopContinuous(ADC_0); // should enable from begining? co nen dat line cuoi cua schedulestask de dung adc ngay khong???

    CorrectBufferMean(samples, number_samples_in);
    unsigned int peak = CalculateCrossCorrelation(samples, number_samples_in, chirp, number_samples_out);

    unsigned int distance = (double)peak * (SAMPLE_PERIOD * SPEED_OF_SOUND / 2000.0); //works out like this by chance //peak*1000/SAMPLE_RATE/5.8;
    //distance = distance*thousandSamplesTime/5668.934;
    Serial.println(distance);
    //Serial.println("Debugkjh33###################fffffffffffffffffffffffffffffffffffffff###gggg");

    //digitalWrite(7, HIGH); //turn on NB-IOT
    delay(500);








    // Read RSSI and BER--------------------------
    SignalStrength = 16;

    //////////////////////////////////////////////////////////////////////////////////////
    //#############?? RSSI


    Serial1.println("AT+CSQ");
    while (Serial1.available() < 1) {}
    strLen = Serial1.readBytesUntil('\0', RSSIResponse, 26);
    Serial.write(RSSIResponse);
    Serial.println(strLen);
    if (RSSIResponse[9] == ',') {
      int16_t convertRSSI = (int)(RSSIResponse[8]) - 48;
      SignalStrength = (int16_t) convertRSSI * 2 - 113;
    }
    else {
      int16_t convertRSSI = (int16_t)(RSSIResponse[9]) - 48;
      int16_t convertRSSI10 = (int16_t)(RSSIResponse[8]) - 48;
      Serial.println(convertRSSI10);
      SignalStrength = (int16_t)(convertRSSI + convertRSSI10 * 10) * 2 - 113;
    }
    Serial.println(RSSIResponse[9]);

    /////////////////////////////////////////////////////////////////

//    rssiNbiot(&SignalStrength);


    ///////////////////////////////////////////////////////////////////////////////////
    Serial.println(SignalStrength);
    SignalStrength = -SignalStrength;
    Serial.println("SignalStrengt RSSI");



    count = data_count;
    //DEBUG_STREAM.println(count);
    message[cursor++] = count >> 8;
    message[cursor++] = count;

    message[cursor++] = SignalStrength >> 8;
    message[cursor++] = SignalStrength;



    // convert to HEX//////////////////////////////////////////////////////////
    depth = (int16_t) distance;
    Serial.print("Depth: ");
    Serial.println(depth);
    message[cursor++] = depth >> 8;
    message[cursor++] = depth;



    int16_t temperatureNbiot = (int16_t) (humiditySensor.readTemperature()) * 100;
    Serial.print("temperatureNbiot:");
    Serial.println(temperatureNbiot);
    message[cursor++] = temperatureNbiot >> 8;
    message[cursor++] = temperatureNbiot;



    int16_t humidityNbiot = (int16_t) (humiditySensor.readHumidity()) * 100;
    Serial.print("humidityNbiot:");
    Serial.println(humidityNbiot);
    message[cursor++] = humidityNbiot >> 8;
    message[cursor++] = humidityNbiot;



    int16_t pressureNbiot = (int16_t) (barometricSensor.readPressureHPA()) * 1;
    
    Serial.print("pressureNbiot: ");
    Serial.println(pressureNbiot);
    message[cursor++] = pressureNbiot >> 8;
    message[cursor++] = pressureNbiot;

    //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // code send directly


    Serial1.print("AT+NSOST=0,\"");
    Serial1.print(IP);
    Serial1.print("\",15683,");
    Serial1.print(cursor);
    Serial1.print(",\"");
    for (unsigned int i = 0; i < cursor; ++i) {
      Serial1.print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(message[i]))));
      Serial1.print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(message[i]))));
    }
    Serial1.println("\"");

    while (Serial1.available() < 1) {}
    strLen = Serial1.readBytesUntil('\0', sendmsgResponse, 26);
    Serial.write(sendmsgResponse);
    Serial.println(strLen);


    //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    ///////////////////////////////////////////////////////////////////////////////



    Serial.println("\r\n after send meassage");
delay(500);
    if ((data_count++) > 50000)
    { data_count = 0;
    }
  }





  /////////////////////////////////sleep faaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa



  // Hibernate****30 min *************************dddddddddddddddddddddddddddddddddddddddddddd
  if (!doneSample && !should_sample)
  {
    Serial.println("Test after RSSI 2");
    digitalWrite(7, LOW); //turn off NB IOT
    //    digitalWrite(13, LOW);
    digitalWrite(21, LOW);
    //    delay(300000);

    ////////////////////////

    //switch off DAC - https://forum.pjrc.com/threads/28122-Achieving-Low-Power-Consumption-with-Teens-3-1-and-the-latest-Teensyduino-Version/page2
        DAC0_C0 = ~DAC_C0_DACEN; //DAC_C0_LPEN
        SIM_SCGC6 &= ~SIM_SCGC6_DAC0;// disable DAC0 clock

    //sleep $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


    digitalWrite(7, LOW);
    while (sleepCount < 60) //number of minutes to sleep-1
    {
      sleepCount++;
//      timer.setTimer(60000 - 2040); //compensate for delay
      timer.setTimer(60000 - 140); //compensate for delay
      Snooze.hibernate( config_teensyLC );// return module that woke processor
    }

    // End of Hilbertnate////////////////////////////////////////////////

    //loop to debud instead of sleep $$$$$$$$$$$$$$/////////////////////////////////////////////////////////////////////////
    /*
        digitalWrite(7, LOW);
        digitalWrite(13, HIGH);
        for (int i = 0; i < 10; i++) {

          delay(200);

          delay(5000);

        }
    */
    digitalWrite(13, LOW);


    ///////////////////////////////////////////////////////////////////////////////////

    digitalWrite(7, HIGH);
    sleepCount = 0;

    delay (200);

  }

}

// End of loop ///////////////////////////////////////////////////////////////////////////////

/*

  This section is function of sensors


*/




/////////////////////////////////////////////////////////////





// function of acoustics//////////////////////////////////////////////////////////////



void scheduledTask(void)
{

  if (should_sample)
  {

    ////////////////////////////////////////////////code for test current
    if (current_sample == 0)
    {
      digitalWrite(3, HIGH);
    }

    ////////////////////////////////////////////////code for test current

    if (current_sample < number_samples_out)
    {
      uint32_t ulOutput = chirp[current_sample];
      analogWrite(26, ulOutput);
    }
    else
    {
      analogWrite(26, 0);
      pinMode(26, INPUT);
    }

    samples[current_sample] = adc->analogReadContinuous(ADC_0) - (2 ^ 11);

    current_sample += 1;
    if (current_sample >= number_samples_in)
    {
      digitalWrite(3, LOW); //code code for test current
      current_sample = 0;
      should_sample = false;
      doneSample = true;
      /// nen stop adc hier????????????????????????
    }
  }
}


/////////////////////////////////////////////////////
void CorrectBufferMean(int16_t *buffer, int num_values)
{
  int32_t sum = 0;
  int16_t max = 0;
  for (uint16_t i = 0; i < num_values; ++i)
  {
    sum += buffer[i];
    if (buffer[i] > max)
    {
      max = buffer[i];
    }
  }

  int16_t mean = (int16_t)(sum / num_values);
  double scaler = 4095 / (max - mean);

  for (int i = 0; i < num_values; ++i)
  {
    buffer[i] = (buffer[i] - mean) * scaler;
  }
}



uint16_t CalculateCrossCorrelation(int16_t *buffer, int buffer_size, uint16_t *reference_buffer, int reference_size)
{
  uint16_t max_time_offset = 0;
  int32_t max_sum = 0;

  for (int i = 0; i < KEEP_MAXS; i++)
  {
    sums[i][0] = 0;
    sums[i][1] = 0;
  }

  for (uint16_t time_offset = 50; time_offset < buffer_size; ++time_offset) //start at 50->20cm
  {
    uint32_t search_bits;// = std::min<uint32_t>(buffer_size - time_offset, reference_size);
    if (buffer_size - time_offset < reference_size)
    {
      search_bits = buffer_size - time_offset;
    }
    else
    {
      search_bits = reference_size;
    }

    int32_t sum = 0;
    for (size_t i = 0; i < search_bits; ++i)
      sum += buffer[time_offset + i] * ((int32_t)(reference_buffer[i]) - 2047) * -1; //-1 to invert signal. Reflection is inverted.

    if (sum > max_sum)
    {
      max_sum = sum;
      max_time_offset = time_offset;
    }

  }

  return max_time_offset;
}












/////////////////////////////////////////////////////////////////////////////////////////////////

bool reRegister() {
  unsigned int strLen;
  char regResponse[5];
  char bandResponse[5];
  char forceResponse[5];
  //  char csqResponse[5];
  int i = 0;


  delay(4000);
  /*
    Clear incoming buffer after reboot module by pin 7
    Turn off this pin for power saving
  */


  //  while (Serial1.available() < 1) {
  //    delay(100);
  //  }
  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }

  Serial1.println("AT+CEREG\?");
  while (Serial1.available() < 1) {}
  strLen = Serial1.readBytesUntil('\0', regResponse, 26);
  Serial.print(regResponse);
  Serial.print(strLen);
  if (regResponse[12] == '5') {
    return true;
  }
  while (regResponse[12] != '5') {
    i++; // count for timeout
    /*
       band register///////////////////////////////////////////////

    */

    //    Serial.println(" AT+NBAND$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    Serial1.println("AT+NBAND=8");
    strLen = Serial1.readBytesUntil('\0', bandResponse, 26);
    Serial.print(bandResponse);
    //    Serial.print("Kthuc AT+NBAND\r\n");
    //////////////////////////////////////////////////

    delay(2000);


    /*
       force operator///////////////////////////////////////////////////

    */

    //    Serial.println("Force $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    Serial1.println("AT+COPS=1,2,\"20416\"");
    strLen = Serial1.readBytesUntil('\0', forceResponse, 26);
    Serial.print(forceResponse);
    //    Serial.println("End Force ");

    /*

         checking register again ///////////////////////////

    */
    delay(3000);
    Serial1.println("AT+CEREG\?");
    while (Serial1.available() < 1) {}

    //strResponse[15]='\0';
    strLen = Serial1.readBytesUntil('\0', regResponse, 26);
    Serial.print(regResponse);

  }
  /////////////////////////////////////////////////////////////


  //Serial1.println("AT+CSQ");
  //  while (Serial1.available() < 1) {}
  //  strLen = Serial1.readBytesUntil('\0', csqResponse, 26);
  //  Serial.print(csqResponse);

  /////////////////////////////////////////////////////////////

  return false;
}

bool createSocket(unsigned int listen_port ) {
  unsigned int strLen;
  char socketResponse[5];
  // char sendmsgResponse[5];
  ///////////////////////////////////////////////////////////////////////////////

  //  Serial.println("Begin Test for socket======================");
  Serial1.print("AT+NSOCR=\"DGRAM\",17,");
  Serial1.print(listen_port);
  Serial1.print(",1\r\n");
  //  Serial1.print("AT+NSOCR=\"DGRAM\",17,7000,1\r\n");
  while (Serial1.available() < 1) {}
  strLen = Serial1.readBytesUntil('\0', socketResponse, 26);
  //  Serial.print(socketResponse);
  //  Serial.println(strLen);
  //  Serial.println("Endn Test for socket======================");

  ///////////////////////////////////////////////////////////////////////////////////

  //Serial.println("Begin register Test for send======================");
  //  Serial1.write("AT+NSOST=0,\"172.27.131.100\",15683,2,\"aaaa\"\r\n");
  //  while (Serial1.available() < 1) {}
  //  strLen = Serial1.readBytesUntil('\0', sendmsgResponse, 26);
  //  Serial.write(sendmsgResponse);
  //Serial.println(strLen);
  //
  //  Serial.println("ENd Test for send in function======================");
  if (strLen == 11) {
    return true;
  }
  else return false;

}

/*
void sendMessageNbiot(const byte* buffer_1, unsigned int size_1)
{

  unsigned int strLen;
  char sendmsgResponse[5];
  //  if (size_1 > 512) {
  //    return false;
  //  }


  //  Serial1.print("AT+NSOST=0,\"172.27.131.100\",15683,");
  Serial1.print("AT+NSOST=0,\"");
  Serial1.print(IP);
  Serial1.print("\",15683,");



  Serial1.print(size_1);

  Serial1.print(",\"");

  for (uint16_t i = 0; i < size_1; ++i) {
    Serial1.print(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(buffer_1[i]))));
    Serial1.print(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(buffer_1[i]))));
  }

  Serial1.println("\"");
  while (Serial1.available() < 1) {}
  strLen = Serial1.readBytesUntil('\0', sendmsgResponse, 26);
  Serial.write(sendmsgResponse);
  Serial.println(strLen);
  if (strLen != 13) {
    //    return false;
  }
  Serial.println("ENd in fuction Test for send======================");
  Serial.println(true);
  //  return true;
}


////////////////////////////////////////////////////////////////////////

bool configtestNbiot( ) {
  unsigned int strLen;
  char socketResponse1[200];
  // char sendmsgResponse[5];
  ///////////////////////////////////////////////////////////////////////////////

  //  Serial.println("Begin Test for socket======================");
  Serial1.println("AT+NCONFIG\?");
  while (Serial1.available() < 1) {}
  strLen = Serial1.readBytesUntil('\0', socketResponse1, 280);
  Serial.println(socketResponse1);

  if (strLen == 21) {
    return true;
  }
  else return false;

}

*/
/////////////////////////////////////////////////////////////////////



/*

  bool testforRSSI(void) {
  char RSSInbiot[5];
  unsigned int strLen1;
  Serial1.write("AT+CSQ\r\n");
  delay(1000);
  while (Serial1.available() < 1) {}
  strLen1 = Serial1.readBytesUntil('\0', RSSInbiot, 26);
  Serial.print(RSSInbiot);
  Serial.println(strLen1);
  Serial.println("tai sao khong thoat");
  while (Serial1.available() > 0) {
    char t = Serial1.read();
  }

  while (Serial.available() > 0) {
    char t = Serial.read();
  }
  delay(1000);
  return true;
  }


*/

















/*


void rssiNbiot(int16_t *rssi) {
  char RSSInbiot[5];
  unsigned int strLen;
  //#############?? RSSI


  Serial1.println("AT+CSQ");
  while (Serial1.available() < 1) {}
  strLen = Serial1.readBytesUntil('\0', RSSInbiot, 26);
  Serial.write(RSSInbiot);
  Serial.println(strLen);
  if (RSSInbiot[9] == ',') {
    int16_t convertRSSI = (int16_t)(RSSInbiot[8]) - 48;
    *rssi = (int16_t) convertRSSI * 2 - 113;
  }
  else {
    int16_t convertRSSI = (int16_t)(RSSInbiot[9]) - 48;
    int16_t convertRSSI10 = (int16_t)(RSSInbiot[8]) - 48;
    *rssi = (int16_t)(convertRSSI + convertRSSI10 * 10) * 2 - 113;
  }


  /////////////////////////////////////////////////////////////////
}
*/

