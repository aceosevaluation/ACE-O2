#include <Wire.h>
#include <SSD1306Ascii.h>
//#include <SSD1306AsciiAvrI2c.h>
//#include <SSD1306AsciiSoftSpi.h>
//#include <SSD1306AsciiSpi.h>
#include <SSD1306AsciiWire.h>
#include <SSD1306init.h>
#include <RFduinoBLE.h>
//#include <Lazarus.h>




#define DEBUG 0
//#define DEBUG 1





// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
#define MCP3221_I2C_ADDR 0x48  //SG

SSD1306AsciiWire oled;

unsigned int kal_buff[20];
int adcRaw;
int Kalibrierfaktor;
float O2_Wert;
int i_kal = 1;
int ble_i = 0;
int ble_timeout = 0;
int Glaettungsfaktor = 100;
bool BLEconnected;
unsigned int glaett_buff;
bool ble_adv_active = false;
bool ble_active = false;
//Lazarus Lazarus;


void O2Kalibrierung(unsigned int cO2)
{
  unsigned char afe_i, i;
  float avg;

    // Mittelung des Sensor-Signal-Puffers:
    avg = 0.0;
    for (i=1;i<10+1;i++)
    {
      avg += (float)kal_buff[i];
    }
    avg /= (float)10;
    avg = -(avg / (log((float)(10000-cO2) / 10000.0)));


    
    //avg = -(avg / (log(1.0 - ((float)cO2 / 10000.0))));

    //avg = -(avg / (log( 1 - exp(-((float)cO2/100)))));
   
    
    avg /= 10.0;
    //avg *= 100.0;
    Kalibrierfaktor = (unsigned int)avg;
  
}


void O2Berechnung(void)
{
  unsigned int next;
  float cO2;

  // StÃ¤ndig Werte fÃ¼r die nÃ¤chste Kalibrierung sammeln:
    
    kal_buff[kal_buff[0]] = adcRaw;
    kal_buff[0]++;
    if (kal_buff[0] > 10) kal_buff[0] = 1;
    if ((Kalibrierfaktor > 0) && (Kalibrierfaktor < 65535))
    {
      cO2 = 100.0 * (1.0 - exp(-((float)adcRaw)/(10.0 * ((float)Kalibrierfaktor))));
      //cO2 *= 100.0;
      O2_Wert= cO2;
    } 
    else {
      O2_Wert = adcRaw;
    }
  
}


//void Glaettung(void)
//{
//  unsigned int  j;
//  unsigned char base_addr;
//  float gf, ge;
//
//      
//      gf = (float)Glaettungsfaktor / 65535.0;
//      ge = ((float)O2_Wert * gf) + ((float)glaett_buff * (1.0 - gf));
//      glaett_buff = (unsigned int)ge;
//    
//  
//}

void read_write_O2 (){

  
 
float cO2;
byte adc_MSB;
byte adc_LSB;

      // Do whatever you need to do in the LOOP while alive again!
     
       Wire.requestFrom(MCP3221_I2C_ADDR, 2);        //requests 2 bytes
       //while(Wire.available() < 2);         //while two bytes to receive
     
       adc_MSB = Wire.read();
       adc_LSB = Wire.read();
       adcRaw = (adc_MSB * 256) + adc_LSB;
      O2Berechnung();
     // Glaettung();
     if (i_kal <= 100) { 
      i_kal = i_kal + 1;
     }

// Display schreiben
    //float temp = RFduino_temperature(CELSIUS); // returns temperature in Celsius and stores in float temp  
    oled.setCursor(0,1); 
    oled.println(O2_Wert);  
    oled.setCursor(0,3);
    oled.println(adcRaw);

    //BLE senden
    RFduinoBLE.sendFloat(O2_Wert);
  
}



void setup()   {                
 
 //TODO nur wenn nicht im ULP mode
// Wire.begin();     
// 
//  
//  oled.begin(&Adafruit128x32, I2C_ADDRESS);
//  oled.set400kHz();  
//  oled.setFont(font8x8);
//  oled.clear();
//  oled.println("idle...");
//  
//

RFduinoBLE.deviceName = "ACE-O2 Eval"; //Sets the device name to RFduino
  // this is the data we want to appear in the advertisement
  // (if the deviceName and advertisementData are too long to fix into the 31 byte
  // ble advertisement packet, then the advertisementData is truncated first down to
  // a single byte, then it will truncate the deviceName)
  RFduinoBLE.advertisementData = "O2_Wert";
  RFduinoBLE.customUUID = "a3a4b1bc-222a-4d2f-83b1-95611de6e0fc";
  //RFduinoBLE.advertisementInterval = 3000;
  // start the BLE stack
  RFduinoBLE.begin();

        Wire.begin();     
        oled.begin(&Adafruit128x32, I2C_ADDRESS);
        oled.set400kHz();  
        oled.setFont(font8x8);
        oled.clear();
        oled.println("O2 Wert:");
        oled.setCursor(0,2);
        oled.println("Raw:");
  
 
//Kal Button Setup
pinMode(2, INPUT_PULLUP);

 
}

void loop() {
 
float cO2;
byte adc_MSB;
byte adc_LSB;
read_write_O2 ();
//Button Handling
// Kalibrieren wenn GPIO 2 auf 3 sekunden auf Masse gezogen wird
if (!digitalRead(2)) {
  ble_i = ble_i + 1;

        //3sekunden a 50ms = 60
        if (ble_i >= 60) {
          oled.clear();
          oled.println("O2 Wert:");
          oled.setCursor(0,2);
          oled.println("Raw:");
          O2Kalibrierung(2090);
          ble_i = 0;
          }
      
} else {
   ble_i = 0;
}




delay(50);

}

//end loop


