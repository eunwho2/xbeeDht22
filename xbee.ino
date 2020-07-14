String fs="M,802"; 

// ZigBee:38400 baud
#define baudXBee 38400
// #define cycleWDT 30          // ~300 sec WDT timing
#define cycleWDT 2          // ~300 sec WDT timing
#define cycleM 5              // Master info send cycle 
#define upperBatteryVolt 4.0  // high battery voltage clamping
#define lowerBatteryVolt 3.3  // high battery voltage clamping
#define countLimit 1000000    // stamp counter limit 

#include <SoftwareSerial.h>
#include <EEPROM.h>
/*
 * Sketch for testing sleep mode with wake up on WDT.
 * Donal Morrissey - 2011.
 *
 */
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT22   

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial mySerial(2, 3); // RX, TX


volatile unsigned int rep0=cycleWDT;//15;// volatile unsigned int rep0=15; //8초 사이클 대기 수 : 실제는 10회 기준
volatile unsigned int cnt;   //8초 사이클 대기 수
unsigned long lastCnt=0;
volatile int f_wdt=1;

unsigned long strCnt=0;

int eead=0;
int een=0;
long ms00=0;

String nameNI="";
String s="";
String ss="";
String cmd="";
String txString="";
String zbInfo="";
String dbString="";

int eer0[30];

long ms0=0;
int ms=10*1000;

float coreTemp;
float currentVolt;   // device voltage

float v1,v2;        // v1=solar, v2=battery
//float f200 = 230;//231.95;
//float f200s = 136.4;//142.1;

float tmpScale;
float tmpOffset;
float roomTemp;
float roomHumi;
void setup() {

  dht.begin();

  float x1,x2,y1,y2;
  
  x1 = 270.0   ; y1 = 25.0;
  x2 = 350.0   ; y2 = 105.0;

  tmpScale = ( y2-y1) / ( x2 - x1 );
  tmpOffset = (( y1 * x2 - y2 * x1 )/ (x2- x1));


// 환경 변수 초기화
          analogReference(INTERNAL);

          pinMode(9,OUTPUT);  // 1= solar down
          pinMode(5,OUTPUT);  // 1= sensor power on 
          pinMode(13,OUTPUT); // 1= ZigBee hibernation 
          pinMode(8,OUTPUT);  // run indicator
          pinMode(2,INPUT_PULLUP); //

          digitalWrite(9,LOW); // 태양전지 연결
          digitalWrite(5,LOW);
          digitalWrite(13,LOW);  //

// ZigBee 통신 설정, 38400
          Serial.begin(baudXBee);//Serial.begin(38400); // ZigBee port
          while (!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
          }
//test(1);
// 센서 통신 설정 및 연결 파악
          // set the data rate for the SoftwareSerial port
          mySerial.begin(9600);

          eead=0;
          EEPROM.write(eead,255);
          digitalWrite(5,HIGH); //avtivate sensor power
          delay(30);
          Find();
          eer();
          digitalWrite(5,LOW);
//       digitalWrite(8,HIGH);
//  test(1); 
// Master info send
          analogReference(DEFAULT);
           getTempVolt();
          analogReference(INTERNAL);
// test(2);          
                masterSend();
// test(3);
        //        delay(300);
                pd_setup();
        }
void loop(){


        loop1();
        
        digitalWrite(5,LOW);
        digitalWrite(13,HIGH);
        digitalWrite(8,HIGH);
        wait_pd();
        digitalWrite(8,LOW);
}

void loop1() 
{ 
        //160526
          if(Serial) Serial.end();
          Serial.begin(baudXBee);//Serial.begin(38400);
          while (!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
          }

          analogReference(DEFAULT);
           getTempVolt();
          analogReference(INTERNAL);
//              Serial.print("t = ");
//              Serial.print(coreTemp);
//              Serial.print(", volt = ");
//              Serial.print(currentVolt);
//              Serial.println("");
            
        //while(Serial.available()){}
        //delay(300);
        v2 = batteryVolt(); //float (analogRead(A3))/f200;
        v1 = solarVolt(); //float (analogRead(A2))/f200s;
        
        if (v2>lowerBatteryVolt) {
          sensorSend();
        }
}

void solarControl()
{
                  digitalWrite(9,LOW);
                     delay(50);
                    v2=batteryVolt(); //float (analogRead(A3))/f200;
                    v1=solarVolt(); //float (analogRead(A2))/f200s;

                    if (v2>upperBatteryVolt)
                    {
                            digitalWrite(9,HIGH); //  Serial.println("Solar Down");
                    }
                    else digitalWrite(9,LOW);
 
}

void masterSend()
{
    solarControl();  

    digitalWrite(13,LOW);
    delay(5);
      ATCmd("DB"); dbString = s;

      zbInfo="";
      ATCmd("MY"); zbInfo+=",MY:"+s;
      ATCmd("MP"); zbInfo+=",MP:"+s;
//      ATCmd("SH"); zbInfo+=",SH:"+s;
//      ATCmd("SL"); zbInfo+=",SL:"+s;
      ATCmd("DH"); zbInfo+=",DH:"+s;
      ATCmd("DL"); zbInfo+=",DL:"+s;
      ATCmd("%V"); zbInfo+=",%V:"+s;
      ATCmd("NI"); nameNI= s; // ss+=":"+s;

    ss=fs;                  //Serial.print(fs);
    ss += zbInfo;             //Serial.print(",");
    ss +=",SV:"+String(v1,2)+",BV:"+String(v2,2);
    // ss +=",CT:"+String(coreTemp,2);
    ss +=",CV:"+String(currentVolt,1);

    ss += ",DB:"+ dbString;
    ss += "," + nameNI;
    ss += ","+String(strCnt++);
    
    Serial.println(ss);
    delay(50);
    digitalWrite(13,HIGH);
}


void sensorSend()
{
  static uint8_t cntSendMast = 0;
  static uint16_t cntSensSend=0;
  String txZbee="";
  String nameZbee="";

  float humidity,temperature;
  
  cntSensSend ++; cntSendMast ++;
  
  if ( cntSendMast > 4 ) {
    cntSendMast = 0 ; masterSend(); return;
  } 
  solarControl();  
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  digitalWrite(13,LOW);
  ATCmd("NI"); nameZbee = s; // ss+=":"+s;
  txZbee += nameZbee;
  txZbee += ",TR:"+String(temperature,2);
  txZbee += ",HR:"+String(humidity,2);
  txZbee += ","+ String(cntSensSend);
  
  Serial.println(txZbee);

  delay(50);//100
  digitalWrite(13,HIGH);
}

/////////////////////////////////////////////////
//
//   Functions
//
/////////////////////////////////////////////////
float solarVolt()
{
  return float (analogRead(A2))/135.6;
}
float batteryVolt()
{
  return float (analogRead(A3))/230;
}

void ATCmd(String c)
{
          if (xBeeConn())
          {
                      c="AT"+c;
                      Serial.println(c);
                      Serial.setTimeout(500);
                      s=Serial.readStringUntil(13);
//                      s.replace("\r","");
          }
          if ( xBeeDisConn())
          {
//            Serial.print(c);
//            Serial.print("= ");
//            Serial.println(s);
          }
}
//////////////////////////////////////////

int xBeeConn()
  {
            Serial.flush();
            delay(5);
            Serial.print("+++");
          //  delay(100);  delay(1400);
          //  delay(20);
            return checkOK();
  }

int xBeeDisConn()
  {
        Serial.println("ATCN");
        return checkOK();
  }

int checkOK()
  {
          String reply="";
          char r=0;
          Serial.setTimeout(300);
          reply=Serial.readStringUntil(13);
          if( reply =="OK")
          {
              return 1;
          } else
          {
              return 0;
          }
   
  }
/////////////////////////////////////////////////////


void getTempVolt()
{

  // read first ADCL and ADCH;
   ADMUX = (_BV(REFS1)|_BV(REFS0)|_BV(MUX3)); //internal ref 1.1V and MUX=1000
   // ADMUX = (_BV(REFS1)|_BV(REFS0)| 8 ); //internal ref 1.1V and MUX=1000
   ADCSRA |= _BV(ADEN);      //enable ADC
   delay(20);         // wait for ADC stable
   ADCSRA |= _BV(ADSC);      //start ADC
   while(bit_is_set(ADCSRA,ADSC));

   coreTemp = (float)( ADCW ) ;
   coreTemp= tmpScale * coreTemp - tmpOffset;
   
   ADMUX = (_BV(REFS1)|_BV(REFS0)|7); //internal ref 1.1V and MUX=0111
   ADCSRA |= _BV(ADSC);      //start ADC
   while(bit_is_set(ADCSRA,ADSC));
   currentVolt = (float) ( ADCW ) ;            
   currentVolt = ( currentVolt / 1024) * 1.1 * 13.3 / 3.3;           
}




void test(int i){
      unsigned long t=millis();

      pinMode(7,1);
      while(millis()-t<i*1000)
      {
          digitalWrite(7,1);
          delay(1);
          digitalWrite(7,0);         
          delay(1);

      }
}


void wait_pd()
{
          digitalWrite(13,HIGH);//160516
          for (cnt=0; cnt<rep0; cnt++)
          { 
              //    Serial.println(millis());delay(10);
                  f_wdt = 0;
                  enterSleep();  //Serial.println(millis());

//                  digitalWrite(13,LOW);
//                  delay(10);
//                  digitalWrite(13,HIGH);
/*
                  Serial.end();
                  Serial.begin(9600);
                  while (!Serial) {
                    ; // wait for serial port to connect. Needed for native USB port only
                  }
                  pinMode(13,OUTPUT); 

                  //digitalWrite(5,LOW);
                  digitalWrite(13,LOW);
                  delay(100);
                  Serial.print("WDT cnt=");
                  Serial.println(cnt);
                  delay(100);
                  
                  digitalWrite(13,HIGH);
*/
          }
}


/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
      if(f_wdt == 0)
      {
        f_wdt=1;
      }
      else
      {
//        Serial.println("WDT Overrun!!!");
      }
}
/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
          set_sleep_mode(SLEEP_MODE_PWR_DOWN);//SLEEP_MODE_PWR_SAVE);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
          sleep_enable();
         
          /* Now enter sleep mode. */
          sleep_mode();
         
          /* The program will continue from here after the WDT timeout*/
          sleep_disable(); /* First thing to do is disable sleep. */
         
          /* Re-enable the peripherals. */
          power_all_enable();
}
/***************************************************
 *  Name:        setup
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Setup for the serial comms and the
 *                Watch dog timeout.
 *
 ***************************************************/
void pd_setup()
{
//        Serial.begin(9600);
//        Serial.println("Initialising...");
//        delay(100); //Allow for serial print to complete.
 //       pinMode(LED_PIN,OUTPUT);
        /*** Setup the WDT ***/
        /* Clear the reset flag. */
        MCUSR &= ~(1<<WDRF);
        /* In order to change WDE or the prescaler, we need to
         * set WDCE (This will allow updates for 4 clock cycles).
         */
        WDTCSR |= (1<<WDCE) | (1<<WDE);
        /* set new watchdog timeout prescaler value */
        WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
        /* Enable the WD interrupt (note no reset). */
        WDTCSR |= _BV(WDIE);
//        Serial.println("Initialisation complete.");
//        delay(100); //Allow for serial print to complete.
}
/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Main application loop.
 *
 ***************************************************/
void eewr(int i)
{
//        Serial.println();
//        Serial.print("EEwr");
//        Serial.println(eead);
        EEPROM.write(eead,i/256);
        eead++;
        EEPROM.write(eead,i%256);
        eead++;
        EEPROM.write(eead,255);
}


void eer()
{
          eead=0;
          while (EEPROM.read(eead)<255)
          {
            eead++;
            eead++;   
          }
 //         Serial.println(eead);
          een=eead/2;
 //         Serial.println(een);
        
        
          for (int i=0; i<een; i++)
          {
            eer0[i]=EEPROM.read(i*2)*256;
            eer0[i]+=EEPROM.read(i*2+1);
//            Serial.print(i);
//            Serial.print(":");
//            Serial.println(eer0[i]);
          }
//            Serial.println("End");
}


void Find()
{
//        Serial.println("[f]ind");
           
          eead=0;
          for (int i=1; i<999; i++)
          {
//          if (Serial.available())
//            if (Serial.read()=='E') i=9999;
           
            mySerial.print("F");
              if (i<1000) mySerial.print("0");
              if (i<100) mySerial.print("0");
              if (i<10) mySerial.print("0");
              mySerial.print(i);
              mySerial.write(13);
          //  delay(20);
          //  delay(10);
          //  delay(2);
            delay(5);
          //  delay(4);
//          Serial.setTimeout(200);
//          reply=Serial.readStringUntil(13);
          
          while (mySerial.available())
          {
            char cc;
            cc=mySerial.read();
//            Serial.write(cc);
            if (cc=='E') eewr(i);
          }
          
          }
//            Serial.println();
//            Serial.println("Find End");
}
