#include <LedControl.h>

#include <LiquidCrystal.h>

/*
 * ADXL345 library, DS1302_RTC library,liquidCrystal I2C library must be installed
 */
 // CONNECTIONS:
// DS1302 CLK/SCLK --> 5
// DS1302 DAT/IO --> 4
// DS1302 RST/CE --> 2
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND

//led matrix pin
// din = 12;
//clk = 11;
// cs = 10;

//speaker -->A1;

//Accelerometer(I2C address : 0x53)
//ADXL345 3.3V -->3.3V
//ADXL345 GDN -->GND
//ADXL345 SCL -->SCL
//ADXL345 SDA -->SDA

//LCD (I2C address : 0x27)
//LCD 5V -->5V
//LCD GDN -->GND
//LCD SCL -->SCL
//LCD SDA -->SDA
#include <Wire.h>
#include <ADXL345.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>
//#include <LiquidCrystal_I2C.h> //for liquidCrystal I2C
//for liquidCrystal
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif
//newest earthquake info
typedef struct quake_inf{
  RtcDateTime q_time;
  int max_jindo;
}quake_info;

quake_info newest_quake;
//setup original accel
double init_ax=0;
double init_ay=0;
double init_az=0;
int max_jindo = 0;

//ADXL345 class object
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

//DS1302 class object
ThreeWire myWire(4,5,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

RtcDateTime quake_time;
unsigned long detect_time;

//liquidCrystal class
//LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
//lcd(Rs,E,4,5,6,7)
LiquidCrystal lcd(44,45,46,47,48,49);
//
RtcDateTime previous;
//


LedControl lc = LedControl(12,11,10,1); //din clk cs
//matrix data
byte row_data[8] = {0b11111111,0b11110000,0b0,0b0,0b0,0b0,0b0,0b0};
byte a[8] = { B10000001, B01000010, B00100100, B00011000, B00011000, B00100100, B01000010, B10000001};


int info_output = 0;
/*
 * @brief setup adxl345(this code use example code)
 */
void setup_adxl(){
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}
/*
 * @brief setup ds1302 and RTC(this code is from example code)
 */
void setup_ds1302(){
  Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    //printDateTime(compiled);
    Serial.println();

    if (!Rtc.IsDateTimeValid()) 
    {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);
    }

    if (Rtc.GetIsWriteProtected())
    {
        Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }

    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }
}
/*
 * @brief get average xyz original acceleration and put it to init_x, init_y, init_z
 */
void set_initXYZ(){
  int i;
  double init_xyz[3];
  for(i=0;i<60;++i){
    adxl.getAcceleration(init_xyz);
    init_ax+=init_xyz[0];
    init_ay+=init_xyz[1];
    init_az+=init_xyz[2];
  }
  init_ax/=60;
  init_ay/=60;
  init_az/=60;
}
/*
 * @brief get acceleration and calculate difference
 * @param accel_info {x_gal,y_gal,z_gal,xy_gal,xyz_gal,jindo}
 */
void get_realAccel(double *accel_info){
  double xyz[3];
  double ax,ay,az;
  adxl.getAcceleration(xyz);
  ax = xyz[0];//unit : g
  ay = xyz[1];
  az = xyz[2];
  double x_gal = (ax-init_ax)*100;//unit : gal
  double y_gal = (ay-init_ay)*100;
  double z_gal = abs((az-init_az))*100;

  double xy_gal = hypot(x_gal,y_gal);
  double xyz_gal = hypot(xy_gal,z_gal);
  //detect interrupt
  accel_info[0] = x_gal; // differnce of x_gal
  accel_info[1] = y_gal;
  accel_info[2] = z_gal;
  accel_info[3] = xy_gal;
  accel_info[4] = xyz_gal;
  accel_info[5] = 0.0;
}
/*
 * @brief get jindo info.
 * @param xyz_acceleration, unit : gal
 */
int get_jindo(double g){
  int jindo=1;
  if(0<=g && g<0.07)
    jindo = 1;
  else if(0.07<=g && g<0.23)
    jindo = 2;
  else if(0.23<=g && g<0.76)
    jindo = 3;
  else if(0.76<=g && g<2.56)
    jindo = 4;
  else if(2.56<=g && g<6.86)
    jindo = 5;
  else if(6.86<=g && 14.73)
    jindo = 6;
  else if(14.73<=g && g<31.66)
    jindo = 7;
  else if(31.66<=g && g<68.01)
    jindo = 8;
  else if(68.01<=g && g<146.14)
    jindo = 9;
  else if(146.14<=g && g<314)
    jindo = 10;
  else if(g>=314)
    jindo = 11;
  else
    jindo = 1;

  return jindo;
}
/*
 * @brief earthquake detect alarm
 */
void alarm(){
  //speakerpin = A1
  tone(A1,500,100);
  noTone(A1);
}
/*
 * @brief detect earthquake from accel_info. if earthquake detected, get time.
 *        if quake is gone, show time and jindo on LCD.
 */
int detect_earthquake(double *accel_info){
  float z_gal = accel_info[2];
  float xy_gal = accel_info[3];
  float xyz_gal = accel_info[4];
  byte interrupts = adxl.getInterruptSource();
  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
    int jindo = get_jindo(xyz_gal);
    accel_info[5] = (double)jindo;
    if(max_jindo<jindo){
      max_jindo = jindo;
      quake_time = Rtc.GetDateTime();
      detect_time = millis();
    }
    Serial.println("Vibration_detected : ");
    alarm();
    Serial.print("xy_gal : ");
    Serial.println(xy_gal,5);
    Serial.print("z_gal : ");
    Serial.println(z_gal,5);
    Serial.print("xyz_gal : ");
    Serial.println(xyz_gal,5);
    Serial.print("jindo : ");
    Serial.println(jindo);
    Serial.print("Time : ");
    Serial.print(String(quake_time.Month()) + "/" + String(quake_time.Day()) + "/" + String(quake_time.Year()) + " " + String(quake_time.Hour()) +":" + String(quake_time.Minute()) + ":" + String(quake_time.Second()));
    Serial.println();
    Serial.println("**********************");
    
  }
  if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
    unsigned long now = millis();
    if(now - detect_time > 10000 && info_output == 0){
      newest_quake.q_time = quake_time;
      newest_quake.max_jindo = max_jindo;
      max_jindo = 0;

      //time_jindo_print(newest_quake.q_time,newest_quake.max_jindo,2,9,3,14);//fix please
      Serial.println("newest quake info update::");
      Serial.print(String(newest_quake.q_time.Month()) + "/" + String(newest_quake.q_time.Day()) + "/" + String(newest_quake.q_time.Year()) + " " + String(newest_quake.q_time.Hour()) +":" + String(newest_quake.q_time.Minute()) + ":" + String(newest_quake.q_time.Second()));
      Serial.println();
      Serial.println("jindo : " + String(newest_quake.max_jindo));
      Serial.println("******************");
      info_output = 1;
    }
      
  }
  return (int)accel_info[5];
}
/*
 * @brief matrix print from jindo data. this code refer arduino book.
 */
void matrix_print(int jindo)
{
  byte height;
  //byte col_data[8] = {0x01, 0x02,0x04,0x08,0x10,0x20,0x40,0x80};
  if(jindo ==0){
    height = 0b00000000;
    lc.setRow(0,0,height);
  }
  else if(jindo == 1 ){
    height = 0b00000001;
    lc.setRow(0,0,height);
  }
  else if(jindo == 2){
    height = 0b00000010;
    lc.setRow(0,0,height);
  }
  else if(jindo == 3){
    height = 0b00000100;
    lc.setRow(0,0,height);
  }
  else if(jindo == 4){
    height = 0b00001000;
    lc.setRow(0,0,height);
  }
  else if(jindo == 5){
    height = 0b00010000;
    lc.setRow(0,0,height);
  }
  else if(jindo == 6){
    height = 0b00100000;
    lc.setRow(0,0,height);
  }
  else if(jindo == 7){
    height = 0b01000000;
    lc.setRow(0,0,height);
  }
  else if(jindo >=8){
    height = 0b10000000;
    lc.setRow(0,0,height);
  }
  else{
    height = 0b00000000;
    lc.setRow(0,0,height);
  }

  int i;
  /*
  for(i=0;i<6;++i){
    row_data[i] = row_data[i+1];
  }
  row_data[7] = height;
  for (int row = 0; row < 8; row++)
  {
    lc.setRow(0, row, row_data[row]);  // 도트매트릭스의 LED를 행단위로 위부터 켭니다.
    delay(25);
  }
  */
}
/*
 * @brief setup LCD. this code from library
 */
void setup_lcd(){
  //lcd.init();        //for liquidcrystal I2C
  //lcd.backlight();  //for liquid Crystal I2C
  lcd.begin(16,2); //for liquidcrystal
  lcd.home();
  lcd.print("time: ");
  lcd.setCursor(0,1);
  lcd.print("int.: ");
  lcd.setCursor(0,2);
  lcd.print("latest: ");
  lcd.setCursor(0,3);
  lcd.print("latest int.: ");
}
/*
 * @brief time and jindo print on LCD
 */
void time_jindo_print(RtcDateTime now, int jindo, int x1, int y1, int x2, int y2){
  lcd.setCursor(x1,y1);//for time print
  lcd.print(String(now.Month()));
  lcd.print(String("/"));
  lcd.print(String(now.Day()));
  lcd.print(String("/"));
  lcd.print(String(int(now.Year())/100));
  lcd.print(String(" "));
  lcd.print(String(now.Hour()));
  lcd.print(String(":"));
  lcd.print(String(now.Minute()));
  lcd.print(String(":"));
  lcd.print(String(now.Second()));
  lcd.setCursor(x2,y2);//for jindo print
  lcd.print(String(jindo));
}
void setup(){
  pinMode(A1,OUTPUT);//for speaker
  Serial.begin(9600);
  setup_adxl();
  setup_ds1302();
  //setup init_xyz
  set_initXYZ();
  setup_lcd();
  //setup newest_quake info
  newest_quake.q_time = Rtc.GetDateTime();
  newest_quake.max_jindo = 0;

  //setup previous RtcDateTime
  previous = Rtc.GetDateTime();


  lc.shutdown(0,false);
  lc.setIntensity(0,2);
  lc.clearDisplay(0);
}
 

void loop(){
    RtcDateTime now = Rtc.GetDateTime();
    int jindo=0;
    double accel_info[6];//x_gal,y_gal,z_gal,xy_gal,xyz_gal,jindo
    get_realAccel(accel_info);//jindo = 0;
    jindo = detect_earthquake(accel_info);
    time_jindo_print(now,jindo,0,0,12,1);
    matrix_print(jindo);
    byte height;
  
  delay(500);
 
}
