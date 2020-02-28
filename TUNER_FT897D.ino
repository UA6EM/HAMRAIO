*****************************************************************************/

#define SERIAL_ON       // Serial ON  if "//" Serial OFF
#define LCD_I2C
//#define DEBUG
#define YAESU

/**********************Arduino Pin Definition************************/

int pwm_L = 11;           // PWM (Enable) output pin for L motor
int up_L = 13;            // Direction Up (I1) output pin for L motor
int down_L  = 12;         // Direction Down (I2) output pin for L motor

int pwm_C = 10;            // PWM (Enable) output pin for C motor
int up_C = 8;             // Direction Up (I1) output pin for C motor
int down_C  = 9;          // Direction Down (I2) output pin for C motor

int ana_pin_L = 1;        // Analog input pin L motor position
int ana_pin_C = 0;        // Analog input pin C motor position

int ana_band = 2;         // Analog input pin for band selection switch
int ana_band2 = 6;        // Analog input pin for band selection switch

int ana_pin_fine_tune_C = 3;      // Analog input pin for fine tuning C motor PB switch

int pin_fine_tune_L_up = 7;      // Discrete input pin for fine tuning L motor PB switch
int pin_fine_tune_L_dwn = 6;      // Discrete input pin for fine tuning L motor PB switch

/* LCD circuit
 * LCD RS pin to digital pin 0
 * LCD Enable pin to digital pin 1
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)*/

/**********************Arduino Constant Definition*******************/

// Thresholds for C / L motor control
#define LOW_SPEED_THRES 20   // Low speed threshold
//#define LOW_SPEED_THRES 10   // Low speed threshold
#define TARGET_THRES_L 0     // Target positioning threshold
#define TARGET_THRES_C 0     // Target positioning threshold

// Commanded positions for L motor depending on band
#define POSN_CMD_L_160 640 // Commanded position L motor for 160m band
#define POSN_CMD_L_80 640  // Commanded position L motor for 80m band
#define POSN_CMD_L_40 640  // Commanded position L motor for 40m band
#define POSN_CMD_L_30 640  // Commanded position L motor for 30m band
#define POSN_CMD_L_20 184  // Commanded position L motor for 20m band
#define POSN_CMD_L_17 972  // Commanded position L motor for 17m band
#define POSN_CMD_L_15 373  // Commanded position L motor for 15m band
#define POSN_CMD_L_12 180  // Commanded position L motor for 12m band
#define POSN_CMD_L_10 160  // Commanded position L motor for 10m band

// Commanded positions for C motor depending on band
#define POSN_CMD_C_160 180 // Commanded position C motor for 160m band
#define POSN_CMD_C_80 180  // Commanded position C motor for 80m band
#define POSN_CMD_C_40 180  // Commanded position C motor for 40m band
#define POSN_CMD_C_30 180  // Commanded position C motor for 30m band
#define POSN_CMD_C_20 582  // Commanded position C motor for 20m band
#define POSN_CMD_C_17 361  // Commanded position C motor for 17m band
#define POSN_CMD_C_15 247  // Commanded position C motor for 15m band
#define POSN_CMD_C_12 277  // Commanded position C motor for 12m band
#define POSN_CMD_C_10 384  // Commanded position C motor for 10m band

#define RELE_160 0 // Rele (0 - OFF, 1 - ON) for 160m band
#define RELE_80 0  // Rele (0 - OFF, 1 - ON) for 80m band
#define RELE_40 0  // Rele (0 - OFF, 1 - ON) for 40m band
#define RELE_30 0  // Rele (0 - OFF, 1 - ON) for 30m band
#define RELE_20 1  // Rele (0 - OFF, 1 - ON) for 20m band
#define RELE_17 1  // Rele (0 - OFF, 1 - ON) for 17m band
#define RELE_15 1  // Rele (0 - OFF, 1 - ON) for 15m band
#define RELE_12 1  // Rele (0 - OFF, 1 - ON) for 12m band
#define RELE_10 1  // Rele (0 - OFF, 1 - ON) for 10m band

// PWM settings
// PWM 0 == Duty Cycle = 0%  255 == Duty Cycle = 100%
#define PWM_CMD_HIGH_SPEED_L 190 // ~75% duty cycle for high speed operation
#define PWM_CMD_LOW_SPEED_L  90  // ~30% duty cycle for low speed operation
#define PWM_CMD_HIGH_SPEED_C 190 // ~75% duty cycle for high speed operation
#define PWM_CMD_LOW_SPEED_C  90  // ~30% duty cycle for low speed operation

// Defines the number of loops before the LCD display will be updated
#define DISPLAY_LOOP_COUNTER_MAX 6

// EEPROM Pause 
#define PAUSE_EEPROM 100

// первичная инициализация EEPROM без нажатия клавиши по проверяемому флагу
uint8_t a73=88;  // для проверки первичной инициализации смените это значение на любое до 250
int eeprom_flag = 0;
int rom_flag = 0;
int eeprom_band = 20;
int rom_band = 2;
byte flag_band = 0;
uint8_t cycle = 8;
uint8_t  relay = 6;
uint8_t rele = 0;
uint8_t  button_relay = 7;     // кнопка реле
uint8_t  flag_button = 0;
volatile unsigned long r_millis = 0;
uint16_t recycle = 100;

/**********************Arduino Libraries*****************************/
#ifdef YAESU
#include <SoftwareSerial.h>
#include "FT897D.h"   
FT897D radio;  
volatile unsigned long myfreq = 0; 
volatile unsigned long cat_millis = 0; 
unsigned int cat_band = 0;
#define CATPIN 5
#endif


// Include the library code:
#include <EEPROM.h>

#ifdef LCD_I2C
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 20, 4); // A0,A1,A2 -HIGH
/*
 * A0 A1 A2 PCF8574A
 * 0  0  0  - 0x38
 * 1  0  0  - 0x39
 * 0  1  0  - 0x3A
 * 1  1  0  - 0x3B
 * 0  0  1  - 0x3C
 * 1  0  1  - 0x3D
 * 0  1  1  - 0x3E
 * 1  1  1  - 0x3F 
 * 
 */
#else
#include <LiquidCrystal.h>
// Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(0, 1, 5, 4, 3, 2);
#endif

/**********************Variable Declaration***************************/

int posn_cmd_L = 0;
int posn_cmd_C = 0;

boolean in_motion_L = false;
boolean in_motion_C = false;

boolean power_up = true;

int posn_L = 0;          // Input position (0..1000) i.e. (0..5V)  for L motor
int posn_L_0 = 0;        // Past values of input position
int posn_L_1 = 0;
int posn_L_2 = 0;
int posn_L_3 = 0;
int posn_L_4 = 0;
int posn_L_5 = 0;
int posn_L_6 = 0;

int posn_C = 0;          // Input position (0..1000) i.e. (0..5V)  for C motor
int posn_C_0 = 0;        // Past values of input position
int posn_C_1 = 0;
int posn_C_2 = 0;
int posn_C_3 = 0;
int posn_C_4 = 0;
int posn_C_5 = 0;
int posn_C_6 = 0;

int posn_diff_L = 0;
int posn_diff_C = 0;

int band = 0;
int band_0 = 0;        // Past values of band selection switch
int band_1 = 0;
int band_2 = 0;
int band_3 = 0;
int band_4 = 0;

int band2 = 0;
int band2_0 = 0;        // Past values of band selection switch
int band2_1 = 0;
int band2_2 = 0;
int band2_3 = 0;
int band2_4 = 0;

int band_sel = 0;
int band_sel_old = 0;

int fine_tune_C = 512;
int fine_tune_C_0 = 512;        // Past values of band selection switch
int fine_tune_C_1 = 512;
int fine_tune_C_2 = 512;
int fine_tune_C_3 = 512;
int fine_tune_C_4 = 512;

int display_loop_counter = DISPLAY_LOOP_COUNTER_MAX;

/***************** FUNCTION ******************/

void setRelay(){
  if (flag_button == 1 && digitalRead(button_relay) == 1){ // кнопка отжата
      delay(20);
      flag_button = 0;
  }
  if (flag_button == 0 && digitalRead(button_relay) == 0){ // кнопка нажата
      delay(20);
      flag_button = 1;
      digitalWrite(relay,!digitalRead(relay));
      //rele = digitalRead(relay);
  }
}

void setaRelay(){
  if (flag_button == 1 && millis()-r_millis >=200 && ((band <855) || (band > 890))){ // кнопка отжата
      delay(20);
      flag_button = 0;
  }
}


/**********************Arduino SETUP*********************************/

void setup()
{
  pinMode(pwm_L, OUTPUT);
  pinMode(up_L, OUTPUT);
  pinMode(down_L, OUTPUT);

  pinMode(pwm_C, OUTPUT);
  pinMode(up_C, OUTPUT);
  pinMode(down_C, OUTPUT);

  //pinMode(pin_fine_tune_L_up, INPUT_PULLUP); // Discrete input pin for fine tuning L motor PB switch
  pinMode(pin_fine_tune_L_dwn, OUTPUT); // Discrete input pin for fine tuning L motor PB switch

  #ifdef SERIAL_ON
  // Output on serial monitor
  // Open the serial port at 9600 bps:
  Serial.begin(9600);
  #endif  
  
  // Set up the LCD's number of columns and rows:
#ifdef LCD_I2C
 lcd.begin();
 #else
 lcd.begin(16, 2);
#endif

  // Initialization EEPROM
  for (uint8_t i = 0; i < 50; i ++) {
    fine_tune_C_0 = analogRead(ana_pin_fine_tune_C);

    fine_tune_C   = (fine_tune_C_0 + fine_tune_C_1 + fine_tune_C_2 + fine_tune_C_3 + fine_tune_C_4)/5;
    fine_tune_C_4 = fine_tune_C_3;
    fine_tune_C_3 = fine_tune_C_2;
    fine_tune_C_2 = fine_tune_C_1;
    fine_tune_C_1 = fine_tune_C_0;
  }

  if ((fine_tune_C >= 740 && fine_tune_C < 810 && fine_tune_C_1 >= 740 && fine_tune_C_1 < 810) || EEPROM.get(rom_flag,eeprom_flag)!=a73) {
    eeprom_flag = a73 ;
    delay(PAUSE_EEPROM);
    EEPROM.put(rom_flag,eeprom_flag);
    delay(PAUSE_EEPROM);
    EEPROM.put(rom_band,eeprom_band);   // Первично выбранный диапазон (20м)
 
    lcd.setCursor(0, 0);
    lcd.print("Init EEPROM");
    delay(PAUSE_EEPROM);
    EEPROM.put(10, POSN_CMD_L_10);
    EEPROM.put(110, POSN_CMD_C_10);
    EEPROM.put(210, RELE_10);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(12, POSN_CMD_L_12);
    EEPROM.put(112, POSN_CMD_C_12);
    EEPROM.put(212, RELE_12);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(15, POSN_CMD_L_15);
    EEPROM.put(115, POSN_CMD_C_15);
    EEPROM.put(215, RELE_15);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(17, POSN_CMD_L_17);
    EEPROM.put(117, POSN_CMD_C_17);
    EEPROM.put(217, RELE_17);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(20, POSN_CMD_L_20);
    EEPROM.put(120, POSN_CMD_C_20);
    EEPROM.put(220, RELE_20);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(30, POSN_CMD_L_30);
    EEPROM.put(130, POSN_CMD_C_30);
    EEPROM.put(230, RELE_30);
    lcd.setCursor(0, 1);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(40, POSN_CMD_L_40);
    EEPROM.put(140, POSN_CMD_C_40);
    EEPROM.put(240, RELE_40);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(80, POSN_CMD_L_80);
    EEPROM.put(180, POSN_CMD_C_80);
    EEPROM.put(280, RELE_80);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    EEPROM.put(160, POSN_CMD_L_160);
    EEPROM.put(260, POSN_CMD_C_160);
    EEPROM.put(360, RELE_160);
    lcd.print('.');
    delay(PAUSE_EEPROM);
    lcd.print(" OK");
    delay(2000);
    lcd.clear();
  }

  // Display intro
  lcd.setCursor(0, 0);
  lcd.print(" Tuner  Control ");
  lcd.setCursor(0, 1);
  lcd.print("v.26 2020  UA6EM");

  delay(3000);

  lcd.clear();
  
#ifdef YAESU
radio.begin(4800);
cat_millis = millis();
pinMode(CATPIN,INPUT_PULLUP);  // пин потом перенести в дефайны
#endif

} //END SETUP

/**********************Arduino LOOP**********************************/

void loop()
{
#ifdef YAESU
//if(millis() - cat_millis >= recycle){
//Serial.print("FREQ = ");
//Serial.println(radio.getFreqMode());
/* *
    Radio CAT port GND -> Arduino GND
    Radio CAT port TX  -> Arduino pin 3
    Radio CAT port RX  -> Arduino pin 2
/* */
cat_millis = millis();
//}
#endif
  
  uint8_t y=0;
  do{
  band_0 = analogRead(ana_band); // Read position band selection switch
  band2_0 = analogRead(ana_band2); // Read position band selection switch

  if (power_up == true)
  {
   band_4 = band_0;
   band_3 = band_0;
   band_2 = band_0;
   band_1 = band_0;
   band2_4 = band_0;
   band2_3 = band_0;
   band2_2 = band_0;
   band2_1 = band_0;
  }

  // Band sensor filter
  band = (band_0 + band_1 + band_2 + band_3+ band_4)/5;
  band_4 = band_3;
  band_3 = band_2;
  band_2 = band_1;
  band_1 = band_0;
  band2 = (band2_0 + band2_1 + band2_2 + band2_3+ band2_4)/5;
  band2_4 = band2_3;
  band2_3 = band2_2;
  band2_2 = band2_1;
  band2_1 = band2_0;
  y++;
  } while(y < cycle);
  y = 0;

  if (((band >= 35) && (band < 70)) && ((band_1 >= 35) && (band_1 < 70)))   // 10m band selected
  {
   band_sel = 10;
   EEPROM.update(rom_band,band_sel); 
   flag_band = 1;
  } else
  if (((band >= 75) && (band < 110)) && ((band_1 >= 75) && (band_1 < 110)))   // 12m band selected    
   {
     band_sel = 12;
      EEPROM.put(rom_band,band_sel); 
       flag_band = 1;
    } else
       if (((band >= 115) && (band < 150)) && ((band_1 >= 115) && (band_1 < 150)))  // 15m band selected      
        {
       band_sel = 15;
        EEPROM.put(rom_band,band_sel); 
         flag_band = 1;
      } else
       if (((band >= 180) && (band < 220)) && ((band_1 >= 180) && (band_1 < 220)))   // 17m band selected
        {
         band_sel = 17;
          EEPROM.put(rom_band,band_sel); 
           flag_band = 1;
        } else
        if (((band >= 290) && (band < 330)) && ((band_1 >= 290) && (band_1 < 330)))    // 20m band selected
           {
           band_sel = 20;
            EEPROM.put(rom_band,band_sel); 
             flag_band = 1;
          } else
          if (((band >= 390) && (band < 450)) && ((band_1 >= 390) && (band_1 < 450)))    // 30m band selected
             {
             band_sel = 30;
              EEPROM.put(rom_band,band_sel); 
               flag_band = 1;
            } else
            if (((band >= 530) && (band < 580)) && ((band_1 >= 530) && (band_1 < 580)))    // 40m band selected
               {
               band_sel = 40;
               EEPROM.put(rom_band,band_sel); 
               flag_band = 1;
              } else
              if (((band >= 675) && (band < 720)) && ((band_1 >= 675) && (band_1 < 720)))    // 80m band selected
                 {
                 band_sel = 80;
                 EEPROM.put(rom_band,band_sel); 
                 flag_band = 1;
                } else
                if (((band >= 770) && (band < 810)) && ((band_1 >= 770) && (band_1 < 810)))    // 160m band selected
                   {
                   band_sel = 160;
                   EEPROM.put(rom_band,band_sel); 
                   flag_band = 1;
                 } else
                   if (((band >= 855) && (band < 900)) && (flag_button == 0) &&((band_1 >= 855) && (band_1 < 900)))    //  RELE is ON 
                    {
                    digitalWrite(relay,!digitalRead(relay));
                    r_millis = millis();
                    flag_button = 1;                    
                    } else
                      if (((band >= 900) && (band < 946)) && ((band_1 >= 900) && (band_1 < 946)))   // Save Memory EEPROM
                      {
                        /* */
                 lcd.setCursor(10, 1);
                 lcd.print("M+");
                 EEPROM.put(band_sel, posn_L);
                 delay(PAUSE_EEPROM);
                 EEPROM.put((band_sel + 100), posn_C); 
                 delay(PAUSE_EEPROM);
                 EEPROM.update(rom_band,band_sel);
                 delay(PAUSE_EEPROM);
                 EEPROM.get(band_sel, posn_cmd_L);
                 EEPROM.get((band_sel + 100), posn_cmd_C);
                 rele = digitalRead(relay);
                 EEPROM.put((band_sel + 200), rele);     
                 delay(2000);
                 lcd.setCursor(10, 1);
                 lcd.print("  ");
                 /* */
                 }else
                  if (((band >= 946) && (band < 1023)) && ((band_1 >= 946) && (band_1 < 1023)))   // STOP
                      {
                        // для функции стоп
                      }

  setaRelay();   // обработать кнопку реле
  rele = digitalRead(relay);

  if (band_sel == 0)
  {
     EEPROM.get(rom_band,band_sel); 
     EEPROM.get(band_sel, posn_cmd_L);
     EEPROM.get((band_sel + 100), posn_cmd_C);
     EEPROM.get((band_sel + 200), rele);
     digitalWrite(pin_fine_tune_L_dwn, rele);
     power_up = false;
  }
   if (band_sel != 0 && flag_band) {
   flag_band = 0; 
   EEPROM.get(band_sel, posn_cmd_L);
   EEPROM.get((band_sel + 100), posn_cmd_C);
   EEPROM.get((band_sel + 200), rele);
   digitalWrite(pin_fine_tune_L_dwn, rele);
   }
   
  if ((band_sel != band_sel_old) && (power_up == false))
  {
    in_motion_L = true;
    in_motion_C = true;
  }
  

  band_sel_old = band_sel;
  power_up = false;

  uint8_t x = 0;
  do{ 
  posn_L_0 = analogRead(ana_pin_L); // Read position L motor

  // Map position sensor value to 0...1000
  posn_L_0 = map(posn_L_0, 0, 1023, 0, 1000);

  // Position sensor filter
  posn_L = (posn_L_0 + posn_L_1 + posn_L_2 + posn_L_3 + posn_L_4 + posn_L_5 + posn_L_6)/7;
  posn_L_6 = posn_L_5;
  posn_L_5 = posn_L_4;
  posn_L_4 = posn_L_3;
  posn_L_3 = posn_L_2;
  posn_L_2 = posn_L_1;
  posn_L_1 = posn_L_0;
  x++;
  } while(x<cycle);
  x=0;

  // Caution C Poti rotates only 1/2 turn, i.e. max. voltage is 4.5V. This related to 5V results in a factor 1.112
  // Removed the value for recalculating the rotation of the capacitor. For R2ARR modification.
  do {
  posn_C_0 = /* 1.11 * */analogRead(ana_pin_C); // Read position C motor

  // Map position sensor value to 0...1000
  posn_C_0 = map(posn_C_0, 0, 1023, 0, 1000);

  // Position sensor filter
  posn_C = (posn_C_0 + posn_C_1 + posn_C_2 + posn_C_3 + posn_C_4 + posn_C_5 + posn_C_6)/7;
  posn_C_6 = posn_C_5;
  posn_C_5 = posn_C_4;
  posn_C_4 = posn_C_3;
  posn_C_3 = posn_C_2;
  posn_C_2 = posn_C_1;
  posn_C_1 = posn_C_0;
  x++;
  } while(x<cycle);
  x=0;

  // Compute position difference between current position and command
  posn_diff_L = posn_cmd_L - posn_L;
  posn_diff_C = posn_cmd_C - posn_C;

  // Motion for C motor and L motor requested (motion of C motor has priority over motion of L motor) or
  // Motion only for C motor requested
  if (((in_motion_C == true) && (in_motion_L == true)) || ((in_motion_C == true) && (in_motion_L == false)))
  {
   // Reset direction and PWM output of L motor (for safety)
   digitalWrite(up_L, LOW);
   digitalWrite(down_L, LOW);
   analogWrite(pwm_L, 0);

   // Control motion of C motor depending on C motor position
   if (posn_diff_C > LOW_SPEED_THRES)
   {
      digitalWrite(up_C, HIGH); // Set direction of C motor to Up
      digitalWrite(down_C, LOW);
      analogWrite(pwm_C, PWM_CMD_HIGH_SPEED_C);  // Set PWM output to High speed
      lcd.setCursor(10, 1);
      lcd.print(">>");
   }
   if (posn_diff_C < -LOW_SPEED_THRES)
   {
      digitalWrite(up_C, LOW); // Set direction of C motor to Down
      digitalWrite(down_C, HIGH);
      analogWrite(pwm_C, PWM_CMD_HIGH_SPEED_C);  // Set PWM output to High speed
      lcd.setCursor(10, 1);
      lcd.print("<<");
   }
   if ((posn_diff_C <= LOW_SPEED_THRES) && (posn_diff_C > TARGET_THRES_C))
   {
      digitalWrite(up_C, HIGH); // Set direction of C motor to Up
      digitalWrite(down_C, LOW);
      analogWrite(pwm_C, PWM_CMD_LOW_SPEED_C);  // Set PWM output to Low speed
      lcd.setCursor(10, 1);
      lcd.print("> ");
   }
   if ((posn_diff_C >= -LOW_SPEED_THRES) && (posn_diff_C < -TARGET_THRES_C))
   {
      digitalWrite(up_C, LOW); // Set direction of C motor to Down
      digitalWrite(down_C, HIGH);
      analogWrite(pwm_C, PWM_CMD_LOW_SPEED_C);  // Set PWM output to Low speed
      lcd.setCursor(10, 1);
      lcd.print("< ");
   }
   if ((posn_diff_C <= TARGET_THRES_C) && (posn_diff_C >= -TARGET_THRES_C))
   {
      digitalWrite(up_C, LOW); // Reset direction of C motor
      digitalWrite(down_C, LOW);
      analogWrite(pwm_C, 0);  // Reset PWM output
      lcd.setCursor(10, 1);
      lcd.print("  ");
      in_motion_C = false;
   }
  }
  else
    // Motion only for L motor requested
    if ((in_motion_C == false) && (in_motion_L == true))
    {
     // Reset direction and PWM output of C motor (for safety)
     digitalWrite(up_C, LOW);
     digitalWrite(down_C, LOW);
     analogWrite(pwm_C, 0);

     // Control motion of L motor depending on L motor position
     if (posn_diff_L > LOW_SPEED_THRES)
     {
        digitalWrite(up_L, HIGH); // Set direction of L motor to Up
        digitalWrite(down_L, LOW);
        analogWrite(pwm_L, PWM_CMD_HIGH_SPEED_L);  // Set PWM output to High speed
        lcd.setCursor(4, 1);
        lcd.print(">>");
     }
     if (posn_diff_L < -LOW_SPEED_THRES)
     {
        digitalWrite(up_L, LOW); // Set direction of L motor to Down
        digitalWrite(down_L, HIGH);
        analogWrite(pwm_L, PWM_CMD_HIGH_SPEED_L);  // Set PWM output to High speed
        lcd.setCursor(4, 1);
        lcd.print("<<");
     }
     if ((posn_diff_L <= LOW_SPEED_THRES) && (posn_diff_L > TARGET_THRES_L))
     {
        digitalWrite(up_L, HIGH); // Set direction of L motor to Up
        digitalWrite(down_L, LOW);
        analogWrite(pwm_L, PWM_CMD_LOW_SPEED_L);  // Set PWM output to Low speed
        lcd.setCursor(4, 1);
        lcd.print(" >");
     }
     if ((posn_diff_L >= -LOW_SPEED_THRES) && (posn_diff_L < -TARGET_THRES_L))
     {
        digitalWrite(up_L, LOW); // Set direction of L motor to Down
        digitalWrite(down_L, HIGH);
        analogWrite(pwm_L, PWM_CMD_LOW_SPEED_L);  // Set PWM output to Low speed
        lcd.setCursor(4, 1);
        lcd.print(" <");
     }
     if ((posn_diff_L <= TARGET_THRES_L) && (posn_diff_L >= -TARGET_THRES_L))
     {
        digitalWrite(up_L, LOW); // Reset direction of L motor
        digitalWrite(down_L, LOW);
        analogWrite(pwm_L, 0);  // Reset PWM output
        lcd.setCursor(4, 1);
        lcd.print("  ");
        in_motion_L = false;
     }
    }
    else
      // No motion requested
      {
       // Reset direction and PWM output of L motor (for safety)
       digitalWrite(up_L, LOW);
       digitalWrite(down_L, LOW);
       analogWrite(pwm_L, 0);

       // Reset direction and PWM output of C motor (for safety)
       digitalWrite(up_C, LOW);
       digitalWrite(down_C, LOW);
       analogWrite(pwm_C, 0);

      // Fine tune is allowed only, neither C motor nor L motor motion is requested
       do{
       fine_tune_C_0 = analogRead(ana_pin_fine_tune_C); // Read position of fine tuning C motor PB switch
        
       // Fine tuning C motor PB switch filter
       fine_tune_C = (fine_tune_C_0 + fine_tune_C_1 + fine_tune_C_2 + fine_tune_C_3+ fine_tune_C_4)/5;
       fine_tune_C_4 = fine_tune_C_3;
       fine_tune_C_3 = fine_tune_C_2;
       fine_tune_C_2 = fine_tune_C_1;
       fine_tune_C_1 = fine_tune_C_0;
       x++;
       } while (x < cycle);
       x = 0;

       if (((fine_tune_C >= 0) && (fine_tune_C < 250)) && ((fine_tune_C_1 >= 0) && (fine_tune_C_1 < 250)))   // Fine tuning C motor PB switch commands the C motor to Up
       {
        digitalWrite(up_C, HIGH); // Set direction of C motor to Up
        digitalWrite(down_C, LOW);
        analogWrite(pwm_C, PWM_CMD_HIGH_SPEED_C);  // Set PWM output to High speed
        lcd.setCursor(11, 1);
        lcd.print(">");

        delay(100);  // Allow C motor to move (fine tune)

        digitalWrite(up_C, LOW); // Reset direction of C motor
        digitalWrite(down_C, LOW);
        analogWrite(pwm_C, 0);  // Reset PWM output
        lcd.setCursor(11, 1);
        lcd.print(" ");

       } else
         if (((fine_tune_C >= 251) && (fine_tune_C < 416)) && ((fine_tune_C_1 >= 251) && (fine_tune_C_1 < 416)))    // Fine tuning C motor PB switch commands the C motor to Down
         {
          digitalWrite(up_C, LOW); // Set direction of C motor to Down
          digitalWrite(down_C, HIGH);
          analogWrite(pwm_C, PWM_CMD_HIGH_SPEED_C);  // Set PWM output to High speed
          lcd.setCursor(11, 1);
          lcd.print("<");

          delay(100);  // Allow C motor to move (fine tune)

          digitalWrite(up_C, LOW); // Reset direction of C motor
          digitalWrite(down_C, LOW);
          analogWrite(pwm_C, 0);  // Reset PWM output
          lcd.setCursor(11, 1);
          lcd.print(" ");

         } else
           if (((fine_tune_C >= 416) && (fine_tune_C < 603)) && ((fine_tune_C_1 >= 416) && (fine_tune_C_1 < 603)))    // Fine tuning L motor PB switch commands the L motor to Up
           {
            digitalWrite(up_L, HIGH); // Set direction of L motor to Up
            digitalWrite(down_L, LOW);
            analogWrite(pwm_L, PWM_CMD_HIGH_SPEED_L);  // Set PWM output to High speed
            lcd.setCursor(4, 1);
            lcd.print(">");

            delay(200);  // Allow L motor to move (fine tune)

            digitalWrite(up_L, LOW); // Reset direction of L motor
            digitalWrite(down_L, LOW);
            analogWrite(pwm_L, 0);  // Reset PWM output
            lcd.setCursor(4, 1);
            lcd.print(" ");          
           } else
             if (((fine_tune_C >= 603) && (fine_tune_C < 740)) && ((fine_tune_C_1 >= 603) && (fine_tune_C_1 < 740)))    // Fine tuning L motor PB switch commands the L motor to Dwn
             {
              digitalWrite(up_L, LOW); // Set direction of L motor to Down
              digitalWrite(down_L, HIGH);
              analogWrite(pwm_L, PWM_CMD_HIGH_SPEED_L);  // Set PWM output to High speed
              lcd.setCursor(4, 1);
              lcd.print("<");

              delay(200);  // Allow L motor to move (fine tune)

              digitalWrite(up_L, LOW); // Reset direction of L motor
              digitalWrite(down_L, LOW);
              analogWrite(pwm_L, 0);  // Reset PWM output
              lcd.setCursor(4, 1);
              lcd.print(" ");
              
             } else
               if (((fine_tune_C >= 740) && (fine_tune_C < 810)) && ((fine_tune_C_1 >= 740) && (fine_tune_C_1 < 810)))    // Save EEPROM
               {
                lcd.setCursor(10, 1);
                lcd.print("M+");
                EEPROM.put(band_sel, posn_L);
                EEPROM.put((band_sel + 100), posn_C);
                EEPROM.put((band_sel + 200), rele);
                delay(2000);
                lcd.setCursor(10, 1);
                lcd.print("  ");
                 }
      }

  delay(30);

  #ifdef SERIAL_ON
  // Output on serial monitor

  Serial.print(band_sel);
  Serial.print("m");
  Serial.print(" aL=");
  Serial.print(posn_L);
  Serial.print(" cmdL=");
  Serial.print(posn_cmd_L);
  Serial.print(" aC=");
  Serial.print(posn_C);
  Serial.print(" cmdC=");
  Serial.println(posn_cmd_C);
  #endif

  if (display_loop_counter >= DISPLAY_LOOP_COUNTER_MAX)
  {

  // Print a fixed message to the LCD.
  // lcd.setCursor(8, 0);
  // lcd.print("m");

  // set the cursor to column 6, line 0
  lcd.setCursor(6, 0);
  lcd.print("    ");
  lcd.setCursor(6, 0);
  // print band_sel
  lcd.print(band_sel);
  lcd.print("m");
  
  // Clear specific column, line on LCD.
  lcd.setCursor(3, 0);
  lcd.print(" ");

  // set the cursor to column 0, line 0
  lcd.setCursor(0, 0);
  // clear
  lcd.print("    ");
  // re-position cursor
  lcd.setCursor(0, 0);
  // print posn_cmd_L
  lcd.print(posn_cmd_L);

  // set the cursor to column 0, line 1
  lcd.setCursor(0, 1);
  // clear
  lcd.print("    ");
  // re-position cursor
  lcd.setCursor(0, 1);
  // print posn_L
  lcd.print(posn_L);

  // set the cursor to column 12, line 0
  lcd.setCursor(12, 0);
  // clear
  lcd.print("    ");
  // re-position cursor
  lcd.setCursor(12, 0);
  // print posn_cmd_C
  lcd.print(posn_cmd_C);

  // set the cursor to column 12, line 1
  lcd.setCursor(12, 1);
  // clear
  lcd.print("    ");
  // re-position cursor
  lcd.setCursor(12, 1);
  // print posn_C
  lcd.print(posn_C);

 // set the cursor to column 7, line 1
  lcd.setCursor(7, 1);
  if (rele)
    // print Rele ON
    lcd.print('R');
  else
    // clear
    lcd.print(' ');

  display_loop_counter = 0;
  } else
  {
    display_loop_counter ++;
  }
  
// ******* Обработчик САТ интерфейса *******
#ifdef YAESU
if(!digitalRead(CATPIN)){
    myfreq = radio.getFreqMode();
     myfreq = myfreq/100000;
     if(myfreq==1){
     cat_band = 160;
     } else
       if(myfreq==3){
       cat_band = 80;  
       } else
         if(myfreq==7){
         cat_band = 40;  
         } else
           if(myfreq==10){
           cat_band = 30;  
           } else
             if(myfreq==14){
             cat_band = 20;  
             } else
               if(myfreq==18){
               cat_band = 17;  
               } else
                 if(myfreq==21){
                 cat_band = 15;  
                 } else
                   if(myfreq==24){
                   cat_band = 12;  
                   } else
                     if(myfreq >= 28){
                     cat_band = 10;  
                     } 

if(cat_band != band_sel){
//   Здесь принятие решения кому отдать первенство, САТ или кнопкам
  }
 }
#endif 
}

/**********************Arduino Functions*****************************/



/************************End****************************************/
