#include <Arduino.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <Wire.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);

//    The pins
#define pulse_input_pin 2//The infrared output
#define battery_voltmeter_pin A3//Battery voltage
#define battery_charge_pin A2//Battery charge signal (5V on this pin if the battery is charging)
#define button1_pin 4//Button up
#define button2_pin 7//Button down
#define button3_pin 8//Button enter
#define trigger_pin 12//Trigger

//Min and max voltage values for the battery, these might need to be changed slightly to compensate for the voltage devider imperfections
#define battery_voltage_min 3.3
#define battery_voltage_max 4.17

//Battery level dampener settings. Using internal analog reference makes the battery voltage readings very unstable, so a dampener is needed.
//This part of the code still needs work.
#define bat_lvl_damp_strength 0.1
#define bat_lvl_damp_interval 10
#define bat_lvl_damp_max 2
#define bat_lvl_sample_rate 500

//Debouce delay (in ms) for all of the buttons
#define debounceDelay 50

//Other variables, nothing to change here
float cur_bat_voltage = 0;
float bat_perc = 0;
float bat_perc_damp = 0;
float bat_chrg_anim = 0;
unsigned long bat_anim_timer = 0;
unsigned long bat_blink_timer = 0;
bool bat_blink_state = false;
float bat_lvl_damp = 0;
float bat_lvl_damp_previus = 0;
unsigned long bat_lvl_damp_timer = 0;
unsigned long bat_lvl_sample_timer = 0;
float bat_lvl_samples[10];
bool bat_samples_empty = true;
int bat_samples_taken = 0;

unsigned long edit_blink_timer = 0;
bool edit_blink_state = false;

bool readingRPM = false;
int curPage = 0;
int subLevel = 0;
bool addToMemory = false;
unsigned long addedToMemoDisplayTimer = 0;

int counter = 0;
float rpm = 0;
float maxRpm = 0;
float minRpm = 999999;
float revs = 0;
int ppr = 5;
unsigned long lastPulse = 0;

int presses = 0;
bool button1State = false;
bool button2State = false;
bool button3State = false;
bool button3HoldState = false;
bool lastTriggerState = false;
unsigned long lastButton1OffState = 0;
unsigned long lastButton2OffState = 0;
unsigned long lastButton3OffState = 0;
unsigned long lastButton3HoldOffState = 0;
unsigned long lastTriggerStateTimer = 0;

unsigned long button3Timer;
bool lastButton3State = false;
bool bttn3StateDeb = false;
bool lastBttn3StateDeb = false;
unsigned long bttn3DebTimer;
bool bttn3PressReady = false;
bool bttn3LongPressComplete = false;

int memorySlotsTaken = 0;
int curMemoryValue = 0;//Current 1, Max 1, Min 1, Current 2, Max 2...
float memoCurrValues[5];
float memoMaxValues[5];
float memoMinValues[5];
bool memoryFull = false;

//      UP
//DOWN      ENTER/MEMORY

//  0       1   2       3        4     5                           6     7
//Current, Max, Min, RevsDone, Memory, PPR (pulse per revolution), Reset, Off

//3.3V - 4.17V on tester

void setup() {

  //Serial.begin(9600);

  analogReference(INTERNAL);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pulse_input_pin, INPUT);
  pinMode(battery_voltmeter_pin, INPUT);
  pinMode(battery_charge_pin, INPUT);

  batteryLevel();
  loadData();

  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
  pinMode(button3_pin, INPUT);
  pinMode(trigger_pin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(pulse_input_pin), pulse, CHANGE);
  u8g2.begin();

}

void loop() {

  if(digitalRead(pulse_input_pin) == HIGH){
    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    digitalWrite(LED_BUILTIN, LOW);
  }

  batteryLevel();
  handleGui();

  if(addToMemory == false){
    handleInput();
  }

}

void loadData () {

   int slotData = EEPROM.read(0);
   int pprData = EEPROM.read(1);
   int curFloatAddr = 2;
  
  if(slotData <= 5 && pprData > 0 && pprData <= 50){
    memorySlotsTaken = slotData;
    ppr = pprData;
  }else{
    fullDeviceReset();
    saveData();
  }

  if(memorySlotsTaken >= 5){
    memoryFull = true;
  }

  EEPROM.get(curFloatAddr, memoCurrValues[0]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMaxValues[0]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMinValues[0]);

  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoCurrValues[1]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMaxValues[1]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMinValues[1]);

  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoCurrValues[2]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMaxValues[2]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMinValues[2]);

  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoCurrValues[3]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMaxValues[3]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMinValues[3]);

  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoCurrValues[4]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMaxValues[4]);
  curFloatAddr += sizeof(float);
  EEPROM.get(curFloatAddr, memoMinValues[4]);

}

void saveData () {

  int curFloatAddr = 2;
  
  EEPROM.write(0, memorySlotsTaken);
  EEPROM.write(1, ppr);

  EEPROM.put(curFloatAddr, memoCurrValues[0]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMaxValues[0]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMinValues[0]);

  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoCurrValues[1]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMaxValues[1]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMinValues[1]);

  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoCurrValues[2]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMaxValues[2]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMinValues[2]);

  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoCurrValues[3]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMaxValues[3]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMinValues[3]);

  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoCurrValues[4]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMaxValues[4]);
  curFloatAddr += sizeof(float);
  EEPROM.put(curFloatAddr, memoMinValues[4]);

  //Serial.println("Save data");
}

void fullDeviceReset () {

  EEPROM.write(0, 0);
  EEPROM.write(1, 1);
  memorySlotsTaken = 0;
  ppr = 1;

  for(int i = 0; i < 5; i++){
    memoMinValues[0] = 999999;
    memoCurrValues[0] = 0;
    memoMaxValues[0] = 0;
  }

  memoryFull = false;
  
}

void batteryLevel () {
  float analogFlt = (float)analogRead(battery_voltmeter_pin);
  cur_bat_voltage = analogFlt / 1023 * 4.45;
  float raw_bat_perc = (cur_bat_voltage - battery_voltage_min) / (battery_voltage_max - battery_voltage_min) * 100;

  if((millis() - bat_lvl_damp_timer) > bat_lvl_damp_interval){

    if((raw_bat_perc / bat_perc) > 0.95 && (raw_bat_perc / bat_perc) < 1.05){
      bat_lvl_damp = 0;
    }

    if(bat_lvl_damp_previus < 0 && bat_lvl_damp > 0){
      bat_lvl_damp = 0;
    }else if(bat_lvl_damp_previus > 0 && bat_lvl_damp < 0){
      bat_lvl_damp = 0;
    }
      
    if(raw_bat_perc < bat_perc){
      bat_lvl_damp -= bat_lvl_damp_strength;
    }else if(raw_bat_perc > bat_perc){
      bat_lvl_damp += bat_lvl_damp_strength;
    }

    if(bat_lvl_damp > bat_lvl_damp_max){
      bat_lvl_damp = bat_lvl_damp_max;   
    }else if((bat_lvl_damp * -1) < (bat_lvl_damp_max * -1)){
      bat_lvl_damp = (bat_lvl_damp_max * -1);
    }

    bat_perc += bat_lvl_damp;
    bat_lvl_damp_previus = bat_lvl_damp;
    
    bat_lvl_damp_timer = millis();
  }

  if((millis() - bat_lvl_sample_timer) > bat_lvl_sample_rate || bat_samples_empty == true){
    
    for(int i = 9; i > 0; i--){
      bat_lvl_samples[i] = bat_lvl_samples[(i - 1)];
    }

    if(bat_samples_empty == true){
      bat_lvl_samples[0] = raw_bat_perc;
    }else{
      bat_lvl_samples[0] = bat_perc;
    }
    
    float avg_bat_perc = 0;
    
    for(int j = 0; j < 10; j++){
      avg_bat_perc += bat_lvl_samples[j];
    }
    
    bat_perc_damp = avg_bat_perc / 10;
    
    if(bat_samples_taken < 10){
      bat_samples_taken++;
    }else{
      bat_samples_empty = false;
    }
    
    bat_lvl_sample_timer = millis();
    
  }
  
}

void handleGui () {

  if((millis() - edit_blink_timer) > 300){
    edit_blink_state = !edit_blink_state;
    edit_blink_timer = millis();
  }

  if(curPage > 6){
    curPage = 0;
  }else if(curPage < 0){
    curPage = 6;
  }
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont29_mr);
  u8g2.setCursor(0,32);

  if(addToMemory == true){
    if((millis() - addedToMemoDisplayTimer) > 1000){
      addToMemory = false;
      if(memorySlotsTaken >= 5){
        memoryFull = true;
      }
    }else{
      if(memoryFull == false){
        u8g2.print("Saved");
      }else{
        u8g2.print("Full");
      }
    }
  }else{
    if(curPage == 0){
      u8g2.print(rpm, 2);  
    }else if(curPage == 1){
      u8g2.print(maxRpm, 2);
    }else if(curPage == 2){
      if(minRpm > maxRpm){
        u8g2.print("-");
      }else{
        u8g2.print(minRpm, 2);
      }
    }else if(curPage == 3){
      u8g2.print(revs, 2);
    }else if(curPage == 4){
      if(subLevel == 0){
        u8g2.print(memorySlotsTaken);
        u8g2.print(" rec.");
      }else if(subLevel == 1){

        switch (curMemoryValue){
          case 0:
          u8g2.print(memoCurrValues[0]);
          break;

          case 1:
          u8g2.print(memoMaxValues[0]);
          break;

          case 2:
          if(memoMinValues[0] > memoMaxValues[0]){
            u8g2.print("-");
          }else{
            u8g2.print(memoMinValues[0]);
          }
          break;

          case 3:
          u8g2.print(memoCurrValues[1]);
          break;

          case 4:
          u8g2.print(memoMaxValues[1]);
          break;

          case 5:
          if(memoMinValues[1] > memoMaxValues[1]){
            u8g2.print("-");
          }else{
            u8g2.print(memoMinValues[1]);
          }
          break;

          case 6:
          u8g2.print(memoCurrValues[2]);
          break;

          case 7:
          u8g2.print(memoMaxValues[2]);
          break;

          case 8:
          if(memoMinValues[2] > memoMaxValues[2]){
            u8g2.print("-");
          }else{
            u8g2.print(memoMinValues[2]);
          }
          break;

          case 9:
          u8g2.print(memoCurrValues[3]);
          break;

          case 10:
          u8g2.print(memoMaxValues[3]);
          break;

          case 11:
          if(memoMinValues[3] > memoMaxValues[3]){
            u8g2.print("-");
          }else{
            u8g2.print(memoMinValues[3]);
          }
          break;

          case 12:
          u8g2.print(memoCurrValues[4]);
          break;

          case 13:
          u8g2.print(memoMaxValues[4]);
          break;

          case 14:
          if(memoMinValues[4] > memoMaxValues[4]){
            u8g2.print("-");
          }else{
            u8g2.print(memoMinValues[4]);
          }
          break;
          
        }
        
      }
    }else if(curPage == 5){
      if(subLevel == 0){
        u8g2.print(ppr);
      }else{
        if(edit_blink_state == true){
          u8g2.print(ppr);
        }
      }
    }else if(curPage == 6){
      if(edit_blink_state == true){
        u8g2.print("Hold 2s");
      }
    }
  }

  u8g2.setFont(u8g2_font_t0_14b_tf);
  u8g2.setCursor(0,9);

  if(curPage == 0){
    u8g2.print("Current");
  }else if(curPage == 1){
    u8g2.print("Max");
  }else if(curPage == 2){
    u8g2.print("Min");
  }else if(curPage == 3){
    u8g2.print("Revolutons");
  }else if(curPage == 4){
    if(subLevel == 0){
      u8g2.print("Memory");
    }else{
      if(edit_blink_state == true){
        u8g2.print("MR ");
        if(curMemoryValue < 3){
          u8g2.print("1 ");
        }else if(curMemoryValue < 6){
          u8g2.print("2 ");
        }else if(curMemoryValue < 9){
          u8g2.print("3 ");
        }else if(curMemoryValue < 12){
          u8g2.print("4 ");
        }else if(curMemoryValue < 15){
          u8g2.print("5 ");
        }

        if(curMemoryValue == 0 || curMemoryValue == 3 || curMemoryValue == 6 || curMemoryValue == 9 || curMemoryValue == 12){
          u8g2.print("Curr:");
        }else if(curMemoryValue == 1 || curMemoryValue == 4 || curMemoryValue == 7 || curMemoryValue == 10 || curMemoryValue == 13){
          u8g2.print("Max:");
        }else if(curMemoryValue == 2 || curMemoryValue == 5 || curMemoryValue == 8 || curMemoryValue == 11 || curMemoryValue == 14){
          u8g2.print("Min:");
        }
      }
    }
  }else if(curPage == 5){
    u8g2.print("PPR");
  }else if(curPage == 6){
    u8g2.print("Reset");
  }

  if(digitalRead(battery_charge_pin) == HIGH){

    if((millis() - bat_anim_timer) >= 100){
      if(bat_chrg_anim <= 100){
        bat_chrg_anim += 5;
      }else{
        bat_chrg_anim = 0;
      }
      bat_anim_timer = millis();
    }
    
    drawBattery(98, 9, 30, bat_chrg_anim);
  }else{
    if(bat_perc_damp < 5){

      if((millis() - bat_blink_timer) > 500){
        bat_blink_state = !bat_blink_state;
        bat_blink_timer = millis();
      }

      if(bat_blink_state == true){
        drawBattery(98, 9, 30, bat_perc_damp);
      }
      
    }else{
      drawBattery(98, 9, 30, bat_perc_damp);
    }
  }

  u8g2.sendBuffer();
}

void reset () {
  rpm = 0;
  counter = 0;
  maxRpm = 0;
  minRpm = 999999;
  revs = 0;
}

void handleInput () {

  if(digitalRead(trigger_pin) != lastTriggerState){
    if((millis() - lastTriggerStateTimer) > debounceDelay){
      readingRPM = digitalRead(trigger_pin);
      lastTriggerState = readingRPM;
    }
  }else{
    lastTriggerStateTimer = millis();
  }

  if(digitalRead(button1_pin) == LOW){
    lastButton1OffState = millis();
  }

  if(digitalRead(button2_pin) == LOW){
    lastButton2OffState = millis();
  }

  if((millis() - lastButton1OffState) > debounceDelay){
    if(button1State == false){
      button1Pressed();
    }
    button1State = true;
  }else{
    button1State = false;
  }

  if((millis() - lastButton2OffState) > debounceDelay){
    if(button2State == false){
      button2Pressed();
    }
    button2State = true;
  }else{
    button2State = false;
  }
  
  if(digitalRead(button3_pin) != lastButton3State || bttn3PressReady == true){
    if((millis() - button3Timer) > debounceDelay){

      if(digitalRead(button3_pin) == HIGH){
        bttn3PressReady = true;
      }

      if(digitalRead(button3_pin) == LOW){
        if(bttn3LongPressComplete == false){
          button3Pressed();
        }else{
          bttn3LongPressComplete = false;
        }
        lastButton3State = digitalRead(button3_pin);
        bttn3PressReady = false;
      }
      
      if((millis() - button3Timer) >= 1500 && digitalRead(button3_pin) == HIGH){
        button3LongPressed();
        lastButton3State = digitalRead(button3_pin);
        bttn3PressReady = false;
      }
    }
  }else{
    button3Timer = millis();
  }

}

void button1Pressed () {//UP
  if(subLevel == 0){
    curPage++;
  }

  if(curPage == 5 && subLevel == 1){
    if(ppr < 50){
      ppr++;
    }
  }

  if(curPage == 4 && subLevel == 1){
    if(curMemoryValue < ((memorySlotsTaken * 3) - 1)){
      curMemoryValue++;
    }else{
      curMemoryValue = 0;
    }
  }
  
}

void button2Pressed () {//DOWN
  if(subLevel == 0){
    curPage--;
  }

  if(curPage == 5 && subLevel == 1){
    if(ppr > 1){
      ppr--;
    }
  }

  if(curPage == 4 && subLevel == 1){
    if(curMemoryValue > 0){
      curMemoryValue--;
    }else{
      curMemoryValue = ((memorySlotsTaken * 3) - 1);
    }
  }
  
}

void button3Pressed () {//ENTER
  if(curPage == 5){
    if(subLevel == 0){
      subLevel = 1;
    }else{
      subLevel = 0;
      saveData();
    }
  }

  if(curPage == 4){
    if(subLevel == 0){
      if(memorySlotsTaken > 0){
        curMemoryValue = 0;
        subLevel = 1;
      }else{
        subLevel = 0;
      }
    }else{
      subLevel = 0;
    }
  }
}

void button3LongPressed () {//MEMORY

  if(subLevel == 0){
    if(curPage == 0 || curPage == 1 || curPage == 2 || curPage == 3){
      addToMemory = true;
      addedToMemoDisplayTimer = millis();
      addRecord();
    }
  }

  if(curPage == 6){
    reset();
    fullDeviceReset();
    curPage = 0;
    subLevel = 0;
    saveData();
  }

  if(curPage == 4 && subLevel == 1 && memorySlotsTaken > 0){

    int deletedMemoId = 0;

    if(curMemoryValue == 0 || curMemoryValue == 1 || curMemoryValue == 2){
      deletedMemoId = 0;
    }else if(curMemoryValue == 3 || curMemoryValue == 4 || curMemoryValue == 5){
      deletedMemoId = 1;
    }else if(curMemoryValue == 6 || curMemoryValue == 7 || curMemoryValue == 8){
      deletedMemoId = 2;
    }else if(curMemoryValue == 9 || curMemoryValue == 10 || curMemoryValue == 11){
      deletedMemoId = 3;
    }else if(curMemoryValue == 12 || curMemoryValue == 13 || curMemoryValue == 14){
      deletedMemoId = 4;
    }

    for(int i = deletedMemoId; i < 5; i++){
      memoCurrValues[i] = memoCurrValues[(i + 1)];
      memoMaxValues[i] = memoMaxValues[(i + 1)];
      memoMinValues[i] = memoMinValues[(i + 1)];
    }

    curMemoryValue = 0;

    if(memorySlotsTaken > 0){
      memorySlotsTaken--;
    }
    
    if(memorySlotsTaken <= 0){
      subLevel = 0;
    }

    memoryFull = false;
    saveData ();

  }

  bttn3LongPressComplete = true;
  
}

void addRecord () {

  if(memorySlotsTaken < 5){
    memorySlotsTaken++;
    memoCurrValues[(memorySlotsTaken - 1)] = rpm;
    memoMaxValues[(memorySlotsTaken - 1)] = maxRpm;
    memoMinValues[(memorySlotsTaken - 1)] = minRpm;
  }
  
  saveData ();
  
}

void drawBattery (int x, int y, int l, float percent){

  int posY = y - 9;
  
  u8g2.drawPixel(x, posY + 1);
  u8g2.drawPixel(x, posY + 2);
  u8g2.drawPixel(x, posY + 3);
  u8g2.drawPixel(x, posY + 4);
  u8g2.drawPixel(x, posY + 5);
  u8g2.drawPixel(x, posY + 6);
  u8g2.drawPixel(x, posY + 7);

  l -= 4;
  int percInt = int(percent / 100 * l);

  for(int i = 0; i < l; i++){
    if(percInt > i){
      u8g2.drawPixel(x + 1 + i, posY + 0);
      u8g2.drawPixel(x + 1 + i, posY + 1);
      u8g2.drawPixel(x + 1 + i, posY + 2);
      u8g2.drawPixel(x + 1 + i, posY + 3);
      u8g2.drawPixel(x + 1 + i, posY + 4);
      u8g2.drawPixel(x + 1 + i, posY + 5);
      u8g2.drawPixel(x + 1 + i, posY + 6);
      u8g2.drawPixel(x + 1 + i, posY + 7);
      u8g2.drawPixel(x + 1 + i, posY + 8);
    }else{
      u8g2.drawPixel(x + 1 + i, posY + 0);
      u8g2.drawPixel(x + 1 + i, posY + 8);
    }
  }

  u8g2.drawPixel(x + l + 1, posY + 0);
  u8g2.drawPixel(x + l + 1, posY + 1);
  u8g2.drawPixel(x + l + 1, posY + 2);
  u8g2.drawPixel(x + l + 1, posY + 3);
  u8g2.drawPixel(x + l + 1, posY + 4);
  u8g2.drawPixel(x + l + 1, posY + 5);
  u8g2.drawPixel(x + l + 1, posY + 6);
  u8g2.drawPixel(x + l + 1, posY + 7);
  u8g2.drawPixel(x + l + 1, posY + 8);

  u8g2.drawPixel(x + l + 2, posY + 2);
  u8g2.drawPixel(x + l + 2, posY + 3);
  u8g2.drawPixel(x + l + 2, posY + 4);
  u8g2.drawPixel(x + l + 2, posY + 5);
  u8g2.drawPixel(x + l + 2, posY + 6);

  u8g2.drawPixel(x + l + 3, posY + 2);
  u8g2.drawPixel(x + l + 3, posY + 3);
  u8g2.drawPixel(x + l + 3, posY + 4);
  u8g2.drawPixel(x + l + 3, posY + 5);
  u8g2.drawPixel(x + l + 3, posY + 6);
  
}

void pulse () {

  if(readingRPM == true){

  unsigned long curMicros = micros();
  
    if(digitalRead(pulse_input_pin) == HIGH){
      if(curMicros - lastPulse < 1000000){
        rpm = ((float)60000000 / (float)(curMicros - lastPulse)) / (float)ppr;
        if(rpm > 99999){
          rpm = 99999;
        }
        counter++;
        revs = (float)counter / (float)ppr;
        if(rpm > maxRpm){
          maxRpm = rpm;
        }else if(rpm < minRpm){
          minRpm = rpm;
        }
        
      }else{
        reset();
      }
      lastPulse = curMicros;
    }
  }
}
