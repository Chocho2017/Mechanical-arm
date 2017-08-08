
#define DEBUG
#define VERSION 0.03

const static char strNum[] = "0123456789";
#ifdef DEBUG
char strADC[] = "ADC: 0000\r\n";
char strAngleBase[] = "angle base: 000\r\n";
char strAngleHandleA[] = "angle handle A: 000\r\n";
char strAngleHandleB[] = "angle handle B: 000\r\n";
char strAngleClip[] = "angle clip: 000\r\n";
char strAngleTime[] = "angle time: 00000\r\n";
char strAnalogBase[] = "analog base: 0000\r\n";
char strAnalogHandleA[] = "analog handle A: 0000\r\n";
char strAnalogHandleB[] = "analog handle B: 0000\r\n";
char strAnalogClip[] = "analog clip: 0000\r\n";
#endif

#define SERVO_ANGLETIME 4  //单位ms
#define SERVO_ANGLETIME_MULTIPLE 4  //时间延长倍数
#define SERVOPULSE_MIN 544
#define SERVOPULSE_MAX 2400
#define PINSERVO_BASE D2
#define PINSERVO_HANDLEA D4
#define PINSERVO_HANDLEB D5
#define PINSERVO_CLIP D6
#define PINADC_BASE A0
#define PINADC_HANDLEA A1
#define PINADC_HANDLEB A2
#define PINADC_CLIP A3
#define PIN_BUTTON D7
#define ACTIONNUM 64
#define SHIVERLIMIT 15
#define PWMCHANNEL_BASE 0
#define PWMCHANNEL_HANDLEA 1
#define PWMCHANNEL_HANDLEB 2
#define PWMCHANNEL_CLIP 3
#define PWM_CYCLE 50
#define PWM_BITS 12
#define BUTTON_MAXTIME 100

int angleArea[ACTIONNUM * 4] = {0};
int angleAnalog[4] = {0};


//用来设计不占用CPU延时，每有一个延时需求，使用此结构体创建一个独立的延时结构体
typedef struct {
  unsigned long lastTime;
  uint8_t state;
}timePassStructDef;


typedef struct {
  int dat[4];
}actionStructDef;
actionStructDef actionStructWrite, actionStructRead;


//将数字转换填入到字符串中第一处有数字的区域
void strSetNum(char *pstr, int num) {

  while(*pstr) {
    if(((*pstr - '0') < 10) && (*pstr - '0' >= 0)) {
      while(((*pstr - 0x30) < 10) && (*pstr - 0x30 >= 0)) {
        pstr ++;
      }
      pstr --;
      while(((*pstr - 0x30) < 10) && (*pstr - 0x30 >= 0)) {
        *pstr = num % 10 + 0x30;
        num /= 10;
        pstr --;
      }
      return;
    }
    pstr ++;
  }
}


//从调用该函数开始计时，时间到则返回1，否则返回0
uint8_t timePass(timePassStructDef *passStruct, uint16_t t) {

  static unsigned long currentTime = 0;
  currentTime = millis();
  if(!passStruct->state) {
    passStruct->lastTime = currentTime;
    passStruct->state = 1;   
  }
  if(((currentTime - t) > passStruct->lastTime) || ((currentTime - t) < (passStruct->lastTime - t))) {
    passStruct->state = 0;
    return 1;
  }
  return 0; 
}


//读取角度存储区中的值
void restoreAngle(uint8_t action, actionStructDef *actionStruct) {

  actionStruct->dat[0] = angleArea[action * 4];
  actionStruct->dat[1] = angleArea[action * 4 + 1];
  actionStruct->dat[2] = angleArea[action * 4 + 2];
  actionStruct->dat[3] = angleArea[action * 4 + 3];
}


//向角度存储器写值
void storeAngle(uint8_t action, actionStructDef *actionStruct) {

  angleArea[action * 4] = actionStruct->dat[0];
  angleArea[action * 4 + 1] = actionStruct->dat[1];
  angleArea[action * 4 + 2] = actionStruct->dat[2];
  angleArea[action * 4 + 3] = actionStruct->dat[3];
}


//对变量进行最大最小值限制
void varLimit(int *var, int varMax, int varMin) {

  if(*var < varMin) {
    *var = varMin;
  }else if (*var > varMax) {
    *var = varMax;
  }
}


//按键读取并返回按下的时间长短（最长返回1s）
uint8_t buttonRead(void) {

  uint8_t i = 0;
  if(digitalRead(PIN_BUTTON)) {
    delay(10);
    while(digitalRead(PIN_BUTTON)) {
      if(digitalRead(PIN_BUTTON)) {
        if(i < 100) {
          i ++;
        }
      }
      delay(10);
    }
  }
  return i;
}


//控制舵机
void pulseSet(uint8_t chan, int angle) {

  int temp = angle;
  temp = angle * (SERVOPULSE_MAX - SERVOPULSE_MIN) / 180 + SERVOPULSE_MIN;
  temp = map(temp, 0, (1000000 / PWM_CYCLE), 0, 4096);
  ledcWrite(chan, temp);
}


//
#define STEP_GO_ONE(action1, action2, actionServo, actionBit) \
        if(action1 != action2) { \
          if(action1 < action2) { \
            action2 --; \
          }else { \
            action2 ++; \
          } \
          pulseSet(actionServo, action2); \
        }else { \
          stepOverBit |= actionBit; \
        }

//执行一个动作，(这个函数对每一度的动作进行了时间上的延长，因为不延长的话舵机动作太快)
uint8_t oneStep(uint8_t currentStep, uint8_t lastStep) {

  static timePassStructDef stepTimeStruct;
  static uint8_t  stepOverBit = 0;
  static uint16_t stepTime = 0, stepDiv = 0, stepStatus = 0;
  static actionStructDef objAction, lastAction;
  if(!stepStatus) {
    stepStatus = 1;
    restoreAngle(currentStep, &objAction);
    restoreAngle(lastStep, &lastAction);
  }
  if(stepStatus) {
    if(!stepTime) {
      STEP_GO_ONE(objAction.dat[0], lastAction.dat[0], PWMCHANNEL_BASE, 0x01);
      STEP_GO_ONE(objAction.dat[1], lastAction.dat[1], PWMCHANNEL_HANDLEA, 0x02);
      STEP_GO_ONE(objAction.dat[2], lastAction.dat[2], PWMCHANNEL_HANDLEB, 0x04);
      STEP_GO_ONE(objAction.dat[3], lastAction.dat[3], PWMCHANNEL_CLIP, 0x08);
      stepTime = 1;
    }
    if(timePass(&stepTimeStruct, SERVO_ANGLETIME * SERVO_ANGLETIME_MULTIPLE)) {
      stepTime = 0;
      if(stepOverBit == 0x0f) {
        stepOverBit = 0x00;
        stepStatus = 0;
        return 0;
      }
    }
  }
  return stepStatus;
}


void setup(void) {

  ledcSetup(PWMCHANNEL_BASE, PWM_CYCLE, PWM_BITS);
  ledcAttachPin(PINSERVO_BASE, PWMCHANNEL_BASE);
  ledcSetup(PWMCHANNEL_HANDLEA, PWM_CYCLE, PWM_BITS);
  ledcAttachPin(PINSERVO_HANDLEA, PWMCHANNEL_HANDLEA);
  ledcSetup(PWMCHANNEL_HANDLEB, PWM_CYCLE, PWM_BITS);
  ledcAttachPin(PINSERVO_HANDLEB, PWMCHANNEL_HANDLEB);
  ledcSetup(PWMCHANNEL_CLIP, PWM_CYCLE, PWM_BITS);
  ledcAttachPin(PINSERVO_CLIP, PWMCHANNEL_CLIP);
  
  pinMode(PIN_BUTTON, INPUT);
  Serial.begin(115200);
  actionStructWrite.dat[0] = 0xff;
  Serial.print("system runing\r\n");
}


void loop(void) {

  static uint8_t buttonTime = 0;
  static uint8_t isAction = 0;
  static int angleWriteCount = 0, readAngleCount = 0;
  
  //按钮读取并设置
  buttonTime = buttonRead();
  if(buttonTime) {
    //长按动作
    if(buttonTime == BUTTON_MAXTIME) {
      if(isAction) {
        isAction = 0;
        angleWriteCount = 0;
        readAngleCount = 0;
      }else {
        isAction = 1;
        actionStructWrite.dat[0] = 0xff;
        storeAngle(angleWriteCount, &actionStructWrite);
        readAngleCount = 0;
      }
#ifdef DEBUG
      Serial.print("button long\r\n");
#endif
    }else{  //短按动作
      if(!isAction) {
        storeAngle(angleWriteCount, &actionStructWrite);
        angleWriteCount ++;
        varLimit(&angleWriteCount, ACTIONNUM, 0);
#ifdef DEBUG
        Serial.print("button short\r\n");
        strSetNum(strAngleBase, actionStructWrite.dat[0]);
        strSetNum(strAngleHandleA, actionStructWrite.dat[1]);
        strSetNum(strAngleHandleB, actionStructWrite.dat[2]);
        strSetNum(strAngleClip, actionStructWrite.dat[3]);
        strSetNum(strAnalogBase, angleAnalog[0]);
        strSetNum(strAnalogHandleA, angleAnalog[1]);
        strSetNum(strAnalogHandleB, angleAnalog[2]);
        strSetNum(strAnalogClip, angleAnalog[3]);
        Serial.print(strAnalogBase);
        Serial.print(strAnalogHandleA);
        Serial.print(strAnalogHandleB);
        Serial.print(strAnalogClip);
        Serial.print(strAngleBase);
        Serial.print(strAngleHandleA);
        Serial.print(strAngleHandleB);
        Serial.print(strAngleClip);
#endif
      }
    }
  }

  if(isAction) {
    restoreAngle(readAngleCount, &actionStructRead);
    if(actionStructRead.dat[0] == 0xff) {
      readAngleCount = 0;
      restoreAngle(readAngleCount, &actionStructRead); 
    }
    if(!(actionStructRead.dat[0] == 0xff)) {
      if(readAngleCount) {
        if(!oneStep(readAngleCount, readAngleCount - 1)) {
          readAngleCount ++;
          Serial.print("to next action \r\n");
        }
      }else {
        if(!oneStep(readAngleCount, angleWriteCount - 1)) {
          readAngleCount ++;
          Serial.print("to next action \r\n");
        }
      }
    }
  }else {
    angleAnalog[0] = analogRead(PINADC_BASE);
    actionStructWrite.dat[0] = map(angleAnalog[0], 0, 4096, 0, 180);
    varLimit(&actionStructWrite.dat[0], 180 - SHIVERLIMIT, SHIVERLIMIT);
    angleAnalog[1] = analogRead(PINADC_HANDLEA);    
    actionStructWrite.dat[1] = map(angleAnalog[1], 0, 4096, 0, 180);
    varLimit(&actionStructWrite.dat[1], 180 - SHIVERLIMIT, SHIVERLIMIT);
    angleAnalog[2] = analogRead(PINADC_HANDLEB);    
    actionStructWrite.dat[2] = 180 - map(angleAnalog[2], 0, 4096, 0, 180);
    varLimit(&actionStructWrite.dat[2], 180 - SHIVERLIMIT, SHIVERLIMIT);
    angleAnalog[3] = analogRead(PINADC_CLIP);
    actionStructWrite.dat[3] = map(angleAnalog[3], 0, 4096, 0, 180);
    varLimit(&actionStructWrite.dat[3], 180 - SHIVERLIMIT, SHIVERLIMIT);
    pulseSet(PWMCHANNEL_BASE, actionStructWrite.dat[0]);
    pulseSet(PWMCHANNEL_HANDLEA, actionStructWrite.dat[1]);
    pulseSet(PWMCHANNEL_HANDLEB, actionStructWrite.dat[2]);
    pulseSet(PWMCHANNEL_CLIP, actionStructWrite.dat[3]);
    delay(10);
  }
}


