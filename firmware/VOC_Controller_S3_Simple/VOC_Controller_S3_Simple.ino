/*
 ESP32-S3 / Arduino-ESP32 3.x 전용. Arduino: ESP32S3 Dev Module.
 설치 라이브러리: Adafruit SGP40 Sensor(의존성 모두), AccelStepper.
 PCA9548A 채널0,1=입구 / 2,3=출구. 각 SGP40 독립 VOC 알고리즘.
 USB Serial 115200. z=수동 시작위치 확인, a=시작, x=중단.
 리셋/오류/중단 후 셔틀을 수동 시작위치에 맞추고 z → a 순서로 전송.
 자동 교체는 프린터가 멈춘 실험 조건에서만 사용(프린터 인터록 없음).

 핵심: 출구 VOC 최대 → 팬 약/강 → 강풍에서도 고값 지속 → 교체.
 VOC는 ppm 또는 활성탄 포화 측정값이 아니다. 설정은 실험용 예시.
 팬: 24V 4선, 25kHz PWM 및 0% 정지 지원 제품을 가정.
 GPIO4→1k→2N3904 B / C→팬PWM / E→GND. PWM 반전 포함.
 TB6600: STEP16, DIR17, ENA18 → 각 NPN → 드라이버 음극 입력.
 드라이버 양극 신호 입력은5V, 모터 전원은24V. GPIO로 모터 전원 공급 금지.
 리미트 스위치 없음: GPIO10/12 미사용. 전진/복귀 모두 펄스 수로 추정.
 서보 열림/닫힘은 명령 후 대기시간만 사용. 실제 위치/막힘은 검출하지 못함.
 서보 신호21 / PCA SDA8 SCL9. 기존 배선의 팬 FG5는 이 코드에서 미사용.

 업로드 전 MECHANICS_VERIFIED와 FAN_STOP_VERIFIED를 실물 확인 후 true.
 그 전에도 센서 측정/CSV는 작동하지만 기구 운전 시작은 차단된다.
 자동 교체에 필요한 실제 이동거리·방향·게이트 각도·팬정지가 미확정이므로
 잘못된 예시값으로 곧바로 기구를 움직이지 않도록 한 최초 설정 조건이다.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SGP40.h>
#include <AccelStepper.h>
#include <esp_arduino_version.h>
#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Install esp32 by Espressif Systems 3.x for this sketch"
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module"
#endif

// ========== 배선과 실물에 맞춰 확인할 설정 ==========
constexpr uint8_t PIN_SDA=8, PIN_SCL=9, PIN_STEP=16, PIN_DIR=17, PIN_ENABLE=18;
constexpr uint8_t PIN_GATE_SERVO=21, PIN_FAN=4;
constexpr uint8_t TCA_ADDR=0x70, SGP_ADDR=0x59;
constexpr bool MECHANICS_VERIFIED=false, FAN_STOP_VERIFIED=false;
constexpr bool DIR_INVERT=false;
// GPIO LOW=NPN OFF=ENA 광입력 OFF일 때 드라이버가 활성화되는 제품 가정.
constexpr bool ENABLE_ACTIVE_LOW=true;
constexpr uint16_t GATE_CLOSED_US=1100, GATE_OPEN_US=1900;
constexpr long STROKE_STEPS=800;       // 실제 셔틀 1회 이동거리로 반드시 변경
constexpr float MOVE_SPEED=200, ACCELERATION=300;
constexpr uint16_t REFILL_STROKES=5;   // 1회 공급량을 계량한 뒤 설정
constexpr uint32_t MOVE_TIMEOUT_MS=20000;
constexpr uint32_t GATE_OPEN_WAIT_MS=1500, GATE_CLOSE_WAIT_MS=1500, DUMP_MS=8000, FILL_MS=1000;
constexpr uint32_t FAN_STOP_WAIT_MS=5000; // 실측 정지시간보다 길게
constexpr uint32_t FAN_FREQUENCY=25000;
constexpr uint8_t FAN_LOW_PERCENT=40, FAN_HIGH_PERCENT=100;
constexpr uint32_t VOC_WARMUP_MS=300000;
constexpr int32_t VOC_HIGH=150, VOC_LOW=110; // 건강/농도 기준이 아닌 실험 설정
constexpr uint32_t HIGH_HOLD_MS=15000, LOW_HOLD_MS=60000;
constexpr uint32_t REPLACE_HOLD_MS=300000; // 강풍 + VOC>=150을 연속5분
constexpr uint32_t COOLDOWN_MS=600000;    // 교체 후10분 재교체 판단 금지
static_assert(STROKE_STEPS>0 && REFILL_STROKES>0,"Stroke/count must be positive");
constexpr float ASSUMED_TEMP_C=25.0f, ASSUMED_RH=50.0f;

// 명시적 선언: Arduino 자동 함수 원형 생성과 사용자 타입 충돌 방지.
enum class State:uint8_t { IDLE,CLOSE_INIT,FILTERING,STOP_FAN,
 OPEN_GATE,DUMPING,CLOSE_GATE,FILL,PUSH,RETURN,ERROR };
void setState(State next);
const char* stateName(State s);
struct Hold {
 bool active=false; uint32_t since=0;
 bool reached(bool condition,uint32_t duration){
  if(!condition){active=false;return false;}
  if(!active){active=true;since=millis();}
  return uint32_t(millis()-since)>=duration;
 }
 void reset(){active=false;}
};
struct VocReading { int32_t index=-1; bool valid=false; uint32_t measuredMs=0; };
void logSample(const VocReading* r,uint8_t mask,int32_t demand);
VocReading voc[4];
portMUX_TYPE vocMux=portMUX_INITIALIZER_UNLOCKED;
AccelStepper shuttle(AccelStepper::DRIVER,PIN_STEP,PIN_DIR);
State state=State::IDLE;
uint32_t entered=0, lastReplacement=0, replacements=0;
uint16_t strokes=0; uint8_t fanPercent=0;
bool pwmReady=false, highMode=true, cooling=false;
bool originConfirmed=false, sensorTaskReady=false; // 실제 시작 위치는 사람이 확인
const char* fault="none";
Hold highHold,lowHold,replaceHold,healthyHold;

const char* stateName(State s){
 switch(s){
 case State::IDLE:return "IDLE";case State::CLOSE_INIT:return "CLOSE_INIT";
 case State::FILTERING:return "FILTERING";
 case State::STOP_FAN:return "STOP_FAN";case State::OPEN_GATE:return "OPEN_GATE";
 case State::DUMPING:return "DUMPING";case State::CLOSE_GATE:return "CLOSE_GATE";
 case State::FILL:return "FILL";case State::PUSH:return "PUSH";
 case State::RETURN:return "RETURN";case State::ERROR:return "ERROR";
 }return "UNKNOWN";
}
void fan(uint8_t percent){
 fanPercent=percent;
 // NPN 반전: 팬100%요구→GPIO0%, 팬0%요구→GPIO100%.
 if(pwmReady) ledcWrite(PIN_FAN,(100-percent)*255UL/100);
}
void gate(uint16_t us){if(pwmReady)ledcWrite(PIN_GATE_SERVO,uint32_t(us)*16383/20000);}
void halt(){shuttle.setCurrentPosition(shuttle.currentPosition());shuttle.disableOutputs();}
void resetTimers(){highHold.reset();lowHold.reset();replaceHold.reset();}
void fail(const char* why){
 halt();originConfirmed=false;fan(0);fault=why;state=State::ERROR;entered=millis();resetTimers();
 // 오류 중 게이트를 추가로 움직이지 않는다. 프린터 정지 실험 조건의 정책.
}
// 절대 목표: 전진은 STROKE_STEPS, 복귀는 수동 설정한 좌표 0.
// distanceToGo()==0은 펄스 전송 완료이며 실제 도달을 증명하지 않음.
void beginMove(long target){
 shuttle.enableOutputs();shuttle.setMaxSpeed(MOVE_SPEED);
 shuttle.setAcceleration(ACCELERATION);shuttle.moveTo(target);
}
void setState(State next){
 state=next;entered=millis();
 switch(next){
 case State::CLOSE_INIT:case State::CLOSE_GATE:gate(GATE_CLOSED_US);break;
 case State::RETURN:beginMove(0);break;
 case State::FILTERING:highMode=true;fan(FAN_HIGH_PERCENT);resetTimers();break;
 case State::STOP_FAN:fan(0);resetTimers();break;
 case State::OPEN_GATE:gate(GATE_OPEN_US);break;
 case State::PUSH:beginMove(STROKE_STEPS);break;
 default:break;
 }
}
uint32_t elapsed(){return uint32_t(millis()-entered);}

bool selectChannel(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(uint8_t(1U << channel)); // exactly ONE channel for identical addresses
  return Wire.endTransmission() == 0;
}
bool ack(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}
uint8_t sensorCRC(const uint8_t* data, uint8_t n) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x31) : uint8_t(crc << 1);
  }
  return crc;
}
bool readRawVOC(uint16_t& raw) {
  uint16_t rh = uint16_t(ASSUMED_RH * 65535.0f / 100.0f + 0.5f);
  uint16_t temp = uint16_t((ASSUMED_TEMP_C + 45.0f) * 65535.0f / 175.0f);
  uint8_t cmd[8] = {0x26, 0x0F, uint8_t(rh >> 8), uint8_t(rh), 0,
                   uint8_t(temp >> 8), uint8_t(temp), 0};
  cmd[4] = sensorCRC(cmd + 2, 2);
  cmd[7] = sensorCRC(cmd + 5, 2);
  Wire.beginTransmission(SGP_ADDR);
  Wire.write(cmd, sizeof(cmd));
  if (Wire.endTransmission() != 0) return false;
  // SGP40 measurement duration max 30ms; task delay yields to other tasks.
  vTaskDelay(pdMS_TO_TICKS(35));
  if (Wire.requestFrom(SGP_ADDR, uint8_t(3)) != 3) return false;
  uint8_t reply[3];
  for (uint8_t i = 0; i < 3; ++i) reply[i] = uint8_t(Wire.read());
  if (sensorCRC(reply, 2) != reply[2]) return false;
  raw = (uint16_t(reply[0]) << 8) | reply[1];
  return raw != 0 && raw != 0xFFFF;
}
void vocTask(void*) {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  VocAlgorithmParams algorithm[4];
  for (uint8_t ch = 0; ch < 4; ++ch) VocAlgorithm_init(&algorithm[ch]);
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    for (uint8_t ch = 0; ch < 4; ++ch) {
      VocReading r;
      uint16_t raw = 0;
      if (selectChannel(ch) && ack(SGP_ADDR) && readRawVOC(raw)) {
        VocAlgorithm_process(&algorithm[ch], raw, &r.index);
        r.valid = r.index > 0; // 0 means initial algorithm blackout
        r.measuredMs = millis();
      } else {
        // Communication gaps break the assumed 1Hz history: restart this channel.
        VocAlgorithm_init(&algorithm[ch]);
      }
      portENTER_CRITICAL(&vocMux);
      voc[ch] = r;
      portEXIT_CRITICAL(&vocMux);
    }
    // Nominal 1-second cadence per sensor; no overlapping bus transactions.
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
  }
}

// Serial 송신이 느려도 스텝 펄스 생성을 오래 막지 않도록 짧게 나누어 송신.
char tx[384]; size_t txSize=0,txPos=0;
uint32_t lastLog=0,sequence=0;
const char* CSV_HEADER="seq,uptime_ms,voc_in_a,voc_in_b,voc_out_a,voc_out_b,valid_mask,control_voc,fan_command_pct,state,replacements,cooldown_s,fault";
void pumpSerial(){
 int freeBytes=Serial.availableForWrite();
 if(freeBytes>0 && txPos<txSize){
  size_t n=txSize-txPos;if(n>32)n=32;if(n>size_t(freeBytes))n=freeBytes;
  txPos+=Serial.write(reinterpret_cast<const uint8_t*>(tx)+txPos,n);
 }
}
void logSample(const VocReading* r,uint8_t mask,int32_t demand){
 if(uint32_t(millis()-lastLog)<1000 || txPos<txSize)return;
 lastLog=millis();char v[4][16];
 for(int i=0;i<4;i++){
  if(mask&(1<<i))snprintf(v[i],sizeof(v[i]),"%ld",long(r[i].index));
  else v[i][0]=0; // 무효값은0으로 위장하지 않고 빈칸
 }
 char d[16]="";if(demand>=0)snprintf(d,sizeof(d),"%ld",long(demand));
 uint32_t remain=cooling?(COOLDOWN_MS-uint32_t(millis()-lastReplacement)+999)/1000:0;
 int n=snprintf(tx,sizeof(tx),"%lu,%lu,%s,%s,%s,%s,%u,%s,%u,%s,%lu,%lu,%s\n",
  (unsigned long)++sequence,(unsigned long)millis(),v[0],v[1],v[2],v[3],mask,d,
  fanPercent,stateName(state),(unsigned long)replacements,(unsigned long)remain,fault);
 txSize=n>0 && size_t(n)<sizeof(tx)?size_t(n):0;txPos=0;
}
void setup(){
 // ENA 비활성 레벨을 먼저 설정. 사진 속 드라이버 극성 가정은 반드시 확인.
 digitalWrite(PIN_ENABLE,ENABLE_ACTIVE_LOW?HIGH:LOW);pinMode(PIN_ENABLE,OUTPUT);
 digitalWrite(PIN_FAN,HIGH);pinMode(PIN_FAN,OUTPUT);
 Serial.begin(115200);
 shuttle.setEnablePin(PIN_ENABLE);
 shuttle.setPinsInverted(DIR_INVERT,false,ENABLE_ACTIVE_LOW);shuttle.disableOutputs();
 shuttle.setMinPulseWidth(10);
 // 서로 다른 타이머 쌍: 팬ch0=25kHz/8bit, 서보ch2=50Hz/14bit(S3 지원).
 bool f=ledcAttachChannel(PIN_FAN,FAN_FREQUENCY,8,0);
 bool s=ledcAttachChannel(PIN_GATE_SERVO,50,14,2);
 pwmReady=f&&s;
 if(f)ledcWrite(PIN_FAN,255);if(s)ledcWrite(PIN_GATE_SERVO,0);
 Serial.println(CSV_HEADER);
 if(!pwmReady)fail("PWM_INIT");
 // 센서의35ms대기는 별도 태스크에서 수행: 메인 루프는 모터를 계속 갱신.
 sensorTaskReady=xTaskCreatePinnedToCore(vocTask,"voc",6144,nullptr,1,nullptr,0)==pdPASS;
 if(!sensorTaskReady)fail("SENSOR_TASK");
}
void loop(){
 VocReading r[4];uint8_t mask=0;
 portENTER_CRITICAL(&vocMux);for(int i=0;i<4;i++)r[i]=voc[i];portEXIT_CRITICAL(&vocMux);
 for(int i=0;i<4;i++)if(r[i].valid&&uint32_t(millis()-r[i].measuredMs)<3000)mask|=1<<i;
 int32_t demand=(mask&12)==12?max(r[2].index,r[3].index):-1;
 // 초기/통신회복 후 연속5분 정상 측정 전까지 자동교체를 금지한다.
 bool ready=healthyHold.reached(mask==15,VOC_WARMUP_MS);
 if(cooling&&uint32_t(millis()-lastReplacement)>=COOLDOWN_MS)cooling=false;
 // z는 실제 원점 탐색이 아니다. 사용자가 맞춘 현재 위치를0으로 선언한다.
 // 운전 중 z/a는 무시: 이동 중 좌표가 바뀌지 않도록 한다.
 while(Serial.available()){
  char c=Serial.read();
  if(c=='x')fail("USER_STOP");
  if((state==State::IDLE||state==State::ERROR)&&c=='z'){
   halt();shuttle.setCurrentPosition(0);originConfirmed=true;
  }
  if(c=='a'&&(state==State::IDLE||state==State::ERROR)){
   if(!pwmReady)fault="PWM_INIT";
   else if(!sensorTaskReady)fault="SENSOR_TASK";
   else if(!MECHANICS_VERIFIED||!FAN_STOP_VERIFIED)fault="VERIFY_CONFIG";
   else if(!originConfirmed)fault="SET_ORIGIN_FIRST";
   else {
    fault="none";strokes=0;
    // 시작 이후 모터를 활성 상태로 유지: 대기 중 좌표가 밀리는 것을 줄임.
    // 손으로 조정할 때는 x로 해제하고 재정렬 후 z/a를 다시 전송.
    shuttle.enableOutputs();setState(State::CLOSE_INIT);
   }
  }
 }
 switch(state){
 case State::IDLE:case State::ERROR:break;
 case State::CLOSE_INIT:
  // 닫힘 센서 없음: 서보 명령 뒤 설정시간이 지나면 다음 상태.
  if(elapsed()>=GATE_CLOSE_WAIT_MS)setState(State::FILTERING);
  break;
 case State::RETURN:
  if(elapsed()>MOVE_TIMEOUT_MS){fail("RETURN_TIMEOUT");break;}
  if(shuttle.distanceToGo()==0){
   // 현재 좌표를 매번 강제로0으로 재설정하지 않는다. moveTo(0) 완료를 사용.
   if(++strokes>=REFILL_STROKES){
    replacements++;lastReplacement=millis();cooling=true;setState(State::FILTERING);
   }else setState(State::FILL);
  }
  break;
 case State::FILTERING:
  if(!ready){highMode=true;fan(FAN_HIGH_PERCENT);resetTimers();break;}
  if(highHold.reached(demand>=VOC_HIGH,HIGH_HOLD_MS))highMode=true;
  if(lowHold.reached(demand<=VOC_LOW,LOW_HOLD_MS))highMode=false;
  fan(highMode?FAN_HIGH_PERCENT:FAN_LOW_PERCENT);
  // 中間帯(111~149)에서는 풍량은 유지하되 교체 타이머는 초기화.
  // 쿨다운 중에는 시간을 누적하지 않으므로 종료 뒤 다시5분 필요.
  if(replaceHold.reached(highMode&&demand>=VOC_HIGH&&!cooling,REPLACE_HOLD_MS))
   setState(State::STOP_FAN);
  break;
 case State::STOP_FAN:
  if(elapsed()>=FAN_STOP_WAIT_MS)setState(State::OPEN_GATE);break;
 case State::OPEN_GATE:
  if(elapsed()>=GATE_OPEN_WAIT_MS)setState(State::DUMPING);
  break;
 case State::DUMPING:
  if(elapsed()>=DUMP_MS)setState(State::CLOSE_GATE);
  break;
 case State::CLOSE_GATE:
  if(elapsed()>=GATE_CLOSE_WAIT_MS){strokes=0;setState(State::FILL);}
  break;
 case State::FILL:
  if(elapsed()>=FILL_MS)setState(State::PUSH);
  break;
 case State::PUSH:
  if(elapsed()>MOVE_TIMEOUT_MS){fail("PUSH_TIMEOUT");break;}
  if(shuttle.distanceToGo()==0)setState(State::RETURN);
  break;
 }
 // 시간 초과는 소프트웨어 진행 감시일 뿐 탈조/기계적 걸림 검출이 아니다.
 shuttle.run();logSample(r,mask,demand);pumpSerial();
}
