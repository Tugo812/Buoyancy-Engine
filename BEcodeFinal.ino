#include <Servo.h>

Servo myServo;


const int SERVO_PIN     = 12;
const int SERVO_ASCEND  = 11;    // angle for ascent
const int SERVO_SINK    = 110;   // angle for sinking
int currentAngle        = SERVO_ASCEND;  // start at ascent


const int PIN_SENSOR = A0;
const float ADC_MAX = 1023.0;

const float P_MAX_MPA   = 1.6;
const float P_PER_V_MPA = P_MAX_MPA / 4.0;
const float P_PER_V_PA  = P_PER_V_MPA * 1e6;

const float RHO = 1000.0;
const float G   = 9.80665;

const float offsetV = 0.4744;


const float DEPTH_SINK_MIN = 0.005;   // start sinking if inside this range
const float DEPTH_SINK_MAX = 0.090;
const float DEPTH_ASCEND   = 0.50;    // start ascent when >= this

void setup() {
  Serial.begin(115200);
  Serial.println("Buoyancy control + pressure sensor");

  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  delay(400);
}

void loop() {
  const unsigned long WINDOW_US = 20000UL; // 20ms averaging window
  unsigned long start = micros();

  long sumRaw = 0;
  float sumDepth = 0.0;
  unsigned long count = 0;

  while (micros() - start < WINDOW_US) {
    int raw = analogRead(PIN_SENSOR);
    float v = raw * (5.0 / ADC_MAX);

    float dV = v - offsetV;
    if (dV < 0) dV = 0;

    float p_pa = dV * P_PER_V_PA;
    float depth_m = p_pa / (RHO * G);
    

    sumRaw += raw;
    sumDepth += depth_m;
    count++;
  }

  float avgRaw   = (float)sumRaw / count;
  float avgDepth = sumDepth / count;
  float avgV     = avgRaw * (5.0 / ADC_MAX);

  
  // SINK ZONE
  if (avgDepth >= DEPTH_SINK_MIN && avgDepth <= DEPTH_SINK_MAX) {
    if (currentAngle != SERVO_SINK) {
      currentAngle = SERVO_SINK;
      myServo.write(currentAngle);
      Serial.println(">>> ENTERING SINK MODE (110°)");
    }
  }

  // ASCEND ZONE
  else if (avgDepth >= DEPTH_ASCEND) {
    if (currentAngle != SERVO_ASCEND) {
      currentAngle = SERVO_ASCEND;
      myServo.write(currentAngle);
      Serial.println(">>> ENTERING ASCENT MODE (11°)");
    }
  }

 
  Serial.print("Depth: ");
  Serial.print(avgDepth, 4);
  Serial.print(" m | Servo: ");
  Serial.println(currentAngle);

  delay(50);
}
