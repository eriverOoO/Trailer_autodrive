// Stroller controller compatible with: s<steer>l<left_pwm>r<right_pwm>\n
// Original hardware convention: steering -7 = left, +7 = right.
const unsigned int MAX_INPUT = 20;
const int STEERING_1 = 3;
const int STEERING_2 = 2;
const int FORWARD_RIGHT_1 = 4;
const int FORWARD_RIGHT_2 = 5;
const int FORWARD_LEFT_1 = 6;
const int FORWARD_LEFT_2 = 7;
const int POT = A2;
const int STEERING_SPEED = 128;
const int RESISTANCE_MOST_LEFT = 600;
const int RESISTANCE_MOST_RIGHT = 445;
const int MAX_STEERING_STEP = 7;
const unsigned long COMMAND_INTERVAL_MS = 50;
const unsigned long COMMAND_TIMEOUT_MS = 300;

int angle = 0;
int leftSpeed = 0;
int rightSpeed = 0;
unsigned long lastControlTime = 0;
unsigned long lastPacketTime = 0;

void setMotorSpeed(int pinForward, int pinReverse, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(pinForward, speed);
    analogWrite(pinReverse, LOW);
  } else {
    analogWrite(pinForward, LOW);
    analogWrite(pinReverse, -speed);
  }
}

void processData(const char *data) {
  const char *s = strchr(data, 's');
  const char *l = strchr(data, 'l');
  const char *r = strchr(data, 'r');
  if (s == NULL || l == NULL || r == NULL || !(s < l && l < r)) return;
  angle = constrain(atoi(s + 1), -MAX_STEERING_STEP, MAX_STEERING_STEP);
  leftSpeed = constrain(atoi(l + 1), -255, 255);
  rightSpeed = constrain(atoi(r + 1), -255, 255);
  lastPacketTime = millis();
}

void readSerial() {
  static char line[MAX_INPUT];
  static unsigned int pos = 0;
  while (Serial.available() > 0) {
    char value = Serial.read();
    if (value == '\n') {
      line[pos] = '\0';
      processData(line);
      pos = 0;
    } else if (value != '\r' && pos < MAX_INPUT - 1) {
      line[pos++] = value;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(POT, INPUT);
  pinMode(STEERING_1, OUTPUT);
  pinMode(STEERING_2, OUTPUT);
  pinMode(FORWARD_RIGHT_1, OUTPUT);
  pinMode(FORWARD_RIGHT_2, OUTPUT);
  pinMode(FORWARD_LEFT_1, OUTPUT);
  pinMode(FORWARD_LEFT_2, OUTPUT);
  delay(4000);
}

void loop() {
  readSerial();
  unsigned long now = millis();
  if (now - lastPacketTime > COMMAND_TIMEOUT_MS) {
    leftSpeed = 0;
    rightSpeed = 0;
  }
  if (now - lastControlTime < COMMAND_INTERVAL_MS) return;
  int resistance = analogRead(POT);
  int currentAngle = map(resistance, RESISTANCE_MOST_LEFT,
                         RESISTANCE_MOST_RIGHT, -MAX_STEERING_STEP,
                         MAX_STEERING_STEP + 1);
  if (currentAngle == angle) {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, LOW);
  } else if (currentAngle > angle) {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, STEERING_SPEED);
  } else {
    analogWrite(STEERING_1, STEERING_SPEED);
    analogWrite(STEERING_2, LOW);
  }
  setMotorSpeed(FORWARD_LEFT_1, FORWARD_LEFT_2, leftSpeed);
  setMotorSpeed(FORWARD_RIGHT_1, FORWARD_RIGHT_2, rightSpeed);
  lastControlTime = now;
}
