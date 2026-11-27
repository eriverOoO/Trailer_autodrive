// 시리얼 모니터: 115200 baud
void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println(analogRead(A2));
  delay(100);
}
