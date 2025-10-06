// MOTOR-A
void motorA_ileri(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannelA, speed);
}

void motorA_geri(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(pwmChannelA, speed);
}

void motorA_dur() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannelA, 0);
}

// MOTOR-B
void motorB_ileri(int speed) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelB, speed);
}

void motorB_geri(int speed) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelB, speed);
}

void motorB_dur() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelB, 0);
}

// OTHERS

void motorAll_dur() {
  motorA_dur();
  motorB_dur();
}

void sagaDon() {
  motorAll_dur();
  delay(1000);

  motorA_ileri(150);
  motorB_geri(150);
  delay(300);

  while (true) {
    sensorleriOku();
    if (sensorDegerleri[3] > 2200 || sensorDegerleri[4] > 2200) {
      break;
    }
  }
  motorAll_dur();
  delay(1000);
}

void solaDon() {
  motorAll_dur();
  delay(1000);

  motorA_geri(150);
  motorB_ileri(150);
  delay(300);

  while (true) {
    sensorleriOku();
    if (sensorDegerleri[3] > 2200 || sensorDegerleri[4] > 2200) {
      break;
    }
  }
  motorAll_dur();
  delay(1000);
}

void geriDon() {
  sagaDon();
  sagaDon();
}
