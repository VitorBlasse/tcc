#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

const int IN1 = 5;
const int IN2 = 6;

float angleAcc = 0;
float angleGyro = 0;
float angleFiltered = 0;
float angleOffset = 0;
float alpha = 0.98;
float dt = 0.01;

float Kp = 22.0;
float Ki = 10.5;
float Kd = 1.0;

float previousError = 0;
float integral = 0;

const int motorSpeedMax = 210;
const int motorSpeedMin = 180;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginMag();
  delay(50);

  mySensor.accelUpdate();
  angleOffset = -atan2(mySensor.accelX(), mySensor.accelZ()) * 180 / PI;

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("Sistema inicializado");
}

void loop() {
  float pidOutput = 0;
  unsigned long now = millis();
  if (now - lastTime >= dt * 1000) {
    lastTime = now;

    mySensor.accelUpdate();
    angleAcc = -atan2(mySensor.accelX(), mySensor.accelZ()) * 180 / PI - angleOffset;

    if (angleAcc > 180) angleAcc -= 360;
    else if (angleAcc < -180) angleAcc += 360;

    angleGyro -= mySensor.gyroX() * dt;
    angleFiltered = alpha * angleGyro + (1 - alpha) * angleAcc;

    float erro = angleFiltered;

    if (erro > 10.0) {
      motorGira(true);
    } else if (erro < -10.0) {
      motorGira(false);
    } else if (abs(erro) < 0.05) {
      motorParado(); // Zona morta
    } else {
      float derivative = erro - previousError;
      integral += erro * dt;
      float pidOutput = Kp * erro + Ki * integral + Kd * derivative;

      motorControle(pidOutput);
      previousError = erro;
    }

    Serial.print(0);
    Serial.print("\t");
    Serial.print(angleFiltered);
    Serial.print("\t");
    Serial.print(erro);
    Serial.print("\t");
    Serial.println(pidOutput);

  }
}

void motorControle(float pidOutput) {
  int motorSpeed = constrain(abs(pidOutput), motorSpeedMin, motorSpeedMax);

  if (pidOutput > 0) {
    analogWrite(IN1, motorSpeed);
    digitalWrite(IN2, LOW);
  } else if (pidOutput < 0) {
    analogWrite(IN2, motorSpeed);
    digitalWrite(IN1, LOW);
  } else {
    motorParado();
  }
}

void motorGira(bool direita) {
  int motorSpeed = 200;

  if (direita) {
    analogWrite(IN1, motorSpeed);
    digitalWrite(IN2, LOW);
  } else {
    analogWrite(IN2, motorSpeed);
    digitalWrite(IN1, LOW);
  }
}

void motorParado() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}
