#define TRIG_PIN 6
#define ECHO_PIN 7

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

#define IR_LEFT A0
#define IR_RIGHT A1
#define IR_BACK A2

void setup() {
  Serial.begin(9600);

  // Capteur ultrason
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Moteurs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Capteurs IR
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_BACK, INPUT);
}

void loop() {
  // Lire les capteurs IR
  int irL = digitalRead(IR_LEFT);
  int irR = digitalRead(IR_RIGHT);
  int irB = digitalRead(IR_BACK);

  // Lire la distance par ultrason
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  distance = duration * 0.034 / 2;

  Serial.print("DIST:");
  Serial.print(distance);
  Serial.print(" | IR:");
  Serial.print(irL);
  Serial.print(",");
  Serial.print(irR);
  Serial.print(",");
  Serial.println(irB);

  // Logique de mouvement :
  if (distance < 20 || irL == 0 || irR == 0) {
    // Obstacle devant ou sur les côtés
    stopMotors();
    delay(200);

    if (irB == 1 && distance < 15) {
      // Rien derrière et trop proche => reculer
      moveBackward();
      delay(500);
    } else {
      // Tourner à gauche ou droite
      moveLeft();
      delay(300);
    }
    stopMotors();
  } else {
    // Route dégagée
    moveForward();
  }

  delay(100);
}

// Fonctions de mouvement
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


