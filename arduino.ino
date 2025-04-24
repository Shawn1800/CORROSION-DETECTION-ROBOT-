#define TRIG_PIN_1 12  // First ultrasonic sensor trigger pin
#define ECHO_PIN_1 13  // First ultrasonic sensor echo pin
#define TRIG_PIN_2 9   // Second ultrasonic sensor trigger pin
#define ECHO_PIN_2 8   // Second ultrasonic sensor echo pin
#define IN1 2          // Motor A direction 1
#define IN2 3          // Motor A direction 2
#define ENA 7          // Motor A PWM speed control
#define IN3 4          // Motor B direction 1
#define IN4 5          // Motor B direction 2
#define ENB 6          // Motor B PWM speed control

// Variables
long duration1, duration2;
float distance1, distance2;

void setup() {
  // Initialize serial communication
  Serial.begin(2000000);

  // Set pin modes for first sensor
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  
  // Set pin modes for second sensor
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  
  // Set pin modes for motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ensure motors are stopped initially
  stopMotors();

  // Brief startup message
  Serial.println("Arduino ready with dual sensors");
}

void loop() {
  // Read and send ultrasonic data from both sensors
  measureDistances();
  Serial.print("ULTRASONIC1: ");
  Serial.print(distance1);
  Serial.print(" | ULTRASONIC2: ");
  Serial.println(distance2);

  // Check for incoming serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    executeCommand(command);
  }

  delay(50); // Small delay to avoid overwhelming serial
}

// Measure distance using both ultrasonic sensors
void measureDistances() {
  // First sensor measurement
  digitalWrite(TRIG_PIN_1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_1, LOW);

  duration1 = pulseIn(ECHO_PIN_1, HIGH, 30000); // Timeout after 30ms (approx 5m)
  if (duration1 == 0) {
    distance1 = 999.0; // Indicate out-of-range or error
  } else {
    distance1 = duration1 * 0.034 / 2; // Distance in cm
  }
  
  // Small delay between sensor readings to avoid interference
  delayMicroseconds(100);
  
  // Second sensor measurement
  digitalWrite(TRIG_PIN_2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_2, LOW);

  duration2 = pulseIn(ECHO_PIN_2, HIGH, 30000); // Timeout after 30ms (approx 5m)
  if (duration2 == 0) {
    distance2 = 999.0; // Indicate out-of-range or error
  } else {
    distance2 = duration2 * 0.034 / 2; // Distance in cm
  }
}

// Execute commands from Raspberry Pi
void executeCommand(char cmd) {
  switch (cmd) {
    case 'F': // Forward
      moveForward();
      Serial.println("Moving forward");
      break;
    case 'B': // Backward
      moveBackward();
      Serial.println("Moving backward");
      break;
    case 'L': // Left
      turnLeft();
      Serial.println("Turning left");
      break;
    case 'R': // Right
      turnRight();
      Serial.println("Turning right");
      break;
    case 'S': // Stop
      stopMotors();
      Serial.println("Stopped");
      break;
    case 'T': // Test motor (e.g., "TA1" for Motor A forward)
      while (Serial.available() < 2) delay(1); // Wait for motor and direction
      char motor = Serial.read();
      char direction = Serial.read();
      testMotor(motor, direction);
      break;
    default:
      Serial.println("Unknown command");
      break;
  }
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150); // Motor A speed (0-255)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 150); // Motor B speed
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 150);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150); // Motor A forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150); // Motor B backward
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 150); // Motor A backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 150); // Motor B forward
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void testMotor(char motor, char direction) {
  if (motor == 'A') {
    if (direction == '1') { // Forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 150);
      Serial.println("Motor A forward");
    } else if (direction == '0') { // Backward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 150);
      Serial.println("Motor A backward");
    } else if (direction == 'S') { // Stop
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      Serial.println("Motor A stopped");
    }
  } else if (motor == 'B') {
    if (direction == '1') { // Forward
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 150);
      Serial.println("Motor B forward");
    } else if (direction == '0') { // Backward
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 150);
      Serial.println("Motor B backward");
    } else if (direction == 'S') { // Stop
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
      Serial.println("Motor B stopped");
    }
  }
}
