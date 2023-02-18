// Define the input pins for the RC receiver
#define CH1_PIN 2
#define CH2_PIN 3

// Define the output pins for the H-bridge
#define MOTOR1_A 6
#define MOTOR1_B 7
#define MOTOR2_A 9
#define MOTOR2_B 8
#define EN_M1 5
#define EN_M2 4

// Create variables to hold the RC channel values
volatile int ch1Val, ch2Val;
int min_value = 1000;
int max_value = 2000;

void setup() {
  // Configure the input and output pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);

  // Set up interrupt service routines for the RC channels
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), readCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), readCh2, CHANGE);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if the channel values are within an acceptable range
  if (ch1Val < min_value || ch1Val > max_value) {
    ch1Val = constrain(ch1Val, min_value, max_value);
  }
  if (ch2Val < min_value || ch2Val > max_value) {
    ch2Val = constrain(ch2Val, min_value, max_value);
  }

  // Map the channel values to motor speeds and directions
  int motorSpeed = map(ch1Val, 1000, 2000, -255, 255);
  int steerValue = map(ch2Val, 1000, 2000, -255, 255);

  // Determine the motor directions based on the steerValue and motorSpeed
  int motor1Direction, motor2Direction;
  if (steerValue > 0) {
    motor1Direction = HIGH;
    motor2Direction = map(steerValue, 0, 255, HIGH, LOW);
  } else if (steerValue < 0) {
    motor1Direction = map(steerValue, -255, 0, LOW, HIGH);
    motor2Direction = HIGH;
  } else {
    motor1Direction = motor2Direction = (motorSpeed >= 0) ? HIGH : LOW;
  }

  // Determine the motor PWM values based on the motor speed
  int motor1PWM = abs(motorSpeed);
  int motor2PWM = abs(motorSpeed);

  // Determine the steering PWM values based on the steerValue
  int steerPWM1 = map(steerValue, -255, 0, 0, 255);
  int steerPWM2 = map(steerValue, 0, 255, 0, 255);

  // Apply the motor and steering PWM values to the H-bridge
  analogWrite(EN_M1, motor1PWM);
  analogWrite(EN_M2, motor2PWM);
  digitalWrite(MOTOR1_A, motor1Direction);
  digitalWrite(MOTOR1_B, !motor1Direction);
  digitalWrite(MOTOR2_A, motor2Direction);
  digitalWrite(MOTOR2_B, !motor2Direction);

  // Print the current channel values and mapped motor and steering values to the serial monitor
  Serial.print("Channel 1 value: ");
  Serial.print(ch1Val);
  Serial.print("\tMapped motor speed: ");
  Serial.println(motorSpeed);
  Serial.print("Channel 2 value: ");
  Serial.print(ch2Val);
  Serial.print("\tMapped steer value: ");
  Serial.println(steerValue);
}

// Interrupt service routine for channel 1
void readCh1() {
  static unsigned long ch1Start = 0;
  if (digitalRead(CH1_PIN) == HIGH) {
    ch1Start = micros();
  } else {
    ch1Val = (int)(micros() - ch1Start);
  }
}

// Interrupt service routine for channel 2
void readCh2() {
  static unsigned long ch2Start = 0;
  if (digitalRead(CH2_PIN) == HIGH) {
    ch2Start = micros();
  } else {
    ch2Val = (int)(micros() - ch2Start);
  }
}
