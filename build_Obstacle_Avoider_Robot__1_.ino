#include <Servo.h> 

Servo Myservo;
#define trigPin 9
#define echoPin 8
#define MLa 4
#define MLb 5
#define MRa 6
#define MRb 7

// ANN Parameters
#define INPUT_NEURONS 3    // Distance, Left scan, Right scan
#define HIDDEN_NEURONS 4   // Hidden layer neurons
#define OUTPUT_NEURONS 3   // Forward, Left, Right

// Neural Network Weights (Pre-trained for obstacle avoidance)
float weightsIH[INPUT_NEURONS][HIDDEN_NEURONS] = {
  {0.8, -0.5, 0.3, 0.6},   // Distance weights
  {0.4, 0.7, -0.3, 0.5},   // Left scan weights
  {-0.4, 0.5, 0.8, -0.2}   // Right scan weights
};

float weightsHO[HIDDEN_NEURONS][OUTPUT_NEURONS] = {
  {0.9, -0.4, -0.5},
  {-0.3, 0.7, -0.2},
  {-0.6, -0.3, 0.8},
  {0.5, 0.6, -0.7}
};

float biasH[HIDDEN_NEURONS] = {0.1, -0.2, 0.3, -0.1};
float biasO[OUTPUT_NEURONS] = {0.2, 0.1, 0.1};

long duration, distance;
float leftDist = 0, rightDist = 0;

// Activation function (Sigmoid)
float sigmoid(float x) {
  return 1.0 / (1.0 + exp(-x));
}

// Measure distance using ultrasonic sensor
float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);   
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  if (duration == 0) return 200; // No obstacle detected
  return duration / 58.2;
}

// Scan left and right using servo
void scanEnvironment() {
  // Look left
  Myservo.write(0);
  delay(300);
  leftDist = measureDistance();
  
  // Look right
  Myservo.write(180);
  delay(300);
  rightDist = measureDistance();
  
  // Return to center
  Myservo.write(90);
  delay(300);
}

// Neural Network Forward Pass
void neuralNetworkDecision(float inputs[INPUT_NEURONS], float outputs[OUTPUT_NEURONS]) {
  float hidden[HIDDEN_NEURONS];
  
  // Input to Hidden Layer
  for (int h = 0; h < HIDDEN_NEURONS; h++) {
    hidden[h] = biasH[h];
    for (int i = 0; i < INPUT_NEURONS; i++) {
      hidden[h] += inputs[i] * weightsIH[i][h];
    }
    hidden[h] = sigmoid(hidden[h]);
  }
  
  // Hidden to Output Layer
  for (int o = 0; o < OUTPUT_NEURONS; o++) {
    outputs[o] = biasO[o];
    for (int h = 0; h < HIDDEN_NEURONS; h++) {
      outputs[o] += hidden[h] * weightsHO[h][o];
    }
    outputs[o] = sigmoid(outputs[o]);
  }
}

// Motor control functions
void moveForward() {
  digitalWrite(MRb, HIGH);
  digitalWrite(MRa, LOW);
  digitalWrite(MLb, HIGH);
  digitalWrite(MLa, LOW);
}

void moveBackward() {
  digitalWrite(MRb, LOW);
  digitalWrite(MRa, HIGH);
  digitalWrite(MLb, LOW);
  digitalWrite(MLa, HIGH);
}

void turnLeft() {
  digitalWrite(MRb, HIGH);
  digitalWrite(MRa, LOW);
  digitalWrite(MLa, LOW);
  digitalWrite(MLb, LOW);
}

void turnRight() {
  digitalWrite(MRb, LOW);
  digitalWrite(MRa, LOW);
  digitalWrite(MLa, HIGH);
  digitalWrite(MLb, LOW);
}

void stopMotors() {
  digitalWrite(MRb, LOW);
  digitalWrite(MRa, LOW);
  digitalWrite(MLb, LOW);
  digitalWrite(MLa, LOW);
}

void setup() {
  Serial.begin(9600);
  pinMode(MLa, OUTPUT);
  pinMode(MLb, OUTPUT);
  pinMode(MRa, OUTPUT);
  pinMode(MRb, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Myservo.attach(10);
  Myservo.write(90); // Center position
  delay(500);
  
  Serial.println("ANN Obstacle Avoider Initialized");
}

void loop() {
  // Measure front distance
  distance = measureDistance();
  Serial.print("Front: ");
  Serial.print(distance);
  Serial.print(" cm");
  
  // If obstacle detected nearby
  if (distance < 30 && distance > 0) {
    // STOP immediately when obstacle detected
    stopMotors();
    delay(200);
    
    Serial.println(" | OBSTACLE DETECTED!");
    
    // Scan environment
    scanEnvironment();
    
    Serial.print("Left: ");
    Serial.print(leftDist);
    Serial.print(" | Right: ");
    Serial.println(rightDist);
    
    // Normalize inputs (0-1 range, inverted so closer = higher value)
    float inputs[INPUT_NEURONS];
    inputs[0] = 1.0 - min(distance / 100.0, 1.0);      // Front distance
    inputs[1] = 1.0 - min(leftDist / 100.0, 1.0);      // Left distance
    inputs[2] = 1.0 - min(rightDist / 100.0, 1.0);     // Right distance
    
    // Get ANN decision
    float outputs[OUTPUT_NEURONS];
    neuralNetworkDecision(inputs, outputs);
    
    // Find highest output (decision)
    int decision = 0;
    float maxOutput = outputs[0];
    for (int i = 1; i < OUTPUT_NEURONS; i++) {
      if (outputs[i] > maxOutput) {
        maxOutput = outputs[i];
        decision = i;
      }
    }
    
    Serial.print("ANN Decision: ");
    
    // Move backward first
    Serial.println("Moving Backward");
    moveBackward();
    delay(400);
    stopMotors();
    delay(200);
    
    // Execute turn decision based on ANN
    if (decision == 1 || (decision == 0 && leftDist > rightDist)) { 
      // Turn Left if ANN suggests or if left is clearer
      Serial.println("Turning Left");
      turnLeft();
      delay(600);
    } 
    else { 
      // Turn Right if ANN suggests or if right is clearer
      Serial.println("Turning Right");
      turnRight();
      delay(600);
    }
    
    stopMotors();
    delay(200);
  } 
  else if (distance >= 30) {
    // No obstacle - move forward only if distance is safe
    Serial.println(" | Clear - Moving Forward");
    Myservo.write(90);
    moveForward();
    delay(100);
  }
  else {
    // Invalid reading - stop for safety
    stopMotors();
  }
  
  delay(50);
}