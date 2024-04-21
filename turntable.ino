#include <AccelStepper.h>
#include <ezButton.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 3 //D1
#define IN2 1 //D2
#define IN3 0 //D3
#define IN4 2 //D4

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

const int CLK = 14;
const int DT = 12;
const int SW = 13;

int encoderValue = 0;
int lastCLKState;
int lastDTState;
bool buttonPressed = false;
bool timerRunning = false;
unsigned long countdownStartTime;
unsigned long countdownDuration = 0;

bool speedWasUpdated = false;
bool isMoving = true;

int MIN_SPEED = 200;
int MAX_SPEED = 1200;

int currentSpeed = MIN_SPEED;

bool isDispClear = false;
unsigned long last_disp_time;
int disp_update_interval = 500;

unsigned long last_clear_disp_time;
int clear_disp_interval = 5000;

// initialize the stepper library
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);
ezButton button(SW, INPUT_PULLUP);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  //GPIO 1 (TX) swap the pin to a GPIO.
  //pinMode(1, FUNCTION_3); 
  //GPIO 3 (RX) swap the pin to a GPIO.
  //pinMode(3, FUNCTION_3); 

  // initialize the serial port
  //Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();

  delay(2000); // Pause for 2 seconds

  button.setDebounceTime(50); 

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  lastCLKState = digitalRead(CLK);
  lastDTState = digitalRead(DT);
  
  // set the speed and acceleration
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(500);
  stepper.setSpeed(currentSpeed);
  // set target position
  stepper.move(stepsPerRevolution);

  last_clear_disp_time = millis();
}

void loop() {
  if (stepper.distanceToGo() == 0){
    stepper.move(stepsPerRevolution);
    Serial.println("Loop");
  }
  
  // if (stepper.distanceToGo() >= stepsPerRevolution/2) {
  //   stepper.runSpeed();
  // } else {
  //   stepper.run();
  // }

  if(isMoving) {
    stepper.setSpeed(currentSpeed);
    stepper.runSpeed();
  } else {
    stepper.stop();
  }

  int currentStateCLK = digitalRead(CLK);
  int currentStateDT = digitalRead(DT);

  if (currentStateCLK != lastCLKState) {
    if (currentStateCLK == HIGH) {
      // On rising edge of CLK
      if (currentStateDT == LOW) {
        // Clockwise rotation
        updateSpeed(10);
      } else {
        // Counter-clockwise rotation
        updateSpeed(-10);
      }
      
      Serial.print("Encoder Value: ");
      Serial.println(encoderValue);
    }
  }

  lastCLKState = currentStateCLK;
  lastDTState = currentStateDT;

  button.loop(); // MUST call the loop() function first

  if(button.isPressed()) {
    Serial.println("Button click");
    isMoving = !isMoving;
  }

  if (millis() - last_disp_time >= disp_update_interval) {
    last_disp_time = millis();

    drawState();
  }

  if (millis() - last_clear_disp_time >= clear_disp_interval) {
    isDispClear = true;
  }
}

void updateSpeed(int value) {
  currentSpeed += value;

  currentSpeed = min(max(MIN_SPEED, currentSpeed), MAX_SPEED);

  wakeUpDisplay();

  Serial.print("Speed: ");
  Serial.println(currentSpeed);
}

void drawState() {
  // Clear the buffer
  display.clearDisplay();

  if(!isDispClear) {
    float dist = float(currentSpeed) - float(MIN_SPEED);
    float totalDist = float(MAX_SPEED) - float(MIN_SPEED);
    float progress = dist/totalDist;

    display.drawRect(1, 1, 122, 10, WHITE);
    display.fillRect(1, 1, int(113 * float(dist/totalDist)), 10, WHITE);

    display.drawRect(1, 20, 10, 10, WHITE);

    if(isMoving) {
      display.fillRect(3, 22, 6, 6, WHITE);
    }

    String stringOne = "";
    stringOne += String(currentSpeed);

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(stringOne, 0, 0, &x1, &y1, &w, &h);

    display.setCursor(123 - w, 20);
    display.print(stringOne);
  }

  display.display();
}

void wakeUpDisplay() {
  last_clear_disp_time = millis();

  isDispClear = false;
}