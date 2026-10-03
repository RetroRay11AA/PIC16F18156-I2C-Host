/*
 * Arduino Nano I2C Slave (Address 0x50)
 * Receives single 8-bit messages from PIC16F18156 host
 * Drives 2004A LCD display (20x4 parallel interface)
 * Displays splash screen on startup for 10 seconds
 * 
 * I2C Connections:
 * - SDA: A4 (Analog pin 4)
 * - SCL: A5 (Analog pin 5)
 * - GND: GND
 * 
 * LCD Connections (Parallel Interface):
 * - RS (Register Select): Pin 7
 * - E (Enable): Pin 6
 * - D4 (Data 4): Pin 5
 * - D5 (Data 5): Pin 4
 * - D6 (Data 6): Pin 3
 * - D7 (Data 7): Pin 2
 * - VSS (GND): GND
 * - VCC (5V): 5V
 * - V0 (Contrast): 10k Potentiometer to GND
 * - RW (Read/Write): GND (Write only)
 * - A (Backlight Anode): 5V (through 220 Ohm resistor)
 * - K (Backlight Cathode): GND
 * 
 * Compiler: Arduino IDE
 */

#include <Wire.h>
#include <LiquidCrystal.h>
#include <stdint.h>

// I2C Slave Address (7-bit)
#define SLAVE_ADDRESS 0x50

// LCD Pin Definitions (4-bit mode)
#define LCD_RS 7
#define LCD_E 6
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2

// Define pins for outputs (8 digital outputs)
#define OUTPUT1 8
#define OUTPUT2 9
#define OUTPUT3 A0
#define OUTPUT4 A1
#define OUTPUT5 A2
#define OUTPUT6 A3
#define OUTPUT7 12
#define OUTPUT8 13

// Define pins for inputs (3 digital inputs)
#define INPUT1 10
#define INPUT2 11
#define INPUT3 A6

// Splash screen duration (milliseconds)
#define SPLASH_SCREEN_DURATION 10000

// Create LCD object (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Global variable to store received data
volatile uint8_t received_data = 0x00;
volatile uint8_t data_received_flag = 0;

// ============================================================================
// SETUP - Run once at startup
// ============================================================================

void setup() {
    // Initialize Serial for debugging (optional)
    Serial.begin(9600);
    delay(100);
    Serial.println("Arduino Nano Slave 0x50 - Starting...");
    
    // Configure digital outputs
    pinMode(OUTPUT1, OUTPUT);
    pinMode(OUTPUT2, OUTPUT);
    pinMode(OUTPUT3, OUTPUT);
    pinMode(OUTPUT4, OUTPUT);
    pinMode(OUTPUT5, OUTPUT);
    pinMode(OUTPUT6, OUTPUT);
    pinMode(OUTPUT7, OUTPUT);
    pinMode(OUTPUT8, OUTPUT);
    
    // Configure digital inputs with pull-ups
    pinMode(INPUT1, INPUT_PULLUP);
    pinMode(INPUT2, INPUT_PULLUP);
    pinMode(INPUT3, INPUT_PULLUP);
    
    // Initialize all outputs to LOW
    setAllOutputs(0x00);
    
    // Initialize LCD (20 columns, 4 rows)
    lcd.begin(20, 4);
    
    // Display splash screen for 10 seconds
    displaySplashScreen();
    
    // Initialize I2C as Slave
    Wire.begin(SLAVE_ADDRESS);
    
    // Register I2C event handlers
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    
    Serial.println("I2C Slave initialized at address 0x50");
    Serial.println("LCD initialized - 20x4 display ready");
}

// ============================================================================
// SPLASH SCREEN DISPLAY
// ============================================================================

void displaySplashScreen() {
    unsigned long splash_start_time = millis();
    unsigned long splash_elapsed_time = 0;
    
    // Clear LCD and display splash screen
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("  PIC I2C System  ");
    
    lcd.setCursor(0, 1);
    lcd.print("    Slave 0x50    ");
    
    lcd.setCursor(0, 2);
    lcd.print("   2004A LCD      ");
    
    lcd.setCursor(0, 3);
    lcd.print(" Initializing...  ");
    
    // Display splash screen for 10 seconds
    while (splash_elapsed_time < SPLASH_SCREEN_DURATION) {
        splash_elapsed_time = millis() - splash_start_time;
        delay(100);  // Update display every 100ms for responsiveness
    }
    
    // Clear LCD after splash screen
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ready");
    delay(500);
    lcd.clear();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Check if data was received from host
    if (data_received_flag) {
        data_received_flag = 0;
        
        Serial.print("Data received: 0x");
        Serial.println(received_data, HEX);
        
        // Process received data
        processReceivedData(received_data);
    }
    
    // Read input states (optional - for debugging)
    uint8_t input1_state = digitalRead(INPUT1);
    uint8_t input2_state = digitalRead(INPUT2);
    uint8_t input3_state = digitalRead(INPUT3);
    
    // Add any other periodic tasks here
    
    delay(50);
}

// ============================================================================
// I2C EVENT HANDLERS
// ============================================================================

/*
 * Called when I2C data is received from host
 * num_bytes: Number of bytes received
 */
void receiveEvent(int num_bytes) {
    // Read all available bytes from I2C buffer
    while (Wire.available()) {
        received_data = Wire.read();  // Read one byte at a time
    }
    
    // Set flag indicating data was received
    data_received_flag = 1;
}

/*
 * Called when host requests data from slave
 * (Not used in send-only mode, but included for completeness)
 */
void requestEvent() {
    // In send-only mode, this should not be called
    // But if it is, send a dummy byte back
    Wire.write(0x00);
}

// ============================================================================
// DATA PROCESSING - Map message byte to LCD test
// ============================================================================

/*
 * Process the received 8-bit message
 * Display corresponding test on LCD and control outputs accordingly
 * 
 * Message codes:
 * 0x00 - Test 0: Display "Test 0" on line 0
 * 0x01 - Test 1: Display "Test 1" on line 1
 * 0x02 - Test 2: Display "Test 2" on line 2
 * 0x03 - Test 3: Display "Test 3" on line 3
 * 0xAA - Test A: Display "Test Pattern A" on all lines
 * 0x55 - Test B: Display "Test Pattern B" on all lines
 * 0xFF - Test C: Display "All Outputs ON"
 * etc.
 */
void processReceivedData(uint8_t data) {
    // Extract individual bits and drive outputs
    uint8_t bit0 = (data >> 0) & 0x01;
    uint8_t bit1 = (data >> 1) & 0x01;
    uint8_t bit2 = (data >> 2) & 0x01;
    uint8_t bit3 = (data >> 3) & 0x01;
    uint8_t bit4 = (data >> 4) & 0x01;
    uint8_t bit5 = (data >> 5) & 0x01;
    uint8_t bit6 = (data >> 6) & 0x01;
    uint8_t bit7 = (data >> 7) & 0x01;
    
    // Set output pins based on bit states
    setOutput1(bit0);
    setOutput2(bit1);
    setOutput3(bit2);
    setOutput4(bit3);
    setOutput5(bit4);
    setOutput6(bit5);
    setOutput7(bit6);
    setOutput8(bit7);
    
    // Display test message on LCD based on received data
    lcd.clear();
    
    switch (data) {
        case 0x00:
            lcd.setCursor(0, 0);
            lcd.print("Test 0");
            lcd.setCursor(0, 1);
            lcd.print("Outputs: 00000000");
            break;
            
        case 0x01:
            lcd.setCursor(0, 0);
            lcd.print("Test 1");
            lcd.setCursor(0, 1);
            lcd.print("Output 1 ON");
            break;
            
        case 0x02:
            lcd.setCursor(0, 0);
            lcd.print("Test 2");
            lcd.setCursor(0, 1);
            lcd.print("Output 2 ON");
            break;
            
        case 0x03:
            lcd.setCursor(0, 0);
            lcd.print("Test 3");
            lcd.setCursor(0, 1);
            lcd.print("Outputs 1,2 ON");
            break;
            
        case 0xAA:
            lcd.setCursor(0, 0);
            lcd.print("Test Pattern A");
            lcd.setCursor(0, 1);
            lcd.print("Data: 0xAA");
            lcd.setCursor(0, 2);
            lcd.print("Binary: 10101010");
            lcd.setCursor(0, 3);
            lcd.print("Slave 0x50");
            break;
            
        case 0x55:
            lcd.setCursor(0, 0);
            lcd.print("Test Pattern B");
            lcd.setCursor(0, 1);
            lcd.print("Data: 0x55");
            lcd.setCursor(0, 2);
            lcd.print("Binary: 01010101");
            lcd.setCursor(0, 3);
            lcd.print("Slave 0x50");
            break;
            
        case 0xFF:
            lcd.setCursor(0, 0);
            lcd.print("Test All ON");
            lcd.setCursor(0, 1);
            lcd.print("All 8 Outputs ON");
            lcd.setCursor(0, 2);
            lcd.print("Data: 0xFF");
            lcd.setCursor(0, 3);
            lcd.print("Slave 0x50");
            break;
            
        default:
            lcd.setCursor(0, 0);
            lcd.print("Custom Test");
            lcd.setCursor(0, 1);
            lcd.print("Data: 0x");
            if (data < 16) lcd.print("0");
            lcd.print(data, HEX);
            lcd.setCursor(0, 2);
            lcd.print("Binary: ");
            printBinary(data);
            lcd.setCursor(0, 3);
            lcd.print("Slave 0x50");
            break;
    }
}

/*
 * Print an 8-bit value in binary format on LCD
 */
void printBinary(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
        lcd.print((value >> i) & 0x01);
    }
}

// ============================================================================
// OUTPUT CONTROL FUNCTIONS
// ============================================================================

/*
 * Set Output 1 (Pin 8)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput1(uint8_t state) {
    digitalWrite(OUTPUT1, state ? HIGH : LOW);
}

/*
 * Set Output 2 (Pin 9)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput2(uint8_t state) {
    digitalWrite(OUTPUT2, state ? HIGH : LOW);
}

/*
 * Set Output 3 (Pin A0)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput3(uint8_t state) {
    digitalWrite(OUTPUT3, state ? HIGH : LOW);
}

/*
 * Set Output 4 (Pin A1)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput4(uint8_t state) {
    digitalWrite(OUTPUT4, state ? HIGH : LOW);
}

/*
 * Set Output 5 (Pin A2)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput5(uint8_t state) {
    digitalWrite(OUTPUT5, state ? HIGH : LOW);
}

/*
 * Set Output 6 (Pin A3)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput6(uint8_t state) {
    digitalWrite(OUTPUT6, state ? HIGH : LOW);
}

/*
 * Set Output 7 (Pin 12)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput7(uint8_t state) {
    digitalWrite(OUTPUT7, state ? HIGH : LOW);
}

/*
 * Set Output 8 (Pin 13)
 * state: 0 = LOW, 1 = HIGH
 */
void setOutput8(uint8_t state) {
    digitalWrite(OUTPUT8, state ? HIGH : LOW);
}

/*
 * Set all outputs to the same state
 * state: 0 = LOW, 1 = HIGH
 */
void setAllOutputs(uint8_t state) {
    digitalWrite(OUTPUT1, state ? HIGH : LOW);
    digitalWrite(OUTPUT2, state ? HIGH : LOW);
    digitalWrite(OUTPUT3, state ? HIGH : LOW);
    digitalWrite(OUTPUT4, state ? HIGH : LOW);
    digitalWrite(OUTPUT5, state ? HIGH : LOW);
    digitalWrite(OUTPUT6, state ? HIGH : LOW);
    digitalWrite(OUTPUT7, state ? HIGH : LOW);
    digitalWrite(OUTPUT8, state ? HIGH : LOW);
}

// ============================================================================
// INPUT READING FUNCTIONS (Optional - for debugging)
// ============================================================================

/*
 * Read Input 1 (Pin 10)
 * Returns: 0 = LOW, 1 = HIGH
 */
uint8_t readInput1(void) {
    return digitalRead(INPUT1);
}

/*
 * Read Input 2 (Pin 11)
 * Returns: 0 = LOW, 1 = HIGH
 */
uint8_t readInput2(void) {
    return digitalRead(INPUT2);
}

/*
 * Read Input 3 (Pin A6)
 * Returns: 0 = LOW, 1 = HIGH
 */
uint8_t readInput3(void) {
    return digitalRead(INPUT3);
}

/*
 * Get all input states combined into one byte
 * Bit 0 -> Input 1
 * Bit 1 -> Input 2
 * Bit 2 -> Input 3
 * Bits 3-7 -> 0
 */
uint8_t readAllInputs(void) {
    uint8_t input_byte = 0x00;
    input_byte |= (readInput1() << 0);
    input_byte |= (readInput2() << 1);
    input_byte |= (readInput3() << 2);
    return input_byte;
}
