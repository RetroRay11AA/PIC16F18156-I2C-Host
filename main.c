/*
 * PIC16F18156 I2C Host Controller (Send Only)
 * Communicates with two Arduino Nano slaves via I2C
 * 7-bit addressing, single 8-bit messages (host send only)
 * Drives outputs based on input states
 * Sends DIFFERENT test messages to each Nano slave with LCD displays
 * 
 * I2C Configuration:
 * - SCL: RA0 (Clock)
 * - SDA: RA1 (Data)
 * - Master mode, 100kHz
 * 
 * Digital Inputs (3):
 * - RA2, RA3, RA4
 * 
 * Digital Outputs (8):
 * - RB0, RB1, RB2, RB3, RB4, RB5, RB6, RB7
 * 
 * Slave Devices:
 * - Nano 0x50: Slave 1 with 2004A LCD (receives message A)
 * - Nano 0x51: Slave 2 with 2004A LCD (receives message B)
 * 
 * Compiler: MPLAB X with XC8
 */

#include <xc.h>
#include <stdint.h>

// PIC16F18156 Configuration Bits
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC
#pragma config CLKOUT = OFF
#pragma config CSWEN = ON
#pragma config FCMEN = ON
#pragma config MCLRE = ON
#pragma config PWRTE = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = ON
#pragma config BORV = LO
#pragma config ZCD = OFF
#pragma config PPS1WAY = ON
#pragma config STVREN = ON
#pragma config XINST = OFF
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = OFF
#pragma config WDTWINDOW = WDTWINDOW_C
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC
#pragma config CP = OFF
#pragma config CPD = OFF

// ============================================================================
// DEFINITIONS AND CONSTANTS
// ============================================================================

#define _XTAL_FREQ 8000000  // 8MHz internal oscillator
#define I2C_SLAVE1_ADDR 0x50  // Arduino Nano slave 1 address (7-bit)
#define I2C_SLAVE2_ADDR 0x51  // Arduino Nano slave 2 address (7-bit)
#define I2C_SPEED 100         // kHz

// Digital Input Port Definitions
#define INPUT1_PIN RA2
#define INPUT2_PIN RA3
#define INPUT3_PIN RA4

// Digital Output Port Definitions
#define OUTPUT1 RB0
#define OUTPUT2 RB1
#define OUTPUT3 RB2
#define OUTPUT4 RB3
#define OUTPUT5 RB4
#define OUTPUT6 RB5
#define OUTPUT7 RB6
#define OUTPUT8 RB7

// Test Message Codes
#define TEST_0      0x00
#define TEST_1      0x01
#define TEST_2      0x02
#define TEST_3      0x03
#define TEST_PATTERN_A  0xAA
#define TEST_PATTERN_B  0x55
#define TEST_ALL_ON     0xFF

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void systemInit(void);
void i2cInit(void);
void ioInit(void);
void i2cStart(void);
void i2cStop(void);
void i2cWrite(uint8_t data);
void i2cSendByte(uint8_t slave_addr, uint8_t data);
void delay_ms(uint16_t ms);

// Input handler subroutines
void handleInput1(uint8_t state);
void handleInput2(uint8_t state);
void handleInput3(uint8_t state);
uint8_t readAllInputs(void);

// Output control subroutines
void setOutput1(uint8_t state);
void setOutput2(uint8_t state);
void setOutput3(uint8_t state);
void setOutput4(uint8_t state);
void setOutput5(uint8_t state);
void setOutput6(uint8_t state);
void setOutput7(uint8_t state);
void setOutput8(uint8_t state);
void setAllOutputs(uint8_t state);

// Message generation subroutines (separate for each slave)
uint8_t generateTestMessageSlave1(uint8_t input_state);
uint8_t generateTestMessageSlave2(uint8_t input_state);

// ============================================================================
// SYSTEM INITIALIZATION
// ============================================================================

void systemInit(void) {
    // Configure oscillator for 8MHz
    OSCCON1 = 0x60;  // 8MHz HFINTOSC
    OSCCON3 = 0x00;
    OSCEN = 0x00;
    
    // Wait for oscillator to stabilize
    while (!OSCFRQ);
    
    ioInit();
    i2cInit();
}

// ============================================================================
// I/O INITIALIZATION
// ============================================================================

void ioInit(void) {
    // Configure PORTA
    // RA0, RA1: I2C (SCL, SDA) - configured automatically by SSP module
    // RA2, RA3, RA4: Digital inputs
    TRISA = 0x1C;  // RA2, RA3, RA4 as inputs; RA0, RA1 for I2C
    ANSELA = 0x00; // All PORTA digital
    WPUA = 0x1C;   // Weak pull-ups on inputs
    
    // Configure PORTB
    // RB0-RB7: Digital outputs
    TRISB = 0x00;  // All PORTB as outputs
    ANSELB = 0x00; // All PORTB digital
    PORTB = 0x00;  // Initialize outputs to LOW
}

// ============================================================================
// I2C INITIALIZATION
// ============================================================================

void i2cInit(void) {
    // SSP Module configuration for I2C Master mode
    SSP1CON1 = 0x28;  // I2C Master mode, clock = FOSC/(4*(SSP1ADD+1))
    SSP1CON2 = 0x00;
    SSP1CON3 = 0x00;
    
    // Baud rate configuration
    // For 100kHz at 8MHz: SSP1ADD = (8000000 / (4 * 100000)) - 1 = 19
    SSP1ADD = 19;
    
    // Clear interrupt flags
    SSP1IF = 0;
    BCL1IF = 0;
    
    // Enable SSP1 module
    SSP1CON1bits.SSPEN = 1;
}

// ============================================================================
// I2C LOW-LEVEL OPERATIONS
// ============================================================================

void i2cStart(void) {
    SSP1CON2bits.SEN = 1;  // Initiate START condition
    while (SSP1CON2bits.SEN);  // Wait for START condition to complete
    while (SSP1STATbits.S == 0);  // Wait for START bit to be set
}

void i2cStop(void) {
    SSP1CON2bits.PEN = 1;  // Initiate STOP condition
    while (SSP1CON2bits.PEN);  // Wait for STOP condition to complete
}

void i2cWrite(uint8_t data) {
    SSP1BUF = data;  // Load data into SSP buffer
    while (SSP1STATbits.BF);  // Wait for buffer to be empty
    while (SSP1CON2bits.ACKSTAT);  // Wait for ACK from slave
}

// ============================================================================
// I2C HIGH-LEVEL OPERATIONS
// ============================================================================

/*
 * Send a single byte to specified I2C slave (Host sends, slave receives only)
 * slave_addr: 7-bit address of slave device
 * data: 8-bit data to send
 */
void i2cSendByte(uint8_t slave_addr, uint8_t data) {
    i2cStart();
    
    // Send slave address with write bit (LSB = 0)
    i2cWrite((slave_addr << 1) | 0x00);
    
    // Send data byte
    i2cWrite(data);
    
    i2cStop();
}

// ============================================================================
// DELAY FUNCTION
// ============================================================================

void delay_ms(uint16_t ms) {
    while (ms--) {
        __delay_us(1000);
    }
}

// ============================================================================
// INPUT HANDLER SUBROUTINES
// ============================================================================

/*
 * Handle Input 1 (RA2)
 * state: Current state of input pin (1 = HIGH, 0 = LOW)
 */
void handleInput1(uint8_t state) {
    // Input 1 processing
    // This routine is called when Input 1 changes
    if (state) {
        // Input 1 is HIGH
        setOutput1(1);
    } else {
        // Input 1 is LOW
        setOutput1(0);
    }
}

/*
 * Handle Input 2 (RA3)
 * state: Current state of input pin (1 = HIGH, 0 = LOW)
 */
void handleInput2(uint8_t state) {
    // Input 2 processing
    // This routine is called when Input 2 changes
    if (state) {
        // Input 2 is HIGH
        setOutput2(1);
    } else {
        // Input 2 is LOW
        setOutput2(0);
    }
}

/*
 * Handle Input 3 (RA4)
 * state: Current state of input pin (1 = HIGH, 0 = LOW)
 */
void handleInput3(uint8_t state) {
    // Input 3 processing
    // This routine is called when Input 3 changes
    if (state) {
        // Input 3 is HIGH
        setOutput3(1);
    } else {
        // Input 3 is LOW
        setOutput3(0);
    }
}

/*
 * Read all input states and combine into single byte
 * Bit 0 (LSB) -> Input 1 (RA2)
 * Bit 1 -> Input 2 (RA3)
 * Bit 2 -> Input 3 (RA4)
 * Bits 3-7 -> 0
 */
uint8_t readAllInputs(void) {
    uint8_t input_byte = 0x00;
    input_byte |= (INPUT1_PIN << 0);
    input_byte |= (INPUT2_PIN << 1);
    input_byte |= (INPUT3_PIN << 2);
    return input_byte;
}

// ============================================================================
// OUTPUT CONTROL SUBROUTINES
// ============================================================================

/*
 * Set Output 1 (RB0)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput1(uint8_t state) {
    OUTPUT1 = state;
}

/*
 * Set Output 2 (RB1)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput2(uint8_t state) {
    OUTPUT2 = state;
}

/*
 * Set Output 3 (RB2)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput3(uint8_t state) {
    OUTPUT3 = state;
}

/*
 * Set Output 4 (RB3)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput4(uint8_t state) {
    OUTPUT4 = state;
}

/*
 * Set Output 5 (RB4)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput5(uint8_t state) {
    OUTPUT5 = state;
}

/*
 * Set Output 6 (RB5)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput6(uint8_t state) {
    OUTPUT6 = state;
}

/*
 * Set Output 7 (RB6)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput7(uint8_t state) {
    OUTPUT7 = state;
}

/*
 * Set Output 8 (RB7)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput8(uint8_t state) {
    OUTPUT8 = state;
}

/*
 * Set all outputs to same state
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setAllOutputs(uint8_t state) {
    PORTB = state ? 0xFF : 0x00;
}

// ============================================================================
// MESSAGE GENERATION SUBROUTINES - SLAVE 1 (0x50)
// ============================================================================

/*
 * Generate test message for Slave 1 (0x50) based on input state
 * input_state: 8-bit value containing all input states
 * 
 * Slave 1 Message Mapping:
 * Input pattern 0x00 (all LOW) -> Test 0 (0x00)
 * Input pattern 0x01 (Input1 HIGH) -> Test 1 (0x01)
 * Input pattern 0x02 (Input2 HIGH) -> Test 2 (0x02)
 * Input pattern 0x03 (Input1,2 HIGH) -> Test 3 (0x03)
 * Input pattern 0x04 (Input3 HIGH) -> Test Pattern A (0xAA)
 * Input pattern 0x05 (Input1,3 HIGH) -> Test Pattern B (0x55)
 * Input pattern 0x06 (Input2,3 HIGH) -> Test All ON (0xFF)
 * Input pattern 0x07 (all HIGH) -> Custom message (0x0F)
 */
uint8_t generateTestMessageSlave1(uint8_t input_state) {
    uint8_t message = TEST_0;
    
    // Extract individual input bits
    uint8_t input1 = (input_state >> 0) & 0x01;
    uint8_t input2 = (input_state >> 1) & 0x01;
    uint8_t input3 = (input_state >> 2) & 0x01;
    
    // Generate message based on input combination for Slave 1
    if (input1 && input2 && input3) {
        // All inputs HIGH
        message = 0x0F;  // Custom message
    } else if (input2 && input3) {
        // Input 2 and 3 HIGH
        message = TEST_ALL_ON;
    } else if (input1 && input3) {
        // Input 1 and 3 HIGH
        message = TEST_PATTERN_B;
    } else if (input3) {
        // Only Input 3 HIGH
        message = TEST_PATTERN_A;
    } else if (input1 && input2) {
        // Input 1 and 2 HIGH
        message = TEST_3;
    } else if (input2) {
        // Only Input 2 HIGH
        message = TEST_2;
    } else if (input1) {
        // Only Input 1 HIGH
        message = TEST_1;
    } else {
        // All inputs LOW
        message = TEST_0;
    }
    
    return message;
}

// ============================================================================
// MESSAGE GENERATION SUBROUTINES - SLAVE 2 (0x51)
// ============================================================================

/*
 * Generate test message for Slave 2 (0x51) based on input state
 * input_state: 8-bit value containing all input states
 * 
 * Slave 2 Message Mapping (DIFFERENT from Slave 1):
 * Input pattern 0x00 (all LOW) -> Test Pattern B (0x55)
 * Input pattern 0x01 (Input1 HIGH) -> Test All ON (0xFF)
 * Input pattern 0x02 (Input2 HIGH) -> Test Pattern A (0xAA)
 * Input pattern 0x03 (Input1,2 HIGH) -> Test 0 (0x00)
 * Input pattern 0x04 (Input3 HIGH) -> Test 1 (0x01)
 * Input pattern 0x05 (Input1,3 HIGH) -> Test 2 (0x02)
 * Input pattern 0x06 (Input2,3 HIGH) -> Test 3 (0x03)
 * Input pattern 0x07 (all HIGH) -> Custom message (0xF0)
 */
uint8_t generateTestMessageSlave2(uint8_t input_state) {
    uint8_t message = TEST_0;
    
    // Extract individual input bits
    uint8_t input1 = (input_state >> 0) & 0x01;
    uint8_t input2 = (input_state >> 1) & 0x01;
    uint8_t input3 = (input_state >> 2) & 0x01;
    
    // Generate message based on input combination for Slave 2 (DIFFERENT mapping)
    if (input1 && input2 && input3) {
        // All inputs HIGH
        message = 0xF0;  // Different custom message than Slave 1
    } else if (input2 && input3) {
        // Input 2 and 3 HIGH
        message = TEST_3;
    } else if (input1 && input3) {
        // Input 1 and 3 HIGH
        message = TEST_2;
    } else if (input3) {
        // Only Input 3 HIGH
        message = TEST_1;
    } else if (input1 && input2) {
        // Input 1 and 2 HIGH
        message = TEST_0;
    } else if (input2) {
        // Only Input 2 HIGH
        message = TEST_PATTERN_A;
    } else if (input1) {
        // Only Input 1 HIGH
        message = TEST_ALL_ON;
    } else {
        // All inputs LOW
        message = TEST_PATTERN_B;
    }
    
    return message;
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

void main(void) {
    uint8_t input_state;
    uint8_t message_data_slave1;
    uint8_t message_data_slave2;
    uint8_t previous_input_state = 0xFF;  // Initialize to different value
    
    systemInit();
    
    // Display startup indication on outputs
    setAllOutputs(1);
    delay_ms(500);
    setAllOutputs(0);
    delay_ms(500);
    
    // Main loop
    while (1) {
        // Read all digital inputs
        input_state = readAllInputs();
        
        // Process inputs only if they changed
        if (input_state != previous_input_state) {
            previous_input_state = input_state;
            
            // Handle individual input changes
            handleInput1((input_state >> 0) & 0x01);
            handleInput2((input_state >> 1) & 0x01);
            handleInput3((input_state >> 2) & 0x01);
            
            // Generate DIFFERENT test messages for each slave based on input state
            message_data_slave1 = generateTestMessageSlave1(input_state);
            message_data_slave2 = generateTestMessageSlave2(input_state);
            
            // Send test message to Slave 1 (0x50)
            i2cSendByte(I2C_SLAVE1_ADDR, message_data_slave1);
            delay_ms(10);
            
            // Send DIFFERENT test message to Slave 2 (0x51)
            i2cSendByte(I2C_SLAVE2_ADDR, message_data_slave2);
            delay_ms(10);
        }
        
        // Periodic message refresh even if inputs unchanged
        // This ensures LCDs stay synchronized
        delay_ms(200);
        
        // Periodically resend DIFFERENT messages to slaves
        message_data_slave1 = generateTestMessageSlave1(input_state);
        message_data_slave2 = generateTestMessageSlave2(input_state);
        
        i2cSendByte(I2C_SLAVE1_ADDR, message_data_slave1);
        delay_ms(10);
        
        i2cSendByte(I2C_SLAVE2_ADDR, message_data_slave2);
        delay_ms(10);
    }
}
