/*
 * PIC16F18156 I2C Host Controller
 * Communicates with two Arduino Nano slaves via I2C
 * 7-bit addressing, single 8-bit messages
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
#define I2C_SLAVE1_ADDR 0x50  // First Arduino Nano slave address (7-bit)
#define I2C_SLAVE2_ADDR 0x51  // Second Arduino Nano slave address (7-bit)
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

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void systemInit(void);
void i2cInit(void);
void ioInit(void);
void i2cStart(void);
void i2cStop(void);
void i2cWrite(uint8_t data);
uint8_t i2cRead(uint8_t ack);
void i2cSendByte(uint8_t slave_addr, uint8_t data);
uint8_t i2cReceiveByte(uint8_t slave_addr);
void delay_ms(uint16_t ms);

// Input handler subroutines
void handleInput1(uint8_t state);
void handleInput2(uint8_t state);
void handleInput3(uint8_t state);

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

uint8_t i2cRead(uint8_t ack) {
    uint8_t data;
    
    SSP1CON2bits.RCEN = 1;  // Enable receive mode
    while (SSP1STATbits.BF == 0);  // Wait for buffer to fill
    data = SSP1BUF;  // Read data from buffer
    
    if (ack) {
        SSP1CON2bits.ACKDT = 0;  // Send ACK
    } else {
        SSP1CON2bits.ACKDT = 1;  // Send NACK
    }
    SSP1CON2bits.ACKEN = 1;  // Initiate ACK/NACK sequence
    while (SSP1CON2bits.ACKEN);  // Wait for ACK/NACK to complete
    
    return data;
}

// ============================================================================
// I2C HIGH-LEVEL OPERATIONS
// ============================================================================

/*
 * Send a single byte to specified I2C slave
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

/*
 * Receive a single byte from specified I2C slave
 * slave_addr: 7-bit address of slave device
 * Returns: 8-bit data received from slave
 */
uint8_t i2cReceiveByte(uint8_t slave_addr) {
    uint8_t data;
    
    i2cStart();
    
    // Send slave address with read bit (LSB = 1)
    i2cWrite((slave_addr << 1) | 0x01);
    
    // Read data byte (NACK = 0 means send ACK after read)
    data = i2cRead(0);
    
    i2cStop();
    
    return data;
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
// INPUT HANDLER SUBROUTINES (Empty - To be implemented by user)
// ============================================================================

/*
 * Handle Input 1 (RA2)
 * state: Current state of input pin (1 = HIGH, 0 = LOW)
 */
void handleInput1(uint8_t state) {
    // TODO: Implement input 1 handling logic
    // Example: Send state to slave via I2C
    // i2cSendByte(I2C_SLAVE1_ADDR, state);
}

/*
 * Handle Input 2 (RA3)
 * state: Current state of input pin (1 = HIGH, 0 = LOW)
 */
void handleInput2(uint8_t state) {
    // TODO: Implement input 2 handling logic
}

/*
 * Handle Input 3 (RA4)
 * state: Current state of input pin (1 = HIGH, 0 = LOW)
 */
void handleInput3(uint8_t state) {
    // TODO: Implement input 3 handling logic
}

// ============================================================================
// OUTPUT CONTROL SUBROUTINES (Empty - To be implemented by user)
// ============================================================================

/*
 * Set Output 1 (RB0)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput1(uint8_t state) {
    // TODO: Implement output 1 control logic
    OUTPUT1 = state;
}

/*
 * Set Output 2 (RB1)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput2(uint8_t state) {
    // TODO: Implement output 2 control logic
    OUTPUT2 = state;
}

/*
 * Set Output 3 (RB2)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput3(uint8_t state) {
    // TODO: Implement output 3 control logic
    OUTPUT3 = state;
}

/*
 * Set Output 4 (RB3)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput4(uint8_t state) {
    // TODO: Implement output 4 control logic
    OUTPUT4 = state;
}

/*
 * Set Output 5 (RB4)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput5(uint8_t state) {
    // TODO: Implement output 5 control logic
    OUTPUT5 = state;
}

/*
 * Set Output 6 (RB5)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput6(uint8_t state) {
    // TODO: Implement output 6 control logic
    OUTPUT6 = state;
}

/*
 * Set Output 7 (RB6)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput7(uint8_t state) {
    // TODO: Implement output 7 control logic
    OUTPUT7 = state;
}

/*
 * Set Output 8 (RB7)
 * state: Output state (1 = HIGH, 0 = LOW)
 */
void setOutput8(uint8_t state) {
    // TODO: Implement output 8 control logic
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
// MAIN PROGRAM
// ============================================================================

void main(void) {
    uint8_t input_state;
    uint8_t slave1_data;
    uint8_t slave2_data;
    
    systemInit();
    
    // Main loop
    while (1) {
        // Read digital inputs
        input_state = INPUT1_PIN;
        handleInput1(input_state);
        
        input_state = INPUT2_PIN;
        handleInput2(input_state);
        
        input_state = INPUT3_PIN;
        handleInput3(input_state);
        
        // Example: Send test data to Slave 1
        i2cSendByte(I2C_SLAVE1_ADDR, 0xAA);
        delay_ms(10);
        
        // Example: Read data from Slave 1
        slave1_data = i2cReceiveByte(I2C_SLAVE1_ADDR);
        delay_ms(10);
        
        // Example: Send test data to Slave 2
        i2cSendByte(I2C_SLAVE2_ADDR, 0x55);
        delay_ms(10);
        
        // Example: Read data from Slave 2
        slave2_data = i2cReceiveByte(I2C_SLAVE2_ADDR);
        delay_ms(10);
        
        // Loop delay
        delay_ms(100);
    }
}
