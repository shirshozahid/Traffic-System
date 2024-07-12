/*
 * I2C Routines
 * 
 * These routines utilise the I2C bus to communicate through a PCF8574
 * expansion chip to communicate with a 16x2 HD44780 controlled LCD display
 * 
 * There are three sections in this library file.
 * The low level I2C routines used to talk to the I2C bus directly.
 * The low level PCF8574 routines, used to send data using the requirements of
 *  the PCF8574 controller chip
 * The high level HD44780 routines which talk to the LCD controller to
 *  manipulate the display
 */

#include <xc.h>
#include <stdint.h>


#define I2C_READ    1
#define I2C_WRITE   0

#define I2C_LCD_BACKLIGHT   8
#define I2C_LCD_ENABLE      4
#define I2C_LCD_RW          2
#define I2C_LCD_RS          1

/**
 * Set up the I2C bus. We need to initialise the default pullups
 * on the two pins (Port C pins 4 and 5) and set the baud rate for the bus.
 * 
 * The baud rate is calculated by the formula:
 * SCL = Fosc / (16 + 2(TWBR).(TWPS[1:0]))
 *     = 16,000,000 / (16 + 2(TWBR).(TWPS[1:0]))
 *
 * We want an I2C clock of about 40kHz. We pick this, as it is slow
 * enough to allow the vagaries of jumper wires. We could run faster
 * If the wiring was printed circuitry and shielded.
 * 
 * 40,000 = 16,000,000 / (16 + 2(TWBR).(TWPS[1:0]))
 *
 * (16 + 2(TWBR).(TWPS[1:0])) = 16,000,000 / 40,000
 *
 * (16 + 2(TWBR).(TWPS[1:0])) = 400
 *
 * 2(TWBR).(TWPS[1:0]) = 400 - 16
 *
 * 2(TWBR).(TWPS[1:0]) = 386
 * 
 * (TWBR).(TWPS[1:0]) = 193
 * 
 */
void setup_I2C() {
    /*
     * I2C needs pullup resistors on the lines.
     * We will use the internal pullup resistors to provide this function.
     */
    DDRC &= 0x0f;   // Port C pins 4 and 5 set to input
    PORTC |= 0x30;  // Port C pins 4 and 5 set to input with pullups
    TWBR = 193;
    TWSR = 0;
}

/**
 * Wait for the current I2C operation to finish.
 * This possibly allows the processor to wait forever, but if the I2C bus
 * is enabled it will eventually finish.
 */
void I2C_wait() {
    while ((TWCR & _BV(TWINT)) == 0) {
        ;
    }
}

/**
 * Send an I2C start bit.
 * 
 * @return true if the start bit was successfully transmitted
 */
int I2C_Start() {
    // Send I2C Start flag
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x08);
}

/**
 * Send an I2C address byte and R/W flag
 * 
 * @param addr I2C address of the slave
 * @param rw whether to read or write: 0 to write, 1 to read
 * @return true if the address byte was successfully transmitted
 */
int I2C_SLA(uint8_t addr, uint8_t rw) {
    // Send I2C slave address
    TWDR = (addr << 1) | (rw & 1);
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x18);
}

/**
 * Send a byte of data through the I2C bus
 * 
 * @param data data to transmit
 * @return true if the data was successfully transmitted
 */
int I2C_Send(uint8_t data) {
    // Send I2C data byte
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Send the stop flag on the I2C bus
 */
void I2C_Stop() {
    // Send I2C Stop flag
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
    for (volatile long x = 0; x < 100; x++) {
        ;
    }
}

/**
 * Check if there is a device on the I2C bus at address
 * 
 * @param addr address to check if a device exists there.
 * @return true if a device acknowledges the address probe
 */
int I2C_CheckAddress(uint8_t addr) {
    int ret;
    
    ret = I2C_Start() & I2C_SLA(addr, I2C_WRITE);
    if (ret) {
        I2C_Stop();
    }
    return ret;
}

/**
 * Send four bits of data to a PCF8574 controlled HD44780 LCD display
 * We need to toggle the E bit (bit 2) from high to low to transmit the data
 * 
 * The 8 bits transmitted are:
 * bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 * DB7  DB6  DB5  DB4  BL   E    R/W  RS
 * BL is the back light (1 = on, 0 = off)
 * E is the enable bit (high to low transition latches the data
 * R/W is the read/write line (1 = read, 0 = write)
 * RS is Register Select (0 = control, 1 = data)
 * 
 * @param data the data to transmit
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Nibble(uint8_t data) {
    TWDR = data | I2C_LCD_ENABLE;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    if ((TWSR & 0xf8) == 0x28) {
        TWDR = data & (~I2C_LCD_ENABLE);
        TWCR = _BV(TWINT) | _BV(TWEN);
        I2C_wait();
    }
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Transmit the 8 bits of data as two four bit nibbles to a HD44780 LCD
 * controller in 4 bit mode attached through a PCF8574 port expander.
 * 
 * The byte is transmitted as the top nibble then the bottom nibble with
 * the bottom four bits being the control flags.
 * 
 * @param data 8 bits of data to transmit
 * @param flags 4 bits if flags
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Byte(uint8_t data, uint8_t flags) {
    return I2C_PCF8574_LCD_Nibble ((data & 0xf0) | (flags & 0x0f)) && 
    I2C_PCF8574_LCD_Nibble (((data << 4) & 0xf0) | (flags & 0x0f));
}

/**
 * Send multiple bytes of data to the LCD display
 * 
 * @param addr address of the display
 * @param array pointer to a char array of data to transmit
 * @param len number of bytes in the array
 * @param flags the flags to transmit as the lower 4 bits
 */
void I2C_SendData(uint8_t addr, uint8_t *array, uint8_t len, uint8_t flags) {
    if (I2C_Start() & I2C_SLA(addr, I2C_WRITE)) {
        while (len > 0) {
            len--;
            if (I2C_PCF8574_LCD_Byte(*array++, flags) == 0) {
                break;  // bad send
            }
        }
    }
    I2C_Stop(); 
}

/**
 * Send the initialisation string for a HD44780 LCD controller connected in
 * 4 bit mode. Taken from the data sheet. Transmit 0x30 three times to ensure
 * it is in 8 bit mode, then 0x20 to switch to 4 bit mode.
 * We then turn on the blinking cursor, backlight and clear the display.
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_PCF8574_Setup(uint8_t addr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        I2C_Stop();
        return -1;
    }
    I2C_Send(0);    // ensure the PCF8574 enable line is low
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_PCF8574_LCD_Nibble(0x20);
    I2C_PCF8574_LCD_Byte(0x0f, I2C_LCD_BACKLIGHT);   // display on, cursor on and blinking
    I2C_PCF8574_LCD_Byte(0x01, I2C_LCD_BACKLIGHT);   // clear and move home
    I2C_Stop();
    return 0;
}

/**
 * Clear the LCD display (and return the cursor to the home position
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_clear(uint8_t addr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x01, I2C_LCD_BACKLIGHT);   // clear screen command
    I2C_Stop();
    return 0;
}

/**
 * Set the cursor position on the LCD display
 * See the data sheet for mappings of position values to screen locations.
 * (0x00 top left, 0x40 second row left)
 * 
 * @param addr address of the LCD display
 * @param posn Location to where the cursor should be positioned
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Position(uint8_t addr, uint8_t posn) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x80 | posn, I2C_LCD_BACKLIGHT);   // set DRAM address
    I2C_Stop();
    return 0;
}

/**
 * Write a string to the LCD display
 * 
 * @param addr address of the LCD display
 * @param str pointer to a character string to display
 * @param len length of the string to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Write(uint8_t addr, char *str, uint8_t len) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    while (len--) {
        I2C_PCF8574_LCD_Byte(*str++, I2C_LCD_BACKLIGHT | I2C_LCD_RS);
    }
    I2C_Stop();
    return 0;
}

/**
 * Write a character to the LCD display
 * 
 * @param addr address of the LCD display
 * @param chr character to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Write_Chr(uint8_t addr, char chr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(chr, I2C_LCD_BACKLIGHT | I2C_LCD_RS);
    I2C_Stop();
    return 0;
}

/**
 * Setup the LCD display
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t setup_LCD(uint8_t addr) {
    if (LCD_PCF8574_Setup(addr) != 0) {
        return -1;
    }
    LCD_clear(addr);
    return 0;
}
