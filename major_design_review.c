/*
 * File:   major_design_review.c
 * Author: shirsho
 *
 * Created on April 29, 2024, 10:27 AM
 */


#include <avr/io.h>
#include <xc.h>
#include <avr/interrupt.h>
#include "ELEC3042_SPI.h"
#include "ELEC3042_I2C_PCF8574.h"

enum PHASE {HZD, RPS_Red, RPS_Yellow, RPS_Green, RPR_NB, RPR_NB_Yellow, RPR_NB_Red,
            RPR_SB, RPR_SB_Yellow, RPR_SB_Red, RPRG, RPRR, RPRY, ESTR, ESTY, ESTG, EST_AR};

volatile uint32_t clock_count = 0;
volatile uint8_t button = 0;
uint16_t ADC_value;

ISR(INT0_vect) {
    button = 1;   
}

uint32_t millis() {
    register uint32_t count;
    register char cSREG;
    
    cSREG = SREG;
    cli();
    count = clock_count;
    SREG = cSREG;
    return count;
}


ISR(ADC_vect) {
    ADC_value = ADC;  
}

ISR(TIMER2_COMPA_vect) {
    clock_count++;
}

void write_LEDs(uint16_t leds) {
    SPI_Send_Command(0x14, (leds & 0xff));
    SPI_Send_Command(0x15, (leds >> 8) & 0xff);
}

void setupTimer2() {
    TCCR2B = 0;             // turn off counter to configure
    TCNT2 = 0;              // set current count to zero
    OCR2A = 125;            // 125 * 128 * 1000 = 16000000
    TIFR2 = 0b00000111;     // clear all existing interrupts
    TIMSK2 = 0b00000010;    // enable interrupt on OCRA
    ASSR = 0;               // no async counting
    TCCR2A = 0b00000010;    // No I/O, mode = 2 (CTC)
    TCCR2B = 0b00000101;    // clock/128, mode = 2 (CTC), start counter
}

void setup() {
    DDRB = 0b00101110;
    PORTB = 0b00111101;
    DDRC = 0b00001110;
    PORTC = 0b11110001;
    DDRD = 0b10110000;
    PORTD = 0b01001100;
    sei();
}

void setupAtoD() {    
    DDRC &= 0b11111110;
    PORTC &= 0b11111110;
    ADMUX = 0b01000000; //AVcc with external cap at AREF, 0 for right adjust, 0000 for ADC0
    ADCSRA = 0b11111110; //enabled, start, auto, flag 1, interrupt 1, 64 prescaler
    ADCSRB = 0b00000000; //  free running
    DIDR0 |= 0b00000001;//Pin0 disabled
}

uint16_t current_atod() {
    uint16_t tmp;
    uint8_t sreg_save = SREG;
    cli();
    tmp = ADC_value;
    SREG = sreg_save;
    return tmp;
}

uint16_t get_atod () {
    uint16_t new_time;
    new_time = current_atod();
    if(new_time < 50) {
        return 50;
    } else if(new_time > 999) {
        return 1000;
    }
    return new_time;
}

int main(void) {
    char strHazard[] = " HZD            ";
    char strRPSG[] = " RPSG           ";
    char strRPSY[] = " RPSY           ";
    char strRPSR[] = " RPSR           ";
    char str4[] = "NRPSG           ";
    char str5[] = "NRPSGRPRG       ";
    char str17[] = "NRPSGRPRY       ";
    char str18[] = "NRPSGRPRR       ";
    char str7[] = "SRPSG           ";
    char str8[] = " ESTG           ";
    char str9[] = " ESTY           ";
    char str10[] = " ESTR           ";
    char str11[] = " RPRR           ";
    char str12[] = " RPRY           ";
    char str13[] = " RPRG           ";
    char str14[] = "SRPSGRPRY       ";
    char str15[] = "SRPSGRPRR       ";
    char str16[] = "SRPSGRPRG       ";
    char strInitial[] = "_______";
    setup();
    setupTimer2();
    setup_SPI();
    setup_I2C();
    setup_LCD(0x27);
    setup_PortExpander();
    setupAtoD();
    uint8_t GPA_Byte = 0;
    uint8_t GPB_Byte = 0;
    uint32_t interval = 0;
    uint8_t S0 = 0;
    uint8_t S1 = 0;
    uint8_t S2 = 0;
    uint8_t S3 = 0;
    uint8_t S4 = 0;
    uint8_t S5 = 0;
    uint8_t S6 = 0;
    uint8_t time_period = 0;
    uint32_t lcd_tp = 0;
    uint32_t last_tick = 0;
    uint32_t current_tick;
    uint16_t pot;
    uint32_t ten_seconds = millis() + 10000;
    uint32_t button_valueS6 = 0b00000001;
    uint32_t button_stateS6 = 0b00000001; 
    uint32_t button_valueS4 = 0b00001000;
    uint32_t button_stateS4 = 0b00001000; 
    uint32_t button_valueS5 = 0b01000000;
    uint32_t button_stateS5 = 0b01000000;     
    uint32_t button_time = 0;
    uint8_t lcd_write = 1;
    uint8_t LCD_Addr = 0x27;
    LCD_Position(LCD_Addr, 0x00);
    LCD_Write(LCD_Addr, strInitial, 7);

    void display_count() {
        char timeStr[6];
        timeStr[4] = lcd_tp%10 + '0'; //Adding +'0' converts int to string.
        timeStr[3] = (lcd_tp/10)%10 + '0';
        timeStr[2] = (lcd_tp/100)%10 + '0';
        timeStr[1] = (lcd_tp/1000)%10 + '0';
        timeStr[0] = (lcd_tp/100000)%10 + '0';
        timeStr[5] = '\0'; //null terminate to determine end of string.
        LCD_Position(LCD_Addr, 0x0b); 
        LCD_Write(LCD_Addr, timeStr, 5);  
        if(lcd_tp > 99999) { //Resetting lcd_tp when overflows.
            lcd_tp = 0;
        }
    }

    enum PHASE phase = HZD;
    while (1) {
        current_tick = millis();
        pot = get_atod();
        if(current_tick - last_tick >= pot) {
            lcd_tp ++;
            time_period ++;
            last_tick = current_tick;
            display_count();
        }
        if(millis() > button_time) {
            button_valueS4 = (PIND & 0b00001000);
            button_valueS6 = (PINB & 0b00000001);
            button_valueS5 = (PIND & 0b01000000);
            if((button_valueS4 ^ button_stateS4) != 0) {
                button_stateS4 = button_valueS4;
                button_time = millis();
                if(button_valueS4 == 0) {
                    S4 = 1;
                    LCD_Position(LCD_Addr, 0x02);
                    LCD_Write_Chr(LCD_Addr, 'X');
                }          
            }
            if((button_valueS5 ^ button_stateS5) != 0) {
                button_stateS5 = button_valueS5;
                button_time = millis();
                if(button_valueS5 == 0) {
                     S5 = 1;                   
                }
            }
            if((button_valueS6 ^ button_stateS6) != 0) {
                button_stateS6 = button_valueS6;
                button_time = millis();
                if(button_valueS6 == 0) {
                    S6 = 1;
                    LCD_Position(LCD_Addr, 0x00);
                    LCD_Write_Chr(LCD_Addr, 'X');
                } else {
                    S6 = 0;
                    LCD_Position(LCD_Addr, 0x00);
                    LCD_Write_Chr(LCD_Addr, '_');                    
                }          
            }
        }
        
        if(button){
            button = 0;
            GPA_Byte = SPI_Read_Command(0x12);
            GPB_Byte = SPI_Read_Command(0x13);
            //button = 0;
            if((GPA_Byte & (1<<4)) == 0) {
                S1 = 1;
                LCD_Position(LCD_Addr, 0x05);
                LCD_Write_Chr(LCD_Addr, 'X');
            }
            if((GPA_Byte & 1) == 0) {
                S0 = 1;
                LCD_Position(LCD_Addr, 0x06);
                LCD_Write_Chr(LCD_Addr, 'X');
            }
            if((GPB_Byte & 1) == 0) {
                S2 = 1;
                LCD_Position(LCD_Addr, 0x04);
                LCD_Write_Chr(LCD_Addr, 'X');                
            }            
            if((GPB_Byte & (1<<4)) == 0) {
                //rv = 0x08;
                S3 = 1;
                LCD_Position(LCD_Addr, 0x03);
                LCD_Write_Chr(LCD_Addr, 'X');
            }
        }
        
        if(S5 == 1) {
            LCD_Position(LCD_Addr, 0x01);
            LCD_Write_Chr(LCD_Addr, 'X');  
            S5 = 0;
        }
        switch (phase) {
            case HZD: 
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,strHazard,12);
                    lcd_write = 0;
                }
                if((S6 == 1) && (millis()>button_time)){
                    if(millis() - interval > 1000 && millis() - interval < 2000) {
                        PORTC &= 0b00000001;
                        write_LEDs(0b0100010001000100);
                        PORTC |= 0b00000100;
                    }else if(millis() - interval > 2000) {
                        write_LEDs(0);
                        PORTC &= 0b11100011;
                        interval = millis();
                    }
                    button_time = millis();
                    ten_seconds = millis() + 10000;
                } else {
                    if(millis() - interval > 1000 && millis() - interval < 2000) {
                        write_LEDs(0b0100010001000100);
                        PORTC |= 0b00000100;
                    }else if(millis() - interval > 2000) {
                        write_LEDs(0);
                        PORTC &= 0b11111011;
                        interval = millis();
                        if(ten_seconds - millis() >= 10000) {
                            PORTC &= 0b00000001;
                            PORTC |= 0b00000010;
                            write_LEDs(0b1000100010001000);
                            lcd_write = 1;
                            time_period = 0;
                            phase = RPS_Red;
                            button_time = millis();
                            interval = millis();                            
                        }
                    }

                }
                break;
            case RPS_Red:
                if(lcd_write == 1) {
                    time_period=0;
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,strRPSR,12);                  
                    lcd_write = 0;
                }
                write_LEDs(0b1000100010001000);
                if(time_period >= 2) {
                    time_period = 0;
                    lcd_write = 1;
                    phase = RPS_Green;
                }
                if(S6 == 1) {
                    button_time = millis();
                    phase = HZD;
                }
                break;
            case RPS_Yellow:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,strRPSY,12);
                    lcd_write = 0;
                }
                write_LEDs(0b1000010010000100);
                if(time_period >= 2) {
                    time_period = 0;
                    lcd_write = 1;
                    phase = RPS_Red;
                }
                if(S6 == 1) {
                    button_time = millis();
                    phase = HZD;
                }                
                break;
            case RPS_Green:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,strRPSG,12);
                    lcd_write = 0;
                }
                write_LEDs(0b1000001010000010);                 
                if((time_period >= 4) && ((S0 == 1) || (S2 == 1))) {  
                    if(S0 == 0) {
                        LCD_Position(LCD_Addr, 0x06);
                        LCD_Write_Chr(LCD_Addr, '_');
                    }
                    if(S2 == 0) {
                        LCD_Position(LCD_Addr, 0x04);
                        LCD_Write_Chr(LCD_Addr, '_');                            
                    }                    
                    if((time_period >= 6) && ((S1 == 1) && (S3 != 1) && (S4 != 1))) {                       
                        time_period = 0;
                        write_LEDs(0b1000010010000010);
                        lcd_write = 1;
                        phase = RPR_SB;
                    } else if((time_period >= 6) && ((S1 != 1) && (S3 == 1) && (S4 != 1))) {
                        time_period = 0;
                        write_LEDs(0b1000001010000100); 
                        lcd_write = 1;
                        phase = RPR_NB;                        
                    } else if((time_period >= 6) && ((S1 == 1) && (S3 == 1) && (S4 != 1))) {
                        time_period = 0;
                        write_LEDs(0b1000010010000100);
                        lcd_write = 1;
                        phase = RPRR;                        
                    } else if((time_period >= 6) && (S4 == 1)) {                     
                        time_period = 0;
                        button_time = millis();
                        write_LEDs(0b1000010010000100);
                        lcd_write = 1;
                        phase = ESTR;                        
                    }
                } else if (((time_period >= 4) && ((S0 != 1) && (S2 != 1)) && ((S1 == 1) && (S3 != 1) && (S4 != 1)))) {
                    time_period = 0;
                    write_LEDs(0b1000010010000010);                
                    lcd_write = 1;
                    phase = RPR_SB;
                } else if (((time_period >= 4) && ((S0 != 1) && (S2 != 1)) && ((S1 != 1) && (S3 == 1) && (S4 != 1)))) {
                    time_period = 0;
                    write_LEDs(0b1000001010000100);
                    lcd_write = 1;
                    phase = RPR_NB;
                } else if (((time_period >= 4) && ((S0 != 1) && (S2 != 1)) && ((S1 == 1) && (S3 == 1) && (S4 != 1)))) {
                    time_period = 0;
                    write_LEDs(0b1000010010000100);
                    lcd_write = 1;
                    phase = RPRR;                       
                } else if ((time_period >= 4) && (S4 == 1)) {                      
                    time_period = 0;
                    button_time = millis();
                    write_LEDs(0b1000010010000100);
                    lcd_write = 1;
                    phase = ESTR;                    
                }
                if((GPA_Byte & 1) != 0) {
                    S0 = 0;
                    LCD_Position(LCD_Addr, 0x06);
                    LCD_Write_Chr(LCD_Addr, '_');                         
                }
                if((GPB_Byte & 1) != 0) {
                    S2 = 0;
                    LCD_Position(LCD_Addr, 0x04);
                    LCD_Write_Chr(LCD_Addr, '_');                         
                }                 
                if(S6 == 1) {
                    ten_seconds = millis();
                    PORTC &= 0b11110001;
                    button_time = millis();
                    lcd_write = 1;
                    phase = HZD;
                }                
                break;
            case RPRR:
                if(time_period >= 2) {
                    if(lcd_write == 1) {
                        LCD_Position(LCD_Addr,0x40);
                        LCD_Write(LCD_Addr,str11,12);
                        lcd_write = 0;
                    }                    
                    write_LEDs(0b1000100010001000);
                    time_period = 0;
                    lcd_write = 1;
                    phase = RPRG;
                }
                break;
            case RPRG:
                if((GPA_Byte & (1<<4)) != 0) {
                    S1 = 0;                    
                    if((GPB_Byte & (1<<4)) != 0) {
                        S3 = 0;                       
                    }  
                }
                if(time_period >= 2)  {
                    if(lcd_write == 1) {
                        LCD_Position(LCD_Addr,0x40);
                        LCD_Write(LCD_Addr,str13,12);
                        lcd_write = 0;
                    }
                    write_LEDs(0b0010100000101000);
                    if((time_period >= 4) && ((S1 != 1) || (S3 != 1)))  {
                        lcd_write = 1;
                        if(lcd_write == 1) {
                            LCD_Position(LCD_Addr,0x40);
                            LCD_Write(LCD_Addr,str12,12);
                            lcd_write = 0;
                        }                        
                        write_LEDs(0b0100100001001000);
                        time_period = 0;
                        lcd_write = 1;
                        phase = RPRY;
                    } else if((time_period >= 4) && ((S1 == 1) || (S3 == 1))) {
                        if(time_period >= 5) {
                            lcd_write = 1;
                            if(lcd_write == 1) {
                                LCD_Position(LCD_Addr,0x40);
                                LCD_Write(LCD_Addr,str12,12);
                                lcd_write = 0;
                            }                        
                            write_LEDs(0b0100100001001000);
                            time_period = 0;
                            lcd_write = 1;
                            phase = RPRY;                            
                        }
                    }
                }
                break;
            case RPRY:
                if(S1 == 0) {
                    LCD_Position(LCD_Addr, 0x05);
                    LCD_Write_Chr(LCD_Addr, '_');                     
                    if(S3 == 0) {
                        LCD_Position(LCD_Addr, 0x03);
                        LCD_Write_Chr(LCD_Addr, '_');                         
                    }  
                }              
                if(time_period >= 2)  {
                    time_period = 0;
                    phase = RPS_Red;
                }
                break;
            case RPR_SB:     
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str7,12);
                    lcd_write = 0;
                }
                if((GPA_Byte & (1<<4)) != 0) {
                    //rv = 0x19;
                    S1 = 0;                   
                }                
                if(time_period >= 2)  {
                    lcd_write = 1;
                    write_LEDs(0b1000100000100010);
                    if(lcd_write == 1) {
                        LCD_Position(LCD_Addr,0x40);
                        LCD_Write(LCD_Addr,str16,12);
                        lcd_write = 0;
                    }
                    if((time_period >= 4) && (S1 != 1))  {                      
                        time_period = 0;
                        lcd_write = 1;
                        phase = RPR_SB_Yellow;
                    } else if (time_period >= 5) {
                        time_period = 0;
                        lcd_write = 1;
                        phase = RPR_SB_Yellow;
                    }
                }
                if(S6 == 1) {
                    ten_seconds = millis();
                    PORTC &= 0b11110001;                    
                    button_time = millis();
                    phase = HZD;
                }                
                break;
            case RPR_SB_Yellow:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str14,12);
                    lcd_write = 0;
                }
                write_LEDs(0b1000100001000010);
                if(time_period >= 2)  {
                    lcd_write = 1;
                    time_period = 0;
                    phase = RPR_SB_Red;
                }
                if(S6 == 1) {
                    ten_seconds = millis();
                    PORTC &= 0b11110001;                    
                    button_time = millis();
                    phase = HZD;
                } 
                break;
            case RPR_SB_Red:
                write_LEDs(0b1000100010000010);
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str15,12);
                    lcd_write = 0;
                }                
                if(time_period >= 2)  {
                    time_period = 0;
                    lcd_write = 1;
                    phase = RPS_Green;
                }
                if(S1 == 0) {
                    LCD_Position(LCD_Addr, 0x05);
                    LCD_Write_Chr(LCD_Addr, '_');                     
                }
                if(S6 == 1) {
                    ten_seconds = millis();
                    PORTC &= 0b11110001;                    
                    button_time = millis();
                    phase = HZD;
                }                
                break;
            case RPR_NB:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str4,12);
                    lcd_write = 0;
                }  
                if((GPB_Byte & (1<<4)) != 0) {
                    S3 = 0;                     
                }                 
                if(time_period >= 2)  {
                    lcd_write = 1;
                    write_LEDs(0b0010001010001000);
                    if(lcd_write == 1) {
                        LCD_Position(LCD_Addr,0x40);
                        LCD_Write(LCD_Addr,str5,12);
                        lcd_write = 0;
                    }
                    if((time_period >= 4) && (S3 != 1))  {
                        time_period = 0;
                        lcd_write = 1;
                        phase = RPR_NB_Yellow;
                    } else if(time_period >= 5) {
                        time_period = 0;
                        lcd_write = 1;
                        phase = RPR_NB_Yellow;
                    }
                }
                if(S6 == 1) {
                    ten_seconds = millis();
                    PORTC &= 0b11110001;                    
                    button_time = millis();
                    phase = HZD;
                }                
                break;
            case RPR_NB_Yellow:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str17,12);
                    lcd_write = 0;
                }
                write_LEDs(0b0100001010001000);
                if(time_period >= 2)  {
                    time_period = 0;
                    lcd_write = 1;
                    phase = RPR_NB_Red;
                }
                if(S6 == 1) {
                    ten_seconds = millis();
                    PORTC &= 0b11110001;                    
                    button_time = millis();
                    lcd_write = 1;
                    phase = HZD;
                }
                break;
            case RPR_NB_Red:
                write_LEDs(0b1000001010001000);
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str18,12);
                    lcd_write = 0;
                }                
                if(time_period >= 2)  {
                    time_period = 0;
                    lcd_write = 1;
                    phase = RPS_Green;
                }
                if(S3 == 0) {
                    LCD_Position(LCD_Addr, 0x03);
                    LCD_Write_Chr(LCD_Addr, '_');                     
                } 
                if(S6 == 1) {
                    ten_seconds = millis();
                    PORTC &= 0b11110001;                    
                    button_time = millis();
                    lcd_write = 1;
                    phase = HZD;
                }
                break;                
            case ESTR:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,strRPSY,12);
                    lcd_write = 0;
                }                
                if(time_period >= 2)  {
                    write_LEDs(0b1000100010001000);
                    PORTC &= 0b00110001;
                    PORTC |= 0b00000010;
                    lcd_write = 1;
                    phase = EST_AR;
                    time_period = 0;
                }
                if(S6 == 1) {
                    button_time = millis();
                    PORTC &= 0b11110001;
                    lcd_write = 1;
                    phase = HZD;
                } 
                break;
            case EST_AR:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str10,12);
                    lcd_write = 0;
                }                
                if(time_period >= 2)  {
                    write_LEDs(0b1000100010001000);
                    PORTC &= 0b00110001;
                    PORTC |= 0b00001000;
                    time_period = 0;
                    lcd_write = 1;
                    phase = ESTG;
                }
                if(S6 == 1) {
                    button_time = millis();
                    PORTC &= 0b11110001;
                    lcd_write = 1;
                    phase = HZD;
                } 
                break;
            case ESTG:
                if(lcd_write == 1) {
                    if(button_valueS4 != 0) {
                        S4 = 0;
                    }                    
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str8,12);
                    lcd_write = 0;
                }
                if((time_period >= 2) && (S4 == 0))  {
                    PORTC &= 0b00110001;
                    PORTC |= 0b00000100; 
                    time_period = 0;
                    lcd_write = 1;
                    phase = ESTY;
                } else if((time_period >= 2) && (S4 == 1)) {
                    if(time_period >= 4) {
                        PORTC &= 0b00110001;
                        PORTC |= 0b00000100; 
                        time_period = 0;
                        lcd_write = 1;
                        phase = ESTY;                        
                    }
                }
                if(S6 == 1) {
                    button_time = millis();
                    PORTC &= 0b11110001;
                    lcd_write = 1;
                    phase = HZD;
                } 
                break;
            case ESTY:
                if(lcd_write == 1) {
                    LCD_Position(LCD_Addr,0x40);
                    LCD_Write(LCD_Addr,str9,12);
                    lcd_write = 0;
                }                
                if(time_period >= 2)  {
                    PORTC &= 0b00110001;
                    PORTC |= 0b00000010;
                    if(S4 == 0) {
                        LCD_Position(LCD_Addr, 0x02);
                        LCD_Write_Chr(LCD_Addr, '_');
                        
                    }                     
                    time_period = 0;
                    lcd_write = 1;
                    phase = RPS_Red;                  
                }
                if(S6 == 1) {
                    button_time = millis();
                    PORTC &= 0b11110001;
                    lcd_write = 1;
                    phase = HZD;
                }                
                break;
        }

    }
    
}
