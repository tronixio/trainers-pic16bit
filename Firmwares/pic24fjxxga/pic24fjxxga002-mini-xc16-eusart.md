# PIC16-Bit Mini Trainer.

## XC16 - EUSART - ADC - SWITCHS - ROTARY ENCODER.

```c
// Configuration Registers.
#pragma config WDTPS = PS32768, FWPSA = PR128, WINDIS = OFF, FWDTEN = OFF
#pragma config ICS = PGx1, GWRP = OFF, GCP = OFF, JTAGEN = OFF
#pragma config POSCMOD = NONE, I2C1SEL = PRI, IOL1WAY = ON
#pragma config OSCIOFNC = OFF, FCKSM = CSDCMD, FNOSC = FRC
#pragma config SOSCSEL = SOSC, WUTSEL = LEG, IESO = OFF

#define FOSC (8000000UL)
#define FCY (FOSC/2)
#define _ISR_FAST __attribute__ ((interrupt, shadow))
#define _ISR_NOPSV __attribute__ ((interrupt, no_auto_psv))
#define _ISR_PSV __attribute__ ((interrupt, auto_psv))

#include <xc.h>
#include <libpic30.h>
// PIC24FJxxGA002 - Compile with XC16(v2.00).
// PIC24FJxxGA002 - @8MHz Internal Oscillator.

// Rotary encoder code from:
// https://www.mikrocontroller.net/articles/Drehgeber

// PIC16-Bit Mini Trainer.
// 2x ADC CHANNELS.
// 1x LCD NHD-C0220BiZ - ST7036.
// 1x EUSART ASYNCHRONOUS TX/RX.
// 1x ROTARY ENCODER with SWITCH.
// 2x SWITCHS.

// JUMPER.URX - Close.
// JUMPER.UTX - Close.
// JUMPER.SDA - Not Use.
// JUMPER.SCL - Not Use.
// JUMPER.VREG - GND.
// JUMPER.VCAP - Close.
// JUMPER.BCKL - Not Use.

// Pinout.
// MCU.RA0 <- ANALOG.AN1.
// MCU.RA1 <- ANALOG.AN2.
// MCU.RB2 <- SWITCH.S1.
// MCU.RB3 <- SWITCH.S2.
// MCU.RA2 <- ROTARY.A.
// MCU.RA3 -> OSCILLOSCOPE.PROBE.A.
// MCU.RA4 <- ROTARY.B.
// MCU.RB4 <- ROTARY.S.

// Definitions.
// EUSART.
#define BAUDRATE             9600
#define BAUDRATE_GENERATOR   (((FCY/BAUDRATE)/16)-1)
// ASCII Characters.
#define ASCII_CR              0x0D
// Rotary Encoder.
#define ROTARY_PHASE_A        PORTAbits.RA2
#define ROTARY_PHASE_B        PORTAbits.RA4
#define ROTARY_SWITCH         PORTBbits.RB4
// Switchs.
#define SWITCH_S1             PORTBbits.RB2
#define SWITCH_S2             PORTBbits.RB3

// Function Prototypes.
uint8_t eusart_readCharacter(void);
void eusart_writeCharacter(uint8_t u8Data);
void eusart_writeString(const uint8_t * u8Data);
int8_t rotary_i8encoderRead(void);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);

// Strings & Custom Patterns.
const uint8_t au8Tronix[] = "\r\n\r\nTronix I/O";
const uint8_t au8WWW[] = "\r\nhttps://www.tronix.io/\r\n";
const uint8_t au8Ready[] = "\r\nREADY> ";
const uint8_t au8Adc0[] = "\r\nADC CHANNEL 0> ";
const uint8_t au8Adc1[] = "\r\nADC CHANNEL 1> ";
const uint8_t au8Encoder[] = "\r\nROTARY ENCODER> ";
const uint8_t au8Encodersw[] = "\r\nROTARY ENCODER SWITCH> ";
const uint8_t au8Eusart[] = "\r\nEUSART ECHO> ";
const uint8_t au8Switch1[] = "\r\nSWITCH 1> ";
const uint8_t au8Switch2[] = "\r\nSWITCH 2> ";
const uint8_t au8Pressed[] = "PRESSED";
const uint8_t au8Released[] = "RELEASED";

// Global Variables.
int8_t i8encoderDelta;
uint16_t u16AdcTimer;
const int8_t encoderFull[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// Interrupts Service Routines
void _ISR_NOPSV _T1Interrupt(void)
{
    if(IFS0bits.T1IF){
        static int8_t u8encoderLast = 0;
        u8encoderLast = (u8encoderLast<<2) & 0x0F;
        if(ROTARY_PHASE_A) u8encoderLast |= 0b01; // CW.
        if(ROTARY_PHASE_B) u8encoderLast |= 0b10; // CCW.
        i8encoderDelta += encoderFull[u8encoderLast];
        IFS0bits.T1IF = 0b0;
    }
    u16AdcTimer++;
}

// Main.
int main(void)
{
    // MCU Initialization.
    // Internal Oscillator Settings.
    CLKDIVbits.RCDIV = 0b000;
    // Ports Initialization.
    // Analog Inputs Settings.
    AD1PCFG = 0b1001111000111100;
    // Port A Settings.
    TRISA = 0b0000000000010111;
    PORTA = 0b0000000000000000;
    LATA = 0b0000000000000000;
    ODCA = 0b0000000000000000;
    // Port B Settings.
    TRISB = 0b0000000000011110;
    PORTB = 0b0000000000000000;
    LATB = 0b0000000000000000;
    ODCB = 0b0000000000000000;
    // PPS Settings.
    __builtin_write_OSCCONL(OSCCON & 0xBF);
    // PPS Inputs.
    RPINR18bits.U1RXR = 0b00001;    // RB1 - RP1 - EUSART.U1RX.
    // PPS Outputs.
    RPOR0bits.RP0R = 0b00011;       // RB0 - RP0 - EUSART.U1TX.
    __builtin_write_OSCCONL(OSCCON | 0x40);

    // Interrupts Settings.
    INTCON1 = 0x8000;
    INTCON2 = 0x0000;

    // ADC Settings.
    AD1CON1 = 0x0000;
    AD1CON1bits.ADSIDL = 0b1;
    AD1CON1bits.FORM = 0b00;
    AD1CON1bits.SSRC = 0b111;
    AD1CON1bits.ASAM = 0b1;
    AD1CON1bits.SAMP = 0b0;
    AD1CON2 = 0x0000;
    AD1CON2bits.VCFG = 0b000;
    AD1CON2bits.CSCNA = 0b1;
    AD1CON2bits.BUFS = 0b0;
    AD1CON2bits.SMPI = 0b0001;
    AD1CON2bits.BUFM = 0b0;
    AD1CON2bits.ALTS = 0b0;
    AD1CON3 = 0x0000;
    AD1CON3bits.ADRC = 0b0;
    AD1CON3bits.SAMC = 0b11111;
    AD1CON3bits.ADCS = 0b00111111;
    AD1CHS = 0x0000;
    AD1CHSbits.CH0NB = 0b0;
    AD1CHSbits.CH0SB = 0b00000;
    AD1CHSbits.CH0NA = 0b0;
    AD1CHSbits.CH0SA = 0b00000;
    AD1CSSL = 0b0000000000000011;
    // ADC Enable.
    AD1CON1bits.ADON = 0b1;

    // EUSART Settings.
    U1BRG = BAUDRATE_GENERATOR;
    U1MODE = 0x0000;
    U1MODEbits.UEN = 0b00;
    U1MODEbits.BRGH = 0b0;
    U1MODEbits.PDSEL = 0b00;
    U1MODEbits.STSEL = 0b0;
    U1STA = 0x0000;
    U1TXREG = 0x0000;
    U1RXREG = 0x0000;
    // UART Enable.
    U1MODEbits.UARTEN = 0b1;
    U1STAbits.UTXEN = 0b1;

    // Timer1 Settings.
    // ~500Hz @8MHz.
    PR1 = 3980;
    T1CON = 0x0000;
    // Timer1 Enable.
    T1CONbits.TON = 0b1;
    // Timer1 Interrupts Enable.
    IFS0bits.T1IF = 0b0;
    IEC0bits.T1IE = 0b1;
    IPC0bits.IC1IP = 0b000;

    // Display String.
    eusart_writeString(au8Tronix);
    eusart_writeString(au8WWW);
    eusart_writeString(au8Ready);

    uint8_t u8Rx;
    uint8_t au8Buffer[6];
    uint16_t u16ADCRead, u16ADC0Last, u16ADC1Last;
    uint8_t u8encoderSwitchPressed;
    uint16_t u8encoderRead, u8encoderLast;
    uint8_t u8switchS1Pressed, u8switchS2Pressed;
    while(1){
        // ADC Read every ~1s.
        if(u16AdcTimer>1000){
            u16ADCRead = 0;
            AD1CHSbits.CH0SA = 0b00000;
            __delay_us(5);
            AD1CON1bits.DONE = 0b1;
            while(!AD1CON1bits.DONE){};
                u16ADCRead = ADC1BUF0;
                if(u16ADC0Last != u16ADCRead){
                    u16toa(u16ADCRead, au8Buffer, 10);
                    eusart_writeString(au8Adc0);
                    eusart_writeString(au8Buffer);
                    u16ADC0Last = u16ADCRead;
                }
            u16ADCRead = 0;
            AD1CHSbits.CH0SA = 0b00001;
            __delay_us(5);
            AD1CON1bits.DONE = 0b1;
            while(!AD1CON1bits.DONE){};
                u16ADCRead = ADC1BUF1;
                if(u16ADC1Last != u16ADCRead){
                    u16toa(u16ADCRead, au8Buffer, 10);
                    eusart_writeString(au8Adc1);
                    eusart_writeString(au8Buffer);
                    u16ADC1Last = u16ADCRead;
                }
            u16AdcTimer = 0;
        }

        // EUSART.
        if(IFS0bits.U1RXIF){
            u8Rx = eusart_readCharacter();
            eusart_writeString(au8Eusart);
            eusart_writeCharacter(u8Rx);
            if(u8Rx == ASCII_CR)
                eusart_writeString(au8Ready);
        }

        // ROTARY ENCODER.
        if(!ROTARY_SWITCH){
            __delay_ms(100);
            u8encoderSwitchPressed = 0b1;
            eusart_writeString(au8Encodersw);
            eusart_writeString(au8Pressed);
        }else if(ROTARY_SWITCH){
            if(u8encoderSwitchPressed){
                u8encoderSwitchPressed = 0b0;
                eusart_writeString(au8Encodersw);
                eusart_writeString(au8Released);
                u8encoderRead = 0;
            }
        }

        u8encoderRead += rotary_i8encoderRead();
        if(u8encoderLast != u8encoderRead){
            u16toa(u8encoderRead, au8Buffer, 10);
            eusart_writeString(au8Encoder);
            eusart_writeString(au8Buffer);
            u8encoderLast = u8encoderRead;
        }

        // SWITCHS.
        if(!SWITCH_S1){
            __delay_ms(100);
            u8switchS1Pressed = 0b1;
            eusart_writeString(au8Switch1);
            eusart_writeString(au8Pressed);
        }else if(SWITCH_S1){
            if(u8switchS1Pressed){
                eusart_writeString(au8Switch1);
                eusart_writeString(au8Released);
                u8switchS1Pressed = 0b0;
            }
        }
        if(!SWITCH_S2){
            __delay_ms(100);
            u8switchS2Pressed = 0b1;
            eusart_writeString(au8Switch2);
            eusart_writeString(au8Pressed);
        }else if(SWITCH_S2){
            if(u8switchS2Pressed){
                eusart_writeString(au8Switch2);
            eusart_writeString(au8Released);
            u8switchS2Pressed = 0b0;
            }
        }
    }
    return(0);
}

// Functions.
uint8_t eusart_readCharacter(void)
{
    while(!U1STAbits.URXDA){};
    return(U1RXREG);
}

void eusart_writeCharacter(uint8_t u8Data)
{
    while(U1STAbits.UTXBF){};
    U1TXREG = u8Data;
}

void eusart_writeString(const uint8_t * u8Data)
{
    while(*u8Data != '\0')
        eusart_writeCharacter(*u8Data++);
}

int8_t rotary_i8encoderRead(void)
{
    int8_t u8encoderRead;

    IEC0bits.T1IE = 0b0;
    u8encoderRead = i8encoderDelta;
    i8encoderDelta = u8encoderRead & 3;
    IEC0bits.T1IE = 0b1;

    return(u8encoderRead>>2);
}

void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base)
{
    uint8_t u8Buffer;
    uint16_t data = u16Data;

    do{
        data /= u8Base;
        au8Buffer++;
    }while(data != '\0');
    *au8Buffer-- = 0;

    do{
        u8Buffer = (uint16_t)(u16Data % u8Base);
        u16Data /= u8Base;
        if(u8Buffer >= 10)
            u8Buffer += 'A' - '0' - 10;
        u8Buffer += '0';
        *au8Buffer-- = u8Buffer;
    }while(u16Data != '\0');
}
```

---
DISCLAIMER: THIS CODE IS PROVIDED WITHOUT ANY WARRANTY OR GUARANTEES.
USERS MAY USE THIS CODE FOR DEVELOPMENT AND EXAMPLE PURPOSES ONLY.
AUTHORS ARE NOT RESPONSIBLE FOR ANY ERRORS, OMISSIONS, OR DAMAGES THAT COULD
RESULT FROM USING THIS FIRMWARE IN WHOLE OR IN PART.
