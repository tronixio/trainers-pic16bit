# PIC16-Bit Mini Trainer.

## XC16 - LCD - ADC - SWITCHS - ROTARY ENCODER.

```c
// Configuration Registers.
#pragma config WDTPS = PS32768, FWPSA = PR128, WINDIS = OFF, FWDTEN = OFF
#pragma config ICS = PGx1, GWRP = OFF, GCP = OFF, JTAGEN = OFF
#pragma config POSCMOD = NONE, I2C1SEL = PRI, IOL1WAY = ON
#pragma config OSCIOFNC = OFF, FCKSM = CSDCMD, FNOSC = FRC
#pragma config SOSCSEL = SOSC, WUTSEL = LEG, IESO = OFF

#define FOSC    (8000000UL)
#define FCY     (FOSC/2)
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

// JUMPER.URX - Not Use.
// JUMPER.UTX - Not Use.
// JUMPER.SDA - Close.
// JUMPER.SCL - Close.
// JUMPER.VREG - GND.
// JUMPER.VCAP - Close.
// JUMPER.BCKL - Open.

// Pinout.
// MCU.RA0 <- ANALOG.AN1.
// MCU.RA1 <- ANALOG.AN2.
// MCU.RB2 <- SWITCH.S1.
// MCU.RB3 <- SWITCH.S2.
// MCU.RA2 <- ROTARY.A.
// MCU.RA3 -> OSCILLOSCOPE.PROBE.A.
// MCU.RA4 <- ROTARY.B.
// MCU.RB4 <- ROTARY.S.
// MCU.RB6 -> LCD.BACKLIGHT.EN.

// Definitions.
// I2C.
#define I2C_READ                                            0b1
#define I2C_WRITE                                           0b0
#define I2C_FSCL_HZ                                         400000
#define I2C_BRG                                             (((FCY/I2C_FSCL_HZ)-(FCY/10000000))-1)
// CAT4002A.
#define CAT4002_DELAY_DIM_MS                                5
#define CAT4002_DELAY_LED_US                                10
// ST7036 I2C Address.
#define ST7036_I2C_ADDRESS_78                               0x78
#define ST7036_I2C_CONTROL_CONTINUOUS_COMMAND               0x00
#define ST7036_I2C_CONTROL_CONTINUOUS_DATA                  0x40
#define ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND            0x80
#define ST7036_I2C_CONTROL_NO_CONTINUOUS_DATA               0xC0
// ST7036 Instruction Table IS2=0, IS1=0.
#define ST7036_CLEAR_DISPLAY                                0x01
#define ST7036_RETURN_HOME                                  0x02
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_NOSHIFT       0x04
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_SHIFT_RIGHT   0x05
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT       0x06
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_SHIFT_LEFT    0x07
#define ST7036_DISPLAY_OFF                                  0x08
#define ST7036_DISPLAY_ON_CURSOR_OFF                        0x0C
#define ST7036_DISPLAY_ON_CURSOR_ON_NOBLINK                 0x0E
#define ST7036_DISPLAY_ON_CURSOR_ON_BLINK                   0x0F
#define ST7036_DISPLAY_CURSOR_SHIFT_LEFT                    0x10
#define ST7036_DISPLAY_CURSOR_SHIFT_RIGHT                   0x14
#define ST7036_DISPLAY_DISPLAY_SHIFT_LEFT                   0x18
#define ST7036_DISPLAY_DISPLAY_SHIFT_RIGH                   0x1C
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8          0x20
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS1      0x21
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS2      0x22
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_FONT5x8          0x28
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_DHFONT5x8        0x24
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_DHFONT5x8        0x2C
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8          0x30
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1      0x31
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS2      0x32
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_DHFONT5x8        0x34
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_FONT5x8          0x38
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_DHFONT5x8        0x3C
#define ST7036_SET_ICON_RAM_ADDRESS                         0X40
#define ST7036_SET_CGRAM_ADDRESS                            0x40
#define ST7036_DDRAM_ADDRESS_FIRST_LINE                     0x80
#define ST7036_DDRAM_ADDRESS_SECOND_LINE                    0xC0
// ST7036 Instruction Table IS2=0, IS1=1.
#define ST7036_BIAS_SET_1_5                                 0x14
#define ST7036_BIAS_SET_1_5_3_LINE                          0x15
#define ST7036_BIAS_SET_1_4                                 0x1C
#define ST7036_BIAS_SET_1_4_3_LINE                          0x1D
#define ST7036_POWER_ICON_OFF_BOOST_OFF_NO_CONTRAST         0x50
#define ST7036_POWER_ICON_ON_BOOST_OFF_NO_CONTRAST          0x58
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_0       0x54
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_1       0x55
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_2       0x56
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_3       0x57
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_0        0x5C
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_1        0x5D
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_2        0x5E
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_3        0x5F
#define ST7036_FOLLOWER_CONTROL_OFF                         0x60
#define ST7036_FOLLOWER_CONTROL_ON_RAB_0                    0x68
#define ST7036_FOLLOWER_CONTROL_ON_RAB_1                    0x69
#define ST7036_FOLLOWER_CONTROL_ON_RAB_2                    0x6A
#define ST7036_FOLLOWER_CONTROL_ON_RAB_3                    0x6B
#define ST7036_FOLLOWER_CONTROL_ON_RAB_4                    0x6C
#define ST7036_FOLLOWER_CONTROL_ON_RAB_5                    0x6D
#define ST7036_FOLLOWER_CONTROL_ON_RAB_6                    0x6E
#define ST7036_FOLLOWER_CONTROL_ON_RAB_7                    0x6F
#define ST7036_CONTRAST_LSB_0                               0x70
#define ST7036_CONTRAST_LSB_1                               0x71
#define ST7036_CONTRAST_LSB_2                               0x72
#define ST7036_CONTRAST_LSB_3                               0x73
#define ST7036_CONTRAST_LSB_4                               0x74
#define ST7036_CONTRAST_LSB_5                               0x75
#define ST7036_CONTRAST_LSB_6                               0x76
#define ST7036_CONTRAST_LSB_7                               0x77
#define ST7036_CONTRAST_LSB_8                               0x78
#define ST7036_CONTRAST_LSB_9                               0x79
#define ST7036_CONTRAST_LSB_10                              0x7A
#define ST7036_CONTRAST_LSB_11                              0x7B
#define ST7036_CONTRAST_LSB_12                              0x7C
#define ST7036_CONTRAST_LSB_13                              0x7D
#define ST7036_CONTRAST_LSB_14                              0x7E
#define ST7036_CONTRAST_LSB_15                              0x7F
// ST7036 Instruction Table IS2=1, IS1=0.
#define ST7036_DOUBLE_HEIGHT_FONT_COM9_COM24                0x10
#define ST7036_DOUBLE_HEIGHT_FONT_COM1_COM16                0x11
// ST7036 Delays.
#define ST7036_CLEAR_DISPLAY_DELAY_MS                       2
#define ST7036_INITIALIZATION_DELAY_MS                      40
// NHD-C0220BiZ Configuration.
#define C0220BiZ_CONFIGURATION_I2C_ADDRESS                  ST7036_I2C_ADDRESS_78
#define C0220BiZ_CONFIGURATION_FIRST_LINE                   ST7036_DDRAM_ADDRESS_FIRST_LINE
#define C0220BiZ_CONFIGURATION_SECOND_LINE                  ST7036_DDRAM_ADDRESS_SECOND_LINE
#define C0220BiZ_CONFIGURATION_CHARACTERS                   20
// ASCII Characters.
#define ASCII_SPACE                                         0x20
// Patterns.
#define PATTERN_BATTERY_FULL                                0x04
#define PATTERN_BATTERY_3                                   0x03
#define PATTERN_BATTERY_2                                   0x02
#define PATTERN_BATTERY_1                                   0x01
#define PATTERN_BATTERY_EMPTY                               0x00
// LCD.
#define LCD_BACKLIGHT_OFF                                   LATBbits.LATB6 = 0b0
#define LCD_BACKLIGHT_ON                                    LATBbits.LATB6 = 0b1
// Rotary Encoder.
#define ROTARY_PHASE_A                                      PORTAbits.RA2
#define ROTARY_PHASE_B                                      PORTAbits.RA4
#define ROTARY_SWITCH                                       PORTBbits.RB4
// Switchs.
#define SWITCH_S1                                           PORTBbits.RB2
#define SWITCH_S2                                           PORTBbits.RB3

// Function Prototypes.
void i2c_restart(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t u8Data);
void lcd_clearDisplay(void);
void lcd_clearLine(uint8_t u8Line);
void lcd_initialize(void);
void lcd_setBacklight(uint8_t u8Backlight);
void lcd_setCursor(uint8_t u8Cursor);
void lcd_writeCharacter(uint8_t u8Data);
void lcd_writeInstruction(uint8_t u8Data);
void lcd_writeString(const uint8_t * u8Data);
void lcd_writeStringSetCursor(const uint8_t * u8Data, uint8_t u8Cursor);
int8_t rotary_i8encoderFilter(void);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);

// Strings & Custom Patterns.
const uint8_t au8Tronix[] = "Tronix I/O";
const uint8_t au8WWW[] = "www.tronix.io";
const uint8_t au8Adc0[] = "ADC CHANNEL 0> ";
const uint8_t au8Adc1[] = "ADC CHANNEL 1> ";
const uint8_t au8Encoder[] = "ROTARY> ";
const uint8_t au8Backlight[] = "BACKLIGHT> ";
const uint8_t au8Switch1[] = "SWITCH 1> ";
const uint8_t au8Switch2[] = "SWITCH 2> ";
const uint8_t au8Pressed[] = "PRESSED";
const uint8_t au8Released[] = "RELEASED";

const uint8_t au8BatteryPattern[5][8] = {
    {0x0e, 0x1b, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f},
    {0x0e, 0x1b, 0x11, 0x11, 0x11, 0x1f, 0x1f, 0x1f},
    {0x0e, 0x1b, 0x11, 0x11, 0x1f, 0x1f, 0x1f, 0x1f},
    {0x0e, 0x1b, 0x11, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f},
    {0x0e, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f}
};

const uint8_t au8encoder[16] = {0, 1, 255, 0, 255, 0, 0, 1, 1, 0, 0, 255, 0, 255, 1, 0};

// Global Variables.
uint8_t u8encoderDelta;
uint16_t u16adcTimer;

// Interrupts Service Routines
void _ISR_NOPSV _T1Interrupt(void)
{
    if(IFS0bits.T1IF){
        static int8_t u8encoder = 0;
        u8encoder = (u8encoder<<2) & 0x0F;
        if(ROTARY_PHASE_A) u8encoder |= 0b01; // CW.
        if(ROTARY_PHASE_B) u8encoder |= 0b10; // CCW.
        u8encoderDelta += au8encoder[u8encoder];
        IFS0bits.T1IF = 0b0;
    }
    u16adcTimer++;
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

    // I2C Master Settings.
    I2C1BRG = I2C_BRG;
    I2C1CON = 0x0000;
    I2C1STAT = 0x0000;
    // I2C Enable.
    I2C1CONbits.I2CEN = 0b1;

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

    // ST7036 Initialization.
    lcd_initialize();

    // Write 5x8 Dots Custom Patterns in ST7036 CGRAM.
    lcd_writeInstruction(ST7036_SET_CGRAM_ADDRESS);
    uint8_t line = 0;
    uint8_t pattern = 0;
    for(pattern=0; pattern<5; pattern++){
        for(line=0; line<8; line++){
            lcd_writeCharacter(au8BatteryPattern[pattern][line]);
        }
    }

    // Display String.
    LCD_BACKLIGHT_ON;
    lcd_writeStringSetCursor(au8Tronix, C0220BiZ_CONFIGURATION_FIRST_LINE);
    lcd_writeStringSetCursor(au8WWW, C0220BiZ_CONFIGURATION_SECOND_LINE);
    lcd_setCursor(C0220BiZ_CONFIGURATION_FIRST_LINE + 19);
    lcd_writeCharacter(PATTERN_BATTERY_FULL);

    uint8_t au8Buffer[6];
    uint16_t u16ADCRead, u16ADC0Last, u16ADC1Last;
    uint8_t u8LCDBacklight=0, u8encoderRotary=0, u8encoderRead=0, u8encoderLast=0, u8encoderSwitchPressed;
    uint8_t u8switchS1Pressed, u8switchS2Pressed;
    while(1){
        // ADC Read every ~1s.
        if(u16adcTimer>1000){
            u16ADCRead = 0;
            AD1CHSbits.CH0SA = 0b00000;
            __delay_us(5);
            while(!AD1CON1bits.DONE){};
            u16ADCRead = ADC1BUF0;
            if(u16ADC0Last != u16ADCRead){
                u16toa(u16ADCRead, au8Buffer, 10);
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Adc0);
                lcd_writeString(au8Buffer);
                u16ADC0Last = u16ADCRead;
            }
            u16ADCRead = 0;
            AD1CHSbits.CH0SA = 0b00001;
            __delay_us(5);
            while(!AD1CON1bits.DONE){};
            u16ADCRead = ADC1BUF1;
            if(u16ADC1Last != u16ADCRead){
                u16toa(u16ADCRead, au8Buffer, 10);
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Adc1);
                lcd_writeString(au8Buffer);
                u16ADC1Last = u16ADCRead;
            }
            u16adcTimer = 0;
        }

        // ROTARY ENCODER.
        if(!ROTARY_SWITCH){
            __delay_ms(100);
            u8encoderSwitchPressed = !u8encoderSwitchPressed;
            lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
            if(u8encoderSwitchPressed){
                u16toa(u8encoderRotary, au8Buffer, 10);
                lcd_writeString(au8Encoder);
                lcd_writeString(au8Buffer);
            }else if(!u8encoderSwitchPressed){
                u16toa(u8LCDBacklight, au8Buffer, 10);
                lcd_writeString(au8Backlight);
                lcd_writeString(au8Buffer);
            }
            while(!ROTARY_SWITCH){};
        }

        u8encoderRead += rotary_i8encoderFilter();
        if(u8encoderLast != u8encoderRead){
          lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
          if(!u8encoderSwitchPressed){
                u8LCDBacklight += u8encoderRead;
                if(u8LCDBacklight >= 33) u8LCDBacklight = 0;
                u16toa(u8LCDBacklight, au8Buffer, 10);
                lcd_writeString(au8Backlight);
                lcd_writeString(au8Buffer);
                lcd_setBacklight(30 - u8LCDBacklight);
            }else if(u8encoderSwitchPressed){
                u8encoderRotary += u8encoderRead;
                u16toa(u8encoderRotary, au8Buffer, 10);
                lcd_writeString(au8Encoder);
                lcd_writeString(au8Buffer);
                lcd_setCursor(C0220BiZ_CONFIGURATION_FIRST_LINE + 19);
                if(u8encoderRotary<51)
                    lcd_writeCharacter(PATTERN_BATTERY_EMPTY);
                else if(u8encoderRotary>50 && u8encoderRotary<101)
                    lcd_writeCharacter(PATTERN_BATTERY_1);
                else if(u8encoderRotary>100 && u8encoderRotary<151)
                    lcd_writeCharacter(PATTERN_BATTERY_2);
                else if(u8encoderRotary>150 && u8encoderRotary<201)
                    lcd_writeCharacter(PATTERN_BATTERY_3);
                else
                    lcd_writeCharacter(PATTERN_BATTERY_FULL);
            }
            u8encoderLast = u8encoderRead;
            u8encoderRead = 0;
        }

        // SWITCHS.
        if(!SWITCH_S1){
            __delay_ms(100);
            u8switchS1Pressed = 0b1;
            lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
            lcd_writeString(au8Switch1);
            lcd_writeString(au8Pressed);
            LCD_BACKLIGHT_ON;
            while(!SWITCH_S1){};
        }else if(SWITCH_S1){
            if(u8switchS1Pressed){
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Switch1);
                lcd_writeString(au8Released);
                u8switchS1Pressed = 0b0;
            }
        }
        if(!SWITCH_S2){
            __delay_ms(100);
            u8switchS2Pressed = 0b1;
            lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
            lcd_writeString(au8Switch2);
            lcd_writeString(au8Pressed);
            LCD_BACKLIGHT_OFF;
            while(!SWITCH_S2){};
        }else if(SWITCH_S2){
            if(u8switchS2Pressed){
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Switch2);
                lcd_writeString(au8Released);
                u8switchS2Pressed = 0b0;
            }
        }
    }
    return(0);
}

// Functions.
void i2c_restart(void)
{
    while(I2C1CON & 0x001F){};
    I2C1CONbits.RSEN = 0b1;
    while(I2C1CONbits.RSEN){};
}

void i2c_start(void)
{
    while(I2C1CON & 0x001F){};
    I2C1CONbits.SEN = 0b1;
    while(I2C1CONbits.SEN){};
}

void i2c_stop(void)
{
    while(I2C1CON & 0x001F){};
    I2C1CONbits.PEN = 0b1;
    while(I2C1CONbits.PEN){};
}

void i2c_write(uint8_t u8Data)
{
    I2C1TRN = u8Data;
    while(I2C1STATbits.TRSTAT){};
}

void lcd_clearDisplay(void)
{
    lcd_writeInstruction(ST7036_CLEAR_DISPLAY);
    __delay_ms(ST7036_CLEAR_DISPLAY_DELAY_MS);
}

void lcd_clearLine(uint8_t u8Line)
{
    uint8_t character = C0220BiZ_CONFIGURATION_CHARACTERS;

    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Line);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_DATA);
    while(character--)
        i2c_write(ASCII_SPACE);
    i2c_restart();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Line);
    i2c_stop();
}

void lcd_initialize(void)
{
    __delay_ms(ST7036_INITIALIZATION_DELAY_MS);

    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_COMMAND);
    // Instruction Table IS2=0, IS1=1.
    i2c_write(ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1);
    i2c_write(ST7036_BIAS_SET_1_5);
    i2c_write(ST7036_CONTRAST_LSB_12);
    i2c_write(ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_1);
    i2c_write(ST7036_FOLLOWER_CONTROL_ON_RAB_5);
    // Instruction Table IS2=0, IS1=0.
    i2c_write(ST7036_FUNCTION_SET_8_BIT_TWO_LINE_FONT5x8);
    i2c_write(ST7036_DISPLAY_ON_CURSOR_OFF);
    i2c_write(ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT);
    i2c_write(ST7036_CLEAR_DISPLAY);
    i2c_stop();
    __delay_ms(ST7036_CLEAR_DISPLAY_DELAY_MS);
}

void lcd_setBacklight(uint8_t u8Backlight)
{
    // CAT4002 DIM Reset.
    LCD_BACKLIGHT_OFF;
    __delay_ms(CAT4002_DELAY_DIM_MS);
    LCD_BACKLIGHT_ON;
    __delay_us(CAT4002_DELAY_LED_US);

    // CAT4002 DIM Pulses.
    IEC0bits.T1IE = 0b0;
    do{
        LCD_BACKLIGHT_OFF;
        LCD_BACKLIGHT_ON;
    } while(u8Backlight--);
    IEC0bits.T1IE = 0b1;
}

void lcd_setCursor(uint8_t u8Cursor)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Cursor);
    i2c_stop();
}

void lcd_writeCharacter(uint8_t u8Data)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_DATA);
    i2c_write(u8Data);
    i2c_stop();
}

void lcd_writeInstruction(uint8_t u8Data)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Data);
    i2c_stop();
}

void lcd_writeString(const uint8_t * u8Data)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_DATA);
    while(*u8Data != '\0')
        i2c_write(*u8Data++);
    i2c_stop();
}

void lcd_writeStringSetCursor(const uint8_t * u8Data, uint8_t u8Cursor)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Cursor);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_DATA);
    while(*u8Data != '\0')
        i2c_write(*u8Data++);
    i2c_stop();
}

int8_t rotary_i8encoderFilter(void)
{
    int8_t i8encoderFilter;

    IEC0bits.T1IE = 0b0;
    i8encoderFilter = u8encoderDelta;
    u8encoderDelta = i8encoderFilter & 3;
    IEC0bits.T1IE = 0b1;

    return(i8encoderFilter>>2);
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
        u8Buffer = (uint8_t)(u16Data % u8Base); // todo
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
