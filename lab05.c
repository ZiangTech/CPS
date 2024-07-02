#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * PWM code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

// Initialize servo motors based on the specified dimension ('x' or 'y')
void init_servo(char dim_of_servo)
{
    // Servo Initialization for 'x' Dimension
    if(dim_of_servo == 'x')
    {
        CLEARBIT(TRISDbits.TRISD7); // Set RD7 (Output Compare 8) as output
        CLEARBIT(T2CONbits.TON);    // Disable Timer 2
        CLEARBIT(T2CONbits.TCS);    // Select internal instruction cycle clock
        CLEARBIT(T2CONbits.TGATE);  // Disable Gated Timer mode
        TMR2 = 0x00;                // Clear Timer 2 register
        T2CONbits.TCKPS = 0b10;     // Set Timer 2 prescaler to 1:64
        CLEARBIT(IFS0bits.T2IF);    // Clear Timer 2 interrupt flag
        CLEARBIT(IEC0bits.T2IE);    // Disable Timer 2 interrupt
        PR2 = PWM_CYC_US * (FCY / 1000000) / 64; // Set Timer 2 period (20ms)
        OC8R = (PWM_CYC_US - PWM_MID_US) * (FCY / 1000000) / 64;  // Set initial duty cycle for OC8
        OC8RS = (PWM_CYC_US - PWM_MID_US) * (FCY / 1000000) / 64; // Set secondary duty cycle for OC8
        OC8CON = 0x0006; // Configure OC8 for PWM mode without fault check, using Timer 2
        SETBIT(T2CONbits.TON); // Enable Timer 2
    }

    // Servo Initialization for 'y' Dimension
    if(dim_of_servo == 'y')
    {
        CLEARBIT(TRISDbits.TRISD6); // Set RD6 (Output Compare 7) as output
        OC7R = (PWM_CYC_US - PWM_MID_US) * (FCY / 1000000) / 64;  // Set initial duty cycle for OC7
        OC7RS = (PWM_CYC_US - PWM_MID_US) * (FCY / 1000000) / 64; // Set secondary duty cycle for OC7
        OC7CON = 0x0006; // Configure OC7 for PWM mode without fault check, using Timer 2
    }
}

// Set the duty cycle of servo motors based on the dimension ('x' or 'y') and duty cycle value
void set_dutycycle(char dim_of_servo, int dutycycle)
{
    // Set Duty Cycle for 'x' Dimension Servo
    if(dim_of_servo == 'x')
    {
        OC8RS = (PWM_CYC_US - dutycycle) * (FCY / 1000000) / 64; // Load new duty cycle for OC8
    }

    // Set Duty Cycle for 'y' Dimension Servo
    if(dim_of_servo == 'y')
    {
        OC7RS = (PWM_CYC_US - dutycycle) * (FCY / 1000000) / 64; // Load new duty cycle for OC7
    }
}

/*
 * touch screen code
 */

// Initialize touch screen functionality
void init_touch_screen()
{
    SETBIT(PORTEbits.RE1); // Set RE1 pin
    SETBIT(PORTEbits.RE2); // Set RE2 pin
    CLEARBIT(PORTEbits.RE3); // Clear RE3 pin
    CLEARBIT(TRISEbits.TRISE1); // Set TRISE1 to output
    CLEARBIT(TRISEbits.TRISE2); // Set TRISE2 to output
    CLEARBIT(TRISEbits.TRISE3); // Set TRISE3 to output
    CLEARBIT(AD1CON1bits.ADON); // Disable ADC module
    SETBIT(TRISBbits.TRISB9); // Set RB9 (AN9) as input
    SETBIT(TRISBbits.TRISB15); // Set RB15 (AN15) as input
    CLEARBIT(AD1PCFGLbits.PCFG9); // Set AN9 as analog
    CLEARBIT(AD1PCFGLbits.PCFG15); // Set AN15 as analog
    CLEARBIT(AD1CON1bits.AD12B); // Set 10-bit Operation Mode
    AD1CON1bits.FORM = 0; // Set output format to integer
    AD1CON1bits.SSRC = 0x7; // Set automatic conversion
    AD1CON2 = 0; // Not using scanning sampling
    CLEARBIT(AD1CON3bits.ADRC); // Use internal clock source
    AD1CON3bits.SAMC = 0x1F; // Auto-sample time = 31 TAD
    AD1CON3bits.ADCS = 0x2; // ADC Conversion Clock = 3 Tcy
    SETBIT(AD1CON1bits.ADON); // Enable ADC module
}

// Change the dimension for touch screen
void dim_change(char dim)
{
    if (dim == 'x')
    {
        CLEARBIT(PORTEbits.RE1); // Clear RE1 pin
        Nop(); 
        SETBIT(PORTEbits.RE2); // Set RE2 pin
        Nop(); 
        SETBIT(PORTEbits.RE3); // Set RE3 pin
        Nop(); 
        AD1CHS0bits.CH0SA = 0x000F; // Select AN15 for ADC
    }
    else if (dim == 'y')
    {
        SETBIT(PORTEbits.RE1); // Set RE1 pin
        Nop(); 
        CLEARBIT(PORTEbits.RE2); // Clear RE2 pin
        Nop(); 
        CLEARBIT(PORTEbits.RE3); // Clear RE3 pin
        Nop(); 
        AD1CHS0bits.CH0SA = 0x0009; // Select AN9 for ADC
    }
}

// Read position from ADC
uint16_t read_position()
{
    SETBIT(AD1CON1bits.SAMP); // Start ADC sampling
    while(!AD1CON1bits.DONE); // Wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); // Clear conversion done flag
    lcd_locate(0, 3);
    lcd_printf("read done");
    return ADC1BUF0; // Return the ADC result
}

/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: 5");
    
    // initialize touchscreen
    init_touch_screen();
    // initialize servos
    init_servo('x');
    init_servo('y');
    uint16_t pos_x;
    uint16_t pos_y; 
    
    while(TRUE) {
        // Move to bottom left corner
        set_dutycycle('x', PWM_MIN_US); // Set x-axis servo to minimum
        set_dutycycle('y', PWM_MIN_US); // Set y-axis servo to minimum
        dim_change('x');
        __delay_ms(10);
        pos_x = read_position();
        dim_change('y');
        __delay_ms(10);
        pos_y = read_position();
        lcd_locate(0, 5);
        lcd_printf("x/y : %03d/%03d", pos_x, pos_y);
        __delay_ms(5000);

        // Move to bottom right corner 
        set_dutycycle('y', PWM_MAX_US); 
        dim_change('x');
        __delay_ms(10);
        pos_x = read_position();
        dim_change('y');
        __delay_ms(10);
        pos_y = read_position();
        lcd_locate(0, 5);
        lcd_printf("x/y : %03d/%03d", pos_x, pos_y);
        __delay_ms(5000);

        // Move to top right corner 
        set_dutycycle('x', PWM_MAX_US);
        dim_change('x');
        __delay_ms(10);
        pos_x = read_position();
        dim_change('y');
        __delay_ms(10);
        pos_y = read_position();
        lcd_locate(0, 5);
        lcd_printf("x/y : %03d/%03d", pos_x, pos_y);
        __delay_ms(5000);
        
        // Move to top left corner
        set_dutycycle('y', PWM_MIN_US); 
        dim_change('x');
        __delay_ms(10);
        pos_x = read_position();
        dim_change('y');
        __delay_ms(10);
        pos_y = read_position();
        lcd_locate(0, 5);
        lcd_printf("x/y : %03d/%03d", pos_x, pos_y);
        __delay_ms(5000); 
    }
}
