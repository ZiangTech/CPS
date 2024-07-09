#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


/*
 * Parameter
 */

#define B1 0.1602004f 
#define B2 0.1602004f
#define A2 -0.6795993f

#define KP_X 0.001f
#define KD_X 0.02f
#define KP_Y 0.001f
#define KD_Y 0.02f

/*
 * Common Definitions
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

#define X_LEVELED_US 1660
#define Y_LEVELED_US 1590

#define RD 100
#define PI 3.14159f
#define X_CEN  295
#define Y_CEN  385
#define T_CIR  600

/*
 * Global Variables
 */

// Setpoint positions in X and Y directions
float set_pos_x;
float set_pos_y;

// Current positions in X and Y directions
uint16_t pos_x;
uint16_t pos_y;

// Filtered positions in X and Y directions
float fpos_x, fpos_y;

// System time counter
uint16_t systime = 0;

// Last error and time values for X-axis in PD control
float last_err_x = 0;
float last_t_x = 0;

// Last error and time values for Y-axis in PD control
float last_err_y = 0;
float last_t_y = 0;

// Frequency for the circular motion calculation
float w_f = 2 * PI / T_CIR;

// Flags for managing task deadlines and axis switching
char deadline_flag;
uint16_t deadline_sum;
char axis_switch_flag;

/*
 * Timer Code
 */

void initialize_timer()
{
    // Enable RTC Oscillator
    // This effectively sets OSCCONbits.LPOSCEN = 1, but OSCCON is lock protected. 
    // __builtin_write_OSCCONL(val) unlocks and writes the value val to OSCCONL.
    __builtin_write_OSCCONL(OSCCONL | 2);

    // Disable Timer1
    T1CONbits.TON = 0;

    // Set Prescaler to 256 (TCKPS_256)
    T1CONbits.TCKPS = TCKPS_256;

    // Set internal instruction cycle clock as Clock Source
    T1CONbits.TCS = 0;

    // Disable Gated Timer mode
    T1CONbits.TGATE = 0;

    // Disable External Clock Input Synchronization
    T1CONbits.TSYNC = 0;

    // Load Timer Periods for 1ms interrupt (PR1)
    PR1 = FCY / 256 * 1 / 100;

    // Reset Timer1 Values
    TMR1 = 0x00;

    // Set Interrupt Priority
    IPC0bits.T1IP = 0x01;

    // Clear Timer1 Interrupt Flags
    IFS0bits.T1IF = 0;

    // Enable Timer1 Interrupts
    IEC0bits.T1IE = 1;

    // Enable Timer1
    T1CONbits.TON = 1;
}

/*
 * Servo Code
 */

void init_servo(char dim_of_servo)
{
    if (dim_of_servo == 'x')
    {
        // Initialize X-axis servo
        CLEARBIT(TRISDbits.TRISD7);          // Configure RD7 as output
        CLEARBIT(T2CONbits.TON);             // Disable Timer2
        CLEARBIT(T2CONbits.TCS);             // Select internal instruction cycle clock
        CLEARBIT(T2CONbits.TGATE);           // Disable Gated Timer mode
        TMR2 = 0x00;                         // Clear Timer2 register
        T2CONbits.TCKPS = 0b10;              // Select 1:64 Prescaler
        CLEARBIT(IFS0bits.T2IF);             // Clear Timer2 interrupt status flag
        CLEARBIT(IEC0bits.T2IE);             // Disable Timer2 interrupt enable control bit
        PR2 = PWM_CYC_US * (FCY / 1000000) / 64; // Set Timer2 period to 20ms
        OC8R = OC8RS = (PWM_CYC_US - PWM_MID_US) * (FCY / 1000000) / 64; // Set initial duty cycle
        OC8CON = 0x0006;                     // Configure OC8 for PWM, no fault check, Timer2
        SETBIT(T2CONbits.TON);               // Turn Timer2 on
    }
    
    if (dim_of_servo == 'y')
    {
        // Initialize Y-axis servo
        CLEARBIT(TRISDbits.TRISD6);          // Configure RD6 as output
        OC7R = OC7RS = (PWM_CYC_US - PWM_MID_US) * (FCY / 1000000) / 64; // Set initial duty cycle
        OC7CON = 0x0006;                     // Configure OC7 for PWM, no fault check, Timer2
    }
}

void set_dutycycle(char dim_of_servo, int dutycycle)
{
    // Set duty cycle for specified servo
    if (dim_of_servo == 'x')
    {
        OC8RS = (PWM_CYC_US - dutycycle) * (FCY / 1000000) / 64; // Set duty cycle for X-axis servo
    }
    else if (dim_of_servo == 'y')
    {
        OC7RS = (PWM_CYC_US - dutycycle) * (FCY / 1000000) / 64; // Set duty cycle for Y-axis servo
    }
}

/*
 * Touch screen code
 */

void init_touch_screen()
{
    // Initialize Touch Screen
    // Set up the I/O pins E1, E2, E3 to be output pins for standby
    SETBIT(PORTEbits.RE1); // Set RE1 high
    SETBIT(PORTEbits.RE2); // Set RE2 high
    CLEARBIT(PORTEbits.RE3); // Set RE3 low

    // Configure E1, E2, E3 as output pins
    CLEARBIT(TRISEbits.TRISE1); 
    CLEARBIT(TRISEbits.TRISE2); 
    CLEARBIT(TRISEbits.TRISE3); 

    // Disable ADC
    CLEARBIT(AD1CON1bits.ADON);

    // Initialize RB9 and RB15 as input pins for touch screen
    SETBIT(TRISBbits.TRISB9); // Set RB9 as input (AN9)
    SETBIT(TRISBbits.TRISB15); // Set RB15 as input (AN15)

    // Configure RB9 and RB15 as analog pins
    CLEARBIT(AD1PCFGLbits.PCFG9); // AN9
    CLEARBIT(AD1PCFGLbits.PCFG15); // AN15

    // ADC Configuration
    CLEARBIT(AD1CON1bits.AD12B); // 10-bit operation mode
    AD1CON1bits.FORM = 0; // Integer output
    AD1CON1bits.SSRC = 0x7; // Automatic conversion

    // ADC Sampling Configuration
    AD1CON2 = 0; // No scanning sampling
    AD1CON3bits.ADRC = 0; // Internal clock source
    AD1CON3bits.SAMC = 0x1F; // Sample-to-conversion clock = 31 Tad
    AD1CON3bits.ADCS = 0x2; // Tad = 3 Tcy (Time cycles)

    // Enable ADC
    SETBIT(AD1CON1bits.ADON);
}

void dim_change(char dim)
{
    // Change dimension for touch screen reading
    if (dim == 'x')
    {
        // Setup for X-coordinate reading
        CLEARBIT(PORTEbits.RE1);
        Nop(); // No operation, delay for signal stabilization
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();
        AD1CHS0bits.CH0SA = 0x000F; // Set ADC to sample AN15 (X-coordinate)
    }
    else if (dim == 'y')
    {
        // Setup for Y-coordinate reading
        SETBIT(PORTEbits.RE1);
        Nop(); // No operation, delay for signal stabilization
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        Nop();
        AD1CHS0bits.CH0SA = 0x0009; // Set ADC to sample AN9 (Y-coordinate)
    }
}

uint16_t read_position()
{
    // Read touch screen position
    SETBIT(AD1CON1bits.SAMP); // Start sampling
    while(!AD1CON1bits.DONE); // Wait until conversion is complete
    CLEARBIT(AD1CON1bits.DONE); // Clear conversion done bit
    return ADC1BUF0; // Return the ADC result
}

/*
 * PD Controller
 */

float PD_Control_X()
{
    // PD Control for X-axis
    float current_t_x = systime;
    float err_x = set_pos_x - fpos_x; // Calculate error in X position
    float P_x = KP_X * err_x;         // Proportional term
    float D_x = (err_x - last_err_x) * KD_X; // Derivative term
    float PD_x = P_x + D_x;           // Total PD control value

    // Update last values for next iteration
    last_t_x = current_t_x;
    last_err_x = err_x;

    //Clamp PD_x to the range [-1, 1]
    if (PD_x > 1) return 1;
    if (PD_x < -1) return -1;
    return PD_x;
}

void apply_PD_X(float PD_x)
{
    // Apply PD control value to X-axis servo
    uint16_t X_PD_US = X_LEVELED_US + PD_x * (PWM_MAX_US - PWM_MIN_US) / 2.0f;
    set_dutycycle('x', X_PD_US);
}

float PD_Control_Y()
{
    // PD Control for Y-axis
    float err_y = set_pos_y - fpos_y; // Calculate error in Y position
    float P_y = KP_Y * err_y;         // Proportional term
    float D_y = (err_y - last_err_y) * KD_Y; // Derivative term
    float PD_y = P_y + D_y;           // Total PD control value

    // Update last values for next iteration
    last_err_y = err_y;

    // Clamp PD_y to the range [-1, 1]
    if (PD_y > 1) return 1;
    if (PD_y < -1) return -1;
    return PD_y;
}

void apply_PD_Y(float PD_y)
{
    // Apply PD control value to Y-axis servo
    uint16_t Y_PD_US = Y_LEVELED_US + PD_y * (PWM_MAX_US - PWM_MIN_US) / 2.0f;
    set_dutycycle('y', Y_PD_US);
}

/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */

float butterworth_filter_x(float x)
{
    // Apply Butterworth filter for X-axis data
    static float x1 = 0;
    static float y1 = 0;
    float y = B1 * x + B2 * x1 - A2 * y1; // Calculate filtered output
    x1 = x; // Update previous input
    y1 = y; // Update previous output
    return y; // Return filtered value
}

float butterworth_filter_y(float x)
{
    // Apply Butterworth filter for Y-axis data
    static float x1 = 0;
    static float y1 = 0;
    float y = B1 * x + B2 * x1 - A2 * y1; // Calculate filtered output
    x1 = x; // Update previous input
    y1 = y; // Update previous output
    return y; // Return filtered value
}

void set_current_pos()
{
    // Calculate and set the current position based on system time
    set_pos_x = RD * cos(w_f * systime) + X_CEN; // X-coordinate
    set_pos_y = RD * sin(w_f * systime) + Y_CEN; // Y-coordinate
}

/*
 * main loop
 */

void control_task()
{
    // Control task for managing PD control loop
    fpos_x = butterworth_filter_x((float)pos_x);
    fpos_y = butterworth_filter_y((float)pos_y);
    set_current_pos(); // Update the current position
    float PD_x = PD_Control_X();
    apply_PD_X(PD_x); // Apply PD control to X-axis
    float PD_y = PD_Control_Y();
    apply_PD_Y(PD_y); // Apply PD control to Y-axis
}

void x_read()
{
    // Read X-axis position
    pos_x = read_position();
}

void y_read()
{
    // Read Y-axis position
    pos_y = read_position();
}

void print_deadline_task()
{
    // Print the number of missed deadlines on the LCD
    lcd_locate(0, 7);
    lcd_printf("missed deadline %d", deadline_sum);
}

void main_loop()
{
    // Main program loop
    lcd_printf("Lab06: Amazing Ball"); // Display assignment information
    lcd_locate(0, 1);
    lcd_printf("Group: 5");

    // Initialization
    init_servo('x');
    init_servo('y');
    init_touch_screen();
    set_dutycycle('x', PWM_MID_US + 160);
    set_dutycycle('y', PWM_MID_US + 60);
    dim_change('x');
    deadline_flag = 1;
    initialize_timer();

    while (TRUE) 
    {
        if (deadline_flag == 0)
        {
            // Read position and switch axis
            if (axis_switch_flag == 0)
            {
                pos_x = read_position();
                dim_change('y');
            }
            else
            {
                pos_y = read_position();
                dim_change('x');
            }

            // Control task and deadline task
            if (systime % 2 == 0) control_task();
            if (systime % 20 == 0) print_deadline_task();

            deadline_flag = 1;
        }
    }
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{
    // Timer1 Interrupt Handler
    IFS0bits.T1IF = 0; // Clear interrupt flag
    systime++;
    axis_switch_flag = !axis_switch_flag; // Toggle axis switch

    // Reset systime if it reaches the circular time
    if (systime >= T_CIR) systime = 0;

    // Increment deadline counter if deadline is missed
    if (deadline_flag == 0) deadline_sum++;
    deadline_flag = 0;
}

