/**
 * Semar Mesem - A line tracker robot with simple PID system control.
 *
 * @author Akeda Bagus <admin@gedex.web.id
 * @copyright 2007
 * @license MIT
 * @chip ATMega16
 * @clock_frequency 12 MHz
 * @compiler CodeVisionAVR
 */

#include <mega16.h>
#include <delay.h>
#include <stdio.h>

// Alphanumeric LCD Module functions
#asm
   .equ __lcd_port=0x15 ;PORTC
#endasm
#include <lcd.h>

typedef unsigned char byte;

/**
 * Table for user defined character,
 * arrow that points to the top-right-corner
 */
flash byte char0[8] = {
    0b1100000,
    0b0011000,
    0b0000110,
    0b1111111,
    0b1111111,
    0b0000110,
    0b0011000,
    0b1100000
};

char lcd_buffer[33];

/**
 * Function to define user character.
 *
 * @param {byte flash} *pc
 * @param {byte} char_code
 */
void define_char(byte flash *pc, byte char_code)
{
    byte i,a;
    a = (char_code << 3) | 0x40;
    for (i = 0; i < 8; i++) lcd_write_byte(a++, *pc++);
}

/**
 * Given char data display it through LCD.
 *
 * @param {unsigned char dat} Character to display
 * @return void
 */
void lcd_please_display(unsigned char dat)
{
    unsigned char data;

    data  = dat / 100;
    data += 0x30;
    lcd_putchar(data);

    dat  %= 100;
    data  = dat / 10;
    data += 0x30;
    lcd_putchar(data);

    dat %= 10;
    data = dat + 0x30;
    lcd_putchar(data);
}

// Switch PINs for menu navigation
#define sw_ok     PINB.0
#define sw_cancel PINB.1
#define sw_up     PINB.2
#define sw_down   PINB.3

// EEPROM and initialization, written when flashing the chip
eeprom byte Kp = 10;                      // Proportional
eeprom byte Ki = 0;                       // Integral
eeprom byte Kd = 5;                       // Derivative
eeprom byte max_speed = 255;
eeprom byte min_speed = 0;

/**
 * Write to EEPROM.
 *
 * @param byte
 */
void write_to_EEPROM (byte menu_num, byte submenu_num, byte var_in_eeprom) {
    lcd_gotoxy(0, 0);
    lcd_putsf("Write to EEPROM ");
    lcd_putsf("...             ");

    switch (menu_num) {
        case 1: // PID
            switch (submenu_num) {
              case 1: // Kp
                    Kp = var_in_eeprom;
                    break;
              case 2: // Ki
                    Ki = var_in_eeprom;
                    break;
              case 3: // Kd
                    Kd = var_in_eeprom;
                    break;
            }
            break;
        case 2: // Speed
            switch (submenu_num) {
              case 1: // MAX
                    max_speed = var_in_eeprom;
                    break;
              case 2: // MIN
                    min_speed = var_in_eeprom;
                    break;
            }
            break;
    }

    delay_ms(200);
}


void set_byte( byte menu_num, byte submenu_num ) {
    byte var_in_eeprom;
    byte plus5 = 0;
    char limitPilih = -1;

    lcd_clear();
    lcd_gotoxy(0, 0);
    switch (menu_num) {
        case 1: // PID
            switch (submenu_num) {
                case 1: // Kp
                    lcd_putsf("Set Kp :        ");
                    var_in_eeprom = Kp;
                    break;
                case 2: // Ki
                    lcd_putsf("Set Ki :        ");
                    var_in_eeprom = Ki;
                    break;
                case 3: // Kd
                    lcd_putsf("Set Kd :        ");
                    var_in_eeprom = Kd;
                    break;
            }
            break;
        case 2: // Speed
            plus5 = 1;
            switch (submenu_num) {
                case 1: // MAX
                    lcd_putsf("Set MAX Speed : ");
                    var_in_eeprom = max_speed;
                    break;
                case 2: // MIN
                    lcd_putsf("Set MIN Speed : ");
                    var_in_eeprom = min_speed;
                    break;
            }
            break;
    }

    while (sw_cancel) {

        delay_ms(150);
        lcd_gotoxy(0, 1);
        lcd_please_display(var_in_eeprom);

        if (!sw_ok) { // If switch OK is pressed
            lcd_clear();
            write_to_EEPROM( menu_num, submenu_num, var_in_eeprom );
            goto exitSetByte;
        }

        if (!sw_down) { // If switch arrow-down is pressed
            if ( plus5 ) {
                if ( var_in_eeprom == 0 ) {
                    var_in_eeprom = 255;
                } else {
                    var_in_eeprom -= 5;
                }
            } else {
                if ( !limitPilih ) {
                    var_in_eeprom--;
                } else {
                    if ( var_in_eeprom == 0 ) {
                        var_in_eeprom = limitPilih;
                    } else {
                        var_in_eeprom--;
                    }
                }
            }
        }

        if (!sw_up) { // If switch arrow-up is pressed
            if ( plus5 ) {
                if ( var_in_eeprom == 255 ) {
                    var_in_eeprom = 0;
                } else {
                    var_in_eeprom += 5;
                }
            } else {
                if ( !limitPilih ) {
                    var_in_eeprom++;
                } else {
                    if ( var_in_eeprom == limitPilih ) {
                        var_in_eeprom = 0;
                    } else {
                        var_in_eeprom++;
                    }
                }
            }
        }
    }

    exitSetByte:
        delay_ms(100);
        lcd_clear();
}

byte pid_cursor, speed_cursor;

/**
 * Routine to display menu. Since we use 2 rows LCD, the scroll up/down
 * menu is by checking the switch state. Since the switch is active low,
 * the logic to check is reversed, e.g., when checking if switch OK is
 * pressed:
 *
 * ~~~
 * if (!sw_ok)
 * ~~~
 */
void show_menu_in_lcd() {
    lcd_clear();

    menu01:
        delay_ms(125); // Prevents switch bouncing
        lcd_gotoxy(0,0);

        lcd_putsf("  Set PID       ");
        lcd_gotoxy(0,1);
        lcd_putsf("  Set Speed     ");

        lcd_gotoxy(0,0); // Cursor in first row
        lcd_putchar(0);

        if (!sw_ok) {
            lcd_clear();
            pid_cursor = 1;
            goto set_PID;
        }
        if (!sw_down) {
            goto menu02;
        }
        if (!sw_up) {
            lcd_clear();
            goto menu05;
        }

        goto menu01;

    menu02:
        delay_ms(125);
        lcd_gotoxy(0,0);

        lcd_putsf("  Set PID       ");
        lcd_gotoxy(0,1);
        lcd_putsf("  Set Speed     ");

        lcd_gotoxy(0,1); // Cursor in 2nd row
        lcd_putchar(0);

        if (!sw_ok) {
            lcd_clear();
            speed_cursor = 1;
            goto set_speed;
        }
        if (!sw_up) {
            goto menu01;
        }
        if (!sw_down) {
            lcd_clear();
            goto menu03;
       }
        goto menu02;

    menu03:
        delay_ms(125);
        lcd_gotoxy(0,0);

        lcd_putsf("  Start!!      ");
        lcd_gotoxy(0,0); // Cursor in first row
        lcd_putchar(0);

        if (!sw_ok) {
            lcd_clear();
            goto start_robot;
        }
        if (!sw_up) {
            lcd_clear();
            goto menu02;
        }
        if (!sw_down) {
            lcd_clear();
            goto menu01;
        }

        goto menu03;

    set_PID:
        delay_ms(150);
        lcd_gotoxy(0,0);

        lcd_putsf("  Kp   Ki   Kd  ");

        lcd_putchar(' ');
        lcd_please_display(Kp); lcd_putchar(' '); lcd_putchar(' ');
        lcd_please_display(Ki); lcd_putchar(' '); lcd_putchar(' ');
        lcd_please_display(Kd); lcd_putchar(' '); lcd_putchar(' ');

        switch (pid_cursor) {
            case 1:
                lcd_gotoxy(1,0); // Kp cursor
                lcd_putchar(0);
                break;
            case 2:
                lcd_gotoxy(6,0); // Ki cursor
                lcd_putchar(0);
                break;
            case 3:
                lcd_gotoxy(11,0); // Kd cursor
                lcd_putchar(0);
                break;
        }

        if (!sw_ok) {
            set_byte( 1, pid_cursor);
            delay_ms(200);
        }
        if (!sw_up) {
            if (pid_cursor == 3) {
                pid_cursor = 1;
            } else {
                pid_cursor++;
            }
        }
        if (!sw_down) {
            if (pid_cursor == 1) {
                pid_cursor = 3;
            } else {
                pid_cursor--;
            }
        }
        if (!sw_cancel) {
            lcd_clear();
            goto menu01;
        }

        goto set_PID;

    set_speed:
        delay_ms(150);
        lcd_gotoxy(0,0);

        lcd_putsf("   MAX    MIN   ");
        lcd_putchar(' ');lcd_putchar(' ');lcd_putchar(' ');

        lcd_please_display(max_speed);
        lcd_putchar(' '); lcd_putchar(' ');lcd_putchar(' '); lcd_putchar(' ');
        lcd_please_display(min_speed);
        lcd_putchar(' ');lcd_putchar(' ');lcd_putchar(' ');

        switch (speed_cursor) {
            case 1:
                lcd_gotoxy(2,0); // kursor MAX
                lcd_putchar(0);
                break;
            case 2:
                lcd_gotoxy(9,0); // kursor MIN
                lcd_putchar(0);
                break;
        }

        if (!sw_ok) {
            set_byte( 2, speed_cursor);
            delay_ms(200);
        }
        if (!sw_up) {
            if (speed_cursor == 2) {
                speed_cursor = 1;
            } else {
                speed_cursor++;
            }
        }
        if (!sw_down) {
            if (speed_cursor == 1) {
                speed_cursor = 2;
            } else {
                speed_cursor--;
            }
        }
        if (!sw_cancel) {
            lcd_clear();
            goto menu02;
        }
        goto set_speed;

    start_robot:
        lcd_clear();

}

#define sensor  PINA
#define s0      PINA.0
#define s1      PINA.1
#define s2      PINA.2
#define s3      PINA.3
#define s4      PINA.4
#define s5      PINA.5
#define s6      PINA.6
#define s7      PINA.7

#define sKi     PINB.5
#define sKa     PINB.6

#define motor_left_enabled  PORTD.1
#define motor_left_plus     PORTD.2
#define motor_left_min      PORTD.3
#define motor_right_enabled PORTD.6
#define motor_right_plus    PORTD.5
#define motor_right_min     PORTD.4

unsigned char xcount;
int lpwm, rpwm, MAXPWM, MINPWM, intervalPWM;
byte diffPWM = 5; // utk kiri
// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
        // Place your code here
    xcount++;
    if (xcount <= lpwm)
        motor_left_enabled=1;
    else
        motor_left_enabled=0;

    if ( xcount <= rpwm )
        motor_right_enabled=1;
    else
        motor_right_enabled=0;

    TCNT0=0xFF;
}

void robot_moves_forward()
{
    motor_right_plus  = 1;
    motor_left_plus = 1;
    motor_right_min   = 0;
    motor_left_min  = 0;
}

void robot_moves_backward()
{
    motor_right_plus  = 0;
    motor_left_plus = 0;
    motor_right_min   = 1;
    motor_left_min  = 1;
}

void robot_turns_right()
{
    motor_right_plus = 0;
    motor_right_min  = 1;
}

void robot_turns_left()
{
    motor_left_plus = 0;
    motor_left_min  = 1;
}

void robot_stops()
{
    rpwm    = 0;
    lpwm    = 0;
    motor_right_plus  = 0;
    motor_left_plus = 0;
    motor_right_min   = 0;
    motor_left_min  = 0;
}

int MV, P, I, D, PV, error, last_error, rate;
int var_Kp, var_Ki, var_Kd;
unsigned char max_MV = 100;
unsigned char min_MV = -100;
unsigned char SP = 0;

/**
 * Scan the line
 */
void scan_line() {
    switch (sensor) {
        case 0b11111110: // Most left
            PV = -7;
            robot_moves_forward();
            break;
        case 0b11111000:
        case 0b11111100:
            PV = -6;
            robot_moves_forward();
            break;
        case 0b11111101:
            PV = -5;
            robot_moves_forward();
            break;
        case 0b11110001:
        case 0b11111001:
            PV = -4;
            robot_moves_forward();
            break;
        case 0b11111011:
            PV = -3;
            robot_moves_forward();
            break;
        case 0b11100011:
        case 0b11110011:
            PV = -2;
            robot_moves_forward();
            break;
        case 0b11110111:
            PV = -1;
            robot_moves_forward();
            break;
        case 0b11100111: // Center
            PV = 0;
            robot_moves_forward();
            break;
        case 0b11101111:
            PV = 1;
            robot_moves_forward();
            break;
        case 0b11000111:
        case 0b11001111:
            PV = 2;
            robot_moves_forward();
            break;
        case 0b11011111:
            PV = 3;
            robot_moves_forward();
            break;
        case 0b10001111:
        case 0b10011111:
            PV = 4;
            robot_moves_forward();
            break;
        case 0b10111111:
            PV = 5;
            robot_moves_forward();
            break;
        case 0b00011111:
        case 0b00111111:
            PV = 6;
            robot_moves_forward();
            break;
        case 0b01111111: // Most right
            PV = 7;
            robot_moves_forward();
            break;
        case 0b11111111: // Loss
            if (PV < 0) {
                // PV = -8;
                lpwm = 150;
                rpwm = 185;
                robot_turns_left();
                goto exit;
            } else if (PV > 0) {
                // PV = 8;
                lpwm = 180;
                rpwm = 155;
                robot_turns_right();
                goto exit;
            }
    }

    error = SP - PV;
    P = (var_Kp * error) / 10;

    I = I + error;
    I = (I * var_Ki) / 10;

    rate = error - last_error;
    D    = (rate * var_Kd) / 10;

    last_error = error;

    MV = P + I + D;

    if (MV == 0) {
        lpwm = MAXPWM - diffPWM;
        rpwm = MAXPWM;
    } else if (MV > 0) { // Moves left
        rpwm = MAXPWM - ((intervalPWM - 20) * MV);
        lpwm = (MAXPWM - (intervalPWM * MV) - 15) - diffPWM;

        if (lpwm < MINPWM) lpwm = MINPWM;
        if (lpwm > MAXPWM) lpwm = MAXPWM;
        if (rpwm < MINPWM) rpwm = MINPWM;
        if (rpwm > MAXPWM) rpwm = MAXPWM;
    } else if (MV < 0) { // Moves right
        lpwm = MAXPWM + ( ( intervalPWM - 20 ) * MV);
        rpwm = MAXPWM + ( ( intervalPWM * MV ) - 15 );

        if (lpwm < MINPWM) lpwm = MINPWM;
        if (lpwm > MAXPWM) lpwm = MAXPWM;
        if (rpwm < MINPWM) rpwm = MINPWM;
        if (rpwm > MAXPWM) rpwm = MAXPWM;
    }

    exit:
}

void main(void)
{
    // sensor
    PORTA=0x00;
    DDRA=0x00;

    //switch & sKi & sKa
    PORTB=0x0F;
    DDRB=0x00;

    //lcd
    PORTC=0x00;
    DDRC=0x00;

    //motor
    PORTD=0x00;
    DDRD=0xFF;

    // Timer/Counter 0 initialization
    TCCR0=0x00;
    TCNT0=0x00;
    OCR0=0x00;

    // Timer/Counter 1 initialization
    TCCR1A=0x00;
    TCCR1B=0x00;
    TCNT1H=0x00;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;

    // Timer/Counter 2 initialization
    ASSR=0x00;
    TCCR2=0x00;
    TCNT2=0x00;
    OCR2=0x00;

    // External Interrupt(s) initialization
    MCUCR=0x00;
    MCUCSR=0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=0x01;

    // Analog Comparator initialization
    // Analog Comparator: Off
    // Analog Comparator Input Capture by Timer/Counter 1: Off
    ACSR=0x80;
    SFIOR=0x00;

    // LCD module initialization
    lcd_init(16);

    /* define user character 0 */
    define_char(char0,0);

    // stop motor
    TCCR0=0x00;
    robot_stops();

    show_menu_in_lcd();

    TCCR0=0x05;
    #asm("sei")

    // read eeprom
    var_Kp  = Kp;
    var_Ki  = Ki;
    var_Kd  = Kd;
    MAXPWM  = (int)max_speed + 1;
    MINPWM  = min_speed;

    intervalPWM = (max_speed - min_speed) / 8;
    PV          = 0;
    error       = 0;
    last_error  = 0;

    robot_moves_forward();
    while (1) {
        scan_line();
    };
}
