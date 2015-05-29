/* 
 * File:   main.c
 * Author: Gerd Bartelt
 *
 * 
 */

#include <xc.h>
#include <stdint.h>
#include "main.h"

//NTC 2112813 B57421V2103J62

// CONFIG1
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config CLKOUTEN = OFF
#pragma config BOREN = OFF
#pragma config CP = OFF
#pragma config MCLRE = ON
#pragma config PWRTE = OFF
#pragma config WDTE = OFF
#pragma config FOSC = HS

// CONFIG2
#pragma config LVP = OFF
#pragma config LPBOR = OFF
#pragma config BORV = 0
#pragma config STVREN = ON
#pragma config PLLEN = ON
#pragma config VCAPEN = OFF
#pragma config WRT = OFF

typedef enum {
 MODE_TEMP = 0,
 MODE_MAX = 1,
 MODE_TRIP = 2,
 MODE_TOTAL = 3,
 MODE_KMH = 4
}en_mode;

typedef enum {
 ADC_VOLTAGE = 0,
 ADC_TEMP = 1,
 ADC_CNT = 2
}en_adc_mux;

const uint8_t stepper_tab[6] = {
    0b00000101,
    0b00000001,
    0b00001000,
    0b00001010,
    0b00000010,
    0b00000100
    };

uint8_t debounce_key_cnt = 0;
uint8_t debounced_key = 0;
uint8_t debounced_key_click = 0;
uint16_t key_long_cnt = 0;
uint8_t suppress_click = 0;


#define OFFSET_STEPPER1 16
#define OFFSET_STEPPER2 75
#define OFFSET_STEPPER3 32

uint8_t stepper1 = 0;
uint8_t stepper2 = 0;
uint8_t stepper3 = 0;

uint16_t stepperPos1 = 0;
uint16_t stepperPos2 = 0;
uint16_t stepperPos3 = 0;

uint16_t stepperSetPoint1 = 0;
uint16_t stepperSetPoint2 = 0;
uint16_t stepperSetPoint3 = 0;

en_mode mode = MODE_KMH;

// Stopwatch
uint16_t stopwatch = 0;
uint16_t stopwatch_prescaler = 0;

//Count pulsed
uint8_t pulse = 0;
uint8_t old_pulse = 0;

// Speed
uint16_t speed_raw = 0;
uint16_t speed_rawFilt = 0;
uint32_t speed_filtL = 0;
uint16_t speed = 0;
uint16_t max_speed = 0;
uint16_t period = 0;
uint8_t tmr1_ov = 0;

uint8_t speed_timeout = 0;
#define SPEED_TIMEOUT 10
// km
// Radumfang: 2160mm
// 28pole Dynamo: 1 Impuls pro 77,142857143 mm
// 1 stepper pulse per 74,074074074m
// 1 stepper pulse each 960
#define KM_PRE_DIV 960
uint16_t total_km_prelog = 0;
uint16_t total_km = 0;
uint16_t trip_km = 0;
uint16_t total_km_pre = 0;
uint16_t trip_km_pre = 0;

uint8_t toggle = 0;
uint8_t tick = 0;

// temperature
int16_t temperature = 0;
uint16_t temperature_cnt;
uint16_t voltage = 0;

en_adc_mux adc_mux = ADC_VOLTAGE;
/**
* The NTC table has 33 interpolation points.
* Unit:0.07407407 °C
*
*/
int16_t NTC_table[33] = {
  -540, -487, -344, -250, -177, -116, -63,
  -15, 30, 72, 112, 151, 189, 226, 264, 301,
  339, 377, 417, 458, 501, 540, 540, 540, 540,
  540, 540, 540, 540, 540, 540, 540, 540
};

//ADC = Volt / 27,83203125
#define UNDERVOLTAGE 107 // 3V
#define VOLTAGE_OK   180 // 5V
#define VOLTAGE_STARTUP   180 // 5V
uint8_t voltage_ok_cnt = 0;
uint8_t voltage_ok = 0;
uint16_t savecnt = 0;


/**
* \brief    Converts the ADC result into a temperature value.
*
*           P1 and p2 are the interpolating point just before and after the
*           ADC value. The function interpolates between these two points
*           The resulting code is very small and fast.
*           Only one integer multiplication is used.
*           The compiler can replace the division by a shift operation.
*
*           In the temperature range from -10°C to 50°C the error
*           caused by the usage of a table is 0.126°C
*
* \param    adc_value  The converted ADC result
* \return              The temperature in 0.07407407 °C
*
*/
int16_t NTC_ADC2Temperature(uint16_t adc_value){

  int16_t p1,p2;
  /* Estimate the interpolating point before and after the ADC value. */
  p1 = NTC_table[ (adc_value >> 5)  ];
  p2 = NTC_table[ (adc_value >> 5)+1];

  /* Interpolate between both points. */
  return p1 + ( (p2-p1) * (adc_value & 0x001F) ) / 32;
};

void Stepper_Step(uint8_t stepperMotor, uint8_t dir) {

    if (stepperMotor == 1) {
        if (dir == 1) {
            // Stepper 1
            if (stepper1<5)
                stepper1 ++;
            else
                stepper1 = 0;
        } else {
            // Stepper 1
            if (stepper1>0)
                stepper1 --;
            else
                stepper1 = 5;
        }
    }


    if (stepperMotor == 2) {
        if (dir == 1) {
            // Stepper 2
            if (stepper2<5)
                stepper2 ++;
            else
                stepper2 = 0;
        } else {
            // Stepper 2
            if (stepper2>0)
                stepper2 --;
            else
                stepper2 = 5;
        }
    }


    if (stepperMotor == 3) {
        if (dir == 1) {
            // Stepper 3
            if (stepper3<5)
                stepper3 ++;
            else
                stepper3 = 0;
        } else {
            // Stepper 3
            if (stepper3>0)
                stepper3 --;
            else
                stepper3 = 5;
        }
    }

    PORTB = stepper_tab[stepper1] | ((stepper_tab[stepper2] & 0b00001100) << 2) ;
    PORTC = (stepper_tab[stepper3] << 4) | (stepper_tab[stepper2] & 0b00000011) ;
}

/*
 * Returns the exponential threshold to cound the 1/15th kilometers
 */
uint16_t km_getThreshold(uint16_t km) {
    if (km >= 405)
        return 1000;
    else if (km >= 270)
        return 100;
    else if (km >= 135)
        return 10;

    return 1;
}

#define EEADR_STOPW_1 0
#define EEADR_STOPW_2 1
#define EEADR_TOTAL_1 2
#define EEADR_TOTAL_2 3
#define EEADR_TOTAL_3 4
#define EEADR_TRIP_1  5
#define EEADR_TRIP_2  6
#define EEADR_MAXS_1  7
#define EEADR_CHECK   8

#define EEADR_OFFSET  16

uint8_t eeprom_page = 0;

uint16_t eeprom_read_word(uint8_t addr) {
    addr = addr<<1;
    return eeprom_read(addr+1)<<8 | eeprom_read(addr);
}

void eeprom_write_word(uint8_t addr, uint16_t data) {
    addr = addr<<1;
    eeprom_write(addr, data & 0x00ff);
    eeprom_write(addr+1, (data>>8) & 0x00ff);
}

void load_content(void) {

    if (eeprom_read_word(EEADR_TOTAL_3) > 540)
        return;
    stopwatch_prescaler = eeprom_read_word(EEADR_STOPW_1);
    stopwatch           = eeprom_read_word(EEADR_STOPW_2);
    total_km_pre        = eeprom_read_word(EEADR_TOTAL_1);
    total_km_prelog     = eeprom_read_word(EEADR_TOTAL_2);
    total_km            = eeprom_read_word(EEADR_TOTAL_3);
    trip_km_pre         = eeprom_read_word(EEADR_TRIP_1);
    trip_km             = eeprom_read_word(EEADR_TRIP_2);
    max_speed           = eeprom_read_word(EEADR_MAXS_1);

    if (max_speed > 540)
        max_speed = 0;
    if (trip_km > 540)
        trip_km = 0;
    if (total_km > 540)
        total_km = 0;
    if (stopwatch > 810)
        stopwatch = 0;


}

void save_content(void) {
    eeprom_write_word(EEADR_STOPW_1,stopwatch_prescaler);
    eeprom_write_word(EEADR_STOPW_2,stopwatch);
    eeprom_write_word(EEADR_TOTAL_1,total_km_pre);
    eeprom_write_word(EEADR_TOTAL_2,total_km_prelog);
    eeprom_write_word(EEADR_TOTAL_3,total_km);
    eeprom_write_word(EEADR_TRIP_1,trip_km_pre);
    eeprom_write_word(EEADR_TRIP_2,trip_km);
    eeprom_write_word(EEADR_MAXS_1,max_speed);
}

/*
 * 
 */
void main() {
uint16_t i;
uint16_t tmp_period;

    OPTION_REG = 0b01111111; // Weak pull up enabled
    ANSELA = 0x0C; //RA2,RA3 analog
    TRISA = 0b11111110; // set RA0 to be output
    WPUA = 0b00000000; // no Pullup
    TRISB = 0b11000000; // set RB0 .. RB5 to be output (stepper motors)
    PORTB = 0b00000000;
    TRISC = 0b00001100; // set RC0,1 and RC4..RC7 to be output (stepper motors)
    PORTC = 0b00000000;
    CCP1CON = 0b00000100; // input capture every falling edge
    T1CON = 0b00110001; // Timer 1 on with 1:8 prescaler

    PIE1 = 0b00000101; // Enable input capture and TMR1 Overflow
    INTCON = 0b11000000;// Enable GIE, Enable PEIE

    //Configure the ADC module:
    //Select ADC conversion clock Frc
    ADCON1bits.ADCS = 0b111;

    //Configure voltage reference using VDD and VSS
    ADCON1bits.ADPREF = 0x00;
    ADCON1bits.ADNREF = 0x00;

    //10 bit ADC
    ADCON0bits.ADRMD = 1;
    //Select ADC input channel Pin 3
    ADCON0bits.CHS = 3;
    // Negative ref as neg input
    ADCON2bits.CHSN = 15;
    //Select result format left justified
    ADCON1bits.ADFM = 0;
    //Turn on ADC module
    ADCON0bits.ADON = 1;

    // Initialize the modules

    do {
        ADCON0bits.GO_nDONE = 1;
        __delay_ms(1);
        while(ADCON0bits.GO_nDONE == 1);
    }while ((ADRES>>6) < VOLTAGE_STARTUP);


    for (i=0; i< 560; i++) {
        Stepper_Step(1,0);
        Stepper_Step(2,0);
        Stepper_Step(3,0);
        __delay_ms(1);
    }
    for (; i< 560; i++) {
        Stepper_Step(1,0);
        Stepper_Step(3,0);
        __delay_ms(1);
    }
    for (; i< 810; i++) {
        Stepper_Step(3,0);
        __delay_ms(1);
    }

    load_content();

    while (1) {

        if ((stepperSetPoint1+OFFSET_STEPPER1) > stepperPos1) {
            stepperPos1 ++;
            Stepper_Step(1,1);
        } else if ((stepperSetPoint1+OFFSET_STEPPER1) < stepperPos1) {
            stepperPos1 --;
            Stepper_Step(1,0);
        }

        if ((stepperSetPoint2+OFFSET_STEPPER2) > stepperPos2) {
            stepperPos2 ++;
            Stepper_Step(2,1);
        } else if ((stepperSetPoint2+OFFSET_STEPPER2) < stepperPos2) {
            stepperPos2 --;
            Stepper_Step(2,0);
        }

        if ((stepperSetPoint3+OFFSET_STEPPER3) > stepperPos3) {
            stepperPos3 ++;
            Stepper_Step(3,1);
        } else if ((stepperSetPoint3+OFFSET_STEPPER3) < stepperPos3) {
            stepperPos3 --;
            Stepper_Step(3,0);
        }


        adc_mux ++;
        if (adc_mux >= ADC_CNT) {
            adc_mux = ADC_VOLTAGE;
        }

        switch (adc_mux) {
            case ADC_VOLTAGE:
                ADCON0bits.CHS = 3;
                break;
            case ADC_TEMP:
                ADCON0bits.CHS = 2;
                break;
        }

        __delay_ms(1);
        
        //Start conversion by setting the GO/DONE bit.
        ADCON0bits.GO_nDONE = 1;

        __delay_ms(1);


        while(ADCON0bits.GO_nDONE == 1);

        switch (adc_mux) {
            case ADC_VOLTAGE:
                /* Divider: 10k/47k
                 * Ref: 5V
                 * ADC = 1024 * Volt * 10k/57k / 5V
                 * ADC = Volt / 27,83203125
                 */
                voltage = ADRES>>6;
                if (voltage > VOLTAGE_OK) {
                    if (voltage_ok_cnt < 255)
                        voltage_ok_cnt ++;
                    else {
                        voltage_ok = 1;
                        if (savecnt < 65535)
                            savecnt ++;
                    }
                } else if (voltage < UNDERVOLTAGE) {
                        voltage_ok_cnt = 0;
                        voltage_ok = 0;
                        if (savecnt > 2500) // 25000 = 10sec
                            save_content();
                        savecnt = 0;
                }

                break;
            case ADC_TEMP:
                temperature_cnt ++;
                if (temperature_cnt > 1000) {
                    temperature_cnt = 0;
                    temperature = NTC_ADC2Temperature(ADRES >> 6);
                }

                break;
        }


        /*
         * Stopwatch.
         * 180 deg = 40min = 2400 sec
         * 1/3 deg = 1 step = 4.444.. sec = 40/9
         *
         * Tick period = 1000000 / 65536 = 15,258789063Hz
         */
        if (tick) {
            tick = 0;
            stopwatch_prescaler ++;
            if (stopwatch_prescaler >= 68) {// 4.4444 * 15,258789063Hz = 67,81684028
                stopwatch_prescaler = 0;
                stopwatch ++ ;
                if (stopwatch >= 810) // 3*270deg
                    stopwatch -= 810;
            }
        }
        
        stepperSetPoint3 = stopwatch;

        // Count the tacho pulses
        pulse = PORTCbits.RC2;
        // Edge detected
        if (pulse & !old_pulse) {
            // Count the total trip exponentiel
            total_km_pre ++;
            if (total_km_pre >= KM_PRE_DIV) {
                total_km_pre = 0;

                total_km_prelog ++;

                if (total_km_prelog >= km_getThreshold(total_km) ) {
                    total_km_prelog = 0;
                    total_km ++;
                }
            }

            // Count the daily trip
            trip_km_pre ++;
            if (trip_km_pre >= KM_PRE_DIV) {
                trip_km_pre = 0;
                if (trip_km< 540)
                    trip_km ++;
            }
        }
        old_pulse = pulse;



        if (!PORTCbits.RC3) {
            if (debounce_key_cnt < 50) {
                debounce_key_cnt ++;
            } else {
                if (!debounced_key) {
                    debounced_key = 1;
                }
            }
        } else {
            if (debounce_key_cnt > 0) {
                debounce_key_cnt --;
            } else {
                if (debounced_key) {
                    debounced_key = 0;
                    if (!suppress_click)
                        debounced_key_click = 1;
                    suppress_click = 0;
                }


            }
        }

        // Reset trip and stopwatch
        if (debounced_key) {
            if (key_long_cnt < 1000) {
                key_long_cnt ++;
            } else {
                trip_km_pre = 0;
                trip_km = 0;
                stopwatch_prescaler = 0;
                stopwatch = 0;
                max_speed = 0;
                suppress_click = 1;
            }
        } else {
            key_long_cnt = 0;
        }


        if (debounced_key_click) {
            debounced_key_click = 0;
            if (mode != MODE_TEMP)
                mode --;
            else
                mode = MODE_KMH;
        }

        // Period: 2500 @100Hz
        // Radumfang: 2160mm
        // 28pole Dynamo: 1 Impuls pro 77,142857143 mm
        // 1Hz = 77,142857143mm/sec = 0,277714286 km/h
        // 100Hz = 27.7714286km/h
        // 20km/h = 270 steps
        // 27.7714286km/h = 374,914285714 steps
        // 374,9.. * 2500 = 937285,714285714

        INTCONbits.GIE = 0;
        tmp_period = period;
        INTCONbits.GIE = 1;
        
        if (tmp_period!=0 && speed_timeout != SPEED_TIMEOUT)
            speed_raw = 2L*937286L / tmp_period;
        else
            speed_raw = 0;

        if (speed_raw > speed_rawFilt)
            speed_rawFilt++;
        else if (speed_raw < speed_rawFilt)
            speed_rawFilt--;


        speed_filtL += speed_rawFilt;
        speed_filtL -= speed;
        speed = speed_filtL / 256;

        //speed = speed_rawFilt;


        if (speed > 540)
            speed = 540;

        if (speed > max_speed)
            max_speed = speed;


        switch (mode) {
            case MODE_TEMP:
                stepperSetPoint2 = 0;
                if (temperature >= 0)
                    stepperSetPoint1 = temperature;
                else
                    stepperSetPoint1 = temperature + 540;
                break;
            case MODE_MAX:
                stepperSetPoint2 = 80;
                stepperSetPoint1 = max_speed;
                break;
            case MODE_TRIP:
                stepperSetPoint2 = 220;
                stepperSetPoint1 = trip_km;
                break;
            case MODE_TOTAL:
                stepperSetPoint2 = 340;
                stepperSetPoint1 = total_km;
                break;
            case MODE_KMH:
                stepperSetPoint2 = 430;
                stepperSetPoint1 = speed;
                break;                
        }


        if (toggle) {
            toggle = 0;
            PORTAbits.RA0 = 0;
        } else {
            toggle = 1;
            PORTAbits.RA0 = 1;
        }
    }
}

void interrupt ISR() {
    uint8_t hb,lb;
    uint16_t now;
    uint8_t tmr1_ov_corr;
    static uint16_t last_capture=0;
    if (CCP1IF) {
        hb = CCPR1H;
        lb = CCPR1L;

        tmr1_ov_corr = tmr1_ov;
        if (TMR1IF && (hb < 0x80))
           tmr1_ov_corr++;
        //tmr1_ov_corr = 0;
        now = (tmr1_ov_corr << 14) |  (hb<<6) | (lb>>2);
        period = now - last_capture;
        last_capture = now;
        speed_timeout = 0;
        CCP1IF = 0;
    }

    if (TMR1IF) {
        tmr1_ov++;
        if (speed_timeout != SPEED_TIMEOUT)
            speed_timeout++;
        tick = 1;
        TMR1IF = 0;
    }


}
