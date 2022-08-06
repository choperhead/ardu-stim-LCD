/* vim: set syntax=c expandtab sw=2 softtabstop=2 autoindent smartindent smarttab : */
/*
 * Arbritrary wheel pattern generator
 *
 * copyright 2014 David J. Andruczyk
 * 
 * Ardu-Stim software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ArduStim software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with any ArduStim software.  If not, see http://www.gnu.org/licenses/
 *
 */

#include "defines.h"
#include "ardustim.h"
#include "enums.h"
#include "comms.h"
#include "storage.h"
#include "user_defaults.h"
#include "wheel_defs.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>   //M.M.


/* Sensistive stuff used in ISR's */
volatile uint8_t fraction = 0;

volatile uint16_t adc0; /* Buttons */  //M.M.
volatile uint16_t adc1; /* POT RPM */  //pot moved to adc1, adc0 now is for buttons

volatile uint32_t oc_remainder = 0;
/* Setting rpm to any value over 0 will enabled sweeping by default */
/* Stuff for handling prescaler changes (small tooth wheels are low RPM) */
volatile uint8_t analog_port = 0;
volatile bool adc0_read_complete = false;
volatile bool adc1_read_complete = false;
volatile bool reset_prescaler = false;
volatile bool normal = true;
volatile bool sweep_reset_prescaler = true; /* Force sweep to reset prescaler value */
volatile bool sweep_lock = false;
volatile uint8_t output_invert_mask = 0x00; /* Don't invert anything */
volatile uint8_t sweep_direction = ASCENDING;
volatile byte total_sweep_stages = 0;
volatile uint8_t sweep_stage = 0;
volatile uint8_t prescaler_bits = 0;
volatile uint8_t last_prescaler_bits = 0;
volatile uint8_t mode = 0;
volatile uint16_t new_OCR1A = 5000; /* sane default */
volatile uint16_t edge_counter = 0;

/* Less sensitive globals */
uint8_t bitshift = 0;
uint16_t sweep_low_rpm = 250;
uint16_t sweep_high_rpm = 4000;
uint16_t sweep_rate = 1;                               

sweep_step *SweepSteps;  /* Global pointer for the sweep steps */

// LCD definitions M.M.
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

unsigned long previousMillis = 0;
const long interval = 200;  
volatile bool debouce = true;  
volatile bool serial_upd = false; 
 
#define pri_out		12
#define pri_outN	13

#define sec_out		2
#define sec_outN	3


// M.M. representacion en grados puede ser 360 o 720 (segun si tiene CMP o no)
// cuando es 360  Prescaler = (cant flancos) / 120
// cuando es 720  Prescaler = (cant flancos) / 240

wheels Wheels[MAX_WHEELS] = {
#if defined	_HONDA_30DEG
  { honda_30deg_friendly_name, Honda_30deg, 0.1, 12, 360 },
#endif
#if defined	_HONDA_30DEG90
  { honda_30deg_90_friendly_name, Honda_30deg90, 0.1, 12, 360 },
#endif
#if defined	_HONDA_30DEG60
  { honda_30deg_60_friendly_name, Honda_30deg60, 0.1, 12, 360 },
#endif
#if defined	_YAMAHA_60DEG
  { yamaha_60deg_friendly_name, Yamaha_60deg, 0.1, 12, 360 },
#endif
#if defined	_SINGLE40
  { single40_friendly_name, Single40, 0.075, 9, 360 },
#endif

#if defined	_KAWAZX6
  { kawasaki_zx6_friendly_name, KawasakiZX6, 0.6, 72, 360 },
#endif
#if defined	_KAWAZXR250
  { kawasaki_zxr250_friendly_name, KawasakiZXR250, 0.5, 60, 360 },
#endif
#if defined	_SUZIBANDIT
  { suzuki_bandit_friendly_name, SuzukiBandit, 0.6, 72, 360 },
#endif


#if defined	_BMW_BOXER
  { bmwp_friendly_name, bmwp, 0.0667, 8, 360 },
#endif

#if defined	_BMW_BOXER_
  { bmwn_friendly_name, bmwn, 0.0667, 8, 360 },
#endif

#if defined	_HONDA_CG150_
  { honda_cg150_friendly_name, hondacg150, 0.6, 72, 360 },
#endif

#if defined	_DUCATI_
  { Ducati_friendly_name, ducati48_3, 0.8, 96, 360 },
#endif

#if defined	_VIRAGO250_
  { Virago250_friendly_name, virago250, 0.3, 36, 360 },
#endif



//--------------------------------------------------------------
  /* Pointer to friendly name string, pointer to edge array, RPM Scaler, Number of edges in the array, whether the number of edges covers 360 or 720 degrees */
#if defined _DIZZY_FOUR_CYLINDER   
  { dizzy_four_cylinder_friendly_name, dizzy_four_cylinder, 0.03333, 4, 360 },
#endif  
#if defined _DIZZY_SIX_CYLINDER    
  { dizzy_six_cylinder_friendly_name, dizzy_six_cylinder, 0.05, 6, 360 },
#endif  
#if defined _DIZZY_EIGHT_CYLINDER     
  { dizzy_eight_cylinder_friendly_name, dizzy_eight_cylinder, 0.06667, 8, 360 },
#endif  
#if defined _SIXTY_MINUS_TWO         
  { sixty_minus_two_friendly_name, sixty_minus_two, 1.0, 120, 360 },
#endif  
#if defined _SIXTY_MINUS_TWO_WITH_CAM    
  { sixty_minus_two_with_cam_friendly_name, sixty_minus_two_with_cam, 1.0, 240, 720 },
#endif  
#if defined _SIXTY_MINUS_TWO_WITH_HALFMOON_CAM   
  { sixty_minus_two_with_halfmoon_cam_friendly_name, sixty_minus_two_with_halfmoon_cam, 1.0, 240, 720 },
#endif  
#if defined _THIRTY_SIX_MINUS_ONE    
  { thirty_six_minus_one_friendly_name, thirty_six_minus_one, 0.6, 72, 360 },
#endif  
#if defined _TWENTY_FOUR_MINUS_ONE    
  { twenty_four_minus_one_friendly_name, twenty_four_minus_one, 0.5, 48, 360 },
#endif  
#if defined _FOUR_MINUS_ONE_WITH_CAM     
  { four_minus_one_with_cam_friendly_name, four_minus_one_with_cam, 0.06667, 16, 720 },
#endif  
#if defined _EIGHT_MINUS_ONE          
  { eight_minus_one_friendly_name, eight_minus_one, 0.13333, 16, 360 },
#endif  
#if defined _SIX_MINUS_ONE_WITH_CAM    
  { six_minus_one_with_cam_friendly_name, six_minus_one_with_cam, 0.15, 36, 720 },
#endif  
#if defined _TWELVE_MINUS_ONE_WITH_CAM   
  { twelve_minus_one_with_cam_friendly_name, twelve_minus_one_with_cam, 0.6, 144, 720 },
#endif  
#if defined _FOURTY_MINUS_ONE        
  { fourty_minus_one_friendly_name, fourty_minus_one, 0.66667, 80, 360 },
#endif  
#if defined _DIZZY_FOUR_TRIGGER_RETURN     
  { dizzy_four_trigger_return_friendly_name, dizzy_four_trigger_return, 0.15, 9, 720 },
#endif  
#if defined _ODDFIRE_VR                
  { oddfire_vr_friendly_name, oddfire_vr, 0.2, 24, 360 },
#endif  
#if defined _OPTISPARK_LT1            
  { optispark_lt1_friendly_name, optispark_lt1, 3.0, 720, 720 },
#endif  
#if defined _TWELVE_MINUS_THREE       
  { twelve_minus_three_friendly_name, twelve_minus_three, 0.4, 48, 360 },
#endif  
#if defined _THIRTY_SIX_MINUS_TWO_TWO_TWO     
  { thirty_six_minus_two_two_two_friendly_name, thirty_six_minus_two_two_two, 0.6, 72, 360 },
#endif  
#if defined _THIRTY_SIX_MINUS_TWO_TWO_TWO_H6    
  { thirty_six_minus_two_two_two_h6_friendly_name, thirty_six_minus_two_two_two_h6, 0.6, 72, 360 },
#endif  
#if defined _THIRTY_SIX_MINUS_TWO_TWO_TWO_WITH_CAM    
  { thirty_six_minus_two_two_two_with_cam_friendly_name, thirty_six_minus_two_two_two_with_cam, 0.6, 144, 720 },
#endif  
#if defined _FOURTY_TWO_HUNDRED_WHEEL   
  { fourty_two_hundred_wheel_friendly_name, fourty_two_hundred_wheel, 0.6, 72, 360 },
#endif  
#if defined _THIRTY_SIX_MINUS_ONE_WITH_CAM_FE3   
  { thirty_six_minus_one_with_cam_fe3_friendly_name, thirty_six_minus_one_with_cam_fe3, 0.6, 144, 720 },
#endif  
#if defined _SIX_G_SEVENTY_TWO_WITH_CAM    
  { six_g_seventy_two_with_cam_friendly_name, six_g_seventy_two_with_cam, 0.6, 144, 720 },
#endif  
#if defined _BUELL_ODDFIRE_CAM       
  { buell_oddfire_cam_friendly_name, buell_oddfire_cam, 0.33333, 80, 720 },
#endif  
#if defined _GM_LS1_CRANK_AND_CAM      
  { gm_ls1_crank_and_cam_friendly_name, gm_ls1_crank_and_cam, 6.0, 720, 720 },
#endif  
#if defined _LOTUS_THIRTY_SIX_MINUS_ONE_ONE_ONE_ONE     
  { lotus_thirty_six_minus_one_one_one_one_friendly_name, lotus_thirty_six_minus_one_one_one_one, 0.6, 72, 360 },
#endif  
#if defined _HONDA_RC51_WITH_CAM     
  { honda_rc51_with_cam_friendly_name, honda_rc51_with_cam, 0.2, 48, 720 },
#endif  
#if defined _THIRTY_SIX_MINUS_ONE_WITH_SECOND_TRIGGER     
  { thirty_six_minus_one_with_second_trigger_friendly_name, thirty_six_minus_one_with_second_trigger, 0.6, 144, 720 },
#endif  
#if defined _CHRYSLER_NGC_THIRTY_SIX_PLUS_TWO_MINUS_TWO_WITH_NGC4_CAM    
  { chrysler_ngc_thirty_six_plus_two_minus_two_with_ngc4_cam_friendly_name, chrysler_ngc_thirty_six_plus_two_minus_two_with_ngc4_cam, 3.0, 720, 720 },
#endif  
#if defined _CHRYSLER_NGC_THIRTY_SIX_MINUS_TWO_PLUS_TWO_WITH_NGC6_CAM    
  { chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc6_cam_friendly_name, chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc6_cam, 3.0, 720, 720 },
#endif  
#if defined _CHRYSLER_NGC_THIRTY_SIX_MINUS_TWO_PLUS_TWO_WITH_NGC8_CAM    
  { chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc8_cam_friendly_name, chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc8_cam, 3.0, 720, 720 },
#endif  
#if defined _WEBER_IAW_WITH_CAM     
  { weber_iaw_with_cam_friendly_name, weber_iaw_with_cam, 1.2, 144, 720 },
#endif  
#if defined _FIAT_ONE_POINT_EIGHT_SIXTEEN_VALVE_WITH_CAM     
  { fiat_one_point_eight_sixteen_valve_with_cam_friendly_name, fiat_one_point_eight_sixteen_valve_with_cam, 3.0, 720, 720 },
#endif  
#if defined _THREE_SIXTY_NISSAN_CAS    
  { three_sixty_nissan_cas_friendly_name, three_sixty_nissan_cas, 3.0, 720, 720 },
#endif  
#if defined _TWENTY_FOUR_MINUS_TWO_WITH_SECOND_TRIGGER    
  { twenty_four_minus_two_with_second_trigger_friendly_name, twenty_four_minus_two_with_second_trigger, 0.3, 72, 720 },
#endif  
#if defined _YAMAHA_EIGHT_TOOTH_WITH_CAM     
  { yamaha_eight_tooth_with_cam_friendly_name, yamaha_eight_tooth_with_cam, 0.26667, 64, 720 },
#endif  
#if defined _GM_FOUR_TOOTH_WITH_CAM    
  { gm_four_tooth_with_cam_friendly_name, gm_four_tooth_with_cam, 0.06666, 8, 720 },
#endif  
#if defined _GM_SIX_TOOTH_WITH_CAM     
  { gm_six_tooth_with_cam_friendly_name, gm_six_tooth_with_cam, 0.1, 12, 720 },
#endif  
#if defined _GM_EIGHT_TOOTH_WITH_CAM    
  { gm_eight_tooth_with_cam_friendly_name, gm_eight_tooth_with_cam, 0.13333, 16, 720 },
#endif  
#if defined _VOLVO_D12ACD_WITH_CAM    
  { volvo_d12acd_with_cam_friendly_name, volvo_d12acd_with_cam, 4.0, 480, 720 },
#endif  
#if defined _MAZDA_THIRTY_SIX_MINUS_TWO_TWO_TWO_WITH_SIX_TOOTH_CAM   
  { mazda_thirty_six_minus_two_two_two_with_six_tooth_cam_friendly_name, mazda_thirty_six_minus_two_two_two_with_six_tooth_cam, 1.5, 360, 720 },
#endif  
#if defined _MITSUBISH_4g63_4_2    
  { mitsubishi_4g63_4_2_friendly_name, mitsubishi_4g63_4_2, 0.6, 144, 720 },
#endif  
#if defined _AUDI_135_WITH_CAM    
  { audi_135_with_cam_friendly_name, audi_135_with_cam, 1.5, 1080, 720 },
#endif  
#if defined _HONDA_D17_NO_CAM   
  { honda_d17_no_cam_friendly_name, honda_d17_no_cam, 0.6, 144, 720 },
#endif  
#if defined _MAZDA_323_AU    
  { mazda_323_au_friendly_name, mazda_323_au, 1, 30, 720 },
#endif  
#if defined _DAIHATSU_3CYL    
  { daihatsu_3cyl_friendly_name, daihatsu_3cyl, 0.8, 144, 360 },
#endif  
#if defined _MIATA_9905    
  { miata_9905_friendly_name, miata_9905, 0.6, 144, 720 },
#endif  
#if defined _TWELVE_WITH_CAM     
  { twelve_with_cam_friendly_name, twelve_with_cam, 0.6, 144, 720 },
#endif  
#if defined _TWENTY_FOUR_WITH_CAM   
  { twenty_four_with_cam_friendly_name, twelve_with_cam, 0.6, 144, 720 },
#endif  
#if defined _SUBARU_SIX_SEVEN      
  { subaru_six_seven_name_friendly_name, subaru_six_seven, 3.0, 720, 720 },
#endif  
#if defined _GM_7X                 
  { gm_seven_x_friendly_name, gm_seven_x, 1.502, 180, 720 },
#endif  
#if defined _FOUR_TWENTY_A         
  { four_twenty_a_friendly_name, four_twenty_a, 0.6, 144, 720 },
#endif  
#if defined _FORD_ST170            
  { ford_st170_friendly_name, ford_st170, 3.0, 720, 720 },
#endif  
#if defined _MITSUBISHI_3A92       
  { mitsubishi_3A92_friendly_name, mitsubishi_3A92, 0.6, 144, 720 },
#endif  
#if defined _TOYOTA_4AGE_CAS       
  { Toyota_4AGE_CAS_friendly_name, toyota_4AGE_CAS, 0.333, 144, 720 },
#endif  
#if defined _TOYOTA_4AGZE    
  { Toyota_4AGZE_friendly_name, toyota_4AGZE, 0.333, 144, 720 },
#endif  
#if defined _SUZUKI_DRZ400
  { Suzuki_DRZ400_friendly_name, suzuki_DRZ400,0.6, 72, 360},
#endif  
};
/* Initialization */
void setup() {
  serialSetup();
  loadConfig();
  lcd.begin(16, 2);
  lcd.print(" Ardu-Stim LCD  ");
  
  lcd.setCursor(0,1);
  lcd.print("  20/07/2022 ");	
  
  delay(2000);
  
  LCD_Update(0);
  LCD_Update(1);

  cli(); // stop interrupts

  /* Configuring TIMER1 (pattern generator) */
  // Set timer1 to generate pulses
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Set compare register to sane default
  OCR1A = 1000;  /* 8000 RPM (60-2) */

  // Turn on CTC mode
  TCCR1B |= (1 << WGM12); // Normal mode (not PWM)
  // Set prescaler to 1
  TCCR1B |= (1 << CS10); /* Prescaler of 1 */
  // Enable output compare interrupt for timer channel 1 (16 bit)
  TIMSK1 |= (1 << OCIE1A);

  // Set timer2 to run sweeper routine
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // Set compare register to sane default
  OCR2A = 249;  /* With prescale of x64 gives 1ms tick */

  // Turn on CTC mode
  TCCR2A |= (1 << WGM21); // Normal mode (not PWM)
  // Set prescaler to x64
  TCCR2B |= (1 << CS22); /* Prescaler of 64 */
  // Enable output compare interrupt for timer channel 2
  TIMSK2 |= (1 << OCIE2A);


  /* Configure ADC as per http://www.glennsweeney.com/tutorials/interrupt-driven-analog-conversion-with-an-atmega328p */
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  //ADMUX &= B11011111; //M.M.
  
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  //ADMUX |= B01000000;  //M.M.
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  //ADMUX &= B11110000;  //M.M.
  
  ADMUX = B01000000;  
  
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  // ADMUX |= 8;
  // ADMUX |= B00001000; // Binary equivalent
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
  
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= B00100000;
  
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= B11111000;
  
  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= B00000111;
  
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;

//  pinMode(7, OUTPUT); /* Debug pin for Saleae to track sweep ISR execution speed */
  pinMode(8, OUTPUT); /* Primary (crank usually) output */
  pinMode(9, OUTPUT); /* Secondary (cam usually) output */
//  pinMode(10, OUTPUT); /* Knock signal for seank, ony on LS1 pattern, NOT IMPL YET */
  

  
  pinMode(10, INPUT);  /* LCD BL must be input*/
  
  pinMode(pri_out, OUTPUT); /* Primary Out */
  pinMode(pri_outN, OUTPUT); /* Primary Out inverted*/
  pinMode(sec_out, OUTPUT); /* Secondary Out */
  pinMode(sec_outN, OUTPUT); /* Secondary Out inverted*/
  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  pinMode(53, OUTPUT); 
  pinMode(52, OUTPUT); 
#endif

  sei(); // Enable interrupts
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |= B01000000;
  /* Make sure we are using the DEFAULT RPM on startup */
  reset_new_OCR1A(wanted_rpm); 

} // End setup


//! ADC ISR for alternating between ADC pins 0 and 1
/*!
 * Reads ADC ports 0 and 1 alternately. Port 0 is for buttons , Port 1 is for RPM
 */
 
ISR(ADC_vect){
uint16_t   adc_conv;	
	
  adc_conv = ADCL | (ADCH << 8);	
	
  switch(analog_port)
  {
	  case 0:
		adc0 = adc_conv;
		ADMUX = B01000000;	
		analog_port = 1;
		adc0_read_complete = true;
	    break;
	 case 1:
		adc1 = adc_conv;
		ADMUX = B01000001;	
		analog_port = 0;
		adc1_read_complete = true;
	    break;		  
  }  
	ADCSRA |= B01000000;
	
}

/* This is the "low speed" 1000x/second sweeper interrupt routine
 * who's sole purpose in life is to reset the output compare value
 * for timer zero to change the output RPM.  In cases where the RPM
 * change per ISR is LESS than one LSB of the counter a set of modulus
 * variabels are used to handle fractional values.
 */
ISR(TIMER2_COMPA_vect) {
//  PORTD = (1 << 7);
  if ( mode != LINEAR_SWEPT_RPM)
  {
//    PORTD = (0 << 7);
    return;
  }
  if (sweep_lock)  // semaphore to protect around changes/critical sections
  {  
 //   PORTD = (0 << 7);
    return;
  }
  sweep_lock = true;
  if (sweep_reset_prescaler)
  {
    sweep_reset_prescaler = false;
    reset_prescaler = true;
    prescaler_bits = SweepSteps[sweep_stage].prescaler_bits;
    last_prescaler_bits = prescaler_bits;  
  }
  /* Sweep code */
  if (sweep_direction == ASCENDING)
  {
    oc_remainder += SweepSteps[sweep_stage].remainder_per_isr;
    /* IF the total is over the threshold we increment the TCNT factor
     * for each multiple it is over by
     */
    while (oc_remainder > FACTOR_THRESHOLD)
    {
      fraction++;
      oc_remainder -= FACTOR_THRESHOLD;
    }
    if (new_OCR1A > SweepSteps[sweep_stage].ending_ocr)
    {
      new_OCR1A -= (SweepSteps[sweep_stage].tcnt_per_isr + fraction);
      fraction = 0;
    }
    else /* END of the stage, find out where we are */
    {
      sweep_stage++;
      oc_remainder = 0;
      if (sweep_stage < total_sweep_stages)
      {
        /* Toggle  when changing stages */
        //PORTD &= ~(1<<7); /* turn DBG pin off */
        //PORTD |= (1<<7);  /* Turn DBG pin on */
        new_OCR1A = SweepSteps[sweep_stage].beginning_ocr;
        if (SweepSteps[sweep_stage].prescaler_bits != last_prescaler_bits)
          sweep_reset_prescaler = true;
      }
      else /* END of line, time to reverse direction */
      {
        sweep_stage--; /*Bring back within limits */
        sweep_direction = DESCENDING;
        new_OCR1A = SweepSteps[sweep_stage].ending_ocr;
        if (SweepSteps[sweep_stage].prescaler_bits != last_prescaler_bits)
          sweep_reset_prescaler = true;
        PORTD |= 1 << 7;  /* Debugginga, ascending */
      }
      /* Reset fractionals or next round */
    }
  }
  else /* Descending */
  {
    oc_remainder += SweepSteps[sweep_stage].remainder_per_isr;
    while (oc_remainder > FACTOR_THRESHOLD)
    {
      fraction++;
      oc_remainder -= FACTOR_THRESHOLD;
    }
    if (new_OCR1A < SweepSteps[sweep_stage].beginning_ocr)
    {
      new_OCR1A += (SweepSteps[sweep_stage].tcnt_per_isr + fraction);
      fraction = 0;
    }
    else /* End of stage */
    {
      sweep_stage--;
      oc_remainder = 0;
      if (sweep_stage >= 0)
      {
        new_OCR1A = SweepSteps[sweep_stage].ending_ocr;
        if (SweepSteps[sweep_stage].prescaler_bits != last_prescaler_bits)
          sweep_reset_prescaler = true;
      }
      else /*End of the line */
      {
        sweep_stage++; /*Bring back within limits */
        sweep_direction = ASCENDING;
        new_OCR1A = SweepSteps[sweep_stage].beginning_ocr;
        if (SweepSteps[sweep_stage].prescaler_bits != last_prescaler_bits)
          sweep_reset_prescaler = true;
        PORTD &= ~(1<<7);  /*Descending  turn pin off */
      }
    }
  }
  sweep_lock = false;
  //wanted_rpm = get_rpm_from_tcnt(&SweepSteps[sweep_stage].beginning_ocr, &SweepSteps[sweep_stage].prescaler_bits);
//  PORTD = (0 << 7);
}

/* Pumps the pattern out of flash to the port 
 * The rate at which this runs is dependent on what OCR1A is set to
 * the sweeper in timer2 alters this on the fly to alow changing of RPM
 * in a very nice way
 */
ISR(TIMER1_COMPA_vect) {
  /* This is VERY simple, just walk the array and wrap when we hit the limit */
  uint8_t aux;
  //PORTB = output_invert_mask ^ pgm_read_byte(&Wheels[selected_wheel].edge_states_ptr[edge_counter]);   /* Write it to the port */
   aux = output_invert_mask ^ pgm_read_byte(&Wheels[selected_wheel].edge_states_ptr[edge_counter]);  

  switch(aux)		
  {
		case 0:
			 digitalWrite(pri_out, LOW);
			 digitalWrite(pri_outN, HIGH);
			 digitalWrite(sec_out, LOW);
			 digitalWrite(sec_outN, HIGH);
			 break;
		case 1:
			 digitalWrite(pri_out, HIGH);
			 digitalWrite(pri_outN, LOW);
			 digitalWrite(sec_out, LOW);
			 digitalWrite(sec_outN, HIGH);
			 break;		 
		case 2:
			 digitalWrite(pri_out, LOW);
			 digitalWrite(pri_outN, HIGH);
			 digitalWrite(sec_out, HIGH);
			 digitalWrite(sec_outN, LOW);
			 break;		 	 
		case 3:
			 digitalWrite(pri_out, HIGH);
			 digitalWrite(pri_outN, LOW);
			 digitalWrite(sec_out, HIGH);
			 digitalWrite(sec_outN, LOW);
			 
			 break;		 	 	 
		default: break;	 
  }
  
  /* Normal direction  overflow handling */
  if (normal)
  {
    edge_counter++;
    if (edge_counter == Wheels[selected_wheel].wheel_max_edges) {
      edge_counter = 0;
    }
  }
  else
  {
    if (edge_counter == 0)
      edge_counter = Wheels[selected_wheel].wheel_max_edges;
    edge_counter--;
  }
  /* The tables are in flash so we need pgm_read_byte() */

  /* Reset Prescaler only if flag is set */
  if (reset_prescaler)
  {
    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); /* Clear CS10, CS11 and CS12 */
    TCCR1B |= prescaler_bits;
    reset_prescaler = false;
  }
  /* Reset next compare value for RPM changes */
  OCR1A = new_OCR1A;  /* Apply new "RPM" from Timer2 ISR, i.e. speed up/down the virtual "wheel" */
}

void loop() 
{
  uint16_t tmp_rpm = 0;
  /* Just handle the Serial UI, everything else is in 
   * interrupt handlers or callbacks from SerialUI.
   */

 unsigned long currentMillis;
 char button;

  //if(Serial.available() > 0) { commandParser(); }

  if(mode == POT_RPM)
  {
    if (adc1_read_complete == true)
    {
      adc1_read_complete = false;
      tmp_rpm = adc1 << TMP_RPM_SHIFT;
      if (tmp_rpm > TMP_RPM_CAP) { tmp_rpm = TMP_RPM_CAP; }
      wanted_rpm = tmp_rpm;
      reset_new_OCR1A(tmp_rpm);
    }
  }
  
  // button read  ----------------------------------
    
  currentMillis = millis();	 
  
  if (currentMillis - previousMillis >= interval) 
  {	  previousMillis = currentMillis;
			
	 if (adc0_read_complete == true)
     {
		 adc0_read_complete = false;
		 
		 if(button = Button_Press(adc0))
		 {
			Proc_keys(button);
		 }			 
				 
	 }	 
	 if(mode == POT_RPM)  LCD_Update(1);
	 
	 if(serial_upd)
	 {
		 LCD_Update(0);
		 LCD_Update(1);
		 serial_upd = false;
	 }
  }
  
}

void serialEvent()
{
	
    commandParser();
	serial_upd = true;
	
}

void reset_new_OCR1A(uint32_t new_rpm)
{
  uint32_t tmp;
  uint8_t bitshift;
  uint8_t tmp_prescaler_bits;
  tmp = (uint32_t)(8000000.0/(Wheels[selected_wheel].rpm_scaler * (float)(new_rpm < 10 ? 10:new_rpm)));
/*  mySUI.print(F("new_OCR1a: "));
  mySUI.println(tmpl);
  */
  get_prescaler_bits(&tmp,&tmp_prescaler_bits,&bitshift);
  /*
  mySUI.print(F("new_OCR1a: "));
  mySUI.println(tmp2);
  */
  new_OCR1A = (uint16_t)(tmp >> bitshift); 
  prescaler_bits = tmp_prescaler_bits;
  reset_prescaler = true; 
}


uint8_t get_bitshift_from_prescaler(uint8_t *prescaler_bits)
{
  switch (*prescaler_bits)
  {
    case PRESCALE_1024:
    return 10;
    case PRESCALE_256:
    return 8;
    case PRESCALE_64:
    return 6;
    case PRESCALE_8:
    return 3;
    case PRESCALE_1:
    return 0;
  }
  return 0;
}


//! Gets RPM from the TCNT value
/*!
 * Gets the RPM value based on the passed TCNT and prescaler
 * \param tcnt pointer to Output Compare register value
 * \param prescaler_bits point to prescaler bits enum
 */
uint16_t get_rpm_from_tcnt(uint16_t *tcnt, uint8_t *prescaler_bits)
{
  bitshift = get_bitshift_from_prescaler(prescaler_bits);
  return (uint16_t)((float)(8000000 >> bitshift)/(Wheels[selected_wheel].rpm_scaler*(*tcnt)));
}


//! Gets prescaler enum and bitshift based on OC value
void get_prescaler_bits(uint32_t *potential_oc_value, uint8_t *prescaler, uint8_t *bitshift)
{
  if (*potential_oc_value >= 16777216)
  {
    *prescaler = PRESCALE_1024;
    *bitshift = 10;
  }
  else if (*potential_oc_value >= 4194304)
  {
    *prescaler = PRESCALE_256;
    *bitshift = 8;
  }
  else if (*potential_oc_value >= 524288)
  {
    *prescaler = PRESCALE_64;
    *bitshift = 6;
  }
  else if (*potential_oc_value >= 65536)
  {
    *prescaler = PRESCALE_8;
    *bitshift = 3;
  }
  else
  {
    *prescaler = PRESCALE_1;
    *bitshift = 0;
  }
}


//! Builds the SweepSteps[] structure
/*!
 * For sweeping we cannot just pick the TCNT value at the beginning and ending
 * and sweep linearily between them as it'll result in a VERY slow RPM change
 * at the low end and a VERY FAST change at the high end due to the inverse
 * relationship between RPM and TCNT. So we compromise and break up the RPM
 * range into octaves (doubles of RPM), and use a linear TCNT change between
 * those two points. It's not perfect, but computationally easy
 *
 * \param low_rpm_tcnt pointer to low rpm OC value, (not prescaled!)
 * \param high_rpm_tcnt pointer to low rpm OC value, (not prescaled!)
 * \param total_stages pointer to tell the number of structs to allocate
 * \returns pointer to array of structures for each sweep stage.
 */
sweep_step *build_sweep_steps(uint32_t *low_rpm_tcnt, uint32_t *high_rpm_tcnt, uint8_t *total_stages)
{
  sweep_step *steps;
  uint8_t prescaler_bits;
  uint8_t bitshift;
  uint32_t tmp = *low_rpm_tcnt;
  /* DEBUG
  mySUI.print(*low_rpm_tcnt);
  mySUI.print(F("<->"));
  mySUI.println(*high_rpm_tcnt);
   */

  steps = (sweep_step *)malloc(sizeof(sweep_step)*(*total_stages));

#ifdef MORE_LINEAR_SWEEP
  for (uint8_t i = 0; i < (*total_stages); i+=2)
#else
  for (uint8_t i = 0; i < (*total_stages); i++)
#endif
  {
    /* The low rpm value will ALWAYS have the highed TCNT value so use that
    to determine the prescaler value
    */
    get_prescaler_bits(&tmp, &steps[i].prescaler_bits, &bitshift);
    
    steps[i].beginning_ocr = (uint16_t)(tmp >> bitshift);
    if ((tmp >> 1) < (*high_rpm_tcnt))
      steps[i].ending_ocr = (uint16_t)((*high_rpm_tcnt) >> bitshift);
    else
      steps[i].ending_ocr = (uint16_t)(tmp >> (bitshift + 1)); // Half the begin value
    tmp = tmp >> 1; /* Divide by 2 */
    /* DEBUG
    mySUI.print(steps[i].beginning_ocr);
    mySUI.print(F("<->"));
    mySUI.println(steps[i].ending_ocr);
    */
  }
  return steps;
}

//--------------------------------------------------
void LCD_Update(bool line)
{
	char buf[16];	
	
	if(!line) //update line 0
	{
		strncpy_P(buf,Wheels[selected_wheel].decoder_name,16);
		lcd.setCursor(0,0);
		lcd.print("                ");		
		lcd.setCursor(0,0);
		lcd.print(buf);		
	}
	
	else //update line 1
	{
		switch(mode)	
		{
			case LINEAR_SWEPT_RPM:
				sprintf(buf,"SWP %5d  %5d",sweep_low_rpm,sweep_high_rpm);	
				break;
			case FIXED_RPM:
				sprintf(buf,"FIX  %5d RPM  ",wanted_rpm);	
				break;
			case POT_RPM:
				sprintf(buf,"POT  %5d RPM  ",wanted_rpm);		
				break;
		}	

		lcd.setCursor(0,1);
		lcd.print(buf);
	}
}

//void Proc_keys(uint16_t adc_but)
void Proc_keys(char but)
{
//char but;
	
	//but = Button_Press(adc_but);	
    
	if(debouce)
    {		
	  switch(but)
	  {	
           case 'L': //left button
		      if(mode == FIXED_RPM)
			  {
				  if(wanted_rpm>100)
				  {
					  wanted_rpm -= 100;   
					  setRPM(wanted_rpm);
					  LCD_Update(1);				  
				  }
			  }
		      break;
			  
			case 'R': //right button
		      if(mode == FIXED_RPM)
			  {
				  if(wanted_rpm<16000)
				  {
					  wanted_rpm += 100;   
					  setRPM(wanted_rpm);
					  LCD_Update(1);				  
				  }
			  }
		      break; 
	  
		   case 'D': //down button
				edge_counter = 0;
			 if (selected_wheel == 0)
				selected_wheel = MAX_WHEELS-1;
			else 
				selected_wheel--;
						
			display_new_wheel();
			debouce = false;	
			LCD_Update(0);
		    break;
						  
		  case 'U': //up button
				edge_counter = 0;
				if (selected_wheel == (MAX_WHEELS-1))
					selected_wheel = 0;
				else 
					selected_wheel++;
				display_new_wheel();
			    debouce = false;
				LCD_Update(0);
			    break;  
						  
		case 'S': //select button
			  mode++;
			  if(mode == MAX_MODES) 
				  mode = 0;
			  if(mode == FIXED_RPM)
			  {
				  wanted_rpm = 2500;   
				  setRPM(wanted_rpm);
			  } 
		
			  if(mode == LINEAR_SWEPT_RPM)
			  {				  
				 sweep_low_rpm = 100;
				 sweep_high_rpm = 4000;
				 compute_sweep_stages(&sweep_low_rpm, &sweep_high_rpm);		  
			  }
			  debouce = false;
			  LCD_Update(1);			  
			  break;  						  
			
      }
		  
	}	
		
}

char Button_Press(uint16_t x)
{
	if(x>750) {debouce = true; return 0;}
	if(x>500) return 'S';
	if(x>300) return 'L';
	if(x>150) return 'D';
	if(x>50)  return 'U';
	if(x<50)  return 'R';
}