/*----------------------------------------------------------------------------*/
// File:     launchpad_jtag_finder.ino
// Date:     9/10/2014
// Author:   D.Meenahan
// Descr:    This file contains code to turn the TI MSP-EXP430G2 LaunchPad
//           development board (v1.5) into a jtag and swd finder.
//
//           Upon reset (or power up), the LaunchPad will prompt the user
//           to enable verbose mode or not, then perform two JTAG finder
//           algorithms. The first one uses the JTAG ID method and
//           the second one uses the Bypass method. Then the SWD finder algorithm
//           is performed.
//
//           The user can connect up to 14 suspected jtag or swd
//           signals to the LaunchPad and then press the reset button S1 to
//           perform the jtag and swd finding functions.
//
//           For this design, a terminal must be connected via VCP to see the
//           results. When using a VCP, the settings must be (9600,8,n,1).
//           For faster baud rates cannot use VCP, must connect terminal
//           cable directly to P1.2 (tx) and P1.1 (rx).
//           The hw uart in the MSP430G2553 is used in this design, not the sw uart.
//
//           Suspected jtag/swd signals can be connected to all 8 pins of P1
//           and P2, except for P1.2 and P1.2, which are dedicated for the hw uart.
//           That leaves 14 pins to connect to suspected jtag or swd signals
//           of the target device.
//
//           For this design, the two J5 shunts for the LEDs must be removed since
//           the corresponding I/O signals are used for possible jtag/swd signals.
//
//           The J3 shunts of the LaunchPad must be configured according to TI
//           documentation. In summary, when developing code and using the VCP,
//           all 5 shunts should be installed, with the TXD and RXD shunts
//           rotated 90 degrees from the other 3 shunts (VCC, TEST, RST).
//           This is required when using the hw uart.
//
//           Caution must be used to make sure the voltage signaling levels
//           of the LaunchPad and the target device are compatible.
//           The LaunchPad can operate between 1.8V and 3.6V, but when using
//           the VCP it operates at 3.6V only and the target device must also.
//
//           When the target device operates at 2.5V, for example, all 5 shunts
//           of J3 must be removed and an external 2.5V source must be connected
//           at the VCC pin of J6 to power the MSP430G2553. In this case,
//           the VCP is not used and a terminal cable must be connected to P1.2
//           (tx) for communication. Connection to P1.1 (rx) is optional
//           since the terminal never sends data to the LaunchPad.
//
//           Refer to TI documentation for further information.
//
// Notes:
/*----------------------------------------------------------------------------*/

// See pins_energia.h for chip definitions.
//#define TXD  BIT2   // P1.2        // These aren't necessary when using
//#define RXD  BIT1   // P1.1        // Energia functions.

// Total number of I/O pins of the 20-pin MSP430G2553 is 16, but 2 are used for
// the hw uart. So that leaves 14 pins that can be connected to the target.
// The code performs several loops of 0 thru 15, but it always skips the 2
// uart pins where the loop index is 1 or 2.
#define PINS 16

// Functions in this file.
void set_all_pins_to_inputs();
void set_pin_to_input(unsigned char pin);
void set_pin_to_output(unsigned char pin);
void set_pin(unsigned char pin);
void clr_pin(unsigned char pin);
unsigned char read_pin(unsigned char pin);
void send_fixed_swd_pattern(unsigned char val, unsigned int num);
void send_16bit_swd_pattern(unsigned int pattern);
int count_ones(unsigned long val);
unsigned char jtag_finder_id_method();
unsigned char jtag_finder_bypass_method();
unsigned char swd_finder();
void setup();
void loop();

// Global variables.
unsigned int n, m;
unsigned char tck, tms, tdo, tdi;    // jtag pins
unsigned char swclk, swdio;          // swd pins
unsigned int j;
unsigned char byte_val, byte_prev;
unsigned char toggle_count;
unsigned char tc;
unsigned long idval;
int cnt1;
int incomingByte = 0;	             // for incoming serial data
unsigned char vflag;                 // Verbose flag for demonstration purposes.

/*----------------------------------------------------------------------------*/
// Function: set_all_pins_to_inputs
// Descr:    This function configures all the I/O pins as inputs, with pullups
//           enabled and the pin set to 1. The pin needs to be high so that
//           tms is high and it resets the jtag state machine instead of
//           causing unknown advancing through the jtag state machine.//
// Globals:  none
// Params:   none
// Returns:  n/a
/*----------------------------------------------------------------------------*/
 void set_all_pins_to_inputs()
{
  // For P1, ignore the two uart pins.
  P1DIR &= ~0xF9;    // dir=0
  P1REN |=  0xF9;    // ren=1
  P1OUT &=  0xF9;    // bit=1

  P2DIR  =  0x00;    // dir=0
  P2REN  =  0xFF;    // ren=1
  P2OUT  =  0xFF;    // bit=1
}


/*----------------------------------------------------------------------------*/
// Function: set_pin_to_input
// Descr:    This function configures a specific I/O pin as an input, with the
//           pullup enabled and the pin set to 0.
// Params:   In: pin
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void set_pin_to_input(unsigned char pin)
{
  // Configure a specific pin as an input.
  if(pin==0)
  {
    P1DIR &= ~BIT0;    // dir=0  Set to input.
    P1REN |=  BIT0;    // ren=1  Enable the pullup.
    P1OUT &= ~BIT0;    // bit=0  Clear the pin.
  }/*
  else if (pin==1)     // Don't use the uart rx pin.
  {
    P1DIR &= ~BIT1;
    P1REN |=  BIT1;
    P1OUT &= ~BIT1;
  }
  else if (pin==2)     // Don't use the uart tx pin.
  {
    P1DIR &= ~BIT2;
    P1REN |=  BIT2;
    P1OUT &= ~BIT2;
  }*/
  else if (pin==3)
  {
    P1DIR &= ~BIT3;
    P1REN |=  BIT3;
    P1OUT &= ~BIT3;
  }
  else if (pin==4)
  {
    P1DIR &= ~BIT4;
    P1REN |=  BIT4;
    P1OUT &= ~BIT4;
  }
  else if (pin==5)
  {
    P1DIR &= ~BIT5;
    P1REN |=  BIT5;
    P1OUT &= ~BIT5;
  }
  else if (pin==6)
  {
    P1DIR &= ~BIT6;
    P1REN |=  BIT6;
    P1OUT &= ~BIT6;
  }
  else if (pin==7)
  {
    P1DIR &= ~BIT7;
    P1REN |=  BIT7;
    P1OUT &= ~BIT7;
  }
  else if (pin==8)
  {
    P2DIR &= ~BIT0;
    P2REN |=  BIT0;
    P2OUT &= ~BIT0;
  }
  else if (pin==9)
  {
    P2DIR &= ~BIT1;
    P2REN |=  BIT1;
    P2OUT &= ~BIT1;
  }
  else if (pin==10)
  {
    P2DIR &= ~BIT2;
    P2REN |=  BIT2;
    P2OUT &= ~BIT2;
  }
  else if (pin==11)
  {
    P2DIR &= ~BIT3;
    P2REN |=  BIT3;
    P2OUT &= ~BIT3;
  }
  else if (pin==12)
  {
    P2DIR &= ~BIT4;
    P2REN |=  BIT4;
    P2OUT &= ~BIT4;
  }
  else if (pin==13)
  {
    P2DIR &= ~BIT5;
    P2REN |=  BIT5;
    P2OUT &= ~BIT5;
  }
  else if (pin==14)
  {
    P2DIR &= ~BIT6;
    P2REN |=  BIT6;
    P2OUT &= ~BIT6;
  }
  else if (pin==15)
  {
    P2DIR &= ~BIT7;
    P2REN |=  BIT7;
    P2OUT &= ~BIT7;
  }
}


/*----------------------------------------------------------------------------*/
// Function: set_pin_to_output
// Descr:    This function configures a specific I/O pin as an output, with the
//           pullup disabled and the pin set to 0.
// Globals:  none
// Params:   In: pin
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void set_pin_to_output(unsigned char pin)
{
  // Configure a specific pin as an output.
  if(pin==0)
  {
    P1DIR |=  BIT0;    // dir=1  Set to output.
    P1REN &= ~BIT0;    // ren=0  Disable the pullup.
    P1OUT &= ~BIT0;    // bit=0  Clear the pin.
  }/*
  else if (pin==1)     // Don't use the uart rx pin.
  {
    P1DIR |=  BIT1;
    P1REN &= ~BIT1;
    P1OUT &= ~BIT1;
  }
  else if (pin==2)     // Don't use the uart tx pin.
  {
    P1DIR |=  BIT2;
    P1REN &= ~BIT2;
    P1OUT &= ~BIT2;
  }*/
  else if (pin==3)
  {
    P1DIR |=  BIT3;
    P1REN &= ~BIT3;
    P1OUT &= ~BIT3;
  }
  else if (pin==4)
  {
    P1DIR |=  BIT4;
    P1REN &= ~BIT4;
    P1OUT &= ~BIT4;
  }
  else if (pin==5)
  {
    P1DIR |=  BIT5;
    P1REN &= ~BIT5;
    P1OUT &= ~BIT5;
  }
  else if (pin==6)
  {
    P1DIR |=  BIT6;
    P1REN &= ~BIT6;
    P1OUT &= ~BIT6;
  }
  else if (pin==7)
  {
    P1DIR |=  BIT7;
    P1REN &= ~BIT7;
    P1OUT &= ~BIT7;
  }
  else if (pin==8)
  {
    P2DIR |=  BIT0;
    P2REN &= ~BIT0;
    P2OUT &= ~BIT0;
  }
  else if (pin==9)
  {
    P2DIR |=  BIT1;
    P2REN &= ~BIT1;
    P2OUT &= ~BIT1;
  }
  else if (pin==10)
  {
    P2DIR |=  BIT2;
    P2REN &= ~BIT2;
    P2OUT &= ~BIT2;
  }
  else if (pin==11)
  {
    P2DIR |=  BIT3;
    P2REN &= ~BIT3;
    P2OUT &= ~BIT3;
  }
  else if (pin==12)
  {
    P2DIR |=  BIT4;
    P2REN &= ~BIT4;
    P2OUT &= ~BIT4;
  }
  else if (pin==13)
  {
    P2DIR |=  BIT5;
    P2REN &= ~BIT5;
    P2OUT &= ~BIT5;
  }
  else if (pin==14)
  {
    P2DIR |=  BIT6;
    P2REN &= ~BIT6;
    P2OUT &= ~BIT6;
  }
  else if (pin==15)
  {
    P2DIR |=  BIT7;
    P2REN &= ~BIT7;
    P2OUT &= ~BIT7;
  }
}


/*----------------------------------------------------------------------------*/
// Function: set_pin
// Descr:    This function sets the specified pin to 1.
// Globals:  none
// Params:   In: pin
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void set_pin(unsigned char pin)
{
  // Set a specific pin to 1.
  if      (pin== 0) P1OUT |= BIT0;
//else if (pin== 1) P1OUT |= BIT1;    // Don't use the uart rx pin.
//else if (pin== 2) P1OUT |= BIT2;    // Don't use the uart tx pin.
  else if (pin== 3) P1OUT |= BIT3;
  else if (pin== 4) P1OUT |= BIT4;
  else if (pin== 5) P1OUT |= BIT5;
  else if (pin== 6) P1OUT |= BIT6;
  else if (pin== 7) P1OUT |= BIT7;
  else if (pin== 8) P2OUT |= BIT0;
  else if (pin== 9) P2OUT |= BIT1;
  else if (pin==10) P2OUT |= BIT2;
  else if (pin==11) P2OUT |= BIT3;
  else if (pin==12) P2OUT |= BIT4;
  else if (pin==13) P2OUT |= BIT5;
  else if (pin==14) P2OUT |= BIT6;
  else if (pin==15) P2OUT |= BIT7;
}


/*----------------------------------------------------------------------------*/
// Function: clr_pin
// Descr:    This function sets the specified pin to 0.
// Globals:  none
// Params:   In: pin
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void clr_pin(unsigned char pin)
{
  // Set a specific pin to 0.
  if      (pin== 0) P1OUT &= ~BIT0;
//else if (pin== 1) P1OUT &= ~BIT1;    // Don't use the uart rx pin.
//else if (pin== 2) P1OUT &= ~BIT2;    // Don't use the uart tx pin.
  else if (pin== 3) P1OUT &= ~BIT3;
  else if (pin== 4) P1OUT &= ~BIT4;
  else if (pin== 5) P1OUT &= ~BIT5;
  else if (pin== 6) P1OUT &= ~BIT6;
  else if (pin== 7) P1OUT &= ~BIT7;
  else if (pin== 8) P2OUT &= ~BIT0;
  else if (pin== 9) P2OUT &= ~BIT1;
  else if (pin==10) P2OUT &= ~BIT2;
  else if (pin==11) P2OUT &= ~BIT3;
  else if (pin==12) P2OUT &= ~BIT4;
  else if (pin==13) P2OUT &= ~BIT5;
  else if (pin==14) P2OUT &= ~BIT6;
  else if (pin==15) P2OUT &= ~BIT7;
}


/*----------------------------------------------------------------------------*/
// Function: read_pin
// Descr:    This function reads the specified pin. If the pin is a 1, then
//           the corresponding bit in the return byte will be a 1. If the bit
//           is a 0, then 0x00 is returned.
// Globals:  none
// Params:   In: pin
// Returns:  byte port value
/*----------------------------------------------------------------------------*/
unsigned char read_pin(unsigned char pin)
{
  unsigned char byte = 0;

  // Read a specific pin and return its value.
  if      (pin== 0) byte = (unsigned char)P1IN & BIT0;
//else if (pin== 1) byte = (unsigned char)P1IN & BIT1;   // Don't use the uart rx pin.
//else if (pin== 2) byte = (unsigned char)P1IN & BIT2;   // Don't use the uart tx pin.
  else if (pin== 3) byte = (unsigned char)P1IN & BIT3;
  else if (pin== 4) byte = (unsigned char)P1IN & BIT4;
  else if (pin== 5) byte = (unsigned char)P1IN & BIT5;
  else if (pin== 6) byte = (unsigned char)P1IN & BIT6;
  else if (pin== 7) byte = (unsigned char)P1IN & BIT7;
  else if (pin== 8) byte = (unsigned char)P2IN & BIT0;
  else if (pin== 9) byte = (unsigned char)P2IN & BIT1;
  else if (pin==10) byte = (unsigned char)P2IN & BIT2;
  else if (pin==11) byte = (unsigned char)P2IN & BIT3;
  else if (pin==12) byte = (unsigned char)P2IN & BIT4;
  else if (pin==13) byte = (unsigned char)P2IN & BIT5;
  else if (pin==14) byte = (unsigned char)P2IN & BIT6;
  else if (pin==15) byte = (unsigned char)P2IN & BIT7;
  return byte;
}


/*----------------------------------------------------------------------------*/
// Function: send_fixed_swd_pattern
// Descr:    This function clocks out a swd pattern of all 1's or all 0's for
//           the specified number of bits.
// Globals:  The swdio pin and the swclk pin.
// Params:   In: pin
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void send_fixed_swd_pattern(unsigned char val, unsigned int num)
{
  unsigned int j;

  // Set the swdio pin to 1 or 0 , depending on val, prior to clocking.
  if(val==1)
    set_pin(swdio);
  else
    clr_pin(swdio);

  // Now clock swclk the specified number of times.
  for(j=0;j<num;j++)
  {
    clr_pin(swclk);
    set_pin(swclk);
  }
  clr_pin(swclk);
}


/*----------------------------------------------------------------------------*/
// Function: send_16bit_swd_pattern
// Descr:    This function clocks out a specified 16-bit swd pattern.
// Globals:  The swdio pin and the swclk pin.
// Params:   In: pattern
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void send_16bit_swd_pattern(unsigned int pattern)
{
  int j;

  for(j=0;j<16;j++)
  {
    clr_pin(swclk);
    if(pattern & 0x8000)         // Set swdio pin to 1 or 0 based on msb.
      set_pin(swdio);
    else
      clr_pin(swdio);

    set_pin(swclk);              // Clock out one of the 16 bits.
    pattern = pattern<<1;        // Shift the pattern for next time thru loop.
  }
  clr_pin(swclk);
}


/*----------------------------------------------------------------------------*/
// Function: count_ones
// Descr:    This function counts the number of 1's in a 32-bit variable.
// Globals:  none
// Params:   In: the 32-bit variable
// Returns:  cnt - the number of bits that are 1's.
/*----------------------------------------------------------------------------*/
int count_ones(unsigned long val)
{
  int j;
  int cnt = 0;

  for(j=0;j<32;j++)
  {
    if(val & 0x1) cnt++;         // Test the lsb, if 1, increment counter.
    val = val>>1;                // Shift the value to the right.
  }
  return cnt;
}


/*----------------------------------------------------------------------------*/
// Function: jtag_finder_id_method
// Descr:    This function attempts to identify the 4 jtag signals of a target
//           device with the assumption that resetting the jtag state machine
//           results in the jtag id being placed in the DR register. The DR
//           register is then clocked out, read and analyzed to determine if a
//           valid jtag id was just obtained.
//           The tdi signal is not necessary to obtain the jtag id, only tck,
//           tms and tdo are required. After identifying tck, tms and tdo,
//           this function then attempts to identify tdi by flushing known data
//           thru the tdi/tdo path.
// Globals:  tck, tms, tdo and tdi global variables corresponding to the pins.
// Params:   none
// Returns:  1 if successful, 0 if not.
/*----------------------------------------------------------------------------*/
unsigned char jtag_finder_id_method()
{
  unsigned char retval = 0;

  for(tck=0;tck<PINS;tck++)
  {
    for(tms=0;tms<PINS;tms++)
    {
      // The 'continue' instruction is used in the 'for' loops to skip a
      // permutation where the same pin is used for a different signal.
      // Also to skip when uart pins are specified (1=p1.1=rx, 2=p1.2=tx).
      if(tck == tms) continue;
      for(tdo=0;tdo<PINS;tdo++)
      {
        if(tck == tdo || tms == tdo ) continue;
        if(tck == 1   || tms == 1 || tdo == 1) continue;
        if(tck == 2   || tms == 2 || tdo == 2) continue;

        if(vflag)
        {
          Serial.print("  ");
          Serial.print("  tck=");Serial.print(tck,DEC);
          Serial.print("  tms=");Serial.print(tms,DEC);
          Serial.print("  tdo=");Serial.print(tdo,DEC);
        //Serial.print("  tdi=");Serial.print(tdi,DEC);
          Serial.print("\r\n");
        }

        // Initialize all the pins for this permutation.
        // Only two pins (tck and tms) are outputs, all others are inputs
        // since you are Iooking for tdo to come from the target.
        set_all_pins_to_inputs();
        set_pin_to_output(tck);
        set_pin_to_output(tms);

        // Initialize some variables before starting the permutation.
        byte_prev    = 0x00;
        toggle_count = 0x00;
        idval        = 0x00000000;

        // Perform a loop where tck is clocked a total of 40 times to attempt
        // to read the DR (data register) containing the jtag id.
        // For the first 5 clocks, set tms to 1 to get the jtag state machine
        // back to the reset position. The current state of the jtag state
        // machine is unknown, but five 1's on tms will get it back to the
        // reset state, no matter what state it was in initially.
        // Then advance the state machine to just before the Shift-DR
        // state by clocking in 010 on tms. Typically the jtag id is placed
        // in the DR at the reset state, therefore the contents of the DR
        // can be shifted out so that it appears on the tdo pin for reading.
        // Therefore, clock 32 more times with tms equal to 0 to remain in
        // the Shift-DR state and shift out the jtag id value on tdo.
        // The tdo pin is then read after each shift forming the jtag id.
        // To summarize, here is the 40-bit sequence clocked in on tms:
        // |first bit                     last bit|
        // 1111101000000000000000000000000000000000
        set_pin(tms);
        // Perform loop to attempt to get activity on tdo.
        // This activity most likely would be the first jtag id being shifted out,
        // There may be additional jtag id's, but only the first one is shifted
        // out by the last 32 clocks of the total of 40.
        for(j=0;j<40;j++)
        {
          set_pin(tck);
          clr_pin(tck);
          if (j==4)
            clr_pin(tms);
          else if (j==5)
            set_pin(tms);
          else if (j==6)
            clr_pin(tms);
          // Have just done JTAG reset and moved to the Sh_DR state.
          // Now shift 32 times to hopefully capture the JTAG ID on tdo.
          if(j > 7)
          {
            byte_val = read_pin(tdo);
            if(byte_val != byte_prev) toggle_count++;
            byte_prev = byte_val;
            idval  = idval >> 1;
            if(byte_val == 0x00)
              idval &= ~0x80000000;
            else
              idval |= 0x80000000;
          }
        }                 // for loop of 40

        // Test if there was some activity (toggles) observed on the suspected
        // tdo pin, if so try to find tdi pin.
        if(toggle_count > 7)
        {
          // Lets try to find tdi among the 11 remaining pins.
          for(tdi=0;tdi<PINS;tdi++)
          {
            // Permutations to skip.
            if(tdi == tck || tdi == tms || tdi == tdo || tdi == 1 || tdi == 2) continue;

            if(vflag)
            {
              Serial.print("  ");
              Serial.print("  tck=");Serial.print(tck,DEC);
              Serial.print("  tms=");Serial.print(tms,DEC);
              Serial.print("  tdo=");Serial.print(tdo,DEC);
              Serial.print("  tdi=");Serial.print(tdi,DEC);
              Serial.print("\r\n");
            }

            byte_prev = 0x00;
            tc        = 0x00;

            // Flush out all the DR registers of all the devices in the jtag chain.
            // 512 clocks should be more than enough.
            set_pin_to_output(tdi);
            for(j=0;j<512;j++)
            {
              set_pin(tck);
              clr_pin(tck);
            }

            // Now do another loop of 512 clocks.
            // First, feed in 32 alternating bits into the suspected tdi pin of
            // the target. Follow this with 480 non-alternating bits (either all
            // l's or all 0's).
            // Depending on the jtag chain length, should eventually see about
            // 32 (±2) toggles appearing on the tdo from the target over the entire
            // 512 clocks.
            // If tdo toggles 32 (±2) times just like tdi did, then tdi has been found.
            for(j=0;j<512;j++)
            {
              set_pin(tck);
              clr_pin(tck);
              // Inject 32 alternating 1's and 0's into jtag chain.
              if( j < 32 && j % 2 )
                set_pin(tdi);
              else
                clr_pin(tdi);

              byte_val = read_pin(tdo);
              // Count how many times tdo toggled.
              if(byte_val != byte_prev) tc++;
              byte_prev = byte_val;
            }

            // Test if 32±2 toggles on tdo were counted.
            // If so, print the result and set return value.
            if(tc > 29 && tc < 35)
            {
              Serial.print("  ");
              Serial.print("  tck=");Serial.print(tck,DEC);
              Serial.print("  tms=");Serial.print(tms,DEC);
              Serial.print("  tdo=");Serial.print(tdo,DEC);
              Serial.print("  tdi=");Serial.print(tdi,DEC);
              Serial.print("  jtag id=");Serial.print(idval,HEX);
              Serial.print("\r\n");
              retval = 1;
            }
          }                    // tdi loop.
        }                      // if tc ...
      }                        // tdo loop.
    }                          // tms loop.
  }                            // tck loop.
  return retval;
}                              // End of jtag_finder_id_method().


/*----------------------------------------------------------------------------*/
// Function: jtag_finder_bypass_method
// Descr:    This function attempts to identify the 4 jtag signals of a target
//           device with the assumption that resetting the jtag state machine
//           selects the jtag bypass mode. The bypass register is then clocked,
//           out, read and analyzed to determine if a there is a one clock delay
//           in the tdi/tdo path.
// Globals:  tck, tms, tdo and tdi global variables corresponding to the pins.
// Params:   none
// Returns:  1 if successful, 0 if not.
/*----------------------------------------------------------------------------*/
unsigned char jtag_finder_bypass_method()
{
  unsigned char retval = 0;

  for(tck=0;tck<PINS;tck++)
  {
    for(tms=0;tms<PINS;tms++)
    {
      // The 'continue' instruction is used in the 'for' loops to skip a
      // permutation where the same pin is used for a different signal.
      // Also to skip when uart pins are specified (1=p1.1=rx, 2=p1.2=tx).
      if(tck == tms) continue;
      for(tdo=0;tdo<PINS;tdo++)
      {
        if(tck == tdo || tms == tdo ) continue;
        if(tck == 1   || tms == 1 || tdo == 1) continue;
        if(tck == 2   || tms == 2 || tdo == 2) continue;

        for(tdi=0;tdi<PINS;tdi++)
        {
          if(tdi == tck || tdi == tms || tdi == tdo || tdi == 1 || tdi == 2) continue;

          if(vflag)
          {
            Serial.print("  ");
            Serial.print("  tck=");Serial.print(tck,DEC);
            Serial.print("  tms=");Serial.print(tms,DEC);
            Serial.print("  tdo=");Serial.print(tdo,DEC);
            Serial.print("  tdi=");Serial.print(tdi,DEC);
            Serial.print("\r\n");
          }

          // Initialize all the pins for this permutation.
          // Three pins (tck, tms, tdi) are outputs, all others are inputs
          // since you are looking for tdo to come from the target.
          set_all_pins_to_inputs();
          set_pin_to_output(tck);
          set_pin_to_output(tms);
          set_pin_to_output(tdi);

          // Initialize some variables before starting the permutation.
          byte_prev    = 0x00;
          toggle_count = 0x00;
          idval        = 0x00000000;

          // Now put the jtag state machine in the Shift-DR state.
          // For the first 5 clocks, set tms to 1 to get the jtag state machine
          // back to the reset position. Then advance the state machine to just before
          // the Shift-DR state by clocking in 0100 on tms.
          set_pin(tms);
          clr_pin(tdi);

          // Perform loop to attempt to get activity on tdo that matches the
          // pattern that was input on tdi.
          for(j=0;j<9;j++)
          {
            if      (j==5) clr_pin(tms);
            else if (j==6) set_pin(tms);
            else if (j==7) clr_pin(tms);
            set_pin(tck);
            clr_pin(tck);
          }
          // Have just done JTAG reset and moved to the Sh_DR state.
          // Now shift 32 times to hopefully capture the JTAG ID on tdo.

          // To avoid false results, flush out whatever register is in the
          // tdi/tdo path after a jtag reset. If the target is in bypass after
          // the jtag reset, then the  1-bit bypass register is flushed.
          // If the target does not go into bypass after the jtag reset, the
          // DR register is probably in the tdi/tdo path containing the jtag id.
          // So flush out the jtag id so that this bypass method will not
          // report false results.
          for(j=0;j<64;j++)
          {
            // Cycle the clock.
            set_pin(tck);
            clr_pin(tck);
          }

          // Now clock in a 1 every 4 bits on tdi to hopefully see on tdo.
          clr_pin(tms);
          for(j=0;j<33;j++)
          {
            // Clock a known pattern into the jtag chain.
            if(j % 4 == 0)
              set_pin(tdi);
            else
              clr_pin(tdi);

            // Cycle the clock.
            set_pin(tck);
            clr_pin(tck);

            // Count how many times tdo toggled.
            byte_val = read_pin(tdo);
            if(byte_val != byte_prev) toggle_count++;

            // Capture the 32 bits coming from the target device.
            byte_prev = byte_val;
            idval  = idval >> 1;     // Right shift by one.
            if(byte_val == 0x00)
              idval &= ~0x80000000;  // Received a 0 on the tdo pin.
            else
              idval |= 0x80000000;   // Received a 1 on the tdo pin.
          }

          // Count the number of ones in the idval.
          cnt1 = count_ones(idval);

          // Test if 16±2 toggles on tdo were counted.
          // If so, print the result.
          if(toggle_count > 13 && toggle_count < 19)
          {
            Serial.print("  ");
            Serial.print("  tck=");Serial.print(tck,DEC);
            Serial.print("  tms=");Serial.print(tms,DEC);
            Serial.print("  tdo=");Serial.print(tdo,DEC);
            Serial.print("  tdi=");Serial.print(tdi,DEC);
            Serial.print("  bypass_data=");Serial.print(idval,HEX);
            Serial.print("\r\n");
            set_pin(tms);
            retval = 1;
          }
        }         // tdi loop
      }           // tdo loop
    }             // tms loop
  }               // tck loop
  return retval;
}                 // End of jtag_finder_bypass_method().


/*----------------------------------------------------------------------------*/
// Function: swd_finder
// Descr:    This function attempts to identify the 2 swd signals of the
//           of a target device by requesting and reading the swd id value.
// Globals:  swclk and swdio global variables corresponding to the pins.
// Params:   none
// Returns:  1 if successful, 0 if not.
/*----------------------------------------------------------------------------*/
unsigned char swd_finder()
{
  unsigned char retval = 0;

  for(swclk=0;swclk<PINS;swclk++)
  {
    for(swdio=0;swdio<PINS;swdio++)
    {
      // Skip permutations when the same pin is used for a different signal.
      // Also skip when the uart pins are specified (1=p1.1=rx, 2=p1.2=tx),
      if(swclk == swdio || swclk == 1 || swdio == 1 || swclk == 2 || swdio == 2) continue;

      if(vflag)
      {
        Serial.print("  ");
        Serial.print("  swclk=");Serial.print(swclk,DEC);
        Serial.print("  swdio=");Serial.print(swdio,DEC);
        Serial.print("\r\n");
      }

      // Initialize all the pins for this permutation.
      // Both SWD pins (SWCLK and SWDlO) are initially set as outputs, then
      // bidirectional SWDIO will be changed to an input to read the response
      // from the target.
      set_all_pins_to_inputs();
      set_pin_to_output(swclk);
      set_pin_to_output(swdio);


      // Initialize some variables before starting the permutation.
      byte_prev    = 0x00;
      toggle_count = 0x00;
      idval        = 0x00000000;

      // Send the bits observed from the Arium HS-1000S when it tries to connect to
      // a target via SWD (not JTAG).
      send_fixed_swd_pattern(0, 8);       // Eight 0's.
      send_fixed_swd_pattern(1, 50);      // Line reset.
      send_fixed_swd_pattern(0, 0x79e7);  // JTAG-to-SWD switching sequence of 0xe79e.
      send_fixed_swd_pattern(0, 8);       // Eight 0's.
      send_fixed_swd_pattern(1, 50);      // Line reset.
      send_fixed_swd_pattern(0, 0x00a5);  // Eight 0's then id request 0xa5.

      clr_pin(swclk);
      clr_pin(swdio);

      // Now need to change the swdio pin from output to input and attempt to
      // read the id from the target appearing on the swdio pin.
      // The id value read could be off a bit or two if the target has more than
      // one turn around bit. Either way, the exact id obtained is not all that
      // important, the detection of activity is what is important because
      // it indicates the finding of the swclk and swdio pins.
      set_pin_to_input(swdio);
      for(j=0;j<38;j++)
      {
        clr_pin(swclk);
        set_pin(swclk);

        if(j > 2 && j< 35)   // May need to be adjusted.
        {
          // Read each bit of the id and count the toggles.
          byte_val = read_pin(swdio);
          if(byte_val != byte_prev) toggle_count++;

          byte_prev = byte_val;
          idval  = idval >> 1;     // Right shift by 1.
          if(byte_val == 0x00)
            idval &= ~0x80000000;  // Received a 0 on swdio pin.
          else
            idval |= 0x80000000;   // Received a 1 on swdio pin.
        }
      }    // for 38

      clr_pin(swclk);
      clr_pin(swdio);

      // If swdio had activity, probably found the swdio signal.
      if(toggle_count > 7)
      {
        Serial.print("  ");
        Serial.print("  swclk=");Serial.print(swclk,DEC);
        Serial.print("  swdio=");Serial.print(swdio,DEC);
        Serial.print("  swd_id=");Serial.print(idval,HEX);
        Serial.print("\r\n");
        retval = 1;
      }
    }                // swdio loop
  }                  // swclk loop
  return retval;
}                    // End of swd_finder().



/*----------------------------------------------------------------------------*/
// Function: setup
// Descr:    This function is the first of two default Energia functions.
//           It runs once after a reset.
// Globals:  none
// Params:   none
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void setup()
{
  vflag = 0;

  Serial.begin(9600);              // Setup hw uart for 9600 baud.

  Serial.print("\r\n\n\n\n\n\n\n");
  Serial.print("----------------------------------------------------------------------\r\n");
  Serial.print("------------------- LaunchPad JTAG/SWD Finder v1.0 -------------------\r\n");
  Serial.print("----------------------------------------------------------------------\r\n");

  Serial.print("\r\n");
  Serial.print("The test results below use the numbers 0 thru 15 as follows to specify\r\n");
  Serial.print("any MSP430G2553 pins that are connected to JTAG or SWD signals:\r\n");
  Serial.print("  0:P1.0   1:n/a    2:n/a    3:Pl.3   4:P1.4   5:P1.5   6:P1.6   7:P1.7\r\n");
  Serial.print("  8:P2.0   9:P2.1  10:P2.2  11:P2.3  12:P2.4  13:P2.5  14:P2.6  15:P2.7\r\n");

  Serial.print("\r\n\n");


  Serial.print("\Press 'v' for verbose mode or any other key for non-verbose mode.");
  while (Serial.available() == 0);
  incomingByte = Serial.read();
  if (incomingByte == 'v' || incomingByte == 'V') vflag = 1;
  if (vflag == 1)
    Serial.print("\r\n  Verbose mode on.");
  else
    Serial.print("\r\n  Verbose mode off.");


  Serial.print("\r\n\n");
  Serial.print("Begin testing...\r\n\n");

  Serial.print("  Start of JTAG Finder using JTAG ID method.\r\n");
  if(!jtag_finder_id_method())
    Serial.print("    Did not find a JTAG interface using the JTAG ID method.\r\n");
  Serial.print("  End of JTAG Finder using JTAG ID method.\r\n\n");

  Serial.print("  Start of JTAG Finder using Bypass method.\r\n");
  if(!jtag_finder_bypass_method())
    Serial.print("    Did not find a JTAG interface using Bypass method.\r\n");
  Serial.print("  End of JTAG Finder using Bypass method.\r\n\n");

  Serial.print("  Start of SWD Finder.\r\n");
  if(!swd_finder())
    Serial.print("    Did not find a SWD interface.\r\n");
  Serial.print("  End of SWD Finder.\r\n\n");

  Serial.print("Done testing.\r\n");

}


/*----------------------------------------------------------------------------*/
// Function: loop
// Descr:    This function is the second of two default Energia functions.
//           It runs forever after setup() has completed.
// Globals:  none
// Params:   none
// Returns:  n/a
/*----------------------------------------------------------------------------*/
void loop()
{
  // This is here just to demonstrate that the LaunchPad can receive
  // keystrokes from the terminal after all the testing is done.
  // This is written in Energia style code.
  // Send data only when you receive data.
  if (Serial.available() > 0)
  {
    // Read the incoming byte.
    incomingByte = Serial.read();

    // Transmit what you got.
    Serial.print("I received: ");
    Serial.print(incomingByte, DEC);
    Serial.print("   ");
    Serial.println(incomingByte, HEX);
	}
}
