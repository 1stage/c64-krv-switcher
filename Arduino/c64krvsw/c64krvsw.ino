#include <EEPROM.h>

// C64 KRV (Kernal ROM Video) Switcher
// v1.1 by Sean Harrington
// Original design by Sven Petersen
// EXROM code from Adrian Black
//
// This sketch allows an Arduino Nano to switch a Commodore C64 computer's
// Kernel ROM and video mode through a combination of keyboard presses.
// For all functionality, there are three hardware components that make this happen:
//
//    1. The C64 KRV Keyboard Switcher board must be plugged into the C64 keyboard
//       pin header (CN1), and the C64 keyboard must be plugged into the KRV board.
//       Additionally, the KRV board must be connected to the C64's RESET line (CN2 
//       pin 3) and EXROM reset line (differs by motherboard), the N-EN and P-EN 
//       pins must attached to a VIC-II² video switcher board, and the A13-A14-A15 
//       pins must be attached to the KRV ROM switcher adapter.
//   *2. The KRV ROM switcher adapter board must be plugged into the Kernal ROM
//       slot on the C64 (U4), and an 27512 EPROM (DIP28) with eight working
//       replacement C64 ROM images must be burned onto it. The A13-A14-A15
//       pins must be attached to the C64 KRV Keyboard Switcher board.
//  **3. The VIC-II² Video Switching card must be plugged into the VIC-II
//       slot on the C64 (U19) and populated with working VIC-II chips 
//       (6567 and 6569) and replacement headers must replace crystal (Y1) and 
//       NTSC/PAL jumpers (depends on motherboard). The N-EN and P-EN pins must be
//       attached to the N-EN and P-EN pins on the C64 KRV Keyboard Switcher
//       board.
//
//  * The system does NOT need to have the KRV ROM switcher installed if ONLY video switching is desired.
// ** The system does NOT need to have the VIC-II² video switcher if ONLY ROM switching is desired.
//
// The system works by sensing when the C64 RESTORE key is pressed in combination with numbers 0-9:
//
//    RESTORE + 1 - Kernal image in bank 1 (0000-1FFF) of 27512 loads, and system resets (should be CBM Kernal v3)
//    RESTORE + 2 - Kernal image in bank 2 (2000-3FFF) of 27512 loads, and system resets
//    RESTORE + 3 - Kernal image in bank 3 (4000-5FFF) of 27512 loads, and system resets
//    RESTORE + 4 - Kernal image in bank 4 (6000-7FFF) of 27512 loads, and system resets
//    RESTORE + 5 - Kernal image in bank 5 (8000-9FFF) of 27512 loads, and system resets
//    RESTORE + 6 - Kernal image in bank 6 (A000-BFFF) of 27512 loads, and system resets
//    RESTORE + 7 - Kernal image in bank 7 (C000-DFFF) of 27512 loads, and system resets
//    RESTORE + 8 - Kernal image in bank 8 (E000-FFFF) of 27512 loads, and system resets
//    RESTORE + 9 - KRV keyboard tells the VIC-II² card to toggle modes, and system resets
//    RESTORE + 0 - Kernal image in bank 1 (0000-1FFF) of 27512 loads, and system resets
//                  but does not wait for a keyboard scan to happen (helps with cartridges)
//
// In modes 1-8, if no keyboard scan is detected, the system will automatically reset after 
// 10 seconds, and the system loads the image in bank 1 (0000-1FFF) of the 27512 EPROM. This helps 
// ensure the system will reset in the event a bad Kernal image is loaded. If this is undesireble,
// mode 0 is used to ignore the lack of keyboard scan.
//
// v1.1, 04 NOV 2019 - Rewrote number key processing routine as switch/case statement
//                     Reworked boolean variables to evaluate themselves rather than using == comparators
//                     Fixed main() loop key and RESTORE scan routines for sanity sake
// v1.0, 10 SEP 2019 - Switched naming of RSVD01 & RSVD02 to NeN and PeN
//                     Switched naming of EEPROMAddr to KernalSlot
//                     Updated NTSC/PAL EEPROM read & write routine
//                     Removed reference to VAddr and replaced with np_mode (duplicate variables)
// v0.4, 05 AUG 2019 - Revised NTSC/PAL switch to PULSE 5V (HIGH) rather than hold
// v0.3, 21 JUL 2019 - Added EXROM reset stuff
//                     Turned systemRESET and systemUNRESET into functions
//                     More error correction & formatting of comments
// v0.2, 14 JUL 2019 - Removed short/long board references (my solution only supports longboards)
//                     Error corrections & commenting
// v0.1, 05 JUL 2019 - Updated RSVD01 and RSVD02 to serve as NTSC/PAL switch toggles
//                     Created RESTORE + 9 function to serve as NTSC/PAL toggle key
//
// Note 1:  Transistor Q1 (2N3904 NPN) is used to flip the HIGH to a LOW for the C64_RESET.
//          It may seem counter-intuitive to set C64RES pin to HIGH to set the C64_RESET pin LOW,
//          but this is in fact what it does.

// *** Constants ***
const int Row0 = 2;                                               // Row signal, input, active low, INT0
const int Row3 = 3;                                               // Row signal, input, active low, INT1
const int KSW_A13 = 4;                                            // Output, A13 to Kernal adaptor board
const int KSW_A14 = 5;                                            // Output, A14 to Kernal adaptor board
const int KSW_A15 = 6;                                            // Output, A15 to Kernal adaptor board
const int NeN = 9;                                                // Output, NTSC Enable (HI), connected to N-EN on VIC-II² video switcher board 
const int PeN = 10;                                               // Output, PAL Enable (HI), connected to P-EN on VIC-II² video switcher board
const int C64RES = 7;                                             // Output, RESET IO, connected through transistor to RESET pin on C64 (active HIGH, see Note 1: at top)
const int EXROMRES = 8;                                           // Input, EXROM, connected through resistor to EXROM reset pin on C64, active low
const int Col1 = A0;                                              // Input, column signal, active low
const int Col2 = A1;                                              // Input, column signal, active low
const int Col3 = A2;                                              // Input, column signal, active low
const int Col4 = A3;                                              // Input, column signal, active low
const int Col7 = A4;                                              // Input, column signal, active low
const int RESTOREPin = A5;                                        // Input, RESTORE key, active low
const int KMax = 8;                                               // Holds the highest Kernal number
const int KernalSlot = 0                                          // Kernal selection address in EEPROM
const int ScanActiveSlot = 1                                      // Keyscan active address in EEPROM
const int NPModeSlot = 2                                          // NTSC/PAL mode address in EEPROM
const int Default_Scan_Count = 10000                              // Delay in milliseconds before reset for no scan activity
const int Default_Restore_Count = 5000;                           // Delay in milliseconds for RESTORE key hold before reset

 // *** Variables ***
byte kernal_num = 0;                                              // Kernal number
byte kernal_addr = 0;                                             // Address bits for Kernal, always kernal_num - 1
byte np_mode = 1;                                                 // Default NTSC/PAL mode, 1 is NTSC, 0 is PAL
bool restore_found = false;                                       // TRUE if the RESTORE key is pressed 
int  restore_counter = 0;                                         // Countdown for RESTORE key
byte key = 128;                                                   // Calculate the Key
byte columns;                                                     // Variable for reading out the column
int  scan_counter;                                                // Countdown for keyboard scanning
bool scan_found;                                                  // Flag for keyboard activity detected
byte scan_active = 1;                                             // Default keyboard scan active, 1 is on 

// *** Data Exchange variables for ISR (have to be volatile) ***
volatile bool numPressed = false;                                 // ISR flag for number pressed
volatile bool numOdd = false;                                     // ISR flag for odd numbers. Row0 => odd numbers, Row3 => even numbers
volatile byte colData = 0x1F;                                     // Column data, read from PORTD

void setup() {                                                    // Keyboard scan signals:
  pinMode( Row0, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( Row3, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( Col1, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( Col2, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( Col3, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( Col4, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( Col7, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( RESTOREPin, INPUT_PULLUP);                             // Pullup required for MOS logic
                                                                  // Control outputs (for Kernal Adaptor and C64):
  pinMode( KSW_A13, OUTPUT );                                     // Make Kernal switcher pin A13 an output   
  pinMode( KSW_A14, OUTPUT );                                     // Make Kernal switcher pin A14 an output
  pinMode( KSW_A15, OUTPUT );                                     // Make Kernal switcher pin A15 an output
  pinMode( NeN, OUTPUT );                                         // Make NTSC video switcher pin an output
  pinMode( PeN, OUTPUT );                                         // Make PAL video switcher pin an output
  pinMode( C64RES, OUTPUT );                                      // Make C64 RESET pin an output
  pinMode( EXROMRES, INPUT );                                     // Make EXROM RESET pin an input

                                                                  // Last Kernal read from EEPROM
  kernal_num = EEPROM.read(KernalSlot);                           // Read the Kernal number from EEPROM
  if ((kernal_num < 1) || (kernal_num > KMax)) {                  // If out of range,
    kernal_num = 1;                                               //   the first Kernal is selected.
    EEPROM.write(KernalSlot, kernal_num);
  }
  
  kernal_addr = kernal_num - 1;                                   // Calculate the address bits. It should be between 0 and 7,
  
  scan_active = EEPROM.read(ScanActiveSlot);
  if ((scan_active != 0) && (scan_active != 1)) {
    scan_active = 1;
    EEPROM.write(ScanActiveSlot, scan_active);
  }
                                                                  // Last NTSC/PAL mode read from EEPROM
  np_mode = EEPROM.read(NPModeSlot);                              // Read the Video Mode from EEPROM
  if ((np_mode != 0) && (np_mode != 1)) {                         // If out of range,
    np_mode = 1;                                                  //   NTSC mode is selected.
    EEPROM.write(NPModeSlot, np_mode);
  }
  
  systemRESET();                                                  // Set system to RESET mode
                                                                  // Switch the KERNAL and Video Mode
  SetAddressBits( kernal_addr );                                  // Set the MS address bits for the Kernal ROM
  SetVideoBits ( np_mode );                                       // Set the video address bits

  scan_counter = Default_Scan_Count;                              // Reset scan countdown
  scan_found = false;                                             // Reset detection flag 

  if (scan_active == 0) {                                         // If keyboard scanning is INACTIVE,
    scan_found = true;                                            // set scan_found to TRUE.
  }
  
  attachInterrupt(digitalPinToInterrupt(Row0), ISR_Row0, LOW); 
  attachInterrupt(digitalPinToInterrupt(Row3), ISR_Row3, LOW);
  
  systemUNRESET();                                                // Remove system from RESET mode
}

// *** RESET function ***
void systemRESET() {                                              // Set C64 into reset mode; may seem backwards, but see Note 1: above
  digitalWrite( C64RES, HIGH );                                   // Reset the C64
  pinMode( C64RES, INPUT );                                       // Make C64 RESET pin an input
  digitalWrite( C64RES, HIGH );                                   // Keep C64 in reset

  // Set EXROM into reset mode
  digitalWrite( EXROMRES, LOW );                                  // Reset the EXROM
  pinMode( EXROMRES, OUTPUT );                                    // Make EXROM RESET pin an output
  digitalWrite( EXROMRES, LOW );                                  // Keep EXROM in reset
}

// *** UNRESET function ***
void systemUNRESET() {
  delay(200);                                                     // 200ms buffer before releasing C64RES line
                                                                  // Remove C64 from reset mode; may seem backwards, but see Note 1: above
  digitalWrite( C64RES, LOW );                                    // Unreset the C64
  pinMode( C64RES, OUTPUT );                                      // Make C64 RESET pin an output
  digitalWrite( C64RES, LOW );                                    // Remove C64 from reset
  
  delay(300);                                                     // 300ms buffer before releasing EXROM line
                                                                  // Set EXROM out of reset mode
  digitalWrite( EXROMRES, HIGH );                                 // Unreset the EXROM
  pinMode( EXROMRES, INPUT );                                     // Make EXROM RESET pin an input
  digitalWrite( EXROMRES, HIGH );                                 // Remove EXROM from reset
}

// *** Set address bits function ***
void SetAddressBits( byte addr ) {                                // Parameter is addr (0..7)
  if ((addr & 1) == 0) {                                          // A13
           digitalWrite(KSW_A13, LOW);
  } else { digitalWrite(KSW_A13, HIGH); }

  if ((addr & 2) == 0) {                                          // A14
           digitalWrite(KSW_A14, LOW);
  } else { digitalWrite(KSW_A14, HIGH); }
  
  if ((addr & 4) == 0) {                                          // A15
           digitalWrite(KSW_A15, LOW);
  } else { digitalWrite(KSW_A15, HIGH); }
}

// *** Set video bits function ***
void SetVideoBits( byte addr ) {                                  // Parameter is addr (0..1)
  if (addr == 0) {                                                // Switch to PAL
    digitalWrite(NeN, LOW); 
    digitalWrite(PeN, HIGH);
    delay( 500 );                                                 // Pulse PeN HI for a half second
    digitalWrite(PeN, LOW);                                       // Set it back to LO
  } else {                                                        // Switch to NTSC
    digitalWrite(NeN, HIGH); 
    digitalWrite(PeN, LOW); 
    delay( 500 );                                                 // Pulse NeN HI for a half second
    digitalWrite(NeN, LOW);                                       // Set it back to LO
  }
}

// *** Switch Kernal function ***
void SwitchKernal() {
  systemRESET();                                                  // Put system in RESET mode
  SetAddressBits( kernal_addr );                                  // Set the Kernal address bits
  delay( 2000 );                                                  // Wait for two seconds
  systemUNRESET();                                                // Remove system from RESET mode
  delay( 2000 );                                                  // Wait another two seconds
  scan_counter = Default_Scan_Count;                              // Reset scan countdown
  if (scan_active == 1) {                                         // If we're checking for scan activity...
    scan_found = false;                                           // ...reset detection flag
  } else {                                                        // Otherwise we're NOT checking for scan activity...
    scan_found = true;                                            // Set scan found to true
  }
}

// *** Switch video mode function ***
void SwitchVideo() {
  systemRESET();                                                  // Put system in RESET mode
  SetVideoBits( np_mode );                                        // Set the video bits NeN & PeN
  delay( 2000 );                                                  // Wait two seconds
  systemUNRESET();                                                // Remove system from RESET mode
  delay( 2000 );                                                  // Wait another two seconds
  scan_counter = Default_Scan_Count;                              // Reset scan countdown
  if (scan_active == 1) {                                         // If we're checking for scan activity...
    scan_found = false;                                           // ...reset detection flag
  } else {                                                        // Otherwise we're NOT checking for scan activity...
    scan_found = true;                                            // Set scan found to true
  }
}

// *** Interrupt Service Routine for Row0 ***
void ISR_Row0() {
  if (!numPressed) {                                              // Only process info after the flag was reset by main routine
     colData = PINC & 0x1F;                                       // Read the columns from Port C input pins register AND 0x1F
     scan_found = true;                                           // Any activity here indicates a working Kernal
     numOdd = true;                                               // Tell system Row0 has the odd numbers
     if ((colData != 0x1F) && (colData !=0)) {                    // Only set the flag when a number is pressed 
       numPressed = true;
     }  
  }
}

// *** Interrupt Service Routine for Row3 ***
void ISR_Row3 () {
  if (!numPressed) {                                              // Only process info after the flag was reset by main routine
     colData = PINC & 0x1F;                                       // Read the columns from Port C input pins register AND 0x1F
     scan_found = true;                                           // Any activity here indicates a working Kernal
     numOdd = false;                                              // Tell system Row3 has the even numbers
     if ((colData != 0x1F) && (colData !=0)) {                    // Only set the flag when a number is pressed 
       numPressed = true;
     }  
  }
}

// *** Main program loop ***
void loop() {
  
  if (!scan_found) {                                              // If previous scan activity was NOT detected...
    columns = PINC & 0x1F;                                        // ...read the columns from Port C input pins register AND 0x1F
    if ((columns != 0x1F)) {                                      // If any NEW activity found...
      scan_found = true;                                          // ...set flag.
    } else {                                                      // Otherwise NEW scan activity was NOT detected...
      if (scan_counter > 0) {                                     // ...If the count down has NOT elapsed...
        scan_counter--;                                           // ......then decrement...
        delayMicroseconds( 33 );                                  // ......and wait 33 microseconds
      } else {                                                    // ...Otherwise the count down HAS elapsed...
        if (scan_active == 1) {                                   // ......If we ARE scanning
          kernal_num = 1;                                         // .........set Kernal Number 1 (it is assumed that this is not empty)
          EEPROM.write(KernalSlot, kernal_num);                   // .........and save in EEPROM
          kernal_addr = kernal_num - 1;                           // .........calculate the address bits
          SwitchKernal();                                         // .........and switch kernal and reset C64
        } else {                                                  // ......Otherwise (we are not scanning)
            scan_counter = Default_Scan_Count;                    // .........Reset scan countdown (because it has elapsed!)
            scan_found = true;                                    // .........and fake the scanner into thinking we're good (because we're not scanning)
        }
      }
    }
  }
  
// *** Process RESTORE key ***
  if (digitalRead( RESTOREPin ) == HIGH) {                        // If RESTORE reading is HIGH...
     restore_found = false;                                       // ...the RESTORE key is not pressed
     restore_counter = Default_Restore_Count;                     // ...so reset the RESTORE count down
  } else {                                                        // Otherwise the RESTORE reading is LOW...
    restore_found = true;                                         // ...and the RESTORE key is pressed,
    if (restore_counter > 0) {                                    // ...make sure countdown hasn't expired
      restore_counter--;                                          // ...and count down
    } else {                                                      // ...Otherwise the RESTORE count down has elapsed...
        systemRESET();                                            // ......so put system into RESET mode
        scan_counter = Default_Scan_Count;                        // ......reset scan countdown
        scan_found = false;                                       // ......reset detection flag
        delay( 2000 );                                            // ......and wait for two seconds
        systemUNRESET();                                          // ......then remove system from RESET mode
    }
  }

// *** Process when a NUMBER key is pressed (set by ISR) ***
  if (numPressed) { 
    if      ((colData & B00010000) == 0) { key = 1; }             // Column 7: "1" key is pressed
    else if ((colData & B00000001) == 0) { key = 3; }             // Column 1: "3" key is pressed
    else if ((colData & B00000010) == 0) { key = 5; }             // Column 2: "5" key is pressed
    else if ((colData & B00000100) == 0) { key = 7; }             // Column 3: "7" key is pressed
    else if ((colData & B00001000) == 0) { key = 9; }             // Column 4: "9" key is pressed
                                                                  // The even numbers are on Row3, the odd numbers are on Row1
                                                                  // numOdd is set by the ISR
    if      (!numOdd)                    { key++;   }             // The even numbers are higher by one
    if      (key == 10)                  { key = 0; }             // In case it is a "0" 

    if ((restore_found) && (key >= 0) && (key <= 9)) {            // RESTORE + number key pressed (0 - 9)
      switch (key) {
        default:                                                  // Keys 1 - 8: Switch to ROM number 1 - 8
          scan_active = 1;                                        // Every valid Kernal enters ACTIVE state
          kernal_num = key;                                       // Get Kernal Number
          EEPROM.write(KernalSlot, kernal_num);                   // Write Kernal number to the EEPROM
          kernal_addr = kernal_num - 1;                           // Calculate the address bits. It should be between 0 and 7,
          EEPROM.write(ScanActiveSlot, scan_active);              // Write scan status to EEPROM
          SwitchKernal();                                         // Switch Kernal Number (also resets C64)
          break;                                                  // --- Exit this case ---
        case 0:                                                   // Key 0: Switch to ROM 1 and turn of keyboard scanning (cartridges sometimes need this)
          scan_active = 0;                                        // Turn scan off
          kernal_num = 1;                                         // Switch to KERNAL 1 (known good CBM v3 KERNAL)
          EEPROM.write(KernalSlot, kernal_num);                   // Write Kernal number to the EEPROM
          kernal_addr = kernal_num - 1;                           // Calculate the address bits. It should be between 0 and 7,
          EEPROM.write(ScanActiveSlot, scan_active);              // Write scan status to EEPROM
          SwitchKernal();                                         // Switch Kernal Number (also resets C64)
          break;                                                  // --- Exit this case ---
        case 9:                                                   // Key 9: Switch video modes
          if (np_mode == 1) {                                     // Flipping np_mode
            np_mode = 0;                                          // Now in PAL mode
          } else {
            np_mode = 1;                                          // Now in NTSC mode
          }
          EEPROM.write(NPModeSlot, np_mode);                      // Write the new np_mode to EEPROM
          SwitchVideo();                                          // Run the SwitchVideo() method
          break;                                                  // --- Exit this case ---
      }
    }
  }
  key = 128;                                                      // Reset the key number
  numPressed = false;                                             // Reset the flag so the ISR can process the key matrix again
  delayMicroseconds( 1000 );                                      // Delay 1 millisecond per loop
}
