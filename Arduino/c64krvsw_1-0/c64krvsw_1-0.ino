#include <EEPROM.h>

// C64 KRV (Kernal ROM Video) Switcher
// v1.0 by Sean Harrington
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
//    RESTORE + 1 - Kernal image in bank 1 (0000-1FFF) of 27512 loads, and system resets (usually CBM Kernal v3)
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
// v1.0, 10 SEP 2019 - Switched naming of RSVD01 & RSVD02 to NeN and PeN
//                     Switched naming of EEPROMAddr to KernalAddr
//                     Updated NTSC/PAL EEPROM read & write routine
//                     Removed reference to VAddr and replaced with NPMode (duplicate variables)
// v0.4, 05 AUG 2019 - Revised NTSC/PAL switch to PULSE 5V (HIGH) rather than hold
// v0.3, 21 JUL 2019 - Added EXROM reset stuff
//                     Turned systemRESET and systemUNRESET into functions
//                     Added debugOutput boolean constant for enabling/disabling Serial output
//                     More error correction & formatting of comments
// v0.2, 14 JUL 2019 - Removed short/long board references (my solution only supports longboards)
//                     Error corrections & commenting
// v0.1, 05 JUL 2019 - Updated RSVD01 and RSVD02 to serve as NTSC/PAL switch toggles
//                     Created RESTORE + 9 function to serve as NTSC/PAL toggle key
//
// Note 1:  Transistor Q1 (2N3904 NPN) is used to flip the HIGH to a LOW for the C64_RESET.
//          It may seem counter-intuitive to set C64RES pin to HIGH to set the C64_RESET pin LOW,
//          but this is in fact what it does.

// *** Configuration ***
const bool recover_empty = true;                                  // Enable recover to Kernal #1, in case an empty/not working Kernal was selected
const int  NumKernal = 8;                                         // This is the highest valid Kernal number
const bool debugOutput = false;                                   // Allows toggling of Serial Monitor output
 
// *** Pin definitions ***
const int row0 = 2;                                               // Row signal, input, active low, INT0
const int row3 = 3;                                               // Row signal, input, active low, INT1
const int KSW_A13 = 4;                                            // Output, A13 to Kernal adaptor board
const int KSW_A14 = 5;                                            // Output, A14 to Kernal adaptor board
const int KSW_A15 = 6;                                            // Output, A15 to Kernal adaptor board
const int NeN = 9;                                                // Output, NTSC Enable (HI), connected to N-EN on VIC-II² video switcher board 
const int PeN = 10;                                               // Output, PAL Enable (HI), connected to P-EN on VIC-II² video switcher board
const int C64RES = 7;                                             // Output, RESET IO, connected through transistor to RESET pin on C64 (active HIGH, see Note 1: at top)
const int EXROMRES = 8;                                           // Input, EXROM, connected through resistor to EXROM reset pin on C64, active low
const int col1 = A0;                                              // Input, column signal, active low
const int col2 = A1;                                              // Input, column signal, active low
const int col3 = A2;                                              // Input, column signal, active low
const int col4 = A3;                                              // Input, column signal, active low
const int col7 = A4;                                              // Input, column signal, active low
const int nRESTORE = A5;                                          // Input, RESTORE key, active low
const int Restore_Count = 2000;                                   // Count down appr. 2 seconds for RESTORE key before reset

// *** Other Constants ***
#define KernalAddr 0                                              // EEPROM Address of last Kernal selection
#define NPModeAddr 2                                              // NTSC/PAL mode address
#define ScanDelay 10000                                           // Delay ibn ms before reset for no scan activity
                                                                  //   or an empty Kernal memory slot, respectively
 // *** Variables ***
byte KOffset = 0;                                                 // Offset for Kernal number
byte KNumber = 0;                                                 // Kernal number
byte KAddr = 0;                                                   // Address bits for Kernal
byte NPMode = 1;                                                  // Default NTSC/PAL mode, 1 is NTSC, 0 is PAL
bool RestoreKey = 0;                                              // Holds the info if the RESTORE key is pressed 
int  countdown = 0;                                               // Count down for something
byte key = 128;                                                   // Calculate the Key
int  KMax;                                                        // Holds the highest Kernal number

// *** Data Exchange variables for ISR (have to be volatile) ***
volatile bool NumPressed = 0;                                     // ISR flag for number pressed
volatile bool NumOdd = 0;                                         // ISR flag for odd numbers. Row0 => odd numbers, Row3 => even numbers
volatile byte ColData = 0x1F;                                     // Column data, read from PORTD

                                                                  // In case the Kernal switch switches to a memory slot without 
                                                                  // a proper Kernal software there is no keyboard scan activity.
                                                                  // That is why the Kernal switch cannot detect keys anymore
                                                                  // to circumvent this, the column scan activity is detected.
                                                                  // If there is no activity after switching Kernals the Kernal
                                                                  // is set to #1 and the C64 is reset.

byte columns;                                                     // Variable for reading out the column
int scan_cnt;                                                     // 5000 miliseconds without scan changing to Kernal #1 and resetting
int scan_found;                                                   // Flag for keyboard activity detected
byte inactive = 0;                                                // Keyboard scan INACTIVE: 1 = INACTIVE, 0 = active

void setup() {                                                    // Keyboard scan signals:
  pinMode( row0, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( row3, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( col1, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( col2, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( col3, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( col4, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( col7, INPUT_PULLUP);                                   // Pullup required for MOS logic
  pinMode( nRESTORE, INPUT_PULLUP);                               // Pullup required for MOS logic
                                                                  // Control outputs (for Kernal Adaptor and C64):
  pinMode( KSW_A13, OUTPUT );                                     // Make Kernal switcher pin A13 an output   
  pinMode( KSW_A14, OUTPUT );                                     // Make Kernal switcher pin A14 an output
  pinMode( KSW_A15, OUTPUT );                                     // Make Kernal switcher pin A15 an output
  pinMode( NeN, OUTPUT );                                         // Make NTSC video switcher pin an output
  pinMode( PeN, OUTPUT );                                         // Make PAL video switcher pin an output
  pinMode( C64RES, OUTPUT );                                      // Make C64 RESET pin an output
  pinMode( EXROMRES, INPUT );                                     // Make EXROM RESET pin an input

                                                                  // Last Kernal read from EEPROM
  KNumber = EEPROM.read(KernalAddr);                              // Read the Kernal number from EEPROM
  if ((KNumber < 1) || (KNumber > 8-KOffset)) {                   // If out of range,
    KNumber = 1;                                                  //   the first Kernal is selected.
    EEPROM.write(KernalAddr, KNumber);
  }
  inactive = EEPROM.read(KernalAddr+1);
  if (inactive > 1) {
    inactive = 0;
    EEPROM.write(KernalAddr+1, inactive);
  }
  KAddr = KNumber - 1 + KOffset; // Calculate the address bits.   // It should be between 0 and 7,

                                                                  // Last NTSC/PAL mode read from EEPROM
  NPMode = EEPROM.read(NPModeAddr);                               // Read the Video Mode from EEPROM
  if ((NPMode != 0) && (NPMode != 1)) {                           // If out of range,
    NPMode = 1;                                                   //   NTSC mode is selected.
    EEPROM.write(NPModeAddr, NPMode);
  }
  NPMode;
  
  systemRESET();                                                  // Set system to RESET mode

                                                                  // Switch the KERNAL and Video Mode
  SetAddressBits( KAddr );                                        // Set the MS address bits for the Kernal ROM
  SetVideoBits ( NPMode );                                        // Set the video address bits

  if (recover_empty == true) {
    scan_cnt = ScanDelay;                                         // Reset scan countdown
    scan_found = false;                                           // Reset detection flag 
    KMax = 8 - KOffset; 
  }
  else {
    scan_cnt = 0;                                                 // Reset scan countdown
    scan_found = true;                                            // Reset detection flag
    KMax = NumKernal;
  }
  if (inactive == 1) {                                            // If scan timeout is INACTIVE,
    scan_found = true;                                            //   set scan_found to TRUE.
    scan_cnt = 0;
  }
  
  attachInterrupt(digitalPinToInterrupt(row0), ISR_row0, LOW); 
  attachInterrupt(digitalPinToInterrupt(row3), ISR_row3, LOW);
  
  systemUNRESET();                                                // Remove system from RESET mode

  if (debugOutput) {                                              // Only if debugOutput is TRUE (1)
    Serial.begin(9600);                                           //   start serial interface
    Serial.println( "\nC64 KRV Switcher v1.0" );
    if (recover_empty == true) {
      Serial.print("Empty Kernal slot (=scan activity) detection: ");
      if (inactive == 1 ) {
        Serial.println("INACTIVE.");
      }
      else {
        Serial.println("active.");
      }
    }
    else {
      Serial.print("Number of Kernals: ");
      Serial.println(NumKernal);
    }
    Serial.print("Kernal number: ");
    Serial.println( KNumber );
    Serial.print("Video Mode: ");
    if (NPMode == 1) {
      Serial.println("NTSC");
    }
    else if (NPMode == 0) {
      Serial.println("PAL");
    }
    else {
      Serial.println("undefined");
    }
  }
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
   if ((addr & 1) == 0) {                                         // A13
    digitalWrite(KSW_A13, LOW);
  }
  else {
    digitalWrite(KSW_A13, HIGH);  
  }
  if ((addr & 2) == 0) {                                          // A14
    digitalWrite(KSW_A14, LOW);
  }
  else {
    digitalWrite(KSW_A14, HIGH);  
  }
  if ((addr & 4) == 0) {                                          // A15
    digitalWrite(KSW_A15, LOW);
  }
  else {
    digitalWrite(KSW_A15, HIGH);  
  }
}

// *** Set video bits function ***
void SetVideoBits( byte addr ) {                                  // Parameter is addr (0..1)
   if (addr == 0) {                                               // Switch to PAL
    digitalWrite(NeN, LOW); 
    digitalWrite(PeN, HIGH);
    delay( 500 );                                                 // Pulse PeN HI for a half second
    digitalWrite(PeN, LOW);                                       // Set it back to LO
  }
  else {                                                          // Switch to NTSC
    digitalWrite(NeN, HIGH); 
    digitalWrite(PeN, LOW); 
    delay( 500 );                                                 // Pulse NeN HI for a half second
    digitalWrite(NeN, LOW);                                       // Set it back to LO
  }
}

// *** Switch Kernal function ***
void SwitchKernal() {

  systemRESET();                                                  // Put system in RESET mode
  
  SetAddressBits( KAddr );                                        // Set the Kernal address bits
  if (debugOutput) {                                              // Only if debugOutput is TRUE (1)
    Serial.print("Change to Kernal number: ");                    //   send the status to serial
    Serial.println( KNumber );
  }
  delay( 2000 );                                                  // Wait for two seconds
  
  systemUNRESET();                                                // Remove system from RESET mode
  
  delay( 2000 );                                                  // Wait another two seconds
  if ((recover_empty == true) && (inactive == 0)) {
    scan_cnt = ScanDelay;                                         // Reset scan countdown
    scan_found = false;                                           // Reset detection flag
  }  
  else {
    scan_cnt = 0;
    scan_found = true;
  }
}

// *** Switch video mode function ***
void SwitchVideo() {

  systemRESET();                                                  // Put system in RESET mode
  
  SetVideoBits( NPMode );                                          // Set the video bits NeN & PeN
  
  if (debugOutput) {                                              // Only if debugOutput is TRUE (1)
    Serial.print("Change to Video Mode: ");                       //   send the status to serial
    if (NPMode == 0) {
      Serial.println("PAL");
    } else {
      Serial.println("NTSC");
    }
  }

  delay( 2000 );                                                  // Wait two seconds

  systemUNRESET();                                                // Remove system from RESET mode
  
  delay( 2000 );                                                  // Wait another two seconds
  if ((recover_empty == true) && (inactive == 0)) {
    scan_cnt = ScanDelay;                                         // Reset scan countdown
    scan_found = false;                                           // Reset detection flag
  }  
  else {
    scan_cnt = 0;
    scan_found = true;
  }
}

// *** Interrupt Service Routine for Row0 ***
void ISR_row0() {
  if (NumPressed == 0) {                                          // Only process info after the flag was reset by main routine
     ColData = PINC & 0x1F;                                       // Read the columns from Port C input pins register AND 0x1F
     scan_found = true;                                           // Any activity here indicates a working Kernal
     NumOdd = 1;                                                  // Tell system row0 has the odd numbers
     if ((ColData != 0x1F) && (ColData !=0)) {                    // Only set the flag when a number is pressed 
       NumPressed = 1;
     }  
  }
}

// *** Interrupt Service Routine for Row3 ***
void ISR_row3 () {
  if (NumPressed == 0) {                                          // Only process info after the flag was reset by main routine
     ColData = PINC & 0x1F;                                       // Read the columns from Port C input pins register AND 0x1F
     scan_found = true;                                           // Any activity here indicates a working Kernal
     NumOdd = 0;                                                  // Tell system row3 has the even numbers
     if ((ColData != 0x1F) && (ColData !=0)) {                    // Only set the flag when a number is pressed 
       NumPressed = 1;
     }  
  }
}

// *** Main program loop ***
void loop() {
                                                                  // Find the scan activity to make sure, the selected Kernal is working
  if (scan_found == false) {                                      // If scan activity is NOT detected
    columns = PINC & 0x1F;                                        //   read the columns from Port C input pins register AND 0x1F
    if ((columns != 0x1F)) {                                      // Any activity found?
      scan_found = true;                                          // Set flag
      if (debugOutput) {                                          // Only if debugOutput is TRUE (1)
        Serial.println( "Keyboard scan detected.");               //   send the status to serial
      }
    }
    else {
      if (scan_cnt > 0) {                                         // If the count down is not elapsed,
        scan_cnt--;                                               //   then decrement
        delayMicroseconds( 33 );
      }
      else {
        if (debugOutput) {                                        // Only if debugOutput is TRUE (1)
          Serial.println( "No keyboard scan detected.");          // the count down is elapsed!
        }
        KNumber = 1;                                              // Set Kernal Number 1 (it is assumed that this is not empty)
        EEPROM.write(KernalAddr, KNumber);                        //   and save in EEPROM
        KAddr = KOffset;                                          // Calculate the address bits
        SwitchKernal();                                           // Switch Kernal Number and reset C64
      }
    }
  }
  
// *** Process restore key ***
  if (digitalRead( nRESTORE ) == HIGH) {                          // If RESTORE reading is HIGH,
     RestoreKey = 0;                                              //   the RESTORE key is not pressed
     countdown = Restore_Count;                                   // Restart the count down
  }
  else {                                                          // The RESTORE reading is LOW,
    RestoreKey = 1;                                               //   and the RESTORE key is pressed,
    if (countdown > 0) {                                          //   make sure countdown hasn't expired
      --countdown;                                                //   and count down
      if (countdown == 0) {                                       // If the ount down elapsed,
        systemRESET();                                            //   put system into RESET mode
        if (debugOutput) {                                        // Only if debugOutput is TRUE (1)
          Serial.println("Restore key => Reseting the C64");      //   send the status to serial
        }
        if (recover_empty == true) {
          scan_cnt = ScanDelay;                                   // Reset scan countdown
          scan_found = false;                                     // Reset detection flag
        }
        delay( 2000 );                                            // Wait for two seconds
        systemUNRESET();                                          // Remove system from RESET mode
      }
    }
  }
                                                                  // The ISR will set NumPressed to 1, in case some number key is pressed 
                                                                  // In case something is found, the column data, which was read by the ISR 
                                                                  // is being processed here.
                                                                  
  if (NumPressed == 1) {                                          // If a number key is pressed, process the data
    if ((ColData & B00010000) == 0) {                             // Column 7: "1"
      key = 1;
    }
    else if ((ColData & B00000001) == 0) {                        // Column 1: "3"
      key = 3;
    }
    else if ((ColData & B00000010) == 0) {                        // Column 2: "5"
      key = 5;
    }
    else if ((ColData & B00000100) == 0) {                        // Column 3: "7"
      key = 7;
    }
    else if ((ColData & B00001000) == 0) {                        // Column 4: "9"
      key = 9;
    }
                                                                  // The even numbers are on row 3, the odd numbers are on row 1
                                                                  // NumOdd is set by the ISR

    if (NumOdd == 0) {                                            // The even numbers are higher by one
      key++;
    }
    if (key == 10) {                                              // In case it is a "0" 
      key = 0;
    }
    if (debugOutput) {                                            // Only if debugOutput is TRUE (1)
      Serial.print("Key ");
      Serial.print(key);
      Serial.println(" detected.");
    }
    if ((key > 0) && (key < KMax+1) && (RestoreKey == 1)) {       // RESTORE key pressed and number key in range
       KNumber = key;                                             // Get Kernal Number
       EEPROM.write(KernalAddr, KNumber);                         // Write Kernal number to the EEPROM
       KAddr = KNumber - 1 + KOffset;                             // Calculate the address bits. It should be between 0 and 7,
       inactive = 0;                                              // Every valid Kernal leaves INACTIVE state
       EEPROM.write(KernalAddr+1, inactive);                      // Write INACTIVE status to EEPROM
       SwitchKernal();                                            // Switch Kernal Number (also resets C64)
    }
    else if ((key == 0) && (RestoreKey == 1)) {                   // Key "0" switches to Kernal #1 and enters INACTIVE state
      KNumber = 1;                                                // INACTIVE means that the initial key scan timeout is INACTIVE
      EEPROM.write(KernalAddr, KNumber);                          //   which is useful for catridges that do not perform a keyboard scan
      KAddr = KNumber - 1 + KOffset;
      inactive = 1;                                               // Enter INACTIVE state
      EEPROM.write(KernalAddr+1, inactive);                       // Write INACTIVE status to EEPROM
      if (debugOutput) {                                          // Only if debugOutput is TRUE (1)
        Serial.println( "Kernal 1 and scan timeout INACTIVE." );  //   send to serial
      }
      SwitchKernal();                                             // Switch Kernal Number and reset C64
    }
    else if ((key == 9) && (RestoreKey == 1)) {                   // RESTORE + Key 9 toggles video mode
      if (NPMode == 1) {                                          // If currently NTSC, switch to PAL
        NPMode = 0;                                               // Flip video mode to 0
        EEPROM.write(NPModeAddr, NPMode);                         // Write video mode to EEPROM
        SwitchVideo();                                            // Set video mode (also resets C64)
      } else {                                                    // If NOT currently NTSC, switch to NTSC
        NPMode = 1;                                               // Flip video mode to 1
        EEPROM.write(NPModeAddr, NPMode);                         // Write video mode to EEPROM
        SwitchVideo();                                            // Set video mode (also resets C64)
      }
    }
    else {
      key = 128;                                                  // Reset the key number
    }
    NumPressed = 0;                                               // Reset the flag, so the ISR can process key matrix again
  }
  else {
    key = 128;                                                    // Reset the key number
  }
  delayMicroseconds( 1000 );                                      // Delay 1 millisecond per loop
}
