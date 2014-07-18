
// SN76489 serial MIDI via Teensy board
// added code to clock SN76489 via C7 (Arduino 10) OC4A fast PWM timer
// modified code to use standard MIDI.h library for MIDI over serial
// Special thanks to Mutable Instruments for their excellent 12 db/oct multi
// -mode VCF filter design featured in thier Anushri synth.
// VCF cutoff control via MCP4921 12-bit SPI DAC
// VCF resonance control via MCP4131 100K 128-step digital pot
/* version 1.0
 
 working in this version:
 note on/off handling
 noteOn stack
 osc2/3 octave up/down
 osc2/3 detune
 osc1/2/3 level
 noise level
 pitch bend
 tremolo (AM) square wave
 VCA envelope (using built-in attenuators)
 glide
 MIDI channel set
 VCF envelope 
 
 to do:
 vcf cutoff tracking
 vibrato
 
 */

// this is a monosynth

/*
 ==== DATA ====
 Teensy PORTB 4 = Teensy Digital Pin 0 -->  SN76489 Pin 10 (Data 0)
 Teensy PORTB 5 = Teensy Digital Pin 1 -->  SN76489 Pin 11 (Data 1)
 Teensy PORTB 6 = Teensy Digital Pin 2 -->  SN76489 Pin 12 (Data 2)
 Teensy PORTB 7 = Teensy Digital Pin 3 -->  SN76489 Pin 13 (Data 3)
 Teensy PORTF 4 = Teensy Digital Pin 13 -->  SN76489 Pin 15 (Data 4)
 Teensy PORTF 5 = Teensy Digital Pin 14 -->  SN76489 Pin 1 (Data 5)
 Teensy PORTF 6 = Teensy Digital Pin 15 -->  SN76489 Pin 2 (Data 6)
 Teensy PORTF 7 = Teensy Digital Pin 4 --> SN76489 Pin 3 (Data 7)
 
 ==== CONTROL AND MIDI INPUT ====
 Teensy PORTD 0 = Teensy Digital Pin 5 --> SN76489 Pin 5 (Write Enable)
 Teensy PORTD 2 = Teensy Digital Pin 7 --> Receive data from MIDI INPUT CIRCUIT
 
 ==== AUDIO OUTPUT ====
 SN76489 Pin 7 --> Audio Output Signal
 SN76489 Pin 8 --> Audio Output Ground
 
 ==== MIDI INPUT CIRCUIT (ONLY IF REQUIRED) ===
 MIDI Input DIN 5 Pin 4 --> 4n28 Pin 1
 MIDI Input DIN 5 Pin 5 --> 4n28 Pin 2
 N4148 Diode (positive anode) --> 4n28 Pin 1
 N4148 Diode (negative cathode) --> 4n28 Pin 2
 4n28 Pin 6 --> 100k resistor (leg 1) 
 100k resistor (leg 2) --> Teensy Ground
 4n28 Pin 5 --> Teensy PORTD 2 = Teensy Digital Pin 7
 4n28 Pin 5 --> 3.3k resistor (leg 1)
 3.3k resistor (leg 1) --> Teensy 5V
 4n28 pin 4 --> Teensy Ground
 
 === MIDI CC setting button ===
 Input pin 12
 
 ==== SPI pins ====
 SCLK pin 1
 MOSI pin 2
 DAC CS pin 0
 Digital Pot CS pin 21
 
 ==== multiplexer pins ====
 A0 pin 20
 A1 pin 6
 
 */

// serial midi
#include <MIDI.h>
// for using atmega eeprom
#include <EEPROM.h>
// Bounce button library
// from here git clone https://github.com/thomasfredericks/Bounce-Arduino-Wiring.git
#include <Bounce2.h>
// SPI library
#include <SPI.h>
// DAC library
#include <DAC_MCP49xx.h>


#define TIMER4_RESOLUTION 1023UL
#define PLL_FREQ 48000000UL
#define LED_PIN 11
#define CHANNEL_SET_PIN 12
#define wr 5
#define COMMAND_DELAY 100
#define DEBOUNCE_MS 20
#define LONG_CLICK_MS 1000
#define EE_CHECKBYTE_1 125
#define EE_CHECKBYTE_2 126
#define EE_MIDI_CHANNEL 127
#define DAC_CS_PIN 0
#define POT_CS_PIN 21
#define MUX_A0_PIN 20
#define MUX_A1_PIN 6

// max AM modulation on or off time
#define AM_MAX_TIME 500
// min AM modulation on or off time
#define AM_MIN_TIME 1

// maxium stack size for noteOn notes
// I chose ten assuming no arm sweeps it should be enough
#define MAX_NOTES 10

// maximum VCA envelope phase time in milliseconds
#define VCA_MAX_ENV_TIME 1000 

// VCA envelope loop tick period in milliseconds
#define VCA_ENVELOPE_TICK_TIME 10

// glide loop tick period in milliseconds
#define GLIDE_TICK_TIME 10

// maximum glide time in milliseconds
#define MAX_GLIDE_TIME 500

// maximum VCF envelope phase in microseconds
#define VCF_MAX_ENV_TIME 5000000.0
// maximum VCF envelope release release time
#define VCF_MAX_ENV_RELEASE_TIME 10000000.0

// VCF tick time in microseconds
#define VCF_ENVELOPE_TICK_TIME 1000

// note that CV to VCF is inverted
// max cutoff is at 0v
// min cutoff is at 5v

// VCF CV where no audio is audible
#define MIN_VCF_CUTOFF 2046
// VCF CV where highest frequencies are passed
#define MAX_VCF_CUTOFF 0

// envelope states
#define STATE_0 B00000000
#define STATE_A B00001000
#define STATE_D B00000100
#define STATE_S B00000010
#define STATE_R B00000001

// digital pot stuff
// set pot 0
#define POT_0 B00000000


// set a pot to zero
#define SET_ZERO B00000000

unsigned long pwmPeriod = 0;
ISR(TIMER4_OVF_vect) {
}

long clock = 2000000; // clock speed in Hz to SN76489

byte working_byte = 0; // working byte for the SN76489 stuff

int bendamount = 2;
int detuneAmount = 7;

// MIDI channel
byte myChannel = 1;
// incoming data from noteOn
byte ccnumber; 
byte ccvalue;

uint8_t coarsePitch = 64;

uint8_t pitchTableOffset = 0;

// variables to hold running state data
// track if note already playing
float bend_data;
byte bendMSB;
byte bend_MSB = 64;
byte bendLSB;

// oscillator volumes
float v1;
float v2;
float v3;
float v4;

// note struct
// contains a pitch and a velocity
typedef struct
{
  byte myPitch;
  byte myVelocity;
} 
note;

// raw oscillator frequency values
// current values
int osc1F1;
int osc2F1;
int osc3F1;
// last values
float osc1F2;
float osc2F2;
float osc3F2;

// flag if glide is in effect
// determined by legato playing
byte glideNote = 0;
// glide time CC value
byte glideTimeValue = 20;
// glide time in milliseconds
int glideTime;
// ticks per glide
int glideTicks;
// glide change per tick
// osc1
float glideChangePerTick1;
// osc2
float glideChangePerTick2;
// osc3
float glideChangePerTick3;
// glide tick timer
unsigned long glideTimer;
// glide tick counter
int glideTickCount;

// array of notes from noteOn handler
// MAX_NOTES + 1 is elements so we always have an extra note which
// is always zero, for when we shift left into a deleted note from
// a noteOff event
note notes[MAX_NOTES + 1];

// noise lookup table
byte noiseLookup[] = {
  B11100000, // C
  B11100000, // C#
  B11100001, // D
  B11100001, // D#
  B11100010, // E
  B11100000, // F
  B11100101, // F#
  B11100110, // G
  B11100110, // G#
  B11100011, // A
  B11100011, // A#
  B11100111, // B
};


// Vibrato Modulation
int vibAccum = 0;
byte vibSpeed = 10;
byte vibModOn = 0;
byte vibDepth = 0;
byte vibFlip = 0;

// oscillator 1
byte osc1Level = 127;

// oscillator 2
// level
byte osc2Level = 0;
// octave
// up > 96, down < 32 to provide some midpoint leeway
byte osc2Octave = 64;
// detune - 64 is none
byte osc2Detune = 64;

// oscillator 3
byte osc3Level = 0;
// octave
// up > 96, down < 32 to provide some midpoint leeway
byte osc3Octave = 64;
// detune - 64 is none
byte osc3Detune = 64;

// noise
// level
byte osc4Level = 0;

// filter mode
// defaults to lowpass
// MIDI CC
byte filterModeValue = 0;

// AM depth from MIDI CC
byte amDepth = 0;
// AM rate from MIDI CC
byte amRate = 0;
// times the AM modulation period
unsigned long amTimer;
// AM modulation rate from calculated from MIDI CC
unsigned long amPeriod = AM_MAX_TIME;
// tracks if the AM is in effect
byte amOn = 0;
// AM envelope level volume adjustment
float amVolume = 1.0;


// tracks if a note is coming from the stack via noteOff
// this allows us to avoid pushing the note back onto the stack
byte wasFromNoteOff = 0;

// envelope stuff
// tracks the timing of an envelope tick cycle
unsigned long vcaTickTimer;
unsigned long vcfTickTimer;
// VCA ADSR
// no bits set means no note playing
// VCA - R 1
// VCA - S 2
// VCA - D 4
// VCA - A 8
// VCA                              ADSR
volatile uint8_t vcaEnvState = B00000000;
volatile uint8_t vcfEnvState = B00000000;

// VCA envelope volume
float vcaVolume; 
// VCF envelope cutoff
float vcfCutoff;
// VCF cutoff output to DAC
int vcfCutoffOutput;

// phase A
// VCA A value
// this is a time value
// set the default here; 0 - 127
byte vcaAValue = 0;
// VCA A time in ticks
float vcaATime;
// ticks per A envelope phase
int vcaATicks;
// VCA A amplitude change per tick
float vcaAChangePerTick;
// VCA A volume
float vcaAVolume;

// VCF A value
// this is a time value
// set the default here; 0 - 127
byte vcfAValue = 0;
// VCF A time in ticks
long vcfATime;
// ticker per A envelope phase
long vcfATicks;
// VCF A cutoff change per tick
float vcfAChangePerTick;


// phase D
// VCA D value
// this is a time value
// set the default here; 0 - 127
byte vcaDValue = 0;
// VCA D time in ticks
float vcaDTime;
// ticks per D envelope phase
int vcaDTicks;
// VCA D amplitude change per tick
float vcaDChangePerTick;

// VCF D value
// this is a time value
// set the default here; 0 - 127
byte vcfDValue = 40;
// VCF D time in ticks
long vcfDTime;
// ticks per D envelope phase
long vcfDTicks;
// VCF D cutoff change per tick
float vcfDChangePerTick;

// phase S
// VCA S value
// this is a level value
// set the default here; 0 - 127
byte vcaSValue = 127;
// VCA S volume
float vcaSVolume;

// VCF S value
// this is a level value
// set the default here; 0 - 127
byte vcfSValue = 0;
// VCF S cutoff level
float vcfSLevel;

// phase R
// VCA R value
// this is a time value
// set the default here; 0 - 127
byte vcaRValue = 40;
// VCA R time
long vcaRTime;
// ticks per R envelope phase
int vcaRTicks;
// R amplitude change per tick
float vcaRChangePerTick;

// VCF R value
// this is a time value
// set the default here; 0 - 127
// NOTE:
// the MIDI CC for VCA R will always be used for VCF R
byte vcfRValue = 40;
// VCF R time
long vcfRTime;
// ticks per R envelope phase
long vcfRTicks;
// R VCF cutoff change per tick
float vcfRChangePerTick;

// VCF depth
// reduces the VCF envelope amplitude from maximum
// set the default here; 0 - 127
byte vcfDepthValue = 127;
// VCF depth setting
float vcfDepth;
// VCF resonance setting
// we are using a 7-bit digital pot so this value can be used directly
byte vcfResonanceValue = 0;


// VCA tick counter
int vcaTickCount = 0;

// VCF tick counter
long vcfTickCount = 0;

// flag if last note playing
byte wasLastNote = 0;

// button setup
Bounce channelButton = Bounce();

// set up the DAC
DAC_MCP49xx dac(DAC_MCP49xx::MCP4921, DAC_CS_PIN);


//---------------------------------------------------------------------------------------------//
// setup
//---------------------------------------------------------------------------------------------//
void setup() {
  
  // set up the fast pwm timer
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  initialize(clock);  // Frequency to generate in Hz
  setPwmDuty(512);     // Duty cycle 0-1024, 1024 is 100%, 512 is 50%
  start();
  
  // set the SPI frequency to 1 mHz
  dac.setSPIDivider(SPI_CLOCK_DIV16);
  
  // enable portf
  // just in case PJRC does not set the fuses
  MCUCR |= _BV(JTD);
  
  // set up the CS pin for the pot
  pinMode(POT_CS_PIN, OUTPUT);
  digitalWrite(POT_CS_PIN, HIGH);
  // set the pot to zero
  digitalWrite(POT_CS_PIN, LOW);
  SPI.transfer(POT_0);
  SPI.transfer(SET_ZERO);
  digitalWrite(POT_CS_PIN, HIGH);

  // set up the LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // set up the button
  pinMode(CHANNEL_SET_PIN, INPUT);
  digitalWrite(CHANNEL_SET_PIN, HIGH);
  channelButton.attach(CHANNEL_SET_PIN);
  channelButton.interval(DEBOUNCE_MS);

  // read the EEPROM checkbytes
  if ((EEPROM.read(EE_CHECKBYTE_1) == 0xBA) && (EEPROM.read(EE_CHECKBYTE_2) == 0xDA))
  {
    // read the MIDI channel from EEPROM
    myChannel = EEPROM.read(EE_MIDI_CHANNEL);
  }
  else
  {
    // set the checkbytes
    EEPROM.write(EE_CHECKBYTE_1, 0xBA);
    EEPROM.write(EE_CHECKBYTE_2, 0xDA);
    // the MIDI channel will default to 1
    EEPROM.write(EE_MIDI_CHANNEL, myChannel);
  }

  // MIDI setup
  MIDI.begin(myChannel);  
  MIDI.setHandleNoteOn(doNoteOn); 
  MIDI.setHandleControlChange(doCC); 
  MIDI.setHandleNoteOff(doNoteOff); 
  MIDI.setHandlePitchBend(doBend);
  
  // set up the multiplexer address lines
  pinMode(MUX_A0_PIN, OUTPUT);
  pinMode(MUX_A1_PIN, OUTPUT);
  // default to both low (lowpass mode)
  digitalWrite(MUX_A0_PIN, LOW);
  digitalWrite(MUX_A1_PIN, LOW);

  pinMode(wr, OUTPUT);
  
  // use upper nibble of PORTB for lower nibble data output to SN74689
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
//  DDRB = DDRB | B11110000; // set the direction for 
  
  // use upper nibble of PORTF for upper nibble data output to SN74689
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
//  DDRF = DDRF | B11110000;

  // initialize the notes struct
  for (int j = 0; j < (MAX_NOTES + 1); j++)
  {
    notes[j].myPitch = 0;
    notes[j].myVelocity = 0;
  }

  // turn all oscillators off
  writeAmplitude(0, 1);
  writeAmplitude(0, 2);
  writeAmplitude(0, 3);
  writeAmplitude(0, 4);

  // get the first time for AM modulation period
  amTimer = millis();
  // get the first time for the VCA enevlope tick
  vcaTickTimer = millis();
  // get the first time for the glide tick
  glideTimer = millis();
  // get the first time for the VCF envelope tick
  vcfTickTimer = micros();

}


//---------------------------------------------------------------------------------------------//
// main loop
//---------------------------------------------------------------------------------------------//
void loop()
{
  // get MIDI notes if available
  MIDI.read();

  // handle AM modulation
  // each time we reach the amPeriod
  // flip the on/off value
  if ((millis() - amTimer) >= amPeriod)
  {
    // call the function
    doAM();

    // restart the AM timer
    amTimer = millis();
  }

  // each time the VCA envelope tick timer expires
  if ((millis() - vcaTickTimer) >= VCA_ENVELOPE_TICK_TIME)
  {
    // service the VCA envelope
    doVCA();
    // restart the tick timer
    vcaTickTimer = millis();
  }
  
  // each time the VCF envelope tick timer expires
  if ((micros() - vcfTickTimer) >= VCF_ENVELOPE_TICK_TIME)
  {
    // service the VCF envelope
    doVCF();
    // restart the tick timer
    vcfTickTimer = micros();
  }

  // each time the glide tick timer expires
  if ((millis() - glideTimer) >= GLIDE_TICK_TIME)
  {
    // glide if enabled by glideNote
    if (glideNote == 1)
    {
      doGlide();
    }
    // restart the timer
    glideTimer = millis();
  }

  // set the MIDI channel if the button is pressed
  if (channelButton.update())
  {
    if (channelButton.read() == LOW)
    {
      setMIDIChannel();
    }
  }


  // doVib();
}

//---------------------------------------------------------------------------------------------//
// function doNoteOn
//---------------------------------------------------------------------------------------------//
void doNoteOn(byte channel, byte pitch, byte velocity)
{
  // note:
  // new notes can happen three ways:
  // - a fresh note from no other keys down
  // - a note interrupting a playing note
  // - a note from the stack after a noteOff

  // if the last note got interrupted in STATE_R
  // clear the note so we don't copy it again
  if (wasLastNote == 1)
  {
    wasLastNote = 0;
    notes[0].myPitch = 0;
    notes[0].myVelocity = 0;
  }

  // test if it is a note interrupting a currently-playing note
  if (notes[0].myVelocity != 0)
  {
    // set the glide flag
    glideNote = 1;
  }
  else
  {
    glideNote = 0;
  }

  // only push onto the stack if a new noteOn
  // don't do it if the note is from the stack
  if (wasFromNoteOff == 0)
  {
    // push the new note onto the stack
    // first push down the stack
    for (int r = (MAX_NOTES - 2); r >= 0; r--)
    {
      notes[r + 1].myPitch = notes[r].myPitch;
      notes[r + 1].myVelocity = notes[r].myVelocity;
    }
    // then set the first element to the new note
    notes[0].myPitch = pitch;
    notes[0].myVelocity = velocity;
  }
  // otherwise it was from noteOff
  // so unset the flag
  else
  {
    // set the glide flag
    glideNote = 1;
    // unset the fromNoteOff flag
    wasFromNoteOff = 0;
  }


  // start the VCA envelope
  vcaEnvState = STATE_A;

  // set the envelope volume to zero
  vcaVolume = 0.0;
  
  // start the VCF envelope
  vcfEnvState = STATE_A;
  
  // set the VCF envelope cutoff to zero
  vcfCutoff = 0;
  // set vcfCutoff to max level
  // vcfCutoff = MIN_VCF_CUTOFF;
  // compute the actual value to send to the DAC after multiplying by the depth factor
  // vcfCutoffOutput = (int)(MIN_VCF_CUTOFF - ((vcfDepthValue / 127.0f) * vcfCutoff));
  // write the value to the DAC
  dac.output(MIN_VCF_CUTOFF);
  
  // set the VCF resonance digital pot
  digitalWrite(POT_CS_PIN, LOW);
  delayMicroseconds(COMMAND_DELAY);
  SPI.transfer(POT_0);
  SPI.transfer(vcfResonanceValue);
  
  digitalWrite(POT_CS_PIN, HIGH);

  // oscillator volume adjustments  
  // osc1
  v1 = osc1Level / 127.0f;
  // osc2
  v2 = osc2Level / 127.0f; 
  // osc3  
  v3 = osc3Level / 127.0f; 
  // osc4/noise  
  v4 = osc4Level / 127.0f;

  // calculate the raw frequency values for gliding
  rawFrequency(pitch, 1, 64, 64);
  rawFrequency(pitch, 2, osc2Octave, osc2Detune);
  rawFrequency(pitch, 3, osc3Octave, osc3Detune);

  // glide if legato playing and glide time not zero  
  if ((glideNote == 1) && (glideTimeValue != 0))
  {
    writeNoisePitch(6);
    // calculate the glide parameters
    glideTime = (glideTimeValue / 127.0) * MAX_GLIDE_TIME;
    glideTicks = glideTime / GLIDE_TICK_TIME;
    glideChangePerTick1 = (osc1F2 - (float)osc1F1) / (float)glideTicks;
    glideChangePerTick2 = (osc2F2 - (float)osc2F1) / (float)glideTicks;
    glideChangePerTick3 = (osc3F2 - (float)osc3F1) / (float)glideTicks;
    // set the glide tick counter to zero
    glideTickCount = 0;
  }
  else
  {
    // set the oscillator frequencies
    writeFrequency(pitch, 1, 64, 64);
    writeFrequency(pitch, 2, osc2Octave, osc2Detune);
    writeFrequency(pitch, 3, osc3Octave, osc3Detune);
    writeNoisePitch(6);
    // unset the glide flag
    glideNote = 0;
  }

  // calculate the envelope stuff

  /* Here's how it works:
   We do the VCA like manually turning the volume control on a stereo. The volume is a gain factor from 0 to 1.
   This is applied on top of each oscillators sub-volume level adjustment. So, if an oscillator is set to 0.5 sub-volume
   and the A phase level is 127 (gain of 1.0), then the oscillator will end up at 0.5 gain.
   
   We do the VCF by tracking up to and down from the maxiumum cutoff frequency value. If tracking is enabled,
   it is subtracted from the maxiumum cutoff frequency based on the received note.
   A gain adjustment is then applied as vcfDepth.
   
   For each envelope phase, we calculate the overall change in volume setting and how much to change it per
   each envelope timer tick.
   */

  // VCA attack time
  // attack is the time it takes to go from zero to the received velocity
  // how long based on the CC and the maximum envelope time
  vcaATime = (vcaAValue / 127.0f) * (VCA_MAX_ENV_TIME / 4);
  // how many ticks it will take
  vcaATicks = (int)(vcaATime / VCA_ENVELOPE_TICK_TIME);
  // gain at the end of A
  //  vcaAVolume = (float)notes[0].myVelocity / 127.0;
  vcaAVolume = 1.0f;
  // how much change in gain per tick
  vcaAChangePerTick = vcaAVolume / vcaATicks;
  
  // VCF attack time
  // attack is the time it takes to go from zero to maxiumum cutoff
  // how long based on the CC and the maximum envelope time
  vcfATime = (vcfAValue / 127.0f) * (VCF_MAX_ENV_TIME);
  // how many ticks it will take
  vcfATicks = vcfATime / VCF_ENVELOPE_TICK_TIME;
  // how much change in cutoff per tick
  vcfAChangePerTick = MIN_VCF_CUTOFF / (float)vcfATicks;

  // VCA sustain value
  // sustain is the level the envelope drops to after attack
  vcaSVolume = vcaSValue / 127.0f;
  
  // VCF sustain value
  // sustain is the level the envelope drops to after attack
  vcfSLevel = (vcfSValue / 127.0f) * MIN_VCF_CUTOFF;

  // VCA decay time
  // decay is the time is takes to go from the received velocity to the sustain level
  // how long based on the CC and the maxiumum envelope time
  vcaDTime = (vcaDValue / 127.0f) * (VCA_MAX_ENV_TIME / 4);
  // how many ticks it will take
  vcaDTicks = (int)(vcaDTime / VCA_ENVELOPE_TICK_TIME);
  // how much change in gain per tick
  // from A volume to S volume
  vcaDChangePerTick = (vcaAVolume - vcaSVolume) / (float)vcaDTicks;
  
  // VCF decay time
  // decay is the time it takes to go from the maxiumum cutoff frequency to the sustain level
  // how long based on the CC and the maxiumum envelope time
  vcfDTime = (vcfDValue / 127.0f) * (VCF_MAX_ENV_TIME);
  // how many ticks it will take
  vcfDTicks = vcfDTime / VCF_ENVELOPE_TICK_TIME;
  // how mucn change in cutoff per tick
  vcfDChangePerTick = (MIN_VCF_CUTOFF - vcfSLevel) / vcfDTicks;

  // VCA release time
  // release is the time it takes to go from sustain to zero
  // how long based on the CC and the maxiumum envelope time
  vcaRTime = (vcaRValue / 127.0f) * VCA_MAX_ENV_TIME;
  // how many ticks will it take
  vcaRTicks = (int)(vcaRTime / VCA_ENVELOPE_TICK_TIME);
  // how much change in gain per tick
  // from S volume to zero
  vcaRChangePerTick = vcaSVolume / vcaRTicks;
  
  // VCF release time
  // release is the time it takes to go from sustain to zero
  // how long based on the CC and the maxiumum envelope time
  vcfRTime = (vcfRValue / 127.0f) * VCF_MAX_ENV_TIME;
  // how many ticks will it take
  vcfRTicks = vcfRTime / VCF_ENVELOPE_TICK_TIME;
  // how much change in cutoff per tick
  // from sustain level to zero
  vcfRChangePerTick = vcfSLevel / vcfRTicks;

  // set the tick counter to zero
  // VCA tick counter
  vcaTickCount = 0;
  // VCF tick counter
  vcfTickCount = 0;

}

//---------------------------------------------------------------------------------------------//
// function doNoteOff
//---------------------------------------------------------------------------------------------//
void doNoteOff(byte channel, byte pitch, byte velocity)
{
  byte r = 0;
  byte i;
  byte noteIndex = 0;
  byte noteFound = 0;
  byte currentVelocity = 0;

  // find the note in the stack
  while (noteFound != 1)
  {
    // note found
    if (notes[r].myPitch == pitch)
    {
      noteFound = 1;
      noteIndex = r;
    }
    r++;
  }

  // save the velocity of the current note before it gets overwritten
  currentVelocity = notes[0].myVelocity;

  // shift the remaining notes over left by one
  // go up to max notes to grab an empty note
  for (i = noteIndex; i < (MAX_NOTES + 1); i++)
  {
    // copy the pitch
    notes[i].myPitch = notes[i + 1].myPitch;
    // copy the velocity
    notes[i].myVelocity = notes[i + 1].myVelocity;
  }

  // if the note was found at zero (current playing note)
  // and the note copied to is has a non-zero pitch
  // play the note
  if ((noteIndex == 0) && (notes[0].myPitch > 0))
  {
    // flag the note as coming from noteOff
    wasFromNoteOff = 1;
    // reset the tick counter
    vcaTickCount = 0;
    vcfTickCount = 0;
    // set the VCF resonance digital pot to zero
    // to mute any self-oscillation
    digitalWrite(POT_CS_PIN, LOW);
    delayMicroseconds(COMMAND_DELAY);
    SPI.transfer(POT_0);
    SPI.transfer(0);
    delayMicroseconds(COMMAND_DELAY);
    digitalWrite(POT_CS_PIN, HIGH);
    // set the cutoff to minimum
    dac.output(MIN_VCF_CUTOFF);
    // handle the note - the vca and vcf envelopes will be restarted
    doNoteOn(channel, notes[0].myPitch, notes[0].myVelocity);
  }
  // if the note found at zero
  // and the note copied has a zero pitch
  // the last note has matched the noteOff
  if ((noteIndex == 0) && (notes[0].myPitch == 0))
  {
    // reset the tick counter
    vcaTickCount = 0;
    vcfTickCount = 0;
    // put the note back so it can be played from STATE_R
    notes[0].myPitch = pitch;
    notes[0].myVelocity = currentVelocity;
    // flag last note
    wasLastNote = 1;
    // unset the glide flag
    glideNote = 0;
    // update the glide raw frequencies from start to end
    // might not be necessary here
    osc1F2 = osc1F1;
    osc2F2 = osc2F1;
    osc3F2 = osc3F1;
    // move to the vca release state
    vcaEnvState = STATE_R;
    // move to the vcf rekease state
    vcfEnvState = STATE_R;
  }

  // otherwise an earlier not matched noteOff
  // and there is nothing to do
}

//---------------------------------------------------------------------------------------------//
// function doCC
//---------------------------------------------------------------------------------------------//
void doCC(byte channel, byte ccnumber, byte ccvalue)
{
  switch (ccnumber)
  {

    // MIDI CC 1 = AM Depth (mod wheel)
    case 1:
      amDepth = ccvalue;
      amVolume = (127 - amDepth) / 127.0f;
      break;

    // MIDI CC 18 = AM Rate
    case 18:
      amRate = ccvalue;
      // avoid calculating amPeriod of zero
      if (amRate == 127)
      {
        amRate = 126;
      }
      amPeriod = AM_MAX_TIME * ((127.0f - amRate) / 127.0f);
      break;
  
    // MIDI CC 19 = oscillator 1 level
    case 19:
      osc1Level = ccvalue;
      // osc1
      v1 = osc1Level / 127;
      writeAmplitude((byte)(v1 * vcaVolume * notes[0].myVelocity), 1);
      break;
  
    // MIDI CC 20 = Vib Speed
    case 20:
      vibSpeed = ccvalue; 
      break;
  
    // MIDI CC 21 = Vib Depth
    case 21:
      vibDepth = ccvalue; 
      break;
  
    // MIDI CC 22 = oscillator 2 level
    case 22:
      osc2Level = ccvalue;
      // osc2
      v2 = osc2Level / 127;
      writeAmplitude((byte)(v2 * vcaVolume * notes[0].myVelocity), 2);
      break;
  
    // MIDI CC 23 = oscillator 2 octave
    case 23:
      osc2Octave = ccvalue;
      // osc2
      writeFrequency(notes[0].myPitch, 2, osc2Octave, osc2Detune);
      break;
  
    // MIDI CC 24 = oscillator 2 detune
    case 24:
      osc2Detune = ccvalue;
      // osc2
      writeFrequency(notes[0].myPitch, 2, osc2Octave, osc2Detune);
      break;
  
    // MIDI CC 25 = oscillator 3 level
    case 25:
      osc3Level = ccvalue;
      // osc3
      v3 = osc3Level / 127;
      writeAmplitude((byte)(v3 * vcaVolume * notes[0].myVelocity), 3);
      break;
  
    // MIDI CC 26 = oscillator 3 octave
    case 26:
      osc3Octave = ccvalue;
      // osc3
      writeFrequency(notes[0].myPitch, 3, osc3Octave, osc3Detune);
      break;
  
    // MIDI CC 27 = oscillator 3 detune
    case 27:
      osc3Detune = ccvalue;
      // osc3
      writeFrequency(notes[0].myPitch, 3, osc3Octave, osc3Detune);
      break;
  
    // MIDI CC 28 = oscillator 4 level
    case 28:
      osc4Level = ccvalue;
      // osc2
      v4 = osc4Level / 127;
      writeAmplitude((byte)(v4 * vcaVolume * notes[0].myVelocity), 4);
      break;  
  
  
    // MIDI CC 42 = coarse tuning
    case 42:
      coarsePitch = ccvalue; 
      writeFrequency(coarsePitch, 1, 64, 64);
      writeFrequency(coarsePitch, 2, osc2Octave, osc2Detune);
      writeFrequency(coarsePitch, 3, osc3Octave, osc3Detune);
      break;
  
    // MIDI CC 71 = VCF resonance
    case 71:
      vcfResonanceValue = ccvalue;
      // update the VCF resonance digital pot
      // only if a note is playing
      // otherwise we might send the filter into self-oscillation
      // when there is no note playing
      if (notes[0].myPitch > 0)
      {
        digitalWrite(POT_CS_PIN, LOW);
        delayMicroseconds(COMMAND_DELAY);
        SPI.transfer(POT_0);
        SPI.transfer(vcfResonanceValue);
        delayMicroseconds(COMMAND_DELAY);
        digitalWrite(POT_CS_PIN, HIGH);
      }
      break;
  
    // MIDI CC 72 = VCA release value
    // also serves as VCF release time
    case 72:
      vcaRValue = ccvalue;
      vcfRTime = (vcaRValue / 127.0f) * VCF_MAX_ENV_RELEASE_TIME;
      break;
  
    // MIDI CC 73 = VCA attack value
    case 73:
      vcaAValue = ccvalue;
      vcaATime = (vcaAValue / 127.0f) * VCA_MAX_ENV_TIME;
      break;
  
    // MIDI CC 74 = VCF cutoff
    case 74:
      vcfDepthValue = ccvalue;
      // VCF cutoff CV is inverse of setting, ie. max voltage is minimum cutoff (lowest frequency)
      // we want maxiumum depth to have zero attenuation of the VCF cutoff envelope
      vcfCutoffOutput = MIN_VCF_CUTOFF - (int)((float)(vcfDepthValue / 127.00) * vcfCutoff);
      // write an updated value to the VCF cutoff DAC based on the update value
      dac.output(vcfCutoffOutput);
      break;

    // MIDI CC 75 = VCF attack value
    case 75:
      vcfAValue = ccvalue;
      vcfATime = (vcfAValue / 127.0f) * VCF_MAX_ENV_TIME;
      break;
  
    // MIDI CC 76 = VCF decay value
    case 76:
      vcfDValue = ccvalue;
      vcfDTime = (vcfDValue / 127.0f) * VCF_MAX_ENV_TIME;
      break;
     
    // MIDI CC 77 = VCF sustain value
    case 77:
      vcfSValue = ccvalue;
      break; 
  
    // MIDI CC 85 = VCA decay value
    case 85:
      vcaDValue = ccvalue;
      vcaDTime = (vcaDValue / 127.0f) * VCA_MAX_ENV_TIME;
      break;
  
    // MIDI CC 86 = VCA sustian value
    // also serves as
    case 86:
      vcaSValue = ccvalue;
      break;
  
    // MIDI CC 87 = glide time value
    case 87:
      glideTimeValue = ccvalue;      
      break;
      
    // MIDI CC 88 = filter mode
    // 0 - 42 lowpass
    // 43 - 84 bandpass
    // 85 - 127 highpass
    case 88:
      filterModeValue = ccvalue;
      if (filterModeValue < 34)
      {
        setFilterMode(0);
      }
      else if ((filterModeValue > 33) && (filterModeValue < 91))
      {
        setFilterMode(1);
      }
      else if (filterModeValue > 90)
      {
        setFilterMode(2);
      }
      break;
  
    // default
    default:
      break;
  }
}


//---------------------------------------------------------------------------------------------//
// function doBend
//---------------------------------------------------------------------------------------------//
void doBend(byte channel, int bend)
{
  bend_data = (float) bend;
  // osc1
  writeFrequency(notes[0].myPitch, 1, 64, 64);
  // osc2
  writeFrequency(notes[0].myPitch, 2, osc2Octave, osc2Detune);
  // osc3
  writeFrequency(notes[0].myPitch, 3, osc3Octave, osc3Detune);  
}

//---------------------------------------------------------------------------------------------//
// function doVCA
//---------------------------------------------------------------------------------------------//
void doVCA(void)
{
  // handle the VCA envelope states
  switch (vcaEnvState)
  {
    // envelope not active
  case STATE_0:
    // nothing happens in this state
    return;
    break;

    // attack state
  case STATE_A:
    // if the vca attack is zero, go right to the velocity
    if (vcaAValue == 0)
    {
      // osc1
      writeAmplitude((byte)(v1 * amVolume * notes[0].myVelocity), 1);
      // osc2
      writeAmplitude((byte)(v2 * notes[0].myVelocity), 2);
      // osc3
      writeAmplitude((byte)(v3 * notes[0].myVelocity), 3);
      // osc4
      writeAmplitude((byte)(v4 * notes[0].myVelocity), 4);
      // set vcaVolume equal to the vcaAVolume
      vcaVolume = vcaAVolume;
      // reset the vcaTickCount to zero
      vcaTickCount = 0;
      // move to the next state
      vcaEnvState = STATE_D;
      return;
    }

    // set the osc volumes on each pass      
    // osc1
    writeAmplitude((byte)(v1 * vcaVolume * amVolume * notes[0].myVelocity), 1);
    // osc2
    writeAmplitude((byte)(v2 * vcaVolume * amVolume * notes[0].myVelocity), 2);
    // osc3
    writeAmplitude((byte)(v3 * vcaVolume * amVolume * notes[0].myVelocity), 3);
    // osc4
    writeAmplitude((byte)(v4 * vcaVolume * amVolume * notes[0].myVelocity), 4);

    // increment the vcaVolume
    vcaVolume = vcaVolume + vcaAChangePerTick;

    // catch rounding errors
    if (vcaVolume > 1.0)
    {
      vcaVolume = 1.0;
    }

    // increment the vcaTickCount
    vcaTickCount++;

    // end of the envelope state
    if (vcaTickCount > vcaATicks)
    {
      // reset the vcaTickCount to zero
      vcaTickCount = 0;
      // move to the next state
      vcaEnvState = STATE_D;
    }
    return;
    break;

    // decay state
  case STATE_D:
    // if the vca decay is zero, go right to the sustain level
    if (vcaDValue == 0)
    {
      // osc1
      writeAmplitude((byte)(v1 * vcaSVolume * amVolume * notes[0].myVelocity), 1);
      // osc2
      writeAmplitude((byte)(v2 * vcaSVolume * amVolume * notes[0].myVelocity), 2);
      // osc3
      writeAmplitude((byte)(v3 * vcaSVolume * amVolume * notes[0].myVelocity), 3);
      // osc4
      writeAmplitude((byte)(v4 * vcaSVolume * amVolume * notes[0].myVelocity), 4);
      // set the vcaVolume to vcaSVolume * velocity
      vcaVolume = vcaSVolume;
      // reset the vcaTickCount to zero
      vcaTickCount = 0;
      // move to the next state
      vcaEnvState = STATE_D;
      return;
    }

    // set the osc volumes on each pass    
    // osc1
    writeAmplitude((byte)(v1 * vcaVolume * amVolume * notes[0].myVelocity), 1);
    // osc2
    writeAmplitude((byte)(v2 * vcaVolume * amVolume * notes[0].myVelocity), 2);
    // osc3
    writeAmplitude((byte)(v3 * vcaVolume * amVolume * notes[0].myVelocity), 3);
    // osc4
    writeAmplitude((byte)(v4 * vcaVolume * amVolume * notes[0].myVelocity), 4);

    // decrement the vcaVolume
    vcaVolume = vcaVolume - vcaDChangePerTick;

    // increment the vcaTickCount
    vcaTickCount++;

    // end of the envelope state
    if (vcaTickCount > vcaATicks)
    {
      // reset the vcaTickCount to zero
      vcaTickCount = 0;

      // force the volume to the vcaSVolume
      vcaVolume = vcaSVolume;

      // osc1
      writeAmplitude((byte)(v1 * vcaVolume * amVolume * notes[0].myVelocity), 1);
      // osc2
      writeAmplitude((byte)(v2 * vcaVolume * amVolume * notes[0].myVelocity), 2);
      // osc3
      writeAmplitude((byte)(v3 * vcaVolume * amVolume * notes[0].myVelocity), 3);
      // osc4
      writeAmplitude((byte)(v4 * vcaVolume * amVolume * notes[0].myVelocity), 4);

      // move to the next state
      vcaEnvState = STATE_S;
    }
    return;
    break;

    // sustain state
  case STATE_S:
    // stay in sustain until noteOff
    return;
    break;

    // release state
  case STATE_R:
    // if the vca release is zero, set the oscillators to zero
    if (vcaRValue == 0)
    {
      // osc1
      writeAmplitude(0, 1);
      // osc2
      writeAmplitude(0, 2);
      // osc3
      writeAmplitude(0, 3);
      // osc4
      writeAmplitude(0, 4);
      // reset the vcaTickCount to zero
      vcaTickCount = 0;
      // set glideNote to zero
      glideNote = 0;
      // set the VCF resonance digital pot to zero resonance
      // this is to mute the filter if it is self-oscillating
      digitalWrite(POT_CS_PIN, LOW);
      delayMicroseconds(COMMAND_DELAY);
      SPI.transfer(POT_0);
      SPI.transfer(0);
      delayMicroseconds(COMMAND_DELAY);
      digitalWrite(POT_CS_PIN, HIGH);
      // set the cutoff to zero
      vcfCutoff = 0;
      // set the DAC to the lowest cutoff frequency
      dac.output(MIN_VCF_CUTOFF);
      // move to the default state
      vcaEnvState = STATE_0;
      return;
    }

    // set the osc volumes on each pass    
    // osc1
    writeAmplitude((byte)(v1 * vcaVolume * amVolume * notes[0].myVelocity), 1);
    // osc2
    writeAmplitude((byte)(v2 * vcaVolume * amVolume * notes[0].myVelocity), 2);
    // osc3
    writeAmplitude((byte)(v3 * vcaVolume * amVolume * notes[0].myVelocity), 3);
    // osc4
    writeAmplitude((byte)(v4 * vcaVolume * amVolume * notes[0].myVelocity), 4);

    // decrement the vcaVolume
    vcaVolume = vcaVolume - vcaRChangePerTick;
    // increment the vcaTickCount
    vcaTickCount++;

    // end of the envelope state
    if (vcaTickCount > vcaRTicks)
    {
      // clear the current note
      notes[0].myPitch = 0;
      notes[0].myVelocity = 0;
      wasLastNote = 0;
      // make sure all oscillators are off
      // osc1
      writeAmplitude(0, 1);
      // osc2
      writeAmplitude(0, 2);
      // osc3
      writeAmplitude(0, 3);
      // osc4
      writeAmplitude(0, 4);
      // reset the vcaTickCount to zero
      vcaTickCount = 0;
      // set glideNote to zero
      glideNote = 0;
      // set the VCF resonance digital pot to zero resonance
      // this is to mute the filter if it is self-oscillating
      digitalWrite(POT_CS_PIN, LOW);
      delayMicroseconds(COMMAND_DELAY);
      SPI.transfer(POT_0);
      SPI.transfer(0);
      delayMicroseconds(COMMAND_DELAY);
      digitalWrite(POT_CS_PIN, HIGH);
      // set the cutoff to zero
      vcfCutoff = 0;
      // set the DAC to the lowest cutoff frequency
      dac.output(MIN_VCF_CUTOFF);
      // move to the default state
      vcaEnvState = STATE_0;
    }
    return;
    break;

    // default
  default:
    return;
    break;
  }
}

//---------------------------------------------------------------------------------------------//
// function doVCF
// note:
//  VCF cutoff response is lowest filter frequency at about 2.5v or value 2048 to the DAC and
//  highest filter frequency is a 0v or value 0 to the DAC.
//  
//  We will handle the envelope values internally the opposite way; lowest frequency at 0 and
//  highest frequency at 2048. We will convert the value before sending it to the DAC.
//---------------------------------------------------------------------------------------------//
void doVCF(void)
{
  // handle the VCF envelope states
  switch (vcfEnvState)
  {
    // envelope not active
    case STATE_0:
      // nothing happens in this state
      return;
      break;
  
    // attack state
    case STATE_A:
      // if the vcf attack is zero, go right to the max cutoff
      if (vcfAValue == 0)
      {
        // set vcfCutoff to max level
        vcfCutoff = MIN_VCF_CUTOFF;
        // compute the actual value to send to the DAC after multiplying by the depth factor
        vcfCutoffOutput = (int)(MIN_VCF_CUTOFF - ((vcfDepthValue / 127.0f) * vcfCutoff));
        // write the value to the DAC
        dac.output(vcfCutoffOutput);
        // reset the vcaTickCount to zero
        // vcfTickCount = 0;
        // move to the next state
        vcfEnvState = STATE_D;
        return;
      }
  
      // set the VCF cutoff on each pass
      // apply the cutoff depth and invert the values    
      vcfCutoffOutput = MIN_VCF_CUTOFF - (int)((vcfDepthValue / 127.0f) * vcfCutoff);
      // write the value to the DAC
      dac.output(vcfCutoffOutput);
  
      // increment the vcfCutoff
      vcfCutoff = vcfCutoff + vcfAChangePerTick;
  
      // catch rounding errors
      if (vcfCutoff >= MIN_VCF_CUTOFF)
      {
        vcfCutoff = MIN_VCF_CUTOFF;
        // vcfTickCount = vcfATicks;
        // move to the next state
        vcfEnvState = STATE_D;
      }
  
      // increment the vcaTickCount
      // vcfTickCount++;
  
      // end of the envelope state
//      if (vcfTickCount > vcfATicks)
//      {
//        // reset the vcaTickCount to zero
//        vcfTickCount = 0;
//        // move to the next state
//        vcfEnvState = STATE_D;
//      }
      return;
      break;
  
    // decay state
    case STATE_D:
      // if the vca decay is zero, go right to the sustain level
      if (vcfDValue == 0)
      {
        // set the cutoff equal to the sustain level
        vcfCutoff = vcfSLevel;
        // apply the cutoff depth and invert the values    
        vcfCutoffOutput = (int)(MIN_VCF_CUTOFF - ((vcfDepthValue / 127.0f) * vcfCutoff));
        // write the value to the DAC
        dac.output(vcfCutoffOutput);
        // reset the vcaTickCount to zero
        // vcfTickCount = 0;
        // move to the next state
        vcfEnvState = STATE_S;
        return;
      }
  
      // set the VCF cutoff on each pass    
      // apply the cutoff depth and invert the values    
      vcfCutoffOutput = MIN_VCF_CUTOFF - (int)((vcfDepthValue / 127.0f) * vcfCutoff);
      // write the value to the DAC
      dac.output(vcfCutoffOutput);
  
      // decrement the vcfCutoff
      vcfCutoff = vcfCutoff - vcfDChangePerTick;
  
      // increment the vcaTickCount
      // vcfTickCount++;
  
      // end of the envelope state
      if (vcfCutoff <= vcfSLevel)
      {
        // reset the vcaTickCount to zero
        // vcfTickCount = 0;
  
        // force the volume to the vcaSVolume
        if (vcfSLevel == 0)
        {
          vcfCutoff = 0;
        }
        else
        {
          vcfCutoff = vcfSLevel;
        }
        
        // apply the cutoff depth and invert the values    
        vcfCutoffOutput = MIN_VCF_CUTOFF - (int)((vcfDepthValue / 127.0f) * vcfCutoff);
        // write the value to the DAC
        dac.output(vcfCutoffOutput);
  
        // move to the next state
        vcfEnvState = STATE_S;
      }
      return;
      break;
  
    // sustain state
    case STATE_S:
      // stay in sustain until noteOff
      return;
      break;
  
    // release state
    case STATE_R:
    
      // if the vcf release is zero or sustain level is zero, set cutoff back to zero
      if ((vcfRValue == 0) || (vcfSValue == 0) || (vcfCutoff == 0))
      {
        // set the VCF resonance digital pot to zero resonance
        // this is to mute the filter if it is self-oscillating
        digitalWrite(POT_CS_PIN, LOW);
        delayMicroseconds(COMMAND_DELAY);
        SPI.transfer(POT_0);
        SPI.transfer(0);
        delayMicroseconds(COMMAND_DELAY);
        digitalWrite(POT_CS_PIN, HIGH);
        // set the DAC to the lowest cutoff frequency
        dac.output(MIN_VCF_CUTOFF);
        // move to the default state
        vcfEnvState = STATE_0;
        vcaTickCount = vcaRTicks;
        return;
      }
  
      // set the VCF cutoff on each pass    
      // apply the cutoff depth and invert the values    
      vcfCutoffOutput = MIN_VCF_CUTOFF - (int)((vcfDepthValue / 127.0f) * vcfCutoff);
      // write the value to the DAC
      dac.output(vcfCutoffOutput);
  
      // decrement the vcfCutoff
      vcfCutoff = vcfCutoff - vcfRChangePerTick;
      
      // catch rounding errors
      if (vcfCutoff < 0)
      {
        vcfCutoff = 0;
      }
  
      // increment the vcaTickCount
      // vcfTickCount++;0
  
      // end of the envelope state
      if (vcfCutoff == 0)
      {
        // set the VCF resonance digital pot to zero resonance
        // this is to mute the filter if it is self-oscillating
        digitalWrite(POT_CS_PIN, LOW);
        delayMicroseconds(COMMAND_DELAY);
        SPI.transfer(POT_0);
        SPI.transfer(0);
        delayMicroseconds(COMMAND_DELAY);
        digitalWrite(POT_CS_PIN, HIGH);
        // set the DAC to the lowest cutoff frequency
        dac.output(MIN_VCF_CUTOFF);
        // move to the default state
        vcfEnvState = STATE_0;
      }
      return;
      break;
  
      // default
    default:
      return;
      break;
  }  
}

//---------------------------------------------------------------------------------------------//
// function writeSN76489
//---------------------------------------------------------------------------------------------//
void writeSN76489(byte data) {
  byte mybit = 0;
  // lower data nibble to upper PORTB nibble
  mybit = (data & B00001000);
  if (mybit)
  {
    digitalWrite(4, HIGH);
  }
  else
  {
    digitalWrite(4, LOW);
  }
  mybit = (data & B00000100);
  if (mybit)
  {
    digitalWrite(15, HIGH);
  }
  else
  {
    digitalWrite(15, LOW);
  }
  mybit = (data & B00000010);
  if (mybit)
  {
    digitalWrite(14, HIGH);
  }
  else
  {
    digitalWrite(14, LOW);
  }
  mybit = (data & B00000001);
  if (mybit)
  {
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }
//  PORTB = PINB | (data << 4);
//  PORTB = PORTB | (data << 4);
  // upper data nibble to upper PORTF nibble
  mybit = (data & B10000000);
  if (mybit)
  {
    digitalWrite(16, HIGH);
  }
  else
  {
    digitalWrite(16, LOW);
  }
  mybit = (data & B01000000);
  if (mybit)
  {
    digitalWrite(17, HIGH);
  }
  else
  {
    digitalWrite(17, LOW);
  }
  mybit = (data & B00100000);
  if (mybit)
  {
    digitalWrite(18, HIGH);
  }
  else
  {
    digitalWrite(18, LOW);
  }
  mybit = (data & B00010000);
  if (mybit)
  {
    digitalWrite(19, HIGH);
  }
  else
  {
    digitalWrite(19, LOW);
  }
//  PORTF = PINF | (data & B11110000);
//  PORTF = PORTF | (data & B11110000);
  // delayMicroseconds(COMMAND_DELAY);
  digitalWrite(wr, LOW);
  delayMicroseconds(COMMAND_DELAY);
  digitalWrite(wr, HIGH);
  delayMicroseconds(COMMAND_DELAY);

}

//---------------------------------------------------------------------------------------------//
// function writeAmplitude
//---------------------------------------------------------------------------------------------//
void writeAmplitude(byte myVelocity, byte osc)
{
  uint8_t oscSelect;
  // latch byte
  // osc1
  // handle it normally
  oscSelect = (osc - 1) << 5;
  myVelocity = 15 - (myVelocity >> 3);
  working_byte = B10010000 + oscSelect + myVelocity;
  writeSN76489(working_byte);
}

//---------------------------------------------------------------------------------------------//
// function writeFrequency
//---------------------------------------------------------------------------------------------//
void writeFrequency(byte myPitch, byte osc, byte oct, byte detune)
{
  byte oscSelect;
  int pdatInt = 0;
  byte pdat1 = 0;
  byte pdat2 = 0;

  float octAdj = 1;
  float bendAdj;
  float detAdj;
  float frequency;

  // octave up
  if (oct > 94)
  {
    octAdj = 2;
  }
  // octave down
  if (oct < 32)
  {
    octAdj = 0.5;
  }

  //frequency = 440.00 * pow(2, (pitch + ((bend_data - 64.00) / 64.00) * bendamount - 69.00) / 12.00);
  // bendAdj = (1 + bend_data / 8190);
  bendAdj = ((bend_data - 8190) / 8190) * bendamount;
  detAdj = ((detune - 64.00) / 64.00) * detuneAmount;
  frequency = 440.00 * pow(2, ((float) myPitch + detAdj + bendAdj - 69.00) / 12.00) * octAdj;
  frequency = clock / (frequency * 32.00);
  pdatInt = frequency;
  if(pdatInt < 0) {
    pdatInt = 0;
  }
  else if(pdatInt > 1023) {
    pdatInt = 1023;
  }
  pdat1 = pdatInt % 16;
  oscSelect = (osc - 1) << 5;
  working_byte = B10000000 | oscSelect | pdat1;
  writeSN76489(working_byte);
  // data byte
  pdat2 = (pdatInt >> 4) & B00111111;
  writeSN76489(pdat2);

}

//---------------------------------------------------------------------------------------------//
// function rawFrequency
//---------------------------------------------------------------------------------------------//
void rawFrequency(byte myPitch, byte osc, byte oct, byte detune)
{
  byte oscSelect;
  int pdatInt = 0;

  float octAdj = 1;
  float bendAdj;
  float detAdj;
  float frequency;

  // octave up
  if (oct > 94)
  {
    octAdj = 2;
  }
  // octave down
  if (oct < 32)
  {
    octAdj = 0.5;
  }

  //frequency = 440.00 * pow(2, (pitch + ((bend_data - 64.00) / 64.00) * bendamount - 69.00) / 12.00);
  // bendAdj = (1 + bend_data / 8190);
  bendAdj = ((bend_data - 8190) / 8190) * bendamount;
  detAdj = ((detune - 64.00) / 64.00) * detuneAmount;
  frequency = 440.00 * pow(2, ((float) myPitch + detAdj + bendAdj - 69.00) / 12.00) * octAdj;
  frequency = clock / (frequency * 32.00);
  pdatInt = frequency;
  if(pdatInt < 0) {
    pdatInt = 0;
  }
  else if(pdatInt > 1023) {
    pdatInt = 1023;
  }

  switch (osc)
  {
    // osc1
  case 1:
    // copy the last current note to the old
    osc1F2 = osc1F1;
    // assign the new raw frequency to current
    osc1F1 = pdatInt;
    break;
    // osc2
  case 2:
    // copy the last current note to the old
    osc2F2 = osc2F1;
    // assign the new raw frequency to current
    osc2F1 = pdatInt;
    break;
  case 3:
    // copy the last current note to the old
    osc3F2 = osc3F1;
    // assign the new raw frequency to current
    osc3F1 = pdatInt;
    break;
  }  
}

//---------------------------------------------------------------------------------------------//
// function writeRawFrequency
//---------------------------------------------------------------------------------------------//
void writeRawFrequency(int frequency, byte osc)
{
  byte oscSelect;

  byte pdat1 = 0;
  byte pdat2 = 0;

  if(frequency < 0) {
    frequency = 0;
  }
  else if(frequency > 1023) {
    frequency = 1023;
  }
  pdat1 = frequency % 16;
  oscSelect = (osc - 1) << 5;
  working_byte = B10000000 | oscSelect | pdat1;
  writeSN76489(working_byte);
  // data byte
  pdat2 = (frequency >> 4) & B00111111;
  writeSN76489(pdat2);

}

//---------------------------------------------------------------------------------------------//
// function writeNoisePitch
//---------------------------------------------------------------------------------------------//
void writeNoisePitch(byte myPitch)
{
  writeSN76489(noiseLookup[myPitch]);
}

//---------------------------------------------------------------------------------------------//
// function doVib
//---------------------------------------------------------------------------------------------//
// not tested - very likely does not work
void doVib() {
  if(notes[0].myVelocity > 0 && vibModOn == 1) {

    if(vibAccum < vibSpeed * 10) {
      vibAccum = vibAccum + 1;
    }

    else if(vibAccum > vibSpeed * 10) {
      vibAccum = 0;
    }

    if(vibFlip == 0 && vibAccum == (vibSpeed * 10)) {
      bend_data = bend_MSB;
      // osc1
      writeFrequency(notes[0].myPitch, 1, 64, 64);
      // osc2
      writeFrequency(notes[0].myPitch, 2, osc2Octave, osc2Detune);
      // osc3
      writeFrequency(notes[0].myPitch, 3, osc3Octave, osc3Detune);
      vibFlip = 1;
      vibAccum = 0;
    }

    else if(vibFlip == 1 && vibAccum == (vibSpeed * 10)) {
      bend_data = bend_MSB + vibDepth;
      // osc1
      writeFrequency(notes[0].myPitch, 1, 64, 64);
      // osc2
      writeFrequency(notes[0].myPitch, 2, osc2Octave, osc2Detune);
      // osc3
      writeFrequency(notes[0].myPitch, 3, osc3Octave, osc3Detune);
      vibFlip = 0;
      vibAccum = 0;
    }
  }
}

//---------------------------------------------------------------------------------------------//
// function doAM
//---------------------------------------------------------------------------------------------//
void doAM()
{

  // toggle the value
  if (amOn == 0)
  {
    amOn = 1;
  }
  else
  {
    amOn = 0;
  }

  // only do something if note on and depth greater than zero
  if((notes[0].myPitch != 0) && (amDepth > 0)) {

    // AM off
    if (amOn == 0)
    {
      amVolume = 1.0;
    }
    // AM on
    else
    {
      amVolume = (float)(127 - amDepth) / 127.0;
    }
    // osc1
    writeAmplitude((byte)(v1 * vcaVolume * amVolume * notes[0].myVelocity), 1);
    // osc2
    writeAmplitude((byte)(v2 * vcaVolume * amVolume * notes[0].myVelocity), 2);
    // osc3
    writeAmplitude((byte)(v3 * vcaVolume * amVolume * notes[0].myVelocity), 3);
    // osc4
    writeAmplitude((byte)(v4 * vcaVolume * amVolume * notes[0].myVelocity), 4);
  }
}

//---------------------------------------------------------------------------------------------//
// function doGlide
//---------------------------------------------------------------------------------------------//
void doGlide()
{
  // glide to the new frequency
  // from the current frequency
  osc1F2 = osc1F2 - glideChangePerTick1;
  osc2F2 = osc2F2 - glideChangePerTick2;
  osc3F2 = osc3F2 - glideChangePerTick3;
  // write the raw frequencies
  writeRawFrequency((int)osc1F2, 1);
  writeRawFrequency((int)osc2F2, 2);
  writeRawFrequency((int)osc3F2, 3);
  // increment the glide tick count
  glideTickCount++;
  // check if done gliding
  if (glideTickCount >= glideTicks)
  {
    // set the exact new frequencies
    writeRawFrequency(osc1F1, 1);
    writeRawFrequency(osc2F1, 2);
    writeRawFrequency(osc3F1, 3);
    // update raw frequencies from start to end
    // might not be necessary
    osc1F2 = osc1F1;
    osc2F2 = osc2F1;
    osc3F2 = osc3F1;
    // clear the glide flag
    glideNote = 0;
  }
}

//---------------------------------------------------------------------------------------------//
// function setMIDIChannel
//---------------------------------------------------------------------------------------------//
void setMIDIChannel()
{
  int clickCount = 0;
  unsigned long previousMillis;
    
  // light the led for 1.0 sec to indicate set mode
  digitalWrite(LED_PIN, HIGH);
  previousMillis = millis();
  while (millis() - previousMillis < 1000)
  {
    // nothing
  }
  digitalWrite(LED_PIN, LOW);
  
  
  // loop until the channel button is held down
  while (1)
  { 
    if (channelButton.update())
    {
      if (channelButton.read() == LOW)
      {
        // start timing
        previousMillis = millis();
        // while the button is down
        while (1)
        {
          if (channelButton.update())
          {
            if (channelButton.read() == HIGH)
            {
              break;
            }
          }
        }
        // check how long the button was down
        if (millis() - previousMillis < LONG_CLICK_MS)
        {
          // count a click
          clickCount++;
        }
        else
        {
          // stop counting
          break;
        }
      }
    }
  }
  
  // light the led for 1.0 sec to indicate set mode done
  digitalWrite(LED_PIN, HIGH);
  previousMillis = millis();
  while (millis() - previousMillis < 1000)
  {
    // nothing
  }
  digitalWrite(LED_PIN, LOW);
  
  // blink back the channel
  for (int r = 0; r <= clickCount; r++)
  {
    digitalWrite(LED_PIN, HIGH);
    previousMillis = millis();
    while (millis() - previousMillis < 50)
    {
      // nothing
    }
    digitalWrite(LED_PIN, LOW);
    previousMillis = millis();
    while (millis() - previousMillis < 500)
    {
      // nothing
    }
  }
 
  // store the setting to EEPROM if not zero or greater than 15
  if ((clickCount > 0) && (clickCount < 16))
  {
    EEPROM.write(EE_MIDI_CHANNEL, clickCount);
  }
 
  return;
  
}

//---------------------------------------------------------------------------------------------//
// function setFilterMode
// accepts byte
// 0 - lowpass
// 1 - bandpass
// 2 - highpass
// returns nothing
//---------------------------------------------------------------------------------------------//
void setFilterMode(byte mode)
{
  switch (mode)
  {
    case 0:
      // lowpass
      digitalWrite(MUX_A0_PIN, LOW);
      digitalWrite(MUX_A1_PIN, LOW);
      break;
      
    case 1:
      // bandpass
      digitalWrite(MUX_A0_PIN, HIGH);
      digitalWrite(MUX_A1_PIN, LOW);
      break;

    case 2:
      // bandpass
      digitalWrite(MUX_A0_PIN, LOW);
      digitalWrite(MUX_A1_PIN, HIGH);
      break;
      
    default:
      // do nothing
      break;
  }
  return;
}
      
// pwm functions below
//---------------------------------------------------------------------------------------------//
// function enable_intr
//---------------------------------------------------------------------------------------------//
void enable_intr(){
  TIMSK4 = _BV(TOIE4);
}

//---------------------------------------------------------------------------------------------//
// function disable_intr
//---------------------------------------------------------------------------------------------//
void disable_intr(){
  TIMSK4 = 0;
}

//---------------------------------------------------------------------------------------------//
// function initialize
//---------------------------------------------------------------------------------------------//
void initialize(unsigned long freq) {

  /* Init the internal PLL */
  PLLFRQ = _BV(PDIV2);
  PLLCSR = _BV(PLLE);
  while(!(PLLCSR & _BV(PLOCK)));
  PLLFRQ |= _BV(PLLTM0); /* PCK 48MHz */

  TCCR4A = (1<<PWM4A);  
  TCCR4E = (1<<ENHC4); 
  TCCR4D = (1<<WGM40); //set it to phase and frequency correct mode    
  TCCR4C = 0;     
  setPeriod(freq);    
}

//---------------------------------------------------------------------------------------------//
// function setPwmDuty
//---------------------------------------------------------------------------------------------//
void setPwmDuty(unsigned int duty) { 	
  unsigned long dutyCycle = pwmPeriod; 	
  dutyCycle *= duty; 	
  dutyCycle >>= 9;
  TC4H = (dutyCycle) >> 8; 
  OCR4A = (dutyCycle) & 255; 
}

//---------------------------------------------------------------------------------------------//
// function start
//---------------------------------------------------------------------------------------------//
void start() {
  TCCR4A |= _BV(COM4A1);
}

//---------------------------------------------------------------------------------------------//
// function stop
//---------------------------------------------------------------------------------------------//
void stop()  {
  TCCR4A &= ~(_BV(COM4A1));
}

//---------------------------------------------------------------------------------------------//
// function setPeriod
//---------------------------------------------------------------------------------------------//
void setPeriod(unsigned long freq)  {
  unsigned long cycles = PLL_FREQ / 2 / freq;
  unsigned char clockSelectBits = 0;

  if (cycles < TIMER4_RESOLUTION) {
    clockSelectBits = _BV(CS40);
    pwmPeriod = cycles;
  } 
  else
    if (cycles < TIMER4_RESOLUTION * 2) {
      clockSelectBits = _BV(CS41);
      pwmPeriod = cycles / 2;
    } 
    else
      if (cycles < TIMER4_RESOLUTION * 4) {
        clockSelectBits = _BV(CS41) | _BV(CS40);
        pwmPeriod = cycles / 4;
      } 
      else
        if (cycles < TIMER4_RESOLUTION * 8) {
          clockSelectBits = _BV(CS42);
          pwmPeriod = cycles / 8;
        } 
        else
          if (cycles < TIMER4_RESOLUTION * 16) {
            clockSelectBits = _BV(CS42) | _BV(CS40);
            pwmPeriod = cycles / 16;
          } 
          else 
            if (cycles < TIMER4_RESOLUTION * 32) {
            clockSelectBits = _BV(CS42) | _BV(CS41);
            pwmPeriod = cycles / 32;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 64) {
            clockSelectBits = _BV(CS42) | _BV(CS41) | _BV(CS40);
            pwmPeriod = cycles / 64;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 128) {
            clockSelectBits = _BV(CS43);
            pwmPeriod = cycles / 128;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 256) {
            clockSelectBits = _BV(CS43) | _BV(CS40);
            pwmPeriod = cycles / 256;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 512) {
            clockSelectBits = _BV(CS43) | _BV(CS41);
            pwmPeriod = cycles / 512;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 1024) {
            clockSelectBits = _BV(CS43) | _BV(CS41) | _BV(CS40);
            pwmPeriod = cycles / 1024;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 2048) {
            clockSelectBits = _BV(CS43) | _BV(CS42);
            pwmPeriod = cycles / 2048;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 4096) {
            clockSelectBits = _BV(CS43) | _BV(CS42) | _BV(CS40);
            pwmPeriod = cycles / 4096;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 8192) {
            clockSelectBits = _BV(CS43) | _BV(CS42) | _BV(CS41);
            pwmPeriod = cycles / 8192;
          } 
          else 	
            if (cycles < TIMER4_RESOLUTION * 16384) { 		
            clockSelectBits = _BV(CS43) | _BV(CS42) | _BV(CS41) | _BV(CS40); 		
            pwmPeriod = cycles / 16384; 	
          }  /*else    		clockSelectBits = _BV(CS43) | _BV(CS42) | _BV(CS41) | _BV(CS40); 		pwmPeriod = TIMER4_RESOLUTION - 1; */
  TCCR4B = clockSelectBits;             
  TC4H = pwmPeriod >> 8; 
  OCR4C = pwmPeriod; 
}



