#include <U8g2lib.h>
#include <Wire.h>
#include <cmath>
#include <string>
#include <knob.h>
#include <LUTs.h>
#include <STM32FreeRTOS.h>

//Define constants used by the program:

#define SAMPLING_FREQUENCY 22000
//The below value is 2^32
#define MAXVAL 4294967296
//Size of the echo buffer
#define ECHO_BUFFER 2300
//Makes the fact that no note is being played more readable
#define NO_KEY 12
//Record the number of keys for polyphony
#define KEYS 12

//Uncomment the below line if testing - else leave it.
//#define NO_SAMPLE_ISR


//-------------------------------------------------------------------------------

//Pin definitions - give pin numbers human-readable identifiers.

//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Key matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

//------------------------------------------------------------------------------

//Define important global variables and data structures.

//Not initialised to any value, it needs to be set.
volatile uint32_t currentStepSize;

//The key array - keeps track of which keys/knobs are pressed/turned.
volatile uint8_t keyArray[7];

//Used to record the current key being pressed (for no polyphony).
volatile int currentKey = NO_KEY;
//The pressed array is set based on which of the 12 keys are pressed - useful for polyphony.
#ifndef NO_SAMPLE_ISR
//If not testing initialise to all zero (no presses).
volatile int pressed[] = {0,0,0,0,0,0,0,0,0,0,0,0};
#else
volatile int pressed[] = {1,1,1,1,1,1,1,1,0,0,0,0};
#endif

//Used to lock key array to ensure safe concurrent access by tasks.
SemaphoreHandle_t keyArrayMutex;

//Simple lookup table to convert int digit index to hex digit.
const char intToHex[] = "0123456789ABCDEF";

//OUTWARD MESSAGES----------------------------
//Used to hold the current message being sent to serial output.
volatile char noteMessage[]= "xxx";

//INWARD MESSAGES-----------------------------
//Used to store latestt message that has been read in from serial input.
volatile char incoming[] = "xxx";

//Used to determine what frequency shift is required i.e. what octave we are playing.
//Can be set by an input message.
volatile int globalShift = 0;

//The message queue for input and output messages.
QueueHandle_t msgOutQ;


//TREMOLO-------------------------------------
//Possible extension - make parameters configurable by user.
//Maximium deviation of amplitude oscillation.
const float tremolo_amplitude = 0.5;
//Rate of amplitude oscillation.
const uint32_t tremolo_freq = 20;

//This is not a parameter but varies sinusoidally with time. Multiplied with output waveform to
//modulate amplitude.
volatile float tremolo_offset = 0;


//VIBRATO---------------------------------------
//Possible extension - make parameters configurable by user.
//Maximum deviation of frequency oscillation as fraction of note frequency.
//volatile uint32_t vibrato_mod_factor = 0.03;
//Rate of frequency oscillation
const uint32_t vibrato_freq = 5;

//An array of positive/negative offsets that is added to the step size of each note during vibrato.
volatile uint32_t vibratoSize[] = {0,0,0,0,0,0,0,0,0,0,0,0};


//ECHO------------------------------------------
//Define echo buffer.
uint8_t echo_buffer[ECHO_BUFFER];


//Below variables determine effect(s) on the sound played.
//XATOCODE: 0 = No effect, 1 = vibrato/echo (if effect available), 2 = tremolo.
volatile int XATOCODE = 0;
//FIR: 0 = No FIR smoothing, 1 = FIR smoothing.
volatile int FIR = 0;
//WAVECODE: 0 = Sawtooth, 1 = Sinusoid, 2 = Square, 3 = Exponential, 4 = Triangle.
volatile int WAVECODE = 0;

//Objects for each knob (class defined in Knob.h - initialised in setup)
//Initialise knobs.
Knob knob3 = Knob(0, true, 16, 0);
Knob knob2 = Knob(0, true, 4, 0);
Knob knob1 = Knob(0, true, 2, 0);
Knob knob0 = Knob(0, true, 1, 0);

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//------------------------------------------------------------------------------

//Define important helper functions used in the program.

//Reads each column output bit and concatenates for output.
uint8_t readCols(){

  uint8_t concatenated = 0;

  uint8_t col0 = digitalRead(C0_PIN);
  concatenated = col0 | concatenated;

  uint8_t col1 = digitalRead(C1_PIN);
  concatenated = (col1 << 1) | concatenated;

  uint8_t col2 = digitalRead(C2_PIN);
  concatenated = (col2 << 2) | concatenated;

  uint8_t col3 = digitalRead(C3_PIN);
  concatenated = (col3 << 3) | concatenated;

  return concatenated;
}


//Used to enable a particular row by setting row enable and the row pin low.
void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);

  if ((rowIdx) == 6) {
    digitalWrite(RA2_PIN, HIGH);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA0_PIN, LOW);
  }
  if ((rowIdx) == 5) {
    digitalWrite(RA2_PIN, HIGH);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA0_PIN, HIGH);
  }
  if ((rowIdx) == 4) {
    digitalWrite(RA2_PIN, HIGH);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA0_PIN, LOW);
  }
  if ((rowIdx) == 3) {
    digitalWrite(RA2_PIN, LOW);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA0_PIN, HIGH);
  }
  if ((rowIdx) == 2) {
    digitalWrite(RA2_PIN, LOW);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA0_PIN, LOW);
  }
  if ((rowIdx) == 1) {
    digitalWrite(RA2_PIN, LOW);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA0_PIN, HIGH);
  }
  if ((rowIdx) == 0) {
    digitalWrite(RA2_PIN, LOW);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA0_PIN, LOW);
  }

  digitalWrite(REN_PIN, HIGH);
}


//Used to update with time the tremolo_offset variable defined above.
void tremolo_update(){
  static uint32_t tremolo_phase = 0;
  uint32_t tremolo_step_size = MAXVAL*tremolo_freq/SAMPLING_FREQUENCY;

  //Find ratio of phase wrt its maximum value
  float ratio = ((float)tremolo_phase)/MAXVAL;
  float phase = 2*M_PI*ratio;

  //Find the sine value, and shift mean to 1. We get a sine oscillating around 1.
  float sinewave = 1 + tremolo_amplitude*sin(phase);
  tremolo_offset = sinewave;

  //Move the phase accumulator ahead.
  tremolo_phase += tremolo_step_size;
}


//Update offset that is added to step size of each note, in order to vary frequency.
void vibrato_update(){
  static uint32_t vibrato_phase = 0;
  uint32_t vibrato_step_size = MAXVAL*vibrato_freq/SAMPLING_FREQUENCY;

  //CALCULATE CURRENT VALUE OF VIBRATO WAVE (sinusoid)

  //Find ratio of phaseAcc wrt its maximum value
  float ratio = ((float)vibrato_phase)/MAXVAL;
  float phase = 2*M_PI*ratio;
  
  //Find the sine value (mean zero).
  float sinewave = (0.03*440*sin(phase));

  //UPDATE CURRENT STEP SIZE BASED ON SINUSOID

  uint32_t setter = MAXVAL*sinewave/SAMPLING_FREQUENCY;
  
  for(int i=0; i<12; i++){
        if(__atomic_load_n(&pressed[i], __ATOMIC_RELAXED) == 1){
          __atomic_store_n(&vibratoSize[i], setter, __ATOMIC_RELAXED);
        }
      }
   vibrato_phase += vibrato_step_size;
}

//------------------------------------------------------------------------------

//The interrupt routine in charge of controlling output synth waveforms.
void sampleISR(){
  //Set the effect-determining codes based on knobs.
  WAVECODE = knob2.get_rotation();
  XATOCODE = knob1.get_rotation();
  FIR =  knob0.get_rotation();

  //Store XATOCODE locally for fast reference.
  int localXATO = XATOCODE;
  //Store currentKey locally for fast reference.
  int localCurrentKey = currentKey;

  //Below variables control reading and writing from/to the echo buffer.
  static uint16_t echo_write_index = 1;
  static uint16_t echo_read_index = 0;
  //The buffer must fill up with writes before reading can start.
  static bool start_reading = false;

  //Update vibrato offsets if effect set and we play sawtooth / triangle (only works well for these).
  if(localXATO == 1 && localCurrentKey != NO_KEY && (WAVECODE == 0 || WAVECODE == 4)) vibrato_update();
  //Update tremolo offsets if effect set.
  if(localXATO == 2 && localCurrentKey != NO_KEY) tremolo_update();

  //Store past outputs for multiplication with FIR coefficients.
  static uint32_t past_outputs[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  //Used to accumulate the products of coefficients and samples.
  float final_output = 0;

  //A phase accumulator for the output wave (if no polyphony implemented).
  static uint32_t phaseAcc = 0;
  //An array of phase accumulators for each note (for polyphony implemented).
  static uint32_t phaseAccArr[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  //To store output value (for polyphony not implemented).
  uint16_t output;
  //To store output value for each key (for polyphony implemented).
  uint8_t outValue[KEYS];

  //Generate different waveforms based on the WAVECODE
  switch(WAVECODE)
  {
    //To generate a sawtooth wave (polyphony, vibrato, tremolo available)
    case 0:
    {
      //Count how many notes are pressed so that scaling can be applied to polyphony.
      int count = 0;
      for(int i = 0; i < KEYS; i++) if(pressed[i] == 1){count++;}
      //Polyphony only applies if a user presses all notes simultaneously. It does not apply to messages
      //as messages arrive sequentially.
      if(incoming[0] == 'R')
      {
        for(int i=0; i < KEYS; i++){
          //Check if note pressed.
          if(pressed[i] == 1){
             phaseAccArr[i] += stepSizes[i];
             //Apply vibrato if effect set.
             if(localXATO == 1){
              phaseAccArr[i]+= vibratoSize[i];
             }
             outValue[i] = phaseAccArr[i]>>24;
             //Scale down for polyphony.
             output += outValue[i]/count;
          }
        }
        //Clip any high output to maximum possible value
        if(output > 255){
            output = 255;
        }
      }
      //In case of no polyphony i.e. an external key press message.
      else output = phaseAcc >> 24;
      break;
    }
    //To generate a sine wave (echo, tremolo available).
    case 1:
    {
      //Find ratio of phaseAcc wrt its maximum value
      float ratio = ((float)phaseAcc)/MAXVAL;
      if(ratio == 1.0) {ratio = 0;}

      //Multiply ratio by 1024 to find integer index in LUT (defined in LUT.h)
      int LUT_phase = (int)(ratio*1024.0);

      //Define amplitude of the sine wave = 255/2
      float amplitude = 127.5;
      //Find the sine value as float value in the range 0 - 255. Shift mean to positive amplitude.
      float sinewave = (amplitude*(1 + sWave[LUT_phase]));

      //Convert to unsigned 8 bit integer to write to output
      output = 2*(uint16_t)sinewave;
      //If output is too high, clip to maximum allowed value
      if(output > 255){output = 255;}
      break;
    }
    //To generate a square wave - (tremolo available).
    case 2:
    {
      //Set wave high for second half of period.
      if (phaseAcc > MAXVAL/2){output = 0xFF;}
      //Else set the wave to low = 0
      else{output = 0;}
      break;
    }
    //To generate an exponential wave - (tremolo available).
    case 3:
    {
        //Find ratio of phaseAcc wrt its maximum value
        float ratio = ((float)phaseAcc)/MAXVAL;
        //Don't use LUT if ratio is 1 - this keeps the LUT to 1024 elements, a nice power of 2.
        if(ratio == 1.0) {output = 255.0;}
        else
        {
          //Multiply ratio by 1024 to find integer index in LUT (defined in LUT.h).
          int LUT_index = (int)(ratio*1024.0);
          //Raise e^power by lookup.
          float expo = eWave[LUT_index];
          //Convert to unsigned 8-bit integer to write to output
          output = (uint8_t)expo;
        }
        break;
    }
    //Triangle wave - (polyphony, vibrato, tremolo available).
    case 4:
    {
      //Count how many notes are pressed so that scaling can be applied to polyphony.
      int count = 0;
      for(int i = 0; i < KEYS; i++) if(pressed[i] == 1){count++;}
      //Polyphony only applies if a user presses all notes simultaneously. It does not apply to messages
      //as messages arrive sequentially.
      if(incoming[0] == 'R')
      {
        for(int i=0; i < KEYS; i++){
          //Check if note pressed.
          if(pressed[i] == 1){
             phaseAccArr[i] += stepSizes[i];
             //Apply vibrato if effect set.
             if(localXATO == 1){
              phaseAccArr[i]+= vibratoSize[i];
             }
             //shift mean of phaseAcc to zero
             int32_t shift_mean = phaseAccArr[i] - MAXVAL/2;
             //Implement math.abs() homebrew for better performance - want to find absolute value.
             uint32_t triangleValue = shift_mean >= 0? shift_mean : -shift_mean;
             //Shift mean back and obtain most significant 8 bits to write to output, multiplying
             //by 2.
             outValue[i] = triangleValue >> 23;
             //Scale for polyphony.
             output += outValue[i]/count;
          }
        }
        //If output is too high clip to maximum value.
        if(output > 255){output = 255;}
      }
      //In case of no polyphony (i.e. for a keyPress message)
      else
      {
        //shift mean to zero
        int32_t shift_mean = phaseAcc - MAXVAL/2;
        //Implement math.abs() homebrew for better performance - want to find absolute value.
        uint32_t triangleValue = shift_mean >= 0? shift_mean : -shift_mean;
        //Shift mean back and obtain most significant 8 bits to write to output, multiplying
        //by 2.
        output = triangleValue >> 23;
      }
      break;
    }
    //default case
    default:
      phaseAcc = 0;
  }

 //Multiply by past outputs by FIR coefficients, shifting the past output array back by 1.
 for(int i = 9; i > 0; i--){
    past_outputs[i] = past_outputs[i-1];
    final_output += fir_coefficients[i]*past_outputs[i];
  }
  past_outputs[0] = output;
  final_output += fir_coefficients[0]*past_outputs[0];

  uint8_t digiwrite = output;

  //Apply FIR value if effect set.
  if(FIR == 1) digiwrite = (uint8_t)final_output;

  //Apply echo if wave is sine and effect is set.
  if(localXATO == 1 && WAVECODE == 1){
    //The attenuated echo term (attenuation factor = 0.25)
    uint8_t echo_term = (echo_buffer[echo_read_index])>>2;
    //Echo term is subtracted from the wave - allows stable reverberation. Must clip negative values to zero.
    digiwrite = (digiwrite > echo_term) ? digiwrite - echo_term : 0;
  }

  //Apply tremolo if effect set.
  if(localXATO == 2 && localCurrentKey != NO_KEY){
    float modulated_output = (digiwrite - 127.5)*(tremolo_offset/(1 + tremolo_amplitude)) + 127.5;
    digiwrite = (uint8_t)modulated_output;
  }

  //After all effects, write to ADC.
  analogWrite(OUTR_PIN, digiwrite >> (8-knob3.get_rotation()/2)); //waveform output

  //Update phase.
  phaseAcc += currentStepSize;

  if(localXATO == 1 && WAVECODE == 1){
      echo_buffer[echo_write_index] = digiwrite;
      //Move echo buffer indices ahead
      echo_write_index = (echo_write_index + 1)%ECHO_BUFFER;
     //Initially we only write to the buffer. After the buffer is fully written, we start reading as well
      if(start_reading) echo_read_index = (echo_read_index + 1)%ECHO_BUFFER;
      //Check if echo buffer is fully written
      if(echo_write_index == ECHO_BUFFER-1) start_reading = true;
  }
}

//------------------------------------------------------------------------------

//Function to set outputs via matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

//------------------------------------------------------------------------------

// Setup code here - run once at start:
void setup() {

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(115200);
  Serial.println("Hello World");

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer= new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
//Only setup sampleISR interrupt routine if we are not in testing mode.
#ifndef NO_SAMPLE_ISR
  sampleTimer->attachInterrupt(sampleISR);
#endif
  sampleTimer->resume();

  //Task to scan key array (knobs, notes etc)
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask,/* Function that implements the task */
              "scanKeysTask",/* Text name for the task */
              64,      /* Stack size in words, not bytes*/
              NULL,/* Parameter passed into the task */
              3,/* Task priority*/
              &scanKeysHandle);   /* Pointer to store the task handle*/

  //Task to write to display
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(displayUpdateTask,/* Function that implements the task */
              "displayUpdateTask",/* Text name for the task */
              256,      /* Stack size in words, not bytes*/
              NULL,/* Parameter passed into the task */
              1,/* Task priority*/
              &displayUpdateHandle);   /* Pointer to store the task handle*/

  //Task to send messages to serial output
  TaskHandle_t msgOutTaskHandle = NULL;
  xTaskCreate(msgOutTask,/* Function that implements the task */
              "msgOutTask",/* Text name for the task */
              32,      /* Stack size in words, not bytes*/
              NULL,/* Parameter passed into the task */
              2,/* Task priority*/
              &msgOutTaskHandle);   /* Pointer to store the task handle*/

  //Task to read a message in from serial input
  TaskHandle_t msgInTaskHandle = NULL;
  xTaskCreate(msgInTask,/* Function that implements the task */
              "msgInTask",/* Text name for the task */
              32,      /* Stack size in words, not bytes*/
              NULL,/* Parameter passed into the task */
              4,/* Task priority*/
              &msgInTaskHandle);   /* Pointer to store the task handle*/
  //Initialise key array mutex.
  keyArrayMutex = xSemaphoreCreateMutex();
  //Create message queue.
  msgOutQ = xQueueCreate(8, 4);

  vTaskStartScheduler();
}

//------------------------------------------------------------------------------

//Implement the tasks:

//Only send out messages if not testing.
#ifndef NO_SAMPLE_ISR
void msgOutTask(void *pvParameters) {
  char outMsg[4];
  while (1) {
    xQueueReceive(msgOutQ, outMsg, portMAX_DELAY);
    Serial.println(outMsg);
  }
}

//--------------------------------

void msgInTask(void *pvParameters){

  const TickType_t xFrequency = 5/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  char inMsg[] = "xxx";
  int count = 0;

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    count = 0;

    while(Serial.available() > 0){

      char value = Serial.read();
      if(count < 3){
        inMsg[count] = value;
        __atomic_store_n(&incoming[count], value, __ATOMIC_RELAXED);
      }
      //For each new message.
      if(value == '\n')
      {
        count = 0;

        if(inMsg[0] == 'R'){
          __atomic_store_n(&globalShift, 0, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
        }
        else if(inMsg[0] == 'P')
        {
              __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
              //Find desired octave.
              int shift = inMsg[1] - '0';
              __atomic_store_n(&globalShift, shift - 4, __ATOMIC_RELAXED);
              //Higher and middle octaves.
              if(globalShift >= 0){
                  //Set the current key based on the provided number.
                  if (inMsg[2] >= '0' && inMsg[2] <= '9'){
                    __atomic_store_n(&currentKey, inMsg[2] - '0', __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey]<< globalShift, __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'A' && inMsg[2] <= 'F'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'A' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] << globalShift, __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'a' && inMsg[2] <= 'f'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'a' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] << globalShift, __ATOMIC_RELAXED);
                  }
              }
              //Lower octaves.
              else{
                  //Set the current key based on the provided number.
                  if (inMsg[2] >= '0' && inMsg[2] <= '9'){
                    __atomic_store_n(&currentKey, inMsg[2] - '0', __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] >> abs(globalShift), __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'A' && inMsg[2] <= 'F'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'A' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] >> abs(globalShift), __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'a' && inMsg[2] <= 'f'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'a' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] >> abs(globalShift), __ATOMIC_RELAXED);
                  }
              }
          }
      }
      //Else move through the message.
      else
      {
        count++;
      }
    }
  }
}

//------------------------------------------

void scanKeysTask(void * pvParameters) {

  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t rowThreePrev;
  uint8_t rowThreeCurr;
  uint8_t rowFourPrev;
  uint8_t rowFourCurr;

  //Records the previous key pressed.
  static int prevPress = NO_KEY;

  //Used to denote whether we have sent a play message to the serial - if so the next message should be release.
  static bool saidRead = 0;

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); //lock keyArray

    //Store previous values of rows 3 and 4 - to detect knob rotation.
    rowThreePrev = keyArray[3];
    rowFourPrev = keyArray[4];

    //Key presses will be recorded in KeyArray here.
    for(uint8_t i = 0; i < 5; i++){
      setRow(i);
      delayMicroseconds(2);
      keyArray[i] = readCols();
    }

    //Store current values of rows 3 and 4.
    rowThreeCurr = keyArray[3];
    rowFourCurr = keyArray[4];

//////////////////////// SCANNING KNOB ROTATIONS ////////////////////////

    switch(rowThreePrev & 0b11){
      case 0b00:
        if((rowThreeCurr & 0b11) == 0b01) knob3.add(2);
        else if ((rowThreeCurr & 0b11) == 0b10) knob3.sub(2);
        else if ((rowThreeCurr & 0b11) != (rowThreePrev & 0b11)) knob3.cond_addsub(2);
        break;
      case 0b01:
        if((rowThreeCurr & 0b11) == 0b00) knob3.sub(2);
        else if ((rowThreeCurr & 0b11) == 0b11) knob3.add(2);
        else if ((rowThreePrev & 0b11) != (rowThreeCurr & 0b11)) knob3.cond_addsub(2);
        break;
      case 0b10:
        if ((rowThreeCurr & 0b11) == 0b00) knob3.add(2);
        else if ((rowThreeCurr & 0b11) == 0b11) knob3.sub(2);
        else if ((rowThreePrev & 0b11) != (rowThreeCurr & 0b11)) knob3.cond_addsub(2);
        break;
      case 0b11:
        if ((rowThreeCurr & 0b11) == 0b01) knob3.sub(2);
        else if ((rowThreeCurr & 0b11) == 0b10) knob3.add(2);
        else if ((rowThreePrev & 0b11) != (rowThreeCurr & 0b11)) knob3.cond_addsub(2);
        break;
      default:
        break;
    }

    switch(rowThreePrev >> 2 & 0b11){
      case 0b00:
        if((rowThreeCurr >> 2 & 0b11) == 0b01) knob2.add(1);
        else if((rowThreeCurr >> 2 & 0b11) == 0b10) knob2.sub(1);
        else if((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      case 0b01:
        if ((rowThreeCurr >> 2 & 0b11) == 0b00) knob2.sub(1);
        else if ((rowThreeCurr >> 2 & 0b11) == 0b11) knob2.add(1);
        else if ((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      case 0b10:
        if((rowThreeCurr >> 2 & 0b11) == 0b00) knob2.add(1);
        else if((rowThreeCurr >> 2 & 0b11) == 0b11) knob2.sub(1);
        else if((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      case 0b11:
        if((rowThreeCurr >> 2 & 0b11) == 0b01) knob2.sub(1);
        else if((rowThreeCurr >> 2 & 0b11) == 0b10) knob2.add(1);
        else if((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      default:
        break;
    }

    switch(rowFourPrev & 0b11){
      case 0b00:
        if((rowFourCurr & 0b11) == 0b01) knob1.add(1);
        else if((rowFourCurr & 0b11) == 0b10) knob1.sub(1);
        else if ((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      case 0b01:
        if((rowFourCurr & 0b11) == 0b00) knob1.sub(1);
        if((rowFourCurr & 0b11) == 0b11) knob1.add(1);
        else if((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      case 0b10:
        if((rowFourCurr & 0b11) == 0b00) knob1.add(1);
        if((rowFourCurr & 0b11) == 0b11) knob1.sub(1);
        else if((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      case 0b11:
        if((rowFourCurr & 0b11) == 0b01) knob1.sub(1);
        if((rowFourCurr & 0b11) == 0b10) knob1.add(1);
        else if((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      default:
        break;
    }

    switch(rowFourPrev >> 2 & 0b11){
      case 0b00:
        if((rowFourCurr >> 2 & 0b11) == 0b01) knob0.add(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b10) knob0.sub(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;
      case 0b01:
        if((rowFourCurr >> 2 & 0b11) == 0b00) knob0.sub(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b11) knob0.add(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;
      case 0b10:
        if((rowFourCurr >> 2 & 0b11) == 0b00) knob0.add(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b11) knob0.sub(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;
      case 0b11:
        if((rowFourCurr >> 2 & 0b11) == 0b01) knob0.sub(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b10) knob0.add(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;

      default:
        break;
    }

//////////////////////// END OF SCANNING KNOB ROTATIONS ////////////////////////

//////////////////////// SCANNING KEY PRESSES ////////////////////////.

    bool keyPress = 0;
    int count = 0;
    uint8_t zero = keyArray[0];

    //Set key presed array to all zeros to start.
    for(int i = 0; i<KEYS; i++){
        pressed[i] = 0;
      }

    //Determine which keys are pressed.
    if(!(zero & 0b1)){
        pressed[0] = 1;
        currentKey = 0;
        keyPress = 1;
        count++;
    }
    if(!(zero>>1 & 0b1)){
        pressed[1] = 1;
        currentKey = 1;
        keyPress = 1;
        count++;
    }
    if(!(zero>>2 & 0b1)){
        pressed[2] = 1;
        currentKey = 2;
        keyPress = 1;
        count++;
    }
    if(!(zero>>3 & 0b1)){
        pressed[3] = 1;
        currentKey = 3;
        keyPress = 1;
        count++;
    }

    if(!(keyArray[1] & 0b1)){
        pressed[4] = 1;
        currentKey = 4;
        keyPress = 1;
        count++;
    }
    if(!(keyArray[1]>>1 & 0b1)){
        pressed[5] = 1;
        currentKey = 5;
        keyPress = 1;
        count++;
    }
    if(!(keyArray[1]>>2 & 0b1)){
        pressed[6] = 1;
        currentKey = 6;
        keyPress = 1;
        count++;
    }
    if(!(keyArray[1]>>3 & 0b1)){
        pressed[7] = 1;
        currentKey = 7;
        keyPress = 1;
        count++;
    }

    if(!(keyArray[2] & 0b1)){
        pressed[8] = 1;
        currentKey = 8;
        keyPress = 1;
        count++;
    }
    if(!(keyArray[2]>>1 & 0b1)){
        pressed[9] = 1;
        currentKey = 9;
        keyPress = 1;
        count++;
    }
    if(!(keyArray[2]>>2 & 0b1)){
        pressed[10] = 1;
        currentKey = 10;
        keyPress = 1;
        count++;
    }
    if(!(keyArray[2]>>3 & 0b1)){
        pressed[11] = 1;
        currentKey = 11;
        keyPress = 1;
        count++;
    }

//////////////////////// ENF OF SCANNING KEY PRESSES ////////////////////////

    //Sends "release" given on key release that "play" message has been sent.
    if(!keyPress && saidRead && prevPress == currentKey){
      noteMessage[0] = 'R';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[prevPress];
      saidRead = 0;
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    //Sends "play" message when a new note is played.
    if(keyPress && prevPress != currentKey){ //
      incoming[0] = 'R'; //this is to interrupt an incoming message with your own keys
      globalShift = 0; //don't want to shift the sound if playing from main board
      noteMessage[0] = 'P';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[currentKey];
      prevPress = currentKey;
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
      saidRead = 1;
    }
    //Sends "play" message when the same note is pressed again.
    if(keyPress && noteMessage[0] == 'R' && prevPress == currentKey){
      noteMessage[0] = 'P';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[currentKey];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    //Sends a release for the above case once key is released.
    else if(!keyPress && noteMessage[0] == 'P'){
      noteMessage[0] = 'R';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[currentKey];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }

    xSemaphoreGive(keyArrayMutex);

    //If a key is released, set the variables to indicate this.
    if(!keyPress && (__atomic_load_n(&incoming[0], __ATOMIC_RELAXED) == 'R'  || __atomic_load_n(&incoming[0], __ATOMIC_RELAXED) == 'x')){
      currentKey = NO_KEY;
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    }
    //Not currently useful, but these two cases would be useful if we wanted to vary the octaves via user input.
    else if(__atomic_load_n(&globalShift, __ATOMIC_RELAXED) >= 0){
      __atomic_store_n(&currentStepSize, stepSizes[currentKey]<<globalShift, __ATOMIC_RELAXED);
    }
    else{
      __atomic_store_n(&currentStepSize, stepSizes[currentKey]>>abs(globalShift), __ATOMIC_RELAXED);
    }
  }
}

//-----------------------------------------------------

void displayUpdateTask(void * pvParameters){

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();
  char* key[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "None"};

  while(1){

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font u8g2_font_ncenB08_tr
    int localXATO = __atomic_load_n(&XATOCODE, __ATOMIC_RELAXED);
    int localWAVE = __atomic_load_n(&WAVECODE, __ATOMIC_RELAXED);
    int localFIR = __atomic_load_n(&FIR, __ATOMIC_RELAXED);
    
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); //lock keyArray

    //Display Volume
    u8g2.setCursor(2,10);
    u8g2.print("Volume: ");
    u8g2.print(knob3.get_rotation());

    //Then display the note(s) played if any
    u8g2.setCursor(2,20);
    u8g2.print("Key: ");

    switch(localWAVE){
      case 0:
      {
        bool anyPressed = 0;
        for(int i = 0; i < KEYS; i++){
          if(pressed[i] == 1){
            u8g2.print(key[i]);
            anyPressed = 1;
          }
        }
        if(!anyPressed) u8g2.print(key[NO_KEY]);
        break;
      }
      case 1:
        u8g2.print(key[currentKey]);
        break;
      case 2:
        u8g2.print(key[currentKey]);
        break;
      case 3:
        u8g2.print(key[currentKey]);
        break;
      case 4:
      {
        bool anyPressed = 0;
        for(int i = 0; i < KEYS; i++){
          if(pressed[i] == 1){
            u8g2.print(key[i]);
            anyPressed = 1;
          }
        }
        if(!anyPressed) u8g2.print(key[NO_KEY]);
        break;
      }
      default:
        u8g2.print("ERROR!");
    }

    //Print waveform.
    u8g2.print(" - ");
    switch(localWAVE){
      case 0:
        u8g2.print("Saw");
        break;
      case 1:
        u8g2.print("Sine");
        break;
      case 2:
        u8g2.print("Square");
        break;
      case 3:
        u8g2.print("Expo");
        break;
       case 4:
        u8g2.print("Triangle");
        break;
      default:
        u8g2.print("ERROR!");
    }

    //Print effects.
    u8g2.setCursor(2,30);
    u8g2.print("Effect:");

     switch(localXATO){
        case 0:
          u8g2.print("None");
          break;
        case 1:
          if(localWAVE == 0 || localWAVE == 4) u8g2.print("Vibrato");
          else if(localWAVE == 1) u8g2.print("Echo");
          else u8g2.print("None");
          break;
        case 2:
          u8g2.print("Tremolo");
          break;
        default:
          u8g2.print("ERROR!");
      }

      u8g2.print(" - ");

      switch(localFIR){
        case 0:
          u8g2.print("None");
          break;
        case 1:
          u8g2.print("Smooth");
          break;
        default:
          u8g2.print("ERROR!");
      }

    xSemaphoreGive(keyArrayMutex);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

}

//----------------------------------------------------------------------------------

//Below code is used for testing purposes i.e measuring execution time of different tasks.
//Each task is run 32 times.

#else
void msgOutTask(void *pvParameters) {
  char outMsg[4];
  uint32_t startTime = micros();
  int counter = 0;
  while(1) {
    counter++;
    xQueueReceive(msgOutQ, outMsg, portMAX_DELAY);
    Serial.println(outMsg);
    if(counter == 8) break;
  }
  uint32_t totalTime = micros() - startTime;
  Serial.print("Total time for msgOutTask (in microseconds): ");
  Serial.println(totalTime);
  Serial.print("Execution time per task (in microseconds): ");
  Serial.println(totalTime/32);
  //while (1) ; //do nothing
}

void msgInTask(void *pvParameters){

  const TickType_t xFrequency = 5/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  char inMsg[] = "xxx";
  int count = 0;
  uint32_t startTime = micros();
  //Run task 32 times
  for (int i = 0; i < 32; i++){

    count = 0;

    while(Serial.available() > 0){

      char value = Serial.read();
      if(count < 3){
        inMsg[count] = value;
        //incoming[count] = value;
        __atomic_store_n(&incoming[count], value, __ATOMIC_RELAXED);
      }
      if(value == '\n'){
        count = 0;

        if(inMsg[0] == 'R'){
          __atomic_store_n(&globalShift, 0, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
        }
        else if(inMsg[0] == 'P'){
              __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
              int shift = inMsg[1] - '0';
              __atomic_store_n(&globalShift, shift - 4, __ATOMIC_RELAXED);
              if(globalShift >= 0){
                  if (inMsg[2] >= '0' && inMsg[2] <= '9'){
                    __atomic_store_n(&currentKey, inMsg[2] - '0', __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey]<< globalShift, __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'A' && inMsg[2] <= 'F'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'A' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] << globalShift, __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'a' && inMsg[2] <= 'f'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'a' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] << globalShift, __ATOMIC_RELAXED);
                  }
              }
              else{
              if (inMsg[2] >= '0' && inMsg[2] <= '9'){
                    __atomic_store_n(&currentKey, inMsg[2] - '0', __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] >> abs(globalShift), __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'A' && inMsg[2] <= 'F'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'A' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] >> abs(globalShift), __ATOMIC_RELAXED);
                  }
                  else if (inMsg[2] >= 'a' && inMsg[2] <= 'f'){
                    __atomic_store_n(&currentKey, inMsg[2] - 'a' + 10, __ATOMIC_RELAXED);
                    __atomic_store_n(&currentStepSize, stepSizes[currentKey] >> abs(globalShift), __ATOMIC_RELAXED);
                  }
              }
        }
      }
      else{
        count++;
      }
    }
  }
  uint32_t totalTime = micros() - startTime;
  Serial.print("Total time for msgInTask (in microseconds): ");
  Serial.println(totalTime);
  Serial.print("Execution time per task (in microseconds): ");
  Serial.println(totalTime/32);
  while (1) vTaskDelayUntil(&xLastWakeTime, xFrequency); //do nothing
}


void scanKeysTask(void * pvParameters) {

  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t rowThreePrev;
  uint8_t rowThreeCurr;
  uint8_t rowFourPrev;
  uint8_t rowFourCurr;

  static int prevPress = NO_KEY;
  //Used to denote whether we have sent a play message to the serial - if so the next message should be release.
  static bool saidRead = 0;

  uint32_t startTime = micros();
  //Run task 32 times
  for (int i = 0; i < 32; i++) {

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); //lock keyArray

    rowThreePrev = keyArray[3];
    rowFourPrev = keyArray[4];

    for(uint8_t i = 0; i < 5; i++){
      setRow(i);
      delayMicroseconds(2);
      keyArray[i] = readCols();
    }

    rowThreeCurr = keyArray[3];
    rowFourCurr = keyArray[4];

//////////////////////// SCANNING KNOB ROTATIONS ////////////////////////

    switch(rowThreePrev & 0b11){
      case 0b00:
        if((rowThreeCurr & 0b11) == 0b01) knob3.add(2);
        else if ((rowThreeCurr & 0b11) == 0b10) knob3.sub(2);
        else if ((rowThreeCurr & 0b11) != (rowThreePrev & 0b11)) knob3.cond_addsub(2);
        break;
      case 0b01:
        if((rowThreeCurr & 0b11) == 0b00) knob3.sub(2);
        else if ((rowThreeCurr & 0b11) == 0b11) knob3.add(2);
        else if ((rowThreePrev & 0b11) != (rowThreeCurr & 0b11)) knob3.cond_addsub(2);
        break;
      case 0b10:
        if ((rowThreeCurr & 0b11) == 0b00) knob3.add(2);
        else if ((rowThreeCurr & 0b11) == 0b11) knob3.sub(2);
        else if ((rowThreePrev & 0b11) != (rowThreeCurr & 0b11)) knob3.cond_addsub(2);
        break;
      case 0b11:
        if ((rowThreeCurr & 0b11) == 0b01) knob3.add(2);
        else if ((rowThreeCurr & 0b11) == 0b10) knob3.sub(2);
        else if ((rowThreePrev & 0b11) != (rowThreeCurr & 0b11)) knob3.cond_addsub(2);
        break;
      default:
        break;
    }

    switch(rowThreePrev >> 2 & 0b11){
      case 0b00:
        if((rowThreeCurr >> 2 & 0b11) == 0b01) knob2.add(1);
        else if((rowThreeCurr >> 2 & 0b11) == 0b10) knob2.sub(1);
        else if((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      case 0b01:
        if ((rowThreeCurr >> 2 & 0b11) == 0b00) knob2.sub(1);
        else if ((rowThreeCurr >> 2 & 0b11) == 0b11) knob2.add(1);
        else if ((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      case 0b10:
        if((rowThreeCurr >> 2 & 0b11) == 0b00) knob2.add(1);
        else if((rowThreeCurr >> 2 & 0b11) == 0b11) knob2.sub(1);
        else if((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      case 0b11:
        if((rowThreeCurr >> 2 & 0b11) == 0b01) knob2.add(1);
        else if((rowThreeCurr >> 2 & 0b11) == 0b10) knob2.sub(1);
        else if((rowThreePrev >> 2 & 0b11) != (rowThreeCurr >> 2 & 0b11)) knob2.cond_addsub(1);
        break;
      default:
        break;
    }

    switch(rowFourPrev & 0b11){
      case 0b00:
        if((rowFourCurr & 0b11) == 0b01) knob1.add(1);
        else if((rowFourCurr & 0b11) == 0b10) knob1.sub(1);
        else if ((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      case 0b01:
        if((rowFourCurr & 0b11) == 0b00) knob1.sub(1);
        if((rowFourCurr & 0b11) == 0b11) knob1.add(1);
        else if((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      case 0b10:
        if((rowFourCurr & 0b11) == 0b00) knob1.add(1);
        if((rowFourCurr & 0b11) == 0b11) knob1.sub(1);
        else if((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      case 0b11:
        if((rowFourCurr & 0b11) == 0b01) knob1.add(1);
        if((rowFourCurr & 0b11) == 0b10) knob1.sub(1);
        else if((rowFourPrev & 0b11) != (rowFourCurr & 0b11)) knob1.cond_addsub(1);
        break;
      default:
        break;
    }

    switch(rowFourPrev >> 2 & 0b11){
      case 0b00:
        if((rowFourCurr >> 2 & 0b11) == 0b01) knob0.add(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b10) knob0.sub(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;
      case 0b01:
        if((rowFourCurr >> 2 & 0b11) == 0b00) knob0.sub(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b11) knob0.add(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;
      case 0b10:
        if((rowFourCurr >> 2 & 0b11) == 0b00) knob0.add(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b11) knob0.sub(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;
      case 0b11:
        if((rowFourCurr >> 2 & 0b11) == 0b01) knob0.add(1);
        else if((rowFourCurr >> 2 & 0b11) == 0b10) knob0.sub(1);
        else if((rowFourPrev >> 2 & 0b11) != (rowFourCurr >> 2 & 0b11)) knob0.cond_addsub(1);
        break;

      default:
        break;
    }

//////////////////////// END OF SCANNING KNOB ROTATIONS ////////////////////////

//////////////////////// SCANNING KEY PRESSES ////////////////////////.

    bool keyPress = 0;
    int count = 0;
    uint8_t zero = keyArray[0];


    for(int i = 0; i<KEYS; i++){
        pressed[i] = 1;
      }

    uint8_t testing = 255;

    if(!(testing & 0b1)){
        pressed[0] = 1;
        currentKey = 0;
        keyPress = 1;
        count++;
    }
    if(!(testing>>1 & 0b1)){
        pressed[1] = 1;
        currentKey = 1;
        keyPress = 1;
        count++;
    }
    if(!(testing>>2 & 0b1)){
        pressed[2] = 1;
        currentKey = 2;
        keyPress = 1;
        count++;
    }
    if(!(testing>>3 & 0b1)){
        pressed[3] = 1;
        currentKey = 3;
        keyPress = 1;
        count++;
    }

    if(!(testing & 0b1)){
        pressed[4] = 1;
        currentKey = 4;
        keyPress = 1;
        count++;
    }
    if(!(testing>>1 & 0b1)){
        pressed[5] = 1;
        currentKey = 5;
        keyPress = 1;
        count++;
    }
    if(!(testing>>2 & 0b1)){
        pressed[6] = 1;
        currentKey = 6;
        keyPress = 1;
        count++;
    }
    if(!(testing>>3 & 0b1)){
        pressed[7] = 1;
        currentKey = 7;
        keyPress = 1;
        count++;
    }

    if(!(testing & 0b1)){
        pressed[8] = 1;
        currentKey = 8;
        keyPress = 1;
        count++;
    }
    if(!(testing>>1 & 0b1)){
        pressed[9] = 1;
        currentKey = 9;
        keyPress = 1;
        count++;
    }
    if(!(testing>>2 & 0b1)){
        pressed[10] = 1;
        currentKey = 10;
        keyPress = 1;
        count++;
    }
    if(!(testing>>3 & 0b1)){
        pressed[11] = 1;
        currentKey = 11;
        keyPress = 1;
        count++;
    }

//////////////////////// ENF OF SCANNING KEY PRESSES ////////////////////////

    if(!keyPress && saidRead && prevPress == currentKey){
      noteMessage[0] = 'R';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[prevPress];
      saidRead = 0;
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    if(keyPress && prevPress != currentKey){ //
      incoming[0] = 'R'; //this is to interrupt an incoming message with your own keys
      globalShift = 0; //don't want to shift the sound if playing from main board
      noteMessage[0] = 'P';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[currentKey];
      prevPress = currentKey;
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
      saidRead = 1;
    }
    if(keyPress && noteMessage[0] == 'R' && prevPress == currentKey){
      noteMessage[0] = 'P';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[currentKey];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    else if(!keyPress && noteMessage[0] == 'P'){
      noteMessage[0] = 'R';
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[currentKey];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }

//    for (i = 0; i <8; i++)
//    {
//        noteMessage[0] = 'P';
//        noteMessage[1] = '4';
//        noteMessage[2] = intToHex[i];
//        xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
//    }
    

    xSemaphoreGive(keyArrayMutex);

    if(!keyPress && (__atomic_load_n(&incoming[0], __ATOMIC_RELAXED) == 'R'  || __atomic_load_n(&incoming[0], __ATOMIC_RELAXED) == 'x')){
      currentKey = NO_KEY;
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    }
    else if(__atomic_load_n(&globalShift, __ATOMIC_RELAXED) >= 0){
      __atomic_store_n(&currentStepSize, stepSizes[currentKey]<<globalShift, __ATOMIC_RELAXED);
    }
    else{
      __atomic_store_n(&currentStepSize, stepSizes[currentKey]>>abs(globalShift), __ATOMIC_RELAXED);
    }
  }

  uint32_t totalTime = micros() - startTime;
  Serial.print("Total time for scanKeysTask (in microseconds): ");
  Serial.println(totalTime);
  Serial.print("Execution time per task (in microseconds): ");
  Serial.println(totalTime/32);
  while (1) vTaskDelayUntil(&xLastWakeTime, xFrequency); //do nothing
}

void displayUpdateTask(void * pvParameters){

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();
  char* key[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "None"};

  uint32_t startTime = micros();
  //Run task 32 times.
  for (int i = 0; i < 32; i++){

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font u8g2_font_ncenB08_tr
    int localXATO = __atomic_load_n(&XATOCODE, __ATOMIC_RELAXED);
    int localWAVE = __atomic_load_n(&WAVECODE, __ATOMIC_RELAXED);
    int localFIR = __atomic_load_n(&FIR, __ATOMIC_RELAXED);
    
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); //lock keyArray

    //Display Volume
    u8g2.setCursor(2,10);
    u8g2.print("Volume: ");
    u8g2.print(knob3.get_rotation());

    //Then display the note(s) played if any
    u8g2.setCursor(2,20);
    u8g2.print("Key: ");

    switch(localWAVE){
      case 0:
      {
        bool anyPressed = 0;
        for(int i = 0; i < KEYS; i++){
          if(pressed[i] == 1){
            u8g2.print(key[i]);
            anyPressed = 1;
          }
        }
        if(!anyPressed) u8g2.print(key[NO_KEY]);
        break;
      }
      case 1:
        u8g2.print(key[currentKey]);
        break;
      case 2:
        u8g2.print(key[currentKey]);
        break;
      case 3:
        u8g2.print(key[currentKey]);
        break;
      case 4:
      {
        bool anyPressed = 0;
        for(int i = 0; i < KEYS; i++){
          if(pressed[i] == 1){
            u8g2.print(key[i]);
            anyPressed = 1;
          }
        }
        if(!anyPressed) u8g2.print(key[NO_KEY]);
        break;
      }
      default:
        u8g2.print("ERROR!");
    }

    //Print waveform.
    u8g2.print(" - ");
    switch(localWAVE){
      case 0:
        u8g2.print("Saw");
        break;
      case 1:
        u8g2.print("Sine");
        break;
      case 2:
        u8g2.print("Square");
        break;
      case 3:
        u8g2.print("Expo");
        break;
       case 4:
        u8g2.print("Triangle");
        break;
      default:
        u8g2.print("ERROR!");
    }

    //Print effects.
    u8g2.setCursor(2,30);
    u8g2.print("Effect:");

     switch(localXATO){
        case 0:
          u8g2.print("None");
          break;
        case 1:
          if(localWAVE == 0 || localWAVE == 4) u8g2.print("Vibrato");
          else if(localWAVE == 1) u8g2.print("Echo");
          else u8g2.print("None");
          break;
        case 2:
          u8g2.print("Tremolo");
          break;
        default:
          u8g2.print("ERROR!");
      }

      u8g2.print(" - ");

      switch(localFIR){
        case 0:
          u8g2.print("None");
          break;
        case 1:
          u8g2.print("Smooth");
          break;
        default:
          u8g2.print("ERROR!");
      }

    xSemaphoreGive(keyArrayMutex);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  uint32_t totalTime = micros() - startTime;
  Serial.print("Total time for displayUpdateTask (in microseconds): ");
  Serial.println(totalTime);
  Serial.print("Execution time per task (in microseconds): ");
  Serial.println(totalTime/32);
  while (1) vTaskDelayUntil(&xLastWakeTime, xFrequency); //do nothing

}
#endif

void loop() {

}
