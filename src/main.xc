#include <platform.h>
#include <xs1.h>
#include "usb.h"
#include "i2c.h"
#include "xud_cdc.h"

#include <stdio.h>
#include "pgmIO.h"
#include <print.h>
#include <stdlib.h>
#include <limits.h>
#include <safestring.h>

#define  IMHT 16
#define  IMWD 16

#define  WORKER_THREADS 4

typedef unsigned char uchar;

on tile[0]: port p_scl = XS1_PORT_1E;
on tile[0]: port p_sda = XS1_PORT_1F;

#define FXOS8700EQ_I2C_ADDR 0x1E
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1

#define SW1_CODE 0xE
#define SW2_CODE 0xD

/* PORT_4A connected to the 4 LEDs */
on tile[0]: port p_led = XS1_PORT_4F;

/* PORT_4C connected to the 2 Buttons */
on tile[0]: port p_button = XS1_PORT_4E;

/* USB Endpoint Defines */
#define XUD_EP_COUNT_OUT   2    //Includes EP0 (1 OUT EP0 + 1 BULK OUT EP)
#define XUD_EP_COUNT_IN    3    //Includes EP0 (1 IN EP0 + 1 INTERRUPT IN EP + 1 BULK IN EP)

/* serial command codes and strings */
char CLEAR_SCREEN_CODE[4] = {"[2J"};
unsigned CLEAR_SCREEN_CODE_LENGTH = 4;

char HOME_CURSOR_CODE[3] = {"[H"};
unsigned HOME_CURSOR_CODE_LENGTH = 3;

char NEW_LINE_CODE[4] = {"\r\n"};
unsigned NEW_LINE_LENGTH = 4;

char EXPORT_START_MSG[13] = {"exporting..."};
unsigned EXPORT_START_MSG_LENGTH = 13;

char EXPORT_END_MSG[16] = {"export finished"};
unsigned EXPORT_END_MSG_LENGTH = 16;

char DATA_NOT_READ_ERMSG[49] = {"You must read in data before using any commands!"};
unsigned DATA_NOT_READ_ERMSG_LENGTH = 49;

char EXIT_SCREEN_MSG[20] = {"Press 'e' to exit"};
unsigned EXIT_SCREEN_MSG_LENGTH = 20;

char SERIAL_MENU[7][50] = {
        {"Game Of Life\r\n"},
        {"-------------------------\r\n"},
        {"Press a key to continue:\r\n"},
        {"i -> print a live image view\r\n"},
        {"r -> print a live status report \r\n"},
        {"s -> export the current game image \r\n"},
        {"-------------------------\r\n"},
};

unsigned SERIAL_MENU_LENGTH = 50;
unsigned SERIAL_MENU_ROWS = 7;

char SERIAL_REPORT_ROUND_ARR[150];
char SERIAL_REPORT_LIVE_CELLS_ARR[150];
char SERIAL_REPORT_BOUNDING_BOX_ARR[150];
char SERIAL_REPORT_PROCESSING_TIME_ARR[150];
char SERIAL_REPORT_PROCESSING_THROUGHPUT_ARR[150];

unsigned SERIAL_REPORT_LENGTH = 150;

//whether to skip the initial button press when loading image
static unsigned QUICK_START_FLAG = 0;

/*helper functions */

int min(int num1, int num2) {
    return num1 < num2 ? num1 : num2;
}

int max(int num1, int num2) {
    return num1 > num2 ? num1 : num2;
}

typedef struct BoundingBox {
   int   left;
   int   right;
   int   top;
   int   bottom;
} BoundingBox;

typedef interface LEDInterface {
  void setSeparate(int enabled); //set the seperate LED on or off
  void setColour(int currentRed, int currentGreen, int currentBlue); //set the values of the red, green and blue LEDs
} LEDInterface;

/**
 * Sets the LEDs using the given values
 */
void setLEDPattern(out port p, int seperateEn, int red, int green, int blue) {
    //1st bit: separate, 2nd bit: blue, 3rd bit: green, 4th bit: red
    p <: (seperateEn ) | (blue * 2) | (green * 4) | (red * 8);
}

/**
 * Controls the LEDs through the LEDInterface
 */
[[distributable]]
void showLEDs(out port p, server LEDInterface l_interface[n], unsigned n) {

  int separateEnabled = 0;
  int currentRed = 0;
  int currentGreen = 0;
  int currentBlue = 0;

  while (1) {

      select{
          case l_interface[int j].setSeparate(int enabled):
                  separateEnabled = enabled;
                  setLEDPattern(p, separateEnabled, currentRed, currentGreen, currentBlue);
              break;
          case l_interface[int j].setColour(int red, int green, int blue):
                  currentRed = red;
                  currentGreen = green;
                  currentBlue = blue;
                  setLEDPattern(p, separateEnabled, currentRed, currentGreen, currentBlue);
              break;
      }

  }
}

typedef interface ButtonInterface {
  void showInterest(); //used to indicate that a channel is waiting for a button click
} ButtonInterface;

/**
 * Listens for button clicks and calls any channels that have registered interest.
 */
void buttonListener(in port button_port, chanend outChan[n], server ButtonInterface b_interface[n], unsigned n) {
  int button_val;
  int enabledChannels[2];
  for(int i=0; i<n; i++) {
      enabledChannels[i] = 0;
  }

  while (1) {

      select {
          case b_interface[int j].showInterest():
                  enabledChannels[j] = 1;
                  break;
          case button_port when pinsneq(15) :> button_val:
              if ((button_val==13) || (button_val==14)) {
                  for(int i=0; i<n; i++) {
                      //let all interested channels know
                      if (enabledChannels[i]) {
                          outChan[i] <: button_val;
                          enabledChannels[i] = 0;
                      }
                  }
              }
              break;
      }

  }
}

/**
 * Reads in a pgm file and then waits to write out a PGM file
 */
void DataManager(char infname[], char outfname[], chanend c_in, chanend c_out) {
  int res;
  uchar line[ IMWD ];

  //Open PGM file
  res = _openinpgm( infname, IMWD, IMHT );
  if( res ) {
    printf( "DataInStream: Error openening %s\n.", infname );
    return;
  }

  //Read image line-by-line and send byte by byte to channel c_out
  for( int y = 0; y < IMHT; y++ ) {

    _readinline( line, IMWD );

    for( int x = 0; x < IMWD; x++ ) {

      c_out <: line[ x ];

    }
  }


  //Close PGM image file
  _closeinpgm();

  //now prepare to write out

  while(1) {
      c_in :> line[0];

      //Open PGM file
      printf( "Beginning file write\n" );
      res = _openoutpgm( outfname, IMWD, IMHT );
      if( res ) {
          printf( "Error opening %s\n.", outfname );
          return;
      }

      //Compile each line of the image and write the image line-by-line
      for( int y = 0; y < IMHT; y++ ) {
          for( int x = 0; x < IMWD; x++ ) {
              c_in :> line[ x ];
          }
          _writeoutline( line, IMWD );
      }

      //Close the PGM image
      _closeoutpgm();
      printf( "Finished file write\n" );
  }
  return;
}

/**
 * Converts from PGM uchars to our internal integer representation
 */
int decode(uchar input) {
    //convert from pgm to pbm
    unsigned int intermediary;
    if(input == 0) {
        intermediary = 0;
    } else {
        intermediary = 1;
    }

    return intermediary;
}

/**
 * Converts from our internal integer representation to PGM uchars
 */
uchar encode(int intermediary) {
    uchar output;
    if(intermediary == 0) {
        output = 0x00;
    } else {
        output = 0xFF;
    }
    return output;
}

/**
 * Ensures that the coordinate pair is within the image's bounds.
 * If not, it updates the pair to wrap around.
 */
void ensureBoundedCoordinates(int &x, int &y) {
    while (x < 0) {
        x += IMWD;
    }
    while (x >= IMWD) {
        x -= IMWD;
    }
    while (y < 0) {
        y += IMHT;
    }
    while (y >= IMHT) {
        y -= IMHT;
    }
}

/**
 * Returns the value at the given coordinates in the internal array representation of the image.
 */
unsafe int getItem(uchar * unsafe inArray, int x, int y) {
    ensureBoundedCoordinates(x, y);

    int elementIndex = y * IMWD + x;
    int ucharIndex = (elementIndex / 8) + 1;
    int indexInUchar = elementIndex % 8;

    return (inArray[ucharIndex] >> indexInUchar) & 1;
}

/**
 * Sets the value at the given coordinates in the internal array representation of the image.
 */
unsafe void setItem(uchar * unsafe inArray, int x, int y, int value) {
    ensureBoundedCoordinates(x, y);

    int elementIndex = y * IMWD + x;
    int ucharIndex = (elementIndex / 8) + 1;
    int indexInUchar = elementIndex % 8;

    inArray[ucharIndex] ^= (-value ^ inArray[ucharIndex]) & (1 << indexInUchar);
}

/**
 * Dtermines whether or not the cell at (startX, startY) will live or die this round
 * based upon its surrounding neighbours.
 */
unsafe int makeDecision(uchar * unsafe arr, int startX, int startY) {

    int liveNeighbours = 0;
    //go through all of the coordinate's neighbours
    for( int y = startY - 1; y <= startY + 1; y++ ) {
        for( int x = startX - 1; x <= startX + 1; x++ ) {
          if (!(x == startX && y == startY)) {
              liveNeighbours += getItem(arr, x, y);
          }
        }
    }

    int live = getItem(arr, startX, startY);

    if (live) {
        if (liveNeighbours < 2) {   /*any live cell with fewer than two live neighbours dies*/
            return 0;
        } else if (liveNeighbours > 3) {    /*any live cell with more than three live neighbours dies*/
            return 0;
        } else {    /*any live cell with two or three live neighbours is unaffected*/
            return 1;
        }
    }else{
        if (liveNeighbours == 3){ /*any dead cell with exactly three live neighbours becomes alive*/
            return 1;
        }
    }

    return live;
}

/**
 * Sends a series of commands across a serial connection to clear the
 * screen connected to the serial port.
 **/
void clearScreen(client interface usb_cdc_interface cdc) {

    cdc.put_char(27); // ESC command

    cdc.write(CLEAR_SCREEN_CODE, CLEAR_SCREEN_CODE_LENGTH); // clear screen

    cdc.put_char(27); // ESC command

    cdc.write(HOME_CURSOR_CODE, HOME_CURSOR_CODE_LENGTH); // set cursor to the beginning again
}

/**
 * Export an image described by currentDataPointer
 */
unsafe void exportImage(chanend c_out, uchar * unsafe currentDataPointer, client LEDInterface l_interface) {
    printf("Received export request\n");

    //with UX in mind we'll light up the LED now
    l_interface.setColour(0, 0, 1);


    printf("starting export process\n");
    c_out <: '0';
    for( int y = 0; y < IMHT; y++ ) {   //go through all lines
         for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
             uchar value = encode(getItem(currentDataPointer, x, y));
             c_out <: value;
         }
    }
    printf("ending export process\n");


    l_interface.setColour(0,0,0);
}

typedef interface ControlInterface {
    void updateState(BoundingBox boundingBox, int currentRound, int liveCells); //update the current game state
    unsigned int getElapsedTime(); //return the total current elapsed time in milliseconds
    void setOutputArrayPointer(uchar * unsafe outputArrayPointer); //set the pointer to last processed round
    void startTiming(); //call to begin timing. This should be called after the file has been read in
} ControlInterface;

/**
 * Acts as a middleman between many of the interfaces and the init.
 */
unsafe void controlServer(chanend c_out, chanend fromButton, client ButtonInterface buttonInterface, server ControlInterface controlInterface, client LEDInterface l_interface,  chanend fromAcc, chanend continueChannel, client interface usb_cdc_interface cdc) {
  int exporting = 0;
  int paused = 0;
  unsigned pointerSet = 0;
  unsigned serialState = 0;
  unsigned usbRowLength = IMWD;

  BoundingBox currentBoundingBox;
  uchar * unsafe currentDataPointer;
  int currentRound;
  int currentLiveCells;

  char usbRow[IMWD];

  timer time;
  unsigned int startTime;
  unsigned int pauseStartTime;
  unsigned int totalLostTime = 0;
  int totalClockCycles = 0;

  unsigned period = 10000000;
  unsigned periodInc = period;
  unsigned lastTime = 0;

  unsigned framePeriod = 10000000;
  unsigned framePeriodInc = framePeriod;

  unsigned deciCounter = 0;

  buttonInterface.showInterest();

  //push to the serial port ready for when someone connects
  for(int i = 0; i < SERIAL_MENU_ROWS; i++) {
     cdc.write(SERIAL_MENU[i], SERIAL_MENU_LENGTH);
  }

  while(1) {

      select {
          case controlInterface.startTiming():
              startTime = 1000 * 42 * totalClockCycles + deciCounter * 100;
              break;
          case controlInterface.getElapsedTime() -> unsigned int elapsed:
              unsigned int currentTime = 1000 * 42 * totalClockCycles + deciCounter * 100;
              elapsed = currentTime - startTime - totalLostTime;
              break;
          case controlInterface.updateState(BoundingBox boundingBox, int round, int liveCells):
              currentBoundingBox = boundingBox;
              currentRound = round;
              currentLiveCells = liveCells;
              break;
          case controlInterface.setOutputArrayPointer(uchar * unsafe outputArrayPointer):
              currentDataPointer = outputArrayPointer;
              pointerSet = 1;
                break;
          case fromButton :> int buttonType:
              if(buttonType == SW2_CODE && !exporting && pointerSet) {
                  exporting = 1;
                  exportImage(c_out, currentDataPointer, l_interface);
                  exporting = 0;
              } else if(buttonType == SW2_CODE && !pointerSet) {
                  printf("Can't export before data is read in!\n");
              }
              //re-register interest
              buttonInterface.showInterest();
              break;
          case fromAcc :> paused:
              if(pointerSet) {
                  if (paused) {
                      pauseStartTime = 1000 * 42 * totalClockCycles + deciCounter * 100;
                      unsigned int elapsedTime =  pauseStartTime - startTime - totalLostTime;
                      printf("----Paused----\n");
                      printf("Round = %d\n", currentRound);
                      printf("Total live cells = %d\n", currentLiveCells);
                      int boundingPixels = (currentBoundingBox.right - currentBoundingBox.left + 3) * (currentBoundingBox.bottom - currentBoundingBox.top + 3);
                      int totalPixels = IMWD * IMHT;
                      printf("Bounding box between points (%d,%d) and (%d,%d) means we're only processing %d out of %d pixels\n",currentBoundingBox.left, currentBoundingBox.top, currentBoundingBox.right, currentBoundingBox.bottom, boundingPixels, totalPixels);
                      printf("Total processing time: %d ms\n", elapsedTime);
                      printf("Total Processing throughput after %d rounds is %u ms per round.\n",currentRound, elapsedTime/currentRound);
                      l_interface.setColour(1, 0, 0);
                  } else {
                      printf("resuming...\n");
                      l_interface.setColour(0, 0, 0);

                      unsigned int endTime = 1000 * 42 * totalClockCycles + deciCounter * 100;
                      totalLostTime += endTime - pauseStartTime;
                  }
              }
              break;
              //if we're not paused or exporting then any calls to continueChannel should go through
          case !paused && !exporting => continueChannel :> int data:
              break;
          case time when timerafter(periodInc) :> unsigned thisTime:

              deciCounter++;
              periodInc += period;

              if(lastTime > thisTime) {
                  deciCounter = 0;
                  totalClockCycles ++;
              }
              lastTime = thisTime;

              //usb drawing section
              if(serialState && !paused && thisTime > framePeriodInc && pointerSet){
                  switch (serialState) {
                  case 2: //live image view
                      framePeriodInc += framePeriod;
                      cdc.write(NEW_LINE_CODE, NEW_LINE_LENGTH);
                      clearScreen(cdc);
                      for( int y = 0; y < IMHT; y++ ) {
                           for( int x = 0; x < IMWD; x++ ) {
                               int baseItem = getItem(currentDataPointer, x, y);
                               uchar value = baseItem == 0 ? '0' : '1';
                               usbRow[x] = value;
                           }
                           cdc.write(usbRow, usbRowLength);
                           cdc.write(NEW_LINE_CODE, NEW_LINE_LENGTH);
                      }

                      cdc.write(EXIT_SCREEN_MSG, EXIT_SCREEN_MSG_LENGTH);
                      break;
                  case 3: //live report
                      clearScreen(cdc);

                      unsigned pauseStartTime = 1000 * 42 * totalClockCycles + deciCounter * 100;
                      unsigned int elapsedTime =  pauseStartTime - startTime - totalLostTime;
                      int boundingPixels = (currentBoundingBox.right - currentBoundingBox.left + 3) * (currentBoundingBox.bottom - currentBoundingBox.top + 3);
                      int totalPixels = IMWD * IMHT;

                      sprintf(SERIAL_REPORT_ROUND_ARR, "Round = %d\r\n", currentRound);
                      cdc.write(SERIAL_REPORT_ROUND_ARR, SERIAL_REPORT_LENGTH);
                      sprintf(SERIAL_REPORT_LIVE_CELLS_ARR, "Total live cells = %d\r\n", currentLiveCells);
                      cdc.write(SERIAL_REPORT_LIVE_CELLS_ARR, SERIAL_REPORT_LENGTH);
                      sprintf(SERIAL_REPORT_BOUNDING_BOX_ARR, "Bounding box between points (%d,%d) and (%d,%d) means we're only processing %d out of %d pixels\r\n",currentBoundingBox.left, currentBoundingBox.top, currentBoundingBox.right, currentBoundingBox.bottom, boundingPixels, totalPixels);
                      cdc.write(SERIAL_REPORT_BOUNDING_BOX_ARR, SERIAL_REPORT_LENGTH);
                      sprintf(SERIAL_REPORT_PROCESSING_TIME_ARR, "Total processing time: %d ms\r\n", elapsedTime);
                      cdc.write(SERIAL_REPORT_PROCESSING_TIME_ARR, SERIAL_REPORT_LENGTH);
                      sprintf(SERIAL_REPORT_PROCESSING_THROUGHPUT_ARR, "Total Processing throughput after %d rounds is %u ms per round.\r\n",currentRound, elapsedTime/currentRound);
                      cdc.write(SERIAL_REPORT_PROCESSING_THROUGHPUT_ARR, SERIAL_REPORT_LENGTH);

                      cdc.write(EXIT_SCREEN_MSG, EXIT_SCREEN_MSG_LENGTH);
                      break;

                  }

              }
              break;
      }

      //receive any messages we get from the serial port
      if(cdc.available_bytes()) {
          uchar charValue = cdc.get_char();
          if(pointerSet) {
              switch (charValue) {
              case 'i': //i(mage)
                  serialState = 2;
                  break;
              case 'r': //r(eport)
                  serialState = 3;
                  break;
              case 's': //s(ave)
                  if(!exporting && pointerSet) {
                      exporting = 1;
                      cdc.write(EXPORT_START_MSG, EXPORT_START_MSG_LENGTH);
                      exportImage(c_out, currentDataPointer, l_interface);
                      cdc.write(EXPORT_END_MSG, EXPORT_END_MSG_LENGTH);
                      exporting = 0;
                  }
                  break;
              case 'e': //e(xit)
              default:
                  serialState = 1;
                  clearScreen(cdc);
                  for(int i = 0; i < SERIAL_MENU_ROWS; i++) {
                      cdc.write(SERIAL_MENU[i], SERIAL_MENU_LENGTH);
                  }
                  break;
              }
          } else {
              clearScreen(cdc);
              for(int i = 0; i < SERIAL_MENU_ROWS; i++) {
                  cdc.write(SERIAL_MENU[i], SERIAL_MENU_LENGTH);
              }
              cdc.write(DATA_NOT_READ_ERMSG, DATA_NOT_READ_ERMSG_LENGTH);
          }
      }
  }
}


unsafe void worker(chanend distribChannel) {
    uchar * unsafe inArrayPointer;
    uchar * unsafe outArrayPointer;
    int startX = 0;
    int startY = 0;
    int endX = 0;
    int endY = 0;

    while(1) {

        distribChannel :> inArrayPointer;
        distribChannel :> outArrayPointer;

        distribChannel :> startX;
        distribChannel :> startY;
        distribChannel :> endX;
        distribChannel :> endY;

        BoundingBox boundingBox;
        boundingBox.left = endX;
        boundingBox.right = startX;
        boundingBox.top = startY;
        boundingBox.bottom = endY;

        unsigned liveCells = 0;
        for(int x = startX; x <= endX; x++){
            for(int y = startY; y <= endY; y++){
                int alive = makeDecision(inArrayPointer, x, y);
                if(alive) {
                    boundingBox.left = min(x, boundingBox.left);
                    boundingBox.right = max(x, boundingBox.right);
                    boundingBox.top = min(y, boundingBox.top);
                    boundingBox.bottom = max(y, boundingBox.bottom);
                }
                liveCells += alive;
                setItem(outArrayPointer, x , y, alive);
            }
        }
        //send back liveCells
        distribChannel <: liveCells;

        //send back bounding box
        distribChannel <: boundingBox;
    }
}

/**
 * Loops through rounds and distributes work to workers until there are no live cells left.
 */
unsafe void distributionLoop(uchar * unsafe inArrayPointer, uchar * unsafe outArrayPointer, chanend workerChannels[n], unsigned n, client ControlInterface controlInterface, client LEDInterface l_interface, chanend continueChannel, BoundingBox boundingBox) {

  int roundNumber = 0;
  BoundingBox roundBoundingBox = boundingBox;
  int totalLiveCells = 1;

  controlInterface.setOutputArrayPointer(inArrayPointer);
  controlInterface.updateState(roundBoundingBox, 0, 0);

  while (totalLiveCells) {
      roundNumber++;

      if(roundNumber == 100){
          unsigned int timeTaken = controlInterface.getElapsedTime();
          printf("Processed the first %d rounds in %u ms\n",roundNumber,timeTaken);
      }

      //alternate LED
      l_interface.setSeparate(roundNumber % 2);

      int completed = 0;
      BoundingBox tempBoundingBox;

      int freeWorkers = n;
      int row = 0;

      //work out a bounding box slightly larger than last round's bounding box (because interactions
      //will take place with cells 1 outside box)
      int top = roundBoundingBox.bottom == (IMHT-1) ? 0 : max(0, roundBoundingBox.top-1);
      int bottom = roundBoundingBox.top == 0 ? IMHT : min(IMHT, roundBoundingBox.bottom+1);
      int left = roundBoundingBox.right == (IMWD-1) ? 0 : max(0, roundBoundingBox.left-1);
      int right = roundBoundingBox.left == 0 ? IMWD : min(IMWD, roundBoundingBox.right+1);

      //send the initial work off until we've either completed all rows
      //or run out of workers.
      while(completed < IMHT && freeWorkers > 0 && row < IMHT) {
          if (row >= top && row <= bottom) {

              int startX = left;
              int startY = row;
              int endX = right;
              int endY = row;
              int targetWorker = n - freeWorkers;

              //the data each worker needs to do its job
              workerChannels[targetWorker] <: inArrayPointer;
              workerChannels[targetWorker] <: outArrayPointer;

              workerChannels[targetWorker] <: startX;
              workerChannels[targetWorker] <: startY;
              workerChannels[targetWorker] <: endX;
              workerChannels[targetWorker] <: endY;
              freeWorkers--;
          } else {
              completed++;
          }
          row++;
      }

      totalLiveCells = 0;
      left = IMWD;
      right = 0;
      top = IMHT;
      bottom = 0;


      //wait for jobs to complete and dish out more if necessary
      while(completed < IMHT) {
          select {
              case workerChannels[int j] :> int liveCells:
              totalLiveCells += liveCells;
              completed++;

              //send back the bounding box, if no live cells then you should ignore
              workerChannels[j] :> tempBoundingBox;

              if(liveCells) {
                  left = min(left, tempBoundingBox.left);
                  right = max(right, tempBoundingBox.right);
                  top = min(top, tempBoundingBox.top);
                  bottom = max(bottom, tempBoundingBox.bottom);
              }

              //see if we have to pause
              continueChannel <: 0;

              if(row < IMHT) {
                  int startX = 0;
                  int startY = row;
                  int endX = IMWD;
                  int endY = row;

                  //the data each worker needs to do its job
                  workerChannels[j] <: inArrayPointer;
                  workerChannels[j] <: outArrayPointer;

                  workerChannels[j] <: startX;
                  workerChannels[j] <: startY;
                  workerChannels[j] <: endX;
                  workerChannels[j] <: endY;
                  row++;
              } else {
                  freeWorkers++;
              }
              break;
          }
      }

      //see if we have to pause (specifically due to exporting at this point)
      continueChannel <: 0;

      uchar * unsafe swap = inArrayPointer;
      inArrayPointer = outArrayPointer;
      outArrayPointer = swap;

      roundBoundingBox.left = left;
      roundBoundingBox.right = right;
      roundBoundingBox.top = top;
      roundBoundingBox.bottom = bottom;

      controlInterface.setOutputArrayPointer(inArrayPointer);
      //update the control interface's current data pointer incase it wants to export what we have so far
      controlInterface.updateState(roundBoundingBox, roundNumber, totalLiveCells);

  }

  printf("No live cells, game stops!!\n");
}


unsafe void init(chanend c_in, chanend fromButton, client ButtonInterface buttonInterface, client LEDInterface l_interface, client ControlInterface controlInterface, chanend continueChannel) {
  uchar val;

  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage:Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for SW1 click...\n" );

  //wait for SW1 to be pressed
  int buttonData = 0;

  if(!QUICK_START_FLAG) {
      while (buttonData != SW1_CODE) {
          buttonInterface.showInterest();
          fromButton :> buttonData;
      }
  }

  l_interface.setColour(0,1,0);

  uchar inArray[(((IMWD * IMHT)/8)+1)];
  uchar outArray[(((IMWD * IMHT)/8)+1)];

  BoundingBox boundingBox;
  boundingBox.left = IMWD;
  boundingBox.right = 0;
  boundingBox.top = IMHT;
  boundingBox.bottom = 0;

  printf( "Processing...\n" );
  int decoded;
  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
    for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
      c_in :> val;

      decoded = decode(val);

      if(decoded) {
          boundingBox.left = min(x, boundingBox.left);
          boundingBox.right = max(x, boundingBox.right);
          boundingBox.top = min(y, boundingBox.top);
          boundingBox.bottom = max(y, boundingBox.bottom);
      }

      //set the out array to all zeros for our sanity
      setItem(outArray, x, y, 0);
      setItem(inArray, x, y, decoded);

    }
  }
  controlInterface.startTiming();

  l_interface.setColour(0,0,0);

  uchar * unsafe inArrayPointer = inArray;
  uchar * unsafe outArrayPointer = outArray;

  chan workerChannels[WORKER_THREADS];

  par {
        distributionLoop(inArrayPointer, outArrayPointer, workerChannels, WORKER_THREADS, controlInterface, l_interface, continueChannel, boundingBox);
        {
            par (int i=0; i<WORKER_THREADS; i++) {
                worker(workerChannels[i]);
            }
        }
      }



}


void accelerometer(client interface i2c_master_if i2c, chanend toDist) {
  i2c_regop_res_t result;
  char status_data = 0;
  int tilted = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the accelerometer x-axis forever
  while (1) {

    //check until new accelerometer data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //send signal to init after first tilt
    if (!tilted) {
      if (x>30) {
        tilted = 1 - tilted;
        toDist <: 1;
      }
    }else{
      if (x<20) {
        tilted = 1 - tilted;
        toDist <: 0;
      }
    }
  }
}


unsafe int main(void) {

  chan c_inIO, c_outIO, c_control, continueChannel;    //extend your channel definitions here

  chan c_buttons[2];

  interface LEDInterface l_interface[2];
  interface ButtonInterface b_interface[2];
  interface ControlInterface controlInterface;

  /* Channels to communicate with USB endpoints */
  chan c_ep_out[XUD_EP_COUNT_OUT], c_ep_in[XUD_EP_COUNT_IN];
  /* Interface to communicate with USB CDC (Virtual Serial) */
  interface usb_cdc_interface cdc_data;
  /* I2C interface */
  i2c_master_if i2c[1];

  par {
      on tile[0]: showLEDs(p_led, l_interface, 2);

      on tile[0]: buttonListener(p_button, c_buttons, b_interface, 2);
      on tile[0]: i2c_master(i2c, 1, p_scl, p_sda, 10);
      on tile[0]: accelerometer(i2c[0],c_control);

      on tile[0]: controlServer(c_outIO, c_buttons[0], b_interface[0], controlInterface, l_interface[0], c_control, continueChannel, cdc_data);
      on tile[0]: init(c_inIO, c_buttons[1], b_interface[1], l_interface[1], controlInterface, continueChannel);//thread to coordinate work on image

      on USB_TILE: xud(c_ep_out, XUD_EP_COUNT_OUT, c_ep_in, XUD_EP_COUNT_IN, null, XUD_SPEED_HS, XUD_PWR_SELF);
      on USB_TILE: Endpoint0(c_ep_out[0], c_ep_in[0]);
      on USB_TILE: CdcEndpointsHandler(c_ep_in[1], c_ep_out[1], c_ep_in[2], cdc_data);
      on USB_TILE: DataManager("test.pgm", "testout.pgm", c_outIO, c_inIO);

  }

  return 0;
}
