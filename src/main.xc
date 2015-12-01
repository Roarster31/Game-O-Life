// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"
#include <print.h>
#include <stdlib.h>

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0]: port p_scl = XS1_PORT_1E;         //interface ports to accelerometer
on tile[0]: port p_sda = XS1_PORT_1F;

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for accelerometer
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6
#define SW1_CODE 0xE
#define SW2_CODE 0xD

//colour codes
#define C_OFF 0x0
#define C_OFF_S 0x1
#define C_BLUE 0x2
#define C_BLUE_S 0x3
#define C_GREEN 0x4
#define C_GREEN_S 0x5
#define C_LGTBLUE 0x6
#define C_LGTBLUE_S 0x7
#define C_ORANGE 0x8
#define C_ORANGE_S 0x9
#define C_PURPLE 0xA
#define C_PURPLE_S 0xB
#define C_YELLOW 0xC
#define C_YELLOW_S 0xD
#define C_WHITE 0xE
#define C_WHITE_S 0xF

on tile[0] : in port buttons = XS1_PORT_4E; //port to access xCore-200 buttons
on tile[0] : out port leds = XS1_PORT_4F;   //port to access xCore-200 LEDs

/////////////////////////////////////////////////////////////////////////////////////////
//
//  Helper Functions provided for you
//
/////////////////////////////////////////////////////////////////////////////////////////


typedef interface LEDInterface {
  void setSeparate(int enabled);
  void setMain(int currentRed, int currentGreen, int currentBlue);
} LEDInterface;

//DISPLAYS an LED pattern
int showLEDs(out port p, server LEDInterface l_interface[n], unsigned n) {
  int pattern; //1st bit...separate green LED
               //2nd bit...blue LED
               //3rd bit...green LED
               //4th bit...red LED
  while (1) {
      int separateEnabled = 0;
      int currentRed = 0;
      int currentGreen = 0;
      int currentBlue = 0;
      select{
          case l_interface[int j].setSeparate(int enabled):
              break;
          case l_interface[int j].setMain(int currentRed, int currentGreen, int currentBlue):
              break;
      }


    pattern = (separateEnabled * 8) | (currentRed * 4) | (currentGreen * 2) | currentBlue;
    p <: pattern;                //send pattern to LED port
  }
  return 0;
}

typedef interface ButtonInterface {
  void showInterest();
} ButtonInterface;

//READ BUTTONS and send button pattern to userAnt
void buttonListener(in port b, chanend outChan[n], unsigned n, server ButtonInterface b_interface[m], unsigned m) {
  int r;
  int enabledChannels[2]; //remember to use the same value as used in main because xc is a piece of crap :)
  for(int i=0; i<n; i++) {
      enabledChannels[i] = 0;
  }

  while (1) {

      select {
          case b_interface[int j].showInterest():
                  enabledChannels[j] = 1;
                  break;
          case b when pinsneq(15) :> r:    // check if some buttons are pressed
              if ((r==13) || (r==14)) {    // if either button is pressed
                  for(int i=0; i<n; i++) {
                      if (enabledChannels[i]) {
                          outChan[i] <: r;             // send button pattern to outChan
                          enabledChannels[i] = 0;
                      }
                  }
              }
              break;
      }

  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out) {
  int res;
  uchar line[ IMWD ];
  printf( "DataInStream: Start...\n" );

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
      printf( "-%4.1d ", line[ x ] ); //show image values
    }
    printf( "\n" );
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream:Done...\n" );
  return;
}

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

uchar encode(int intermediary) {
    uchar output;
    if(intermediary == 0) {
        output = 0x00;
    } else {
        output = 0xFF;
    }
    return output;
}

unsafe int getItem(int * unsafe inArray, int x, int y) {
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



    return inArray[y * IMWD + x];
}

unsafe void setItem(int * unsafe inArray, int x, int y, int value) {
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

    inArray[y * IMWD + x] = value;
}

unsafe int makeDecision(int * unsafe arr, int startX, int startY) {

    int liveNeighbours = 0;
    for( int y = startY - 1; y <= startY + 1; y++ ) {   //go through all lines
        for( int x = startX - 1; x <= startX + 1; x++ ) { //go through each pixel per line
          if (!(x == startX && y == startY)) {
              liveNeighbours += getItem(arr, x, y);
          }
        }
    }

    int live = getItem(arr, startX, startY);

    if (live) {
        if (liveNeighbours < 2) {
//          any live cell with fewer than two live neighbours dies
//            printf("Live cells dies [%d is fewer than 2 live neighbours] (%d,%d)\n", liveNeighbours, startX, startY);
            return 0;
        } else if (liveNeighbours > 3) {
//          any live cell with more than three live neighbours dies
//            printf("Live cells dies [%d is more than 3 live neighbours] (%d,%d)\n", liveNeighbours, startX, startY);
            return 0;
        } else {
//          any live cell with two or three live neighbours is unaffected
//            printf("Live cells unaffected (%d,%d)\n", startX, startY);
            return 1;
        }

    } else {
            if (liveNeighbours == 3){
//          any dead cell with exactly three live neighbours becomes alive
//            printf("dead cells lives (%d,%d)\n", startX, startY);
            return 1;
        }
    }

    return live;
}


typedef interface ExportInterface {
    int isExporting();
    void setCurrentArraypointer(int * unsafe currentDataPointer);
} ExportInterface;

unsafe void exportServer(chanend c_out, chanend fromButton, client ButtonInterface buttonInterface, server ExportInterface exportInterface, client LEDInterface l_interface) {
  int currentState = 0;
  int * unsafe currentDataPointer;
  while(1) {
      buttonInterface.showInterest();
      select {
          case exportInterface.isExporting() -> int returnVal:
              printf("exporting query received, sending %d\n",currentState);
              returnVal = currentState;
              break;
          case exportInterface.setCurrentArraypointer(int * unsafe dataPointer):
                currentDataPointer = dataPointer;
                  break;
          case fromButton :> int buttonType:
              if(buttonType == SW2_CODE) {
                  printf("Received export request\n");

                  currentState = 1;
                  printf("starting export process\n");
                  c_out <: '0';
                  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
                       for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
                           int pointerValue = currentDataPointer[x * IMWD + y];
                           uchar value = encode(pointerValue); // outputs inverted value
//                           printf("%d",value);
                           c_out <: value;
                       }
//                       printf("\n");
                  }
                  printf("ending export process\n");
                  currentState = 0;

                  //with UX in mind we'll light up the LED now
                  l_interface.setMain(0, 0, 1);
              }
              buttonInterface.showInterest();
              break;
      }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
unsafe void distributor(chanend c_in, chanend fromAcc, chanend fromButton, client ButtonInterface buttonInterface, client LEDInterface l_interface, client ExportInterface exportInterface) {
  uchar val;

  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage:Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for SW1 click...\n" );

  //wait for SW1 to be pressed
  int buttonData = 0;

  buttonInterface.showInterest();
  while (buttonData != SW1_CODE) {
      fromButton :> buttonData;
  }

  l_interface.setMain(0,1,0);

  int inArray[IMWD * IMHT];
  int outArray[IMWD * IMHT];

  //Read in and do something with your image values..
  //This just inverts every pixel, but you should
  //change the image according to the "Game of Life"
  printf( "Processing...\n" );
  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
    for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
      c_in :> val;
      setItem(inArray, x, y, decode(val)); //reads in intermediate
    }
  }

  int * unsafe inArrayPointer = inArray;
  int * unsafe outArrayPointer = outArray;



          int i = 0;
          while (1) {
              i++;

              exportInterface.setCurrentArraypointer(inArrayPointer);

              l_interface.setSeparate(i % 2);

          //we do all our processing
          for( int y = 0; y < IMHT; y++ ) {   //go through all lines
              for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
                  setItem(outArrayPointer, x, y, makeDecision(inArrayPointer, x, y));
              }
            }




          printf("\n\n------------ round %d ------------\n\n", i);

          for( int y = 0; y < IMHT; y++ ) {   //go through all lines
               for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
                 int gotItem = getItem(outArrayPointer, x, y);
                 printf("%d",gotItem);
               }
               printf("\n");
             }

          printf("done printing round\n");
          while(exportInterface.isExporting()) {
                        //export
              printf("waiting\n");
          }

          printf("beginning pointer swap\n");
          int * unsafe swap = inArrayPointer;
          inArrayPointer = outArrayPointer;
          outArrayPointer = swap;


          }







  printf( "\nOne processing round completed...\n" );
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in) {
  int res;
  uchar line[ IMWD ];



  while(1) {

      c_in :> line[0];

      //Open PGM file
        printf( "DataOutStream:Start...\n" );
        res = _openoutpgm( outfname, IMWD, IMHT );
        if( res ) {
          printf( "DataOutStream:Error opening %s\n.", outfname );
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
      printf( "DataOutStream:Done...\n" );
  }
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read accelerometer, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
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

    //send signal to distributor after first tilt
    if (!tilted) {
      if (x>30) {
        tilted = 1 - tilted;
        toDist <: 1;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
unsafe int main(void) {

  i2c_master_if i2c[1];               //interface to accelerometer

//  char infname[] = "test.pgm";     //put your input image path here
//  char outfname[] = "testout.pgm"; //put your output image path here
  chan c_inIO, c_outIO, c_control;    //extend your channel definitions here

  chan c_buttons[2];

  interface LEDInterface l_interface[2];
  interface ButtonInterface b_interface[2];
  interface ExportInterface exportInterface;
  par {
      on tile[0]: buttonListener(buttons, c_buttons, 2, b_interface, 2);
      on tile[0]: showLEDs(leds, l_interface, 2);
      on tile[0]: i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing accelerometer data
      on tile[1]: accelerometer(i2c[0],c_control);        //client thread reading accelerometer data
      on tile[1]: DataInStream("test.pgm", c_inIO);          //thread to read in a PGM image
      on tile[1]: DataOutStream("testout.pgm", c_outIO);       //thread to write out a PGM image
      on tile[1]: exportServer(c_outIO, c_buttons[0], b_interface[0], exportInterface, l_interface[0]);
      on tile[1]: distributor(c_inIO, c_control, c_buttons[1], b_interface[1], l_interface[1], exportInterface);//thread to coordinate work on image
  }

  return 0;
}
