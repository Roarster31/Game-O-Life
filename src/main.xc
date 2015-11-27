// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width

typedef unsigned char uchar;      //using uchar as shorthand

port p_scl = XS1_PORT_1E;         //interface ports to accelerometer
port p_sda = XS1_PORT_1F;

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

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out)
{
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

int getItem(int inArray[IMWD * IMHT], int x, int y) {
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

//    printf("returning inArray[%d][%d]\n", x, y);
    return inArray[x * IMWD + y];
}

int makeDecision(int arr[IMWD * IMHT], int startX, int startY) {

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

    return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc)
{
  uchar val;

  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage:Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Board Tilt...\n" );
//  fromAcc :> int value;

  int inArray[IMWD * IMHT];
  int outArray[IMWD * IMHT];

  //Read in and do something with your image values..
  //This just inverts every pixel, but you should
  //change the image according to the "Game of Life"
  printf( "Processing...\n" );
  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
    for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
      c_in :> val;
      inArray[x * IMWD + y] = decode(val); //reads in intermediate
    }
  }

  int *inArrayPointer = inArray;
  int *outArrayPointer = outArray;

  int i = 0;
  while (1) {
      i++;

  //we do all our processing
  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
      for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
          outArrayPointer[x * IMWD + y] = makeDecision(inArrayPointer, x, y);
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

  int * swap = inArrayPointer;
  inArrayPointer = outArrayPointer;
  outArrayPointer = swap;


  }


  //we're all done
  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
       for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
           c_out <: encode(outArray[x * IMWD + y]); // outputs inverted value
       }
     }

  printf( "\nOne processing round completed...\n" );
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{
  int res;
  uchar line[ IMWD ];

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
int main(void) {

  i2c_master_if i2c[1];               //interface to accelerometer

  char infname[] = "test.pgm";     //put your input image path here
  char outfname[] = "testout.pgm"; //put your output image path here
  chan c_inIO, c_outIO, c_control;    //extend your channel definitions here

  par {
    i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing accelerometer data
    accelerometer(i2c[0],c_control);        //client thread reading accelerometer data
    DataInStream(infname, c_inIO);          //thread to read in a PGM image
    DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    distributor(c_inIO, c_outIO, c_control);//thread to coordinate work on image
  }

  return 0;
}
