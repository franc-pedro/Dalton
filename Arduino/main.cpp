#include <Arduino.h>
#include "Pixy2.h"
#include "PIDLoop.h"
#include "DualVNH5019MotorShieldMega.h"
#include <Adafruit_NeoPixel.h>

#define PIN 6
#define NUMPIXELS 1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(
    NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


#define MAX_TRANSLATE_VELOCITY  250

Pixy2 pixy;

PIDLoop panLoop(350, 0, 600, true);
PIDLoop tiltLoop(500, 0, 700, true);
PIDLoop rotateLoop(300, 600, 300, false);
PIDLoop translateLoop(400, 800, 300, false);

DualVNH5019MotorShield md;

#define MOTOR_VEL_CONST 150

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setLed(int r, int g, int b) // function to set NEO Pixel LED
{
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(r, g, b)); // set pixel color
  pixels.show(); // show pixel color
}

void setup() {
  Serial.begin(115200);
  Serial.print("Starting...\n");
  // need to initialize pixy object
  pixy.init();
  // user color connected components program
  pixy.changeProg("color_connected_components");

  md.init();

  pixels.begin();
  pixels.clear();
  //pixels.setBrightness(50);
  //setLed(125, 255, 0); 
}

// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
// and return its index, otherwise return -1
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age>30)
    return pixy.ccc.blocks[0].m_index;

  return -1;
}

// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
Block *trackBlock(uint8_t index)
{
  uint8_t i;

  for (i=0; i<pixy.ccc.numBlocks; i++)
  {
    if (index==pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }

  return NULL;
}



void loop() {
  static int16_t index = -1;
  int32_t panOffset, tiltOffset, headingOffset, left, right;
  Block *block=NULL;
  
  pixy.ccc.getBlocks();

  if (index==-1) // search....
  {
    setLed(0,150,0);
    //Serial.println("Searching for block...");
    index = acquireBlock();
    if (index>=0)
      Serial.println("Found block!");
 }
  // If we've found a block, find it, track it
  if (index>=0)
     block = trackBlock(index);

  // If we're able to track it, move motors
  if (block)
  {
    // calculate pan and tilt errors
    panOffset = (int32_t)pixy.frameWidth/2 - (int32_t)block->m_x;
    tiltOffset = (int32_t)block->m_y - (int32_t)pixy.frameHeight/2;  

    // calculate how to move pan and tilt servos
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // move servos
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);

    // calculate translate and rotate errors
    panOffset += panLoop.m_command - PIXY_RCS_CENTER_POS;
    tiltOffset += tiltLoop.m_command - PIXY_RCS_CENTER_POS - PIXY_RCS_CENTER_POS/2 + PIXY_RCS_CENTER_POS/8;

    rotateLoop.update(panOffset);
    translateLoop.update(-tiltOffset);

    // keep translation velocity below maximum
    if (translateLoop.m_command>MAX_TRANSLATE_VELOCITY)
      translateLoop.m_command = MAX_TRANSLATE_VELOCITY;

    // calculate left and right wheel velocities based on rotation and translation velocities
    left = -rotateLoop.m_command + translateLoop.m_command + MOTOR_VEL_CONST;
    right = rotateLoop.m_command + translateLoop.m_command + MOTOR_VEL_CONST;

    setLed(150,0,0);
    // set wheel velocities
    md.setM1Speed(left);
    md.setM2Speed(right);    
    /*Serial.print(left);
    Serial.print(" ------ ");
    Serial.println(left);*/
    // print the block we're tracking -- wait until end of loop to reduce latency
    block->print();
  }  
  else // no object detected, stop motors, go into search state
  {
    rotateLoop.reset();
    translateLoop.reset();
    md.setBrakes(0,0);    
    index = -1; // set search state
  }
}

