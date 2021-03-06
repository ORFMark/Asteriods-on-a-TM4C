/*
*  Author: Mark Burrell, Luke Baird
*  Project: CEC322 Lab 8
*  File: Lab_8_Main.c
*  Description: A program that implements a bubble level over the I2C protocol
*               with a built-in MPU9150 sensor.
*  Modified: March 26 2019 LB
*  Repository: https://github.com/ORFMark/CEC322
*/
// 
// This is part of revision 2.1.4.178 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************


/******************************* begin includes ******************************/
#include <stdint.h> 
#include <stdbool.h> //booleans
#include <math.h> //some sine and drawing ops

#include "inc/hw_ints.h" //location of who interrupts on what pin
#include "inc/hw_memmap.h" //location of who communicates at what address

#include "driverlib/debug.h" //error routine
#include "driverlib/fpu.h" //floating point ops in ISR
#include "driverlib/gpio.h" //GPIO config ops
#include "driverlib/pin_map.h" //GPIO pin defines
#include "driverlib/interrupt.h" //interrupt ops
#include "driverlib/sysctl.h" // peripheral lists
#include "driverlib/uart.h" //uart
#include "driverlib/rom.h"
#include "driverlib/i2c.h" //internal i2c op

#include "grlib/grlib.h" //drawing ops


#include "drivers/cfal96x64x16.h" //OLED Driver
#include "drivers/buttons.h" //button enum

/***** includes needed for the MPU ********/
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h" 
/*****end includes needed for the MPU ********/


#include "mrbUtil/cec322util.h" //menu ops
#include "mrbUtil/cec322peripherals.h" //display and configure ops


#include "mrbUtil/data_structs/queue.h" //queue for the UART
#include "mrbUtil/data_structs/buffer.h" //circular buffer for the MPU

#include "lbHeaders/lb_buttons.h"

#include "lbHeaders/graphics/luke_graphics.h"
#include "lbHeaders/graphics/asteroids_graphics.h"

#include "lbHeaders/Game/sprite.h"
#include "lbHeaders/Game/asteroids.h"
#include "lbHeaders/Game/bullet.h"
#include "lbHeaders/Game/spaceship.h"


#define ONE_KHZ_LOAD 80000
#define SIXTYFOUR_FPS 1250000
#define EIGHTY_MHZ
#define M_PI 3.14159265359
#define M_G 9.80665
#define DEBUG_MODE USER_TOGGLE_1
#define DISPLAY_MODE_CYCLER USER_TOGGLE_2
#define ACCELDATA_NORM 1
#define OFF_SCREEN_BUF_SZ GrOffScreen8BPPSize(96,64)

/***************** End Includes ************/

/***************** Prototypes and struct defs *********************************/
typedef enum disp {
  level, game, numDisp, numModes } displayMode;

void enableTimers(void);
void Timer0InterruptHandler(void);
void initI2C();
void UARTIntHandler(void);
void buttonsISR(void);
void MPU9150I2CIntHandler(void);

void MPU9150Callback(void *pvCallbackData, uint_fast8_t ui8Status);
void configureMPU(tI2CMInstance *thisI2C, tMPU9150 *thisMPU, uint8_t adr);
void getMPUDataAccel(tMPU9150 *thisMPU, float* accelArray);
void getMPUDataGyro(tMPU9150 *thisMPU, float* gyroArray);
void debugWriteToOLED(float* accelData, float* gyroData);
void bubbleLevel(float* accelData, Point* location);

void runAsteroids(void);
void initAsterioids(Spaceship *mySpaceship, Asteroid Asteroids[],
                    Asteroid asteroidsSM[],  Bullet bullets[]); 
/************** End Prototypes and struct defs ********************************/

/*begin globals*/
Queue UARTQueue;
uint8_t booleanToggles = START_STATE;
volatile bool g_MPUDone = false;
tContext g_sContext; /* global OLED sContext */
tDisplay g_sOffScreen; /* offscreen buffer in SRAM to reduce flickering */

uint32_t g_pui8Palette[7] = {
  ClrBlack,
  ClrWhite,
  ClrRed,
  ClrGold,
  ClrBlue,
  ClrGreen,
  ClrSilver
};
uint8_t pui8_offscreenbuffer[OFF_SCREEN_BUF_SZ];
/* an a toggle related to the screen overwrites in the offscreen buffer */
#define NUM_PALETTE_ENTRIES sizeof(g_pui8Palette) / sizeof(uint32_t)

tI2CMInstance MPUI2C; /* MPU I2C Global Instance */
displayMode dispMode = level;
tMPU9150 MPU;
float g_accelData[3];
/*This should really be define constants or an enum instead of an array*/
uint8_t asteroidFlags[4] = {0x01, 0x02, 0x04, 0x08}; /* left, right, shoot */
uint8_t asteroidFlagsEnabled = 0x00;
bool shouldClearScreen;
/************************ End Globals ****************************************/

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif



int main(void)
{
  /**** configure the intial op state of the machine ****/
  FPULazyStackingEnable();
#ifdef EIGHTY_MHZ
  /* set the clock to run at 80MHZ */
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_16MHZ);
#else
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_16MHZ);
#endif
  /****end configure the intial op state of the machine ****/
  const char* userToggles[4];
  uint8_t sizes[4];
  const uint8_t prompts = 2;
  userToggles[0] = "Enable Debug Mode";
  userToggles[1] = "Cycle between display modes";
  sizes[0] = 18;
  sizes[1] = 27;
  tMPU9150 MPU;
  float accelData[3];
  float gyroData[3];
  Point bubble;
  bubble.x = 0;
  bubble.y = 0;
  /*color palette array */
  //-----------------------------------------
  
  
  IntMasterDisable();
  
  /* Enable Peripherals*/
  initBlinky();
  configureUART();
  initDisplay(&g_sContext);
  displaySplashAnimated(&g_sContext);
  UARTQueue = newQueue();
  enableTimers();
  configureButtons();
  initI2C();
  /*End Periph Enables*/
  
  /* Configure the off screen buffer */ 
  GrOffScreen8BPPInit(&g_sOffScreen, pui8_offscreenbuffer, 96, 64); 
  GrOffScreen8BPPPaletteSet(&g_sOffScreen, g_pui8Palette, 0 
                            ,NUM_PALETTE_ENTRIES);
  /* end off screen buffer config */
  
  /*Begin Interrupt Config*/
  UARTIntRegister(UART0_BASE, UARTIntHandler);
  UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT);
  IntEnable(INT_UART0);
  IntMasterEnable();
  /*End Interrupt Config*/
  
  /* Enable MPU */
  configureMPU(&MPUI2C, &MPU, 0x69);
  /* End Enable MPU */
  
  shouldClearScreen = true;
  while(!(booleanToggles & QUIT)) {
    //point the drawing context at the off screen buffer

    if (shouldClearScreen == true) { 
      GrContextInit(&g_sContext, &g_sOffScreen);
      clearDisplay(&g_sContext, false);
    }
    getMPUDataAccel(&MPU, g_accelData);
    if(peek(&UARTQueue) != -1) {
      processMenuChar(&booleanToggles, dequeue(&UARTQueue));
    }
    if (booleanToggles & ENABLE_BLINKY) {
      blinky();
    }
    if (booleanToggles & DISPLAY_SPLASH) {
      booleanToggles ^= DISPLAY_SPLASH;
      IntMasterDisable();
      displaySplashAnimated(&g_sContext);
      IntMasterEnable();
    }
    if (booleanToggles & PRINT_MENU) {
      booleanToggles ^= PRINT_MENU;
      printMenu(userToggles, sizes, prompts);
    }
    if (booleanToggles & SCREEN_CLEAR) {
      booleanToggles ^= SCREEN_CLEAR;
      UARTConsolePrint("\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r", 24);
    }
    if (booleanToggles & DEBUG_MODE) {
      booleanToggles ^= DEBUG_MODE;
      dispMode = numDisp;
    }
    if (booleanToggles & DISPLAY_MODE_CYCLER) {
      booleanToggles ^= DISPLAY_MODE_CYCLER;
      shouldClearScreen = true;
      dispMode = (dispMode + 1) % numModes;
    }
    if (asteroidFlagsEnabled & asteroidFlags[3]) {
      asteroidFlagsEnabled ^= asteroidFlags[3];
      TimerEnable(TIMER0_BASE, TIMER_A);
    }
    switch (dispMode) {
    case level:
      bubbleLevel(g_accelData, &bubble);
      drawCircle(&g_sContext, &bubble, 3);
      break; 
    case game:
      break; 
    case numDisp:
      debugWriteToOLED(g_accelData, gyroData);
      break;
    }
    if (shouldClearScreen) {
      IntMasterDisable();
      //point the drawing context at the OLED
      GrContextInit(&g_sContext, &g_sCFAL96x64x16);   
      //draw the data from the off screen buffer
      GrImageDraw(&g_sContext, g_sOffScreen.pvDisplayData, 0, 0);
      IntMasterEnable();
    }
  }
}
/*
* Function Name: Timer0InterruptHander
* Purpose: drives the asteriod framerate when requried
* Inputs: none
* Outputs: none
* Notes: only important during asteroids ops
*/
void Timer0InterruptHandler(void) {
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  if (dispMode == game)
    runAsteroids();
}

/*
* Function Name: initAsteriods
* Purpose: initalizes the arrays and structs requried to run an asteroids game
* Inputs: a spaceship pointer, two asteroid arrays, and a bullet array
* Outputs: none
* Notes: relies on defines for length checking. not the greatest code style but
*        functional
*/
void initAsterioids(Spaceship* mySpaceship, Asteroid asteroids[]
                    ,Asteroid asteroidsSM[],  Bullet bullets[]) {
                      *mySpaceship = newSpaceship();
                      shouldClearScreen = true;
                      for (int i = 0; i < MD_ASTEROIDS; i++) {
                        asteroids[i] = newAsteroid();
                        asteroids[i].sprite.center = newPoint(60 * V_MOD, 56 * V_MOD);
                        asteroids[i].sprite.velocity = newPoint(-3, 2);
                        asteroids[i].sprite.isAlive = 1;
                        
                        /* allocate small asteroids */
                        asteroidsSM[i * 2] = newAsteroid();
                        asteroidsSM[i * 2 + 1] = newAsteroid();
                      }
                      mySpaceship->sprite.isAlive = 1;
                      /* allocate bullets */
                      for (int i = 0; i < MAX_BULLETS; i++) {
                        bullets[i] = newBullet(mySpaceship);
                        bullets[i].sprite.isAlive = 0;
                      }
                    }

/*
* Function Name: runAsteroids
* Purpose: driver for the asterioids game
* Inputs: none
* Outputs: none
* Notes: Should probably be modularized some more. meant to be called at a set
*        Rate in an ISR
*/
void runAsteroids(void) {
  /* create spaceship sprite */
  static Spaceship mySpaceship;
  /* create asteroids sprite */
  static Asteroid asteroids[MD_ASTEROIDS];
  /* create sm asteroids sprite */
  static Asteroid asteroidsSM[MD_ASTEROIDS * 2];
  /* create bullets sprites */
  static Bullet bullets[MAX_BULLETS];
  static uint8_t onload = 1;
  static uint8_t bulletCounter = 0;
  Point accelVector;
  if (onload) {
    initAsterioids(&mySpaceship, asteroids, asteroidsSM, bullets);
    onload ^= onload;
  } else {
    
    /* accelerate the spaceship */
    /* accelerometer data is updated in main psuedo-inifinte loop */
    /* generate and output vector */
    accelVector = newPoint((int16_t)(g_accelData[0])
                           , (int16_t)(g_accelData[1]));
    accelerate(&mySpaceship.sprite, &accelVector);
    move(&mySpaceship.sprite);
    
    /* rotate the spaceship */
    if (asteroidFlagsEnabled & asteroidFlags[0]) {
      asteroidFlagsEnabled ^= asteroidFlags[0];
      mySpaceship.angle = (mySpaceship.angle + 324) % 360;
    }
    if (asteroidFlagsEnabled & asteroidFlags[1]) {
      asteroidFlagsEnabled ^= asteroidFlags[1];
      mySpaceship.angle = (mySpaceship.angle + 36) % 360;
    }
    
    
    /* kill old bullets and increment life*/
    for (int i = 0; i < MAX_BULLETS; i++) {
      bullets[i].timeAlive++;
      move(&bullets[i].sprite);
      if (bullets[i].timeAlive > MAX_BULLET_LIFE * V_MOD)
        bullets[i].sprite.isAlive = 0;
    }
    
    /* generate new bullets */
    if (asteroidFlagsEnabled & asteroidFlags[2]) {
      asteroidFlagsEnabled ^= asteroidFlags[2];
      bullets[bulletCounter] = newBullet(&mySpaceship);
      bullets[bulletCounter].sprite.isAlive = 1;
      bulletCounter = (bulletCounter + 1) % MAX_BULLETS;
    }
    
    /* test asteroid collisions */
    for (int i = 0; i < MD_ASTEROIDS; i++) {
      move(&asteroids[i].sprite);
      if (asteroids[i].sprite.isAlive
          && isCollided(&asteroids[i].sprite, &mySpaceship.sprite)) {
            mySpaceship.sprite.isAlive = 0; /* kill it */
          }
      
      /* test bullet collisions */
      for (int j = 0; j < MAX_BULLETS; j++) {
        if (isCollided(&asteroids[i].sprite, &bullets[j].sprite)
            && bullets[j].sprite.isAlive  && asteroids[i].sprite.isAlive) {
              asteroids[i].sprite.isAlive = 0;
              bullets[j].sprite.isAlive = 0;
              /* spawn new asteroids */
              asteroidsSM[i * 2] = newSMAsteroid(&asteroids[i].sprite.center);
              asteroidsSM[i * 2 + 1] = newSMAsteroid(&asteroids[i].sprite.center);
              asteroidsSM[i * 2].sprite.isAlive = 1;
              asteroidsSM[i * 2 + 1].sprite.isAlive = 1;
            }
      }
      displaySprite(&g_sContext, &(asteroids[i].sprite), 0, 0x0);
    }
    
    /* process small asteroids */
    for (int i = 0; i < MD_ASTEROIDS * 2; i++) {
      if (isCollided(&asteroidsSM[i].sprite, &mySpaceship.sprite)
          && asteroidsSM[i].sprite.isAlive) {
            mySpaceship.sprite.isAlive = 0; /* kill it */
          }
      for (int j = 0; j < MAX_BULLETS; j++) {
        if (isCollided(&asteroidsSM[i].sprite, &bullets[j].sprite)
            && bullets[j].sprite.isAlive && asteroidsSM[i].sprite.isAlive) {
              asteroidsSM[i].sprite.isAlive = 0;
              bullets[j].sprite.isAlive = 0;
            }
      }
      move(&asteroidsSM[i].sprite);
      displaySprite(&g_sContext, &(asteroidsSM[i].sprite), 0, 0x0);
    }
    
    for (int i = 0; i < MAX_BULLETS; i++) {
      displaySprite(&g_sContext, &(bullets[i].sprite), 0, 0x0);
    }
    displaySpaceship(&g_sContext, &(mySpaceship));
    
    /* check win/loss conditions and act accordingly */
    if (!mySpaceship.sprite.isAlive) {
      /* decrease lives */
      mySpaceship.lives--;
      mySpaceship.sprite.center = newPoint(48 * V_MOD, 32 * V_MOD);
      mySpaceship.sprite.isAlive = 1;
      /* TODO: updates lives text */
    }
    
    displayLives(&g_sContext, mySpaceship.lives);
    if (mySpaceship.lives == 0) {
      shouldClearScreen = false;
      displayTextToOLED(&g_sContext, OLED_TEXT_LINE1, "Game over.");
      displayTextToOLED(&g_sContext, OLED_TEXT_LINE2, "Up to Reset");
      TimerDisable(TIMER0_BASE, TIMER_A);
      onload = 1;
    }
    uint8_t won = 1;
    for (int i = 0; i < MD_ASTEROIDS; i++) {
      if (asteroids[i].sprite.isAlive ||asteroidsSM[2*i].sprite.isAlive || asteroidsSM[2*i + 1].sprite.isAlive) {
        won = 0;
      }
    }
    if (won) {
      /* inform the user */
      shouldClearScreen = false;
      displayTextToOLED(&g_sContext, OLED_TEXT_LINE1, "You won!!!");
      displayTextToOLED(&g_sContext, OLED_TEXT_LINE2, "Up to Reset");
      TimerDisable(TIMER0_BASE, TIMER_A);
      onload = 1; /* flag reset game */
    }
  }
  /* initialize lives */
}

/* Function: enableTimers
* Purpose: enables timer0 at a 1Khz frequency
*/
void enableTimers(){
  //
  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  // Wait for peripheral to be turned on
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
  //
  // Configure the two 32-bit periodic timers.
  //
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER0_BASE, TIMER_A,  SIXTYFOUR_FPS);
  //
  // Setup the interrupts for the timer timeouts.
  //
  IntEnable(INT_TIMER0A);
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0InterruptHandler);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A);
}

/* Function: initI2C
* Purpose: Initalizes the I2C3 periperhal to listen on pins A6 and A7
*/
void initI2C() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  // Wait for peripheral to be turned on
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3));
  
  /* enable GPIO Pins */
  /* GPIOPinTypeI2C & other */
  /* GPIOPinConfigure x2 */
  GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
  GPIOPinConfigure(GPIO_PD0_I2C3SCL);
  GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
  GPIOPinConfigure(GPIO_PD1_I2C3SDA);
  
  /* confogure the GPIO points and their interrupt conditions */
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
  IntEnable(INT_GPIOB);
  
  I2CMasterInitExpClk(I2C3_BASE,SysCtlClockGet(), false);
  
  /* initialize the I2C Slave address */
  I2CMasterSlaveAddrSet(I2C3_BASE, 0x69, false);
  
  I2CMasterEnable(I2C3_BASE);
}

void MPU9150Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
  //
  // See if an error occurred.
  //
  if(ui8Status != I2CM_STATUS_SUCCESS)
  {
    //
    // An error occurred, so handle it here if required.
    //
  }
  //
  // Indicate that the MPU9150 transaction has completed.
  //
  g_MPUDone = true;
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void MPU9150I2CIntHandler(void)
{
  //
  // Pass through to the I2CM interrupt handler provided by sensor library.
  // This is required to be at application level so that I2CMIntHandler can
  // receive the instance structure pointer as an argument.
  //
  I2CMIntHandler(&MPUI2C);
}

void configureMPU(tI2CMInstance *thisI2C, tMPU9150 *thisMPU, uint8_t adr) {
  g_MPUDone = false;
  I2CMInit(thisI2C, I2C3_BASE, INT_I2C3, 0xff, 0xff,
           SysCtlClockGet());
  MPU9150Init(thisMPU, thisI2C, adr, MPU9150Callback, 0);
  while(!g_MPUDone) {
  }
  g_MPUDone = false;
  MPU9150ReadModifyWrite(thisMPU, MPU9150_O_ACCEL_CONFIG,
                         ~MPU9150_ACCEL_CONFIG_AFS_SEL_M,
                         MPU9150_ACCEL_CONFIG_AFS_SEL_4G, MPU9150Callback,
                         0);
  while(!g_MPUDone)
  {
  }
}

/*
* Function: getMPUDataAccel
* Purpose:  loads an array of floats with data from the accelerometer
*/
void getMPUDataAccel(tMPU9150 *thisMPU, float* accelArray) {
  g_MPUDone = false;
  static bool hasRun = false;
  static CircularBuffer xBuff;
  static CircularBuffer yBuff;
  static CircularBuffer zBuff;
  static float dataArray[3];
  int i = 0;
  MPU9150DataRead(thisMPU, MPU9150Callback, 0);
  if(hasRun == false) {
    xBuff = newBuffer();
    yBuff= newBuffer();
    zBuff = newBuffer();
    
    for(i = 0; i < 3; i++) {
      dataArray[i] = 0;
    }
    hasRun =  true;
  }
  addValueToBuffer(&xBuff, dataArray[0]);
  addValueToBuffer(&yBuff, dataArray[1]);
  addValueToBuffer(&zBuff, dataArray[2]);
  
  accelArray[0] = averageBuffer(&xBuff);
  accelArray[1] = averageBuffer(&yBuff);
  accelArray[2] = averageBuffer(&zBuff);
  
  
  while(!g_MPUDone) {;}
  
  MPU9150DataAccelGetFloat(thisMPU, &dataArray[0], &dataArray[1]
                           , &dataArray[2]);
  
}

/*
* Function: getMPUDataGyro
* Purpose:  loads an array of floats with data from the gyro
*/
void getMPUDataGyro(tMPU9150 *thisMPU, float* gyroArray) {
  g_MPUDone = false;
  MPU9150DataRead(thisMPU, MPU9150Callback, 0);
  while(!g_MPUDone) {;}
  
  MPU9150DataGyroGetFloat(thisMPU, &gyroArray[0], &gyroArray[1]
                          , &gyroArray[2]);
}

/*
* Function: bubbleLevel
* Purpose:  calculates the position of a bubble based on accelerometer data
* Inputs:   accelData - float array with {x,y,z} accelerations
*           location  - pointer to a Point struct
* Postcondition: populated Point structure
* Notes:    Point is defined in luke_graphics.h
*           y and x in code a reversed to coordinate OLED with accelerometer.
*/
void bubbleLevel(float* accelData, Point* location) {
  float temp = 0.0;
  float width = GrContextDpyWidthGet(&g_sContext) / 2.0;
  float height = GrContextDpyHeightGet(&g_sContext) / 2.0;
  tRectangle rect;
  prepareOLED(&g_sContext, ClrWhite, ClrBlack);
  rect.i16YMin = (GrContextDpyHeightGet(&g_sContext) /2) + 5;
  rect.i16YMax = (GrContextDpyHeightGet(&g_sContext) /2) - 5;
  rect.i16XMin = (GrContextDpyWidthGet(&g_sContext) /2);
  rect.i16XMax = (GrContextDpyWidthGet(&g_sContext) /2);
  GrRectFill(&g_sContext, &rect);
  rect.i16YMin = (GrContextDpyHeightGet(&g_sContext) /2);
  rect.i16YMax = (GrContextDpyHeightGet(&g_sContext) /2);
  rect.i16XMin = (GrContextDpyWidthGet(&g_sContext) /2) - 5;
  rect.i16XMax = (GrContextDpyWidthGet(&g_sContext) /2) + 5;
  GrRectFill(&g_sContext, &rect);
  /* code for y-location */
  temp = accelData[0] / M_G;
  location->y = (uint8_t)((-1.0 * temp) * height + height);
  /* code for x-location */
  temp = accelData[1] / M_G;
  location->x = (uint8_t)((-1.0 * temp) * width + width);
}

/*
* Function: debugWriteToOLED
* Purpose:  write accelerometer and gyoscopic data to the OLED
*/
void debugWriteToOLED(float* accelData, float* gyroData) {
  char tmpString[32];
  sprintf(tmpString, "x: %s %s ", floatToString(accelData[0])
          , floatToString(gyroData[0]));
  displayTextToOLED(&g_sContext, OLED_TEXT_LINE1, tmpString);
  sprintf(tmpString, "y: %s %s ", floatToString(accelData[1])
          , floatToString(gyroData[1]));
  displayTextToOLED(&g_sContext, OLED_TEXT_LINE2, tmpString);
  sprintf(tmpString, "z: %s %s ", floatToString(accelData[2])
          , floatToString(gyroData[2]));
  displayTextToOLED(&g_sContext, OLED_TEXT_LINE3, tmpString);
}



/*
* Function: buttonsISR
* Purpose: Interrupt Service Routine for the GPIO Buttons
*/
void buttonsISR() {
  GPIOIntClear(GPIO_PORTM_BASE, ALL_BUTTONS);
  if (detectButtonPresses() == SELECT_BUTTON){
    /* call the shoot function! */
    asteroidFlagsEnabled ^= asteroidFlags[2]; /* toggle shoot */
    //displayLastButtonPressed(SELECT_BUTTON, &g_sContext); 
  }
  if (detectButtonPresses() == LEFT_BUTTON){
    /* rotate left */
    asteroidFlagsEnabled ^= asteroidFlags[0]; /* toggle left */
  }
  if (detectButtonPresses() == RIGHT_BUTTON){
    /* rotate right */
    asteroidFlagsEnabled ^= asteroidFlags[1]; /* toggle right */
  }
  if (detectButtonPresses() == UP_BUTTON) {
    /* flag reset game */
    asteroidFlagsEnabled ^= asteroidFlags[3];
  }
}

void UARTIntHandler(void)
{
  uint32_t tempChar;
  uint32_t ui32Status = UARTIntStatus(UART0_BASE, true);
  //
  // Clear the asserted interrupts.
  //
  UARTIntClear(UART0_BASE, ui32Status);
  
  //
  // Loop while there are characters in the receive FIFO.
  //
  while(UARTCharsAvail(UART0_BASE))
  {
    //
    // Read the next character from the UART and enqueue it
    //
    tempChar = UARTCharGetNonBlocking(UART0_BASE);
    if(tempChar != -1) {
      enqueue(&UARTQueue,tempChar);
    }
  }
}