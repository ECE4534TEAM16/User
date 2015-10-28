/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/
void readIR()
{
    //chipkit pit 48-53
    /* Read from sensors
    appData.IRData ^= (-(PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_8) ^ appData.IRData) & (1<<0));
    appData.IRData ^= (-(PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11) ^ appData.IRData) & (1<<1));
    appData.IRData ^= (-(PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7) ^ appData.IRData) & (1<<2));
    appData.IRData ^= (-(PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8) ^ appData.IRData) & (1<<3));
    appData.IRData ^= (-(PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_6) ^ appData.IRData) & (1<<4));
    appData.IRData ^= (-(PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9) ^ appData.IRData) & (1<<5));
    */
    
    /* Regular Switching
    switch(appData.state)
    {
        case APP_STATE_INIT:
        {
            appData.IRData = 0;
            break;
        }
        case APP_STATE_STRAIGHT:
        {
            appData.IRData = 51;
            break;
        }
        case APP_STATE_LEFT:
        {
            appData.IRData = 57;
            break;
        }
        case APP_STATE_RIGHT:
        {
            appData.IRData = 39;
            break;
        }
        
        default:
        {
            break;
        }
    }
    */
    
    uint8_t fakeIRData[] = {60,57,51,39,15,3,48,0};
    
    appData.IRData = fakeIRData[rand() % 8];
    
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, appData.IRData);
    //debugOut(printf("readIR() - IR Data: %i", appData.IRData))
    
    //interpretIR();
}

void moveRobot(int leftSpeed, int rightSpeed)
{
    if(leftSpeed > 0)
    {
        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    else
    {
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    
    if(rightSpeed > 0)
    {
        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    else
    {
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    
    PLIB_OC_PulseWidth16BitSet(1,leftSpeed);
    PLIB_OC_PulseWidth16BitSet(0,rightSpeed);
    
    //debugOut(printf("moveRobot() - Left Motor: %i, Right Motor: %i", leftSpeed, rightSpeed))
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/
void interpretIR()
{
    //ASSUMPTION - Dark line = 0
    switch(appData.IRData)
    {
        //NORMAL
        case 51: //00110011
        {
            moveRobot(800,800);
            break;
        }
        
        //WOBBLES
        //Off left
        case 57: //00111001
        {
            moveRobot(800,0);
            break;
        }
        case 60: //00111100
        {
            moveRobot(800,0);
            break;
        }
        
        //Off right
        case 39: //00100111
        {
            moveRobot(0,800);
            break;
        }
        case 15: //00001111
        {
            moveRobot(0,800);
            break;
        }
        
        //INTERSECTIONS
        //Left
        case 3: 
        {
            moveRobot(0,0);
            break;
        }
        //Right
        case 48: 
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
        //Both
        case 0: 
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
        
        
        default:
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
    }
    
    //debugOut(printf("interpretIR() - Case Read: %i", appData.IRData))
}

void getFromMessageQueue()
{
    char dir;
    if(!xQueueReceive( MsgQueue_User_Directions, &dir, 5))
    {
        //BAD
    }
    
    switch(dir)
    {
        case 'l':
        {
            hardLeft();
        }
        case 'r':
        {
            hardRight();
        }
        case 's':
        {
            hardStraight();
        }
    }
}

void hardLeft()
{
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 'L');
}
void hardRight()
{
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 'R');
}
void hardStraight()
{
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 'S');
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    //Seed RNG for testing w/o sensors
    srand(time(NULL));
    
    DRV_TMR0_Start();
    DRV_OC0_Start();
    DRV_OC1_Start();
    
    PLIB_TMR_Period16BitSet(1,1000);
            
    PLIB_OC_PulseWidth16BitSet(0,0);
    PLIB_OC_PulseWidth16BitSet(1,0);
    
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);

    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    
    //FreeRTOS Timer for testing
    TimerHandle_t irTimer = xTimerCreate("IR Timer", 5000/portTICK_PERIOD_MS,pdTRUE, (void*) 1, readIR);
    xTimerStart(irTimer, 100);
    
    //Message Queues
    MsgQueue_User_Directions = xQueueCreate( 50, sizeof( char ) );
    if( MsgQueue_User_Directions == 0 )
    {
        //BAD
    }
    
    //init test queue
    xQueueSend(MsgQueue_User_Directions, 'l', 5);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            //readIR();
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
