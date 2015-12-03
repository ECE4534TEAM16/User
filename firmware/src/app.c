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
/* USART Driver Demo Banner Message */

char preMsg[] = "\r\n User Rover Connected";

#ifdef APP_TEST
    char temp[INSTRUCTION_BUFFER_SIZE];
    char test[INSTRUCTION_BUFFER_SIZE];
#endif

/* User Application Data Structure */
APP_DATA appData;

extern SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/
void readIR()
{    
    char dir;
    
    PLIB_PORTS_DirectionInputSet(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);

    const TickType_t waitDecay = 4 / portTICK_PERIOD_MS;
    vTaskDelay(waitDecay);
    
    appData.IRData = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E);
    
    appData.IRData = appData.IRData & 0x3F;
    
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, appData.IRData);
    
    if(!appData.running)
    {
        xQueueReceive( MsgQueue_User_Directions, &dir, APP_NUMBER_OF_TICKS);
    }
    
    if(dir == 'S')  
    {
        //appData.running = true;
        xQueueReset(MsgQueue_User_Directions);
    }
    
    if(appData.currentState != APP_STATE_END && appData.running)
    {
        interpretIR();
    }
    
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
}

void moveRobot(int leftSpeed, int rightSpeed)
{
    //setError("moveRobot");
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
    //setError("interpretIR");
    //ASSUMPTION - Dark line = 0
    switch(appData.IRData)
    {
        //NORMAL
        case 12: //00110011
        {
            moveRobot(600,600);
            break;
        }
        
        //WOBBLES
        //Off left
        case 8: //00111001
        {
            moveRobot(600,0);
            break;
        }
        case 40: //00111100
        {
            moveRobot(600,0);
            break;
        }
        case 32: //00111001
        {
            moveRobot(600,0);
            break;
        }
        case 48: //00111100
        {
            moveRobot(600,0);
            break;
        }
        case 16: //00111001
        {
            moveRobot(600,0);
            break;
        }
        
        //Off right
        case 2: //00100111
        {
            moveRobot(0,600);
            break;
        }
        case 6: //00001111
        {
            moveRobot(0,600);
            break;
        }
        case 4: //00001111
        {
            moveRobot(0,600);
            break;
        }
        case 3: //00001111
        {
            moveRobot(0,600);
            break;
        }
        case 1: //00001111
        {
            moveRobot(0,600);
            break;
        }
        
        //INTERSECTIONS
        case 47:  //00000011
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
        case 15:  //00000011
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
        case 62: //00110000
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
        case 60: //00110000
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
        //Both
        case 0:  //00000000
        {
            moveRobot(0,0);
            getFromMessageQueue();
            break;
        }
        
        default: //Everything else - handles weird characters, completely off the line, etc.
        {
            moveRobot(0,0);
            break;
        }
    }
}

void getFromMessageQueue()
{
    char dir;
    char err[50];
    while(!xQueueReceive( MsgQueue_User_Directions, &dir, APP_NUMBER_OF_TICKS))
    {
        //BAD
    }
    
    sprintf(err, "getFromMessageQueue() - Case Read: %c", dir);
    
    if(appData.currentState != APP_STATE_END)
    {
        setError(err);
        switch(dir)
        {
            case 'l':
            {
                hardLeft();
                break;
            }
            case 'r':
            {
                hardRight();
                break;
            }
            case 'f':
            {
                hardStraight();
                break;
            }
            case 'E':
            {
                setError("Destination");
                appData.currentState = APP_STATE_END;
                break;
            }
            case 0:
            {
                break;
            }
            default:
            {
                setError("Invalid Character in Message Queue!");
                appData.currentState = APP_STATE_END;
                break;
            }
        }
    }
}

void hardLeft()
{
    int x = 0;
    char dir;
    xQueueReset(MsgQueue_LeftEncoder);
    moveRobot(800,800);
    while(x < 17)
    {
        if(xQueueReceive( MsgQueue_RightEncoder, &dir, 0))
        {
            x++;
        }
    }
    x = 0;
    xQueueReset(MsgQueue_LeftEncoder);
    moveRobot(-800,800);
    while(x < 8)
    {
        if(xQueueReceive( MsgQueue_LeftEncoder, &dir, 0))
        {
            x++;
        }
    }
    moveRobot(0,0);
    setError("Hard Left");
}
void hardRight()
{
    int x = 0;
    char dir;
    xQueueReset(MsgQueue_RightEncoder);
    moveRobot(800,800);
    while(x < 5)
    {
        if(xQueueReceive( MsgQueue_RightEncoder, &dir, 0))
        {
            x++;
        }
    }
    x = 0;
    xQueueReset(MsgQueue_RightEncoder);
    moveRobot(800,-800);
    while(x < 8)
    {
        if(xQueueReceive( MsgQueue_RightEncoder, &dir, 0))
        {
            x++;
        }
    }
    moveRobot(0,0);
    setError("Hard Right");
}
void hardStraight()
{
    int x = 0;
    char dir;
    xQueueReset(MsgQueue_RightEncoder);
    moveRobot(800,800);
    while(x < 5)
    {
        if(xQueueReceive( MsgQueue_RightEncoder, &dir, 0))
        {
            x++;
        }
    }
    moveRobot(0,0);
    setError("Hard Forward");
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
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    //Seed RNG for testing w/o sensors
    srand(time(NULL));
    
    //Motor Control
    DRV_TMR0_Start();
    DRV_OC0_Start();
    DRV_OC1_Start();
    
    PLIB_TMR_Period16BitSet(1,1000);
            
    PLIB_OC_PulseWidth16BitSet(0,0);
    PLIB_OC_PulseWidth16BitSet(1,0);
    
    //motor direction
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    
    //leds 4 and 5
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);

    //for ir sensors
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_B, 0xFF);
    
    //Encoders
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    
    //Message Queues
    MsgQueue_temp = xQueueCreate( 50, sizeof( char ) );
    if( MsgQueue_temp == 0 )
    {
        //BAD
    }
    MsgQueue_RightEncoder = xQueueCreate( 50, sizeof( char ) );
    if( MsgQueue_RightEncoder == 0 )
    {
        //BAD
    }
    MsgQueue_LeftEncoder = xQueueCreate( 50, sizeof( char ) );
    if( MsgQueue_LeftEncoder == 0 )
    {
        //BAD
    }
    
    //FreeRTOS timer
    appData.randCounter = 0;
    
    //UART STUFF
     /* Set the App. previous state to its initial state. */
    appData.prevState         = APP_DRV_OPEN;
    /* Set the App. current state to its initial state. */
    appData.currentState      = APP_DRV_OPEN;
    /* Initialise buffer Size */
    appData.bufferSize        = 0;
    /* Demo App. message index */
    appData.usrMsgIndex       = 0;
    /* Set the USART Clent Statud to error */
    appData.usartStatus       = DRV_USART_CLIENT_STATUS_ERROR;
    /* Set the USART buffer handler to invalid */
    appData.usartBufferHandle = DRV_HANDLE_INVALID;
    /* Set the USART handler to invalid */
    appData.usartHandle       = DRV_HANDLE_INVALID;
    /* Set the USART buffer event to invalid */
    appData.usartBufferEvent  = DRV_USART_BUFFER_EVENT_ERROR;
    /* Set the initial state of event flags for driver messages */
    appData.drvBufferEventComplete = false;
    /* Set the initial state of event flags for user messages */
    appData.usrBufferEventComplete = false;
    /* Clear Application Buffer */
    strcpy(appData.buffer, "");
    /* Set the flag for Queue fill*/
    appData.queued            = false;
    /* Set the initialized flag to false*/
    appData.initialized       = false;
    /* Set the error flag to false*/
    appData.error             = false;
    /* set the error_sent flag to true*/
    appData.error_sent        = true;
    
    MsgQueue_Error_Log = xQueueCreate(APP_MAX_ERROR_LOG, APP_ERROR_BUFFER_SIZE);
    if(MsgQueue_Error_Log == 0)
    {
        //Big failure
        //Need to set an LED on to alert us of this error as
        //we wont be able to send the error code through UART without this
        //Message Queue.
    }

    MsgQueue_User_Directions = xQueueCreate(APP_BUFFER_SIZE, sizeof( char ));
    if( MsgQueue_User_Directions == 0 )
    {
        //BAD
        //fatal error
    }
    
    appData.running = false;
    
    appData.irTimer = xTimerCreate("IR Timer", 100/portTICK_PERIOD_MS,pdTRUE, (void*) 1, readIR);
    xTimerStart(appData.irTimer, 100);
    
}

//if error occurs before UART is opened the error will be fatal
//otherwise the system will try to return to a functioning state
void setError(char* error)
{
    
    xQueueSendToBack(MsgQueue_Error_Log, error, APP_NUMBER_OF_TICKS);
    appData.error = true;
    
}

//will take the instruction set and put it into a message queue
//fills the user_instructions with the data from the Pi after the 
//Pi has finished sending the entire instruction set
void fillQueue()
{
    int count = 0;
    appData.queued = true;
    for(count; count <= strlen(appData.InstructionSet); count++)
    {
        xQueueSendToBack(MsgQueue_User_Directions, 
                          (void *) &appData.InstructionSet[count], 
                          APP_NUMBER_OF_TICKS);
    }
    
    appData.currentState = APP_USR_MSG_WRITE;
    
}

//copies the current buffer character to the instruction set array
void AddInstr()
{
    if(APP_TEST)
    {
        if(temp[0] == NULL)  
        strcpy(temp, appData.buffer);
        else
        {
            strcat(temp, ",");
            strcat(temp, appData.buffer); 
        }
    }
    if(appData.InstructionSet[0] == NULL)  
        strcpy(appData.InstructionSet, appData.buffer);
    else
    {
        strcat(appData.InstructionSet, appData.buffer); 
    }
    if(appData.buffer[0] == 'E' && appData.queued == false)
        fillQueue();
    if(APP_ERROR_TESTING)
    {
        if(appData.buffer[0] == '1')
            setError("Testing Error Message Queue");
    }
        
}

void APP_BufferEventHandler(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context)
{
    switch(buffEvent)
    {
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            if(context == APP_DRV_CONTEXT)
            {
                /* Update buffer event status */
                appData.drvBufferEventComplete = true;
            }
            else if (context == APP_USR_CONTEXT)
            { 
                /* Update buffer event status */
                appData.usrBufferEventComplete = true;   
            }
        }
            break;

        /* Buffer event has some error */
        case DRV_USART_BUFFER_EVENT_ERROR:
        {
            setError("Buffer Event Error");
        }
            break;

        /* Buffer event has aborted */
        case DRV_USART_BUFFER_EVENT_ABORT:
        {
            setError("Buffer Event has Aborted");
        }
            break;
    }
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
        
void APP_Tasks ( void )
{
    if(appData.error && appData.error_sent)
    {
        appData.currentState = APP_ERROR;
        appData.error_sent = false;
    }
    /* Check the Application State*/
    switch ( appData.currentState )
    {
        /* Open USART Driver and set the Buffer Event Handling */
        case APP_DRV_OPEN:
        {
            appData.usartHandle = DRV_USART_Open(APP_USART_DRIVER_INDEX,
                         (DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING));

            if (appData.usartHandle != DRV_HANDLE_INVALID )
            {
                DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                                       APP_BufferEventHandler, APP_DRV_CONTEXT);
                appData.prevState    = APP_DRV_OPEN;
                appData.currentState = APP_DRV_READY;
            }
            else
            {
                appData.currentState = APP_ERROR; //fatal error
            }
        }
        break;

        case APP_DRV_READY:
        {
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy(appData.buffer, "\r\n");
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                strlen(appData.buffer));
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize );

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    setError("Invalid Driver Handle in DRV_READY");
                }
                else
                {
                    appData.prevState    = APP_DRV_READY;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
            else
            {
                appData.currentState = APP_ERROR;
            }
        }
        break;

        case APP_WAIT_FOR_DONE:
        {
            if(appData.drvBufferEventComplete)
            {
                appData.drvBufferEventComplete = false;
                appData.error_sent = true;
                App_GetNextTaskState(appData.prevState);
            }
            else if(appData.usrBufferEventComplete)
            {
                appData.usrBufferEventComplete = false;
                App_GetNextTaskState(appData.prevState);
            }
        }
        break;

          //writes to terminal that the connection was a success
        case APP_DRV_MSG_WRITE:
        {
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy( appData.buffer, preMsg );
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    setError("Invalid Driver Handle in DRV_MSG_WRITE");
                }
                else
                {
                    appData.prevState    = APP_DRV_MSG_WRITE;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

            //reads from terminal
            //eventually will read from serial port on PI
        case APP_USR_MSG_READ:
        {
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );
            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy( appData.buffer, " " );
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                DRV_USART_BufferAddRead( appData.usartHandle,
                                         &(appData.usartBufferHandle),
                                         appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    setError("Invalid Driver Handle in USR_MSG_READ");
                }
                else
                {
                    appData.prevState    = APP_USR_MSG_READ;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

        case APP_USR_MSG_WRITE:
        {

            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                if(APP_TEST) //Tests the rover on receiving the instruction set
                {
                strcpy(test, "\r\n");
                strcat(test, temp );
                
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(test));
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          test, appData.bufferSize);
                }
                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    appData.prevState    = APP_USR_MSG_WRITE;
                    setError("Invalid Driver Handle");
                }
                else
                {
                    appData.prevState    = APP_USR_MSG_WRITE;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
                
            }
        }
        break;

        case APP_IDLE:
        {
            /* Close USART Driver */
            DRV_USART_Close( appData.usartHandle );
            /* Deinitialize the driver */
            DRV_USART_Deinitialize( sysObj.drvUsart0 );
            /* The appliction comes here when the rover has completed its path
             * successfully. Need to implement in code. */
        }
        break;

        case APP_ERROR:
        {
            char error_buffer[APP_BUFFER_SIZE];
            char pre_error[APP_ERROR_BUFFER_SIZE];
            if(uxQueueMessagesWaiting(MsgQueue_Error_Log) > 0 && appData.prevState != APP_DRV_OPEN)
            {
                //set the handler
                DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                                           APP_BufferEventHandler, APP_DRV_CONTEXT);

                appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

                if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
                {
                    if(xQueueReceive(MsgQueue_Error_Log, &pre_error, APP_NUMBER_OF_TICKS))
                    {
                        strcpy( error_buffer, "\r\n" );
                        strcat( error_buffer, pre_error);
                        appData.bufferSize = min(APP_BUFFER_SIZE,
                                                               strlen(error_buffer));
                        DRV_USART_BufferAddWrite( appData.usartHandle,
                                                  &(appData.usartBufferHandle),
                                                  error_buffer, appData.bufferSize);
                    }
                    if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                    {
                        //set LED, this is fatal error
                    }
                    else
                    {
                        appData.prevState    = APP_ERROR;
                        appData.currentState = APP_WAIT_FOR_DONE;
                    }
                }
            }
            else
            {
                //comm error(fatal), set LED
                //state will stay in the error state
            }
            
        }
        break;

        case APP_STATE_END:
        {
            break;
        }
        
        default:
            break;
    }
}

void App_GetNextTaskState(uint32_t appState)
{
    switch ( appState )
    {
        case APP_DRV_READY:
            appData.currentState = APP_DRV_MSG_WRITE;
            break;

        case APP_DRV_MSG_WRITE:
            strcpy(appData.buffer, "\n");
            appData.bufferSize = APP_NO_OF_BYTES_TO_READ;
            /* Set the buffer event for user data */
            DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                              APP_BufferEventHandler, APP_USR_CONTEXT);
            /* Set the next Demo App. State */
            appData.currentState = APP_USR_MSG_READ; 
            appData.initialized = true; //safeguard in error handling
        break;

        case APP_USR_MSG_READ:
            AddInstr();
            if(APP_TEST || appData.queued == true )
                appData.currentState = APP_USR_MSG_WRITE;
            break;

        case APP_USR_MSG_WRITE:
            if(APP_TEST)
                appData.currentState = APP_USR_MSG_READ;
            else
                appData.currentState = APP_USR_MSG_WRITE;
            break;
            
        case APP_ERROR:
            if(uxQueueMessagesWaiting(MsgQueue_Error_Log) > 0)
            {
                appData.currentState = APP_ERROR;
            }
            else //no errors left to be written
            {
                appData.error = false;
                if(appData.initialized)
                {
                    DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                              APP_BufferEventHandler, APP_USR_CONTEXT);
                    
                    if(appData.queued)
                        appData.currentState = APP_USR_MSG_WRITE;
                    else
                        appData.currentState = APP_USR_MSG_READ;
                    
                }
                else
                    appData.currentState = APP_DRV_MSG_WRITE;       
            }
            
            break;

    }
}

/*******************************************************************************
 End of File
 */
