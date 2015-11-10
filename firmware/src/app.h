/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <time.h>
#include "messageQueue.h"
#include "system/system.h"
#include "system/clk/sys_clk.h"
#include "driver/usart/drv_usart.h"
#include "system/devcon/sys_devcon.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 
 
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define APP_TEST                true 
#define APP_ERROR_TESTING       false
#define APP_MAX_ERROR_LOG       5
#define APP_NUMBER_OF_TICKS     5
#define APP_NO_OF_BYTES_TO_READ 1          
#define APP_BUFFER_SIZE         80
#define APP_ERROR_BUFFER_SIZE   75      //must be 5 less than APP_BUFFER_SIZE
#define INSTRUCTION_BUFFER_SIZE 160
#define APP_UART_BAUDRATE       57600
#define APP_USR_ESC_KEY         0x1B
#define APP_USR_RETURN_KEY      0x0D
#define APP_USART_DRIVER_INDEX  DRV_USART_INDEX_0

#define APP_DRV_CONTEXT         1
#define APP_USR_CONTEXT         2
    
    
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,

	/* TODO: Define states used by the application state machine. */
            APP_DRV_OPEN,
            APP_DRV_READY,
            APP_DRV_MSG_WRITE,
            APP_WAIT_FOR_DONE,
            APP_USR_MSG_READ,
            APP_USR_MSG_WRITE,
            APP_IDLE,
            APP_ERROR,
            APP_STATE_END
            
} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES currentState;

    /* TODO: Define any additional data used by the application. */
    uint8_t IRData;
    int randCounter;
    
    //encoder stuff
    int leftEncoder;
    int rightEncoder;
    
    /* Application Previous state */
    APP_STATES prevState;

    /* USART buffer for display */
    char buffer[APP_BUFFER_SIZE];

    /* Data Size in the USART Buffer */
    uint32_t bufferSize;

    /* Demo Application Message Indices */
    uint32_t usrMsgIndex;
    
    /* Client Status */
    DRV_USART_CLIENT_STATUS usartStatus;

    /* USART driver handle */
    DRV_HANDLE usartHandle;

    /* Handle returned by USART for buffer submitted */
    DRV_USART_BUFFER_HANDLE usartBufferHandle;

    /* Handle returned by USART for buffer event */
    DRV_USART_BUFFER_EVENT  usartBufferEvent;

    /* Flag to indicate the driver message is been processed */
    bool drvBufferEventComplete;

    /* Flag to indicate the user message is been processed */
    bool usrBufferEventComplete;
    
    char minBuffer[80];
    char InstructionSet[INSTRUCTION_BUFFER_SIZE];
    
    bool queued;
    bool initialized;
    bool error;
    bool error_sent;

} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
void moveRobot();
void interpretIR();
void readIR();

void getFromMessageQueue();
void hardLeft();
void hardRight();
void hardStraight();

void App_GetNextTaskState ( uint32_t appState );
void fillQueue();
void AddInstr();
void setError(char* error);
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );


#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

