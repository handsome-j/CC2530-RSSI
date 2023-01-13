/***********************************************************************************
Filename: 	per_test.c

Description:  This application functions as a packet error rate (PER) tester.
One node is set up as transmitter and the other as receiver. The role and
configuration parameters for the PER test of the node is chosen on initalisation
by navigating the joystick and confirm the choices with S1.

The configuration parameters are channel, burst size and tx power. Push S1 to
enter the menu. Then the configuration parameters are set by pressing
joystick to right or left (increase/decrease value) and confirm with S1.

After configuration of both the receiver and transmitter, the PER test is
started by pressing joystick up on the transmitter. By pressing joystick up
again the test is stopped.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_int.h"
#include "hal_timer_32k.h"
#include "hal_joystick.h"
#include "hal_button.h"
#include "hal_board.h"
#include "hal_rf.h"
#include "hal_assert.h"
#include "util_lcd.h"
#include "basic_rf.h"
#include "per_test.h"
#include "string.h"

/***********************************************************************************
* CONSTANTS
*/
// Application states
#define IDLE                      0
#define TRANSMIT_PACKET           1
#define LED1 P1_3                //定义P1.0口为LED1控制端
#define LED2 P0_4

/***********************************************************************************
* LOCAL VARIABLES
*/
static basicRfCfg_t basicRfConfig;
static perTestPacket_t txPacket;
static perTestPacket_t rxPacket;
static volatile uint8 appState;
static volatile uint8 appStarted;
char Myreceive[10] = {0},Myrssi[5]={0},Myper[10] = {0};
/***********************************************************************************
* LOCAL FUNCTIONS
*/
static void appTimerISR(void);
static void appStartStop(void);
static void appTransmitter();
static void appReceiver();
void initUART(void);
void UartTX_Send_String(int8 *Data, int len);


void initUART(void)
{
  PERCFG=0X00;
  P0SEL=0X0C;
  P2DIR&=~0X0C;
  U0CSR|=0X80; 
  U0GCR|=11;
  U0BAUD|=216;
  UTX0IF=0;
  P1DIR |= 0x08;
  P0DIR|=0x10;
}



void UartTX_Send_String(int8 *Data, int len)
{
  int j;
  for(j=0;j<len;j++)
  {
    U0DBUF=*Data++;
    while(UTX0IF==0);
    UTX0IF=0;
  }
}

/***********************************************************************************
* @fn          appTimerISR
*
* @brief       32KHz timer interrupt service routine. Signals PER test transmitter
*              application to transmit a packet by setting application state.
*
* @param       none
*
* @return      none
*/
static void appTimerISR(void)
{
  appState = TRANSMIT_PACKET;
}


/***********************************************************************************
* @fn          appStartStop
*
* @brief       Joystick up interrupt service routine. Start or stop 32KHz timer,
*              and thereby start or stop PER test packet transmission.
*
* @param       none
*
* @return      none
*/
static void appStartStop(void)
{
  // toggle value
  appStarted ^= 1;
  
  if(appStarted) {
    halTimer32kIntEnable();
  }
  else {
    halTimer32kIntDisable();
  }
}


/***********************************************************************************
* @fn          appConfigTimer
*
* @brief       Configure timer interrupts for application. Uses 32KHz timer
*
* @param       uint16 rate - Frequency of timer interrupt. This value must be
*              between 1 and 32768 Hz
*
* @return      none
*/
static void appConfigTimer(uint16 rate)
{
  halTimer32kInit(TIMER_32K_CLK_FREQ/rate);
  halTimer32kIntConnect(&appTimerISR);
}


/***********************************************************************************
* @fn          appReceiver
*
* @brief       Application code for the receiver mode. Puts MCU in endless loop
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              rxPacket - file scope variable of type perTestPacket_t
*
* @return      none
*/
static void appReceiver()
{
  initUART();
  uint32 segNumber=0;
  int16 perRssiBuf[RSSI_AVG_WINDOW_SIZE] = {0};    // Ring buffer for RSSI
  uint8 perRssiBufCounter = 0;                     // Counter to keep track of the
  // oldest newest byte in RSSI
  // ring buffer
  perRxStats_t rxStats = {0,0,0,0};
  int16 rssi;
  uint8 resetStats=FALSE;
  int32 nper,temp_receive;
  
  char str[32];
  
  
  // Initialize BasicRF
  basicRfConfig.myAddr = RX_ADDR;
  if(basicRfInit(&basicRfConfig)==FAILED)
  {
    HAL_ASSERT(FALSE);
  }
  basicRfReceiveOn();
  
  // Main loop
  while (TRUE) 
  {
    while(!basicRfPacketIsReady());
    if(basicRfReceive((uint8*)&rxPacket, MAX_PAYLOAD_LENGTH, &rssi)>0) 
    {
      halLedToggle(1);
      
      
      //uint8 len =  sprintf(str,"addr:%02X%02X,rss=%d\n",rxPacket.padding[0],rxPacket.padding[1],rssi);
      uint8 len =  sprintf(str,"%d\n",rssi);
      UartTX_Send_String(str,len);
      
      
    }
  }
}





/***********************************************************************************
* @fn          appTransmitter
*
* @brief       Application code for the transmitter mode. Puts MCU in endless
*              loop
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              txPacket - file scope variable of type perTestPacket_t
*              appState - file scope variable. Holds application state
*              appStarted - file scope variable. Controls start and stop of
*                           transmission
*
* @return      none
*/
static void appTransmitter()
{
  uint32 burstSize=0;
  uint32 pktsSent=0;
  uint8 appTxPower;
  uint8 n;
  
  // Initialize BasicRF
  basicRfConfig.myAddr = TX_ADDR;
  if(basicRfInit(&basicRfConfig)==FAILED) {
    HAL_ASSERT(FALSE);
  }
  
  // Set TX output power
  halRfSetTxPower(2);
  
  // Set burst size
  burstSize =1000;
  
  // Basic RF puts on receiver before transmission of packet, and turns off
  // after packet is sent
  basicRfReceiveOff();
  
  // Config timer and IO
  
  appConfigTimer(0xC8);
  
  // Initalise packet payload
  txPacket.seqNumber = 0;

  //address
  txPacket.padding[0] = 0x01;
  txPacket.padding[1] = 0x02;
  
  
  // Main loop
  while (TRUE) {
    
    // Wait for user to start application
    while(appStarted) 
    {
      
      if (pktsSent < burstSize) 
      {
        
        // Make sure sequence number has network byte order
        UINT32_HTON(txPacket.seqNumber);
        
        basicRfSendPacket(RX_ADDR, (uint8*)&txPacket, PACKET_SIZE);
        
        // Change byte order back to host order before increment
        UINT32_NTOH(txPacket.seqNumber);
        txPacket.seqNumber++;
              pktsSent++;
        
        appState = IDLE; 
        halLedToggle(1);
        halMcuWaitMs(40);
        
        
      }
      else
      {
        pktsSent = 0;
        txPacket.seqNumber = 0;
      }
      
    }
    
    // Reset statistics and sequence number
    
  }
}


/***********************************************************************************
* @fn          main
*
* @brief       This is the main entry of the "PER test" application.
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              appState - file scope variable. Holds application state
*              appStarted - file scope variable. Used to control start and stop of
*              transmitter application.
*
* @return      none
*/
void main (void)
{
  uint8 appMode;
  
    
  appState = IDLE;
  
  
  
  appStarted = TRUE;
  
  // Config basicRF
  basicRfConfig.panId = PAN_ID;
  basicRfConfig.ackRequest = FALSE;
  
  // Initialise board peripherals
  halBoardInit();
  
  // Initalise hal_rf
  if(halRfInit()==FAILED) {
    HAL_ASSERT(FALSE);
  }
  
  // Indicate that device is powered
  halLedSet(1);
  
  // Print Logo and splash screen on LCD
  utilPrintLogo("PER Tester");
  
  // Wait for user to press S1 to enter menu
  halMcuWaitMs(350);
  
  // Set channel
  basicRfConfig.channel = 0x0B;
  
  // Set mode
  appMode = MODE_RX;
  
  // Transmitter application
  if(appMode == MODE_TX) {
    // No return from here
    appTransmitter();
  }
  // Receiver application
  else if(appMode == MODE_RX) {
    // No return from here
    appReceiver();
  }
  // Role is undefined. This code should not be reached
  HAL_ASSERT(FALSE);
}


