/**************************************************************************************************
 
  Shanghai QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Shanghai QST 
  Corporation ("QST"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of QST. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a QST Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/**************************************************************************************************
  Filename:       simpleBLEperipheral.h
  Revised:         
  Revision:        

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

 
**************************************************************************************************/

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H


/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

#define INVALID_CONNHANDLE                    0xFFFF

// Simple BLE Peripheral Task Events
// Simple BLE Peripheral Task Events
#define BUP_OSAL_EVT_START_DEVICE                         0x0001
#define BUP_OSAL_EVT_BLE_TIMER                            0x0002
#define BUP_OSAL_EVT_ENTER_NOCONN                         0x0004
#define BUP_OSAL_EVT_RESET_ADV                            0x0008

#define BUP_OSAL_EVT_CCCD_UPDATE                          0x0010
#define BUP_OSAL_EVT_UART_DATA_RX                         0x0020
#define BUP_OSAL_EVT_NOTIFY_DATA                          0x0040
#define BUP_OSAL_EVT_UARTRX_TIMER                         0x0080
#define BUP_OSAL_EVT_UART_TX_COMPLETE                     0x0100
#define BUP_OSAL_EVT_UART_TO_TIMER                        0x0200



#define FLOW_CTRL_IO_UART_TX          P18 //mobile --> ble --> uart --> host
#define FLOW_CTRL_IO_BLE_TX           P23 //host-->uart-->ble-->mobile
#define FLOW_CTRL_IO_HOST_WAKEUP      P15 //host mcu wakeup befor host-->uart-->QMS7926
#define FLOW_CTRL_IO_BLE_CONNECTION   P20 //indicate host QMS7926 BLE connection status: 1: connected; 0: advertising


#define io_lock(io) {hal_gpio_write(io, 1);hal_gpio_pull_set(io, STRONG_PULL_UP);}
#define io_unlock(io) {hal_gpio_write(io, 0);hal_gpio_pull_set(io, PULL_DOWN);}

#define FLOW_CTRL_UART_TX_LOCK()    io_lock(FLOW_CTRL_IO_UART_TX)
#define FLOW_CTRL_UART_TX_UNLOCK()  io_unlock(FLOW_CTRL_IO_UART_TX)

#define FLOW_CTRL_BLE_TX_LOCK()     io_lock(FLOW_CTRL_IO_BLE_TX)
#define FLOW_CTRL_BLE_TX_UNLOCK()   io_unlock(FLOW_CTRL_IO_BLE_TX)

#define FLOW_CTRL_BLE_CONN()     io_lock(FLOW_CTRL_IO_BLE_CONNECTION)
#define FLOW_CTRL_BLE_DISCONN()   io_unlock(FLOW_CTRL_IO_BLE_CONNECTION)

extern uint8 bleuart_TaskID;   // Task ID for internal task/event processing
extern uint16 gapConnHandle;


void bleuart_Init( uint8 task_id );
uint16_t bleuart_conn_interval(void);
uint16 bleuart_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#endif /* SIMPLEBLEPERIPHERAL_H */
