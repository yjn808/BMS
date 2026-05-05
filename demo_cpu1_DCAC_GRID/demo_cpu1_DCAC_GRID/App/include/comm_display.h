/*
 * comm.h
 *
 *  Created on: May 17, 2025
 *      Author: MAO
 */

#ifndef APP_INCLUDE_COMM_DISPLAY_H_
#define APP_INCLUDE_COMM_DISPLAY_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"
#include "basic.h"
#define TXBUF_MAX_BYTES    256 // Maximum size of transmit buffer is 256 bytes
#define RXBUF_MAX_BYTES    256 // Maximum size of receive buffer is 256 bytes

#define CTRL_SIZE    6

/**
 * Enumeration defining SCI communication ports
 */
enum SCI_COM
{
    SCI_A=0,  // SCI port A
    SCI_B,    // SCI port B
    SCI_C     // SCI port C
};

/**
 * Enumeration defining communication status
 */
enum COM_STATUS
{
    RX_BUSY=0,  // Receiving data (busy)
    RX_IDLE,    // Receive idle (no data being received)
    TX          // Transmitting data
};

/**
 * Enumeration defining transmit commands
 */
enum TX_CMD
{
    ONCE=0,   // Transmit data once
    REPEAT    // Transmit data repeatedly
};

/**
 * Structure for SCI communication management
 */
typedef struct {
                  enum SCI_COM Sci_Com_x;       // SCI port identifier
                  enum COM_STATUS Com_Status;   // Current communication status

                  Uint16 Tx_Data[TXBUF_MAX_BYTES];  // Transmit data buffer
                  Uint16 Tx_Length;                // Length of data to transmit
                  Uint16 Tx_Index;                 // Current index in transmit buffer
                  enum TX_CMD Tx_Cmd;               // Transmit command (once/repeat)

                  Uint16 Rx_Data[RXBUF_MAX_BYTES];  // Receive data buffer
                  Uint16 Wr_Index;                  // Write index for receive buffer
                  Uint16 Analysis_Index;            // Analysis index for receive buffer
                  Uint16 Rx_Idle_Count;             // Counter for receive idle time
               } SCI_COM;


/**
 * Initialize SCI communication structure
 * @param sci_com_x  SCI port (SCI_A/SCI_B/SCI_C)
 * @param p          Pointer to SCI_COM structure to initialize
 */
void Init_Sci_Com_Strcut(enum SCI_COM sci_com_x , SCI_COM *p);

/**
 * Handle SCI transmit and receive operations
 * @param p Pointer to SCI_COM structure
 */
void Sci_Tx_Rx( SCI_COM *p );

// External declaration of SCI_C_Com instance
extern SCI_COM Sci_C_Com;


/**
 * Structure for storing sampling coefficient data as strings
 */
typedef struct {
    char I_H_Gain_txt[20];      // High current gain (string format)
    char I_H_Offset_txt[20];    // High current offset (string format)

    char I_L_Gain_txt[20];      // Low current gain (string format)
    char I_L_Offset_txt[20];    // Low current offset (string format)

    char I_Inv_Gain_txt[20];    // Inverter current gain (string format)
    char I_Inv_Offset_txt[20];  // Inverter current offset (string format)

    char V_H_Gain_txt[20];      // High voltage gain (string format)
    char V_H_Offset_txt[20];    // High voltage offset (string format)

    char V_L_Gain_txt[20];      // Low voltage gain (string format)
    char V_L_Offset_txt[20];    // Low voltage offset (string format)

} SAMP_COEFF_TXT;

/**
 * Structure for storing device information data as strings
 */
typedef struct {
    char V_H_Rated_txt[20];     // Rated high voltage (string format)
    char V_L_Rated_txt[20];     // Rated low voltage (string format)
    char I_Rated_txt[20];       // Rated current (string format)
    char LINV_R_txt[20];        // Inverter resistance (string format)
    char Dev_Info[40];          // Device information string
    char Code_Info[20];         // Firmware/software version information

} DEV_INFO_TXT;


/**
 * Structure for storing power information data as strings
 */
typedef struct {
    char V_H_txt[20];           // High voltage measurement (string format)
    char I_H_txt[20];           // High current measurement (string format)

    char V_L_txt[20];           // Low voltage measurement (string format)
    char I_L_txt[20];           // Low current measurement (string format)

    char I_Inv_txt[20];         // Inverter current measurement (string format)

    char Power_H_txt[20];       // High power calculation (string format)
    char Power_L_txt[20];       // Low power calculation (string format)

} POWER_INFO_TXT;


/**
 * Structure for storing error flag data as strings
 */
typedef struct {
    char V_H_HW_Err_txt[20];    // High voltage hardware error status (string format)
    char V_L_HW_Err_txt[20];    // Low voltage hardware error status (string format)
    char I_INV_HW_Err_txt[20];  // Inverter current hardware error status (string format)

    char V_H_SW_Err_txt[20];    // High voltage software error status (string format)
    char V_L_SW_Err_txt[20];    // Low voltage software error status (string format)
    char I_INV_SW_Err_txt[20];  // Inverter current software error status (string format)

} ERR_FLAG_TXT;


/**
 * Structure for storing protection information data as strings
 */
typedef struct {
    char V_H_Prot_Rate_txt[20];   // High voltage protection threshold (string format)
    char V_L_Prot_Rate_txt[20];   // Low voltage protection threshold (string format)
    char I_INV_Prot_Rate_txt[20]; // Inverter current protection threshold (string format)

    ERR_FLAG_TXT Err_Flag_txt;    // Error flag statuses (string format)
    char Tim_Status_txt[20];      // Timing status (string format, read-only)
} PROT_INFO_TXT;

/**
 * Structure for storing control information data as strings
 */
typedef struct {
    char Ctrl_Loop_txt[20];        // Control loop mode (string format, writable only when stopped)

    char Feed_Ford_Enable_txt[20];    // Feed-forward enable status (string format)
    char Dead_Time_Comp_Enable_txt[20]; // Dead time compensation enable status (string format)

    char Out_Loop_Kp_txt[20];     // Outer loop proportional gain (string format)
    char Out_Loop_Ki_txt[20];     // Outer loop integral gain (string format)

    char In_Loop_Kp_txt[20];      // Inner loop proportional gain (string format)
    char In_Loop_Ki_txt[20];      // Inner loop integral gain (string format)

    char V_INV_Ref_txt[20];       // Inverter voltage reference (string format, used in OPEN_LOOP)
    char In_Loop_Ref_txt[20];     // Inner loop reference (string format, used in CLOSE_IN_LOOP)
    char Out_Loop_Ref_txt[20];    // Outer loop reference (string format, used in CLOSE_IN_OUT_LOOP)

} CTRL_TXT;



enum OLED_PAGE
{
    MAIN_PAGE=0,
    POWER_PAGE,
    PROTET_PAGE,
    CTRL_PAGE
};


void OLED_Page_Slect(void);
void OLED_Data_Show(void);
void All_Data_Update(void);
/**
 * Parse received JSON data from SCI
 * @param p Pointer to SCI_COM structure containing received data
 */
void parseJSON(SCI_COM *p);

/**
 * Pack sampling coefficient data into JSON format for transmission
 * @param p Pointer to SCI_COM structure for transmitting the JSON data
 */
void packJSON_Samp_Coff(SCI_COM *p);

/**
 * Pack device information into JSON format for transmission
 * @param p Pointer to SCI_COM structure for transmitting the JSON data
 */
void packJSON_Dev_Info(SCI_COM *p);

/**
 * Pack power information into JSON format for transmission
 * @param p Pointer to SCI_COM structure for transmitting the JSON data
 */
void packJSON_Power_Info(SCI_COM *p);

/**
 * Pack protection information into JSON format for transmission
 * @param p Pointer to SCI_COM structure for transmitting the JSON data
 */
void packJSON_Prot_Info(SCI_COM *p);

/**
 * Pack control information into JSON format for transmission
 * @param p Pointer to SCI_COM structure for transmitting the JSON data
 */
void packJSON_Ctrl_Info(SCI_COM *p);

/**
 * Convert an integer to its string representation
 * @param num    The integer to convert
 * @param buffer The buffer to store the resulting string
 * @param pos    Pointer to the current position in the buffer (updated during conversion)
 */
void int_to_str(int num, char* buffer, int* pos);

/**
 * Convert a float to its string representation (with 3 decimal places)
 * @param num    The float to convert
 * @param buffer The buffer to store the resulting string
 * @param pos    Pointer to the current position in the buffer (updated during conversion)
 */
void float_to_str(float num, char* buffer, int* pos);

/**
 * Custom implementation of sprintf (supports %d and %f formats)
 * @param buffer The buffer to store the formatted string
 * @param format The format string specifying the output format
 * @param ...    Variable arguments to be formatted
 */
void my_sprintf(char* buffer, const char* format, ...);

#endif /* APP_INCLUDE_COMM_DISPLAY_H_ */
