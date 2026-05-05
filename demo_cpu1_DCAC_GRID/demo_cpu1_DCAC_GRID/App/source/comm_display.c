/*
 * comm.c
 *
 *  Created on: May 17, 2025
 *      Author: MAO
 *  Description: This file implements SCI (Serial Communication Interface) transmission/reception control,
 *               JSON data parsing and packaging functions for communication between the device and external systems,
 *               supporting real-time data transmission and parameter configuration.
 */

#include "cJSON.h"
#include "comm_display.h"
#include <string.h>
#include <stdlib.h>
#include "sci.h"
#include <stdarg.h>
#include "led.h"
#include "key.h"
#include "oled.h"
// SCI communication structure instance (manages transmission/reception status and data buffers for SCI ports)
SCI_COM Sci_C_Com;

// Various parameter text structure instances (store string representations of parameters for transmission)
SAMP_COEFF_TXT Samp_Coff_Txt;    // Sampling coefficient text structure (stores string representations of coefficients)
DEV_INFO_TXT Dev_Info_Txt;      // Device information text structure (stores string representations of device parameters)
POWER_INFO_TXT Power_Info_Txt;  // Power information text structure (stores string representations of power parameters)
PROT_INFO_TXT Prot_Info_Txt;    // Protection information text structure (stores string representations of protection parameters)
CTRL_TXT Ctrl_Txt;              // Control information text structure (stores string representations of control parameters)

enum OLED_PAGE Oled_Page = MAIN_PAGE;
/**
 * Initialize SCI communication structure
 * @param Com_x  SCI port selection (SCI_A/SCI_B/SCI_C)
 * @param p      Pointer to SCI_COM structure (communication structure to be initialized)
 * Function: Resets all structure members to zero and sets initial states, including reception status,
 *           transmission/reception indices, port selection, etc.
 */
void Init_Sci_Com_Strcut(enum SCI_COM Com_x , SCI_COM *p)
{
    memset( (void *) p , 0 , sizeof( SCI_COM) );  // Initialize all structure members to zero
    p->Com_Status = RX_BUSY;       // Initialize to receive busy status (waiting for data reception)
    p->Tx_Length = 0;              // Initialize transmit data length to 0
    p->Tx_Index = 0;               // Initialize transmit index to 0 (points to the start of data to be sent)
    p->Tx_Cmd = ONCE;              // Initialize transmit command to single transmission (stop after one transmission)
    p->Wr_Index = 0;               // Initialize write index to 0 (points to the next position to store received data)
    p->Analysis_Index = 0;         // Initialize analysis index to 0 (points to the next data to be parsed)
    p->Rx_Idle_Count = 0;          // Initialize receive idle counter to 0 (counts idle time between receptions)
    p->Sci_Com_x = Com_x;          // Set SCI port (specify which SCI peripheral to use)
}

#pragma CODE_SECTION(Sci_Tx_Rx,".TI.ramfunc");  // Place function in RAM for faster execution
/**
 * SCI transmit and receive processing function
 * @param p Pointer to SCI_COM structure (manages current communication state)
 * Function: Handles data reception when no transmission is active; handles data transmission when there is data to send.
 *           Switches between receive/transmit modes and manages FIFO buffers for efficient data handling.
 */
void Sci_Tx_Rx( SCI_COM *p )
{
    volatile struct SCI_REGS *v;   // Pointer to SCI register structure (accesses hardware registers)
    // Select corresponding registers based on SCI port
    switch(p->Sci_Com_x)
    {
        case SCI_A:
        {
            v = &SciaRegs;
            break;
        }
        case SCI_B:
        {
            v = &ScibRegs;
            break;
        }
        case SCI_C:
        {
            v = &ScicRegs;
            break;
        }
    }
    // Perform reception processing when no data is being transmitted
    if( (p->Tx_Length == 0) && (p->Tx_Index==0) )
    {
        // Configure hardware for receive mode based on port
        if(p->Sci_Com_x==SCI_B)
        {
            SCI_B_RX;
        }
        else if(p->Sci_Com_x==SCI_C)
        {
            SCI_C_RX;
        }
        v->SCIFFRX.all |= 0x4040;   // Clear receive FIFO interrupt flag (0x4040 resets RXFFINT and RXFFOVF)
        // Read received data if receive FIFO interrupt is triggered (data available)
        if( v->SCIFFRX.bit.RXFFINT == 1 )
        {
            p->Rx_Idle_Count=0;         // Reset idle counter (data is being received)
            p->Com_Status = RX_BUSY;    // Set status to receive busy
            // Cyclically read all data in the receive FIFO
            do
            {
                p->Rx_Data[p->Wr_Index]=v->SCIRXBUF.bit.SAR;  // Read received data from SCI receive buffer
                p->Wr_Index++;                                 // Increment write index (move to next storage position)
                if(p->Wr_Index==RXBUF_MAX_BYTES)               // Reset index when maximum buffer size is reached (circular buffer)
                {
                    p->Wr_Index=0;
                }
            }while(v->SCIFFRX.bit.RXFFST!=0);  // Continue until receive FIFO is empty (RXFFST = FIFO status)
        }
        else
        {
            p->Rx_Idle_Count++;    // Increment idle counter (no new data received)
            // Set to receive idle status if idle time exceeds threshold
            if( p->Rx_Idle_Count > (TXFIFO_MAX_BYTES>>3) )
            {
                p->Rx_Idle_Count = (TXFIFO_MAX_BYTES>>3);  // Cap idle counter at threshold
                p->Com_Status = RX_IDLE;                   // Set status to receive idle (no more data expected)
            }
            else
            {
                p->Com_Status = RX_BUSY;  // Maintain receive busy status (potential data still arriving)
            }
        }
    }
    else
    {
        // Configure hardware for transmit mode based on port
        if(p->Sci_Com_x==SCI_B)
        {
            SCI_B_TX;
        }
        else if(p->Sci_Com_x==SCI_C)
        {
            SCI_C_TX;
        }
        p->Com_Status = TX;         // Set status to transmitting
        p->Rx_Idle_Count = 0;       // Reset receive idle counter during transmission
        v->SCIFFTX.bit.TXFFINTCLR = 1;  // Clear transmit FIFO interrupt flag
        // Handle transmit FIFO interrupt (FIFO has space for more data)
        if( v->SCIFFTX.bit.TXFFINT == 1 )
        {
            if( p->Tx_Length == p->Tx_Index ) // All data has been written to FIFO
            {
                    if(p->Tx_Cmd==REPEAT)    // If repeat transmission is enabled
                    {
                        p->Tx_Index=0;        // Reset transmit index to retransmit data
                    }
                    else
                    {
                        p->Tx_Index=0;        // Reset transmit index
                        p->Tx_Length=0;       // Clear transmit length (transmission complete)
                        return;
                    }
            }
        }
        // Write data to transmit FIFO until full or all data is sent
        while( v->SCIFFTX.bit.TXFFST != TXFIFO_MAX_BYTES && p->Tx_Index < p->Tx_Length)
        {
            v->SCITXBUF.bit.TXDT = p->Tx_Data[p->Tx_Index];  // Write data to SCI transmit buffer
            p->Tx_Index++;                                   // Increment transmit index (move to next data byte)
        }
    }
}

#pragma CODE_SECTION(parseJSON,".TI.ramfunc");  // Place function in RAM for faster execution
/**
 * Parse JSON data from received SCI data
 * @param p Pointer to SCI_COM structure (contains received data in Rx_Data buffer)
 * Function: Parses JSON-formatted data from the receive buffer when reception is complete.
 *           Handles "read" commands by triggering corresponding JSON packaging functions based on the requested data type.
 */
void parseJSON(SCI_COM *p)
{
    int i = 0;
    float data_temp[CTRL_SIZE];
    float data_check=0.0f;

    cJSON *json = NULL;           // cJSON root object pointer (parsed JSON structure)
    cJSON *json_temp = NULL;      // Temporary cJSON object pointer (accesses specific fields)
    char *text = (char *)( &(p->Rx_Data) );  // Pointer to received data buffer (treats byte array as string)


    // Process data when reception is idle and there is unanalyzed data
    if(p->Com_Status == RX_IDLE && p->Wr_Index != p->Analysis_Index)
    {
        p->Wr_Index = 0;           // Reset write index (prepare for next reception)
        p->Analysis_Index = 0;     // Reset analysis index (mark data as processed)

        json=cJSON_Parse(text);    // Parse JSON data from the receive buffer
        if(!json)                  // Check if parsing failed (invalid JSON format)
        {
            // No handling for parsing failure (can be extended with error logging)
        }
        else
        {
         json_temp=cJSON_GetObjectItem(json,"Fun");  // Get "Fun" field (specifies command type: read/write)

         // Handle write command ("W" indicates write operation)
         if( strcmp( json_temp->valuestring , "W" ) ==0 )
         {
             json_temp=cJSON_GetObjectItem(json,"Data");  // Get "Data" field (specifies data type to read)
             if(strcmp( json_temp->valuestring , "Ctrl" ) ==0)
             {
                 json_temp=cJSON_GetObjectItem(json,"Dead_Time");
                 if( strcmp( json_temp->valuestring , "Enable" ) ==0 )
                 {
                     Ctrl.Dead_Time_Comp_Enable = true;
                 }
                 else if(strcmp( json_temp->valuestring , "Disable" ) ==0)
                 {
                     Ctrl.Dead_Time_Comp_Enable = false;
                 }

                 json_temp=cJSON_GetObjectItem(json,"Feed_Ford");
                 if( strcmp( json_temp->valuestring , "Enable" ) ==0 )
                 {
                     Ctrl.Feed_Ford_Enable = true;
                 }
                 else if(strcmp( json_temp->valuestring , "Disable" ) ==0)
                 {
                     Ctrl.Feed_Ford_Enable = false;
                 }

                 if(Prot_Info.Tim_Status == STOP)
                 {
                     json_temp=cJSON_GetObjectItem(json,"Loop");
                     if(strcmp( json_temp->valuestring , "OPEN" ) ==0)
                     {
                         Ctrl.Ctrl_Loop = OPEN_LOOP;

                         Ctrl.In_Loop_Ref = 0.0f;
                         Ctrl.Out_Loop_Ref = 0.0f;
                     }
                     else if(strcmp( json_temp->valuestring , "IN" ) ==0 )
                     {
                         Ctrl.Ctrl_Loop = CLOSE_IN_LOOP;

                         Ctrl.Out_Loop_Ref = 0.0f;
                         Ctrl.V_INV_Ref = 0.0f;
                     }
                     else if(strcmp( json_temp->valuestring , "OUT" ) ==0 )
                     {
                         Ctrl.Ctrl_Loop = CLOSE_IN_OUT_LOOP;

                         Ctrl.In_Loop_Ref = 0.0f;
                         Ctrl.V_INV_Ref = 0.0f;
                     }
                 }

                 json_temp=cJSON_GetObjectItem(json,"Data_Item");

                if ( cJSON_IsArray(json_temp) && cJSON_GetArraySize(json_temp) == CTRL_SIZE )
                {
                  for ( i = 0; i < CTRL_SIZE-1; i++ )
                  {
                      data_temp[i] = cJSON_GetArrayItem(json_temp, i)->valuedouble;
                      data_check += data_temp[i];
                  }

                  if( fabsf (data_check - cJSON_GetArrayItem(json_temp, CTRL_SIZE-1)->valuedouble)<0.001f )
                  {
                      Ctrl.In_Loop_Kp = data_temp[0];
                      Ctrl.In_Loop_Ki = data_temp[1];
                      Ctrl.Out_Loop_Kp = data_temp[2];
                      Ctrl.Out_Loop_Ki = data_temp[3];
                      if(Ctrl.Ctrl_Loop == OPEN_LOOP)
                      {
                          Ctrl.V_INV_Ref = UPD_LIMIT(data_temp[4], 16.0f, 0.0f);
                      }else if(Ctrl.Ctrl_Loop == CLOSE_IN_LOOP)
                      {
                          Ctrl.In_Loop_Ref = UPD_LIMIT(data_temp[4], 3.0f, -3.0f);
                      }
                      else
                      {
                          Ctrl.Out_Loop_Ref = UPD_LIMIT(data_temp[4], 10.0f, 6.0f);
                      }
                  }
                }
             }
             else if(strcmp( json_temp->valuestring , "CMD" ) ==0 )
             {
                 json_temp=cJSON_GetObjectItem(json,"Tim_Cmd");
                 if( strcmp( json_temp->valuestring , "Stop" ) ==0 )
                 {
                     Cmd.Ctrl_Cmd = STOP_CMD;
                 }
                 else if( strcmp( json_temp->valuestring , "Start" ) ==0 )
                 {
                     Cmd.Ctrl_Cmd = START_CMD;
                 }

                 json_temp=cJSON_GetObjectItem(json,"Rst_Cmd");
                 if( strcmp( json_temp->valuestring , "Enable" ) ==0 )
                 {
                     Cmd.Err_Flag_Rst_Cmd = true;
                 }
             }
             // Write command handling (to be implemented: update device parameters from JSON data)
         }
         // Handle read command ("R" indicates read operation)
         else if( strcmp( json_temp->valuestring , "R" ) ==0 )
         {
             json_temp=cJSON_GetObjectItem(json,"Data");  // Get "Data" field (specifies data type to read)

             // Pack and send sampling coefficient data if requested
             if(strcmp( json_temp->valuestring , "Samp" ) ==0)
             {
                 packJSON_Samp_Coff(p);
             }
             // Pack and send device information if requested
             else if(strcmp( json_temp->valuestring , "Dev" ) ==0)
             {
                 packJSON_Dev_Info(p);
             }
             // Pack and send power information if requested
             else if(strcmp( json_temp->valuestring , "Power" ) ==0)
             {
                 packJSON_Power_Info(p);
             }
             // Pack and send protection information if requested
             else if(strcmp( json_temp->valuestring , "Prot" ) ==0)
             {
                 packJSON_Prot_Info(p);
             }
             // Pack and send control information if requested
             else if(strcmp( json_temp->valuestring , "Ctrl" ) ==0)
             {
                 packJSON_Ctrl_Info(p);
             }
         }
           cJSON_Delete(json);     // Delete cJSON object to free allocated memory
           free(json);             // Free json pointer (prevent memory leaks)
        }
    }
}

#pragma CODE_SECTION(packJSON_Samp_Coff,".TI.ramfunc");  // Place function in RAM for faster execution
/**
 * Pack sampling coefficient data into JSON format
 * @param p Pointer to SCI_COM structure (used to store JSON string for transmission)
 * Function: Converts sampling coefficient data (gain, offset) into a JSON-formatted string,
 *           stores it in the transmit buffer, and sets the transmit length for transmission.
 */
void packJSON_Samp_Coff(SCI_COM *p)
{
    cJSON *json_data=NULL;        // cJSON object pointer (constructs JSON structure)
    char *out_data;               // Pointer to output JSON string (formatted JSON data)

    json_data=cJSON_CreateObject();  // Create root JSON object

    // Add sampling coefficient fields to JSON object (gain and offset for various channels)
    cJSON_AddStringToObject(json_data,"I_H_K",Samp_Coff_Txt.I_H_Gain_txt);
    cJSON_AddStringToObject(json_data,"I_H_B",Samp_Coff_Txt.I_H_Offset_txt);

    cJSON_AddStringToObject(json_data,"I_Inv_K",Samp_Coff_Txt.I_Inv_Gain_txt);
    cJSON_AddStringToObject(json_data,"I_Inv_B",Samp_Coff_Txt.I_Inv_Offset_txt);

    cJSON_AddStringToObject(json_data,"I_L_K",Samp_Coff_Txt.I_L_Gain_txt);
    cJSON_AddStringToObject(json_data,"I_L_B",Samp_Coff_Txt.I_L_Offset_txt);

    cJSON_AddStringToObject(json_data,"V_H_K",Samp_Coff_Txt.V_H_Gain_txt);
    cJSON_AddStringToObject(json_data,"V_H_B",Samp_Coff_Txt.V_H_Offset_txt);

    cJSON_AddStringToObject(json_data,"V_L_K",Samp_Coff_Txt.V_L_Gain_txt);
    cJSON_AddStringToObject(json_data,"V_L_B",Samp_Coff_Txt.V_L_Offset_txt);

    out_data = cJSON_Print(json_data);  // Convert JSON object to formatted string
    cJSON_Delete(json_data);            // Delete cJSON object to free memory

    // Copy JSON string to transmit buffer and set transmit length
    strcpy( (char *)(&(p->Tx_Data)) ,  out_data);
    p->Tx_Length=strlen(out_data);

    cJSON_free(out_data);  // Free memory allocated for the JSON string
}

#pragma CODE_SECTION(packJSON_Dev_Info,".TI.ramfunc");  // Place function in RAM for faster execution
/**
 * Pack device information into JSON format
 * @param p Pointer to SCI_COM structure (used to store JSON string for transmission)
 * Function: Converts device information (rated parameters, device codes) into a JSON-formatted string,
 *           stores it in the transmit buffer, and sets the transmit length for transmission.
 */
void packJSON_Dev_Info(SCI_COM *p)
{
    cJSON *json_data=NULL;        // cJSON object pointer (constructs JSON structure)
    char *out_data;               // Pointer to output JSON string (formatted JSON data)

    json_data=cJSON_CreateObject();  // Create root JSON object

    // Add device information fields to JSON object (identification and rated parameters)
    cJSON_AddStringToObject(json_data,"Code_Info",Dev_Info_Txt.Code_Info);
    cJSON_AddStringToObject(json_data,"Dev_Info",Dev_Info_Txt.Dev_Info);

    cJSON_AddStringToObject(json_data,"I_Rated",Dev_Info_Txt.I_Rated_txt);
    cJSON_AddStringToObject(json_data,"LINV_R",Dev_Info_Txt.LINV_R_txt);
    cJSON_AddStringToObject(json_data,"V_H_Rated",Dev_Info_Txt.V_H_Rated_txt);
    cJSON_AddStringToObject(json_data,"V_L_Rated",Dev_Info_Txt.V_L_Rated_txt);

    out_data = cJSON_Print(json_data);  // Convert JSON object to formatted string
    cJSON_Delete(json_data);            // Delete cJSON object to free memory

    // Copy JSON string to transmit buffer and set transmit length
    strcpy( (char *)(&(p->Tx_Data)) ,  out_data);
    p->Tx_Length=strlen(out_data);

    cJSON_free(out_data);  // Free memory allocated for the JSON string
}

#pragma CODE_SECTION(packJSON_Power_Info,".TI.ramfunc");  // Place function in RAM for faster execution (critical for real-time data transmission)
/**
 * Pack power-related measurement data into a JSON-formatted string for transmission
 * @param p Pointer to SCI_COM structure that holds the transmit buffer and transmission parameters
 * Function: Constructs a JSON object containing voltage, current, and calculated power values,
 *           converts it to a string, and loads it into the SCI transmit buffer for sending.
 */
void packJSON_Power_Info(SCI_COM *p)
{
    cJSON *json_data=NULL;        // Pointer to root cJSON object (will hold all power data fields)
    char *out_data;               // Pointer to the serialized JSON string (output of cJSON_Print)

    json_data=cJSON_CreateObject();  // Create empty root JSON object

    // Add voltage and current measurement fields (high-voltage and low-voltage channels)
    cJSON_AddStringToObject(json_data,"V_H",Power_Info_Txt.V_H_txt);  // High voltage value string
    cJSON_AddStringToObject(json_data,"I_H",Power_Info_Txt.I_H_txt);  // High current value string

    cJSON_AddStringToObject(json_data,"V_L",Power_Info_Txt.V_L_txt);  // Low voltage value string
    cJSON_AddStringToObject(json_data,"I_L",Power_Info_Txt.I_L_txt);  // Low current value string

    cJSON_AddStringToObject(json_data,"I_Inv",Power_Info_Txt.I_Inv_txt);  // Inverter current value string

    // Add calculated power fields for high and low voltage channels
    cJSON_AddStringToObject(json_data,"Power_H",Power_Info_Txt.Power_H_txt);  // High channel power string
    cJSON_AddStringToObject(json_data,"Power_L",Power_Info_Txt.Power_L_txt);  // Low channel power string

    out_data = cJSON_Print(json_data);  // Convert cJSON object to human-readable JSON string
    cJSON_Delete(json_data);            // Free memory allocated for the cJSON object (prevent leaks)

    // Copy JSON string to SCI transmit buffer and set transmission length
    strcpy( (char *)(&(p->Tx_Data)) ,  out_data);  // Load string into transmit buffer
    p->Tx_Length=strlen(out_data);                 // Set length of data to transmit

    cJSON_free(out_data);  // Free memory allocated by cJSON_Print for the JSON string
}

#pragma CODE_SECTION(packJSON_Prot_Info,".TI.ramfunc");  // Place function in RAM for faster execution (critical for real-time protection messaging)
/**
 * Pack protection system information into a JSON-formatted string for transmission
 * @param p Pointer to SCI_COM structure that holds the transmit buffer and transmission parameters
 * Function: Constructs a JSON object containing protection thresholds, error flags, and system status,
 *           converts it to a string, and loads it into the SCI transmit buffer for sending.
 */
void packJSON_Prot_Info(SCI_COM *p)
{
    cJSON *json_data=NULL;        // Pointer to root cJSON object (will hold all protection data fields)
    char *out_data;               // Pointer to the serialized JSON string (output of cJSON_Print)

    json_data=cJSON_CreateObject();  // Create empty root JSON object

    // Add protection threshold fields (maximum allowable values for safety)
    cJSON_AddStringToObject(json_data,"I_INV_P",Prot_Info_Txt.I_INV_Prot_Rate_txt);  // Inverter current threshold
    cJSON_AddStringToObject(json_data,"V_H_P",Prot_Info_Txt.V_H_Prot_Rate_txt);      // High voltage threshold
    cJSON_AddStringToObject(json_data,"V_L_P",Prot_Info_Txt.V_L_Prot_Rate_txt);      // Low voltage threshold

    // Add hardware and software error flags for each monitored parameter
    cJSON_AddStringToObject(json_data,"I_INV_HW",Prot_Info_Txt.Err_Flag_txt.I_INV_HW_Err_txt);  // Inverter current hardware error
    cJSON_AddStringToObject(json_data,"I_INV_SW",Prot_Info_Txt.Err_Flag_txt.I_INV_SW_Err_txt);  // Inverter current software error

    cJSON_AddStringToObject(json_data,"V_H_HW",Prot_Info_Txt.Err_Flag_txt.V_H_HW_Err_txt);      // High voltage hardware error
    cJSON_AddStringToObject(json_data,"V_H_SW",Prot_Info_Txt.Err_Flag_txt.V_H_SW_Err_txt);      // High voltage software error

    cJSON_AddStringToObject(json_data,"V_L_HW",Prot_Info_Txt.Err_Flag_txt.V_L_HW_Err_txt);      // Low voltage hardware error
    cJSON_AddStringToObject(json_data,"V_L_SW",Prot_Info_Txt.Err_Flag_txt.V_L_SW_Err_txt);      // Low voltage software error

    // Add timing system status (running/stopped)
    cJSON_AddStringToObject(json_data,"Timing",Prot_Info_Txt.Tim_Status_txt);  // Timing system status string

    out_data = cJSON_Print(json_data);  // Convert cJSON object to human-readable JSON string
    cJSON_Delete(json_data);            // Free memory allocated for the cJSON object (prevent leaks)

    // Copy JSON string to SCI transmit buffer and set transmission length
    strcpy( (char *)(&(p->Tx_Data)) ,  out_data);  // Load string into transmit buffer
    p->Tx_Length=strlen(out_data);                 // Set length of data to transmit

    cJSON_free(out_data);  // Free memory allocated by cJSON_Print for the JSON string
}

#pragma CODE_SECTION(packJSON_Ctrl_Info,".TI.ramfunc");  // Place function in RAM for faster execution (critical for real-time control parameter messaging)
/**
 * Pack control system parameters into a JSON-formatted string for transmission
 * @param p Pointer to SCI_COM structure that holds the transmit buffer and transmission parameters
 * Function: Constructs a JSON object containing PID gains, reference values, and control mode flags,
 *           converts it to a string, and loads it into the SCI transmit buffer for sending.
 */
void packJSON_Ctrl_Info(SCI_COM *p)
{
    cJSON *json_data=NULL;        // Pointer to root cJSON object (will hold all control data fields)
    char *out_data;               // Pointer to the serialized JSON string (output of cJSON_Print)

    json_data=cJSON_CreateObject();  // Create empty root JSON object

    // Add inner loop PID controller parameters (proportional and integral gains)
    cJSON_AddStringToObject(json_data,"In_Kp",Ctrl_Txt.In_Loop_Kp_txt);  // Inner loop proportional gain
    cJSON_AddStringToObject(json_data,"In_Ki",Ctrl_Txt.In_Loop_Ki_txt);  // Inner loop integral gain

    // Add outer loop PID controller parameters (proportional and integral gains)
    cJSON_AddStringToObject(json_data,"Out_Kp",Ctrl_Txt.Out_Loop_Kp_txt);  // Outer loop proportional gain
    cJSON_AddStringToObject(json_data,"Out_Ki",Ctrl_Txt.Out_Loop_Ki_txt);  // Outer loop integral gain

    // Add reference/setpoint values for control loops and inverter voltage
    cJSON_AddStringToObject(json_data,"In_Ref",Ctrl_Txt.In_Loop_Ref_txt);      // Inner loop reference value
    cJSON_AddStringToObject(json_data,"Out_Ref",Ctrl_Txt.Out_Loop_Ref_txt);    // Outer loop reference value
    cJSON_AddStringToObject(json_data,"V_INV_Ref",Ctrl_Txt.V_INV_Ref_txt);     // Inverter voltage reference

    // Add control mode and auxiliary function flags
    cJSON_AddStringToObject(json_data,"Ctrl",Ctrl_Txt.Ctrl_Loop_txt);          // Control loop mode (open/closed)
    cJSON_AddStringToObject(json_data,"Feed_Ford",Ctrl_Txt.Feed_Ford_Enable_txt);  // Feed-forward control enable status
    cJSON_AddStringToObject(json_data,"Dead_Time",Ctrl_Txt.Dead_Time_Comp_Enable_txt);  // Dead-time compensation enable status

    out_data = cJSON_Print(json_data);  // Convert cJSON object to human-readable JSON string
    cJSON_Delete(json_data);            // Free memory allocated for the cJSON object (prevent leaks)

    // Copy JSON string to SCI transmit buffer and set transmission length
    strcpy( (char *)(&(p->Tx_Data)) ,  out_data);  // Load string into transmit buffer
    p->Tx_Length=strlen(out_data);                 // Set length of data to transmit

    cJSON_free(out_data);  // Free memory allocated by cJSON_Print for the JSON string
}



/**
 * Update all parameter text buffers with current values from system data structures
 * Function: Converts numerical parameters (floats, enums, booleans) from core system structures
 *           to formatted strings and stores them in corresponding text buffers. These text buffers
 *           are later used by JSON packaging functions to construct transmission data.
 */
void All_Data_Update(void)
{
    // Convert sampling coefficient floating-point values to formatted strings
    my_sprintf(Samp_Coff_Txt.I_H_Gain_txt, "%f", Samp_Coff.I_H_Gain);       // High current channel gain
    my_sprintf(Samp_Coff_Txt.I_H_Offset_txt, "%f", Samp_Coff.I_H_Offset);   // High current channel offset

    my_sprintf(Samp_Coff_Txt.I_L_Gain_txt, "%f", Samp_Coff.I_L_Gain);       // Low current channel gain
    my_sprintf(Samp_Coff_Txt.I_L_Offset_txt, "%f", Samp_Coff.I_L_Offset);   // Low current channel offset

    my_sprintf(Samp_Coff_Txt.I_Inv_Gain_txt, "%f", Samp_Coff.I_Inv_Gain);   // Inverter current channel gain
    my_sprintf(Samp_Coff_Txt.I_Inv_Offset_txt, "%f", Samp_Coff.I_Inv_Offset); // Inverter current channel offset

    my_sprintf(Samp_Coff_Txt.V_H_Gain_txt, "%f", Samp_Coff.V_H_Gain);       // High voltage channel gain
    my_sprintf(Samp_Coff_Txt.V_H_Offset_txt, "%f", Samp_Coff.V_H_Offset);   // High voltage channel offset

    my_sprintf(Samp_Coff_Txt.V_L_Gain_txt, "%f", Samp_Coff.V_L_Gain);       // Low voltage channel gain
    my_sprintf(Samp_Coff_Txt.V_L_Offset_txt, "%f", Samp_Coff.V_L_Offset);   // Low voltage channel offset

    // Convert device information parameters to formatted strings
    my_sprintf(Dev_Info_Txt.V_H_Rated_txt, "%f", Dev_Info.V_H_Rated);       // Rated high voltage
    my_sprintf(Dev_Info_Txt.V_L_Rated_txt, "%f", Dev_Info.V_L_Rated);       // Rated low voltage
    my_sprintf(Dev_Info_Txt.I_Rated_txt, "%f", Dev_Info.I_Rated);           // Rated current
    my_sprintf(Dev_Info_Txt.LINV_R_txt, "%f", Dev_Info.LINV_R);             // Inverter resistance

    strcpy( Dev_Info_Txt.Dev_Info , Dev_Info.Dev_Info);    // Device description string
    strcpy( Dev_Info_Txt.Code_Info , Dev_Info.Code_Info);  // Device code/version string

    // Convert power measurement values to formatted strings
    my_sprintf(Power_Info_Txt.I_H_txt, "%f", Power_Info.I_H);       // Measured high current
    my_sprintf(Power_Info_Txt.I_Inv_txt, "%f", Power_Info.I_Inv);   // Measured inverter current
    my_sprintf(Power_Info_Txt.I_L_txt, "%f", Power_Info.I_L);       // Measured low current
    my_sprintf(Power_Info_Txt.V_H_txt, "%f", Power_Info.V_H);       // Measured high voltage
    my_sprintf(Power_Info_Txt.V_L_txt, "%f", Power_Info.V_L);       // Measured low voltage

    my_sprintf(Power_Info_Txt.Power_H_txt, "%f", Power_Info.Power_H); // Calculated high channel power
    my_sprintf(Power_Info_Txt.Power_L_txt, "%f", Power_Info.Power_L); // Calculated low channel power

    // Convert protection system parameters to formatted strings
    my_sprintf(Prot_Info_Txt.I_INV_Prot_Rate_txt, "%f", Prot_Info.I_INV_Prot_Rate); // Inverter current protection threshold
    my_sprintf(Prot_Info_Txt.V_H_Prot_Rate_txt, "%f", Prot_Info.V_H_Prot_Rate);     // High voltage protection threshold
    my_sprintf(Prot_Info_Txt.V_L_Prot_Rate_txt, "%f", Prot_Info.V_L_Prot_Rate);     // Low voltage protection threshold

    // Convert timing system status enum to string ("STOP" or "RUNNING")
    Prot_Info.Tim_Status == STOP ? strcpy(Prot_Info_Txt.Tim_Status_txt, "STOP") : strcpy(Prot_Info_Txt.Tim_Status_txt, "RUNNING");

    // Convert boolean error flags to "Err" or "No_Err" strings
    Prot_Info.Err_Flag.I_INV_HW_Err == true ? strcpy(Prot_Info_Txt.Err_Flag_txt.I_INV_HW_Err_txt, "Err") : strcpy(Prot_Info_Txt.Err_Flag_txt.I_INV_HW_Err_txt, "No_Err");
    Prot_Info.Err_Flag.I_INV_SW_Err == true ? strcpy(Prot_Info_Txt.Err_Flag_txt.I_INV_SW_Err_txt, "Err") : strcpy(Prot_Info_Txt.Err_Flag_txt.I_INV_SW_Err_txt, "No_Err");
    Prot_Info.Err_Flag.V_H_HW_Err == true ? strcpy(Prot_Info_Txt.Err_Flag_txt.V_H_HW_Err_txt, "Err") : strcpy(Prot_Info_Txt.Err_Flag_txt.V_H_HW_Err_txt, "No_Err");
    Prot_Info.Err_Flag.V_H_SW_Err == true ? strcpy(Prot_Info_Txt.Err_Flag_txt.V_H_SW_Err_txt, "Err") : strcpy(Prot_Info_Txt.Err_Flag_txt.V_H_SW_Err_txt, "No_Err");
    Prot_Info.Err_Flag.V_L_HW_Err == true ? strcpy(Prot_Info_Txt.Err_Flag_txt.V_L_HW_Err_txt, "Err") : strcpy(Prot_Info_Txt.Err_Flag_txt.V_L_HW_Err_txt, "No_Err");
    Prot_Info.Err_Flag.V_L_SW_Err == true ? strcpy(Prot_Info_Txt.Err_Flag_txt.V_L_SW_Err_txt, "Err") : strcpy(Prot_Info_Txt.Err_Flag_txt.V_L_SW_Err_txt, "No_Err");

    // Convert control system parameters to formatted strings
    my_sprintf(Ctrl_Txt.In_Loop_Ki_txt, "%f", Ctrl.In_Loop_Ki);   // Inner loop integral gain
    my_sprintf(Ctrl_Txt.In_Loop_Kp_txt, "%f", Ctrl.In_Loop_Kp);   // Inner loop proportional gain
    my_sprintf(Ctrl_Txt.Out_Loop_Ki_txt, "%f", Ctrl.Out_Loop_Ki); // Outer loop integral gain
    my_sprintf(Ctrl_Txt.Out_Loop_Kp_txt, "%f", Ctrl.Out_Loop_Kp); // Outer loop proportional gain
    my_sprintf(Ctrl_Txt.Out_Loop_Ref_txt, "%f", Ctrl.Out_Loop_Ref); // Outer loop reference value
    my_sprintf(Ctrl_Txt.V_INV_Ref_txt, "%f", Ctrl.V_INV_Ref);     // Inverter voltage reference
    my_sprintf(Ctrl_Txt.In_Loop_Ref_txt, "%f", Ctrl.In_Loop_Ref); // Inner loop reference value

    // Convert boolean enable flags to "Enable" or "Disable" strings
    Ctrl.Feed_Ford_Enable == true ? strcpy(Ctrl_Txt.Feed_Ford_Enable_txt, "Enable") : strcpy(Ctrl_Txt.Feed_Ford_Enable_txt, "Disable");
    Ctrl.Dead_Time_Comp_Enable == true ? strcpy(Ctrl_Txt.Dead_Time_Comp_Enable_txt, "Enable") : strcpy(Ctrl_Txt.Dead_Time_Comp_Enable_txt, "Disable");

    // Convert control loop mode enum to corresponding string
    if(Ctrl.Ctrl_Loop==OPEN_LOOP)
    {
    strcpy(Ctrl_Txt.Ctrl_Loop_txt, "OPEN_LOOP");               // Open-loop control mode
    }
    else if(Ctrl.Ctrl_Loop==CLOSE_IN_LOOP)
    {
    strcpy(Ctrl_Txt.Ctrl_Loop_txt, "CLOSE_IN_LOOP");           // Inner loop closed control mode
    }
    else
    {
    strcpy(Ctrl_Txt.Ctrl_Loop_txt, "CLOSE_IN_OUT_LOOP");       // Both inner/outer loops closed mode
    }
}

void OLED_Page_Slect(void)
{
    switch(Oled_Page)
    {
        case MAIN_PAGE:
        {
            Oled_Page = POWER_PAGE;
            break;
        }
        case POWER_PAGE:
        {
            Oled_Page = PROTET_PAGE;
            break;
        }
        case PROTET_PAGE:
        {
            Oled_Page = CTRL_PAGE;
            break;
        }
        case CTRL_PAGE:
        {
            Oled_Page = MAIN_PAGE;
            break;
        }
    }
}

void OLED_Data_Show(void)
{

        DELAY_US(500000);
        LED_GPIO_99_TOGGLE;

        if( OLED_Clear() == I2C_ERROR )
        {
            LED_GPIO_133_ON;
        }

        if( Prot_Info.Tim_Status == RUNNING )
        {
            LED_GPIO_94_ON;
        }
        else
        {
            LED_GPIO_94_OFF;
        }

        switch(Oled_Page)
        {
            case MAIN_PAGE:
            {
                if(OLED_ShowString(0,2,"K29:Rst",16)                                    ==   I2C_ERROR    ||
                   OLED_ShowString(0,0,"K36:Run",16)                                    ==   I2C_ERROR    ||
                   OLED_ShowString(60,0,"K28:Stop",16)                                  ==   I2C_ERROR    ||
                   OLED_ShowString(60,2,"K30:View",16)                                  ==   I2C_ERROR    ||
                   OLED_ShowString(0,5,(unsigned char *)&Dev_Info_Txt.Dev_Info,16)      ==   I2C_ERROR
                   )
                {
                    LED_GPIO_133_ON;
                }
                break;
            }
            case POWER_PAGE:
            {
                if(OLED_ShowString(10,2,"VH:",16)                                              ==   I2C_ERROR    ||
                   OLED_ShowString(70,2,(unsigned char *)&Power_Info_Txt.V_H_txt,16)           ==   I2C_ERROR    ||
                   OLED_ShowString(10,4,"VL:",16)                                              ==   I2C_ERROR    ||
                   OLED_ShowString(70,4,(unsigned char *)&Power_Info_Txt.V_L_txt,16)           ==   I2C_ERROR    ||
                   OLED_ShowString(10,6,"I_INV:",16)                                           ==   I2C_ERROR    ||
                   OLED_ShowString(70,6,(unsigned char *)&Power_Info_Txt.I_Inv_txt,16)         ==   I2C_ERROR    ||
                   OLED_ShowString(10,0,"State:",16)                                           ==   I2C_ERROR    ||
                   OLED_ShowString(70,0,(unsigned char *)&Prot_Info_Txt.Tim_Status_txt,16)     ==   I2C_ERROR
                   )
                {
                    LED_GPIO_133_ON;
                }
                break;
            }
            case PROTET_PAGE:
            {
                if(OLED_ShowString(5,0,"PROTET_ERR_PAGE",16)                                    ==   I2C_ERROR)
                {
                    LED_GPIO_133_ON;
                }

                if( Prot_Info.Err_Flag.I_INV_HW_Err==true || Prot_Info.Err_Flag.I_INV_SW_Err==true )
                {
                    OLED_ShowString(10,2,"I_INV:  ERR",16);
                }
                else
                {
                    OLED_ShowString(10,2,"I_INV:  NO_ERR",16);
                }

                if( Prot_Info.Err_Flag.V_H_HW_Err==true || Prot_Info.Err_Flag.V_H_SW_Err==true )
                {
                    OLED_ShowString(10,4,"VH:     ERR",16);
                }
                else
                {
                    OLED_ShowString(10,4,"VH:     NO_ERR",16);
                }

                if( Prot_Info.Err_Flag.V_L_HW_Err==true || Prot_Info.Err_Flag.V_L_SW_Err==true )
                {
                    OLED_ShowString(10,6,"VL:     ERR",16);
                }
                else
                {
                    OLED_ShowString(10,6,"VL:     NO_ERR",16);
                }

                break;
            }
            case CTRL_PAGE:
            {
                if(OLED_ShowString(10,2,"In_Ref:",16)                                                    ==   I2C_ERROR    ||
                   OLED_ShowString(70,2,(unsigned char *)&Ctrl_Txt.In_Loop_Ref_txt,16)                   ==   I2C_ERROR    ||
                   OLED_ShowString(10,4,"Ford:",16)                                                      ==   I2C_ERROR    ||
                   OLED_ShowString(70,4,(unsigned char *)&Ctrl_Txt.Feed_Ford_Enable_txt,16)              ==   I2C_ERROR    ||
                   OLED_ShowString(10,6,"Dead:",16)                                                      ==   I2C_ERROR    ||
                   OLED_ShowString(70,6,(unsigned char *)&Ctrl_Txt.Dead_Time_Comp_Enable_txt,16)         ==   I2C_ERROR    ||
                   OLED_ShowString(10,0,(unsigned char *)&Ctrl_Txt.Ctrl_Loop_txt,16)                     ==   I2C_ERROR
                   )
                {
                    LED_GPIO_133_ON;
                }
                break;
            }
        }
}

/**
 * Convert an integer to its string representation
 * @param num    The integer to convert
 * @param buffer The buffer to store the resulting string
 * @param pos    Pointer to the current position in the buffer (updated during conversion)
 */
void int_to_str(int num, char* buffer, int* pos) {
    char temp[20]; // Temporary buffer to store reversed digits
    int i = 0;
    int is_negative = 0;

    // Check if the number is negative
    if (num < 0) {
        is_negative = 1;
        num = -num; // Convert to positive for digit extraction
    }

    // Handle the case where the number is zero
    if (num == 0) {
        temp[i++] = '0';
    }

    // Extract digits in reverse order
    while (num > 0) {
        temp[i++] = '0' + (num % 10); // Get last digit and convert to character
        num /= 10; // Remove the last digit
    }

    // Add negative sign if needed
    if (is_negative) {
        temp[i++] = '-';
    }

    // Reverse the temporary buffer to get the correct order and store in output buffer
    while (i > 0) {
        buffer[(*pos)++] = temp[--i];
    }
}

/**
 * Convert a float to its string representation (with 3 decimal places)
 * @param num    The float to convert
 * @param buffer The buffer to store the resulting string
 * @param pos    Pointer to the current position in the buffer (updated during conversion)
 */
void float_to_str(float num, char* buffer, int* pos) {
    int is_negative = 0; // Track if the original float is negative
    int start_pos = *pos; // Save buffer position before writing integer part (for sign insertion)
    int i = 0;
    // Step 1: Check overall sign of the float (fixes -0.5 issue)
    if (num < 0 && num != 0) { // Avoid "negative zero" edge case
        is_negative = 1;
        num = -num; // Convert to positive for uniform processing
    }

    // Step 2: Extract and convert integer part (now uses positive num)
    int int_part = (int)num;
    int_to_str(int_part, buffer, pos);

    // Step 3: Add decimal point
    buffer[(*pos)++] = '.';

    // Step 4: Extract and convert fractional part (already positive)
    float frac_part = num - int_part;
    for (i = 0; i < 3; i++) {
        frac_part *= 10; // Shift decimal point right
        int digit = (int)frac_part; // Get current digit
        buffer[(*pos)++] = '0' + digit; // Convert digit to character
        frac_part -= digit; // Remove the extracted digit
    }

    // Step 5: Insert negative sign if original float was negative
    if (is_negative) {
        // Shift all characters (integer + decimal + digits) right to make space for '-'
        for (i = *pos; i > start_pos; i--) {
            buffer[i] = buffer[i - 1];
        }
        buffer[start_pos] = '-'; // Add negative sign at the start
        (*pos)++; // Update position to account for the new '-'
    }
}

/**
 * Custom implementation of sprintf (supports %d and %f formats)
 * @param buffer The buffer to store the formatted string
 * @param format The format string specifying the output format
 * @param ...    Variable arguments to be formatted
 */
void my_sprintf(char* buffer, const char* format, ...) {
    va_list args;
    va_start(args, format); // Initialize variable argument list

    int pos = 0; // Current position in the output buffer

    while (*format) {
        if (*format == '%') {
            format++; // Move past the '%'

            // Handle different format specifiers
            switch (*format) {
                case 'd': { // Integer format
                    int num = va_arg(args, int);
                    int_to_str(num, buffer, &pos);
                    break;
                }
                case 'f': { // Float format (promoted to double in variadic functions)
                    float num = (float)va_arg(args, double);
                    float_to_str(num, buffer, &pos);
                    break;
                }
                default: { // Handle unsupported format specifiers
                    buffer[pos++] = '%';
                    buffer[pos++] = *format;
                    break;
                }
            }
            format++;
        } else {
            // Copy regular characters directly to buffer
            buffer[pos++] = *format++;
        }
    }

    // Null-terminate the string
    buffer[pos] = '\0';

    va_end(args); // Clean up variable argument list
}






