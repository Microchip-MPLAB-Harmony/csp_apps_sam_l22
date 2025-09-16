/*******************************************************************************
  SERCOM Universal Synchronous/Asynchrnous Receiver/Transmitter PLIB

  Company
    Microchip Technology Inc.

  File Name
    plib_sercom3_usart.c

  Summary
    USART peripheral library interface.

  Description
    This file defines the interface to the USART peripheral library. This
    library provides access to and control of the associated peripheral
    instance.

  Remarks:
    None.
*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "interrupts.h"
#include "definitions.h"
#include "plib_sercom3_usart.h"
#include "peripheral/port/plib_port.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************
#define OUTPUT_CLOCK    3571200U

#define USART_SEND      0U
#define USART_RCV       1U

/** Case for APDU commands. */
#define CASE1           1U
#define CASE2           2U
#define CASE3           3U

/** NULL byte to restart byte procedure. */
#define ISO_NULL_VAL    0x60U

static uint8_t usart_state = USART_RCV;


// *****************************************************************************
// *****************************************************************************
// Section: SERCOM3 USART Interface Routines
// *****************************************************************************
// *****************************************************************************

static void SERCOM3_USART_ErrorClear( void )
{
    uint8_t  u8dummyData = 0U;
    USART_ERROR errorStatus = (USART_ERROR) (SERCOM3_REGS->USART_INT.SERCOM_STATUS & (uint16_t)(SERCOM_USART_INT_STATUS_PERR_Msk | SERCOM_USART_INT_STATUS_FERR_Msk | SERCOM_USART_INT_STATUS_BUFOVF_Msk ));

    if(errorStatus != USART_ERROR_NONE)
    {
        /* Clear error flag */
        SERCOM3_REGS->USART_INT.SERCOM_INTFLAG = (uint8_t)SERCOM_USART_INT_INTFLAG_ERROR_Msk;
        /* Clear all errors */
        SERCOM3_REGS->USART_INT.SERCOM_STATUS = (uint16_t)(SERCOM_USART_INT_STATUS_PERR_Msk | SERCOM_USART_INT_STATUS_FERR_Msk | SERCOM_USART_INT_STATUS_BUFOVF_Msk);

        /* Flush existing error bytes from the RX FIFO */
        while((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & (uint8_t)SERCOM_USART_INT_INTFLAG_RXC_Msk) == (uint8_t)SERCOM_USART_INT_INTFLAG_RXC_Msk)
        {
            u8dummyData = (uint8_t)SERCOM3_REGS->USART_INT.SERCOM_DATA;
        }
    }

    /* Ignore the warning */
    (void)u8dummyData;
}

void SERCOM3_USART_Initialize( void )
{
    /*
     * Configures USART Clock Mode
     * Configures TXPO and RXPO
     * Configures Data Order
     * Configures Standby Mode
     * Configures Sampling rate
     * Configures IBON
     */

    SERCOM3_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK | SERCOM_USART_INT_CTRLA_CMODE(1U) | SERCOM_USART_INT_CTRLA_RXPO(0x2UL) | SERCOM_USART_INT_CTRLA_TXPO(0x1UL) | SERCOM_USART_INT_CTRLA_DORD_Msk | SERCOM_USART_INT_CTRLA_IBON_Msk | SERCOM_USART_INT_CTRLA_FORM(0x7UL) | SERCOM_USART_INT_CTRLA_SAMPR(0UL) | SERCOM_USART_INT_CTRLA_RUNSTDBY_Msk;

    /* Configure Baud Rate */
    SERCOM3_REGS->USART_INT.SERCOM_BAUD = 185;

    /*
     * Configures RXEN
     * Configures TXEN
     * Configures CHSIZE
     * Configures Parity
     * Configures Stop bits
     */
    SERCOM3_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE_8_BIT | SERCOM_USART_INT_CTRLB_SBMODE_1_BIT | SERCOM_USART_INT_CTRLB_RXEN_Msk | SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
    while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }

    /* Configures IEC7816 Guard Time and Maxiter values*/
    SERCOM3_REGS->USART_INT.SERCOM_CTRLC = SERCOM_USART_INT_CTRLC_GTIME(2U) | SERCOM_USART_INT_CTRLC_MAXITER(7U) ;

    /* Enable the UART after the configurations */
    SERCOM3_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;

    /* Wait for sync */
    while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
}





uint32_t SERCOM3_USART_FrequencyGet( void )
{
    return 3555555UL;
}

bool SERCOM3_USART_SerialSetup( USART_SERIAL_SETUP * serialSetup, uint32_t clkFrequency )
{
    bool setupStatus       = false;
    uint32_t baudValue     = 0U;
    uint32_t sampleRate    = 0U;
    uint32_t sampleCount   = 0U;

    if((serialSetup != NULL) && (serialSetup->baudRate != 0U))
    {
        if(clkFrequency == 0U)
        {
            clkFrequency = SERCOM3_USART_FrequencyGet();
        }

        if(clkFrequency >= (16U * serialSetup->baudRate))
        {
            sampleRate = 0U;
            sampleCount = 16U;
        }
        else if(clkFrequency >= (8U * serialSetup->baudRate))
        {
            sampleRate = 2U;
            sampleCount = 8U;
        }
        else if(clkFrequency >= (3U * serialSetup->baudRate))
        {
            sampleRate = 4U;
            sampleCount = 3U;
        }
        else
        {
            /* Do nothing */
        }
        baudValue = 65536U - (uint32_t)(((uint64_t)65536U * sampleCount * serialSetup->baudRate) / clkFrequency);
        /* Disable the USART before configurations */
        SERCOM3_REGS->USART_INT.SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE_Msk;

        /* Wait for sync */
        while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }

        /* Configure Baud Rate */
        SERCOM3_REGS->USART_INT.SERCOM_BAUD = (uint16_t)SERCOM_USART_INT_BAUD_BAUD(baudValue);

        /* Configure Parity Options */
        if(serialSetup->parity == USART_PARITY_NONE)
        {
            SERCOM3_REGS->USART_INT.SERCOM_CTRLA =  (SERCOM3_REGS->USART_INT.SERCOM_CTRLA & ~(SERCOM_USART_INT_CTRLA_SAMPR_Msk | SERCOM_USART_INT_CTRLA_FORM_Msk)) | SERCOM_USART_INT_CTRLA_FORM(0x0UL) | SERCOM_USART_INT_CTRLA_SAMPR((uint32_t)sampleRate); 
            SERCOM3_REGS->USART_INT.SERCOM_CTRLB = (SERCOM3_REGS->USART_INT.SERCOM_CTRLB & ~(SERCOM_USART_INT_CTRLB_CHSIZE_Msk | SERCOM_USART_INT_CTRLB_SBMODE_Msk)) | ((uint32_t) serialSetup->dataWidth | (uint32_t) serialSetup->stopBits);
        }
        else
        {
            SERCOM3_REGS->USART_INT.SERCOM_CTRLA =  (SERCOM3_REGS->USART_INT.SERCOM_CTRLA & ~(SERCOM_USART_INT_CTRLA_SAMPR_Msk | SERCOM_USART_INT_CTRLA_FORM_Msk)) | SERCOM_USART_INT_CTRLA_FORM(0x1UL) | SERCOM_USART_INT_CTRLA_SAMPR((uint32_t)sampleRate); 
            SERCOM3_REGS->USART_INT.SERCOM_CTRLB = (SERCOM3_REGS->USART_INT.SERCOM_CTRLB & ~(SERCOM_USART_INT_CTRLB_CHSIZE_Msk | SERCOM_USART_INT_CTRLB_SBMODE_Msk | SERCOM_USART_INT_CTRLB_PMODE_Msk)) | (uint32_t) serialSetup->dataWidth | (uint32_t) serialSetup->stopBits | (uint32_t) serialSetup->parity ;
        }

        /* Wait for sync */
        while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }

        /* Enable the USART after the configurations */
        SERCOM3_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;

        /* Wait for sync */
        while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }

        setupStatus = true;
    }

    return setupStatus;
}

USART_ERROR SERCOM3_USART_ErrorGet( void )
{
    USART_ERROR errorStatus = (USART_ERROR) (SERCOM3_REGS->USART_INT.SERCOM_STATUS & (uint16_t)(SERCOM_USART_INT_STATUS_PERR_Msk | SERCOM_USART_INT_STATUS_FERR_Msk | SERCOM_USART_INT_STATUS_BUFOVF_Msk ));

    if(errorStatus != USART_ERROR_NONE)
    {
        SERCOM3_USART_ErrorClear();
    }

    return errorStatus;
}

void SERCOM3_USART_Enable( void )
{
    if((SERCOM3_REGS->USART_INT.SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_ENABLE_Msk) == 0U)
    {
        SERCOM3_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;

        /* Wait for sync */
        while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }
    }
}

void SERCOM3_USART_Disable( void )
{
    if((SERCOM3_REGS->USART_INT.SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_ENABLE_Msk) != 0U)
    {
        SERCOM3_REGS->USART_INT.SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE_Msk;

        /* Wait for sync */
        while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }
    }
}


void SERCOM3_USART_TransmitterEnable( void )
{
    SERCOM3_REGS->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
    while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
}

void SERCOM3_USART_TransmitterDisable( void )
{
    SERCOM3_REGS->USART_INT.SERCOM_CTRLB &= ~SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
    while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
}

bool SERCOM3_USART_Write( void *buffer, const size_t size )
{
    bool writeStatus      = false;
    uint8_t *pu8Data      = (uint8_t*)buffer;
    uint16_t *pu16Data    = (uint16_t*)buffer;
    uint32_t u32Index     = 0U;

    if(buffer != NULL)
    {
        /* Blocks while buffer is being transferred */
        while(u32Index < size)
        {
            /* Check if USART is ready for new data */
            while((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & (uint8_t)SERCOM_USART_INT_INTFLAG_DRE_Msk) == 0U)
            {
                /* Do nothing */
            }

            /* Write data to USART module */
            if (((SERCOM3_REGS->USART_INT.SERCOM_CTRLB & SERCOM_USART_INT_CTRLB_CHSIZE_Msk) >> SERCOM_USART_INT_CTRLB_CHSIZE_Pos) != 0x01U)
            {
                /* 8-bit mode */
                SERCOM3_REGS->USART_INT.SERCOM_DATA = pu8Data[u32Index];
            }
            else
            {
                /* 9-bit mode */
                SERCOM3_REGS->USART_INT.SERCOM_DATA = pu16Data[u32Index];
            }

            /* Increment index */
            u32Index++;
        }
        writeStatus = true;
    }

    return writeStatus;
}


bool SERCOM3_USART_TransmitterIsReady( void )
{
    bool transmitterStatus = false;

    if ((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk) == SERCOM_USART_INT_INTFLAG_DRE_Msk)
    {
        transmitterStatus = true;
    }

    return transmitterStatus;
}

void SERCOM3_USART_WriteByte( int data )
{
    /* Check if USART is ready for new data */
    while((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk) == 0U)
    {
        /* Do nothing */
    }

    SERCOM3_REGS->USART_INT.SERCOM_DATA = (uint16_t)data;
}

bool SERCOM3_USART_TransmitComplete( void )
{
    bool transmitComplete = false;

    if ((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC_Msk) == SERCOM_USART_INT_INTFLAG_TXC_Msk)
    {
        transmitComplete = true;
    }

    return transmitComplete;
}

void SERCOM3_USART_ReceiverEnable( void )
{
    SERCOM3_REGS->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_RXEN_Msk;

    /* Wait for sync */
    while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
}

void SERCOM3_USART_ReceiverDisable( void )
{
    SERCOM3_REGS->USART_INT.SERCOM_CTRLB &= ~SERCOM_USART_INT_CTRLB_RXEN_Msk;

    /* Wait for sync */
    while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
}

bool SERCOM3_USART_Read( void *buffer, const size_t size )
{
    bool readStatus         = false;
    uint8_t* pu8Data        = (uint8_t*)buffer;
    uint16_t *pu16Data      = (uint16_t*)buffer;
    uint32_t u32Index       = 0U;
    USART_ERROR errorStatus = USART_ERROR_NONE;

    if(buffer != NULL)
    {

        /* Clear error flags and flush out error data that may have been received when no active request was pending */
        SERCOM3_USART_ErrorClear();

        while(u32Index < size)
        {
            /* Check if USART has new data */
            while((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) == 0U)
            {
                /* Do nothing */
            }

            errorStatus = (USART_ERROR) (SERCOM3_REGS->USART_INT.SERCOM_STATUS & (uint16_t)(SERCOM_USART_INT_STATUS_PERR_Msk | SERCOM_USART_INT_STATUS_FERR_Msk | SERCOM_USART_INT_STATUS_BUFOVF_Msk));

            if(errorStatus != USART_ERROR_NONE)
            {
                break;
            }

            if (((SERCOM3_REGS->USART_INT.SERCOM_CTRLB & SERCOM_USART_INT_CTRLB_CHSIZE_Msk) >> SERCOM_USART_INT_CTRLB_CHSIZE_Pos) != 0x01U)
            {
                /* 8-bit mode */
                pu8Data[u32Index] = (uint8_t)SERCOM3_REGS->USART_INT.SERCOM_DATA;
            }
            else
            {
                /* 9-bit mode */
                pu16Data[u32Index] = (uint16_t)SERCOM3_REGS->USART_INT.SERCOM_DATA;
            }

            /* Increment index */
            u32Index++;
        }

        if(size == u32Index)
        {
            readStatus = true;
        }
    }

    return readStatus;
}

bool SERCOM3_USART_ReceiverIsReady( void )
{
    bool receiverStatus = false;

    if ((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) == SERCOM_USART_INT_INTFLAG_RXC_Msk)
    {
        receiverStatus = true;
    }

    return receiverStatus;
}

int SERCOM3_USART_ReadByte( void )
{
    return (int)SERCOM3_REGS->USART_INT.SERCOM_DATA;
}





void SERCOM3_ISO7816_Icc_Power_On( void )
{
    PORT_PinWrite(PORT_PIN_PB18, true);
}

void SERCOM3_ISO7816_Icc_Power_Off( void )
{
    PORT_PinWrite(PORT_PIN_PB18, false);
}

bool SERCOM3_ISO7816_Card_Detect(void)
{
    if(PORT_PinRead(PORT_PIN_PC16) == true){
        return true;
    }else{
        return false;
    }
}

void SERCOM3_ISO7816_Vcc_Enable( void )
{
    PORT_PinWrite(PORT_PIN_PB19, true);
}

void SERCOM3_ISO7816_Vcc_Disable( void )
{
    PORT_PinWrite(PORT_PIN_PB19, false);
}

static uint32_t Receive_Timeoutcount(void)
{
    return ((CPU_CLOCK_FREQUENCY/OUTPUT_CLOCK)*40000U);
}

static uint32_t Reset_Waitcount(void)
{
    return ((CPU_CLOCK_FREQUENCY/OUTPUT_CLOCK)*400U);
}

void SERCOM3_ISO7816_Cold_Reset(void)
{
    uint32_t i, rst_wait_time;

    rst_wait_time = Reset_Waitcount();

    /* tb: wait 400 cycles */
    for (i = 0; i < rst_wait_time; i++) {
    }

    //Read all the leftover data from card
    while(SERCOM3_USART_ReadByte() != 0){
    }

    //usart_reset_status(ISO7816_USART);

    /*ISO7816 reset iterations*/
    if((SERCOM3_REGS->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_ITER_Msk) != 0U)
    {
        SERCOM3_REGS->USART_INT.SERCOM_STATUS |= SERCOM_USART_INT_STATUS_ITER_Msk;
    }

    //Enable Reset pin to high
    SERCOM3_ISO7816_Icc_Power_On();
}

void SERCOM3_ISO7816_Warm_Reset(void)
{
    uint32_t count, rst_wait_time;

    rst_wait_time = Reset_Waitcount();

    //Enable Reset pin to high
    SERCOM3_ISO7816_Icc_Power_Off();

    /* tb: wait 400 cycles */
    for (count = 0; count < rst_wait_time; count++) {
    }

    //Read all the leftover data from card
    while(SERCOM3_USART_ReadByte() != 0){
    }

    //usart_reset_status(ISO7816_USART);

    /*ISO7816 reset iterations*/
    if((SERCOM3_REGS->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_ITER_Msk) != 0U)
    {
        SERCOM3_REGS->USART_INT.SERCOM_STATUS |= SERCOM_USART_INT_STATUS_ITER_Msk;
    }

    //Enable Reset pin to high
    SERCOM3_ISO7816_Icc_Power_On();
}

void SERCOM3_ISO7816_Decode_Atr(uint8_t *p_atr)
{
    uint32_t index;
    uint8_t j, y, HB_count, uc_offset;

    index = 2;

    /*Interface Bytes*/
    y = p_atr[1] & 0xF0U;

    /* Read ATR Ti. */
    uc_offset = 1;
    while (y != 0U) {
        if ((y & 0x10U) == 0X10U) { /* TA[i] */
            index++;
        }
        if ((y & 0x20U) == 0X20U){ /* TB[i] */
            index++;
        }
        if ((y & 0x40U) == 0X40U) { /* TC[i] */
            index++;
        }
        if ((y & 0x80U) == 0X80U) { /* TD[i] */
            y = p_atr[index++] & 0xF0U;
        } else {
            y = 0;
        }
        uc_offset++;
    }

    /*Historical Bytes*/
    HB_count = p_atr[1] & 0x0FU;
    for (j = 0; j < HB_count; j++) {
        index++;
    }
}

static uint8_t SERCOM3_ISO7816_Get_Char(uint8_t *p_char_received)
{
    uint32_t timeout = 0, rx_timeout;

    rx_timeout = Receive_Timeoutcount();

    if (usart_state == USART_SEND)
    {
        while (SERCOM3_USART_TransmitComplete() != true){
        }

        SERCOM3_USART_TransmitterDisable();

        /*ISO7816 reset iterations*/
        if((SERCOM3_REGS->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_ITER_Msk) != 0U)
        {
            SERCOM3_REGS->USART_INT.SERCOM_STATUS |= SERCOM_USART_INT_STATUS_ITER_Msk;
        }

        SERCOM3_USART_ReceiverEnable();

        usart_state = USART_RCV;
    }

    while((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) == 0U)
    {
        if (timeout++ > rx_timeout) {
            return (0);
        }
    }

    *p_char_received = (uint8_t)SERCOM3_REGS->USART_INT.SERCOM_DATA;

    return (1);

}

static void SERCOM3_ISO7816_Send_Char(uint8_t uc_char)
{

    if (usart_state == USART_RCV)
    {
        while((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) != 0U)
        {
            (void)SERCOM3_USART_ReadByte();
        }

        /*ISO7816 reset iterations*/
        if((SERCOM3_REGS->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_ITER_Msk) != 0U)
        {
            SERCOM3_REGS->USART_INT.SERCOM_STATUS |= SERCOM_USART_INT_STATUS_ITER_Msk;
        }

        SERCOM3_USART_TransmitterEnable();
        usart_state = USART_SEND;
    }

    while((SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & (uint8_t)SERCOM_USART_INT_INTFLAG_DRE_Msk) == 0U)
    {
        /* Do nothing */
    }

    SERCOM3_REGS->USART_INT.SERCOM_DATA = uc_char;

}

uint8_t SERCOM3_ISO7816_Data_Read_Atr( uint8_t *p_atr )
{
    uint8_t j, response_length, uc_value;
    uint8_t status;

    if((SERCOM3_REGS->USART_INT.SERCOM_CTRLB & SERCOM_USART_INT_CTRLB_TXEN_Msk) == SERCOM_USART_INT_CTRLB_TXEN_Msk)
    {
        SERCOM3_REGS->USART_INT.SERCOM_CTRLB &= ~SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
        while((SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }

        usart_state = USART_RCV;
    }

    /* Read ATR TS. */
    (void)SERCOM3_ISO7816_Get_Char(&p_atr[0]);

    /* Read ATR T0. */
    status = SERCOM3_ISO7816_Get_Char(&p_atr[1]);
    if (status == 0U)
    {
            return 0;
    }

    uc_value = p_atr[1] & 0xF0U;
    response_length = 2;

    /* Read ATR Ti. */
    while (uc_value != 0U) {
        if ((uc_value & 0x10U) == 0x10U) { /* TA[response_length] */
            status = SERCOM3_ISO7816_Get_Char(&p_atr[response_length++]);
            if (status == 0U)
            {
                    return 0;
            }
        }

        if ((uc_value & 0x20U) == 0x20U) { /* TB[response_length] */
            status = SERCOM3_ISO7816_Get_Char(&p_atr[response_length++]);
            if (status == 0U)
            {
                    return 0;
            }
        }

        if ((uc_value & 0x40U) == 0x40U) { /* TC[response_length] */
            status = SERCOM3_ISO7816_Get_Char(&p_atr[response_length++]);
            if (status == 0U)
            {
                    return 0;
            }
        }

        if ((uc_value & 0x80U) == 0X80U) { /* TD[response_length] */
            status = SERCOM3_ISO7816_Get_Char(&p_atr[response_length]);
            if (status == 0U)
            {
                    return 0;
            }

            uc_value = p_atr[response_length++] & 0xF0U;
        } else {
            uc_value = 0;
        }
    }

    /* Historical Bytes. */
    uc_value = p_atr[1] & 0x0FU;
    for (j = 0; j < uc_value; j++) {
        status = SERCOM3_ISO7816_Get_Char(&p_atr[response_length++]);
        if (status == 0U)
        {
                return 0;
        }
    }

    return (response_length);
}

uint16_t SERCOM3_ISO7816_Xfr_Block_Tpdu( uint8_t *apdu_cmd_buffer, uint8_t *apdu_res_buffer, const size_t apdu_cmd_length )
{
    uint16_t us_ne_nc, cmd_index = 4;
    uint16_t resp_index = 0;
    uint8_t sw1_rcvd = 0, cmd_type, status;
    uint8_t proc_byte, dummy_byte=0;

    SERCOM3_ISO7816_Send_Char(apdu_cmd_buffer[0]);    /* CLA */
    SERCOM3_ISO7816_Send_Char(apdu_cmd_buffer[1]);    /* INS */
    SERCOM3_ISO7816_Send_Char(apdu_cmd_buffer[2]);    /* P1 */
    SERCOM3_ISO7816_Send_Char(apdu_cmd_buffer[3]);    /* P2 */
    SERCOM3_ISO7816_Send_Char(apdu_cmd_buffer[4]);    /* P3 */

    /* Handle the four structures of command APDU. */
    switch (apdu_cmd_length)
    {
        case CMD_LEN_4:

            cmd_type = CASE1;
            us_ne_nc = 0;

            break;

        case CMD_LEN_5:

            cmd_type = CASE2;
            us_ne_nc = apdu_cmd_buffer[4];                                                              /* C5, only Standard Le */
            if (us_ne_nc == 0U)
            {
                us_ne_nc = 256;
            }
            break;

        case CMD_LEN_6:

            us_ne_nc = apdu_cmd_buffer[4];                                                              /* C5, only Standard Lc */
            cmd_type = CASE3;

            break;

        case CMD_LEN_7:

            us_ne_nc = apdu_cmd_buffer[4];                                                              /* C5 */
            if (us_ne_nc == 0U)
            {
                cmd_type = CASE2;
                us_ne_nc = ((uint16_t)apdu_cmd_buffer[5] << 8) + (uint16_t)apdu_cmd_buffer[6];          /*Extended Le */
            }
            else
            {
                cmd_type = CASE3;                                                                       /*Standard Lc*/
            }
            break;

        default:

            us_ne_nc = apdu_cmd_buffer[4];                                                              /* C5 */

            if (us_ne_nc == 0U)
            {
                cmd_type = CASE3;
                us_ne_nc = ((uint16_t)apdu_cmd_buffer[5] << 8) + (uint16_t)apdu_cmd_buffer[6];          /*Extended Lc */
            }
            else
            {
                cmd_type = CASE3;                                                                       /*Standard Lc*/
            }

            break;
    }

    /* Handle Procedure Bytes. */
    do{
        /* Dummy Read - Start */
        (void)SERCOM3_ISO7816_Get_Char(&dummy_byte);
        (void)SERCOM3_ISO7816_Get_Char(&dummy_byte);
        (void)SERCOM3_ISO7816_Get_Char(&dummy_byte);
        /* Dummy Read - End */

        status = SERCOM3_ISO7816_Get_Char(&proc_byte);
        if(status == 0U)
        {
            return 0;
        }

        /* Handle NULL. */
        if (ISO_NULL_VAL == proc_byte) {
            continue;
        }
        /* Handle sw1. */
        else if (((proc_byte & 0xF0U) == 0x60U) || ((proc_byte & 0xF0U) == 0x90U))
        {
            sw1_rcvd = 1;
        }
        /* Handle INS. */
        else if (apdu_cmd_buffer[1] == proc_byte)
        {
            if (cmd_type == CASE2)
            {
                /* Receive data from card. */
                do {
                    status = SERCOM3_ISO7816_Get_Char(&apdu_res_buffer[resp_index]);
                    resp_index++;
                } while (0U != --us_ne_nc);
            }
            else
            {
                /* Send data. */
                do {
                    cmd_index++;
                    SERCOM3_ISO7816_Send_Char(apdu_cmd_buffer[cmd_index]);
                } while (0U != --us_ne_nc);

                /* Dummy Read - Start */
                status = SERCOM3_ISO7816_Get_Char(&dummy_byte);
                status = SERCOM3_ISO7816_Get_Char(&dummy_byte);
                /* Dummy Read - End */
            }
        }
        /* Handle INS ^ 0xff. */
        else if ((apdu_cmd_buffer[1] ^ 0xffU) == proc_byte)
        {
            if (cmd_type == CASE2)
            {
                /* receive data from card. */
                status = SERCOM3_ISO7816_Get_Char(&apdu_res_buffer[resp_index]);
                resp_index++;
            }
            else
            {
                SERCOM3_ISO7816_Send_Char(apdu_cmd_buffer[cmd_index]);
                cmd_index++;
            }
            us_ne_nc--;
        }
        else
        {
            break;
        }
    } while (us_ne_nc != 0U);

    /* Status Bytes. */
    if (sw1_rcvd == 0U)
    {
        (void)SERCOM3_ISO7816_Get_Char(&apdu_res_buffer[resp_index]);                 /* sw1_rcvd */
        resp_index++;
    }
    else
    {
        apdu_res_buffer[resp_index] = proc_byte;
        resp_index++;
    }
    (void)SERCOM3_ISO7816_Get_Char(&apdu_res_buffer[resp_index]);                     /* SW2 */

    resp_index++;

    return (resp_index);

}


