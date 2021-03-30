#include <CC1200.h>
///////////////////////////////////////////////////////////////////////////////////////////
/****************************************flags*********************************************/
int timerflag=0;
uint16 i=0;
uint32 c1=0;
uint8 flag=0;
int adcflag=7;
int gpio2flag=0;
uint8 readValue;
/***********************************Sensors' variables****************************************/
uint16 sensor1=0;
uint16 sensor2=0;
uint16 sensor3=0;
uint16 sensor4=0;
uint16 sensor5=0;
/********************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);
static void registerConfig(void);
rfStatus_t trxSpiCmdStrobe(uint8 cmd);
rfStatus_t cc120xSpiWriteTxFifo(uint8 *pData, uint8 len);
rfStatus_t trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len);
rfStatus_t cc120xSpiWriteReg(uint16 addr, uint8 *pData, uint8 len);
rfStatus_t cc120xSpiReadRxFifo(uint8 *pData, uint8 len);
rfStatus_t cc120xSpiWriteReg(uint16 addr, uint8 *pData, uint8 len);
rfStatus_t cc120xSpiReadReg(uint16 addr, uint8 *pData, uint8 len);
static void createPacket(uint8 txBuffer[]);
void initADC(void);
void initTimer(void);
void readADC(void);
//////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************************************************/
#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00
/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDYn_BM             0x80
#define STATUS_STATE_BM                 0x70
#define STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F
#define ISR_ACTION_REQUIRED     1
#define ISR_IDLE                0
static uint8  packetSemaphore;
static uint32 packetCounter;
uint8 PKTLEN=12;                   // We have the information of 5 sensors which are uint16 (10 bytes).

/******************Definition of serial functions for easier working. **************************/
#define SPI_BEGIN                      P2OUT &= ~BIT1;
#define SPI_TX(x)                      do{while(!(IFG2 & UCA0TXIFG));UCA0TXBUF= (x);}while(1==-1)
#define SPI_RX(x)                      do{while(!(IFG2 & UCA0RXIFG));(x)=UCA0RXBUF;}while(1==-1)
#define SPI_END                        P2OUT |= BIT1;
#define SPI_WAIT_MISO_GO_LOW           while(P1IN & BIT1);
/****************************************txBuffer***************************************/
uint8 txBuffer[13] = {0};
/***************************************************************************************/
int main(void)
{
       WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
/****************************************************************************************/
      BCSCTL1 = CALBC1_1MHZ;          // Set range   DCOCTL = CALDCO_1MHZ;
      BCSCTL2 &= ~(DIVS_3);           // SMCLK = DCO = 1MHz
/**************************************Micro-controller Peripherals***********************/
       /**************GPIO2 Interrupt is enabled on P2.2 high>>low transition**********************/
            P2REN |=  BIT2;                             //Pullp/pulldown resistor is activated.
            P2OUT &= ~BIT2;                            //The default of the P2.2 is low.
            P2DIR &= ~BIT2;                            //P2.2 is defined as input pin.
            P2IE |=  BIT2;                            // The interrupt is activated for P2.2.
            P2IES |= BIT2;                            // The interrupt is activated on high to low transition.
            P2IFG &= ~BIT2;                           // P2.2 IFG cleared   /************************************************************************************                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      ********/
          //Configure the SPI interface.
          //UCA0CTL1 |= UCSWRST;                     // **Initialize USCI state machine**
          P1SEL =  BIT1 + BIT2 + BIT4 ;      // UCA0SOMI | UCA0SIMO | UCA0CLK |
          P1SEL2 = BIT1 + BIT2 + BIT4;     // UCA0SOMI | UCA0SIMO | UCA0CLK | The function select,PxSEL, defines a 3 wire SPI that is on P1.1/P1.2/P1.4. P1.5 is GPIO. (Controlled manually)
          UCA0CTL0 |=  UCCKPH + UCMST + UCMSB + UCSYNC;  // 3-pin, 8-bit, SPI master
          UCA0CTL1 |= UCSSEL_2;                     // SMCLK
         // UCA0BR0 |= 0x02;                          // /2
          //UCB0BR1 = 0x00;                              //
         /*****************************************/
          P1OUT |= BIT1;/* Pullup on UCA0SOMI */
          P1REN |= BIT1;
          /****************************************/
          P2DIR |= BIT1;
          P2OUT |= BIT1;   /*CSn on P2.1*/
          /***************************************/
          /* In case not automatically set */
          P1DIR |= BIT2 + BIT4;
          P1DIR &= ~BIT1;
          UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**


       initADC();
    //   initTimer();
/*******************Configure CC1200 with registers exported from SamartRF Studio**************************/

     registerConfig();                         // CC1200.h

/***********************************************************************************************************/
       __bis_SR_register(GIE);       // Enter interrupt

/*************************************************************************************************************/
        while (1)
        {
           //if (flag==1)
           //{
              readADC();
              packetCounter++;
              createPacket(txBuffer);
              cc120xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));
              trxSpiCmdStrobe(CC120X_STX);
              // Wait for interrupt that packet has been sent.
              // (Assumes the GPIO connected to the radioRxTxISR function is
              // set to GPIOx_CFG = 0x06)
              while(packetSemaphore != ISR_ACTION_REQUIRED);
               // Clear semaphore flag
              packetSemaphore = ISR_IDLE;
              //trxSpiCmdStrobe(CC120X_SIDLE);
              trxSpiCmdStrobe(CC120X_SFTX);
              flag++;

                        }
    //    }
}


// Timer A0 interrupt service routine
/*
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{   if (timerflag==5)
{
                     P1OUT |= BIT0;
                     flag=1;


   timerflag=0;
}
else
{
    timerflag++;
    P1OUT &= ~BIT0;


}
}
*/

// Port 2 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
             gpio2flag++;
            // Set packet semaphore
            packetSemaphore = ISR_ACTION_REQUIRED;

           // Clear ISR flag
             P2IFG &=~ BIT2;
}
/////////////////////////////////////////////////////////////////////////////////////////
void initADC(void)
{
    /****************************Configure ADC****************************************/

   ADC10CTL0 = SREF_1 + ADC10SHT_0 + REFON + ADC10ON + CONSEQ_0;
   ADC10CTL1 = ADC10SSEL_3;                       // SMCLK (1MHz)

}
void readADC(void)
{
    /*************************************************************************************************/
                  ADC10CTL1 = INCH_7;                       // Input A7
                  ADC10AE0 |= BIT7;                         // PA.7
                  ADC10CTL0 |= ENC + ADC10SC; //Enable conversion
                  while (ADC10CTL1 & ADC10BUSY);           // Wait if ADC10 core is active
                  sensor1=ADC10MEM;
                  ADC10CTL0 &= ~(ENC); //Disable conversion
    /**************************************************************************************************/
                  ADC10CTL1 = INCH_6;                       // Input A6
                  ADC10AE0 |= BIT6;                         // PA.6
                  ADC10CTL0 |= ENC + ADC10SC; //Enable conversion
                  while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
                  sensor2=ADC10MEM;
                  ADC10CTL0 &= ~(ENC); //Disable conversion
    /*******************************************************************************************************/
                  ADC10CTL1 = INCH_5;                       // Input A0/A3/A5/A6/A7
                  ADC10AE0 |= BIT5;                                       // PA.7,6,5,3,0 ADC options select
                  ADC10CTL0 |= ENC + ADC10SC; //Enable conversion
                 while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
                  sensor3=ADC10MEM;
                  ADC10CTL0 &= ~(ENC); //Disable conversion
    /********************************************************************************************************/
                  ADC10CTL1 = INCH_3;                       // Input A0/A3/A5/A6/A7
                  ADC10AE0 |= BIT3;                                       // PA.7,6,5,3,0 ADC options select
                  ADC10CTL0 |= ENC + ADC10SC; //Enable conversion
                 while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
                  sensor4=ADC10MEM;
                  ADC10CTL0 &= ~(ENC); //Disable conversion
    /*****************************************************************************************************/
                  ADC10CTL1 = INCH_0;                       // Input A0/A3/A5/A6/A7
                  ADC10AE0 |= BIT0;                                       // PA.7,6,5,3,0 ADC options select
                  ADC10CTL0 |= ENC + ADC10SC; //Enable conversion
                  while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
                  sensor5=ADC10MEM;
                  ADC10CTL0 &= ~(ENC); //Disable conversion
    /*****************************************************************************************************/
    }
void initTimer(void)
{
    /***************************Configure Timer****************************************************************/

  // TA1CCTL0 = CCIE;                                     // CCR0 interrupt enabled
 //   TA1CTL =  0x00C0 + TASSEL_2 + MC_2 + TACLR;         // ID=8 | ACLK | contmode| clear TAR |

    }
//////////////////////////////////////////////////////////////////////////////////////
rfStatus_t trxSpiCmdStrobe(uint8 cmd)
{
        uint8 rc;
       SPI_BEGIN;
       SPI_WAIT_MISO_GO_LOW;
       SPI_TX(cmd);
       SPI_RX(rc);
       SPI_END;
       return(rc);
}
//////////////////////////////////////////////////////////////////////////////////////
/*
* @fn          cc120xSpiWriteTxFifo
*
* @brief       Write pData to radio transmit FIFO.
*
* input parameters
*
* @param       *pData - pointer to data array that is written to TX FIFO
* @param       len    - Length of data array to be written
*
* output parameters
*
* @return      rfStatus_t
*/
rfStatus_t cc120xSpiWriteTxFifo(uint8 *pData, uint8 len)
{
 uint8 rc;
 rc = trx8BitRegAccess(0x00,CC120X_BURST_TXFIFO, pData, len);
 return (rc);
}
//////////////////////////////////////////////////////////////////////////////////////
rfStatus_t trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len)
{
  uint8 readValue;

  /* Pull CS_N low and wait for SO to go low before communication starts */
  SPI_BEGIN;
  SPI_WAIT_MISO_GO_LOW;
  /* send register address byte */
  SPI_TX(accessType|addrByte);
 // SPI_WAIT_DONE();
  /* Storing chip status */
  SPI_RX(readValue);
  trxReadWriteBurstSingle(accessType|addrByte,pData,len);
  SPI_END;
  /* return the status byte value */
  return(readValue);
}

/////////////////////////////////////////////////////////////////////////////////////
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len)
{

    uint16 i;
    /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
  if(addr&RADIO_READ_ACCESS)
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      for (i = 0; i < len; i++)
      {
          SPI_TX(0);            /* Possible to combining read and write as one access type */
         // SPI_WAIT_DONE();
          SPI_RX(*pData);     /* Store pData from last pData RX */
          pData++;
      }
    }
    else
    {
      SPI_TX(0);
     // SPI_WAIT_DONE();
      SPI_RX(*pData);
    }
  }

  else
  {
      if(addr&RADIO_BURST_ACCESS)
    {
      /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
      for (i = 0; i < len; i++)
      {
        SPI_TX(*pData);
      //  SPI_WAIT_DONE();
        pData++;
      }
    }
    else
    {
      SPI_TX(*pData);
      //SPI_WAIT_DONE();
    }
}
return;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
rfStatus_t trx16BitRegAccess(uint8 accessType, uint8 extAddr, uint8 regAddr, uint8 *pData, uint8 len)
{
  uint8 readValue;

  SPI_BEGIN;
  SPI_WAIT_MISO_GO_LOW;
  /* send extended address byte with access type bits set */
  SPI_TX(accessType|extAddr);
  //SPI_WAIT_DONE();
  /* Storing chip status */
  SPI_RX(readValue);
  SPI_TX(regAddr);
  //SPI_WAIT_DONE();
  /* Communicate len number of bytes */
  trxReadWriteBurstSingle(accessType|extAddr,pData,len);
  SPI_END;
  /* return the status byte value */
  return(readValue);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
static void registerConfig(void) {
        uint8 writeByte;

        // Reset radio
        trxSpiCmdStrobe(CC120X_SRES);

        // Write registers to radio
        for(i = 0;
            i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
            writeByte = preferredSettings[i].data;
            cc120xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
        }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////
rfStatus_t cc120xSpiWriteReg(uint16 addr, uint8 *pData, uint8 len)
{
  uint8 tempExt  = (uint8)(addr>>8);
  uint8 tempAddr = (uint8)(addr & 0x00FF);
  uint8 rc;

  /* Checking if this is a FIFO access - returns chip not ready */
  if((CC120X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;

  /* Decide what register space is accessed */
  if(!tempExt)
  {
    rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempAddr,pData,len);
  }
  else if (tempExt == 0x2F)
  {
    rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempExt,tempAddr,pData,len);
  }
  return (rc);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
rfStatus_t cc120xSpiReadReg(uint16 addr, uint8 *pData, uint8 len)
{
  uint8 tempExt  = (uint8)(addr>>8);
  uint8 tempAddr = (uint8)(addr & 0x00FF);
  uint8 rc;

  /* Checking if this is a FIFO access -> returns chip not ready  */
  if((CC120X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;

  /* Decide what register space is accessed */
  if(!tempExt)
  {
    rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempAddr,pData,len);
  }
  else if (tempExt == 0x2F)
  {
    rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempExt,tempAddr,pData,len);
  }
  return (rc);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************/
static void createPacket(uint8 txBuffer[])
{
    txBuffer[0] = PKTLEN;                            // Length byte
    txBuffer[1] = (uint8) (packetCounter >> 8);      // MSB of packetCounter
    txBuffer[2] = (uint8)  packetCounter;            // LSB of packetCounter
    txBuffer[3] =  sensor1>>8;                       // MSB sensor1
    txBuffer[4] =  sensor1;                          // LSB
    txBuffer[5] =  sensor2>>8;                       // MSB sensor2
    txBuffer[6] =  sensor2;                          // LSB
    txBuffer[7] =  sensor3>>8;                       // MSB sensor3
    txBuffer[8] =  sensor3;                          // LSB
    txBuffer[9] =  sensor4>>8;                       // MSB sensor4
    txBuffer[10] = sensor4;                          // LSB
    txBuffer[11] = sensor5>>8;                       // MSB sensor5
    txBuffer[12] = sensor5;                          // LSB
}
/********************************************************************************/

