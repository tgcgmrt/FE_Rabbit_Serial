//filename : fe_serial_mcm_cmd_4300_Scan_cmds_fast_femon _userscan.c
//apr2018:fe_mon2(), run_adc_femon2() and modified mk_pkt() for all scan mode cmds.
//the scan mode has got user defined ana mask and not fixed ana mask which selects all chnl.
///but if fe mon cmd is fired in between scan mode, ana mask and data monitored changes...corrected
///the prog, must have separate scan mode. This is upgrade of prog fe_serial_mcm_cmd_4300_Scan_cmds_fast_femon.c
//21apr17 : This prog work under both existing Online and mcmprn..final version as on may2017.
//21apr17 : CMN Box mon and fe monitoring(including L band) also control part is working fine in FE Lab over serial link.
//21apr17 : C:\Users\charu\Desktop\FE\fe_serial_mcm_cmd_code\fe_serial_mcm_cmd_4300_Scan_cmds_fast_femon.c
//fe_mon adbc and rfcm mux bits are as same as old mcm.
//27jan17 : At C01:-cmds are working, CMN BOX is working but FE Box is not working.
//21jan17 : positive mcm cmds in scan mode are working.
//19oct16 : cmd15, cmd2 working fine. two cmn_box_mon resp but just a fe_box_mon
//resp at antenna base.
//fe mon working 10 jan 2016
//copied on 20 july 2015 in following area
// C:\Users\charu\Desktop\FE\fe_serial_mcm_cmd_code\fe_serial_mcm_cmd_4300.c
// This program communicate with FE system over RS485 serial link.
// FE Box monitoring to be tested with this program in the lab.

#class auto
#use RCM43xx.lib
#use sdflash.lib

#memmap xmem

#define ADC_SCLKBRATE 115200ul    //top speed ..original
//#define ADC_SCLKBRATE 57600ul
//#define ADC_SCLKBRATE 19200ul
#define STARTCHAN 0
#define ENDCHAN 4
#define MAX_ADC_CHNL 4
#define GAINSET GAIN_1

#define CINBUFSIZE  31
#define COUTBUFSIZE 31             //4aug11 change to 31 from 1
#define DINBUFSIZE 255             //spi_ser_d ...28nov2011
#define DOUTBUFSIZE 255
#define RS232_NOCHARASSYINBRK      //23 july 2011


// This timeout period determines when an active input data stream is
// considered to have ended and is implemented within serCread.
// will discontinue collecting data 3 seconds after
// receiving any character or when maxSize are read.
#define MSSG_TMOUT 6UL //for serial, tel lab, 6msec, 6UL
//#define MSSG_TMOUT 3UL //3UL dly betn end of cmd and strt of resp for antenna

// This timeout period determines when to give up waiting for any data to
// arrive and must be implemented within the user's program, as it is below.
// will timeout after 5 seconds if no data is read.
#define IDLE_TMOUT 5000UL


//spi settings
#define SPI_MASTER
#define SPI_SER_D
#define SPI_CLK_DIVISOR 100
#define SPI_RX_PORT SPI_RX_PC

#use "spi.lib"

/*
		NULL CMD receive array in_data[] details
      in_data[0] = mcm address
      in_data[1] = lsb pkt len
      in_data[2] = usb pkt_len
      in_data[3] = id_code
      in_data[4] = lsb ctrl wrd .....mcm cmd
      in_data[5] = usb ctrl wrd
      in_data[6] = lsb arg_len
      in_data[7] = usb arg_len
      in_data[8] = chksum

		NULL CMD transmit array out_data[] details
      out_data[0] = mcm address
      out_data[1] = lsb pkt len
      out_data[2] = usb pkt len
      out_data[3] = id_code
      out_data[4] = status flag for any error in received pkt
      out_data[5] = no of logical pkt
      out_data[6] = self test code
      out_data[7] = any error in tmr0/1 and adc
      out_data[8] = chk_sum
*/

#define MAX_SET_VAL 6


//void rd_wr_mcm_addr(void);  //21july11//not required for FE
void null_cmd(void);
void set_cmd(void);
void set_idle_cmd(void);
void set_scan_cmd(void);
void set_mean_cmd(void);
void set_limit_cmd(void);
void set_ana_mask(void);
void set_digi16bit_mask(void);
void set_digi32bit_mask(void);
void set_digi64bit_mask(void);
void clr_ctrl_port(void);
void wr_ctrl_port(void);
void wr_ctrl_port_modified();
void wr_ctrl_port_32bit(void);
void wr_ctrl_port_16bit_fes();
void read_cmd(void);
void rd_ana_mask(void);
void rd_digi16bit_mask(void);
void rd_digi32bit_mask();
void rd_version(void);
void rd_mode(void);
void rd_digi64bit_mask(void);
void command(void);
void fe_ctrl_modified(void);
void fe_cmn_mon(void);
void cmn_box_mon(void);
//void fe_mon(void);   //10apr18 can be deleted
void fe_mon2(void);
void fe_run_adc(void);

void set_cmdcode(void);
void mk_pkt(void);
char chksum(char *);
void transmit(void);
void unknown_cmd(void);
void enable_485Tx(void);
void disable_485Tx(void);
void dly_rts(void);
void run_adc(void);
//void run_adc_femon(void);  //can be deleted
void run_adc_femon2(void); //for fe monitoring
void adc_set(void);


//char far in_d[40];     //receive array..abc2mcm cmd + mcm addr
char in_d[40];
char in_data[40];  //receive array..abc2mcm cmd only data
//char out_data[100]; //tranmit array..mcm2abc cmd....more size than rquired
char out_data[150]; //tranmit array..mcm2abc cmd....more size than rquired
char para_ary[8];  //stores para to be send as reply to read cmd
char ver;			 //version of prog 1.40
char para_len;
char flag_status;
char id_code;
char no_of_lp;     //no of logical pkt
char cmdl1;        //cmd   null/set/read/femcmctrl/self/reboot/femon/febox
char cmdl2;        //arg1  mode/anamask/digi16/32/64/threshold/cmn,febox
char mode;         //arg2  idle/scan/limit/limit
	                //cmdl3 to be used when cmdl2 = 0 ie mode cmd selected
char recv_mcm_addr; //abc2mcm cmd pkt mcm addr
int null_cmd_flag;//if cmd is null cmd
int cmd_err_flag; //if cmd code is wrong
int chksum_err_flag;//if chksum err deteched
int too_lgpacket;//cmd pkt too large
int icr_flag;

int cnt_scan;    // for cmd 2
//char mcm_addr;    //mcm addr set on mcm card
int mcm_addr;
char chksum_in;   //calculated chksum of received abc2mcm cmd
char ana_mask[8];
char digi16_mask[2];
char digi32_mask[4];
char digi64_mask[8];
char digi_port[8];

//SD FLASH PAGE 0
char sd_flash_buf[512];

//ADC variables
char run_adc_flag; //setmodescan will set and next cmd will acquire 64 chnls
char adc_data_len;
char scan_ana_mask_len;
char fnl_adc[64];
char ch0,ch1,ch2,ch3; //rawdata value of each channel
//char scan_ana_mask[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; //feb2018 commented out
char scan_ana_mask[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //Apr2018
char old_scan_ana_mask[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //Apr2018
char cmn_box_ana_mask[8]={0xE0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x07};
char fe_box_ana_mask[8]={0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x08};//mon chnl-0,1,2,3,4,59
char fe_cmn_mon_flag;

float v0,v1,v2,v3,v4,v5,v6,v7,volt,voltequ; //channel voltage
unsigned int value, rawdata;
//char k; //k==90 mcm addr is read from mcm board
unsigned long int t_start,t_null,t_recv_cmd;


int main()
{
   int i,j;
   int bytecntr_in;  //mcm cmd len including mcm addr byte
   int maxpktlen;    //holds maximum mcm cmd len
   int tmout_flag;   //abc2mcm cmd is not received in specific time
//int chksum_err_flag;//if chksum err deteched
//int null_cmd_flag;//if cmd is null cmd                                                                          +
//int chksum_err_flag;//if chksum err deteched
//int cmd_err_flag; //if cmd code is wrong

//char in_data[18]; //receive array..abc2mcm cmd
//char mcm_addr;    //mcm addr set on mcm card
   int chnl_num_cal;
   int ch4,ch5,ch6,ch7;
   int sermask,sermask1;
   float v0,v1,v2,v3,v4,v5,v6,v7,volt,voltequ; //channel voltage
   unsigned int rawdata;
   char spi_tx[9],spi_rx[9];
   int spi_tx_cntr;
   int adc_cmd_rd;

//t_start = MS_TIMER;
   brdInit();
//   printf("bord initiaisation time = %ld \n",MS_TIMER-t_start);
//   printf("New MCM Rabbit4010...mcm cmd implementation\n");
   i = 0;
   bytecntr_in = 0;
   mcm_addr = 0;
   maxpktlen = 0;
   chksum_in = 0;
   tmout_flag = 0;
   too_lgpacket = 0;
   null_cmd_flag = 0;
   chksum_err_flag = 0;
   cmd_err_flag = 0;
   flag_status = 0;
   icr_flag = 0;
   id_code = 0;
   ver = 20; ///ver 1.4
   mode = 0;//testing mode = idle mode by default, scan mode = 1 when set scan
   para_len = 0;
   run_adc_flag = 0;
   fe_cmn_mon_flag = 0;
   cnt_scan = 0;

//default value of ana_mask  and para_ary
   for(i=0; i<8; i++)
   {
    para_ary[i] = 0xff;   //memset(para_ary,0xFF,8);
    ana_mask[i] = scan_ana_mask[i]; // by default ana mask is all 00.
    }
   para_len = 8;
   scan_ana_mask_len = 8;

//default values for spi_tx[]
   //for(i=0; i<9; i++)
    //printf("spi_tx[%d] = %d \n",i,spi_tx[i] =1<<i);

   //printf("CPU type is 0x%x  \t ie R4000\n",(CPU_ID_MASK(_CPU_ID_)) );
   //printf("Board type is 0x%x \t ie RCM4000 with ADC \n",_BOARD_TYPE_);

   //clockDoublerOff(); //30 MHz frq max.//working fine 31aug16//fe_mon affected.

//printf("SPCR = 0x%x \t PADR =0x%x \t PBDR = 0x%x \t PBDDR = 0x%x\n",RdPortI(SPCR),RdPortI(PADR),RdPortI(PBDR),RdPortI(PBDDR));
   WrPortI(PADR,&PADRShadow,0x00); //all output pins are low
   WrPortI(SPCR,&SPCRShadow,0x84); //all pins are output

   WrPortI(PBDDR,&PBDDRShadow,0xFF);//PB4-7 mux addr as output and PB3-PB2 enable latch as output
   WrPortI(PBDR,&PBDRShadow,0xFF); //PB2-PB7 are pins are initially low

//printf("SPCR = 0x%x \t PADR =0x%x \t PBDR = 0x%x \t PBDDR = 0x%x\n",RdPortI(SPCR),RdPortI(PADR),RdPortI(PBDR),RdPortI(PBDDR));
//printf("PCFR = 0x%x \t PCDDR = 0x%x \t PCDR = 0x%x \n",RdPortI(PCFR),RdPortI(PCDDR),RdPortI(PCDR));
//getchar();

//printf("BEFORE PCALR = 0X%2x \t PCAHR = 0X%2x \n",RdPortI(PCALR),RdPortI(PCAHR));
   WrPortI(PCALR, &PCALRShadow, 0x00);
   WrPortI(PCAHR, &PCAHRShadow, 0x00);
//printf("AFTER PCALR = 0X%2x \t PCAHR = 0X%2x \n",RdPortI(PCALR),RdPortI(PCAHR));
/*
 printf("BEFORE PFCR = 0X%2x \n",RdPortI(PCFR) );
   WrPortI(PCFR,  &PCFRShadow,  (PCFRShadow & 0xC0) | 0x14);//TxD not required TxC set for serial comm
 printf("AFTER PFCR = 0X%2x \n",RdPortI(PCFR) );
   WrPortI(PCDCR, &PCDCRShadow, (PCDCRShadow & 0xC0));//PC7,PC6 output pins, open drain, serial comm
   WrPortI(PCDDR, &PCDDRShadow, (PCDDRShadow & 0xC0) | 0x17); //0x17
   WrPortI(PCDR,  &PCDRShadow,  (PCDRShadow  & 0xF9)); //set pc0pc2 /OE,TxC as high and RTS as low
*/
//printf("PCFR = 0x%x \t PCDDR = 0x%x \t PCDR = 0x%x \n",RdPortI(PCFR),RdPortI(PCDDR),RdPortI(PCDR));
 WrPortI(PCFR,&PCFRShadow,(PCFRShadow  & 0xF5) );   //set TxC for serial comm, TxD for SPI
 //WrPortI(PCDDR,&PCDDRShadow,(PCDDRShadow | 0x07) ); //pc1 as output RTS of rs485, pc0 /oe set as output
 WrPortI(PCDDR,&PCDDRShadow,(PCDDRShadow | 0x05) ); //pc1 as output RTS of rs485, pc0 /oe set as output
 //WrPortI(PCDR,&PCDRShadow,(PCDRShadow & 0x00) ); //set /OE high,TxC high and RTS low
 WrPortI(PCDR,&PCDRShadow,(PCDRShadow | 0x00) );


// Parallel port D initialisation
 	 WrPortI(PDCR,  &PDCRShadow,  0x00);	// clear all bits to pclk/2
	 WrPortI(PDFR,  &PDFRShadow,  0x00);	// No special functions
    WrPortI(PDDDR, &PDDDRShadow, 0x00);	// all bits are set as input

//PB4-7... MUX ADDR
// WrPortI(PEFR, &PEFRShadow, PEFRShadow | 0x80);    //rcm4000  //changed to 80 from a0..sep09
// WrPortI(PEAHR, &PEAHRShadow, PEAHRShadow & ~0xCC | 0x41);  //changed to 41 fron 44

   WrPortI(PECR,  &PECRShadow,  0x00);	// clear all bits to pclk/2
   WrPortI(PEAHR, &PEAHRShadow, 0x00);
   WrPortI(PEALR, &PEALRShadow, 0xC0); //PE3 output3
   WrPortI(PEFR,  &PEFRShadow,  0x08);	// PE3 used as SCLKD for SPI
   WrPortI(PEDCR, &PEDCRShadow, 0x00);	// clear all bits to drive high and low
   WrPortI(PEDDR, &PEDDRShadow, 0xFF); //all lines as output
   WrPortI(PEDR, &PEDRShadow, 0x00); //all lines set as low

//printf("PEFR=0x%x \tPEDR=0x%x \tPEDDR=0x%x\n",RdPortI(PEFR),RdPortI(PEDR),RdPortI(PEDDR));
//printf("PCFR=0x%x \tPCDR=0x%x \tPCDDR=0x%x\n",RdPortI(PCFR),RdPortI(PCDR),RdPortI(PCDDR));
//printf("PADR=0x%x \tPBDR=0x%x \tPCDR=0x%x \t PDDR=0x%x\n",RdPortI(PADR),RdPortI(PBDR),RdPortI(PCDR),RdPortI(PDDR));

//printf("initialisation of port time = %ld \n",MS_TIMER-t_start);

//ADC calibration
 for(chnl_num_cal=STARTCHAN; chnl_num_cal< ENDCHAN; chnl_num_cal++)
  {if( (anaInCalib(chnl_num_cal, SINGLE, 0,8,0.1,1820,20.0) == -1))
    {printf("ADC calibration is failed \t %d\n",chnl_num_cal);}
//   else
//    {printf("adc_Cali done for 0-20 V range \t %d\n",chnl_num_cal);}
   }
   //set ADC Vref to 2V
   //adc_cmd_rd = anaInConfig(0x07,0x3b,0);
   //printf("OSC/Ref register value is %d\n",adc_cmd_rd);
   //getchar();
//printf("ADC initiaisation time = %ld \n",MS_TIMER-t_start);


//============================================================================
/*
//In older RCM 4000 design adc inputs were used for reading MCM card address.
//on board mcm addr
 for(i=0;i<3;i++)
  {
   for(chnl_num_cal=STARTCHAN; chnl_num_cal<=ENDCHAN; chnl_num_cal++)
    {
    //anaInInfo(inputnum, &rawdata, &voltequ);
    value = anaIn(chnl_num_cal, SINGLE, GAINSET);
	 volt = (value - _adcCalibS[chnl_num_cal][GAINSET].offset)*(_adcCalibS[chnl_num_cal][GAINSET].kconst);

	 if (value == ADOVERFLOW)
	 {
		rawdata = ADOVERFLOW;  // -4096 out of  range
		voltequ = ADOVERFLOW;
      printf("ERROR out of range\n");
      //set ADC FAIL FLAG and call unknown cmd function
	 }
	 else
	 {
		rawdata = value;
      if((chnl_num_cal == 4) && i==2)      //mcm addr 0
       {ch4=rawdata; v4 = volt;}
      if((chnl_num_cal == 5) && i==2)      //mcm addr 1
       {ch5=rawdata;  v5 = volt;}
      if((chnl_num_cal == 6) && i==2)      //mcm addr 2
       {ch6=rawdata; v6 = volt;}
      if((chnl_num_cal == 7) && i==2)      //mcm addr 3
       {ch7=rawdata; v7 = volt;}

		if ((int)value <= 0) //code is not nessesary
		 voltequ	 = 0.000;
		else
		 voltequ = volt;
	  }
//printf("CH%2d is %.5f V from raw data %d\n", chnl_num_cal, voltequ, rawdata);
    }
//delay
//  for(i=0; i<10000; i++)
//   for(j=0; j<100; j++);
   }//for k<100 ends here
//printf("ch7 = %d \tch6 = %d \tch5 = %d \tch4 = %d \t",ch7,ch6,ch5,ch4);
//printf("v7 = %f \tv6 = %f \tv5 = %f \tv4 = %f \t",v7,v6,v5,v4);

//in old MCM card design adc inputs were used for reading MCM card address.
      if(ch7 >180)
       mcm_addr += 0x08;
      else
       mcm_addr += 0x00;

      if(ch6 >180)
       mcm_addr += 0x04;
      else
       mcm_addr += 0x00;

      if(ch5 >180)
       mcm_addr += 0x02;
      else
       mcm_addr += 0x00;

      if(ch4 >180)                //2.0 V logic high
       mcm_addr += 0x01;
      else
       mcm_addr += 0x00;

       mcm_addr = mcm_addr&0x0f;
*/
//===================================================================
//RCM 4300 Read mcm addr from SD CARD
 //rd_wr_mcm_addr();   //read mcm addr from SD CARD
 //mcm_addr = sd_flash_buf[0];

//Read MCM address from port for FE System.

 mcm_addr = (RdPortI(PDDR) >> 4);
 printf("on board Rabbit Mcm addr = 0x%x \t %d \n",mcm_addr,mcm_addr);
//printf("board MCM addr read time = %ld \n",MS_TIMER-t_start);


//SPI Interface for LO system
 SPIinit();
 //for(;;)
 SPIWrRd((void*)&spi_tx,(void*)&spi_rx,9);


// 1second time.
//t_start = MS_TIMER;
 value = anaIn(STARTCHAN, SINGLE, GAINSET);
 volt = (value - _adcCalibS[STARTCHAN][GAINSET].offset)*(_adcCalibS[STARTCHAN][GAINSET].kconst);
//printf("l380...sercread completion time is = %ld \n",MS_TIMER-t_start);

   maxpktlen = sizeof(in_d);
//printf("the size of in_d[] = 0X%2x and in_data[] array is  %d dec\n",maxpktlen,maxpktlen);


   //serCopen(9600L);
   if (serCopen(9600L))
    printf("Serial C Port opened \n");
   else
    printf("Serial C port problem\n");

   serCdatabits(PARAM_8BIT);  //by default databits are 8
   serCparity(PARAM_MPARITY); //0x04...mark parity
   serCflowcontrolOff();
   serCrdFlush();// Remove any waiting chars from buffer.
   serCwrFlush();

//serial port C registers
//printf("SCCR = %d \t 0X%2x \n",RdPortI(SCCR),RdPortI(SCCR) );
//printf("SCDR = %d \t 0X%2x \n",RdPortI(SCDR),RdPortI(SCDR) );
//printf("SCSR = %d \t 0X%2x \n",RdPortI(SCSR),RdPortI(SCSR) );

/* not useful to solved problem
   printf("TAT6RShadow = %d\n", TAT6RShadow);
   printf("TAT1RShadow = %d\n", TAT1RShadow);
   if(TACRShadow & 1<<6)
       printf("Timer A6 gets input from A1\n");
   else
       printf("Timer A6 gets input from pclk/2\n");
    printf("\n");

    WrPortI(TAT6R, &TAT6RShadow,96);

*/

/*
printf("Osc clock \t GCSR = %d \t 0X%2x \n",RdPortI(GCSR),RdPortI(GCSR) );
printf("Freq doubler ON \t GCDR = %d \t 0X%2x \n\n",RdPortI(GCDR),RdPortI(GCDR) );

printf("Perclk/2 \t GOCR = %d \t 0X%2x \n",RdPortI(GOCR),RdPortI(GOCR) );
//WrPortI(GOCR,&GOCRShadow,0x63); //perclk/2
printf("Perclk/2 \t GOCR = %d \t 0X%2x \n",RdPortI(GOCR),RdPortI(GOCR) );

printf("Async clock \t SCER = %d \t 0X%2x \n",RdPortI(SCER),RdPortI(SCER) );
//WrPortI(SCER,&SCERShadow,0xE6); //async clk 8 times of data rate- not working
printf("Async clock \t SCER = %d \t 0X%2x \n",RdPortI(SCER),RdPortI(SCER) );

printf("serial port div \t SCDHR = %d \t 0X%2x \n",RdPortI(SCDHR),RdPortI(SCDHR) );
printf("serial port div \t SCDLR = %d \t 0X%2x \n",RdPortI(SCDLR),RdPortI(SCDLR) );
*/

printf("waiting for cmd\n");

//cntr_while1++;
//printf("cntr_while1 = %d \n",cntr_while1);
//scsr bit2 = 0 = transmitter is IDLE and
//scsr bit3 = 0 = transmit buffer is empty
//printf("SCSR2 = 0x%x \t SCSR3 = 0x%x \n",BitRdPortI(SCSR,2),BitRdPortI(SCSR,3) );
//printf("SCSR7 = 0x%x \t SCSR5 = 0x%x \n",BitRdPortI(SCSR,7),BitRdPortI(SCSR,5) );
//printf("BEFORE SCSR = 0x%x \n",RdPortI(SCSR));

//while(!((BitRdPortI(SCSR,2) == 0)&&(BitRdPortI(SCSR,3)== 0)) ); //original  16/8/16
//printf("l408 bit 2,3 AFTER SCSR = 0x%x \n",RdPortI(SCSR));

    //while(!((BitRdPortI(SCSR,7) == 0)||(BitRdPortI(SCSR,5)== 1)) )
   // while((BitRdPortI(SCSR,7) == 0))
   //  {printf("do nothing \n");
   //   printf("SCSR7 = 0x%x \t SCSR5 = 0x%x \n",BitRdPortI(SCSR,7),BitRdPortI(SCSR,5) ); }
//printf("l413...completion time is = %ld \n",MS_TIMER-t_start);
  // while(1) //testing under software control //925mse

//t_start = MS_TIMER;
while((!serCwrUsed()) && (!BitRdPortI(SCSR,2)))
    {
    do{
//    printf("do while loop\n");

      serCrdFlush();// Remove any waiting chars.
//shifted to end of transmit function
      memset(in_d,40,0);
      memset(in_data,40,0);
      memset(out_data,100,0);

//printf("mark parity bit = %d \n",BitRdPortI(SCSR,6)); // 1 for mark parity
//printf("PCFR = 0x%x \t PCDDR = 0x%x \t PCDR = 0x%x \n",RdPortI(PCFR),RdPortI(PCDDR),RdPortI(PCDR));
//getchar();
      recv_mcm_addr = 0; //mcm addr in abc2mcm cmd pkt
      bytecntr_in = 0;  //pkt len in abc2mcmc cmd pkt
//printf("before serCread bytecntr_in = %d \t 0X%2x \t",bytecntr_in,bytecntr_in);
//printf("recv_mcm_addr = %d \t 0x%2x \n",in_d[0],in_d[0]);
//printf(" l431 before while... serCread = 0x%x \n",RdPortI(SCSR));


      while((bytecntr_in = serCread(in_d,maxpktlen,MSSG_TMOUT)) <= 0);    //original
//printf("l479...sercread completion time is 5.5 msec = %ld \n",MS_TIMER-t_start);

     //{if(bytecntr_in > 0)break;}
//printf("time required to receive cmd in in_d[] = %ld \n",MS_TIMER-t_recv_cmd);
//copy data except mcm addr from in_d[] to in_data[]
      recv_mcm_addr = in_d[0]; //mcm addr in abc2mcm cmd pkt
      bytecntr_in = in_d[1];  //pkt len in abc2mcmc cmd pkt
 		id_code = in_d[3]; //id_code for antenna testing

//id_code = 0; //id_code = 0 for Lab testing == TO BE DELETED AFTER TESTING

//printf("id_code = %x \t bytecntr_in = %d \t 0X%2x \t",id_code,bytecntr_in,bytecntr_in);
//printf("recv_mcm_addr = %d \t 0x%2x \n",in_d[0],in_d[0]);
//printf("MCM has received cmd\n");
       }
   while((mcm_addr != recv_mcm_addr) );

      for(i=0; i<maxpktlen; i++)
       in_data[i] = 0x00;

      for(i=0,j=1; i<bytecntr_in; i++,j++)
       in_data[i] = in_d[j];      //without mcm_addr

/*
//printf("received recv_mcm_addr = %x \n",recv_mcm_addr);
printf("display cmd pkt\n");
for(i=0; i<bytecntr_in; i++)
printf("%x  ",in_data[i]);
printf("\n");
//printf("mcm_addr = 0x%2x \t in_data[0] = 0x%2x \n",mcm_addr,in_d[0] );
*/
      cmdl1 = in_data[3]; //null/set/read/fe_ctrl_mod/femon_cmn_mon
      cmdl2 = in_data[7]; //set or read mode/anamask/digi16/32/64 febox/CMN mon

      if( (cmdl1 == 1)&&(in_data[8] == 0))// valid for set cmds
       {mode = 0;    //set idle
//printf("L513 mode 0x%x \n",mode); //idle/scan/mean/limit
       }

      if( (cmdl1 == 1)&&(in_data[8] == 1))// valid for set cmds
       {mode = 1;  //set scan
//printf("L518 mode 0x%x \n",mode); //idle/scan/mean/limit
       }
/* // find out mode of cmd.
      if((cmdl1 == 1)&&(cmdl2 == 0))
      {mode = in_data[8];
       printf("mode 0x%x \n",mode); //idle/scan/mean/limit
       }
*/
//check pkt len
     if( ((in_data[0] > 18) && (in_data[1] != 0)) )
      {
       too_lgpacket = 1;
//printf("mcm command pkt to large\n");
       unknown_cmd();
      }
     else
      {too_lgpacket = 0;
//printf("MCM CMD received successfully \n");
      }

      chksum_in = chksum(&in_data[0]);

  		if(chksum_in != in_data[bytecntr_in-1])
       {chksum_err_flag = 1;
//printf("chksum error..= 0x%2x \t chksum_in = 0x%2x \n",in_data[bytecntr_in],chksum_in);
        unknown_cmd();
       }
      else
       { chksum_err_flag = 0;
//printf("chksum okay chksum_in = 0x%2x \n",chksum_in);
       }

//check cmdcode for mcm command
      if( (cmdl1 < 0) && (cmdl1 > 7))
       {cmd_err_flag = 1;
        unknown_cmd();
//printf("mcm cmd error......mcm cmd error......mcm cmd error......\n");
       }
      else
       {
       cmd_err_flag = 0;
//printf("mcm cmd is valid \t mcm cmd = 0x%2x \n",in_data[3]);
       }

//if cmd code is 4 ie self test..unknown cmd
//added in switch statement
//      if( cmdl1 == 0x04 )
//       unknown_cmd();
//which mcm command

//printf("l557 .. time required to cmd pkt receive and decode is = %ld \n",MS_TIMER-t_start);
      switch(cmdl1)
       {
         case 0x00:
          {
          //printf("NULL CMD \n");
          //run_adc_flag = 0;
          null_cmd();
          }
          break;
         case 0x01:
          {
          //printf("SET CMD\n");
          adc_data_len = 0;
          run_adc_flag = 0;
          set_cmd();}
          break;
         case 0x02:
          {
           //printf("READ CMD\n");
           run_adc_flag = 0;
           adc_data_len = 0;
           read_cmd();}
          break;
/*         case 0x03:
          {printf("FE MCM CTRL CMD\n");
           read_cmd();}
          break;
*/
         case 0x04:
          {
           //printf("Self Test CMD\n");
           //run_adc_flag = 0;
           unknown_cmd();}
          break;
/*
         case 0x05:
          {printf("REBOOT CMD\n");
           read_cmd();}
          break;
*/

         case 0x06:
          {
           //printf("FE CTRL CMD\n");
           run_adc_flag = 0; // do not activate adc function
           adc_data_len = 0; // monitoring chnls are zero
           fe_ctrl_modified();
           }
          break;
// FES monitoring disable on 22 Dec 2016.
         case 0x07:
          {
           //printf("FE_CMN BOX CMD\n");
           fe_cmn_mon_flag = 1;
           run_adc_flag = 0; // default activate adc function
           adc_data_len = 0; // monitoring chnls are zero
//printf("l 622 .. FE_MON time required to cmd pkt receive and decode is = %ld \n",MS_TIMER-t_start);
           fe_cmn_mon();
           }
          break;

         default:
          unknown_cmd();
        }//switch ends here
//printf("l619...completion time is = %ld \n",MS_TIMER-t_start);
    } //while 1 infinite loop ends here line 501
  printf("\nl579  ...problem......closing serial port\n");
  serCclose();
return 0;
}//main ends here


void rd_wr_mcm_addr(void)
{
   char inchar;
   int value, rc, error_code;
   long pagenum,number;

   sd_device *dev;

   pagenum = 0L; //page0 of SD Flash by default
   number = 0L;
   value = 0;

   dev = &SD[0];
	if (rc = sdspi_initDevice(0, &SD_dev0))
   {
//printf("Flash init failed (%d): %ls\n\n", rc, error_message(rc)); //dc1064
   	printf("Flash init failed (%d): %ls\n\n", rc, error_message(rc));//dc1062
      exit(rc);
   }
//   else
//   {
//   	printf("Flash init OK\n");
//      printf("# of blocks: %ld\n", dev->sectors);
//      printf("Size of block: 512\n\n");
//   }

//WRITE MCM ADDRESS ONTO SD FLASH
     // printf("Enter MCM ADDRESS : ");
     // inchar = getchar();
     // printf("Enter char is \t %c \n",inchar);
      inchar = '3';  // mcm address hard coded to 3 for testing purpose
      printf(" MCM Address is \t %c \n",inchar);
      if(inchar >='0' && inchar <= '9')
       {number = number*10 + (inchar - '0');
        value = (int) number;}
      else
       {value = 0;} // MCM Address 0 is selected by default

//printf("\nValue is %d dec is %02x hex", value,value);
//printf("\nFilling page %ld with value %02x hex\n", pagenum,value);
	   memset(sd_flash_buf, value, 512);
      if (rc = sdspi_write_sector(dev, pagenum, sd_flash_buf))
         {
         printf("\nSD write error (%d): %ls\n", rc, error_message(rc));
         }

//READ MCM ADDRESS FROM SD FLASH
      pagenum = 0L;
//printf("\nPage %ld \n", pagenum);
      if (rc = sdspi_read_sector(dev, pagenum, sd_flash_buf))
         {
         printf("\nSD read error (%d): %ls\n", rc,error_message(rc));
         }
//printf("mcm_addr = %d \n",sd_flash_buf[0]);
}//rd_wr_mcm_addr end here

nodebug
void dly_rts(void)
{
#asm
dly:
 	ld		a, 0xff
  	djnz 	dly
#endasm
}

nodebug
void enable_485Tx( void )
{
#asm
	push	ip							;save off IP state
	ipset	1							;set interrupt priority to level 1
	//ld		a,(PDDRShadow)		;get copy of shadow reg V2 - PD4 as RTS
   ld		a,(PBDRShadow)			;get copy of shadow reg V3 - PB7 as RTS
	and		0x7F					;set bit PB7 high v3 PB7// v2 PD4 // old PC1 high
	ld		(PBDRShadow),a			;update shadow reg
	ioi	ld	(PBDR),a				;set PE0 high
  	;IOWRITE_A(PBDR)            // commented on 21 July 2014
	pop	ip							;restore IP to the previous state
  ;	ret
#endasm
}

nodebug
void disable_485Tx( void )
{
#asm
	push	ip							;save off IP state
	ipset	1							;set interrupt priority to level 1
	ld		a,(PBDRShadow)			;get copy of shadow reg
	or		0x80						;V3 set bit PB7 low; v20xEF clear bit PC1
	ld		(PBDRShadow),a			;update shadow reg
	ioi	ld	(PBDR),a				;set PB7 low
  	;IOWRITE_A(PBDR)           // commented on 21 July 2014
	pop	ip							;restore IP to the previous state
 	;ret
#endasm
}

void unknown_cmd()
{null_cmd_flag = 1; //set lp to 1
 icr_flag = 1;
 mode = 0; //set idle mode
 set_cmdcode();
 mk_pkt();
 transmit();
 printf("UNKNOWN command \n");
}
void command()
{
//printf("FE mon completion time is = %ld \n",MS_TIMER-t_start);
 null_cmd_flag = 0;      // not a NULL cmd
 set_cmdcode();
 mk_pkt();
 transmit();
 //printf("-----------command-------------------------------------------------------\n");
}

void set_cmdcode()
{
//printf("set_cmdcode function \n");
 if(chksum_err_flag)
  flag_status |= 0x02;
 if(cmd_err_flag)
  flag_status |= 0x04;
  //cmd_err_flag |= 0x04;
 if(too_lgpacket)
  flag_status |= 0x08;
  //too_lgpacket |= 0x08;
 //printf("flag status = 0x%2x \n",flag_status);
 //out_data[3] = flag_status; //Out_data[3] is ctrl word status

 if(null_cmd_flag == 1)
  no_of_lp = 1;          //Out_data[4] is Number of logical packets
 else
  no_of_lp = 2;
// printf("number of logical pkts = 0x%2x \n",no_of_lp);
}


char chksum(char *ary)
{ char temp;
  char i;
  temp = 0;
  i = 0;

  for(i=0; i< (ary[0] -1); i++)
   {
//  printf("ary[%d] = 0x%x \n",i,ary[i]);
    temp += ary[i];
   }
  temp = ~temp;
  temp += 1;
  //printf("chksum = 0x%2x \n",temp);
  return temp;
}

void transmit()
{int i,j;
 int tran_len;

 tran_len = 0;

// t_start = MS_TIMER;

 serCparity(PARAM_MPARITY); //0x04...mark parity

 enable_485Tx();

 if( !(serCputc(mcm_addr)) )
  printf("return_pkt...Serial C port trans buf is full or locked \n");

  //i=5000 for 6 msec for old mcm on old dos platform only. 8may2018.
  for(i=0; i<1200; i++); //1687@60M=1.5ms //1200@60M=600usec

 serCparity(PARAM_SPARITY); //0x05...space parity

 serCdmaOff();  //should be off otherwise data send out very fast.

 for(i=0; i<out_data[0]; i++)
  {
   serCputc(out_data[i]);
    for(j=0; j<1030; j++); //1030 count = 300-350u delay.
    }

 while(!((BitRdPortI(SCSR,2) == 0)&&(BitRdPortI(SCSR,3)== 0)) );  //6 sep 16 needed.
/*
 if( (tran_len = serCwrite(out_data,out_data[0])) > 0 )
  printf("tran_len = 0x%2x  \n",tran_len);
 else
  printf("return pkt transmission is failed\n");
*/
//dly_rts();
//for(i=0; i<1687; i++);       //(j687 for lab) creating problem for continuos null cmd..delete it
  disable_485Tx();

// printf("after function time = %ld \n",MS_TIMER-t_start);
// for(i=0; i<1687; i++);        //1687  adding delay post resp pkt

 //work at both places nicely.
      //memset(in_d,40,0);
      //memset(in_data,40,0);
      //memset(out_data,100,0);
/*
//print resp packet takes 50mses time to process prinf statements.
printf("resp pkt \n");
for(i=0; i<out_data[0]; i++)
printf("%2x ",out_data[i]);
  printf("\n\n");
*/

//printf("l773...completion time is = %ld \n",MS_TIMER-t_start);
}


void mk_pkt()
{
//printf("Assemble transmit data to be send to ABC \n");
 char j, out_cntr,lp2_len; // counter for resp pkt
 j=0;
 out_cntr=0;
 lp2_len = 0; //lp2 pkt len cntr

//printf("L873 cmdl1 = 0x%x \t cmdl2 = 0x%x \t mode = 0x%x \n",cmdl1,cmdl2,mode);
//assemble response packet
/*
//working code for cmds in idle mode
 out_data[out_cntr++] = 0x00; //pkt_len
 out_data[out_cntr++] = 0x00;
 out_data[out_cntr++] = id_code; //id_code
 out_data[out_cntr++] = flag_status;
 out_data[out_cntr++] = no_of_lp;
 //if(!((cmdl1 == 1) && (cmdl2 == 0)&& (mode == 1))) //not cmdl1 = set AND cmdl2 = scan
 if(!(run_adc_flag == 1))  //run_adc_flag should be zero to evaluate if statement to true
 {
  out_data[out_cntr++] = 0x04;
  out_data[out_cntr++] = 0x00;
  out_data[out_cntr++] = 0x04;
  out_data[out_cntr++] = 0x00;
 }
 if(null_cmd_flag == 1)               //null cmd resp pkt
  {out_data[0] = out_cntr+1;          // pkt_len cntr = 9 and 9+1 = 10
   chksum_in = chksum(&out_data[0]);
//printf("out_cntr = %d \t resp pkt chksum = 0x%2x\n",out_cntr,chksum_in);
   out_data[out_cntr] = chksum_in;    //out_data[a] = chksum_in
  }
 else
  {
printf("L895 cmdl1 = 0x%x \t cmdl2 = 0x%x \t mode = 0x%x \n",cmdl1,cmdl2,mode);
   //if((cmdl1 == 1) && (cmdl2 == 0)&&(mode == 1)) //not cmdl1 = set AND cmdl2 = scan
   //logical packet2 length lsb byte calculation for NON-NULL cmds
   if(run_adc_flag == 1)    //scan_mode
    {
//     if(cmdl1 == 7) //fe_cmn_mon func
//      {out_data[out_cntr++] = 3+in_data[5]+adc_data_len;}
     out_data[out_cntr++] = 3+in_data[5]+para_len+adc_data_len; //set scan mode
     //printf("para_len = 0x%x adc_data_len = 0x%x \n",para_len,adc_data_len);
    } //logical pkt len usb
   else if(fe_cmn_mon_flag == 1)
    {out_data[out_cntr++] = 3+in_data[5]+adc_data_len;} //logical pkt len usb
   else if((cmdl1 == 2) && (in_data[7] <6))
    {out_data[out_cntr++] = 3+in_data[5]+para_len;
     printf("correct 2lp pkt_len \n"); } //Read cmd pkt
   else
     {out_data[out_cntr++] = 3+in_data[5];
      printf("incorrect 2lp pkt_len \n"); } //logical pkt2 len usb

   out_data[out_cntr++] = 0;                //logical pkt2 len lsb
   out_data[out_cntr++] = cmdl1;            //set,read...what cmd received
   for(j=0; j<in_data[5]; j++,out_cntr++)   //arg length
    out_data[out_cntr] = in_data[7+j];      //actual arg starts here

   //READ cmd
   if(cmdl1 == 2)
    {
     for(j=0; j<para_len; j++,out_cntr++) //ana mask....
     {out_data[out_cntr] = para_ary[j];}
    }//if

   //lp2_len = 0; only for set scan mode
   //if((cmdl1 == 0x01) && (cmdl2 == 0x00) && (mode == 0x01))   //for scan cmd only
   if((run_adc_flag == 1) && (cmdl1 != 7))
    {
     for(j=0; j<para_len; j++,out_cntr++) //ana mask....
      {out_data[out_cntr] = para_ary[j];
       //printf("cntr = %d  para_ary[%d] = 0x%x \n",out_cntr,j,out_data[out_cntr]);
       }
     }//cmdl1 != 7

   //for cmd7(cmn_box and fe box monitoring) and set scan mode
    for(j=0; j<adc_data_len; j++)    //write data on to resp pkt
     out_data[out_cntr++] = fnl_adc[j];

    lp2_len = 0; //only for set scan mode
    if((run_adc_flag == 1) && (cmdl1 != 7))
    {
       out_data[out_cntr++] = 0;       //lp2 pkt len
       lp2_len++;
       out_data[out_cntr++] = 0;
       lp2_len++;
       out_data[out_cntr++] = cmdl1;
       lp2_len++;
       for(j=0; j<in_data[5]; j++,out_cntr++,lp2_len++)//arg length
        out_data[out_cntr] = in_data[7+j]; //actual arg starts here mode scan

       out_data[out_cntr - lp2_len] = lp2_len; //lp2_pkt len
    }//if run_adc_flag == 1

   out_data[0] = out_cntr+1;          // total pkt_len
   chksum_in = chksum(&out_data[0]); // chksum
//printf("out_cntr = %d \t resp pkt chksum = 0x%2x\n",out_cntr,chksum_in);
   out_data[out_cntr] = chksum_in;
   }
// getchar(); //soft test mode

*/

//if mode is idle
if (mode == 0)
{
 out_data[out_cntr++] = 0x00; //pkt_len
 out_data[out_cntr++] = 0x00;
 out_data[out_cntr++] = id_code; //id_code
 out_data[out_cntr++] = flag_status;
 out_data[out_cntr++] = no_of_lp;
 //if(!((cmdl1 == 1) && (cmdl2 == 0)&& (mode == 1))) //not cmdl1 = set AND cmdl2 = scan
 if(!(run_adc_flag == 1))  //run_adc_flag should be zero to evaluate if statement to true
 {
  out_data[out_cntr++] = 0x04;
  out_data[out_cntr++] = 0x00;
  out_data[out_cntr++] = 0x04;
  out_data[out_cntr++] = 0x00;
 }
 if(null_cmd_flag == 1)               //null cmd resp pkt
  {out_data[0] = out_cntr+1;          // pkt_len cntr = 9 and 9+1 = 10
   chksum_in = chksum(&out_data[0]);
//printf("out_cntr = %d \t resp pkt chksum = 0x%2x\n",out_cntr,chksum_in);
   out_data[out_cntr] = chksum_in;    //out_data[a] = chksum_in
  }
 else
  {
//printf("L895 cmdl1 = 0x%x \t cmdl2 = 0x%x \t mode = 0x%x \n",cmdl1,cmdl2,mode);
   //if((cmdl1 == 1) && (cmdl2 == 0)&&(mode == 1)) //not cmdl1 = set AND cmdl2 = scan
   //logical packet2 length lsb byte calculation for NON-NULL cmds
   if(run_adc_flag == 1)    //scan_mode
    {
//     if(cmdl1 == 7) //fe_cmn_mon func
//      {out_data[out_cntr++] = 3+in_data[5]+adc_data_len;}
     out_data[out_cntr++] = 3+in_data[5]+para_len+adc_data_len; //set scan mode
     //printf("para_len = 0x%x adc_data_len = 0x%x \n",para_len,adc_data_len);
    } //logical pkt len usb
   else if(fe_cmn_mon_flag == 1)
    {out_data[out_cntr++] = 3+in_data[5]+adc_data_len;} //logical pkt len usb
   else if((cmdl1 == 2) && (in_data[7] <6))
    {out_data[out_cntr++] = 3+in_data[5]+para_len; } //Read cmd pkt
   else
     {out_data[out_cntr++] = 3+in_data[5]; } //logical pkt2 len usb

   out_data[out_cntr++] = 0;                //logical pkt2 len lsb
   out_data[out_cntr++] = cmdl1;            //set,read...what cmd received
   for(j=0; j<in_data[5]; j++,out_cntr++)   //arg length
    out_data[out_cntr] = in_data[7+j];      //actual arg starts here

   //READ cmd
   if(cmdl1 == 2)
    {
     for(j=0; j<para_len; j++,out_cntr++) //ana mask....
     {out_data[out_cntr] = para_ary[j];}
    }//if

   //lp2_len = 0; only for set scan mode
   //if((cmdl1 == 0x01) && (cmdl2 == 0x00) && (mode == 0x01))   //for scan cmd only
   if((run_adc_flag == 1) && (cmdl1 != 7))
    {
     for(j=0; j<para_len; j++,out_cntr++) //ana mask....
      {out_data[out_cntr] = para_ary[j];
       //printf("cntr = %d  para_ary[%d] = 0x%x \n",out_cntr,j,out_data[out_cntr]);
       }
     }//cmdl1 != 7

   //for cmd7(cmn_box and fe box monitoring) and set scan mode
    for(j=0; j<adc_data_len; j++)    //write data on to resp pkt
     out_data[out_cntr++] = fnl_adc[j];

    lp2_len = 0; //only for set scan mode
    if((run_adc_flag == 1) && (cmdl1 != 7))
    {
       out_data[out_cntr++] = 0;       //lp2 pkt len
       lp2_len++;
       out_data[out_cntr++] = 0;
       lp2_len++;
       out_data[out_cntr++] = cmdl1;
       lp2_len++;
       for(j=0; j<in_data[5]; j++,out_cntr++,lp2_len++)//arg length
        out_data[out_cntr] = in_data[7+j]; //actual arg starts here mode scan

       out_data[out_cntr - lp2_len] = lp2_len; //lp2_pkt len
    }//if run_adc_flag == 1

   out_data[0] = out_cntr+1;          // total pkt_len
   chksum_in = chksum(&out_data[0]); // chksum
//printf("out_cntr = %d \t resp pkt chksum = 0x%2x\n",out_cntr,chksum_in);
   out_data[out_cntr] = chksum_in;
   }
// getchar();
 }//if mode is idle

if (mode == 1)     // mk pkt for cmds in scan mode
{
 run_adc_flag = 1;

 out_data[out_cntr++] = 0x00; //pkt_len
 out_data[out_cntr++] = 0x00;
 out_data[out_cntr++] = id_code; //id_code
 out_data[out_cntr++] = flag_status;

 if(cmdl1 == 0)
  no_of_lp = 1;
 else
  no_of_lp = 2;


  out_data[out_cntr++] = no_of_lp;

  for(j=0; j<8; j++){
   if((cmdl1 == 1) && (cmdl2 == 1)) // for set ana mask
    ana_mask[j] = old_scan_ana_mask[j];
   else
    ana_mask[j] = scan_ana_mask[j]; // for rest of cmds
   }
   run_adc(); // use ana mask set by set_ana_mask

   //for cmd7(cmn_box and fe box monitoring) and set scan mode
   //printf("l086 ..adc_data_len = %d \t para_len = %d \n",adc_data_len, para_len);


   out_data[out_cntr++] = 3+2+scan_ana_mask_len+adc_data_len; //logical pkt len lsb
// printf("l1099 adc_data_len = %d \t para_len = %d \t in_data[7] = %x \n",adc_data_len, para_len, in_data[7]);
   out_data[out_cntr++] = 0;                //logical pkt2 len usb

   out_data[out_cntr++] = 1;            // set command
   out_data[out_cntr++] = 0;            // set mode command
   out_data[out_cntr++] = 1;            // set scan mode command


   for(j=0; j<8; j++,out_cntr++) //ana mask
     {
      if( (cmdl1 == 1) && (cmdl2 == 1))
       out_data[out_cntr] = old_scan_ana_mask[j];
      else
       out_data[out_cntr] = scan_ana_mask[j];
     }

   for(j=0; j<adc_data_len; j++)    //write adc data on to resp pkt
     out_data[out_cntr++] = fnl_adc[j];


    if((cmdl1 == 7)&&(in_data[7] == 1)) //cmnbox mon
     {
      for(j=0; j<8; j++)
      ana_mask[j] = cmn_box_ana_mask[j];

      run_adc();

      out_data[out_cntr++] = 2+2+adc_data_len; //2nd logical pkt len lsb
      out_data[out_cntr++] = 0;
      out_data[out_cntr++] = cmdl1;
      out_data[out_cntr++] = in_data[7];

      for(j=0; j<adc_data_len; j++)    //write adc data on to resp pkt
       out_data[out_cntr++] = fnl_adc[j];
//printf("adc_data_len = %d \t in_data[7] = %x \n",adc_data_len, in_data[7]);
     }


     if((cmdl1 == 7)&&(in_data[7] == 0)) //febox mon
     {
      for(j=0; j<8; j++)
      ana_mask[j] = fe_box_ana_mask[j];

      run_adc_femon2();
//printf("l1132 femon..adc_data_len = %d \t in_data[7] = %x \n",adc_data_len, in_data[7]);
      out_data[out_cntr++] = 3+2+adc_data_len; //2nd logical pkt len lsb
      out_data[out_cntr++] = 0;
      out_data[out_cntr++] = cmdl1;
      out_data[out_cntr++] = in_data[7];
      out_data[out_cntr++] = in_data[8]; //fe box no

      for(j=0; j<adc_data_len; j++)    //write adc data on to resp pkt
       out_data[out_cntr++] = fnl_adc[j];
     }

    lp2_len = 0; //only for set scan mode
    if((cmdl1 != 7) && (cmdl1 !=0))
    {
       out_data[out_cntr++] = 0;       //lp2 pkt len   lsb
       lp2_len++;
       out_data[out_cntr++] = 0;
       lp2_len++;
       out_data[out_cntr++] = cmdl1;
       lp2_len++;
       for(j=0; j<in_data[5]; j++,out_cntr++,lp2_len++)//arg length
        out_data[out_cntr] = in_data[7+j]; //actual arg starts here mode scan
//printf("i am in read ana maks cmdl1 = %x \t cmdl2 =%x \n",cmdl1,cmdl2);
       if( (cmdl1 == 2) && (in_data[5] ==1) && (in_data[7] == 0))   //for read ana mask cmd
        {
         for(j=0; j<8; j++,lp2_len++)
          out_data[out_cntr++] = scan_ana_mask[j];
        }

       if( (cmdl1 == 2) && (in_data[5] == 1) && (in_data[7] == 1) )  //read 16 bit digi mask
        {for(j=0; j<2; j++,lp2_len++)
         out_data[out_cntr++] = digi_port[j];
        }
       if( (cmdl1 == 2) && (in_data[5] == 1) && (in_data[7] == 2) )  //read 32 bit digi mask
        {for(j=0; j<4; j++,lp2_len++)
         out_data[out_cntr++] = digi_port[j];
        }
       if( (cmdl1 == 2) && (in_data[5] == 1) && (in_data[7] == 5) )  //read 64 bit digi mask
        {for(j=0; j<8; j++,lp2_len++)
         out_data[out_cntr++] = digi_port[j];
        }
       if( (cmdl1 == 2) && (in_data[5] == 1) && (in_data[7] == 3) )  //read version
        {for(j=0; j<1; j++,lp2_len++)
         out_data[out_cntr++] = ver;
        }
       if( (cmdl1 == 2) && (in_data[5] == 1) && (in_data[7] == 4) )  //read version
        {for(j=0; j<1; j++,lp2_len++)
         out_data[out_cntr++] = mode;
        }

       out_data[out_cntr - lp2_len] = lp2_len; //lp2_pkt len
    }//if run_adc_flag == 1

   out_data[0] = out_cntr+1;          // total pkt_len
   chksum_in = chksum(&out_data[0]); // chksum
//printf("out_cntr = %d \t resp pkt chksum = 0x%2x\n",out_cntr,chksum_in);
   out_data[out_cntr] = chksum_in;

// getchar();
 }//if mode is scan

}

void null_cmd()
{
//printf("null cmd function \n");
//lsb arg_len
 if(in_data[5] != 0x00)
  printf("null cmd failed \n");
//cmd_failed();

 null_cmd_flag = 1;
 set_cmdcode();
 mk_pkt();
 transmit();
}


void set_cmd()
{
/* //no effect on prog. execution
printf("set_cmd()PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",RdPortI(PADR),RdPortI(PBDR),RdPortI(PCDR));
  WrPortI(PADR,&PADRShadow,PADRShadow & (~0xFF));
  WrPortI(PBDR,&PBDRShadow,PBDRShadow & (~0xFE));
  WrPortI(PCDR,&PCDRShadow,PCDRShadow & (~0x01));
printf("set_cmd()PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",RdPortI(PADR),RdPortI(PBDR),RdPortI(PCDR));
*/
 if((cmdl2 == 0) && (mode == 0))
  {run_adc_flag = 0;cnt_scan = 0; set_idle_cmd();}
 if((cmdl2 == 0) && (mode == 1))   // set scan mode 1st
   {
   //run_adc_flag = 0;// default de-activate adc function
    adc_data_len = 0; // monitoring chnls are zero
    set_scan_cmd();
    }
 if((cmdl2 == 0) && (mode == 2))
  {run_adc_flag = 0; set_mean_cmd();}
 if((cmdl2 == 0) && (mode == 3))
  {run_adc_flag = 0; set_limit_cmd();}
 if(cmdl2 == 1)
  set_ana_mask();
 if(cmdl2 == 2)
  set_digi16bit_mask();
 if(cmdl2 == 3)
  set_digi32bit_mask();
 if(cmdl2 == 4)
  set_digi64bit_mask();
}

void set_idle_cmd()
{
//printf("set_idle_cmd function \n");
command();
}

void set_scan_cmd()
{int i;
//printf("set_scan_cmd function \n");
//copy scan mask in ana_mask
 for(i=0; i<8; i++)
  {ana_mask[i] = scan_ana_mask[i];
      //printf("scan mask[%d] = 0X%x",i,ana_mask[i]);
      //mask is use defined
  }
//printf(" before IF LOOP mode = %d \t run_adc+flag = %d \t cnt_scan = %d \n",mode,run_adc_flag,cnt_scan);
 if((mode == 1) && (cnt_scan == 1))
  {run_adc_flag = 1;
   run_adc(); // use ana mask set by set_ana_mask
   command();
//printf(" if ...mode = %d \t run_adc_flag = %d \t cnt_scan = %d\n",mode,run_adc_flag,cnt_scan);
  }
 else
  {
//printf("else... mode = %d \t run_adc_flag = %d \t cnt_scan = %d\n",mode,run_adc_flag,cnt_scan);
   cnt_scan = 1;
   command();
  }

/*
 run_adc_flag = 1;

 if(run_adc_flag == 1)
 {//printf("run_adc_flag.......mode is scan\n");
   run_adc(); // use ana mask set by set_ana_mask
  }
command();
*/
}

void run_adc()
{char i,j,k,l;
 char adc_data[64];
 int mux_addr;
 int chnl_num;



  for(mux_addr=0,k=0; mux_addr <=0x0F; mux_addr++,k++)
   {
//t_start = MS_TIMER;
    WrPortI(PEDR,&PEDRShadow,(mux_addr<<4)); //mux on Rabbit mcm card set mux address
     //printf("PEDR 0x%2x \n",RdPortI(PEDR));
     //printf("mux_addr shift = %2x \n",mux_addr<<4);

   for(chnl_num = STARTCHAN,l=0; chnl_num < MAX_ADC_CHNL; chnl_num++,l++)
    {
//t_start = MS_TIMER;
     value = anaIn(chnl_num, SINGLE, GAINSET);
	  volt = (value - _adcCalibS[chnl_num][GAINSET].offset)*(_adcCalibS[chnl_num][GAINSET].kconst);
//printf("value = %d \t chnl_num = %d \t volt = %f \n",value,chnl_num,volt);
//printf("l969...sercread completion time is = %ld \n",MS_TIMER-t_start);

	  if(value == ADOVERFLOW)
	   {
		 rawdata = ADOVERFLOW;  // -4096 out of  range
		 voltequ = ADOVERFLOW;
       //printf("ERROR ADC DATA out of range\n");
	   }
	  else
	   {
       if((int)value <= 0)
		  voltequ = 0.000;
		 else
		  voltequ = volt;

//mcm adc ouput and rabbit adc output mapping
       value = (int)value/7.8; 	//V3 7.8 //1.95 v2 //original 1.784 july12
		 rawdata = ~value;
       rawdata = (char) rawdata;

//printf("mux_addr = %d  chnl_num = %d  rawdat = %d ",mux_addr,chnl_num,rawdata);
//printf("value = 0x%x \t rawdat = %x \t",value,rawdata);
       if(chnl_num == 0)      //adc input 0 //mcm chnl 00-15
        {ch0 = rawdata;
         v0 = voltequ;
//printf("ch0 = %d  ch0 = 0x%x  v0 = %.2f \n",ch0,ch0,v0);
//printf("chnl_num = %2d mux_addr = %2d (chnl_num*16)+mux_addr = %2d \n",chnl_num,mux_addr,(chnl_num*16)+mux_addr);
         adc_data[(chnl_num*16)+mux_addr]=ch0;
         }      //17nov11

       if(chnl_num == 1)      //adc input 1 //mcm chnl 16-31
        {ch1=rawdata; v1 = voltequ;
//printf("ch1 = %d  ch1 = 0x%x  v1 = %.2f \n",ch1,ch1,v1);
//printf("chnl_num = %2d mux_addr = %2d (chnl_num*16)+mux_addr = %2d \n",chnl_num,mux_addr,(chnl_num*16)+mux_addr);
         adc_data[(chnl_num*16)+mux_addr]=ch1;}      //17nov11

       if(chnl_num == 2)      //adc input 2 //mcm chnl 32-47
        {ch2=rawdata; v2 = voltequ;
//printf("ch2 = %d  ch2 = 0x%x  v2 = %.2f \n",ch2,ch2,v2);
//printf("chnl_num = %2d mux_addr = %2d (chnl_num*16)+mux_addr = %2d \n",chnl_num,mux_addr,(chnl_num*16)+mux_addr);
        adc_data[(chnl_num*16)+mux_addr]=ch2;}      //17nov11

       if(chnl_num == 3)      //adc input 3 //mcm chnl 48-63
        {ch3=rawdata; v3 = voltequ;
//printf("ch3 = %d  ch3 = 0x%x  v3 = %.2f \n\n",ch3,ch3,v3);
//printf("chnl_num = %2d mux_addr = %2d (chnl_num*16)+mux_addr = %2d \n",chnl_num,mux_addr,(chnl_num*16)+mux_addr);
        adc_data[(chnl_num*16)+mux_addr]=ch3;}

	   }//else ends here
//printf("l073..for loop chnl_num..2048 times for 32 fe mon points RUN_ADC completion time is = %ld l = %d\n",MS_TIMER-t_start,l);
     }// for ..chnl_num ends here
//getchar(); //mux channel by channel for testing purpose
//printf("l076..for loop mux_addr.. 512 times for 32 fe mon points RUN_ADC completion time is = %ld k = %d\n",MS_TIMER-t_start,k);
    }//for...mux_addr ends here
//printf("l078..for loop mux_addr.. one data scan(6 chnls) of fe mon points RUN_ADC completion time is 0 msec = %ld k = %d\n",MS_TIMER-t_start,k);
// print 64 channel data
//  for(i=0; i<64; i++)
//   printf("adc_data[%d] = %d \t",i,adc_data[i]);

// print analog mask
// for(i=0; i<8; i++)
//   printf("ana_mask[%d] = 0x%x \t",i,ana_mask[i]);
// printf("\n");

//t_start = MS_TIMER;

  for(i=0,adc_data_len=0; i<8; i++)  //mask bytes //14msec
   for(j=0; j<8; j++) //bits in each mask bytes
    {
     if( ana_mask[i] & (1<<j) ) //check if mux chnl is set for monitoring
     {
      fnl_adc[adc_data_len++] = adc_data[(i*8)+j]; //original
//printf(" i = %d j= %d \t fnl_adc[%d]=%d adc_data_len = %d\n",i,j,((i*8)+j),fnl_adc[(i*8)+j],adc_data_len);
     }
    }
//printf("number of channel set are adc_data_len = %d \n",adc_data_len);
// fe_mon- fe_box 6 data is (ch59) in fnl_adc[5] array element
//for(i=0; i<adc_data_len; i++)  // fe mon data 6 values displyed 0,1,2,3,4,59
// printf("fnl_adc[%d]=%d \t adc_data_len = %d\n",i,fnl_adc[i],adc_data_len);
//getchar();
//printf("l104..RUN_ADC completion time for one fe mon scan is i msec = %ld \n",MS_TIMER-t_start);
}//run_adc ends here


void run_adc_femon(void)
{char i,j,k,l;
 char adc_data[64];
 int mux_addr, fe_mux_addr;
 int chnl_num;

//acquire data for one channel at a time.
  if(in_data[8] == 5)
   fe_mux_addr = 11;
  else
   fe_mux_addr = in_data[8];

 //printf("in_data[8] = args1 = %x \t fe_mux_addr = %x \n",in_data[8],fe_mux_addr);

  for(mux_addr=0,k=0; mux_addr <=0x0F; mux_addr++,k++)
   {
    if(mux_addr == fe_mux_addr)    // fe_mux_addr 0/1/2/3/4/11
    {
     WrPortI(PEDR,&PEDRShadow,(fe_mux_addr<<4)); //mux on Rabbit mcm card set mux address
     for(chnl_num = STARTCHAN,l=0; chnl_num < MAX_ADC_CHNL; chnl_num++,l++)
     {
     value = anaIn(chnl_num, SINGLE, GAINSET);
	  volt = (value - _adcCalibS[chnl_num][GAINSET].offset)*(_adcCalibS[chnl_num][GAINSET].kconst);

	  if(value == ADOVERFLOW)
	   {
		 rawdata = ADOVERFLOW;  // -4096 out of  range
		 voltequ = ADOVERFLOW;
       //printf("ERROR ADC DATA out of range\n");
	   }
	  else
	   {
       if((int)value <= 0)
		  voltequ = 0.000;
		 else
		  voltequ = volt;

//mcm adc ouput and rabbit adc output mapping
       value = (int)value/7.8; 	//V3 7.8 //1.95 v2 //original 1.784 july12
		 rawdata = ~value;
       rawdata = (char) rawdata;

//printf("mux_addr = %d  chnl_num = %d  rawdat = %d \t volt = %f \n",mux_addr,chnl_num,rawdata,volt);
//printf("value = 0x%x \t rawdat = %x \t",value,rawdata);

       if(chnl_num == 0)      //adc input 0 //mcm chnl 00-15
        {ch0 = rawdata;
         v0 = voltequ;
//printf("mux_addr = %d \t chnl_num = %d \t ch0 = %d  ch0 = 0x%x\n",mux_addr,chnl_num,ch0,ch0);
         }      //17nov11

       if(chnl_num == 1)      //adc input 1 //mcm chnl 16-31
        {ch1=rawdata; v1 = voltequ;
//printf("ch1 = %d  ch1 = 0x%x  v1 = %.2f \n",ch1,ch1,v1);
        }      //17nov11

       if(chnl_num == 2)      //adc input 2 //mcm chnl 32-47
        {ch2=rawdata; v2 = voltequ;
//printf("ch2 = %d  ch2 = 0x%x  v2 = %.2f \n",ch2,ch2,v2);
        }      //17nov11

       if(chnl_num == 3)      //adc input 3 //mcm chnl 48-63
        {ch3=rawdata; v3 = voltequ;
//printf("ch3 = %d  ch3 = 0x%x  v3 = %.2f fe_box_6 chnl = %d \n\n",ch3,ch3,v3,(16*3)+mux_addr);
        }
	   }//else ends here
//printf("l073..for loop chnl_num..2048 times for 32 fe mon points RUN_ADC completion time is = %ld l = %d\n",MS_TIMER-t_start,l);
     }// for ..chnl_num ends here
//getchar(); //mux channel by channel for testing purpose
//printf("l076..for loop mux_addr.. 512 times for 32 fe mon points RUN_ADC completion time is = %ld k = %d\n",MS_TIMER-t_start,k);
    }//if mux_addr == data_in[8] ends here
   }//for...mux_addr ends here
}//run_adc_femon ends here


void run_adc_femon2(void)
{char i,j,k,l;
 char adc_data[64];
 char temp_fe_mon;
 char fe_box_data[32];
 int mux_addr, fe_mux_addr;
 int chnl_num;

//--------------------------------------------
//acquire data for one channel at a time.
  if(in_data[8] == 5)
   fe_mux_addr = 11;
  else
   fe_mux_addr = in_data[8];

//generate mux address for RFCM CARD - FE BOX.
 for(i=0,j=0,temp_fe_mon=0; i<32; i++,j++)
  {
     if(i/16)
      {
       temp_fe_mon = (i<<4);  //adr3 adr2 adr1 adr0 d c b a
       temp_fe_mon |= (0x0F); //set A5 high for selecting 2nd mux// original
//printf("second i = %d \t temp_fe_mon = %2x \t digi_port[0] = %x \n",i,temp_fe_mon,digi_port[0]);
      }
     else
      {
       temp_fe_mon = (i<<4); //control bit 12..15
       temp_fe_mon &= (0xF0); //set A5 low for selecting 1st mux (default mux)
//printf("first i = %d \t temp_fe_mon = %2x \t digi_port[0] = %x \n",i,temp_fe_mon,digi_port[0]);
      }

//printf("i = %d \t temp_fe_mon = %2x \t digi_port[0] = %x \n",i,temp_fe_mon,digi_port[0]);

//set RFCM card MUX addr(A3-A0) and A4(mux ic select)
      digi_port[0] = 0x00;   // as per old mcm, LSB BYTE control data set to 00H
		digi_port[1] = temp_fe_mon; //mux addr byte
//printf("ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);
      wr_ctrl_port_16bit_fes();  // write mux addr and mux sel on control port


 //printf("in_data[8] = args1 = %x \t fe_mux_addr = %x \n",in_data[8],fe_mux_addr);

  for(mux_addr=0,k=0; mux_addr <=0x0F; mux_addr++,k++)
   {
    if(mux_addr == fe_mux_addr)    // fe_mux_addr 0/1/2/3/4/11
    {
     WrPortI(PEDR,&PEDRShadow,(fe_mux_addr<<4)); //mux on Rabbit mcm card set mux address
     for(chnl_num = STARTCHAN,l=0; chnl_num < MAX_ADC_CHNL; chnl_num++,l++)
     {
     value = anaIn(chnl_num, SINGLE, GAINSET);
	  volt = (value - _adcCalibS[chnl_num][GAINSET].offset)*(_adcCalibS[chnl_num][GAINSET].kconst);

	  if(value == ADOVERFLOW)
	   {
		 rawdata = ADOVERFLOW;  // -4096 out of  range
		 voltequ = ADOVERFLOW;
       //printf("ERROR ADC DATA out of range\n");
	   }
	  else
	   {
       if((int)value <= 0)
		  voltequ = 0.000;
		 else
		  voltequ = volt;

//mcm adc ouput and rabbit adc output mapping
       value = (int)value/7.8; 	//V3 7.8 //1.95 v2 //original 1.784 july12
		 rawdata = ~value;
       rawdata = (char) rawdata;

//printf("mux_addr = %d  chnl_num = %d  rawdat = %d \t volt = %f \n",mux_addr,chnl_num,rawdata,volt);
//printf("value = 0x%x \t rawdat = %x \t",value,rawdata);

       if(chnl_num == 0)      //adc input 0 //mcm chnl 00-15
        {ch0 = rawdata;
         v0 = voltequ;
//printf("mux_addr = %d \t chnl_num = %d \t ch0 = %d  ch0 = 0x%x\n",mux_addr,chnl_num,ch0,ch0);
         }      //17nov11

       if(chnl_num == 1)      //adc input 1 //mcm chnl 16-31
        {ch1=rawdata; v1 = voltequ;
//printf("ch1 = %d  ch1 = 0x%x  v1 = %.2f \n",ch1,ch1,v1);
        }      //17nov11

       if(chnl_num == 2)      //adc input 2 //mcm chnl 32-47
        {ch2=rawdata; v2 = voltequ;
//printf("ch2 = %d  ch2 = 0x%x  v2 = %.2f \n",ch2,ch2,v2);
        }      //17nov11

       if(chnl_num == 3)      //adc input 3 //mcm chnl 48-63
        {ch3=rawdata; v3 = voltequ;
//printf("ch3 = %d  ch3 = 0x%x  v3 = %.2f fe_box_6 chnl = %d \n\n",ch3,ch3,v3,(16*3)+mux_addr);
        }
	   }//else ends here
//printf("l073..for loop chnl_num..2048 times for 32 fe mon points RUN_ADC completion time is = %ld l = %d\n",MS_TIMER-t_start,l);
     }// for ..chnl_num ends here
//getchar(); //mux channel by channel for testing purpose
//printf("l076..for loop mux_addr.. 512 times for 32 fe mon points RUN_ADC completion time is = %ld k = %d\n",MS_TIMER-t_start,k);
    }//if mux_addr == data_in[8] ends here
   }//for...mux_addr ends here

//--------------------------------------------------
 // fe box no used for coping data into array
// fe box 6th data in fnl_adc[5]
  //fe_box_data[j] = fnl_adc[in_data[8]];
  if(in_data[8] == 5)
   fe_box_data[j] = ch3;
  else
   fe_box_data[j] = ch0;

//printf("j = %d \t fe box = %d \t = %x \n",j,in_data[8],fe_box_data[j]);

//getchar();

/*
  // added on25 july 2016  and deleted on 13 jan17
  //set RFCM card MUX addr(A3-A0) and A4(mux ic select)
      digi_port[0] = 0x00;   // as per old mcm, control data set to 00H
		digi_port[1] = 0x00; //mux addr byte
//printf("ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);
      wr_ctrl_port_16bit_fes();  // write mux addr on control port
*/
//getchar();

  } //for i<32 ends here.  l1449

//make mux addr adresses to zero.
    //digi_port[0] = 0x00;   // as per old mcm, LSB BYTE control data set to 00H
	 digi_port[1] = 0x00; //mux addr byte
//printf("ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);
    wr_ctrl_port_16bit_fes();  // write mux addr on control port

//printf(" after in loop FE_MON ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);

//following lines required for return pkt
//t_start = MS_TIMER;
  for(i=0,adc_data_len=0; i<32; i++,adc_data_len++){
   fnl_adc[i] = fe_box_data[i]; //mk_pkt() uses fnl_adc and adc_data_len for updating ret_pkt
//printf("i = %d \t adc_data_len = %d \t fnl_adc[%d] = 0X%x \n",i,adc_data_len,i,fnl_adc[i]);
   }
   adc_data_len = i++;   // adc_data_len should be 32
//printf("l511..for loop ..FE mon array copy completion time is  0 msec = %d \n",MS_TIMER-t_start);


}//run_adc_femon ends here

void set_mean_cmd()
{
//printf("set_mean_cmd function \n");
command();
}
void set_limit_cmd()
{
//printf("set_limit_cmd function \n");
command();
}
void set_ana_mask()
{int i;

//printf("set_ana_mask function \n");
 for(i=0; i<(in_data[5]-1); i++)
  {old_scan_ana_mask[i] = scan_ana_mask[i];
   scan_ana_mask[i] = in_data[5+3+i];
   para_ary[i] = scan_ana_mask[i];
   //printf("set_ana_mask()..ana_mask[%d]= 0x%x \n",i,ana_mask[i]);
   }
   para_len = i;
//   printf("set_ana_mask()..para_len = %d \n",para_len);
command();
}

void clr_ctrl_port()
{ //this function is not in use
  WrPortI(PADR,&PADRShadow,PADRShadow & (~0xFF));
  WrPortI(PBDR,&PBDRShadow,PBDRShadow & (~0xFE));
  WrPortI(PCDR,&PCDRShadow,PCDRShadow & (~0x01));
}

void wr_ctrl_port_32bit()
{int i;
 int j,l,k;//for testing

 //PB2 = LE1 = latch 1 enable = makes latch 1, 8 bit write operation
 //PB3 = LE2 = latch 2 enable = makes latch 2, 8 bit write operation


   for(i=0; i<(in_data[5]-1);i++)
   {
    BitWrPortI(PBDR,&PBDRShadow,1,6); //set PB6 high to disable OE\ of U14 and U15
    BitWrPortI(PBDR,&PBDRShadow,0,2); //set PB2 low to disable latch1 U14 D00-07
    BitWrPortI(PBDR,&PBDRShadow,0,3); //set PB3 low to disable latch2 U14 D08-15
    BitWrPortI(PBDR,&PBDRShadow,0,4); //set PB2 low to disable latch1 U15 D16-23
    BitWrPortI(PBDR,&PBDRShadow,0,5); //set PB3 low to disable latch2 U15 D24-31

    WrPortI(PADR,&PADRShadow,digi_port[i]);        //data 0
    BitWrPortI(PBDR,&PBDRShadow,1,2); //3.set PB2 high to enable latch1 U14 D00-07
    //printf("i=[%d] PADR = %x \n",i,RdPortI(PADR));

    WrPortI(PADR,&PADRShadow,(digi_port[++i]));    //data 1
    BitWrPortI(PBDR,&PBDRShadow,1,3); //set PB3 low LE1
    //printf("i=[%d] PADR = %x \n",i,RdPortI(PADR));

    WrPortI(PADR,&PADRShadow,digi_port[++i]);      //data 2
    BitWrPortI(PBDR,&PBDRShadow,1,4); //
    //printf("i=[%d] PADR = %x \n",i,RdPortI(PADR));

    WrPortI(PADR,&PADRShadow,(digi_port[++i]));    //data 3
    BitWrPortI(PBDR,&PBDRShadow,1,5); //set PB5 high
    //printf("i=[%d] PADR = %x \n",i,RdPortI(PADR));

    BitWrPortI(PBDR,&PBDRShadow,0,6); //set PB6 low to OE\ of latches U14,U15
    //printf(" i = %d \t in_data = %d \n",i,in_data[5]-1);
    //getchar(); //test purpose
   }
}


void wr_ctrl_port_16bit_fes()
{int i,j;

    for(j=0,i=0; j<((in_data[5])/2);j++,i++)
    {
     //BitWrPortI(PBDR,&PBDRShadow,1,6); //set PB6 high to disable OE\ of U14   - 09july2016
	  BitWrPortI(PBDR,&PBDRShadow,0,2); //set PB2 low to disable latch1 U14 D00-07
     BitWrPortI(PBDR,&PBDRShadow,0,3); //set PB3 low to disable latch2 U13 D08-15


      WrPortI(PADR,&PADRShadow,digi_port[i]);   // write first byte of data
      BitWrPortI(PBDR,&PBDRShadow,1,2); //set PB2 high to enable latch U14 D00-07
//printf(" first data \t i=[%d] PADR = %2x \n",i,RdPortI(PADR));

      WrPortI(PADR,&PADRShadow,digi_port[++i]);   // write second byte of data
      BitWrPortI(PBDR,&PBDRShadow,1,3); //set PB3 high to enable latch U14 D08-15
//printf(" second data \t i=[%d] PADR = %2x \n",i,RdPortI(PADR));

      BitWrPortI(PBDR,&PBDRShadow,0,6); //set PB6 low to OE\ of latch U14 D00-15
//printf(" i = %d \t in_data = %d \n",i,in_data[5]-1);
      //getchar();
     }

}

void wr_ctrl_port_modified()
{int i;

   //BitWrPortI(PCDR,&PCDRShadow,1,0); //2.set PC0 low to disable control port output
   BitWrPortI(PBDR,&PBDRShadow,0,2); //set PB2 low to disable latch1 U14 D00-15
   BitWrPortI(PBDR,&PBDRShadow,0,3); //set PB3 low to disable latch2 U13 D16-31
   //printf("wr_ctrl_port_modified()  \n");

   for(i=0; i<(in_data[5]-1);i++)
   {
//printf("%d set_digi PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",i,RdPortI(PADR),RdPortI(PBDR)& 0xFE,RdPortI(PCDR)& 0x01);

//printf("%d set_digi PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",i,RdPortI(PADR),RdPortI(PBDR)& 0xFE,RdPortI(PCDR)& 0x01);
    WrPortI(PADR,&PADRShadow,digi_port[i]);
    WrPortI(PEDR,&PEDRShadow,(digi_port[++i]));
    //BitWrPortI(PDDR,&PDDRShadow,(digi_port[i]& 0x04),7 ); //write data1.2(PE2) bit on PD7
    BitWrPortI(PDDR,&PDDRShadow,(digi_port[i]& 0x04),6 ); //write data10 data1.2(PE2) bit on PD6
    BitWrPortI(PDDR,&PDDRShadow,(digi_port[i]& 0x08),7 ); //write data11 data1.2(PE3) bit on PD7

    BitWrPortI(PBDR,&PBDRShadow,1,2); //3.set PB2 high to enable latch1 U14 D00-15

    BitWrPortI(PDDR,&PDDRShadow,0,5); //2.set PD5 /OE low to enable control port output
    BitWrPortI(PDDR,&PDDRShadow,0,5); //2.set PD5 /OE low to enable control port output
    //BitWrPortI(PCDR,&PCDRShadow,0,0); //set PC0 low .. /OE
    //BitWrPortI(PCDR,&PCDRShadow,0,0); //set PC0 low .. /OE

    BitWrPortI(PBDR,&PBDRShadow,0,2); //3.set PB2 low to disable latch1 U13 D16-31
    //BitWrPortI(PCDR,&PCDRShadow,1,0); //1.set PC0 high to disable control port output
    BitWrPortI(PDDR,&PDDRShadow,1,5); //1.set PC0 high to disable control port output
//printf("%d set_digi PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",i,RdPortI(PADR),RdPortI(PBDR)& 0xFE,RdPortI(PCDR)& 0x01);
//printf("....................\n");
    }
}

//rcm4000
void wr_ctrl_port()
{int i;
   for(i=0; i<(in_data[5]-1);i++)
   {
//printf("%d set_digi PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",i,RdPortI(PADR),RdPortI(PBDR)& 0xFE,RdPortI(PCDR)& 0x01);

//printf("%d set_digi PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",i,RdPortI(PADR),RdPortI(PBDR)& 0xFE,RdPortI(PCDR)& 0x01);
    WrPortI(PADR,&PADRShadow,digi_port[i]);
    WrPortI(PEDR,&PEDRShadow,(digi_port[i+1]));
    i++; //second byte of 2byte control output
//printf("%d set_digi PA = 0x%x \t PB = 0x%x \t PC = 0x%x \n",i,RdPortI(PADR),RdPortI(PBDR)& 0xFE,RdPortI(PCDR)& 0x01);
//printf("....................\n");
    }
}

void set_digi16bit_mask()
{int i;
//printf("set_digi16bit_mask function \n");
//printf("digi mask values ara %x \n",(in_data[5]-1));
 for(i=0; i<(in_data[5]-1); i++)      //(arg_len - 1) gives set of 16 bit to send
  {digi16_mask[i] = in_data[5+3+i];
   digi_port[i] = in_data[5+3+i];     //fill the dmask values
   //printf("digi16_mask[%d]= 0x%x\n",i,digi16_mask[i]);
  }
  //printf("set_digi16bit\n");
  //wr_ctrl_port(); //rcm4000
  //wr_ctrl_port_32bit();  //32 bit control port output
  //wr_ctrl_port_modified(); //16 bit control port output
   wr_ctrl_port_16bit_fes();
command();
}

void set_digi32bit_mask()
{int i;
//printf("set_digi32bit_mask function \n");
//printf("digi mask values ara %x \n",(in_data[5]-1));
 for(i=0; i<(in_data[5]-1); i++)
  {digi32_mask[i] = in_data[5+3+i];
   digi_port[i] = in_data[5+3+i];
   //printf("digi32_mask[%d]= 0x%x\n",i,digi32_mask[i]);
  }

  //printf("set_digi32bit\n"); //testing purpose
  //wr_ctrl_port();
  //wr_ctrl_port_modified();
  //wr_ctrl_port_32bit();   // 32 bit modeWORKING 09SEP15
  wr_ctrl_port_16bit_fes(); // 16 bit mode
command();
}
void set_digi64bit_mask()
{int i;
//printf("set_digi64bit_mask function \n");
//printf("digi mask values ara %x \n",(in_data[5]-1));
 for(i=0; i<(in_data[5]-1); i++)
  {digi64_mask[i] = in_data[5+3+i];
   digi_port[i] = in_data[5+3+i];
   //printf("digi64_mask[%d]= 0x%x\n",i,digi64_mask[i]);
  }
  //wr_ctrl_port();
  //wr_ctrl_port_32bit();   // 32 bit modeWORKING 09SEP15
  //wr_ctrl_port_modified();
  wr_ctrl_port_16bit_fes(); // 16 bit mode
command();
}

void read_cmd()
{
 if(cmdl2 == 0)
  rd_ana_mask();
 if(cmdl2 == 1)
  rd_digi16bit_mask();
 if(cmdl2 == 2)
  rd_digi32bit_mask();
 if(cmdl2 == 3)
  rd_version();
 if(cmdl2 == 4)
  rd_mode();
 if(cmdl2 == 5)
  rd_digi64bit_mask();
}
void rd_ana_mask()
{int k;
//printf("rd_ana_mask function \n");
//only for test purpose                                                                           l
para_len = sizeof(ana_mask);
//printf("para_len = %d \n",para_len);
for(k=0; k<para_len; k++)
 para_ary[k] = ana_mask[k];

command();
}

void rd_digi16bit_mask()
{int k;
//printf("rd_digi16bit_mask function \n");
//only for test purpose
para_len = sizeof(digi16_mask);
//printf("para_len = %d \n",para_len);
for(k=0; k<para_len; k++)
 para_ary[k] = digi16_mask[k];

command();
}
void rd_digi32bit_mask()
{int k;
//printf("rd_digi32bit_mask function \n");
//only for test purpose
para_len =sizeof(digi32_mask);
//printf("para_len = %d \n",para_len);

for(k=0; k<para_len; k++)
 para_ary[k] = digi32_mask[k];

command();
}
void rd_version()
{int k;
//printf("rd_version function \n");
//only for test purpose
para_len = sizeof(ver);
//printf("para_len = %d \n",para_len);
for(k=0; k<para_len; k++)
 para_ary[k] = ver;  //prog version number is set to 9

command();
}
void rd_mode()
{int k;
//printf("rd_mode function \n");
//only for test purpose
para_len = sizeof(mode);
//printf("para_len = %d \n",para_len);
for(k=0; k<para_len; k++)
 para_ary[k] = mode;

command();
}

void rd_digi64bit_mask()
{int k;
//printf("rd_digi64bit_mask function \n");
//only for test purpose
para_len = sizeof(digi64_mask);
//printf("para_len = %d \n",para_len);
for(k=0; k<para_len; k++)
 para_ary[k] = digi64_mask[k];

command();
}

void fe_ctrl_modified()
{
 char fe_data, fe_addr;
 char fe_box_no;
 int i;

fe_box_no =  in_data[8];

digi_port[0] = in_data[7];
digi_port[1] = in_data[8];
wr_ctrl_port_16bit_fes();
for(i=0; i<350;i++);

digi_port[0] = in_data[7];    // to latch data on Interface card.
digi_port[1] = 0;
wr_ctrl_port_16bit_fes();

command();
}

void fe_cmn_mon()
{int i;

 if(in_data[5]>2)
  { cmd_err_flag = 1;
    unknown_cmd(); }

 if(in_data[5] == 1)     //arg_len = 1 cmn_box cmd
 {
  for(i=0; i<8; i++)
     {ana_mask[i] = cmn_box_ana_mask[i];
      //printf("mask[%d] = 0X%x",i,ana_mask[i]);
      //mask is 0xE0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x07
      }
//printf("l 1418 .. CMN_BOX MON time required before execution of cmd is = %ld \n",MS_TIMER-t_start);

  cmn_box_mon();

 }

 if(in_data[5] == 2)    //arg_len = 2 fe_mon cmd
 {
//printf("ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);

  for(i=0; i<8; i++)
     {ana_mask[i] = fe_box_ana_mask[i];
     //printf("mask[%d] = 0X%x",i,ana_mask[i]);
     //mask is 0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x08//mon chnl-0,1,2,3,4,59
     }
//printf("l 1431 .. FE_BOX MON time required before execution of cmd is = %ld \n",MS_TIMER-t_start);
   fe_mon2();

  } // if for arg_len = 2 ends here

 } //fe_cmn_mon() ends here.

void fe_mon(void)
{
 int i,j;
 char fe_mon_data[32],temp_fe_mon;
 char fe_box_data[32];

 if(in_data[8] > 6) //fe box no. should not grater than 6
  unknown_cmd();
 //printf("You are in FE_MON\n");

//generate mux address for RFCM CARD - FE BOX.
 for(i=0,j=0,temp_fe_mon=0; i<32; i++,j++)
  {
     if(i/16)
      {
       temp_fe_mon = (i<<4);  //adr3 adr2 adr1 adr0 d c b a
       temp_fe_mon |= (0x0F); //set A5 high for selecting 2nd mux// original
//printf("second i = %d \t temp_fe_mon = %2x \t digi_port[0] = %x \n",i,temp_fe_mon,digi_port[0]);
      }
     else
      {
       temp_fe_mon = (i<<4); //control bit 12..15
       temp_fe_mon &= (0xF0); //set A5 low for selecting 1st mux (default mux)
//printf("first i = %d \t temp_fe_mon = %2x \t digi_port[0] = %x \n",i,temp_fe_mon,digi_port[0]);
      }

//printf("i = %d \t temp_fe_mon = %2x \t digi_port[0] = %x \n",i,temp_fe_mon,digi_port[0]);

//set RFCM card MUX addr(A3-A0) and A4(mux ic select)
      digi_port[0] = 0x00;   // as per old mcm, LSB BYTE control data set to 00H
		digi_port[1] = temp_fe_mon; //mux addr byte
//printf("ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);
      wr_ctrl_port_16bit_fes();  // write mux addr and mux sel on control port

//printf(" l1722 ... before in loop FE_MON ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);

//run_adc();
    if(fe_cmn_mon_flag == 1)
     {//printf("run_adc_flag.......cmd is cmn_box_mon\n");
//t_start = MS_TIMER;
     // run_adc();    //20apr17
     run_adc_femon();
//printf("l1470..run_adc ..FE mon completion time is 1 msec * 32 = %ld \n",MS_TIMER-t_start);
     }

//check if data from selected feed box
//depending on box no. data written on fe_box_data

// fe box no used for coping data into array
// fe box 6th data in fnl_adc[5]
  //fe_box_data[j] = fnl_adc[in_data[8]];
  if(in_data[8] == 5)
   fe_box_data[j] = ch3;
  else
   fe_box_data[j] = ch0;

//printf("j = %d \t fe box = %d \t = %x \n",j,in_data[8],fe_box_data[j]);

//getchar();

/*
  // added on25 july 2016  and deleted on 13 jan17
  //set RFCM card MUX addr(A3-A0) and A4(mux ic select)
      digi_port[0] = 0x00;   // as per old mcm, control data set to 00H
		digi_port[1] = 0x00; //mux addr byte
//printf("ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);
      wr_ctrl_port_16bit_fes();  // write mux addr on control port
*/
//getchar();

  } //for i<32 ends here.

//make mux addr adresses to zero.
    //digi_port[0] = 0x00;   // as per old mcm, LSB BYTE control data set to 00H
	 digi_port[1] = 0x00; //mux addr byte
//printf("ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);
    wr_ctrl_port_16bit_fes();  // write mux addr on control port

//printf(" after in loop FE_MON ctrlwrd msb = 0X%2x \t ctrlwrd lsb = 0X%2x\n",digi_port[1],digi_port[0]);

//following lines required for return pkt
//t_start = MS_TIMER;
  for(i=0,adc_data_len=0; i<32; i++,adc_data_len++){
   fnl_adc[i] = fe_box_data[i]; //mk_pkt() uses fnl_adc and adc_data_len for updating ret_pkt
//printf("i = %d \t adc_data_len = %d \t fnl_adc[%d] = 0X%x \n",i,adc_data_len,i,fnl_adc[i]);
   }
   adc_data_len = i++;   // adc_data_len should be 32
//printf("l511..for loop ..FE mon array copy completion time is  0 msec = %d \n",MS_TIMER-t_start);

//t_start = MS_TIMER;
//printf("l 1513 .. FE_MON() time required before execution of cmd is = %ld \n",MS_TIMER-t_start);
command();
//printf("l 1515 .. FE_MON() after command() time required before execution of cmd is = %ld \n",MS_TIMER-t_start);

}
void fe_mon2(void)
{
 if(in_data[8] > 6) //fe box no. should not grater than 6
  unknown_cmd();
 //printf("You are in FE_MON\n");

 if(fe_cmn_mon_flag == 1)
  {//printf("run_adc_flag.......cmd is cmn_box_mon\n");
     run_adc_femon2();
//printf("l1470..run_adc ..FE mon completion time is 1 msec * 32 = %ld \n",MS_TIMER-t_start);
  }

command();
//printf("l 1515 .. FE_MON() after command() time required before execution of cmd is = %ld \n",MS_TIMER-t_start);

}

void cmn_box_mon()
{
//t_start = MS_TIMER;
 if(fe_cmn_mon_flag == 1)
 {//printf("run_adc_flag.......cmd is cmn_box_mon\n");
  run_adc();
  }
//printf("CMN mon completion time is = %ld \n",MS_TIMER-t_start);
command();
}