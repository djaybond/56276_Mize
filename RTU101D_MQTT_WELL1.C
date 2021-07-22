/****************************************************
 RTU101D THIRD TEST
 08/24/18
 ***************************************************

Notes:
1. 8/16/18   Digital Inputs and Outputs are working
2. 8/16/18   Serial Ports 1 and 2 are transmitting (have not tested receive yet)
3. 8/16/18   Serial ports working at 4800, 9600 and 19200 baud
4. 8/18/18   SPI bus working.
4. 8/18/18   Analog Inputs working.
5. 8/19/18   Analog Outputs working.
6. 8/19/18   Temperature Reading working (change to SPIMode 0 for this chip)
7. 8/23/18   RS485 port working and Battery OK circuit works.

*******************************************************************/
#memmap xmem

#define TESTING 1

#define Epsilon 0.001
#define BRICKNETSES 1
// mask, offset and size for fixed header fields
#define MSG_TYPE_OFFSET 0x04
#define DUP_FLAG_OFFSET 0x03
#define QOS_LEVEL_OFFSET 0x01
#define RETAIN_FLAG_OFFSET 0x00

// MQTT message types
#define MQTT_MSG_CONNECT_TYPE 0x01
#define MQTT_MSG_CONNACK_TYPE 0x02
#define MQTT_MSG_PUBLISH_TYPE 0x03
#define MQTT_MSG_PUBACK_TYPE 0x04
#define MQTT_MSG_PUBREC_TYPE 0x05
#define MQTT_MSG_PUBREL_TYPE 0x06
#define MQTT_MSG_PUBCOMP_TYPE 0x07
#define MQTT_MSG_SUBSCRIBE_TYPE 0x08
#define MQTT_MSG_SUBACK_TYPE 0x09
#define MQTT_MSG_UNSUBSCRIBE_TYPE 0x0A
#define MQTT_MSG_UNSUBACK_TYPE 0x0B
#define MQTT_MSG_PINGREQ_TYPE 0x0C
#define MQTT_MSG_PINGRESP_TYPE 0x0D
#define MQTT_MSG_DISCONNECT_TYPE 0x0E

// [v3.1.1] MQTT flag bits
#define MQTT_MSG_CONNECT_FLAG_BITS 0x00
#define MQTT_MSG_CONNACK_FLAG_BITS 0x00
#define MQTT_MSG_PUBLISH_FLAG_BITS 0x00 // just defined as 0x00 but depends on publish props (dup, qos, retain)
#define MQTT_MSG_PUBACK_FLAG_BITS 0x00
#define MQTT_MSG_PUBREC_FLAG_BITS 0x00
#define MQTT_MSG_PUBREL_FLAG_BITS 0x02
#define MQTT_MSG_PUBCOMP_FLAG_BITS 0x00
#define MQTT_MSG_SUBSCRIBE_FLAG_BITS 0x02
#define MQTT_MSG_SUBACK_FLAG_BITS 0x00
#define MQTT_MSG_UNSUBSCRIBE_FLAG_BITS 0x02
#define MQTT_MSG_UNSUBACK_FLAG_BITS 0x00
#define MQTT_MSG_PINGREQ_FLAG_BITS 0x00
#define MQTT_MSG_PINGRESP_FLAG_BITS 0x00
#define MQTT_MSG_DISCONNECT_FLAG_BITS 0x00

// QOS levels
#define QOS_LEVEL_AT_MOST_ONCE 0x00
#define QOS_LEVEL_AT_LEAST_ONCE 0x01
#define QOS_LEVEL_EXACTLY_ONCE 0x02

// SUBSCRIBE QoS level granted failure [v3.1.1]
#define QOS_LEVEL_GRANTED_FAILURE 0x80

#define MAX_TOPIC_LENGTH 65535
#define MIN_TOPIC_LENGTH 1
#define MESSAGE_ID_SIZE 2

// protocol name supported
#define PROTOCOL_NAME_V3_1_1 "MQTT" // [v.3.1.1]

// variable header fields
#define PROTOCOL_NAME_LEN_SIZE 2
#define PROTOCOL_NAME_V3_1_1_SIZE 4 // [v.3.1.1]
#define PROTOCOL_VERSION_SIZE 1
#define CONNECT_FLAGS_SIZE 1
#define KEEP_ALIVE_TIME_SIZE 2

#define PROTOCOL_VERSION_V3_1_1 0x04 // [v.3.1.1]
#define KEEP_ALIVE_PERIOD_DEFAULT 0x3C // 60 seconds
#define MAX_KEEP_ALIVE 65535 // 16 bit

// connect flags
#define USERNAME_FLAG_OFFSET 0x07
#define PASSWORD_FLAG_OFFSET 0x06
#define WILL_RETAIN_FLAG_OFFSET 0x05
#define WILL_QOS_FLAG_OFFSET 0x03
#define WILL_FLAG_OFFSET 0x02
#define CLEAN_SESSION_FLAG_OFFSET 0x01
#define MM 0x4D
#define QQ 0x51
#define TT 0x54
#define PINGRESP_TIMEOUT 10

#define NUMBEROFDI 2
#define NUMBEROFAI 2
#define NUMBEROFDO 4
#define NUMBEROFAO 2
#define NUMBEROFWELLS 4

// DEFINES FOR DEBUG REDIRECT OF PRINTF TO SERIAL PORT DURING RUNTIME
#define STDIO_DEBUG_SERIAL SDDR
//#define STDIO_DEBUG_SERIAL SADR
#define STDIO_DEBUG_BAUD 57600
#define STDIO_DEBUG_ADDCR


// --- SPI SETUP FOR DYNAMIC C ROUTINES ---
#define SPI_MASTER
#define SPI_SER_A               // use serial port A for SPI
#define SPI_RX_PORT SPI_RX_PC   // receive bit on parallel port C
#define SPI_CLOCK_MODE 1        // normal polarity, inactive low CLK signal
#define CLOCK_PORT B            // use parallel port B for CLK
#define CLOCK_BIT 1             // use parallel port bit 1 for CLK
#define SPI_CLK_DIVISOR 200
#use SPI.LIB       // put SPI defines before this line

//-----------------------

// *** SERIAL SETUP ***
#use RS232.LIB
#define XINBUFSIZE 63
#define XOUTBUFSIZE 127
#define BINBUFSIZE 31
#define BOUTBUFSIZE 31

#define MODBUS_RTU   1
#define MODBUS_ASCII 2
#define MODBUS_PROTOCOL MODBUS_RTU


#define PORT1_BAUD_RATE 4800       //RS232 PORT C (#1)
#define PORT2_BAUD_RATE 57600       //RS232 PORT D (#2)
#define PORT3_BAUD_RATE 4800       //RS485 PORT E (#3)
#define PORT4_BAUD_RATE 115200      //CELLMODEM PORT F (#4)

#define MODBUS      1
#define BRICKNET    2
#define DF1         3
#define UNUSED		4
#define MQTT        5

#define DEFAULT_ADDR_PORT1   1
#define DEFAULT_ADDR_PORT2   2

#define TOTAL_PORTS 2

#define PORT1		BRICKNET				// BRICKNET/MODBUS
#define	P1_MASTER	0				// 1/0 for MASTER/SLAVE
#define	P1_RADIO	1					// 1/0 for RADIO/DIRECT
#define	P1_DEBUG	0					// 1 to enable DEBUG

#define PORT2      MODBUS
#define P2_MASTER  0
#define P2_RADIO   0               // 1/0 for RADIO/DIRECT
#define P2_DEBUG   0               // 1 to enable DEBUG

#define PORT3   UNUSED               // RS485 PORT: MODBUS/BRICKNET/DF1

#define SER_PORT1 0
#define SER_PORT2 1

#define TRUE 1
#define FALSE 0
// This macro should be used unless DMA-available CTS/RTS pins are wired up.
//  DMA is not on by default, but #defining this reduces code size.
#define SER_DMA_DISABLE

#define PROGRAM_REVISION_DATE "11/09/20" // MM/DD/YY

#define RECV_STATE_LOCKOUT 2    // 2 sec

#define MAX_RESP_TIMEOUT 3       // 3 sec

#define PING_TIME 10             // every 30 sec need to ping the failed RTU's

#define MAX_RETRIES 3

#define HIGHEST_EXPECTED_RTU_NUM 100

#define MAXREPEATERROUTES 10

#define BANKSIZE 10000

//----------------------------------------------------------------------------
//-----------------------------   END OF USER DEFINES --------------------------
//----------------------------------------------------------------------------
#define BRICKNET_MASTER (PORT1 == BRICKNET & P1_MASTER)|(PORT2 == BRICKNET & P2_MASTER)

#define BRICKNET_SLAVE  (PORT1 == BRICKNET & !P1_MASTER)|(PORT2 == BRICKNET & !P2_MASTER)

#define MODBUS_MASTER   (PORT1 == MODBUS & P1_MASTER)|(PORT2 == MODBUS & P2_MASTER)

#define MODBUS_SLAVE    (PORT1 == MODBUS & !P1_MASTER)|(PORT2 == MODBUS & !P2_MASTER)

#if ((MODBUS_SLAVE || MODBUS_MASTER) && (MODBUS_PROTOCOL != MODBUS_RTU && MODBUS_PROTOCOL != MODBUS_ASCII))
#fatal "Pick a Modbus protcol (Modbus RTU or Modbus ASCII) for the Modbus port on line 156!!"
#endif

#define RELOAD_ISR_A 60   // 1ms - 60
#define RELOAD_ISR_B 9   // 1ms - 1.86

shared unsigned int scaler_a, scaler_b;
extern shared unsigned long MS_TIMER;


// RS232 #1 (PORT C)
#if PORT1 == BRICKNET
#define CINBUFSIZE 511
#define COUTBUFSIZE 511
#define BUFSIZE1 COUTBUFSIZE+1
#elif PORT1 == MODBUS
#define CINBUFSIZE 511
#define COUTBUFSIZE 511
#define BUFSIZE1 COUTBUFSIZE+1
#elif PORT1 == DF1
#define CINBUFSIZE 1023
#define COUTBUFSIZE 511
#define BUFSIZE1 COUTBUFSIZE+1
#else
#define CINBUFSIZE 63
#define COUTBUFSIZE 63
#define BUFSIZE1 COUTBUFSIZE+1
#endif

#define SERC_RTS_PORT PCDR
#define SERC_RTS_SHADOW PCDRShadow
#define SERC_RTS_BIT 4
#define SERC_CTS_PORT PEDR
#define SERC_CTS_BIT 2

// RS232 #2 (PORT D)
#if PORT2 == BRICKNET
#define DINBUFSIZE 511
#define DOUTBUFSIZE 511
#define BUFSIZE2 DOUTBUFSIZE+1
#elif PORT2 == MODBUS
#define DINBUFSIZE 511
#define DOUTBUFSIZE 511
#define BUFSIZE2 DOUTBUFSIZE+1
#elif PORT2 == DF1
#define DINBUFSIZE 1023
#define DOUTBUFSIZE 511
#define BUFSIZE2 DOUTBUFSIZE+1
#else
#define DINBUFSIZE 63
#define DOUTBUFSIZE 63
#define BUFSIZE2 DOUTBUFSIZE+1
#endif

#define SERD_RTS_PORT PCDR
#define SERD_RTS_SHADOW PCDRShadow
#define SERD_RTS_BIT 5
#define SERD_CTS_PORT PEDR
#define SERD_CTS_BIT 3

// RS485 (PORT E)
#if PORT3 == MODBUS
#define EINBUFSIZE 255
#define EOUTBUFSIZE 255
#define BUFSIZE3 EOUTBUFSIZE+1
#elif PORT3 == MODBUS
#define EINBUFSIZE 511
#define EOUTBUFSIZE 511
#define BUFSIZE3 EOUTBUFSIZE+1
#elif PORT3 == DF1
#define EINBUFSIZE 1023
#define EOUTBUFSIZE 511
#define BUFSIZE3 EOUTBUFSIZE+1
#else
#define EINBUFSIZE 63
#define EOUTBUFSIZE 63
#define BUFSIZE3 EOUTBUFSIZE+1
#endif

#define SERE_TXPORT PEDR
#define SERE_RXPORT PEDR

#define SERF_TXPORT PDDR
#define SERF_RXPORT PDDR

// CELL MODEM (PORT F)
#define FINBUFSIZE 511		// DO NOT SET HIGHER THAN 511
#define FOUTBUFSIZE 511		// DO NOT SET HIGHER THAN 511

#define BUFSIZE 512

#define UWORD unsigned int
#define UINT8 unsigned char
#define UINT32 unsigned long
#define ON 1
#define OFF 0
#define TRUE 1
#define FALSE 0
#define DI_BANK 1
#define DO_BANK 2
#define AI_BANK 3
#define AO_BANK 4
#define AI_SCALED_BANK 5
#define AO_SCALED_BANK 6
#define PING 1
#define READ 2
#define WRITE 3

#define TYPE_CHAR 1
#define TYPE_INT8 2
#define TYPE_INT16 3
#define TYPE_INT32 4
#define TYPE_FLOAT 5
#define TYPE_DOUBLE 6
#define MAXEVENTS 25
#define TOTAL_EDGE_BLOCKS 10
#define TOTAL_DELTA_BLOCKS 15
#define ANALOG_DEBOUNCE_TIME 30    // sec
#define DIGITAL_DEBOUNCE_TIME 5    // sec

#define TOTAL_STAT_REGS 20

#define SPI_CLK 0
#define SPI_DIN 0
#define SPI_DOUT 1
#define ADC_CS  1
#define DAC_CS 4


// macros
#define DO1_LOW         BitWrPortI(PBDR,&PBDRShadow,0,2)  //(PB2)
#define DO1_HIGH        BitWrPortI(PBDR,&PBDRShadow,1,2)  //(PB2)
#define DO2_LOW         BitWrPortI(PBDR,&PBDRShadow,0,3)  //(PB3)
#define DO2_HIGH        BitWrPortI(PBDR,&PBDRShadow,1,3)  //(PB3)
#define DO3_LOW         BitWrPortI(PBDR,&PBDRShadow,0,4)  //(PB4)
#define DO3_HIGH        BitWrPortI(PBDR,&PBDRShadow,1,4)  //(PB4)
#define DO4_LOW         BitWrPortI(PBDR,&PBDRShadow,0,5)  //(PB5)
#define DO4_HIGH        BitWrPortI(PBDR,&PBDRShadow,1,5)  //(PB5)

// Temperature
#define TEMP_CE_HIGH       BitWrPortI(PBDR,&PBDRShadow,1,7)  //(PB7)
#define TEMP_CE_LOW        BitWrPortI(PBDR,&PBDRShadow,0,7)  //(PB7)
float Deg_F = 0.0;

// Analog Input D/A numbers
unsigned int   AI1_LRV = 0x2850;  //(4ma)  READ 8/17/18
unsigned int   AI1_URV = 0xCAD0;  //(20ma) READ 8/17/18
unsigned int   AI2_LRV = 0x2850;
unsigned int   AI2_URV = 0xCAFA;
char AI_STATE = 0;

// Analog Output A/D numbers
unsigned int AO1_LRV = 0x30C0;  //12,288 (Checked from value to AO from AI)
unsigned int AO1_URV = 0xF400;  //62,464 (Checked from value to AO from AI)
unsigned int AO2_LRV = 0x30C0;  //12,288 (checked from value to AO from AI)
unsigned int AO2_URV = 0xF400;  //62,464 (checked from value to AO from AI)
char AO_MIRROR_STATE = 0;

// cellular modem flag
char CELLMODEM  = 1;

char PORTA_OUT = 0; // data to write to Parallel Port A (Byte-Wide Port)
char DI[10] = {0,0,0,0,0,0,0,0,0,0};  // data storage from digital inputs
float AI[3] = {0,0,0};                // Analog input values (percentages)
char toggle = 0;


const static char c_text[] = "Port-C ";
const static char d_text[] = "Port-D ";
const static char e_text[] = "Port-E ";

bbram char topics[10][75];
bbram char subscribetopics[10][75];
char receivedTopics[75];
char receivedmessage[256];
int stateCase;
int stateCase_Last;
int stateCaseTemp;
int stateCaseResetModem;
int ResetRetries;
unsigned int stateCaseMessage;
char LookingForTemp;
int rxTimeOuttimer;
int connTimeOuttimer;
int rxMQTT_TimeOuttimer;
unsigned long DigitalInputs;
unsigned long DigitalInputs_Last;
unsigned char ConnAck;
char PingResp;
char temperature[2];
unsigned char publishState;
unsigned char startPubOrSub;
int PingTimer;
char TimeToPing;
int WaitOnPingResp;
int MQTTRxData;
int	MQTTPort2Use;
char willFlag;
char willTopic[50];
char willMessage[50];
char username[50];
char password[50];
char willRetain;
char willQosLevel;
char cleanSession;
unsigned int keepAlivePeriod;
//char topic[100];
char qosLevel;
char dupFlag;
char retain;
bbram char clientID[75];
bbram char theTopic[100];
bbram char theMessage[200];
char tempMess[10];
unsigned int messageId;
unsigned int returnMessageId;
char returnMaxQoS;
char SubscribeAck;
char subLength;
char publishVariable;
int pubLength;
int receiveTopicLen;
char qosLevels[10];
bbram int Din_Invert[NUMBEROFDI + 1];
bbram int Din_Enable[NUMBEROFDI + 1];
int Din_Alarm_Timer[NUMBEROFDI + 1];
bbram int Din_Alarm_Delay[NUMBEROFDI + 1];
//bbram float Level_HLA_SP[NUMBEROFAI + 1];
//int High_Level_Timer[NUMBEROFAI + 1];
bbram int High_Level_Alarm_Delay[NUMBEROFAI + 1];
//bbram float Level_LLA_SP[NUMBEROFAI + 1];
//int Low_Level_Timer[NUMBEROFAI + 1];
bbram int Low_Level_Alarm_Delay[NUMBEROFAI + 1];
unsigned int publishCounter;
unsigned int subscribeCounter;
unsigned char tempHasBeenRead;
unsigned char receiveOK;
bbram int MQTT_Poll_Enable[10];
unsigned int MQTT_PollRate_Timer[10];
unsigned char timeToPublish[10];
bbram int MQTT_PollRate[10];
int receivePubIndexer;
struct tm rtc;
int NotConnectedToBrokerTimer;
int NotConnectedToBrokerTimeOut;
bbram int pwDay;
bbram long SecsSinceMidnight;
float residualTime;
float Filtered_Input[5];
bbram float tau;
int delay_to_next_step;
int MoveOnToCase5;
int carryOver;

const int DLST_Year[] = {2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020, 2021};
const int DLST_MarDay[] = {11, 10, 9, 8, 13, 12, 11, 10, 8, 14};
const int DLST_NovDay[] = {4, 3, 2, 1, 6, 5, 4, 3, 1, 7};
bbram char DLST_OneShot;

bbram long LastLoadTimeStamp;
//extern long dc_timestamp;	// date/time stamp at the end of the compile (seconds since Jan. 1 1980)
void ResetSoft(void);
void ResetHard(void);
void ScanIO(void);
void InitBrd();
void ReadDins();
void WriteDouts();
void DO_Mirror_DI();
void ReadAins();
void ScaleAins();
float Filter_Update(float xin, float dt, float tau, int i);
void WriteAouts(int channel, float percent);
void AO_Mirror_AI();
float Temperature_read();
void msDelay(unsigned int delay);

unsigned long CheckCallfors();
void MessageStateHandler();
void MonitorPublishItems();
void PingControl();
char* FloatToString(char *str, float f, char size);

void MQTT_ReceiveData(char ch);
void MQTTDriver();
void makeConnectString(void);
void makePublishString(char *pubTopic,char *pubMessage);
void makeSubscribeString();
void makePingString(void);
void makeDisconnectString(void);
void makePubAckString(void);
void makePubRecString(void);
void makePubRelString(void);
void ParseMessage(char *Topic, char *Message);

void MQTT_SendData(unsigned char* Data, unsigned int len);
char ResetModemLogic(unsigned char isConnected);
char ResetModem(void);
typedef union
{
    unsigned int uWord;
    struct
    {
        unsigned char lsb;
        unsigned char msb;
    } bytes;
} TUWord;

typedef union
{
    unsigned char i[4];
    float r; // 32 bit
} HRfloat;

unsigned char BNMCrcLsb[TOTAL_PORTS];
char BNRRoutePkt[TOTAL_PORTS];
unsigned int BNRFinalDst[TOTAL_PORTS];
unsigned int BNRSrc[TOTAL_PORTS], BNRSrcMsb[TOTAL_PORTS];
unsigned int TransactionID[TOTAL_PORTS];
unsigned int BNRPktLenMsb[TOTAL_PORTS];
unsigned int BNRByteCnt[TOTAL_PORTS];
unsigned int BNRPktLen[TOTAL_PORTS];
unsigned int BNMDstMSB[TOTAL_PORTS], BNMFinalDst[TOTAL_PORTS];
unsigned int BNMSrcMSB[TOTAL_PORTS], BNMSrc[TOTAL_PORTS];
unsigned int BNMDataGramLenMSB[TOTAL_PORTS], BNMDataGramLen[TOTAL_PORTS];
unsigned int BNMRespCmd[TOTAL_PORTS];
unsigned int BNMNumRegs[TOTAL_PORTS], BNMNumRegsCnt[TOTAL_PORTS];
int BNSDst[TOTAL_PORTS], BNSDstMsb[TOTAL_PORTS];
int BNSDataGramLen[TOTAL_PORTS], BNSDataGramLenMsb[TOTAL_PORTS];
int BNSReqBank[TOTAL_PORTS], BNSReqBankMsb[TOTAL_PORTS];
unsigned char MBMCmdReq[TOTAL_PORTS];
unsigned int MBMStartAddr[TOTAL_PORTS];
unsigned int RxState[TOTAL_PORTS], RxPrevState[TOTAL_PORTS];
unsigned char BNRxPrevByte[TOTAL_PORTS];
unsigned char BNMCrcMsb[TOTAL_PORTS];
int BNSCmdReq[TOTAL_PORTS];
unsigned int BNSStartAddr[TOTAL_PORTS];
bbram unsigned int UnitAddr[TOTAL_PORTS];
unsigned int BNSNumRegs[TOTAL_PORTS], BNSNumRegsCnt[TOTAL_PORTS], BNSByteCount[TOTAL_PORTS];
unsigned int BNSDataType[TOTAL_PORTS];
unsigned int SPktTmr[TOTAL_PORTS];
char SRespAvail[TOTAL_PORTS];
int SPktIndex[TOTAL_PORTS];
unsigned int SRespLen[TOTAL_PORTS];
unsigned int BNMByteCnt[TOTAL_PORTS], BNMTempCnt[TOTAL_PORTS];
char SerialPortProtocol[TOTAL_PORTS], SerialPortMaster[TOTAL_PORTS];
char SendViaRadio[TOTAL_PORTS], DebugEnable[TOTAL_PORTS];
char SendState[TOTAL_PORTS];
shared unsigned char ChannelClearTime[TOTAL_PORTS];
shared unsigned char LeadLagTimer[TOTAL_PORTS];
char OnDelay[TOTAL_PORTS], OffDelay[TOTAL_PORTS];
unsigned int BNMTranxID[TOTAL_PORTS];
bbram unsigned int RoutingTable[TOTAL_PORTS][MAXREPEATERROUTES][4]; // format: [i][MasterAddr, PollAddr, AltPollAddr, TransID]
typedef struct
{
    UINT8 Cmd; 			// 0=No action, 1=Ping, 2=Read, 3=Write
    int UnitNumber;		// Remote Unit Addr
    UINT8 DataType; 	//
    unsigned int BankID_Src; 	// Note: RT101 uses only 1,2,3,4 bank Ids
    unsigned int StartAddr_Src;
    unsigned int BankID_Dst; // Note: If Writing to ICL, bankID is not limited; however, Datatype
    unsigned int StartAddr_Dst; // must match the BankId you are writing to.
    unsigned int BlockSize;
    unsigned int PollRate; // Secs;
    unsigned int PollRate_Failed; // Secs; Probe Time
    unsigned int TimeLastPolledSecs;
    char FireAtStartup; 	//TRUE/FALSE
    char FireOnEdgeChange;	//TRUE/FALSE
    char EdgeBankID;		//Which Bank to monitor???
    int EdgeStartAddr;		//Starting from where ???
    int EdgeBlockSize;		//How many registers ???
    char IsDeltaValueAbsolute; // Is the delta value absolute or percent; TRUE for absolute
    float ConstDelta;		// 0 - No need to fire/ xx.xx - Delta value
    char DeltaBankID;		//Which Bank to monitor???
    int DeltaStartAddr;	//Starting from where ???
    int DeltaBlockSize;	//How many registers ???
    char EventNum;
    char SessionNum;
    char eActive;
    char sActive;
    char EdgeChanged;		// Change happened at specified block size
    char DeltaChanged;		// Change happened at specified block size
    char TimesUp;			// Timer timed out since last send

    unsigned int StartAddrHi;	// Modbus
    unsigned int StartAddrLo;	// Modbus
    unsigned int NumPtsHi;		// Modbus
    unsigned int NumPtsLo;		// Modbus
    unsigned int StartAddr;			// Modbus
} tComEvent;

typedef struct
{
    unsigned int Start;
    unsigned int End;
    unsigned char TotalEvents;
    unsigned char BankId;
    unsigned char EventNums[MAXEVENTS];
    unsigned char DebouceTimer;
    int DebouceRegNumber;
    char IsDeltaValueAbsolute;
    float Delta;
    char Enabled;
} tTrigBlock;

typedef struct
{
    tTrigBlock EdgeBlock[TOTAL_EDGE_BLOCKS];
    tTrigBlock DeltaBlock[TOTAL_DELTA_BLOCKS];
    int TotalEdgeBlocks;
    int TotalDeltaBlocks;
} tTriggers;

typedef struct
{
    unsigned int NumEvents;
    tComEvent Events[MAXEVENTS];
    tTriggers Triggers;
    char EventStartUp[MAXEVENTS];	//12/14/09
} tComSession;

tComSession ComSession[TOTAL_PORTS];
int CommEvent[TOTAL_PORTS];
int GoodResponse[TOTAL_PORTS], BadCRC[TOTAL_PORTS];
char MasterPacketReady[TOTAL_PORTS];
unsigned char SRespPkt[TOTAL_PORTS][BUFSIZE];
unsigned char SPkt[TOTAL_PORTS][BUFSIZE];
unsigned char MPkt[TOTAL_PORTS][BUFSIZE];
int Unique_RTU_Nums[TOTAL_PORTS][MAXEVENTS];   // Holds Unique RTU number's among the events
char Ping[TOTAL_PORTS][HIGHEST_EXPECTED_RTU_NUM];
unsigned int PingPollTimer[TOTAL_PORTS][HIGHEST_EXPECTED_RTU_NUM];
char RetryCounter[TOTAL_PORTS][HIGHEST_EXPECTED_RTU_NUM];
unsigned int ResponseTimer[TOTAL_PORTS];
unsigned int MPktLen[TOTAL_PORTS];
int Max_Rtu_Num[TOTAL_PORTS];
unsigned char ModbusMSecTmr[TOTAL_PORTS];
unsigned int SlaveAlgLkout[TOTAL_PORTS];
unsigned int MasterAlgLkout[TOTAL_PORTS];
char MasterState[TOTAL_PORTS];
unsigned int DataSize[TOTAL_PORTS];
TUWord crc_value;
HRfloat HR;
//unsigned int NumRoutesDefined;
TUWord U_Word;
bbram float Gals[TOTAL_STAT_REGS];
bbram unsigned long KGals[TOTAL_STAT_REGS], MGals[TOTAL_STAT_REGS];
bbram unsigned long MHours[TOTAL_STAT_REGS], KHours[TOTAL_STAT_REGS], Hours[TOTAL_STAT_REGS];
bbram float HRtimer[TOTAL_STAT_REGS];
char Ain_Sigfail[3]; // Ain SigFail Alarms
float  Ain_raw[3];
bbram unsigned long  Din_Starts[TOTAL_STAT_REGS], Din_KStarts[TOTAL_STAT_REGS]; // Added 1/5/01
bbram char Din_last[TOTAL_STAT_REGS]; // Added 1/5/01
bbram float AinScaleFactor[3], AoutScaleFactor[3];
int Ain_Sigfail_Timer[3], Ain_High_Timer[3], Ain_Low_Timer[3];
bbram float Ain_High_SP[3], Ain_Low_SP[3];
bbram long Ain_HLA_Timer_Preset[3], Ain_LLA_Timer_Preset[3];
char toggle;
unsigned long timesecs;
float msFraction;
unsigned char DACIn[16];
unsigned char mainBuffer[512];
int TotalReadFromPort, pqr;
char ActivePortNumber;

void BricknetReceiver( int, unsigned char ) ;

unsigned int PackData(int PortNum, unsigned int index, UINT8 BNSDataType1, unsigned int BankID, unsigned int Addr, unsigned int BNSNumRegs1);
int AddBricknetEvent(int PortNum,
                     UINT8 cmd, int unitnumber, UINT8 datatype,
                     unsigned int bankid_src,  unsigned int startaddr_src,
                     unsigned int bankid_dst,  unsigned int startaddr_dst,
                     unsigned int blocksize, unsigned int pollrate, unsigned int pollratefailed,
                     char FireAtStartup, char FireOnEdgeChange, char EBankID,
                     int ESAddr, int EBSize, char IsDeltaValueAbsolute, float ConstDelta, char DBankID,
                     int DSAddr, int DBSize, char eventNum, char sessionNum
                    );
int EdgeTrigger(int, int, int, int, int);
int DeltaTrigger(int, int, int, int, int, float, char);
void PrepareCmdPacket(int, int);
void CreateBNMPacket(int, UINT8 command); // BN Packet Create and Send ; Ret TRUE if errors
void AddBricknetEvents(int);
void DoEvents(int PortNum);
void BricknetMaster(int PortNum);
void SessionActivateDeactivate( char, char, char );
unsigned char CalculateLRC( unsigned char *msg, unsigned int len);
unsigned char ByteFromASCII(char *ascii);
void ByteToASCII(char *str, unsigned char inbyte);
void CreateMBASCIIPacket(int PortNum, int EventNo);
void ModbusASCIIMasterReceiver(int PortNum, char Data, int EventNo);
void ModbusASCIISlave(int, unsigned char );
void ModbusRTUSlave(int, char );
void CreateMBRTUPacket(int, int EventNo);
void ModbusRTUMasterReceiver(int, char Data, int EventNo);
int AddModbusEvent(int, UINT8, int, unsigned int,  unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int );
void ModbusMaster(int);
void AddModbusEvents(int);

void TotalizeAins(void);
void TotalizeDins(void);
unsigned int CalculateCRC( unsigned char *msg, unsigned int len);
void CommFailure(void);
void MPCT(void);

#define Y 3   // Used for stdio window offset

int isr_a;
nodebug root interrupt void TimerA_ISR()
{
    RdPortI(TACSR);
    if( --scaler_a <= 0 )
    {
        scaler_a = RELOAD_ISR_A;
        for(isr_a = 0; isr_a < TOTAL_PORTS; isr_a++)
            if( SendViaRadio[isr_a] )
            {
                if(SendState[isr_a] == 1)
                    ChannelClearTime[isr_a]++;
                else if(SendState[isr_a] == 3 || SendState[isr_a] == 5)
                    LeadLagTimer[isr_a]++;
            }
    }
}

int isr_b;
nodebug root interrupt void TimerB_ISR()
{
    RdPortI(TBCSR);
    scaler_b = scaler_b - 1;
    if( scaler_b <= 0 )
    {
        for(isr_b = 0; isr_b < TOTAL_PORTS; isr_b++)
        {
            if( SerialPortProtocol[isr_b] == BRICKNET || SerialPortProtocol[isr_b] == MODBUS )
            {
                if( SRespAvail[isr_b] || MasterPacketReady[isr_b] || BNRRoutePkt[isr_b] )
                {
                    if( !SendState[isr_b] )
                        SendState[isr_b] = SendViaRadio[isr_b] ? 1 : 3;

                    if(SendState[isr_b] == 1 && ChannelClearTime[isr_b] >= 75)
                    {
                        ChannelClearTime[isr_b] = 0;
                        SendState[isr_b] = 2;
                    }
                    else if(SendState[isr_b] == 2)
                    {
                        if(isr_b == 0) BitWrPortI(PDDR, &PDDRShadow, 0, 4);
                        else BitWrPortI(PGDR, &PGDRShadow, 0, 0);
                        LeadLagTimer[isr_b] = 0;
                        SendState[isr_b] = 3;
                    }
                    else if(SendState[isr_b] == 3 && (!SendViaRadio[isr_b] || (LeadLagTimer[isr_b] >= OnDelay[isr_b])))
                    {
                        if(SRespLen[isr_b] > 0 && SRespAvail[isr_b])
                        {
                            if(isr_b == 0) serCwrite(SRespPkt[isr_b], SRespLen[isr_b]);
                            else serDwrite(SRespPkt[isr_b], SRespLen[isr_b]);
                            SRespLen[isr_b] = 0;
                            SlaveAlgLkout[isr_b] = 0;
                        }
                        else if(BNRRoutePkt[isr_b] && BNRPktLen[isr_b] > 0)
                        {
                            if(isr_b == 0) serCwrite(SPkt[isr_b], BNRPktLen[isr_b]);
                            else serDwrite(SPkt[isr_b], BNRPktLen[isr_b]);
                            BNRPktLen[isr_b] = 0;
                            MasterAlgLkout[isr_b] = 0;
                            SlaveAlgLkout[isr_b] = 0;
                        }
                        else if(MPktLen[isr_b] > 0 && MasterPacketReady[isr_b])
                        {
                            if(isr_b == 0) serCwrite(MPkt[isr_b], MPktLen[isr_b]);
                            else serDwrite(MPkt[isr_b], MPktLen[isr_b]);
                            MPktLen[isr_b] = 0;
                            MasterAlgLkout[isr_b] = 0;
                        }
                        RxState[isr_b] = 0;
                        SendState[isr_b] = 4;
                    }
                    else if(SendState[isr_b] == 4 && ( (isr_b == 0 && serCwrFree() == COUTBUFSIZE) || (isr_b == 1 && serDwrFree() == DOUTBUFSIZE) ))
                    {
                        SendState[isr_b] = 5;
                        if( SendViaRadio[isr_b] )
                            LeadLagTimer[isr_b] = 0;
                    }

                    if(SendState[isr_b] == 5 && (!SendViaRadio[isr_b] || (LeadLagTimer[isr_b] >= OffDelay[isr_b])))
                    {
                        if( SendViaRadio[isr_b] )
                        {
                            if(isr_b == 0) BitWrPortI(PDDR, &PDDRShadow, 1, 4);
                            else BitWrPortI(PGDR, &PGDRShadow, 1, 0);
                        }

                        SendState[isr_b] = 0;

                        if(SRespAvail[isr_b])
                            SRespAvail[isr_b] = FALSE;
                        else if(BNRRoutePkt[isr_b])
                            BNRRoutePkt[isr_b] = FALSE;
                        else if(MasterPacketReady[isr_b])
                        {
                            MasterPacketReady[isr_b] = FALSE;
                            ResponseTimer[isr_b] = 0;
                        }
                    }
                }
            }
        }
        scaler_b = RELOAD_ISR_B;
    }
    WrPortI(TBL1R, NULL, 0x00);
    WrPortI(TBM1R, NULL, 0x00);
}


void ResetPort(int);

unsigned long pf, sr, wdto;


unsigned long piAin, piAout, pAin, pAout, pDin, pDout, pTAin, pTAout, pTDin, pTDout; // physical memory address

nodebug root int iAinRead(int index)
{
    if( index >= BANKSIZE ) return -1;
    return xgetint(piAin + (index * 2));
}

nodebug root void iAinWrite(int index, int val)
{
    if( index >= BANKSIZE ) return;
    xsetint((piAin + (index * 2)), val);
}

nodebug root int iAoutRead(int index)
{
    if( index >= BANKSIZE ) return -1;
    return xgetint(piAout + (index * 2));
}

nodebug root void iAoutWrite(int index, int val)
{
    if( index >= BANKSIZE ) return;
    xsetint((piAout + (index * 2)), val);
}

nodebug root float AinRead(int index)
{
    if( index >= BANKSIZE ) return -1;
    return xgetfloat(pAin + (index * 4));
}

nodebug root float AinWrite(int index, float val)
{
    if( index >= BANKSIZE ) return -1;
    xsetfloat((pAin + (index * 4)), val);
}

nodebug root float AoutRead(int index)
{
    if( index >= BANKSIZE ) return -1;
    return xgetfloat(pAout + (index * 4));
}

nodebug root float AoutWrite(int index, float val)
{
    if( index >= BANKSIZE ) return -1;
    xsetfloat((pAout + (index * 4)), val);
}

nodebug root char DinRead(int index)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    if(!xmem2root(buffer, (pDin + index), 1))
        return ((char)buffer[0]);
}

nodebug root char DinWrite(int index, char val)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    buffer[0] = val;
    root2xmem((pDin + index), buffer, 1);
}

nodebug root char DoutRead(int index)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    if(!xmem2root(buffer, (pDout + index), 1))
        return ((char)buffer[0]);
}

nodebug root char DoutWrite(int index, char val)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    buffer[0] = val;
    root2xmem((pDout + index), buffer, 1);
}
///////////////////////////////////////////
nodebug root float TAinRead(int index)
{
    if( index >= BANKSIZE ) return -1;
    return xgetfloat(pTAin + (index * 4));
}

nodebug root float TAinWrite(int index, float val)
{
    if( index >= BANKSIZE ) return -1;
    xsetfloat((pTAin + (index * 4)), val);
}

nodebug root float TAoutRead(int index)
{
    if( index >= BANKSIZE ) return -1;
    return xgetfloat(pTAout + (index * 4));
}

nodebug root float TAoutWrite(int index, float val)
{
    if( index >= BANKSIZE ) return -1;
    xsetfloat((pTAout + (index * 4)), val);
}

nodebug root char TDinRead(int index)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    if(!xmem2root(buffer, (pTDin + index), 1))
        return ((char)buffer[0]);
}

nodebug root char TDinWrite(int index, char val)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    buffer[0] = val;
    root2xmem((pTDin + index), buffer, 1);
}

nodebug root char TDoutRead(int index)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    if(!xmem2root(buffer, (pTDout + index), 1))
        return ((char)buffer[0]);
}

nodebug root char TDoutWrite(int index, char val)
{
    char buffer[1];
    if( index >= BANKSIZE ) return -1;
    buffer[0] = val;
    root2xmem((pTDout + index), buffer, 1);
}

nodebug void Set_TimerA()
{
#if __SEPARATE_INST_DATA__
    interrupt_vector timera_intvec TimerA_ISR;
#else
    SetVectIntern(0x0A, TimerA_ISR);
    SetVectIntern(0x0A, GetVectIntern(0xA));   // re-setup ISR to show example of retrieving ISR address
#endif
    WrPortI(TAT1R, &TAT1RShadow, 244);   // divisor = 245
    scaler_a = RELOAD_ISR_A;
    WrPortI(TACSR, &TACSRShadow, 0x03); // enable A1 int
    WrPortI(TACR, &TACRShadow, 0x01);   // int priority 2 uses pclk/2
}

nodebug void Set_TimerB()
{
    scaler_b = RELOAD_ISR_B;

#if __SEPARATE_INST_DATA__
    interrupt_vector timerb_intvec TimerB_ISR;
#else
    SetVectIntern(0x0B, TimerB_ISR);            // set up ISR
    SetVectIntern(0x0B, GetVectIntern(0xB));   // re-setup ISR to show example of retrieving ISR address
#endif

    WrPortI(TBCR, &TBCRShadow, 0x09);// clock timer B with (perclk/16) and set interrupt level to 1
    WrPortI(TBL1R, NULL, 0x00);// set initial match!
    WrPortI(TBM1R, NULL, 0x00);
    WrPortI(TBCSR, &TBCSRShadow, 0x03);   // enable timer B and B1 match interrupts
}

int CTU_Comm_Fail_Timer;
bbram char tCTU_Comm_Fail;
void CTU_Comm_Fail();

main()
{
    unsigned long t0;
    int state;
    int port, loop, i, j, wd;

    state = 1;

    InitBrd();   // DAVID'S BOARD INITIALIZATION

    SPIinit();  // initialize SPI routines

    // OPEN SERIAL PORTS
    //serCopen(PORT1_BAUD_RATE);     //RS232 #1
//   serCflowcontrolOn();      //enable flow control
    //serCwrFlush();
    //serCrdFlush();

    //serDopen(PORT2_BAUD_RATE);     //RS232 #2
    //serDflowcontrolOn();      //enable flow control
    //serDwrFlush();
    //serDrdFlush();

    serEopen(PORT3_BAUD_RATE);     //RS485
    serEwrFlush();
    serErdFlush();

    if(CELLMODEM)  // ZERO BY DEFAULT - must be manually set to 1 for cellular operation
    {
        serFopen(PORT4_BAUD_RATE);     //INTERNAL CELLULAR MODEM OPTION
        serFwrFlush();
        serFrdFlush();
    }
    if(LastLoadTimeStamp != dc_timestamp)
        ResetHard();

    ResetSoft();

    Set_TimerA();
    Set_TimerB();

    while (1)
    {
        switch(state)
        {
        // turn LED on/off every 1/2 second

        case 1:
            t0 = MS_TIMER;                 // Intialize timer
            ++state;                       // Next state
            break;
        case 2:
            if (MS_TIMER - t0 > 500)     // Check timer
            {
                //ReadDins();                // tested and working 8/15/18
                //DO_Mirror_DI();           // tested and working 8/15/18
                //WriteDouts();
                //ReadAins();
                //ScaleAins();
                // tested and working 8/18/18
                //AO_Mirror_AI();           // tested and working 8/19/18

                Deg_F = Temperature_read();
//               printf("Temperature:%.1f \n\r",Deg_F); //TEST

                ++state;                   // Next state if expired
            }
            break;
        case 3:

            // PROGRAM RUNNGING LED
            toggle = !toggle;
            //if(toggle && DI[9])   // stop blinking if battery low (TEST)
            if(toggle)   // stop blinking if battery low (TEST)
            {
                PORTA_OUT |= 0x80;                         // SET LED BIT ON
                WrPortI(PADR, &PADRShadow, PORTA_OUT);
            }
            else
            {
                PORTA_OUT &= 0x7f;                         // SET LED BIT OFF
                WrPortI(PADR, &PADRShadow, PORTA_OUT);
            }

            t0 = MS_TIMER;              // Intialize timer
            ++state;                     // Next state
            break;
        case 4:
            if (MS_TIMER - t0 > 500)    // Check timer
            {
                ScanIO();
                ++timesecs;
                state = 1;         // back to first state if expired

                for(port = 0; port < TOTAL_PORTS ; port++)
                {
                    if( SerialPortProtocol[port] == BRICKNET && SerialPortMaster[port] )
                    {
                        for(j = 0; j < Max_Rtu_Num[port]; j++)
                        {
                            if(Ping[port][Unique_RTU_Nums[port][j]])
                            {
                                PingPollTimer[port][Unique_RTU_Nums[port][j]]++;
                                if(PingPollTimer[port][Unique_RTU_Nums[port][j]] >= 65535)
                                    PingPollTimer[port][Unique_RTU_Nums[port][j]] = 0;
                            }
                            else
                                PingPollTimer[port][Unique_RTU_Nums[port][j]] = 0;
                        }
                        DoEvents(port);
                    }

                    if( SerialPortProtocol[port] == MODBUS && SerialPortMaster[port] )
                    {
                        if(++ModbusMSecTmr[port] >= 255) ModbusMSecTmr[port] = 0;
                    }

                    if( SerialPortProtocol[port] == BRICKNET || SerialPortProtocol[port] == MODBUS )
                    {
                        SPktTmr[port]++;
                        if(SPktTmr[port] >= 65535) SPktTmr[port] = 0;
                    }

                    if( SerialPortMaster[port] && MasterState[port] == 1 )
                    {
                        ResponseTimer[port]++;
                        if(ResponseTimer[port] >= 65535) ResponseTimer[port] = 0;
                    }

                    if( RxPrevState[port] == RxState[port] )
                    {
                        if( RxState[port] != 0 && SPktTmr[port] >= RECV_STATE_LOCKOUT )
                        {
                            SPktTmr[port] = 0;
                            RxState[port] = 0;
                            SendState[port] = 0;
                            SRespAvail[port] = FALSE;
                            BNRRoutePkt[port] = FALSE;
                            if(port == 0) BitWrPortI(PDDR, &PDDRShadow, 1, 4);
                            else BitWrPortI(PGDR, &PGDRShadow, 1, 0);
                            if( DebugEnable[port] )
                                printf("\r\n**********");
                        }
                    }
                    else
                        SPktTmr[port] = 0;
                }
            }
            break;
        default:
        }
        for(port = 0; port < TOTAL_PORTS; port++)
        {
            if( SerialPortProtocol[port] == BRICKNET || SerialPortProtocol[port] == MODBUS )
            {
                if( port ==  0 ) TotalReadFromPort = serCread(mainBuffer, serCrdUsed(), 2);
                if( port ==  1 ) TotalReadFromPort = serDread(mainBuffer, serDrdUsed(), 2);

                for( pqr = 0; pqr < TotalReadFromPort; pqr++ )
                {
                    if( SerialPortProtocol[port] == BRICKNET )
                        BricknetReceiver(port, mainBuffer[pqr]);
                    else if( SerialPortProtocol[port] == MODBUS )
                    {
                        if( SerialPortMaster[port] )
#if MODBUS_PROTOCOL == MODBUS_ASCII
                            ModbusASCIIMasterReceiver(port, mainBuffer[pqr], CommEvent[port]);
#else
                            ModbusRTUMasterReceiver(port, mainBuffer[pqr], CommEvent[port]);
#endif
                        else
#if MODBUS_PROTOCOL == MODBUS_ASCII
                            ModbusASCIISlave(port, mainBuffer[pqr]);
#else
                            ModbusRTUSlave(port, mainBuffer[pqr]);
#endif
                    }
                }

                if( SerialPortMaster[port] && ComSession[port].NumEvents )
                {
                    if( SerialPortProtocol[port] == BRICKNET )
                        BricknetMaster(port);
                    else if( SerialPortProtocol[port] == MODBUS)
                        ModbusMaster(port);
                }
            }
        }

        MQTTDriver();
    }
}

void ReadDins()
{
    // Digital Inputs  (tested and working 8/15/18)
    // 1-8: DI[1] - DI[8] from demultiplexer MC74HC4051A
    // 9:   DI[9] - from PE0 (BAT_OK)
    // PORTA_OUT - Global storage for Port-A output data (bits 0-8 only)

    char i;

    // READ DIGITAL INPUTS 1-8
    for(i=0; i<8; i++)
    {
        //set address 0-7 (don't change any other bits in PORTA_OUT)
        if(i & 0x01) PORTA_OUT |= 0x01;
        else PORTA_OUT &=0xFE;   //A0
        if(i & 0x02) PORTA_OUT |= 0x02;
        else PORTA_OUT &=0xFD;   //A1
        if(i & 0x04) PORTA_OUT |= 0x04;
        else PORTA_OUT &=0xFB;   //A2
        WrPortI(PADR,&PADRShadow,PORTA_OUT);  // Send address to port A

        //read data bit from selected input into DI[1] - [8]
        DI[i+1] = BitRdPortI(PBDR,6);    //PB6
        DinWrite(i+1,DI[i+1]);
    }

    // BATTERY OK INPUT
    DI[9] = BitRdPortI(PEDR,0);    // battery failure = 0
    DinWrite(9,DI[9]);
}

void DO_Mirror_DI()
{
    char i;

    // Test Routine:   (tested and working 8/15/18)
    if(DI[1] || DI[5]) DO1_HIGH;
    else DO1_LOW;
    if(DI[2] || DI[6]) DO2_HIGH;
    else DO2_LOW;
    if(DI[3] || DI[7]) DO3_HIGH;
    else DO3_LOW;
    if(DI[4] || DI[8]) DO4_HIGH;
    else DO4_LOW;

}

void WriteDouts()
{
    if(DoutRead(1)) DO1_HIGH;
    else DO1_LOW;
    if(DoutRead(2)) DO2_HIGH;
    else DO2_LOW;
    if(DoutRead(3)) DO3_HIGH;
    else DO3_LOW;
    if(DoutRead(4)) DO4_HIGH;
    else DO4_LOW;
}

void ReadAins()
{
    // TESTED AND WORKING 8/18/18
    // CHIP: LINEAR LTC1865 16-BIT A/D converter
    // global float storage... AI[0]:ignore  AI[1]:Channel-1 Reading   AI[2]:Channel-2 Reading (percentages)

    // Currently reads one channel every second (first reading is ignored)

    char i, j, data_out[2], raw[2];
    unsigned int raw_int1, raw_int2;

    data_out[1] = 0x00;  // LSB always zero

    switch ( AI_STATE )
    {
    case 0:  // first read (ignore reading)
        data_out[0] = 0x80;  // set MSB address for chan 1

        // TAKE CONV. LINE LOW TO READ
        BitWrPortI(PEDR, &PEDRShadow, 0, 1);
        for(j=0; j<5; j++);                  //delay

        // send address and receive data from last address (full duplex)
        SPIWrRd(data_out, raw, 2);          // Write/Read two bytes
        raw[0] = 0;
        raw[1] = 0;

        // TAKE CONV. LINE HIGH TO CONVERT NEXT CHANNEL AND GO TO SLEEP
        BitWrPortI(PEDR, &PEDRShadow, 1, 1);
        for(j=0; j<5; j++);                  //delay

        AI_STATE = 1;
        break;
    case 1:  //READ FIRST INPUT
        data_out[0] = 0xC0;  // set MSB address for chan 2   1100 0000 (next conversion)
        raw[0] = 0;
        raw[1] = 0;

        // TAKE CONV. LINE LOW TO READ
        BitWrPortI(PEDR, &PEDRShadow, 0, 1);
        for(j=0; j<5; j++);                  //delay

        // send address and receive data from last address (full duplex)
        SPIWrRd(data_out, raw, 2);          // Write/Read two bytes

        // TAKE CONV. LINE HIGH TO CONVERT NEXT CHANNEL AND GO TO SLEEP
        BitWrPortI(PEDR, &PEDRShadow, 1, 1);
        for(j=0; j<5; j++);                  //delay

        // combine raw bytes into integer storage
        raw_int1  = ((unsigned int)(raw[0]) << 8) | raw[1];

        // SCALE 16-BIT RAW READINGS (chan 1)to PERCENTAGE
        AI[1] = ((float)(raw_int1) - AI1_LRV)/(AI1_URV - AI1_LRV) * 100.0 + 0.01;
        //printf("AI1 Raw:%u  PRCT:%.2f \n\r",raw_int1, AI[1]);

        data_out[0] = 0x80;  // set MSB address for chan 1  1000 0000 (next conversion)
        raw[0] = 0;
        raw[1] = 0;

        // TAKE CONV. LINE LOW TO READ
        BitWrPortI(PEDR, &PEDRShadow, 0, 1);
        for(j=0; j<5; j++);                  //delay

        // send address and receive data from last address (full duplex)
        SPIWrRd(data_out, raw, 2);          // Write/Read two bytes

        // TAKE CONV. LINE HIGH TO CONVERT NEXT CHANNEL AND GO TO SLEEP
        BitWrPortI(PEDR, &PEDRShadow, 1, 1);
        for(j=0; j<5; j++);                  //delay

        // combine raw bytes into integer storage
        raw_int2  = ((unsigned int)(raw[0]) << 8) | raw[1];

        // SCALE 16-BIT RAW READINGS (chan 2) to PERCENTAGE
        AI[2] = ((float)(raw_int2) - AI2_LRV)/(AI2_URV - AI2_LRV) * 100.0 + 0.01;
        //printf("AI2 Raw:%u  PRCT:%.2f \n\r",raw_int2, AI[2]);

        break;
    } // end switch statement
}

nodebug void ScaleAins()
{
    int i;
    for(i = 1; i < 3; i++)
    {
        // Scale Analog inputs 0-100.0 (floats)
        AI[i] =  AI[i] * AinScaleFactor[i] * 0.01;
        AinWrite(i, Filter_Update(AI[i],1.0,tau,i));
        //printf("AI[%d]:%.2f AinRead:%.2f\r\n",i,AI[i],AinRead(i));
        // range of -25 to 100 (0-20ma after calibration)
        Ain_Sigfail[i] = (AinRead(i) <=  (-0.15 * AinScaleFactor[i]) );
        // Keep Ain >=0.0
        //AinWrite(i, (AinRead(i) < 0.0) ? 0.0 : AinRead(i));
    }
}
nodebug float Filter_Update(float xin,float dt, float tau, int i)
{
    if(tau > 0)
        Filtered_Input[i] += (xin - Filtered_Input[i])*(1.0 - exp(-dt/tau));

    return Filtered_Input[i];
}
void WriteAouts(int channel, float percent)
{
    //SPI routine for LTC1655 16-bit A/D converter
    //SPI using serial port A
    //Input Range(value):  -12 to 105.5 percent (about 2 to 20.8ma)
    //Output Range: 4ma = 12288 and 20ma = 62,464 using 19.6K ohm resistor)
    // Be careful sending data to portA which is a byte-wide port.  Only change bits needed for this routine.

    char data[2];  // MSB=[0]   LSB=[1]
    unsigned int range1, range2;
    unsigned int AO_value;

    range1 = AO1_URV - AO1_LRV;
    range2 = AO2_URV - AO2_LRV;

    // limits output range from 2 to 21ma (if LRV and URV change, must re-evaluate these numbers)
    if(percent > 105.5)
        percent = 105.5;
    if(percent < -12)
        percent = -12;

    if(channel == 1)
        AO_value = (unsigned int)(((percent * 0.01) * range1) + AO1_LRV);
    else
        AO_value = (unsigned int)(((percent * 0.01) * range2) + AO2_LRV);

    // Extra limit on AO value to FF00
    if(AO_value > 65280)
        AO_value = 65280;

    data[0] = (AO_value >> 8) & 0xFF; //MSB
    data[1] =  AO_value & 0xff;       //LSB

//   printf("AO#:%u PRCT:%.2f AO_value:%4x MSB:%2x LSB:%2x \n\r",channel, percent, AO_value, data[0], data[1]); //TEST

    // Chip Select Control
    if(channel == 1)  // AOUT-1
        PORTA_OUT &= 0xEF; // Set PA4 LOW (Enable data input)
    else              // AOUT-2
        PORTA_OUT &= 0xDF; // Set PA5 LOW (Enable data input)
    WrPortI(PADR,&PADRShadow,PORTA_OUT);  // Set port A bits

    // SPI routine to send 16-bits of data to LTC1655 chip
    SPIWrite(data,2);

    // Take Both Aout1 and Aout2 /CS's HIGH
    PORTA_OUT |= 0x30; // Set PA4 and PA5 HIGH (set data into output)
    WrPortI(PADR,&PADRShadow,PORTA_OUT);  // Set port A bits

}

void AO_Mirror_AI()
{
    // Test program to send Analog input percentages to Analog Outputs

    switch (AO_MIRROR_STATE)
    {
    case 0:
        WriteAouts(1,AI[1]);
        AO_MIRROR_STATE = 1;
        break;
    case 1:
        WriteAouts(2,AI[2]);
        AO_MIRROR_STATE = 0;
        break;
    }
}

float Temperature_read()
{
    int i;
    char inv, msb;
    char Deg_input[3];
    char Deg_command[3];
    float x,y, cent, fahr;

    /*********************************************************
         RTU101B Processor pins:  CE= PB7
         Using standard SPI routines on Serial Port A.
         Chip: DS1722S+ (-55 to +120 deg C. ---> -67 to +248 deg F.)
         Chip Select (CE) is high to enable.
         Chip outputs Centigrade in 2's complement form.
         This routine converts and returns Fahrenheit.

         Data is input on the falling edge of the SCLK, MSB first.
         Data is output on the rising edge of the SCLK.

         Configuration: read address 00h, write address 80h
         Temperature LSB: read address 01h
         Temperature MSB: read address 02h
    ************************************************************/

// ***** configure device (9-bits and read continuously *****
    SPImode(0);

    TEMP_CE_HIGH;
    for(i=0; i<20; i++); // delay

    Deg_command[0] = 0x80; // Configuration register write address
    Deg_command[1] = 0xE2; // configuration data
    SPIWrite(Deg_command,2);
    TEMP_CE_LOW;
    for(i=0; i<20; i++); // delay

    // Read Temperature data into Deg_input[]
    TEMP_CE_HIGH;  // START
    for(i=0; i<20; i++); // delay

    Deg_command[0] = 0x01;   // set temperature LSB read address
    SPIWrite(Deg_command,1); // set read address of temperature LSB
    SPIRead(Deg_input,2);    // LSB and MSB Temperature data
    TEMP_CE_LOW; // END

    SPImode(1);

    // Deg_input[0] is the deg. C. LSB
    // Deg_input[1] is the deg. C. MSB

    //convert from 2's complement
    if(Deg_input[1] < 0x79)             // 0 to 120 degrees C
    {
        cent = Deg_input[1];
        if(Deg_input[0]==0x80)          //0.5 degree resolution (default)
            cent += 0.5;
        else if(Deg_input[0]==0x20)     //0.125 degree resolution
            cent += 0.125;
        else if(Deg_input[0] ==0x10)    //0.625 degree resolution
            cent += 0.0625;
    }
    else                            // convert 2's complement
    {
        msb = Deg_input[1];
        inv = ~msb;                       // invert number
        inv += 0x01;              // add 1 to number
        y = inv*2;
        x  = inv - y;                         // make negative
        cent = x;                         // convert to float
        if(Deg_input[0] == 0x80)        //.5 resolution
            cent += 0.5;
        else if(Deg_input[0] == 0x40) //.25 resolution
            cent += 0.75;
        else if(Deg_input[0] == 0xE0)   //.125 resolution
            cent += 0.875;
        else if(Deg_input[0] == 0xF0)   //.0625 resolution
            cent += 0.9375;
    }
    //convert to degrees Fahrenheit
    fahr = (cent*1.8)+32;

    return fahr;
}


void InitBrd()
{
    // RCM6760 I/O PORTS GLOBAL INITIALIZATION

    // Temperature Chip Parallel Port pins Used:
    // PC6 = (MOSI)  Master-Out / Serial-In (processor output)
    // PC7 = (MISO)  Master-In  / Serial-Out (processor input)
    // PB1 = SPI CLK
    // PB7 = CE (Chip Enable)

    //********************* CONFIGURE PARALLEL PORT A ***************************
    // Configure Port A as a byte-wide OUTPUT port
    // PA0: A0 PA1:A1 PA2:A2 PA3:SPARE
    // PA4: AOUT1_/CS PA5: AOUT2_/CS PA6: RS485 /RE  PA7: Prog. Running LED
    WrPortI(SPCR, &SPCRShadow, 0x84);   // 1000 0000  (bytewide output port)
    WrPortI(PADR, &PADRShadow, 0x70);   // 0111 0000 (outputs 4-6 HIGH)

    //********************* CONFIGURE PARALLEL PORT B ***************************
    // PB0: not used  PB1: CLK-A (SPI clk) PB2: DO-1 PB3: DO-2
    // PB4: DO-3 PB5: DO-4 PB6:MUX-IN PB7: Temperature CE line
    WrPortI(PBDDR, &PBDDRShadow, 0xBe);   // 1011 1110 (1=OUT)

    // Port B Data Register; initialize all outputs LOW
    WrPortI(PBDR, &PBDRShadow, 0x00);     // 0000 0000

    //********************* CONFIGURE PARALLEL PORT C ***************************
    // PC0: TX-D  PC1: RX-D  PC2: TX-C  PC3: RX-C
    // PC4: RTS-C PC5: RTS-D PC6: TX-A (mosi)  PC7: RX-A (miso)

    // Port C Data Direction Register:
    WrPortI(PCDDR, &PCDDRShadow, 0x75);   // 0111 0101 (1=OUT)

    // Port C Data Register: take all outputs LOW
    WrPortI(PCDR, &PCDRShadow, 0x00);   //0000 0000

    //********************* CONFIGURE PARALLEL PORT D ***************************
    // PD0: CTS-F PD1: RTS-F PD2: TX-F PD3: RX-F   (TX/RX-F: Cell Modem port)
    // PD4 - PD7: not available

    // Port D Drive Control Register: driven high/low
    WrPortI(PDDCR, &PDDCRShadow, 0x00);

    // Port D Function Register; normal functioning
    WrPortI(PDFR, &PDFRShadow, 0x00);

    //Port D Data Direction Register:
    WrPortI(PDDDR, &PDDDRShadow, 0x06);  // 0000 0110  (1=OUT)

    // Port D Data Register: Take all outputs LOW.
    WrPortI(PDDR, &PDDRShadow, 0x00);


    //********************* CONFIGURE PARALLEL PORT E ***************************
    // PE0: Low Battery PE1: AI_CONVERT  PE2: CTS-C PE3: CTS-D
    // PE4: not avail PE5: spare PE6:TX-E PE7: RX-E    (TX/RX-E: RS485 port)

    // INPUTS: 0,2,3,7  OUTPUTS: 1,5,6
    // Port E Data Direction Register (1=OUTPUT)
    WrPortI(PEDDR, &PEDDRShadow, 0x62);   // 0110 0010

    //Initialize all output bits on Port E
    WrPortI(PEDR, &PEDRShadow, 0x02);     // 0000 0010

}  // END OF InitBrd()

nodebug void ScanIO()
{
    MonitorPublishItems();
    PingControl();
    ReadDins();
    WriteDouts();
    ReadAins();
    ScaleAins();		
}
nodebug void ResetSoft()
{
    int ResetWatchDog;                // ID for a virtual watchdog
    int i, j;

    ResetWatchDog = VdGetFreeWd(255);

    printf("\nGoing through Soft Reset");
    //printf("\r\nTotal amount of XRAM memory left for allocation: %ld bytes.", AmountOf_XRAMLeft());

    printf("\r\nAllocating Ain, Aout, Din, and Dout banks...");
    piAin = xalloc(BANKSIZE * sizeof(int));      // physical memory address (SRAM)
    piAout = xalloc(BANKSIZE * sizeof(int));      // physical memory address (SRAM)
    pAin = xalloc(BANKSIZE * sizeof(float));      // physical memory address (SRAM)
    pAout = xalloc(BANKSIZE * sizeof(float));      // physical memory address (SRAM)
    pDin = xalloc(BANKSIZE * sizeof(char));         // physical memory address (SRAM)
    pDout = xalloc(BANKSIZE * sizeof(char));      // physical memory address (SRAM)

    VdHitWd(ResetWatchDog);        // Hit watch dog

    if( !piAin || !piAout || !pAin || !pAout || !pDin || !pDout )
    {
        printf("\r\nFailed to allocate memory for %s bank. Exiting the program....", !pAin ? "AIN" : !pAout ? "AOUT" : !pDin ? "DIN" : "DOUT");
        exit(1);
    }
    printf("\r\nInitializing banks to ZERO ...");
    // Reset data on Soft reset
    for( i = 0; i < BANKSIZE; i++ )
    {
        DinWrite(i, 0);
        DoutWrite(i, 0);
        AinWrite(i, 0.00);
        AoutWrite(i, 0.00);
        iAinWrite(i, 0);
        iAoutWrite(i, 0);
        VdHitWd(ResetWatchDog);        // Hit watch dog
    }

    VdHitWd(ResetWatchDog);        // Hit watch dog

    printf("\r\nAllocating TAin, TAout, TDin, and TDout banks...");
    pTAin = xalloc(BANKSIZE * sizeof(float));      // physical memory address (SRAM)
    pTAout = xalloc(BANKSIZE * sizeof(float));      // physical memory address (SRAM)
    pTDin = xalloc(BANKSIZE * sizeof(char));      // physical memory address (SRAM)
    pTDout = xalloc(BANKSIZE * sizeof(char));      // physical memory address (SRAM)

    if( !pTAin || !pTAout || !pTDin || !pTDout )
    {
        printf("\r\nFailed to allocate memory for %s bank. Exiting the program....", !pTAin ? "TAIN" : !pTAout ? "TAOUT" : !pTDin ? "TDIN" : "TDOUT");
        exit(1);
    }

    VdHitWd(ResetWatchDog);        // Hit watch dog

    printf("\r\nInitializing T banks to ZERO ...");
    // Reset data on Soft reset
    for( i = 0; i < BANKSIZE; i++ )
    {
        TDinWrite(i, 0);
        TDoutWrite(i, 0);
        TAinWrite(i, 0.00);
        TAoutWrite(i, 0.00);
        VdHitWd(ResetWatchDog);        // Hit watch dog
    }

    VdHitWd(ResetWatchDog);        // Hit watch dog

    printf("\r\nFinished with memory allocation & initialization ...");

    crc_value.uWord = 0;
    HR.r = 0.0;
    U_Word.uWord = 0;
    memset(Ain_Sigfail, 0, sizeof(Ain_Sigfail));
    memset(Ain_raw, 0, sizeof(Ain_raw));
    toggle = 0;
    timesecs = 0;
    msFraction = 0;
    memset(DACIn, 0, sizeof(DACIn));
    memset(mainBuffer, 0, sizeof(mainBuffer));
    TotalReadFromPort = 0;
    pqr = 0;
    ActivePortNumber = 0;

    VdHitWd(ResetWatchDog);        // Hit watch dog

    memset(SerialPortProtocol, 0, sizeof(SerialPortProtocol));
    memset(SerialPortMaster, 0, sizeof(SerialPortMaster));
    memset(SendViaRadio, 0, sizeof(SendViaRadio));
    memset(DebugEnable, 0, sizeof(DebugEnable));

#if PORT1 == BRICKNET
    SerialPortProtocol[0] = BRICKNET;
#elif PORT1 == MODBUS
    SerialPortProtocol[0] = MODBUS;
#endif
#if P1_MASTER && PORT1 != UNUSED
    SerialPortMaster[0] = TRUE;
#endif
#if P1_RADIO  && PORT1 != UNUSED
    SendViaRadio[0] = TRUE;
#endif
#if P1_DEBUG  && PORT1 != UNUSED
    DebugEnable[0] = TRUE;
#endif

#if PORT2 == BRICKNET
    SerialPortProtocol[1] = BRICKNET;
#elif PORT2 == MODBUS
    SerialPortProtocol[1] = MODBUS;
#endif
#if P2_MASTER && PORT2 != UNUSED
    SerialPortMaster[1] = TRUE;
#endif
#if P2_RADIO && PORT2 != UNUSED
    SendViaRadio[1] = TRUE;
#endif
#if P2_DEBUG && PORT2 != UNUSED
    DebugEnable[1] = TRUE;
#endif

    for(i = 0; i < TOTAL_PORTS; i++)
    {
        printf("\r\nPort %d Protocol %s is %s", i+1, SerialPortProtocol[i] == BRICKNET ? "BRICKNET" : SerialPortProtocol[i] == MODBUS ? "MODBUS" : "UNUSED", SerialPortMaster[i] ? "MASTER" : "NOT MASTER");
        ResetPort(i);
        VdHitWd(ResetWatchDog);        // Hit watch dog
    }


    memset(Ain_Sigfail_Timer,0,sizeof(Ain_Sigfail_Timer));
    memset(Ain_High_Timer, 0, sizeof(Ain_High_Timer));
    memset(Ain_Low_Timer, 0, sizeof(Ain_Low_Timer));

//   DisplayMemoryUsage();

    tCTU_Comm_Fail = FALSE;	// 06/13/2012
    CTU_Comm_Fail_Timer = 0;	// 06/13/2012

    VdHitWd(ResetWatchDog);        // Hit watch dog
    VdReleaseWd(ResetWatchDog);


    memset(Din_Alarm_Timer, 0, sizeof(Din_Alarm_Timer));
    memset(MQTT_PollRate_Timer, 0, sizeof(MQTT_PollRate_Timer));
    memset(timeToPublish, 0, sizeof(timeToPublish));
    memset(receivedTopics, 0, sizeof(receivedTopics));
    memset(receivedmessage, 0, sizeof(receivedmessage));

    stateCase = 0;
    stateCase_Last = 0;
    stateCaseTemp = 0;
    stateCaseMessage = 0;
    LookingForTemp = 0;
    rxTimeOuttimer = 0;
    connTimeOuttimer = 0;
    rxMQTT_TimeOuttimer = 0;
    ConnAck = 0;
    PingResp = 0;
    publishCounter = 0;
    subscribeCounter = 0;
    MQTTRxData = 0;
    MQTTPort2Use = 0;
    tempHasBeenRead = 0;
    receiveOK = 0;
    DigitalInputs = 0;
    DigitalInputs_Last = 0;
    publishState = 0;
    startPubOrSub = 0;
    WaitOnPingResp = 0;
    SubscribeAck = 0;
    publishVariable = 0;
    subLength = 0;
    pubLength = 0;
    receiveTopicLen = 0;
    NotConnectedToBrokerTimer=0;
    NotConnectedToBrokerTimeOut=180;
    PingTimer = 0;
    TimeToPing = 0;
    willFlag = 0;
    qosLevel = 0;
    memset(willTopic, 0, sizeof(willTopic));
    memset(willMessage, 0, sizeof(willMessage));
    memset(username, 0, sizeof(username));
    memset(password, 0, sizeof(password));
    memset(Filtered_Input, 0, sizeof(Filtered_Input));
    strcpy(willTopic, "56276/RTU/3/NDEATH");
    strcpy(willMessage, "1");
    //memset(topic, 0, sizeof(topic));
    willRetain = 0;
    willQosLevel = 0;
    cleanSession = 1;
    keepAlivePeriod = 60;
    dupFlag = 0;
    retain = 0;
    messageId = 1;
    returnMessageId = 0;
    returnMaxQoS = 0;
    stateCaseResetModem=0;
    ResetRetries = 0;
    qosLevels[0] = 0;
    qosLevels[1] = 0;
    qosLevels[2] = 0;
    qosLevels[3] = 0;
    qosLevels[4] = 0;
    qosLevels[5] = 0;
    qosLevels[6] = 0;
    qosLevels[7] = 0;
    qosLevels[8] = 0;
    qosLevels[9] = 0;
    delay_to_next_step=0;
    MoveOnToCase5=0;
    carryOver=0;
    mktm(&rtc, read_rtc());

    printf("\nLeaving Soft Reset");

}

nodebug void ResetHard(void)
{
    int i, j, k, t[3];
    char strTime[12];
    char* token;
    printf("\nGoing through Hard Reset");

    DLST_OneShot = 0;
    mktm(&rtc, (unsigned long)dc_timestamp);
    strcpy(strTime, __TIME__);
    token = strtok(strTime, ":");
    for (i = 0; token != NULL; i++)
    {
        t[i] = atoi(token);
        token = strtok(NULL, ":");
    }
    rtc.tm_hour = t[0];
    rtc.tm_min = t[1];
    rtc.tm_sec = t[2];

    tm_wr(&rtc);
    SEC_TIMER = mktime(&rtc);

    SecsSinceMidnight = (long)((rtc.tm_hour * 3600l) + (rtc.tm_min * 60l) + rtc.tm_sec);

    printf("\nCURRENT TIME: %02d/%02d/%04d %02d:%02d:%02d %02d %ld\n\n",rtc.tm_mon, rtc.tm_mday, 1900+rtc.tm_year,rtc.tm_hour, rtc.tm_min, rtc.tm_sec, rtc.tm_wday, SecsSinceMidnight);

    UnitAddr[0] = DEFAULT_ADDR_PORT1;
    UnitAddr[1] = DEFAULT_ADDR_PORT2;

    //NumRoutesDefined = 0;

    for(i = 0; i < TOTAL_PORTS; i++)
        for(j = 0; j < MAXREPEATERROUTES; j++)
            for(k = 0; k < 4; k++)
                RoutingTable[i][j][k] = 0;
    /*
       RoutingTable[SER_PORT1][0][0] = 99;
       RoutingTable[SER_PORT1][0][1] = 33;
       RoutingTable[SER_PORT1][0][2] = 133;

       RoutingTable[SER_PORT1][1][0] = 99;
       RoutingTable[SER_PORT1][1][1] = 34;
       RoutingTable[SER_PORT1][1][2] = 134;

       RoutingTable[SER_PORT1][2][0] = 99;
       RoutingTable[SER_PORT1][2][1] = 35;
       RoutingTable[SER_PORT1][2][2] = 135;

       RoutingTable[SER_PORT1][3][0] = 99;
       RoutingTable[SER_PORT1][3][1] = 36;
       RoutingTable[SER_PORT1][3][2] = 136;
    */

    memset(Gals, 0, sizeof(Gals));
    memset(KGals, 0, sizeof(KGals));
    memset(MGals, 0, sizeof(MGals));
    memset(HRtimer, 0, sizeof(HRtimer));
    memset(Hours, 0, sizeof(Hours));
    memset(KHours, 0, sizeof(KHours));
    memset(MHours, 0, sizeof(MHours));
    memset(Din_Starts, 0, sizeof(Din_Starts));
    memset(Din_KStarts, 0, sizeof(Din_KStarts));

    memset(Ain_HLA_Timer_Preset, 0, sizeof(Ain_HLA_Timer_Preset));
    memset(Ain_LLA_Timer_Preset, 0, sizeof(Ain_LLA_Timer_Preset));

    memset(Din_last, FALSE, sizeof(Din_last));

    for(i = 0; i < 3; i++)
    {
        AinScaleFactor[i] = 350; // Default value 1.0
        AoutScaleFactor[i] = 350; // Default value 1.0
        Ain_HLA_Timer_Preset[i] = 300;	// seconds
        Ain_LLA_Timer_Preset[i] = 300;	// seconds
    }

    Ain_High_SP[1] = 32;
    Ain_Low_SP[1] = 20;

    Ain_High_SP[2] = 32;
    Ain_Low_SP[2] = 20;


    LastLoadTimeStamp = dc_timestamp;
    memset(Din_Invert, 0, sizeof(Din_Invert));
    memset(Din_Enable, 0, sizeof(Din_Enable));

    memset(Din_Alarm_Delay, 0, sizeof(Din_Alarm_Delay));
    //memset(Level_HLA_SP, 0, sizeof(Level_HLA_SP));
    memset(High_Level_Alarm_Delay, 0, sizeof(High_Level_Alarm_Delay));
    //memset(Level_LLA_SP, 0, sizeof(Level_LLA_SP));
    memset(Low_Level_Alarm_Delay, 0, sizeof(Low_Level_Alarm_Delay));
    memset(theTopic, 0, sizeof(theTopic));
    memset(theMessage, 0, sizeof(theMessage));
    memset(tempMess, 0, sizeof(tempMess));
    memset(MQTT_PollRate, 0, sizeof(MQTT_PollRate));
    memset(MQTT_Poll_Enable, 0, sizeof(MQTT_Poll_Enable));
    memset(clientID, 0, sizeof(clientID));

    memset(topics[0], 0, sizeof(topics[0]));
    memset(topics[1], 0, sizeof(topics[1]));
    memset(topics[2], 0, sizeof(topics[2]));
    memset(topics[3], 0, sizeof(topics[3]));
    memset(topics[4], 0, sizeof(topics[4]));
    memset(topics[5], 0, sizeof(topics[5]));
    memset(topics[6], 0, sizeof(topics[6]));
    memset(topics[7], 0, sizeof(topics[7]));
    memset(topics[8], 0, sizeof(topics[8]));
    memset(topics[9], 0, sizeof(topics[9]));

    memset(subscribetopics[0], 0, sizeof(subscribetopics[0]));
    memset(subscribetopics[1], 0, sizeof(subscribetopics[1]));
    memset(subscribetopics[2], 0, sizeof(subscribetopics[2]));
    memset(subscribetopics[3], 0, sizeof(subscribetopics[3]));
    memset(subscribetopics[4], 0, sizeof(subscribetopics[4]));
    memset(subscribetopics[5], 0, sizeof(subscribetopics[5]));
    memset(subscribetopics[6], 0, sizeof(subscribetopics[6]));
    memset(subscribetopics[7], 0, sizeof(subscribetopics[7]));
    memset(subscribetopics[8], 0, sizeof(subscribetopics[8]));
    memset(subscribetopics[9], 0, sizeof(subscribetopics[9]));

    strcpy(topics[0], "56276/RTU/3/DI_STATUS");
	//TODO: need the sparkplug message for the failures to signalwire.
	

    strcpy(subscribetopics[0], "56276/RTU/2/DI_STATUS");


    strcpy(theTopic,"56276/TemperatureF");
    strcpy(theMessage, "55.5");
    strcpy(clientID, "56276_RTU3");

    for(i = 0; i < 10; i++)
    {
        MQTT_PollRate[i] = 60;
        MQTT_Poll_Enable[i] = 1;
        printf("MQTT_PollRate[%d] -- %d\r\n",i,MQTT_PollRate[i]);
    }
    for(i = 2; i < 10; i++)
    {
        MQTT_Poll_Enable[i] = 0;
        printf("MQTT_Poll_Enable[%d] -- %d\r\n",i,MQTT_Poll_Enable[i]);
    }

    Din_Enable[1] = 1;
    Din_Enable[2] = 1;
    Din_Enable[3] = 1;
    Din_Enable[4] = 1;
    Din_Enable[5] = 1;
    Din_Enable[6] = 1;
    Din_Enable[7] = 1;
    Din_Enable[8] = 1;
    //MQTT_PollRate[0] = 300;
    tCTU_Comm_Fail = FALSE;	// 06/13/2012
    pf = 0;
    sr = 0;
    wdto = 0;
    tau=5.;
}

nodebug void PingControl()
{
    ++PingTimer;
    if(ConnAck && (PingTimer >= keepAlivePeriod))
    {
        TimeToPing = 1;
        PingTimer = 0;
        PingResp = 0;
    }
}
nodebug void MonitorPublishItems()
{
    char i;
    int theval;
	char strInputs[(NUMBEROFDI*2)];
	char messageBit = 0;
	char length[3];							  
// create a string for the values of the DIN

	for(i = 0; i <=(NUMBEROFDI*2); i+=2)
	{
		if (i < (NUMBEROFDI*2))
      {
         strInputs[i] = '0';
		 strInputs[i+1] = ',';
      }
		else strInputs[(NUMBEROFDI*2)-1] = '\0';
	}
	//printf("strInputs: %s\n", strInputs);											 
    for(i = 0; i < 10; i++)
    {
        if(MQTT_Poll_Enable[i])
        {
            ++MQTT_PollRate_Timer[i];
            if(MQTT_PollRate_Timer[i] >= MQTT_PollRate[i])
            {
                timeToPublish[i] = 1;
                MQTT_PollRate_Timer[i] = 0;
            }
        }
        else
        {
            timeToPublish[i] = 0;
            MQTT_PollRate_Timer[i] = 0;
        }
    }

    DigitalInputs = CheckCallfors();
    if(ResetModemLogic(ConnAck))
    {
        publishState = 6;
        printf("In the process of resetting the modem\r\n");
    }
    if(TESTING)
    {
        printf("DigitalInputs:%lu\r\n",DigitalInputs);
        printf("\r\nPublishState:%u\r\n",publishState);
        printf("Enable:%u/%u/%u\r\n",MQTT_Poll_Enable[0],MQTT_Poll_Enable[1],MQTT_Poll_Enable[2]);
        //printf("Publish:%u/%u/%u\r\n",timeToPublish[0],timeToPublish[1],timeToPublish[2]);
        //printf("Rate:%d/%d/%d\r\n",MQTT_PollRate[0],MQTT_PollRate[1],MQTT_PollRate[2]);
        //printf("Timer:%d/%d/%d\r\n",MQTT_PollRate_Timer[0],MQTT_PollRate_Timer[1],MQTT_PollRate_Timer[2]);

        printf("ConnAck:%u\r\n",ConnAck);
        printf("theTopic:%s theMessage:%s\r\n",theTopic,theMessage);
        printf("SubACK:%d\r\n",SubscribeAck);
        //printf("recTopic:%s recMess:%s\r\n",receivedTopics,receivedmessage);
    }
    switch (publishState)
    {
    case 0: //Digital Inputs
        if(timeToPublish[0] || (DigitalInputs != DigitalInputs_Last) && !startPubOrSub)
        {
            startPubOrSub = 1;
            memset(theMessage, 0, sizeof(theMessage));
            memset(theTopic, 0, sizeof(theTopic));
            strcpy(theTopic,topics[0]);


            //itoa(DigitalInputs, theMessage);

			// we will update the strInputs with the data.
			for(i=0;i<8;i++)
			{
			  messageBit = ((DigitalInputs >> i)  & 0x01);
			  if (messageBit == 1) strInputs[i*2] = messageBit + '0';
			  printf("Messbit:%d\r\n",messageBit);
			}

			printf("strInputs: %s\n", strInputs);
			//strcpy(theMessage, "10201,"); //add destination address at modbusMQTT server
			//strcat(theMessage, RTU1_DI_DestAdd);
			//strcat(theMessage,",");
         //strcpy(DestAddress,"10201,");
			//strcpy(theMessage, DestAddress + '0');
			//strcat(theMessage,strInputs);
			//strcat(theMessage,',');
        // itoa(NUMBEROFDI,length);
			//strcat(theMessage,length); //add the length
			//strcat(theMessage,",");
			strcat(theMessage,strInputs);

            if(TESTING)
                printf("DigitalInputs:%lu topicDI:%s Message:%s\r\n",DigitalInputs,theTopic,theMessage);
            timeToPublish[0] = 0;
            ++publishState;
        }
        else
            ++publishState;

        DigitalInputs_Last = DigitalInputs;
        break;

    case 1:
        if(timeToPublish[1] && !startPubOrSub) //We don't publish analogs. Just keep going.
        {
            //printf("msgSize:%d topicSize:%d\r\n",sizeof(theMessage),sizeof(theTopic));
			/*
            startPubOrSub = 1;
            memset(theMessage, 0, sizeof(theMessage));
            memset(theTopic, 0, sizeof(theTopic));
            strcpy(theTopic,topics[1]);
            strcpy(theMessage, "40337,4,");
            for(i=1; i <= 4; i++)
            {
                itoa(AinRead(336+i), tempMess);
                strcat(theMessage, tempMess);
                if(i<4)
                    strcat(theMessage, ",");
            }
            //FloatToString(theMessage,AinRead(1),1);
            printf("topic1:%s message:%s\r\n",topics[1],theMessage);
			*/
            timeToPublish[1] = 0;
            ++publishState;
        }
        else
            ++publishState;
        break;

    case 2: 
	
        if(timeToPublish[2] && !startPubOrSub)
        {
			/*
            //printf("msgSize:%d topicSize:%d\r\n",sizeof(theMessage),sizeof(theTopic));
            startPubOrSub = 1;
            memset(theMessage, 0, sizeof(theMessage));
            memset(theTopic, 0, sizeof(theTopic));
            strcpy(theTopic,topics[2]);
            strcpy(theMessage, "30201,11,");
            for(i=1; i <= 11; i++)
            {
                //memset(tempMess, 0, sizeof(tempMess));
                //FloatToString(tempMess,AinRead(200+i),1);
                itoa(AinRead(200+i), tempMess);
                strcat(theMessage, tempMess);
                if(i<11)
                    strcat(theMessage, ",");
            }
            //FloatToString(theMessage,AinRead(1),1);
            printf("topic1:%s message:%s\r\n",topics[2],theMessage);
            
			*/
			timeToPublish[2] = 0;
            ++publishState;
        }
        else
			
            ++publishState;
        break;

    case 3:
        if(!startPubOrSub && !SubscribeAck)
        {
            startPubOrSub = 2;//if startPubOrSub == 2 then subscribe

            ++publishState;
        }
        else
            ++publishState;

        break;

    case 4:
        printf("Moveto5:%d\r\n",MoveOnToCase5);
        if(TimeToPing  && !startPubOrSub)
        {
            //printf("Sending Ping!");
            makePingString();
            TimeToPing = 0;
            ++publishState;
        }
        else
        {
            ++MoveOnToCase5;
            if(MoveOnToCase5 >= keepAlivePeriod/2)
            {
                publishState = 5;
                MoveOnToCase5 = 0;
            }
            else
                publishState = 0;
        }

        break;

    case 5:

        if(PingResp)
        {
            ConnAck = 1;
            publishState = 0;
            PingResp = 0;
            MoveOnToCase5 = 0;
            //printf("Got PingResp\r\n");
        }
        else
        {
            //printf("Waiting on PingResp\r\n");
            ++WaitOnPingResp;
            if(WaitOnPingResp >= PINGRESP_TIMEOUT)
            {
                ConnAck = 0;
                publishState = 0;
                PingResp = 0;
                WaitOnPingResp = 0;
                startPubOrSub = 0;
                SubscribeAck = 0;
                //printf("Reset PingResp\r\n");
            }
        }

        break;

    case 6:
        if(ResetModem())
        {
            publishState = 0;
            NotConnectedToBrokerTimer = 0;
            if(TESTING)
                printf("Resetting Modem\r\n");

        }
        break;
    default:
        publishState = 0;

    }
    MessageStateHandler(startPubOrSub);
}
nodebug void MessageStateHandler(char _startPubOrSub)
{
    int i;
    i = 0;
    printf("stateCase:%d\r\n",stateCaseMessage);
    if(_startPubOrSub == 1)
    {
        switch (stateCaseMessage)
        {
        case 0:
            if(!ConnAck)
                makeConnectString();
            ++stateCaseMessage;
            break;

        case 1:
            if(ConnAck)
            {
                ++stateCaseMessage;
            }
            else
            {
                ++connTimeOuttimer;
                if(connTimeOuttimer > 5)
                {
                    stateCaseMessage = 0;
                    connTimeOuttimer = 0;
                    printf("Connect Timed Out\r\n");
                }
            }
            break;

        case 2:
            if(ConnAck)
            {
                ++publishCounter;
                if(publishCounter > 2)
                {
                    makePublishString(theTopic,theMessage);
                    publishCounter = 0;
                    ++stateCaseMessage;
                    connTimeOuttimer = 0;
                }
            }
            else
            {
                ++connTimeOuttimer;
                if(connTimeOuttimer > 5)
                {
                    stateCaseMessage = 0;
                    connTimeOuttimer = 0;
                    publishCounter = 0;
                    //printf("Connect Timed Out\r\n");
                }
            }
            break;

        case 3:
            if(ConnAck)
            {
                stateCaseMessage = 0;
                startPubOrSub = 0;
            }
            else
            {
                ++connTimeOuttimer;
                if(connTimeOuttimer > 5)
                {
                    stateCaseMessage = 0;
                    connTimeOuttimer = 0;
                    //printf("Connect Timed Out\r\n");
                }
            }
            break;

        default:
            stateCaseMessage = 0;
        }
    }
    else if(_startPubOrSub == 2)
    {
        switch (stateCaseMessage)
        {
        case 0:
            if(!ConnAck)
                makeConnectString();
            ++stateCaseMessage;
            break;

        case 1:
            if(ConnAck)
            {
                ++stateCaseMessage;
            }
            else
            {
                ++connTimeOuttimer;
                if(connTimeOuttimer > 5)
                {
                    stateCaseMessage = 0;
                    connTimeOuttimer = 0;
                    //printf("Connect Timed Out\r\n");
                }
            }
            break;

        case 2:
            if(ConnAck)
            {
                ++subscribeCounter;
                if(subscribeCounter > 2)
                {
                    printf("Subscribing\r\n");
                    makeSubscribeString();
                    subscribeCounter = 0;
                    ++stateCaseMessage;
                    connTimeOuttimer = 0;
                }
            }
            else
            {
                ++connTimeOuttimer;
                if(connTimeOuttimer > 5)
                {
                    stateCaseMessage = 0;
                    connTimeOuttimer = 0;
                    subscribeCounter = 0;
                    //printf("Connect Timed Out\r\n");
                }
            }
            break;

        case 3:
            if(ConnAck)
            {
                stateCaseMessage = 0;
                startPubOrSub = 0;
            }
            else
            {
                ++connTimeOuttimer;
                if(connTimeOuttimer > 5)
                {
                    stateCaseMessage = 0;
                    connTimeOuttimer = 0;
                    //printf("Connect Timed Out\r\n");
                }
            }
            break;

        default:
            stateCaseMessage = 0;
        }
    }
    else
        stateCaseMessage = 0;
}

nodebug unsigned long CheckCallfors()
{
    unsigned long alarmNum;
    int i;
    alarmNum = 0;
    /*
    for(i=1; i <= NUMBEROFDI; i++)
    {
    	if(((!Din_Invert[i] && DinRead(i)) || (Din_Invert[i] && !DinRead(i))) && Din_Enable[i])
    	//if(((!Din_Invert[i] && DI[i]) || (Din_Invert[i] && !DI[i])) && Din_Enable[i])
    	{
    		++Din_Alarm_Timer[i];
    		if(Din_Alarm_Timer[i] >= Din_Alarm_Delay[i])
    		{
    			alarmNum = alarmNum | (0x01  << (i-1));
    			Din_Alarm_Timer[i] = Din_Alarm_Delay[i];
    		}
    	}
    	else
    	{
    		Din_Alarm_Timer[i] = 0;
    	}
    }
    */
    for(i=1; i <= NUMBEROFAI; i++)
    {
        if(DinRead(i))
            //if(DinRead(i))
            //if(((!Din_Invert[i] && DI[i]) || (Din_Invert[i] && !DI[i])) && Din_Enable[i])
        {
            ++Din_Alarm_Timer[i];
            if(Din_Alarm_Timer[i] >= Din_Alarm_Delay[i])
            {
                alarmNum = alarmNum | (0x01  << (i-1));
                Din_Alarm_Timer[i] = Din_Alarm_Delay[i];
            }
        }
        else
        {
            Din_Alarm_Timer[i] = 0;
        }
    }
    /*
    	for(i=1; i <= NUMBEROFAI; i++)
    	{
    		if(AI[i] >= Level_HLA_SP[i])
    		{
    			++High_Level_Timer[i];
    			if(High_Level_Timer[i] >= High_Level_Alarm_Delay[i])
    			{
    				alarmNum = alarmNum | (0x01  << (i+3));
    				High_Level_Timer[i] = High_Level_Alarm_Delay[i];
    			}
    		}
    		else
    		{
    			High_Level_Timer[i] = 0;
    		}
    		if(AI[i] <= Level_LLA_SP[i])
    		{
    			++Low_Level_Timer[i];
    			if(Low_Level_Timer[i] >= Low_Level_Alarm_Delay[i])
    			{
    				alarmNum = alarmNum | (0x01  << (i+5));
    				Low_Level_Timer[i] = Low_Level_Alarm_Delay[i];
    			}
    		}
    		else
    		{
    			Low_Level_Timer[i] = 0;
    		}
    	}
    */
    return alarmNum;
}

// convert float to string one decimal digit at a time
// assumes float is < 65536 and ARRAYSIZE is big enough
// problem: it truncates numbers at size without rounding
// str is a char array to hold the result, float is the number to convert
// size is the number of decimal digits you want
nodebug char* FloatToString(char *str, float f, char size)
{
    char pos;  // position in string
    char len;  // length of decimal part of result
    char* curr;  // temp holder for next digit
    int value;  // decimal digit(s) to convert
    pos = 0;  // initialize pos, just to be sure
    value = 0;
    len = 0;
    curr = "\0";
    //printf("Im in FloatToString\r\n");
    value = (int)f;  // truncate the floating point number
    itoa(value,str);  // this is kinda dangerous depending on the length of str
    // now str array has the digits before the decimal

    if (f < 0 )  // handle negative numbers
    {
        f *= -1;
        value *= -1;
    }

    len = strlen(str);  // find out how big the integer part was
    pos = len;  // position the pointer to the end of the integer part
    str[pos++] = '.';  // add decimal point to string
    //printf("Len:%d Pos:%d value:%d\r\n",len,pos,value);
    while(pos < (size + len + 1) )  // process remaining digits
    {
        f = f - (float)value;  // hack off the whole part of the number
        f *= 10;  // move next digit over
        value = (int)f;  // get next digit
        itoa(value, curr); // convert digit to string
        str[pos++] = *curr; // add digit to result string and increment pointer
    }
    //printf("str:%s\r\n",str);
    return str;
}

nodebug void MQTTDriver()
{
    MQTTRxData = serFgetc();

    if( MQTTRxData != -1 )
        MQTT_ReceiveData((char) (MQTTRxData & 0xff));
    else
    {
        if((stateCase != 0) && (stateCase == stateCase_Last))
        {
            ++rxMQTT_TimeOuttimer;
            if(rxMQTT_TimeOuttimer > 25)
            {
                stateCaseTemp =  0;
                stateCase = 0;
                rxMQTT_TimeOuttimer = 0;
            }
        }
    }

}
nodebug void MQTT_ReceiveData(char ch)
{
    //printf("inbyte:%x sCase:%d\r\n",ch,stateCase);

    switch (stateCase)
    {
    case 0:
        if(ch == 0x20)//Conn-ack
            stateCase = 100;
        if(ch == 0x4f)//receive O in OK from modem, go to 200 and look for K
            stateCase = 200;
        if(ch == 0xd0)//PingResp
            stateCase = 400;
        if(ch == 0x40)//PubAck
            stateCase = 500;
        if(ch == 0x90)//SubscribeAck
        {
            stateCase = 600;
            returnMessageId = 0;
            subLength = 0;
        }
        if(ch == 0x30)//Received a Publish
        {
            publishVariable = ch;
            receivePubIndexer = 0;
            pubLength = 0;
            receiveTopicLen = 0;
            stateCase = 700;
            carryOver = 1;
        }
        break;

    case 100:
        if(ch == 0x02)//Conn-ack
            ++stateCase;
        break;

    case 101:
        if(ch == 0x00)//Conn-ack
            ++stateCase;
        break;

    case 102:
        if(ch == 0x00)//Conn-ack
        {
            stateCase = 0;
            ConnAck = 1;
            //printf("Got to 102\r\n");
        }
        else
        {
            stateCase = 0;
            ConnAck = 0;
        }
        break;

    case 200://Looking for the K in OK from the modem
        if(ch == 0x4b)//Looking for K
        {
            //receiveOK = 1;
            ++stateCase;
        }
        else
        {
            stateCase = 0;
        }
        break;

    case 201://Looking for the cr from the modem
        if(ch == 0x0d)//Looking for carriage return
        {
            receiveOK = 1;
            stateCase = 0;
        }
        else
        {
            stateCase = 0;
        }
        break;

    case 300://Looking for Temperature first ch
        LookingForTemp = 0;
        temperature[0] = ch;
        stateCase = 301;
        //printf("ch:%x Case:%d\r\n",ch,stateCase);
        break;

    case 301://Looking for Temperature second ch
        temperature[1] = ch;
        tempHasBeenRead = 1;
        stateCase = 0;
        //printf("ch:%x Case:%d\r\n",ch,stateCase);
        break;

    case 400:
        if(ch == 0x00)//PingResp
        {
            stateCase = 0;
            PingResp = 1;
        }
        break;

    case 500://PubAck
        if(ch == 0x02)//Looking for the length
            ++stateCase;
        else
            stateCase = 0;
        break;

    case 501://Looking for the returnMessageId MSB
        returnMessageId = (char)((ch << 8) & 0x00FF);
        ++stateCase;
        break;
    case 502://Looking for the returnMessageId LSB
        returnMessageId = (char)(ch & 0x00FF);
        stateCase = 0;
        break;


    case 600://SubscribeAck
        subLength = ch;//Looking for the length
        ++stateCase;
        break;

    case 601://Looking for the returnMessageId MSB
        returnMessageId = (char)((ch << 8) & 0x00FF);
        ++stateCase;
        break;
    case 602://Looking for the returnMessageId LSB
        returnMessageId = (char)(ch & 0x00FF);
        ++stateCase;
        break;
    case 603://Looking for the Max QoS
        returnMaxQoS = ch;
        if((returnMessageId = messageId) && (returnMaxQoS == qosLevels[0]))
            SubscribeAck = 1;
        else
            SubscribeAck = 0;

        stateCase = 0;
        break;

    case 700://Receiving a Publish Message
        memset(receivedTopics, 0, sizeof(receivedTopics));
        memset(receivedmessage, 0, sizeof(receivedmessage));

        pubLength += (ch & 0x7F) * carryOver;
        carryOver *= 128;
        if((ch & 0x80) == 0)
            ++stateCase;
        //printf("pubLen:%d CO:%d ch:%x\r\n",pubLength,carryOver,ch);

        break;

    case 701://Looking for the receiveTopicLen MSB
        receiveTopicLen = (char)((ch << 8) & 0x00FF);
        //printf("MSBTLen:%d ch:%x\r\n",receiveTopicLen,ch);
        ++stateCase;
        break;
    case 702://Looking for the receiveTopicLen LSB
        receiveTopicLen = (char)(ch & 0x00FF);
        //printf("LSBTLen:%d ch:%x\r\n",receiveTopicLen,ch);
        ++stateCase;
        break;
    case 703://Parse out the Topic and Message
        ++receivePubIndexer;
        if(receivePubIndexer <= receiveTopicLen)
            receivedTopics[receivePubIndexer-1] = ch;
        if(receivePubIndexer > receiveTopicLen && (receivePubIndexer < pubLength))
            receivedmessage[receivePubIndexer-receiveTopicLen-1] = ch;

        if(receivePubIndexer >= pubLength-2)
        {
            stateCase = 0;
            //printf("rxIndex-PubLen-TopicLen:%d-%d-%d T:%s M:%s ch:%x\r\n",receivePubIndexer,pubLength,receiveTopicLen,receivedTopics,receivedmessage,ch);
            ParseMessage(receivedTopics,receivedmessage);
        }
        break;

    default:
        stateCase = 0;
    }

    stateCase_Last = stateCase;

}
nodebug void ParseMessage(char *Topic, char *Message)
{
    int messageValue = 0;
    char messageBit = 0;
    int i = 0;
    int j = 0;
    const char s[2] = ",";
    char *token;

    //printf("Message:%s\r\n",Message);
    //printf("Topic:%s\r\n",Topic);
    if(strcmp(subscribetopics[0],Topic) == 0)	//This is the DI_STATUS message from the elevated tank
    {
        i = 0;
        token = strtok(Message, s);
        printf("Message:%s\r\n",Message);
        printf("Token:%s\r\n",token);

        while( token != NULL )
        {
			/*
			i = 
			3 then Power_Failure - do nothing with this.
			4 then Lead_Well_Call - write to DO 1
			5 then Lag_Well_Call - write to DO 2
			6 then Tank_Low_Level - write to DO 3
			7 then Tank_High_Level - write to DO 4

			*/
            i++;
            if ((i > 3) and (i < 8)) //we have to skip base address, length, and power failure
			
			{
				DoutWrite(i-3,atoi(token));
				printf("Dout%d:%d\r\n",i-3,DoutRead(i-3)); 
			}
            token = strtok(NULL, s);
			printf("Token:%s\r\n",token);
        }
    }

}
nodebug void MQTT_SendData(unsigned char* Data, unsigned int len)
{
    serFwrite(Data, len);
}
nodebug void makeConnectString(void)
{
    int fixedHeaderSize;
    int varHeaderSize;
    int payloadSize;
    int remainingLength;
    unsigned char buffer[510];
    int index;
    char clientIdUtf8[76];
    char willTopicUtf8[51];
    char willMessageUtf8[51];
    char usernameUtf8[51];
    char passwordUtf8[51];
    int temp;
    char connectFlags;
    int i;
    i=0;
    fixedHeaderSize = 0;
    varHeaderSize = 0;
    payloadSize = 0;
    remainingLength = 0;
    index = 0;
    temp = 0;
    connectFlags = 0x00;
    memset(buffer, 0, sizeof(buffer));
    memset(clientIdUtf8, 0, sizeof(clientIdUtf8));
    memset(willTopicUtf8, 0, sizeof(willTopicUtf8));
    memset(willMessageUtf8, 0, sizeof(willMessageUtf8));
    memset(usernameUtf8, 0, sizeof(usernameUtf8));
    memset(passwordUtf8, 0, sizeof(passwordUtf8));
    strcpy(clientIdUtf8, clientID);
    printf("clientID:%s\r\n",clientIdUtf8);
    if(willFlag && (willTopic != 0))
        strcpy(willTopicUtf8, willTopic);
    if(willFlag && (willMessage != 0))
        strcpy(willMessageUtf8, willMessage);
    if((username != 0) && (username != 0))
        strcpy(usernameUtf8, username);
    if((password != 0) && (password != 0))
        strcpy(passwordUtf8, password);

    varHeaderSize += (PROTOCOL_NAME_LEN_SIZE + PROTOCOL_NAME_V3_1_1_SIZE);
    // protocol level field size
    varHeaderSize += PROTOCOL_VERSION_SIZE;
    // connect flags field size
    varHeaderSize += CONNECT_FLAGS_SIZE;
    // keep alive timer field size
    varHeaderSize += KEEP_ALIVE_TIME_SIZE;
    //printf("clientIDSize:%d\r\n",strlen(clientIdUtf8));
    // client identifier field size
    payloadSize += strlen(clientIdUtf8) + 2;

    // will topic field size
    payloadSize += (strlen(willTopicUtf8) > 0) ? strlen(willTopicUtf8) : 0;
    // will message field size
    payloadSize += (strlen(willMessageUtf8) > 0) ? strlen(willMessageUtf8) : 0;
    // username field size
    payloadSize += (strlen(usernameUtf8) > 0) ? strlen(usernameUtf8) : 0;
    // password field size
    payloadSize += (strlen(passwordUtf8) > 0) ? strlen(passwordUtf8) : 0;

    remainingLength += (varHeaderSize + payloadSize);
    // first byte of fixed header
    fixedHeaderSize = 1;

    temp = remainingLength;
    // increase fixed header size based on remaining length
    // (each remaining length byte can encode until 128)
    do
    {
        fixedHeaderSize++;
        temp = temp / 128;
    }
    while (temp > 0);
    // first fixed header byte
    buffer[index++] = (MQTT_MSG_CONNECT_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_CONNECT_FLAG_BITS; // [v.3.1.1]
    //printf("buff0:%x\r\n",buffer[0]);
    buffer[index++] = (char)remainingLength;
    //printf("buff1:%x\r\n",buffer[1]);
    // protocol name
    buffer[index++] = 0; // MSB protocol name size
    //printf("buff2:%x\r\n",buffer[2]);
    // MQTT version 3.1.1
    buffer[index++] = PROTOCOL_NAME_V3_1_1_SIZE; // LSB protocol name size
    //printf("buff3:%x\r\n",buffer[3]);
    buffer[index++] = MM;
    //printf("buff4:%x\r\n",buffer[4]);
    buffer[index++] = QQ;
    //printf("buff5:%x\r\n",buffer[5]);
    buffer[index++] = TT;
    //printf("buff6:%x\r\n",buffer[6]);
    buffer[index++] = TT;
    //printf("buff7:%x\r\n",buffer[7]);
    // protocol version
    buffer[index++] = PROTOCOL_VERSION_V3_1_1;
    //printf("buff8:%x\r\n",buffer[8]);
    // connect flags
    connectFlags = 0x00;
    if(strlen(usernameUtf8) > 0)
        connectFlags |= (char)(1 << USERNAME_FLAG_OFFSET);
    if(strlen(passwordUtf8) > 0)
        connectFlags |= (char)(1 << PASSWORD_FLAG_OFFSET);
    if(willRetain)
        connectFlags |= (char)(1 << WILL_RETAIN_FLAG_OFFSET);
    //connectFlags |= (strlen(usernameUtf8) > 0) ? (char)(1 << USERNAME_FLAG_OFFSET) : (char)0x00;
    //connectFlags |= (strlen(passwordUtf8) > 0) ? (char)(1 << PASSWORD_FLAG_OFFSET) : (char)0x00;
    //connectFlags |= (willRetain) ? (char)(1 << WILL_RETAIN_FLAG_OFFSET) : (char)0x00;
    // only if will flag is set, we have to use will QoS level (otherwise is MUST be 0)
    if (willFlag)
        connectFlags |= (char)(willQosLevel << WILL_QOS_FLAG_OFFSET);

    if(willFlag)
        connectFlags |= (char)(1 << WILL_FLAG_OFFSET);
    if(cleanSession)
        connectFlags |= (char)(1 << CLEAN_SESSION_FLAG_OFFSET);
    //connectFlags |= (willFlag) ? (char)(1 << WILL_FLAG_OFFSET) : (char)0x00;
    //connectFlags |= (cleanSession) ? (char)(1 << CLEAN_SESSION_FLAG_OFFSET) : (char)0x00;
    buffer[index++] = connectFlags;
    //printf("buff9:%x\r\n",buffer[9]);
    // keep alive period
    buffer[index++] = (char)((keepAlivePeriod >> 8) & 0x00FF); // MSB
    //printf("buff10:%x\r\n",buffer[10]);
    buffer[index++] = (char)(keepAlivePeriod & 0x00FF); // LSB
    //printf("buff11:%x\r\n",buffer[11]);

    // client identifier
    buffer[index++] = (char)((strlen(clientIdUtf8) >> 8) & 0x00FF); // MSB
    //printf("buff12:%x\r\n",buffer[12]);
    buffer[index++] = (char)(strlen(clientIdUtf8) & 0x00FF); // LSB
    //printf("buff13:%x\r\n",buffer[13]);
    for(i = 0; i < strlen(clientIdUtf8); i++)
    {
        buffer[index++] = clientIdUtf8[i];
    }
    // will topic
    if (willFlag && (strlen(willTopicUtf8) > 0))
    {
        buffer[index++] = (char)((strlen(willTopicUtf8) >> 8) & 0x00FF); // MSB
        buffer[index++] = (char)(strlen(willTopicUtf8) & 0x00FF); // LSB
        for(i = 0; i < strlen(willTopicUtf8); i++)
        {
            buffer[index++] = willTopicUtf8[i];
        }
    }
    // will message
    if (willFlag && (strlen(willMessageUtf8) > 0))
    {
        buffer[index++] = (char)((strlen(willMessageUtf8) >> 8) & 0x00FF); // MSB
        buffer[index++] = (char)(strlen(willMessageUtf8) & 0x00FF); // LSB
        for(i = 0; i < strlen(willMessageUtf8); i++)
        {
            buffer[index++] = willMessageUtf8[i];
        }
    }

    // username
    if (strlen(usernameUtf8) > 0)
    {
        buffer[index++] = (char)((strlen(usernameUtf8) >> 8) & 0x00FF); // MSB
        buffer[index++] = (char)(strlen(usernameUtf8) & 0x00FF); // LSB
        for(i = 0; i < strlen(usernameUtf8); i++)
        {
            buffer[index++] = usernameUtf8[i];
        }
    }

    // password
    if (strlen(passwordUtf8) > 0)
    {
        buffer[index++] = (char)((strlen(passwordUtf8) >> 8) & 0x00FF); // MSB
        buffer[index++] = (char)(strlen(passwordUtf8) & 0x00FF); // LSB
        for(i = 0; i < strlen(passwordUtf8); i++)
        {
            buffer[index++] = passwordUtf8[i];
        }
    }
    //printf("Connecting Done\r\n");
    MQTT_SendData(buffer,index);
    //return buffer;
}

nodebug void makePublishString(char *pubTopic,char *pubMessage)
{
    int fixedHeaderSize;
    int varHeaderSize;
    int payloadSize;
    int remainingLength;
    unsigned char buffer[510];
    int index;
    char topicUtf8[101];
    char message[201];
    int temp;
    int i;
    i=0;
    //printf("publishing\r\n");
    fixedHeaderSize = 0;
    varHeaderSize = 0;
    payloadSize = 0;
    remainingLength = 0;
    index = 0;
    temp = 0;

    memset(buffer, 0, sizeof(buffer));
    memset(topicUtf8, 0, sizeof(topicUtf8));
    memset(message, 0, sizeof(message));
    strcpy(topicUtf8,pubTopic);
    strcpy(message, pubMessage);
    //printf("topic:%s\r\n",topicUtf8);
    //printf("message:%s\r\n",message);
    // topic name
    varHeaderSize += strlen(topicUtf8) + 2;

    // message id is valid only with QOS level 1 or QOS level 2
    if ((qosLevel == QOS_LEVEL_AT_LEAST_ONCE) || (qosLevel == QOS_LEVEL_EXACTLY_ONCE))
    {
        varHeaderSize += MESSAGE_ID_SIZE;
    }

    // check on message with zero length
    if (strlen(message) > 0)
        // message data
        payloadSize += strlen(message);

    remainingLength += (varHeaderSize + payloadSize);

    // first byte of fixed header
    fixedHeaderSize = 1;

    temp = remainingLength;
    // increase fixed header size based on remaining length
    // (each remaining length byte can encode until 128)
    /*
    do
    {
    	fixedHeaderSize++;
    	temp = temp / 128;
    } while (temp > 0);
    */

    printf("FixedHeader:%d remain:%d VarSize:%d payload:%d\r\n",fixedHeaderSize,remainingLength,varHeaderSize,payloadSize);

    // first fixed header byte
    buffer[index] = (char)((MQTT_MSG_PUBLISH_TYPE << MSG_TYPE_OFFSET) |
                           (qosLevel << QOS_LEVEL_OFFSET));
    if(dupFlag)
        buffer[index] |= (char)(1 << DUP_FLAG_OFFSET);
    if(retain)
        buffer[index] |= (char)(1 << RETAIN_FLAG_OFFSET);
    index++;
    while (temp > 0x7f)
    {
        buffer[index++] = ((temp & 0x7f) | 0x80);
        temp >>= 7;
    }
    buffer[index++] = temp;
    //buffer[index++] = (char)remainingLength;
    // topic name
    buffer[index++] = (char)((strlen(topicUtf8) >> 8) & 0x00FF); // MSB
    //printf("msb:%x len:%d\r\n",buffer[index],strlen(topicUtf8));
    buffer[index++] = (char)(strlen(topicUtf8) & 0x00FF); // LSB
    //printf("lsb:%x\r\n",(char)(strlen(topicUtf8) & 0x00FF));
    for(i = 0; i < strlen(topicUtf8); i++)
    {
        buffer[index++] = topicUtf8[i];
    }

    // message id is valid only with QOS level 1 or QOS level 2
    if ((qosLevel == QOS_LEVEL_AT_LEAST_ONCE) || (qosLevel == QOS_LEVEL_EXACTLY_ONCE))
    {
        // check message identifier assigned
        buffer[index++] = (char)((messageId >> 8) & 0x00FF); // MSB
        buffer[index++] = (char)(messageId & 0x00FF); // LSB
    }

    for(i = 0; i < strlen(message); i++)
    {
        buffer[index++] = message[i];
    }
    MQTT_SendData(buffer,index);
}

nodebug void makeSubscribeString()
{
    int fixedHeaderSize;
    int varHeaderSize;
    int payloadSize;
    int remainingLength;
    unsigned char buffer[510];
    int index;
    int topicIdx;
    int temp;
    int i;
    int numberOfTopics;
    char topicsUtf8[10][101];

    fixedHeaderSize = 0;
    varHeaderSize = 0;
    payloadSize = 0;
    remainingLength = 0;
    index = 0;
    topicIdx = 0;
    temp = 0;
    i = 0;
    numberOfTopics = 0;

    memset(buffer, 0, sizeof(buffer));
    memset(topicsUtf8[0], 0, sizeof(topicsUtf8[0]));
    memset(topicsUtf8[1], 0, sizeof(topicsUtf8[1]));
    memset(topicsUtf8[2], 0, sizeof(topicsUtf8[2]));
    memset(topicsUtf8[3], 0, sizeof(topicsUtf8[3]));
    memset(topicsUtf8[4], 0, sizeof(topicsUtf8[4]));
    memset(topicsUtf8[5], 0, sizeof(topicsUtf8[5]));
    memset(topicsUtf8[6], 0, sizeof(topicsUtf8[6]));
    memset(topicsUtf8[7], 0, sizeof(topicsUtf8[7]));
    memset(topicsUtf8[8], 0, sizeof(topicsUtf8[8]));
    memset(topicsUtf8[9], 0, sizeof(topicsUtf8[9]));
    // message identifier
    varHeaderSize += MESSAGE_ID_SIZE;
    numberOfTopics = 4; //Watch out for this!!!!! TODO:

    for (topicIdx = 0; topicIdx < numberOfTopics; topicIdx++)
    {
        if(strlen(subscribetopics[topicIdx]) > 0)
        {
            strcpy(topicsUtf8[topicIdx],subscribetopics[topicIdx]);
            payloadSize += 2; // topic size (MSB, LSB)
            payloadSize += strlen(topicsUtf8[topicIdx]);
            payloadSize++; // byte for QoS
        }
    }

    remainingLength += (varHeaderSize + payloadSize);

    // first byte of fixed header
    fixedHeaderSize = 1;

    temp = remainingLength;
    // increase fixed header size based on remaining length
    // (each remaining length byte can encode until 128)
    do
    {
        fixedHeaderSize++;
        temp = temp / 128;
    }
    while (temp > 0);

    // first fixed header byte
    buffer[index++] = (MQTT_MSG_SUBSCRIBE_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_SUBSCRIBE_FLAG_BITS;
    buffer[index++] = (char)remainingLength;
    buffer[index++] = (char)((messageId >> 8) & 0x00FF); // MSB
    buffer[index++] = (char)(messageId & 0x00FF); // LSB

    topicIdx = 0;
    for (topicIdx = 0; topicIdx < numberOfTopics; topicIdx++)
    {
        if(strlen(topicsUtf8[topicIdx]) > 0)
        {
            buffer[index++] = (char)((strlen(topicsUtf8[topicIdx]) >> 8) & 0x00FF); // MSB
            buffer[index++] = (char)(strlen(topicsUtf8[topicIdx]) & 0x00FF); // LSB
            for(i = 0; i < strlen(topicsUtf8[topicIdx]); i++)
            {
                buffer[index++] = topicsUtf8[topicIdx][i];
            }
            // requested QoS
            buffer[index++] = qosLevels[topicIdx];
        }
    }
    MQTT_SendData(buffer,index);
}

nodebug void makePingString(void)
{
    unsigned char buffer[5];
    int index;
    index = 0;

    memset(buffer, 0, sizeof(buffer));
    // first fixed header byte
    buffer[index++] = (MQTT_MSG_PINGREQ_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_PINGREQ_FLAG_BITS;

    buffer[index++] = 0x00;
    MQTT_SendData(buffer,index);
}

nodebug void makeDisconnectString(void)
{
    unsigned char buffer[5];
    int index;
    index = 0;

    memset(buffer, 0, sizeof(buffer));
    // first fixed header byte
    buffer[index++] = (MQTT_MSG_DISCONNECT_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_DISCONNECT_FLAG_BITS; // [v.3.1.1]

    buffer[index++] = 0x00;
    MQTT_SendData(buffer,index);
}

nodebug void makePubAckString(void)
{
    int fixedHeaderSize;
    int varHeaderSize;
    int remainingLength;
    unsigned char buffer[10];
    int index;
    int temp;
    fixedHeaderSize = 0;
    varHeaderSize = 0;
    remainingLength = 0;
    index = 0;
    temp = 0;

    memset(buffer, 0, sizeof(buffer));
    // message identifier
    varHeaderSize += MESSAGE_ID_SIZE;

    remainingLength += varHeaderSize;

    // first byte of fixed header
    fixedHeaderSize = 1;

    temp = remainingLength;
    // increase fixed header size based on remaining length
    // (each remaining length byte can encode until 128)
    do
    {
        fixedHeaderSize++;
        temp = temp / 128;
    }
    while (temp > 0);

    // first fixed header byte
    buffer[index++] = (MQTT_MSG_PUBACK_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_PUBACK_FLAG_BITS; // [v.3.1.1]
    buffer[index++] = (char)remainingLength;
    // get message identifier
    buffer[index++] = (char)((messageId >> 8) & 0x00FF); // MSB
    buffer[index++] = (char)(messageId & 0x00FF); // LSB

    MQTT_SendData(buffer,index);
}

nodebug void makePubRecString(void)
{
    int fixedHeaderSize;
    int varHeaderSize;
    int remainingLength;
    unsigned char buffer[10];
    int index;
    int temp;

    fixedHeaderSize = 0;
    varHeaderSize = 0;
    remainingLength = 0;
    index = 0;
    temp = 0;

    memset(buffer, 0, sizeof(buffer));
    // message identifier
    varHeaderSize += MESSAGE_ID_SIZE;

    remainingLength += varHeaderSize;

    // first byte of fixed header
    fixedHeaderSize = 1;

    temp = remainingLength;
    // increase fixed header size based on remaining length
    // (each remaining length byte can encode until 128)
    do
    {
        fixedHeaderSize++;
        temp = temp / 128;
    }
    while (temp > 0);


    buffer[index++] = (MQTT_MSG_PUBREC_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_PUBREC_FLAG_BITS; // [v.3.1.1]
    buffer[index++] = (char)remainingLength;

    // get message identifier
    buffer[index++] = (char)((messageId >> 8) & 0x00FF); // MSB
    buffer[index++] = (char)(messageId & 0x00FF); // LSB

    MQTT_SendData(buffer,index);
}

nodebug void makePubRelString(void)
{
    int fixedHeaderSize;
    int varHeaderSize;
    int remainingLength;
    unsigned char buffer[10];
    int index;
    int temp;

    fixedHeaderSize = 0;
    varHeaderSize = 0;
    remainingLength = 0;
    index = 0;
    temp = 0;

    memset(buffer, 0, sizeof(buffer));

    // message identifier
    varHeaderSize += MESSAGE_ID_SIZE;

    remainingLength += varHeaderSize;

    // first byte of fixed header
    fixedHeaderSize = 1;

    temp = remainingLength;
    // increase fixed header size based on remaining length
    // (each remaining length byte can encode until 128)
    do
    {
        fixedHeaderSize++;
        temp = temp / 128;
    }
    while (temp > 0);

    buffer[index++] = (MQTT_MSG_PUBREL_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_PUBREL_FLAG_BITS; // [v.3.1.1]
    buffer[index++] = (char)remainingLength;
    // get next message identifier
    buffer[index++] = (char)((messageId >> 8) & 0x00FF); // MSB
    buffer[index++] = (char)(messageId & 0x00FF); // LSB

    MQTT_SendData(buffer,index);
}

nodebug void makePubCompString(void)
{
    int fixedHeaderSize;
    int varHeaderSize;
    int remainingLength;
    unsigned char buffer[10];
    int index;
    int temp;

    fixedHeaderSize = 0;
    varHeaderSize = 0;
    remainingLength = 0;
    index = 0;
    temp = 0;

    memset(buffer, 0, sizeof(buffer));

    // message identifier
    varHeaderSize += MESSAGE_ID_SIZE;

    remainingLength += varHeaderSize;

    // first byte of fixed header
    fixedHeaderSize = 1;

    temp = remainingLength;
    // increase fixed header size based on remaining length
    // (each remaining length byte can encode until 128)
    do
    {
        fixedHeaderSize++;
        temp = temp / 128;
    }
    while (temp > 0);

    buffer[index++] = (MQTT_MSG_PUBCOMP_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_PUBCOMP_FLAG_BITS; // [v.3.1.1]
    buffer[index++] = (char)remainingLength;
    // get next message identifier
    buffer[index++] = (char)((messageId >> 8) & 0x00FF); // MSB
    buffer[index++] = (char)(messageId & 0x00FF); // LSB

    MQTT_SendData(buffer,index);
}
/*
nodebug void makeUnScribeString(void)
{
	int fixedHeaderSize;
	int varHeaderSize;
	int remainingLength;
	char buffer[10];
	int index;
	int temp;
	int topicIdx;

	fixedHeaderSize = 0;
	varHeaderSize = 0;
	remainingLength = 0;
	index = 0;
	temp = 0;
	topicIdx = 0;

	memset(buffer, 0, sizeof(buffer));
	// add topics list empty check

	// message identifier
	varHeaderSize += MESSAGE_ID_SIZE;

	byte[][] topicsUtf8 = new byte[this.topics.Length][];

	for (topicIdx = 0; topicIdx < this.topics.Length; topicIdx++)
	{
		// check topic length
		if ((this.topics[topicIdx].Length < MIN_TOPIC_LENGTH) || (this.topics[topicIdx].Length > MAX_TOPIC_LENGTH))
			throw new MqttClientException(MqttClientErrorCode.TopicLength);

		topicsUtf8[topicIdx] = Encoding.UTF8.GetBytes(this.topics[topicIdx]);
		payloadSize += 2; // topic size (MSB, LSB)
		payloadSize += topicsUtf8[topicIdx].Length;
	}

	remainingLength += (varHeaderSize + payloadSize);

	// first byte of fixed header
	fixedHeaderSize = 1;

	int temp = remainingLength;
	// increase fixed header size based on remaining length
	// (each remaining length byte can encode until 128)
	do
	{
		fixedHeaderSize++;
		temp = temp / 128;
	} while (temp > 0);

	// allocate buffer for message
	buffer = new byte[fixedHeaderSize + varHeaderSize + payloadSize];

	// first fixed header byte
	buffer[index++] = (MQTT_MSG_UNSUBSCRIBE_TYPE << MSG_TYPE_OFFSET) | MQTT_MSG_UNSUBSCRIBE_FLAG_BITS; // [v.3.1.1]

	// encode remaining length
	index = this.encodeRemainingLength(remainingLength, buffer, index);

	// check message identifier assigned
	if (messageId == 0)
		//throw flag

	buffer[index++] = (byte)((messageId >> 8) & 0x00FF); // MSB
	buffer[index++] = (byte)(messageId & 0x00FF); // LSB

	topicIdx = 0;
	for (topicIdx = 0; topicIdx < this.topics.Length; topicIdx++)
	{
		// topic name
		buffer[index++] = (byte)((topicsUtf8[topicIdx].Length >> 8) & 0x00FF); // MSB
		buffer[index++] = (byte)(topicsUtf8[topicIdx].Length & 0x00FF); // LSB
		Array.Copy(topicsUtf8[topicIdx], 0, buffer, index, topicsUtf8[topicIdx].Length);
		index += topicsUtf8[topicIdx].Length;
	}

	MQTT_SendData(buffer,index);
}
*/
/*
nodebug void CheckTemp()
{
	char buffer[30];
	int txIndex;
	txIndex = 0;

	memset(buffer, 0, sizeof(buffer));
	switch (stateCaseTemp)
	{
	   case 0:
			buffer[txIndex++] = '+';
			buffer[txIndex++] = '+';
			buffer[txIndex++] = '+';
			printf("Sending +++\r\n");
			MQTT_SendData(buffer,txIndex);
			++stateCaseTemp;
	   break;

	   case 1:
			if(receiveOK)
			{
				receiveOK = 0;
				printf("ReceivedOK\r\n");
				++stateCaseTemp;
			}
			else
			{
				++rxTimeOuttimer;
				if(rxTimeOuttimer > 2)
				{
					stateCaseTemp =  0;
					rxTimeOuttimer = 0;
					receiveOK = 0;
					printf("TimedOut\r\n");
				}
			}
	   break;

	   case 2://Ask for Temperature
			buffer[txIndex++] = 'A';
			buffer[txIndex++] = 'T';
			buffer[txIndex++] = 'T';
			buffer[txIndex++] = 'P';
			buffer[txIndex++] = 0x0d;

			printf("Asking for Temp\r\n");
			LookingForTemp = 1;
			MQTT_SendData(buffer,txIndex);
			++stateCaseTemp;
	   break;

	   case 3:
			if(tempHasBeenRead)
			{
				printf("CurrentTemp:%s\r\n",temperature);
				tempHasBeenRead = 0;
				stateCaseTemp = 0;
			}
			++rxTimeOuttimer;
			if(rxTimeOuttimer > 2)
			{
				stateCaseTemp =  0;
				rxTimeOuttimer = 0;
				receiveOK = 0;
				tempHasBeenRead = 0;
				printf("TimedOut\r\n");
			}
	   break;
	}
}
*/


// Added by Srinivasa on 06/13/12
nodebug void CTU_Comm_Fail()
{
    if(!tCTU_Comm_Fail)
    {
        if(++CTU_Comm_Fail_Timer >= 900)
            tCTU_Comm_Fail = TRUE;
        else
            tCTU_Comm_Fail = FALSE;
    }
    else
        CTU_Comm_Fail_Timer = 0;
}

nodebug void BricknetReceiver(int PortNum, unsigned char inbyte)
{
    char RouteCheck;
    int result;
    int iMaxRegisters, i, j;
    unsigned int BNMRespCmd, BNMTempCnt;
    unsigned int HRword;
    HRfloat HR;
    unsigned char CRC_MSB, CRC_LSB;
    unsigned int BNSDst, bytecount;
    unsigned char HRbyte;
    unsigned long t1;
    if( inbyte == 0xA1 && RxState[PortNum] != 0 )
    {
        BNRxPrevByte[PortNum] = 0xA1;
    }
    else if(inbyte == 0xA2 && RxState[PortNum] != 1 && BNRxPrevByte[PortNum] == 0xA1)
    {
        SPkt[PortNum][0] = 0xA1;
        SPktIndex[PortNum] = 1;
        SPktTmr[PortNum] = 0;
        RxState[PortNum] = 1;
        if(DebugEnable[PortNum]) printf("\r\n^^^^^^^^^^");
    }
    else
        BNRxPrevByte[PortNum] = 0;

    RxPrevState[PortNum] = RxState[PortNum];
    if(RxState[PortNum] == 0)  // Look for first Framing Byte
    {
        SPkt[PortNum][0] = inbyte;
        SPktIndex[PortNum] = 1;
        SPktTmr[PortNum] = 0;
        if(inbyte== 0xA1)
            ++RxState[PortNum]; // Frame One OK
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==1)  // Frame TWO
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        if(inbyte== 0xA2)
            ++RxState[PortNum]; // BrickNet Addr
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==2)  // Message TYPE
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        result = inbyte & 0x02;
        if(result == 0x02)
            ++RxState[PortNum];
        else if(result == 0)
            RxState[PortNum] = 500;
        else
            RxState[PortNum] = 0;
    }
    //**************************** MASTER & REPEATER ****************************
    else if(RxState[PortNum]==3)  // Final Dst (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNMDstMSB[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==4)  // Final Dst (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNRFinalDst[PortNum] = BNMFinalDst[PortNum] = (BNMDstMSB[PortNum] << 8) | inbyte;

        if(UnitAddr[PortNum] == BNMFinalDst[PortNum])   // Added 11/14/08
            ++RxState[PortNum];
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==5)  // Intermediate Src (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==6)  // Intermediate Src (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==7)  // Orig Src (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNMSrcMSB[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==8)  // Orig Src (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNRSrc[PortNum] = BNMSrc[PortNum] = (BNMSrcMSB[PortNum] << 8) | inbyte;  // Dst should be Src
        RouteCheck = FALSE;

        for( i = 0; i < MAXREPEATERROUTES; i++ )
        {
            if(BNMSrc[PortNum] == RoutingTable[PortNum][i][0] || BNMSrc[PortNum] == RoutingTable[PortNum][i][2] || BNMFinalDst[PortNum] == RoutingTable[PortNum][i][0] || BNMFinalDst[PortNum] == RoutingTable[PortNum][i][2])
            {
                RouteCheck = TRUE;
                break;
            }
        }

        if(ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber == BNMSrc[PortNum])
            ++RxState[PortNum];
        else if(RouteCheck == TRUE)
            RxState[PortNum] = 2000;
        else
            RxState[PortNum] = 0;
    }
    //**************************** MASTER ****************************
    else if(RxState[PortNum]==9)  // Path ID
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==10)  // Hop Limit
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==11)  // Transaction ID
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        if(BNMTranxID[PortNum] == inbyte)
            ++RxState[PortNum];
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==12)  // BNSDataGramLen[PortNum] (MS Bytes)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNMDataGramLenMSB[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==13)  // BNSDataGramLen[PortNum] (LS Bytes)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNMDataGramLen[PortNum] = (BNMDataGramLenMSB[PortNum] << 8) | inbyte;
        if(BNMDataGramLen[PortNum] > 512) // Too Many
            RxState[PortNum] = 0;
        else
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==14)  // Type of (CMD) DataGram
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNMRespCmd = inbyte;

        if(BNMRespCmd==0x81) // Error Response
            RxState[PortNum] = 0;
        else if(BNMRespCmd==0xA0)
            RxState[PortNum]=200;
        else if(BNMRespCmd==0x90)
            RxState[PortNum]=300;
        else if(BNMRespCmd==0x82)
            RxState[PortNum]=400;
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum] == 200)  // Read Response (Capture Data)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte; // BNSNumRegs[PortNum] (MS Byte)
        BNMNumRegs[PortNum] = inbyte<<8;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==201)  // BNSNumRegs[PortNum] (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNMNumRegs[PortNum] += inbyte;
        BNMNumRegsCnt[PortNum] = BNMNumRegs[PortNum];
        BNMByteCnt[PortNum] = BNMDataGramLen[PortNum]-3;   // Read Response Size

        if(BNMByteCnt[PortNum] < 0 || BNMNumRegs[PortNum] > 512)
            RxState[PortNum] = 0;
        else
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==202)  // Receive Data
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;

        if(--BNMByteCnt[PortNum] < 1)
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==203)  // Get CRC (MS byte)
    {
        BNMCrcMsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = BNMCrcMsb[PortNum];

        if(ComSession[PortNum].Events[CommEvent[PortNum]].DataType==1) // Bool
            ++RxState[PortNum];
        else if(ComSession[PortNum].Events[CommEvent[PortNum]].DataType > 1 && ComSession[PortNum].Events[CommEvent[PortNum]].DataType < 6) // Int8/16/32/float
            RxState[PortNum]=250; // Analog
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==204)  // CRC_Lo, Check CRC   (LS byte)
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], BNMDataGramLen[PortNum]+14);

        if( DebugEnable[PortNum] ) printf("\r\nDRR %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);

        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))
        {
            BNMTempCnt = 0; // ByteCount
            BNMNumRegsCnt[PortNum] = BNMNumRegs[PortNum];
            while(BNMNumRegsCnt[PortNum] > 0)
            {
                for(j = 0; j < 8 && BNMNumRegsCnt[PortNum] > 0; j++, BNMNumRegsCnt[PortNum]--)  // Bits
                {
                    if(ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == DO_BANK)
                        DoutWrite(ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + 8 * BNMTempCnt + j, (SPkt[PortNum][17+BNMTempCnt]>>j) & 0x01); // Bit Extract
                    else  // Write to DI_BANK
                        DinWrite(ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + 8 * BNMTempCnt + j, (SPkt[PortNum][17+BNMTempCnt]>>j) & 0x01); // Bit Extract
                } // endfor
                ++BNMTempCnt;
            }
            RxState[PortNum] = 0;
            GoodResponse[PortNum] = TRUE;
        }
        else  // CRC FAIL
        {
            RxState[PortNum] = 0;
            BadCRC[PortNum] = TRUE;
        }
    }

    else if(RxState[PortNum]==250) //Read Analog Response
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        crc_value.uWord = CalculateCRC(SPkt[PortNum], BNMDataGramLen[PortNum]+14);

        if( DebugEnable[PortNum] ) printf("\r\nARR %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);

        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) &&   (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            if(ComSession[PortNum].Events[CommEvent[PortNum]].DataType == TYPE_INT16)  // Int16 {Scale like a Modbus Reg : xxx.x}
            {
                if( (ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + BNMNumRegs[PortNum]) > BANKSIZE)
                {
                    RxState[PortNum] = 0;
                    GoodResponse[PortNum] = FALSE;
                    return;
                }
            }
            else  // 32 bit
            {
                if( (ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + BNMNumRegs[PortNum]) > BANKSIZE)
                {
                    RxState[PortNum] = 0;
                    GoodResponse[PortNum] = FALSE;
                    return;
                }
            }

            BNMTempCnt = 0; // ByteCount
            BNMNumRegsCnt[PortNum] = 0;
            while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
            {
                if(ComSession[PortNum].Events[CommEvent[PortNum]].DataType == TYPE_INT16)
                {
                    HRword = SPkt[PortNum][17+BNMTempCnt] << 8 | SPkt[PortNum][18+BNMTempCnt]; // Msb|Lsb
                    BNMTempCnt = BNMTempCnt + 2;
                }
                else  // 32 bit
                {
                    for(i = 0; i < 4; ++i)
                    {
                        HR.i[3-i] = SPkt[PortNum][17+BNMTempCnt];
                        ++BNMTempCnt;
                    }
                } // end datatype

                if((ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AI_SCALED_BANK || ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AI_BANK)
                        && ComSession[PortNum].Events[CommEvent[PortNum]].DataType == TYPE_INT16)
                    AinWrite(ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + BNMNumRegsCnt[PortNum], (float)(HRword * 0.1));
                else if((ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AO_SCALED_BANK || ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AO_BANK)
                        && ComSession[PortNum].Events[CommEvent[PortNum]].DataType == TYPE_INT16)
                    AoutWrite(ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + BNMNumRegsCnt[PortNum], (float)(HRword * 0.1));
                else if((ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AI_BANK || ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AI_SCALED_BANK)
                        && ComSession[PortNum].Events[CommEvent[PortNum]].DataType == TYPE_FLOAT)
                    AinWrite(ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + BNMNumRegsCnt[PortNum], HR.r); // 32bit float
                else if((ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AO_BANK || ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst == AO_SCALED_BANK)
                        && ComSession[PortNum].Events[CommEvent[PortNum]].DataType == TYPE_FLOAT)
                    AoutWrite(ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst + BNMNumRegsCnt[PortNum], HR.r); // 32bit float

                ++BNMNumRegsCnt[PortNum];
            } // End While

            RxState[PortNum] = 0;
            GoodResponse[PortNum] = TRUE;
        }
        else
        {
            RxState[PortNum] = 0;
            BadCRC[PortNum] = TRUE;
        }
    }
    else if(RxState[PortNum]==300)  // Check CRC  ;  High CRC Byte
    {
        BNMCrcMsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = BNMCrcMsb[PortNum];
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==301)  // CRC (LS byte):  Write Response
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        crc_value.uWord = CalculateCRC(SPkt[PortNum], 15);

        if( DebugEnable[PortNum] ) printf("\r\nWR %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);

        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            // Response is OK
            RxState[PortNum] = 0;
            GoodResponse[PortNum] = TRUE;
        }
        else  // CRC FAIL
        {
            RxState[PortNum] = 0;
            BadCRC[PortNum] = TRUE;
        }
    }
    else if(RxState[PortNum]==400)
    {
        BNMCrcMsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = BNMCrcMsb[PortNum];
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==401)  // CRC (LS byte):  Ping Response
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        crc_value.uWord = CalculateCRC(SPkt[PortNum], 15);

        if( DebugEnable[PortNum] ) printf("\r\nPR %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);

        if(  (crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]) )  // CRC Passed
        {
            RxState[PortNum] = 0;
            GoodResponse[PortNum] = TRUE;
        }
        else  // CRC FAIL
        {
            RxState[PortNum] = 0;
            BadCRC[PortNum] = TRUE;
        }
    } // End StateVar
    //**************************** SLAVE & REPEATER ****************************
    else if(RxState[PortNum]== 500)  // Final Dst (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSDstMsb[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==501)  // Final Dst (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNRFinalDst[PortNum] = BNSDst = (BNSDstMsb[PortNum] << 8) | inbyte;
        if(UnitAddr[PortNum] == BNSDst)
            ++RxState[PortNum];
        else
            RxState[PortNum] = 1996; //Could be a response message for message for routing
    }
    //**************************** SLAVE ****************************
    else if(RxState[PortNum]==502)  // Intermediate Dst (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==503)  // Intermediate Dst (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==504)  // Orig Src (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==505)  // Orig Src (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==506)  // Path ID
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==507)  // Hop Limit
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==508)  // Transaction ID
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==509)  // BNSDataGramLen[PortNum] (MS Bytes)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSDataGramLenMsb[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==510)  // BNSDataGramLen[PortNum] (LS Bytes)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;

        BNSDataGramLen[PortNum] = (BNSDataGramLenMsb[PortNum] << 8) | inbyte;
        BNSByteCount[PortNum] = BNSDataGramLen[PortNum] - 8;
        if(BNSDataGramLen[PortNum] > 512) // Too Many
            RxState[PortNum] = 0;
        else
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==511)  // Type of (CMD) DataGram      Beginning of DataGram
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSCmdReq[PortNum] = inbyte;

        if(BNSCmdReq[PortNum]==0x20 || BNSCmdReq[PortNum]==0x10)
            RxState[PortNum]=1200;
        else if(BNSCmdReq[PortNum]==0x02)
            RxState[PortNum]=1400;
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==1200)  // Read : Bank ID (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSReqBankMsb[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1201)  // Bank ID (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSReqBank[PortNum] = (BNSReqBankMsb[PortNum] << 8) | inbyte;
        if( (BNSReqBank[PortNum] >= 1) && (BNSReqBank[PortNum] <= 6) )
            ++RxState[PortNum];
        else if(BNSReqBank[PortNum] == 0x0A)
        {
            RxState[PortNum]=3000;
            //iButtonSerNumEnd[PortNum] = SPktIndex[PortNum]+19;
        }
        else
            RxState[PortNum] = 0;

    }
    else if( RxState[PortNum] == 3000 )
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        /*
              if( SPktIndex[PortNum] >= iButtonSerNumEnd[PortNum] )
              {
                 crc_value.uWord = CalculateCRC(SPkt[PortNum], BNSDataGramLen[PortNum]+14);
                 t1 = MS_TIMER;
                 if((crc_value.bytes.msb == SPkt[PortNum][34]) && (crc_value.bytes.lsb == SPkt[PortNum][35]))
                 {
                    i  = ((SPkt[PortNum][18] << 8) + SPkt[PortNum][19]) / 6;

                    #if IBUTTON_SECURITY
                       if( BNSReqBank[PortNum] == 0x0A ) AddiBSerialNumber( SPkt[PortNum]+22, i+1 );
                    #endif
                    #if ONE_WIRE_BUS
                       if( BNSReqBank[PortNum] == 0x0B ) AddCTDevice( SPkt[PortNum]+22, i );
                    #endif

                    SRespPkt[PortNum][0] = 0xA1;
                    SRespPkt[PortNum][1] = 0xA2;
                    SRespPkt[PortNum][2] = 0x2;
                    SRespPkt[PortNum][3] = SPkt[PortNum][7];
                    SRespPkt[PortNum][4] = SPkt[PortNum][8];
                    SRespPkt[PortNum][5] = SPkt[PortNum][3];
                    SRespPkt[PortNum][6] = SPkt[PortNum][4];
                    SRespPkt[PortNum][7] = SPkt[PortNum][3];
                    SRespPkt[PortNum][8] = SPkt[PortNum][4];
                    SRespPkt[PortNum][9] = SPkt[PortNum][9];
                    SRespPkt[PortNum][10] = SPkt[PortNum][10];
                    SRespPkt[PortNum][11] = SPkt[PortNum][11];
                    SRespPkt[PortNum][12] = 0;
                    SRespPkt[PortNum][13] = 1;
                    SRespPkt[PortNum][14] = 0x90;

                    crc_value.uWord = CalculateCRC(SRespPkt[PortNum], 15);
                    SRespPkt[PortNum][15] = crc_value.bytes.msb;
                    SRespPkt[PortNum][16] = crc_value.bytes.lsb;

                    SRespAvail[PortNum] = TRUE;
                    SRespLen[PortNum] = 17;
                    RxState[PortNum] = 0;
                    ChannelClearTime[PortNum] = abs(MS_TIMER - t1) > 255 ? 255 : abs(MS_TIMER - t1);
                 }
                 else
                    RxState[PortNum] = 0;
              }
        	  */
        RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==1202)  // Data Type
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSDataType[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1203)  // BNSStartAddr[PortNum] (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSStartAddr[PortNum] = inbyte << 8;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1204)  // BNSStartAddr[PortNum] (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSStartAddr[PortNum] += inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1205)  // BNSNumRegs[PortNum] (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSNumRegs[PortNum] = inbyte << 8;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1206)  // BNSNumRegs[PortNum] (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNSNumRegs[PortNum] += inbyte;
        BNSNumRegsCnt[PortNum] = BNSNumRegs[PortNum];

        if(BNSCmdReq[PortNum]==0x10)
            RxState[PortNum]=1300; // Write Command : Goto Recv Data
        else
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1207)  // Get CRC (MS byte)
    {
        BNMCrcMsb[PortNum] = SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;

        if(BNSDataType[PortNum] == 1) // Bool
            ++RxState[PortNum];
        else if(BNSDataType[PortNum] > 1 && BNSDataType[PortNum] < 6) // Int8/16/32/float
            RxState[PortNum]=1250;
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==1208)  // CRC_Lo, Check CRC   (LS byte)
    {
        BNMCrcLsb[PortNum] = inbyte;

        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], 22);

        if( DebugEnable[PortNum] ) printf("\r\nDRC %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);
        t1 = MS_TIMER;
        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            bytecount = 0; // ByteCount
            BNSNumRegsCnt[PortNum] = BNSNumRegs[PortNum];
            while(BNSNumRegsCnt[PortNum] > 0)
            {
                HRbyte = 0;

                for(j = 0; j < 8; ++j)  // Bits
                {
                    if(BNSReqBank[PortNum] == DI_BANK)
                    {
                        if(DinRead(BNSStartAddr[PortNum]+8*bytecount+j))
                            set(&HRbyte, j);
                    }
                    else  // Read DO_BANK
                    {
                        if(DoutRead(BNSStartAddr[PortNum]+8*bytecount+j))
                            set(&HRbyte, j);
                    }

                    if(--BNSNumRegsCnt[PortNum] < 1) break;
                } // endfor
                // Store
                ++bytecount;
                SRespPkt[PortNum][16+bytecount] = HRbyte; // Store Results
            }

            SRespPkt[PortNum][0] = 0xA1;  // Frame
            SRespPkt[PortNum][1] = 0xA2;
            SRespPkt[PortNum][2] = 0x2; //      Type
            SRespPkt[PortNum][3] = SPkt[PortNum][7]; //  Final Dst
            SRespPkt[PortNum][4] = SPkt[PortNum][8];
            SRespPkt[PortNum][5] = SPkt[PortNum][3]; // Inter Dst
            SRespPkt[PortNum][6] = SPkt[PortNum][4];
            SRespPkt[PortNum][7] = SPkt[PortNum][3]; // Orig Src
            SRespPkt[PortNum][8] = SPkt[PortNum][4];
            SRespPkt[PortNum][9] = SPkt[PortNum][9]; // Path ID
            SRespPkt[PortNum][10] = SPkt[PortNum][10]; // Hop Limit
            SRespPkt[PortNum][11] = SPkt[PortNum][11]; // Transaction ID
            SRespPkt[PortNum][14] = 0xA0; // Data Type
            SRespPkt[PortNum][15] = SPkt[PortNum][20];  // Npts Length
            SRespPkt[PortNum][16] = SPkt[PortNum][21];

            // Finished Packing Data
            SRespPkt[PortNum][12] = 0; // DataGram Length
            SRespPkt[PortNum][13] = bytecount + 3;
            // Calulate CRC and Send Data to Master
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], bytecount+17);
            SRespPkt[PortNum][17+bytecount] = crc_value.bytes.msb; // 8/2/99 rev l/m
            SRespPkt[PortNum][18+bytecount] = crc_value.bytes.lsb;
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = bytecount+19;
            RxState[PortNum] = 0;
            ChannelClearTime[PortNum] = abs(MS_TIMER - t1) > 255 ? 255 : abs(MS_TIMER - t1);
            tCTU_Comm_Fail = FALSE;	// Added 06/13/12
            CTU_Comm_Fail_Timer = 0; // Added 06/13/12
        }
        else // End CRC Pass
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==1250)  // ================================= Read Ains
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        crc_value.uWord = CalculateCRC(SPkt[PortNum], 22);

        if( DebugEnable[PortNum] ) printf("\r\nARC %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);
        t1 = MS_TIMER;
        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            if(BNSDataType[PortNum] == TYPE_INT16)  // Int16 {Scale like a Modbus Reg : xxx.x}
            {
                if((BNSStartAddr[PortNum] + BNSNumRegs[PortNum]) > BANKSIZE)
                {
                    RxState[PortNum] = 0;
                    return;
                }
            }
            else  // 32 bit
            {
                if((BNSStartAddr[PortNum] + BNSNumRegs[PortNum]) > BANKSIZE)
                {
                    RxState[PortNum] = 0;
                    return;
                }
            }

            bytecount = 0; // ByteCount
            BNSNumRegsCnt[PortNum] = 0;
            while(BNSNumRegsCnt[PortNum] < BNSNumRegs[PortNum])
            {
                if((BNSReqBank[PortNum] == AI_SCALED_BANK || BNSReqBank[PortNum] == AI_BANK) && BNSDataType[PortNum] == TYPE_INT16)
                    HRword = (unsigned int)(AinRead(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum]) * 10.0); // float to int
                else if((BNSReqBank[PortNum] == AO_SCALED_BANK || BNSReqBank[PortNum] == AO_BANK) && BNSDataType[PortNum] == TYPE_INT16)
                    HRword = (unsigned int)(AoutRead(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum]) * 10.0); // float to int
                else if((BNSReqBank[PortNum] == AI_BANK || BNSReqBank[PortNum] == AI_SCALED_BANK) && BNSDataType[PortNum] == TYPE_FLOAT)
                    HR.r = AinRead(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum]); // 32bit float
                else if((BNSReqBank[PortNum] == AO_BANK || BNSReqBank[PortNum] == AO_SCALED_BANK) && BNSDataType[PortNum] == TYPE_FLOAT)
                    HR.r = AoutRead(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum]); // 32bit float

                if(BNSDataType[PortNum] == TYPE_INT16)  // signed 16bit
                {

                    ++bytecount;
                    SRespPkt[PortNum][16+bytecount] = ((HRword >> 8) & 0xff); // Store High byte
                    ++bytecount;
                    SRespPkt[PortNum][16+bytecount] = (HRword & 0xff); // Store Lo byte
                }
                else // 32 bit
                {
                    for(i = 0; i < 4; ++i)  // Pack to send
                    {
                        ++bytecount;
                        SRespPkt[PortNum][16+bytecount] = HR.i[3-i];
                    }
                }
                ++BNSNumRegsCnt[PortNum];
            } // End While

            SRespPkt[PortNum][0] = 0xA1;  // Frame
            SRespPkt[PortNum][1] = 0xA2;
            SRespPkt[PortNum][2] = 0x2; //      Type
            SRespPkt[PortNum][3] = SPkt[PortNum][7]; //  Final Dst
            SRespPkt[PortNum][4] = SPkt[PortNum][8];
            SRespPkt[PortNum][5] = SPkt[PortNum][3]; // Inter Dst
            SRespPkt[PortNum][6] = SPkt[PortNum][4];
            SRespPkt[PortNum][7] = SPkt[PortNum][3]; // Orig Src
            SRespPkt[PortNum][8] = SPkt[PortNum][4];
            SRespPkt[PortNum][9] = SPkt[PortNum][9]; // Path ID
            SRespPkt[PortNum][10] = SPkt[PortNum][10]; // Hop Limit
            SRespPkt[PortNum][11] = SPkt[PortNum][11]; // Transaction ID
            SRespPkt[PortNum][14] = 0xA0; //  Read Resp.
            SRespPkt[PortNum][15] = SPkt[PortNum][20];  // Npts Length
            SRespPkt[PortNum][16] = SPkt[PortNum][21];
            SRespPkt[PortNum][12] = ((bytecount+3 >> 8) & 0xff); // DataGram Length
            SRespPkt[PortNum][13] = ((bytecount + 3)& 0xff);
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], bytecount+17);
            SRespPkt[PortNum][17+bytecount] = crc_value.bytes.msb; // 8/2/99 rev l/m
            SRespPkt[PortNum][18+bytecount] = crc_value.bytes.lsb;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = bytecount+19;
            RxState[PortNum] = 0;
            ChannelClearTime[PortNum] = abs(MS_TIMER - t1) > 255 ? 255 : abs(MS_TIMER - t1);
            tCTU_Comm_Fail = FALSE;	// Added 06/13/12
            CTU_Comm_Fail_Timer = 0; // Added 06/13/12
        }
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==1300)  // Write: Begin to Recv Data
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;

        if(--BNSByteCount[PortNum] < 1)  // Finished Getting Data
        {
            if(BNSDataType[PortNum]==1) // Bool
                ++RxState[PortNum];
            else if(BNSDataType[PortNum] > 1 && BNSDataType[PortNum] < 6) // Int8/16/32/float
                RxState[PortNum]=1350;
            else
                RxState[PortNum] = 0;
        }

    }
    else if(RxState[PortNum]==1301)  // Get CRC (MS byte)
    {
        BNMCrcMsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = BNMCrcMsb[PortNum];
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1302)  // CRC_Lo, Check CRC     (LS byte)
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], BNSDataGramLen[PortNum]+14);

        if( DebugEnable[PortNum] ) printf("\r\nDWC %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);

        t1 = MS_TIMER;
        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            bytecount = 0; // ByteCount
            BNSNumRegsCnt[PortNum] = BNSNumRegs[PortNum];
            while(BNSNumRegsCnt[PortNum] > 0)
            {
                for(j = 0; j < 8; ++j)  // Bits
                {
                    if(BNSReqBank[PortNum] == DO_BANK)
                        DoutWrite(BNSStartAddr[PortNum]+8*bytecount+j, (SPkt[PortNum][22+bytecount]>>j)&0x01); // Bit Extract
                    else  // Write to DI_BANK
                        DinWrite(BNSStartAddr[PortNum]+8*bytecount+j, (SPkt[PortNum][22+bytecount]>>j)&0x01); // Bit Extract

                    if(--BNSNumRegsCnt[PortNum] < 1) break;
                } // endfor
                ++bytecount;
            }
            SRespPkt[PortNum][0] = 0xA1;  // Frame
            SRespPkt[PortNum][1] = 0xA2;
            SRespPkt[PortNum][2] = 0x2; //      Type
            SRespPkt[PortNum][3] = SPkt[PortNum][7]; //  Final Dst
            SRespPkt[PortNum][4] = SPkt[PortNum][8];
            SRespPkt[PortNum][5] = SPkt[PortNum][3]; // Inter Dst
            SRespPkt[PortNum][6] = SPkt[PortNum][4];
            SRespPkt[PortNum][7] = SPkt[PortNum][3]; // Orig Src
            SRespPkt[PortNum][8] = SPkt[PortNum][4];
            SRespPkt[PortNum][9] = SPkt[PortNum][9]; // Path ID
            SRespPkt[PortNum][10] = SPkt[PortNum][10]; // Hop Limit
            SRespPkt[PortNum][11] = SPkt[PortNum][11]; // Transaction ID
            SRespPkt[PortNum][12] = 0; // DataGram Length
            SRespPkt[PortNum][13] = 1;
            SRespPkt[PortNum][14] = 0x90; // Write Response
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], 15);
            SRespPkt[PortNum][15] = crc_value.bytes.msb;
            SRespPkt[PortNum][16] = crc_value.bytes.lsb;
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 17;
            RxState[PortNum] = 0;
            ChannelClearTime[PortNum] = abs(MS_TIMER - t1) > 255 ? 255 : abs(MS_TIMER - t1);
        }
        else
            RxState[PortNum] = 0;
        tCTU_Comm_Fail = FALSE;	// Added 06/13/12
        CTU_Comm_Fail_Timer = 0; // Added 06/13/12
    }
    else if(RxState[PortNum]==1350)  // ======================================Write Analog
    {
        BNMCrcMsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = BNMCrcMsb[PortNum];
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1351)  // LS CRC byte
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        crc_value.uWord = CalculateCRC(SPkt[PortNum], BNSDataGramLen[PortNum]+14);

        if( DebugEnable[PortNum] ) printf("\r\nAWC %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);

        t1 = MS_TIMER;
        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            if(BNSDataType[PortNum] == TYPE_INT16)  // Int16 {Scale like a Modbus Reg : xxx.x}
            {
                if((BNSStartAddr[PortNum] + BNSNumRegs[PortNum]) > BANKSIZE)
                {
                    RxState[PortNum] = 0;
                    return;
                }
            }
            else  // 32 bit
            {
                if((BNSStartAddr[PortNum] + BNSNumRegs[PortNum]) > BANKSIZE)
                {
                    RxState[PortNum] = 0;
                    return;
                }
            }

            bytecount = 0; // ByteCount
            BNSNumRegsCnt[PortNum] = 0;

            while(BNSNumRegsCnt[PortNum] < BNSNumRegs[PortNum]) // ============= Write Analog to Internal Regs
            {
                if(BNSDataType[PortNum] == TYPE_INT16)
                {
                    HRword = SPkt[PortNum][22+bytecount] << 8 | SPkt[PortNum][23+bytecount]; // Msb|Lsb
                    bytecount = bytecount + 2;
                }
                else  // 32 bit
                {
                    // Unpack
                    for(i=0; i<4; ++i)
                    {
                        HR.i[3-i] = SPkt[PortNum][22+bytecount];
                        ++bytecount;
                    }
                }

                if((BNSReqBank[PortNum] == AI_SCALED_BANK || BNSReqBank[PortNum] == AI_BANK) && BNSDataType[PortNum] == TYPE_INT16)
                    AinWrite(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum], (float)(HRword * 0.1));
                else if((BNSReqBank[PortNum] == AO_SCALED_BANK || BNSReqBank[PortNum] == AO_BANK) && BNSDataType[PortNum] == TYPE_INT16)
                    AoutWrite(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum], (float)(HRword * 0.1));
                else if((BNSReqBank[PortNum] == AI_BANK || BNSReqBank[PortNum] == AI_SCALED_BANK) && BNSDataType[PortNum] == TYPE_FLOAT)
                    AinWrite(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum], HR.r); // 32bit float
                else if((BNSReqBank[PortNum] == AO_BANK || BNSReqBank[PortNum] == AO_SCALED_BANK) && BNSDataType[PortNum] == TYPE_FLOAT)
                    AoutWrite(BNSStartAddr[PortNum] + BNSNumRegsCnt[PortNum], HR.r); // 32bit float

                ++BNSNumRegsCnt[PortNum];
            } // End While

            SRespPkt[PortNum][0] = 0xA1;  // Frame
            SRespPkt[PortNum][1] = 0xA2;
            SRespPkt[PortNum][2] = 0x2; //      Type
            SRespPkt[PortNum][3] = SPkt[PortNum][7]; //  Final Dst
            SRespPkt[PortNum][4] = SPkt[PortNum][8];
            SRespPkt[PortNum][5] = SPkt[PortNum][3]; // Inter Dst
            SRespPkt[PortNum][6] = SPkt[PortNum][4];
            SRespPkt[PortNum][7] = SPkt[PortNum][3]; // Orig Src
            SRespPkt[PortNum][8] = SPkt[PortNum][4];
            SRespPkt[PortNum][9] = SPkt[PortNum][9]; // Path ID
            SRespPkt[PortNum][10] = SPkt[PortNum][10]; // Hop Limit
            SRespPkt[PortNum][11] = SPkt[PortNum][11]; // Transaction ID
            SRespPkt[PortNum][12] = 0; // DataGram Length
            SRespPkt[PortNum][13] = 1;
            SRespPkt[PortNum][14] = 0x90; // Write Response
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], 15);
            SRespPkt[PortNum][15] = crc_value.bytes.msb;
            SRespPkt[PortNum][16] = crc_value.bytes.lsb;
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 17;
            RxState[PortNum] = 0;
            ChannelClearTime[PortNum] = abs(MS_TIMER - t1) > 255 ? 255 : abs(MS_TIMER - t1);
            tCTU_Comm_Fail = FALSE;	// Added 06/13/12
            CTU_Comm_Fail_Timer = 0; // Added 06/13/12
        }
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum]==1400)  // Ping    ; Get CRC (MS byte)
    {
        BNMCrcMsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = BNMCrcMsb[PortNum];
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1401)  // CRC (LS byte):  Ping
    {
        BNMCrcLsb[PortNum] = inbyte;
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        crc_value.uWord = CalculateCRC(SPkt[PortNum], 15);

        if( DebugEnable[PortNum] ) printf("\r\nPC %02x/%02x == %02x/%02x [%ld]", crc_value.bytes.msb, crc_value.bytes.lsb, BNMCrcMsb[PortNum], BNMCrcLsb[PortNum], timesecs);
        t1 = MS_TIMER;
        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            SRespPkt[PortNum][0] = 0xA1;  // Frame
            SRespPkt[PortNum][1] = 0xA2;
            SRespPkt[PortNum][2] = 0x2; //      Type
            SRespPkt[PortNum][3] = SPkt[PortNum][7];   //    Final Dst
            SRespPkt[PortNum][4] = SPkt[PortNum][8];
            SRespPkt[PortNum][5] = SPkt[PortNum][3]; // Inter Dst
            SRespPkt[PortNum][6] = SPkt[PortNum][4];
            SRespPkt[PortNum][7] = SPkt[PortNum][3];   // Orig Src
            SRespPkt[PortNum][8] = SPkt[PortNum][4];
            SRespPkt[PortNum][9] = SPkt[PortNum][9];   // Path ID
            SRespPkt[PortNum][10] = SPkt[PortNum][10]; // Hop Limit
            SRespPkt[PortNum][11] = SPkt[PortNum][11]; // Transaction ID
            SRespPkt[PortNum][12] = 0; // DataGram Length
            SRespPkt[PortNum][13] = 1;
            SRespPkt[PortNum][14] = 0x82; // Data Type

            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], 15);

            SRespPkt[PortNum][15] = crc_value.bytes.msb;
            SRespPkt[PortNum][16] = crc_value.bytes.lsb;
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 17;
            RxState[PortNum] = 0;
            ChannelClearTime[PortNum] = abs(MS_TIMER - t1) > 255 ? 255 : abs(MS_TIMER - t1);
        }
        else
            RxState[PortNum] = 0;
    }
    //**************************** FROM SLAVE (REPEATER CONTD...)****************************
    else if(RxState[PortNum] == 1996)  // Intermediate Src (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 1997)  // Intermediate Src (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 1998)  // Orig Src (MS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNRSrcMsb[PortNum] = inbyte;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 1999)  // Orig Src (LS Byte)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNRSrc[PortNum] = (BNRSrcMsb[PortNum] << 8) | inbyte;  // Dst should be Src
        RouteCheck = FALSE;
        for( i = 0; i < MAXREPEATERROUTES; i++)
        {
            if(BNRSrc[PortNum] == RoutingTable[PortNum][i][0] || BNRSrc[PortNum] == RoutingTable[PortNum][i][2] || BNRFinalDst[PortNum] == RoutingTable[PortNum][i][0] || BNRFinalDst[PortNum] == RoutingTable[PortNum][i][2])
            {
                RouteCheck = TRUE;
                break;
            }
        }

        if(RouteCheck == TRUE) // Message needs to be routed
            RxState[PortNum]++;
        else
            RxState[PortNum] = 0;
    }
    //**************************** FROM MASTER (REPEATER CONTD...)****************************
    else if(RxState[PortNum] == 2000 || RxState[PortNum] == 2001)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        RxState[PortNum]++;
    }
    else if(RxState[PortNum] == 2002) // Trasaction ID
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = TransactionID[PortNum] = inbyte;
        RxState[PortNum]++;
    }
    else if(RxState[PortNum] == 2003)   // ===================== PackLen (MS Bytes)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = BNRPktLenMsb[PortNum] = inbyte;
        RxState[PortNum]++;
    }
    else if(RxState[PortNum] == 2004)// PackLen (LS Bytes)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNRPktLen[PortNum] = (BNRPktLenMsb[PortNum] << 8) | inbyte;
        BNRByteCnt[PortNum] = BNRPktLen[PortNum] + 16;   // 16 = total overhead
        if(BNRPktLen[PortNum] > 512 || BNRPktLen[PortNum] < 1) // Too Many, or Too Few
            RxState[PortNum] = 0;
        else
            RxState[PortNum]++;
    }
    else if(RxState[PortNum] == 2005) // Packet Routing
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = inbyte;
        BNRRoutePkt[PortNum] = FALSE; // New Packet, assume no routing; set TRUE if to route

        // If the Packet is to be Echoed, then no chars are sent to the Slave Port
        //  and the Packet is stored and Echoed out the BricknetMaster Port (where it came in)
        // From This point on, the packet is modified for correct routing paramaters
        // OR,
        // This Packet is not to be Echoed and passed to the RTU Port
        // BN Packet Path Info:  BNRFinalDst[PortNum], RePtr_InterDst, BNRSrc[PortNum]


        if(SPktIndex[PortNum] >= BNRByteCnt[PortNum])  // End of Packet (Full Packet)
        {
            RxState[PortNum] = 0;

            CRC_MSB =  SPkt[PortNum][BNRByteCnt[PortNum]-2];//Get incoming CRC
            CRC_LSB =  SPkt[PortNum][BNRByteCnt[PortNum]-1];//Get incoming CRC

            crc_value.uWord = CalculateCRC(SPkt[PortNum], BNRByteCnt[PortNum]-2);//Calculate new CRC based in the incoming string

            //Check CRC from incoming string to make sure it matches the calculated CRC.
            //If they do not match, skip routing.
//         if(crc_value.bytes.msb != CRC_MSB || crc_value.bytes.lsb != CRC_LSB)
//            goto RouteCheckComplete;//if crc is not valid skip routing

            t1 = MS_TIMER;
            if(crc_value.bytes.msb == CRC_MSB && crc_value.bytes.lsb == CRC_LSB)
            {
                // Determine if Route-to, or Pass-thru
                for(i = 0; i < MAXREPEATERROUTES; i++)
                {
                    if(RoutingTable[PortNum][i][0]==0 || RoutingTable[PortNum][i][1]==0 || RoutingTable[PortNum][i][2]==0)   // End of Table
                        break;

                    if(  (BNRSrc[PortNum] ==  RoutingTable[PortNum][i][0]) && (BNRFinalDst[PortNum] == RoutingTable[PortNum][i][1])  )   // Forward Path to Route
                    {
                        RoutingTable[PortNum][i][3] = TransactionID[PortNum];   // Save for Reverse Packet Ref.

                        U_Word.uWord = RoutingTable[PortNum][i][2];
                        SPkt[PortNum][3] = U_Word.bytes.msb;    // Final Dst
                        SPkt[PortNum][4] = U_Word.bytes.lsb;

                        U_Word.uWord = UnitAddr[PortNum];

                        SPkt[PortNum][5] = U_Word.bytes.msb;  // Inter
                        SPkt[PortNum][6] = U_Word.bytes.lsb;

                        SPkt[PortNum][7] = SPkt[PortNum][5]; // Orig   (same as Inter)
                        SPkt[PortNum][8] = SPkt[PortNum][6];

                        // Calulate New CRC Value
                        crc_value.uWord = CalculateCRC(SPkt[PortNum], BNRByteCnt[PortNum]-2);
                        SPkt[PortNum][BNRByteCnt[PortNum]-2] =  crc_value.bytes.msb;
                        SPkt[PortNum][BNRByteCnt[PortNum]-1] =  crc_value.bytes.lsb;

                        BNRRoutePkt[PortNum]=TRUE;
                        BNRPktLen[PortNum] = BNRByteCnt[PortNum];
                        break;
                    }
                    else if( (BNRFinalDst[PortNum] == UnitAddr[PortNum]) && (TransactionID[PortNum] == RoutingTable[PortNum][i][3]) )  // Repeater RETURN PATH // From Remote RTU (Response)
                    {
                        U_Word.uWord = RoutingTable[PortNum][i][0];
                        SPkt[PortNum][3] = U_Word.bytes.msb;    // Final Dst
                        SPkt[PortNum][4] = U_Word.bytes.lsb;

                        U_Word.uWord = UnitAddr[PortNum];
                        SPkt[PortNum][5] = U_Word.bytes.msb;  // Inter
                        SPkt[PortNum][6] = U_Word.bytes.lsb;

                        U_Word.uWord = RoutingTable[PortNum][i][1];
                        SPkt[PortNum][7] = U_Word.bytes.msb;  // Orig : Source
                        SPkt[PortNum][8] = U_Word.bytes.lsb;


                        crc_value.uWord = CalculateCRC(SPkt[PortNum], BNRByteCnt[PortNum]-2);
                        SPkt[PortNum][BNRByteCnt[PortNum]-2] =  crc_value.bytes.msb;
                        SPkt[PortNum][BNRByteCnt[PortNum]-1] =  crc_value.bytes.lsb;

                        BNRRoutePkt[PortNum]=TRUE;
                        BNRPktLen[PortNum] = BNRByteCnt[PortNum];
                        break;
                    }
                } // next i
                if(BNRRoutePkt[PortNum]) ChannelClearTime[PortNum] = abs(MS_TIMER - t1) > 255 ? 255 : abs(MS_TIMER - t1);
            }
            else
                RxState[PortNum] = 0;
        }
    }
}

nodebug unsigned int PackData(int PortNum, unsigned int index, UINT8 BNSDataType1, unsigned int BNSReqBank1, unsigned int BNSStartAddr1, unsigned int BNSNumRegs1)
{
    // Outstream= Results, starting at Index

    // =======PACK DATA (for Write) =========
    // Datatypes:
    //    TYPE     Supported
    // 1 - Bit      Yes
    // 2 - Int8       No
    // 3 - Int16    Yes
    // 4 - Int32    No
    // 5 - float   Yes
    // 6 - double   No
    // 7 - buffer   No
    unsigned int bytecount, BNSNumRegsCnt1;
    unsigned int i, HRword;
    unsigned char P1_HRbyte;

    if(BNSDataType1==1)
    {
        // Boolean (BIT)
        // Execute Command:   Read From BANK_ID = 1

        bytecount = 0; // ByteCount
        BNSNumRegsCnt1 = BNSNumRegs1;

        while(BNSNumRegsCnt1 > 0)
        {
            P1_HRbyte = 0;
            for(i = 0; i < 8; ++i)
            {
                // Bits
                if(BNSReqBank1 == DI_BANK)
                {
                    if(DinRead(BNSStartAddr1+8*bytecount+i))
                        set(&P1_HRbyte, i);
                }
                else
                {
                    // Read DO_BANK
                    if(DoutRead(BNSStartAddr1+8*bytecount+i))
                        set(&P1_HRbyte, i);
                }

                if(--BNSNumRegsCnt1 < 1) break;
            } // endfor

            // Store
            MPkt[PortNum][index+bytecount] = P1_HRbyte; // Store Results
            ++bytecount;
        }
    }
    else if(BNSDataType1 == TYPE_INT16 || BNSDataType1 == TYPE_FLOAT)
    {
        // Int 16 Analog , or Float
        bytecount = 0; // ByteCount
        BNSNumRegsCnt1 = 0;
        while(BNSNumRegsCnt1 < BNSNumRegs1)
        {
            if((BNSReqBank1 == AI_SCALED_BANK || BNSReqBank1 == AI_BANK) && BNSDataType1 == TYPE_INT16)
                HRword = (int)(AinRead(BNSStartAddr1 + BNSNumRegsCnt1) * 10.0); // 16bit from float
            else if(BNSReqBank1 == AO_SCALED_BANK || BNSReqBank1 == AO_BANK && BNSDataType1 == TYPE_INT16)
                HRword = (int)(AoutRead(BNSStartAddr1 + BNSNumRegsCnt1) * 10.0); // 16bit from float
            else if((BNSReqBank1 == AI_BANK || BNSReqBank1 == AI_SCALED_BANK) && BNSDataType1 == TYPE_FLOAT)
                HR.r = AinRead(BNSStartAddr1 + BNSNumRegsCnt1); // 32bit float
            else if((BNSReqBank1 == AO_BANK || BNSReqBank1 == AO_SCALED_BANK) && BNSDataType1 == TYPE_FLOAT)
                HR.r = AoutRead(BNSStartAddr1 + BNSNumRegsCnt1); // 32bit float
            if(BNSDataType1 == TYPE_INT16)
            {
                // signed 16 bit
                MPkt[PortNum][index+bytecount] = ((HRword >> 8) & 0xff); // Store High byte
                ++bytecount;
                MPkt[PortNum][index+bytecount] = (HRword & 0xff); // Store Lo byte
                ++bytecount;
            }
            else
            {
                // Float

                for(i=0; i<4; ++i)
                {
                    // Pack to send
                    MPkt[PortNum][index+bytecount] = HR.i[3-i];
                    ++bytecount;
                }
            }
            ++BNSNumRegsCnt1;
        }
        //printf("bytecount:%u\r\n",bytecount);
    }
    return bytecount;
}

nodebug int AddBricknetEvent(int PortNum,
                             UINT8 cmd, int unitnumber, UINT8 datatype,
                             unsigned int bankid_src, unsigned int startaddr_src,
                             unsigned int bankid_dst, unsigned int startaddr_dst,
                             unsigned int blocksize, unsigned int pollrate, unsigned int pollratefailed,
                             char FireAtStartup, char FireOnEdgeChange, char EBankID,
                             int ESAddr, int EBSize, char IsDeltaValueAbsolute, float ConstDelta, char DBankID,
                             int DSAddr, int DBSize, char eNum, char sNum
                            )
{
    int m, p;

    if( ActivePortNumber != PortNum ) return;

    if(++ComSession[PortNum].NumEvents < MAXEVENTS)
    {
        // Setting up properties for events
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].Cmd = cmd;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].UnitNumber = unitnumber;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].DataType = datatype;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].BankID_Src = bankid_src;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddr_Src = startaddr_src;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].BankID_Dst = bankid_dst;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddr_Dst = startaddr_dst;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].BlockSize = blocksize;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].PollRate = pollrate; // Secs
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].PollRate_Failed = pollratefailed; // Secs
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].FireAtStartup = FireAtStartup;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].FireOnEdgeChange = FireOnEdgeChange;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].EdgeBankID = EBankID;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].EdgeStartAddr = ESAddr;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].EdgeBlockSize = EBSize;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].IsDeltaValueAbsolute = IsDeltaValueAbsolute;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].ConstDelta = ConstDelta;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].DeltaBankID = DBankID;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].DeltaStartAddr = DSAddr;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].DeltaBlockSize = DBSize;

        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].EventNum = eNum;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].SessionNum = sNum;

        ComSession[PortNum].EventStartUp[ComSession[PortNum].NumEvents] = FireAtStartup;

        // MARK DIFFERENT RTU's TO POLL
        for(m = 0; m < Max_Rtu_Num[PortNum]; m++)
            // If the unit number is already on the list then don't add it again
            if(Unique_RTU_Nums[PortNum][m] == unitnumber) return;

        // if the unit number is not on the list then add it to the list
        Unique_RTU_Nums[PortNum][Max_Rtu_Num[PortNum]++] = unitnumber;
    }
    else	// Exceeded the max allowable number of events
    {
        printf("\r\nExceeded maximum events allowed....Expand MAXEVENTS events...Quitting here....");
        exit(3);
    }
}

nodebug int EdgeTrigger(int PortNum, int BlockNo, int BankID, int StartAddrs, int EndingAddrs)
{
    int tAddrs, start, end;
    char current, previous;

    if( ComSession[PortNum].Triggers.EdgeBlock[BlockNo].DebouceRegNumber )
    {
        tAddrs = ComSession[PortNum].Triggers.EdgeBlock[BlockNo].DebouceRegNumber;
        if(DI_BANK == BankID)
        {
            current = DinRead(tAddrs);
            previous = TDinRead(tAddrs);
        }
        else
        {
            current = DoutRead(tAddrs);
            previous = TDoutRead(tAddrs);
        }

        if(current != previous)
        {
            if(++ComSession[PortNum].Triggers.EdgeBlock[BlockNo].DebouceTimer >= DIGITAL_DEBOUNCE_TIME) // Safe to say input change is stable
            {
                start = (StartAddrs / 100) * 100 + 1;
                end = start + 99;
                //for( tAddrs = StartAddrs; tAddrs <= EndingAddrs; tAddrs++ )
                for( tAddrs = start; tAddrs <= end; tAddrs++ )
                {
                    if(DI_BANK == BankID)
                    {
                        current = DinRead(tAddrs);
                        TDinWrite(tAddrs, current);
                    }
                    else
                    {
                        current = DoutRead(tAddrs);
                        TDoutWrite(tAddrs, current);
                    }
                }
                ComSession[PortNum].Triggers.EdgeBlock[BlockNo].DebouceRegNumber = 0;
                return TRUE;
            }
            return FALSE;
        }
        else
            ComSession[PortNum].Triggers.EdgeBlock[BlockNo].DebouceRegNumber = 0;
    }

    for( tAddrs = StartAddrs; tAddrs <= EndingAddrs; tAddrs++ )
    {
        if(DI_BANK == BankID)
        {
            current = DinRead(tAddrs);
            previous = TDinRead(tAddrs);
        }
        else
        {
            current = DoutRead(tAddrs);
            previous = TDoutRead(tAddrs);
        }

        if(current != previous)
        {
            ComSession[PortNum].Triggers.EdgeBlock[BlockNo].DebouceRegNumber = tAddrs;
            ComSession[PortNum].Triggers.EdgeBlock[BlockNo].DebouceTimer = 0;
            break;
        }
    }
    return FALSE;
}

nodebug int DeltaTrigger(int PortNum, int BlockNo, int BankID, int StartAddrs, int EndingAddrs, float Delta, char isDeltaAbsolute)
{
    int tAddrs, start, end;
    float temp, diff, current, previous;
    temp = diff = 0;
    if( ComSession[PortNum].Triggers.DeltaBlock[BlockNo].DebouceRegNumber )
    {
        tAddrs = ComSession[PortNum].Triggers.DeltaBlock[BlockNo].DebouceRegNumber;

        if(AI_BANK == BankID)
        {
            current = AinRead(tAddrs);
            previous = TAinRead(tAddrs);
        }
        else
        {
            current = AoutRead(tAddrs);
            previous = TAoutRead(tAddrs);
        }

        if(isDeltaAbsolute)
        {
            temp = current - previous;
            temp = temp < 0 ? (temp * -1.0) : temp;
        }
        else
        {
            diff = Delta * 0.01 * ((EndingAddrs <= 4 && tAddrs <= 4) ? ((AI_BANK == BankID) ? AinScaleFactor[tAddrs] : AoutScaleFactor[tAddrs]) : previous);
            if((current > (previous + diff)) || (current < (previous - diff)))
                temp = Delta + 1;
        }

        if(temp > Delta)
        {
            if(++ComSession[PortNum].Triggers.DeltaBlock[BlockNo].DebouceTimer >= ANALOG_DEBOUNCE_TIME) // Safe to say input change is stable
            {
                start = (StartAddrs / 100) * 100 + 1;
                end = start + 99;
                //for( tAddrs = StartAddrs; tAddrs <= EndingAddrs; tAddrs++ )
                for( tAddrs = start; tAddrs <= end; tAddrs++ )
                {
                    if(AI_BANK == BankID)
                    {
                        current = AinRead(tAddrs);
                        TAinWrite(tAddrs, current);
                    }
                    else
                    {
                        current = AoutRead(tAddrs);
                        TAoutWrite(tAddrs, current);
                    }
                }
                ComSession[PortNum].Triggers.DeltaBlock[BlockNo].DebouceRegNumber = 0;
                return TRUE;
            }
            return FALSE;
        }
        else
            ComSession[PortNum].Triggers.DeltaBlock[BlockNo].DebouceRegNumber = 0;
    }

    for( tAddrs = StartAddrs; tAddrs <= EndingAddrs; tAddrs++ )
    {
        if(AI_BANK == BankID)
        {
            current = AinRead(tAddrs);
            previous = TAinRead(tAddrs);
        }
        else
        {
            current = AoutRead(tAddrs);
            previous = TAoutRead(tAddrs);
        }

        if(isDeltaAbsolute)
        {
            temp = current - previous;
            temp = temp < 0 ? (temp * -1.0) : temp;
        }
        else
        {
            diff = Delta * 0.01 * ((EndingAddrs <= 4 && tAddrs <= 4) ? ((AI_BANK == BankID) ? AinScaleFactor[tAddrs] : AoutScaleFactor[tAddrs]) : previous);
            if((current > (previous + diff)) || (current < (previous - diff)))
                temp = Delta + 1;
        }

        if(temp > Delta)
        {
            ComSession[PortNum].Triggers.DeltaBlock[BlockNo].DebouceRegNumber = tAddrs;
            ComSession[PortNum].Triggers.DeltaBlock[BlockNo].DebouceTimer = 0;
            break;
        }
    }
    return FALSE;
}

nodebug void PrepareCmdPacket(int PortNum, int EventNo)
{
    if(EventNo != 0)
        CreateBNMPacket(PortNum, ComSession[PortNum].Events[EventNo].Cmd);
    else
        CreateBNMPacket(PortNum, PING);

}

nodebug void CreateBNMPacket(int PortNum, UINT8 cmd)
{
    MasterPacketReady[PortNum] = FALSE;

    MPkt[PortNum][0]=0xA1;
    MPkt[PortNum][1]=0xA2;
    MPkt[PortNum][2]=0;

    U_Word.uWord = ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber;

    MPkt[PortNum][3]= U_Word.bytes.msb;   // Final Dst
    MPkt[PortNum][4]= U_Word.bytes.lsb;

    U_Word.uWord = UnitAddr[PortNum];

    MPkt[PortNum][5]= U_Word.bytes.msb;   // Inter Dst
    MPkt[PortNum][6]= U_Word.bytes.lsb;

    MPkt[PortNum][7]= U_Word.bytes.msb;   // Src
    MPkt[PortNum][8]= U_Word.bytes.lsb;

    MPkt[PortNum][9]= 0; // Path ID
    MPkt[PortNum][10]= 0;   // Hop count

    if( ++BNMTranxID[PortNum] > 255 )
        BNMTranxID[PortNum] = 1;

    MPkt[PortNum][11]= BNMTranxID[PortNum];

    switch (cmd)   // 0=No action, 1=Ping, 2=Read, 3=Write
    {

    case 1: // Ping   Req
        MPkt[PortNum][12]=0; // Len
        MPkt[PortNum][13]=1;

        MPkt[PortNum][14]=0x02;   // MasterBN Ping

        // Add CRC
        // Calulate New CRC Value
        crc_value.uWord = CalculateCRC(MPkt[PortNum], 15);
        MPkt[PortNum][15] =  crc_value.bytes.msb;
        MPkt[PortNum][16] =  crc_value.bytes.lsb;
        MasterPacketReady[PortNum] = TRUE; // Ready to Send Packet
        MPktLen[PortNum] = 17;
        break;

    case 2: // Read   Req
        MPkt[PortNum][12]=0; // Len
        MPkt[PortNum][13]=8;

        // Packet Start
        MPkt[PortNum][14]=0x20;   // MasterBN Read

        // BankID
        U_Word.uWord = ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Src;

        MPkt[PortNum][15]=U_Word.bytes.msb;   // MasterBN Read
        MPkt[PortNum][16]=U_Word.bytes.lsb;   // MasterBN Read

        // Data Type
        MPkt[PortNum][17]=ComSession[PortNum].Events[CommEvent[PortNum]].DataType;
        // Start Addr
        U_Word.uWord =ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Src;
        MPkt[PortNum][18]=U_Word.bytes.msb;
        MPkt[PortNum][19]=U_Word.bytes.lsb;
        // Num of Pts
        U_Word.uWord =ComSession[PortNum].Events[CommEvent[PortNum]].BlockSize;
        MPkt[PortNum][20]=U_Word.bytes.msb;
        MPkt[PortNum][21]=U_Word.bytes.lsb;

        // Add CRC
        // Calulate New CRC Value
        crc_value.uWord = CalculateCRC(MPkt[PortNum], 22);
        MPkt[PortNum][22] =  crc_value.bytes.msb;
        MPkt[PortNum][23] =  crc_value.bytes.lsb;

        MasterPacketReady[PortNum] = TRUE; // Ready to Send Packet
        MPktLen[PortNum] = 24;

        break;

    case 3: // Write Command (Req)
        MPkt[PortNum][14]=0x10;   // MasterBN Write
        // BankID Dest.
        U_Word.uWord = ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Dst;
        MPkt[PortNum][15]=U_Word.bytes.msb;   // MasterBN Read
        MPkt[PortNum][16]=U_Word.bytes.lsb;   // MasterBN Read

        // Data Type
        MPkt[PortNum][17]=ComSession[PortNum].Events[CommEvent[PortNum]].DataType;

        // Start Addr
        U_Word.uWord =ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Dst;
        MPkt[PortNum][18]=U_Word.bytes.msb;
        MPkt[PortNum][19]=U_Word.bytes.lsb;
        // Num of Pts
        U_Word.uWord =ComSession[PortNum].Events[CommEvent[PortNum]].BlockSize;
        MPkt[PortNum][20]=U_Word.bytes.msb;
        MPkt[PortNum][21]=U_Word.bytes.lsb;

        // =======PACK DATA =========
        DataSize[PortNum] = PackData(PortNum, 22, ComSession[PortNum].Events[CommEvent[PortNum]].DataType,
                                     ComSession[PortNum].Events[CommEvent[PortNum]].BankID_Src,
                                     ComSession[PortNum].Events[CommEvent[PortNum]].StartAddr_Src,
                                     ComSession[PortNum].Events[CommEvent[PortNum]].BlockSize);

        if(DataSize[PortNum] > 0)
        {
            U_Word.uWord = DataSize[PortNum]+8; // Record Packet Length
            MPkt[PortNum][12]=U_Word.bytes.msb; // Len
            MPkt[PortNum][13]=U_Word.bytes.lsb;

            // Add CRC
            // Calulate New CRC Value
            crc_value.uWord = CalculateCRC(MPkt[PortNum], DataSize[PortNum]+22);
            MPkt[PortNum][DataSize[PortNum]+22] = crc_value.bytes.msb;
            MPkt[PortNum][DataSize[PortNum]+23] = crc_value.bytes.lsb;

            MasterPacketReady[PortNum] = TRUE; // Ready to Send Packet
            MPktLen[PortNum] = DataSize[PortNum]+24;
        }
        else
        {
            // Error
            MasterPacketReady[PortNum] = FALSE; // Not Ready to Send Packet
            MPktLen[PortNum] = 0;
        }

        break;
    }
}

nodebug int qsortcmp(int *p, int *q)
{
    return(*p - *q);
}

nodebug void AddBricknetEvents(int i)
{
    int pq, j, k, m, n, bank, bExit, b;
    int LowestStartAddr, HighestEndAddr;
    int StartingAddrs[MAXEVENTS], EndingAddrs[MAXEVENTS], sa, ea, Start, End, s;
    int EventNums[MAXEVENTS], CntEventNums;
    float Delta;
    char isDeltaAbsolute;

    CommEvent[i] = 1;

    ComSession[i].NumEvents = 0;

    for(j = 0; j < MAXEVENTS; j++)
    {
        ComSession[i].Events[j].UnitNumber = 0;
        ComSession[i].Events[j].TimeLastPolledSecs = 0;
        ComSession[i].Events[j].PollRate = 0; // Secs
        ComSession[i].Events[j].PollRate_Failed = 0; // Secs
        ComSession[i].Events[j].Cmd = 0;
        ComSession[i].Events[j].DataType = 0;
        ComSession[i].Events[j].BankID_Src = 0;
        ComSession[i].Events[j].StartAddr_Src = 0;
        ComSession[i].Events[j].BankID_Dst = 0;
        ComSession[i].Events[j].StartAddr_Dst = 0;
        ComSession[i].Events[j].BlockSize = 0;
        ComSession[i].Events[j].FireAtStartup = FALSE;
        ComSession[i].Events[j].FireOnEdgeChange = FALSE;
        ComSession[i].Events[j].EdgeBankID = 0;
        ComSession[i].Events[j].EdgeStartAddr = 0;
        ComSession[i].Events[j].EdgeBlockSize = 0;
        ComSession[i].Events[j].IsDeltaValueAbsolute = TRUE;
        ComSession[i].Events[j].ConstDelta = 0.0; //Kind of FALSE
        ComSession[i].Events[j].DeltaBankID = 0;
        ComSession[i].Events[j].DeltaStartAddr = 0;
        ComSession[i].Events[j].DeltaBlockSize = 0;
        ComSession[i].Events[j].EventNum = 0;
        ComSession[i].Events[j].SessionNum = 0;
        ComSession[i].Events[j].eActive = 1;
        ComSession[i].Events[j].sActive = TRUE;
        ComSession[i].Events[j].EdgeChanged = FALSE;
        ComSession[i].Events[j].DeltaChanged = FALSE;
        ComSession[i].Events[j].TimesUp = FALSE;
        ComSession[i].EventStartUp[j] = FALSE;
    }

    ComSession[i].Triggers.TotalEdgeBlocks = 0;
    for(j = 0; j < TOTAL_EDGE_BLOCKS; j++)
    {
        ComSession[i].Triggers.EdgeBlock[j].BankId = 0;
        ComSession[i].Triggers.DeltaBlock[j].IsDeltaValueAbsolute = 0;
        ComSession[i].Triggers.EdgeBlock[j].Delta = 0;
        ComSession[i].Triggers.EdgeBlock[j].Enabled = 0;
        ComSession[i].Triggers.EdgeBlock[j].End = 0;
        ComSession[i].Triggers.EdgeBlock[j].Start = 0;
        ComSession[i].Triggers.EdgeBlock[j].DebouceTimer = 0;
        ComSession[i].Triggers.EdgeBlock[j].DebouceRegNumber = 0;
        ComSession[i].Triggers.EdgeBlock[j].TotalEvents = 0;
        for(k = 0; k < MAXEVENTS; k++) ComSession[i].Triggers.EdgeBlock[j].EventNums[k] = 0;
    }

    ComSession[i].Triggers.TotalDeltaBlocks = 0;
    for(j = 0; j < TOTAL_DELTA_BLOCKS; j++)
    {
        ComSession[i].Triggers.DeltaBlock[j].BankId = 0;
        ComSession[i].Triggers.DeltaBlock[j].IsDeltaValueAbsolute = 0;
        ComSession[i].Triggers.DeltaBlock[j].Delta = 0;
        ComSession[i].Triggers.DeltaBlock[j].Enabled = 0;
        ComSession[i].Triggers.DeltaBlock[j].End = 0;
        ComSession[i].Triggers.DeltaBlock[j].Start = 0;
        ComSession[i].Triggers.DeltaBlock[j].DebouceTimer = 0;
        ComSession[i].Triggers.DeltaBlock[j].DebouceRegNumber = 0;
        ComSession[i].Triggers.DeltaBlock[j].TotalEvents = 0;
        for(k = 0; k < MAXEVENTS; k++) ComSession[i].Triggers.DeltaBlock[j].EventNums[k] = 0;
    }

    ActivePortNumber = i;
    //Read Digital Inputs from Local RTU
    AddBricknetEvent(SER_PORT1,
                     READ, 		// Cmds: READ, WRITE		  // ACTION
                     2,   // Unit Number							  // FROM WHICH RTU
                     TYPE_CHAR, // datatype							  // WHAT KIND OF DATA
                     1, 1, // Source:  BankID_Src,  Starting Address	  // SRC. RTU BANK ID/START ADDR
                     1, 1, // Dest:  BankID_Dst,  Starting Address	  // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                     48, // Block Size								  // # OF REGISTERS TO READ
                     5,  // Pollrate SECS 				  // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                     PING_TIME, 	 // Failed Probetime SECS				  //
                     TRUE,		 // Fire at Startup
                     FALSE, 		 // Fire on Edge Change
                     1, 			// BankID
                     1, 			//Starting Addr
                     100,			//BlockSize
                     FALSE, 		 // Delta Value absolute change
                     0, 		 // Delta value	(0 - no need to trigger based on delta)
                     0, 			// BankID
                     0, 			//Starting Addr
                     0,			//BlockSize
                     1,		// Event Number
                     1		// Session this event belongs
                    );


    //Read Analog Inputs from Well
    AddBricknetEvent(SER_PORT1,
                     READ, 		// Cmds: READ, WRITE		  // ACTION
                     2,   // Unit Number							  // FROM WHICH RTU
                     TYPE_INT16, // datatype							  // WHAT KIND OF DATA
                     3, 1, // Source:  BankID_Src,  Starting Address	  // SRC. RTU BANK ID/START ADDR
                     3, 1, // Dest:  BankID_Dst,  Starting Address	  // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                     48, // Block Size								  // # OF REGISTERS TO READ
                     5,  // Pollrate SECS 				  // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                     PING_TIME, 	 // Failed Probetime SECS				  //
                     TRUE,		 // Fire at Startup
                     FALSE, 		 // Fire on Edge Change
                     1, 			// BankID
                     1, 			//Starting Addr
                     100,			//BlockSize
                     FALSE, 		 // Delta Value absolute change
                     0, 		 // Delta value	(0 - no need to trigger based on delta)
                     0, 			// BankID
                     0, 			//Starting Addr
                     0,			//BlockSize
                     2,		// Event Number
                     1		// Session this event belongs
                    );

    //Write Digital Inputs from Well3 to Local Tank RTU
    AddBricknetEvent(SER_PORT1,
                     WRITE, 		// Cmds: READ, WRITE		  // ACTION
                     2,   // Unit Number							  // FROM WHICH RTU
                     TYPE_CHAR, // datatype							  // WHAT KIND OF DATA
                     1, 401, // Source:  BankID_Src,  Starting Address	  // SRC. RTU BANK ID/START ADDR
                     1, 401, // Dest:  BankID_Dst,  Starting Address	  // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                     8, // Block Size								  // # OF REGISTERS TO READ
                     5,  // Pollrate SECS 				  // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                     PING_TIME, 	 // Failed Probetime SECS				  //
                     TRUE,		 // Fire at Startup
                     FALSE, 		 // Fire on Edge Change
                     1, 			// BankID
                     1, 			//Starting Addr
                     100,			//BlockSize
                     FALSE, 		 // Delta Value absolute change
                     0, 		 // Delta value	(0 - no need to trigger based on delta)
                     0, 			// BankID
                     0, 			//Starting Addr
                     0,			//BlockSize
                     3,		// Event Number
                     1		// Session this event belongs
                    );

    //Write Digital Inputs from Well5 to Local Tank RTU
    AddBricknetEvent(SER_PORT1,
                     WRITE, 		// Cmds: READ, WRITE		  // ACTION
                     2,   // Unit Number							  // FROM WHICH RTU
                     TYPE_CHAR, // datatype							  // WHAT KIND OF DATA
                     1, 501, // Source:  BankID_Src,  Starting Address	  // SRC. RTU BANK ID/START ADDR
                     1, 501, // Dest:  BankID_Dst,  Starting Address	  // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                     8, // Block Size								  // # OF REGISTERS TO READ
                     5,  // Pollrate SECS 				  // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                     PING_TIME, 	 // Failed Probetime SECS				  //
                     TRUE,		 // Fire at Startup
                     FALSE, 		 // Fire on Edge Change
                     1, 			// BankID
                     1, 			//Starting Addr
                     100,			//BlockSize
                     FALSE, 		 // Delta Value absolute change
                     0, 		 // Delta value	(0 - no need to trigger based on delta)
                     0, 			// BankID
                     0, 			//Starting Addr
                     0,			//BlockSize
                     4,		// Event Number
                     1		// Session this event belongs
                    );

    //Read Analog Inputs from Well
    AddBricknetEvent(SER_PORT1,
                     WRITE, 		// Cmds: READ, WRITE		  // ACTION
                     2,   // Unit Number							  // FROM WHICH RTU
                     TYPE_INT16, // datatype							  // WHAT KIND OF DATA
                     4, 37, // Source:  BankID_Src,  Starting Address	  // SRC. RTU BANK ID/START ADDR
                     4, 37, // Dest:  BankID_Dst,  Starting Address	  // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                     8, // Block Size								  // # OF REGISTERS TO READ
                     5,  // Pollrate SECS 				  // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                     PING_TIME, 	 // Failed Probetime SECS				  //
                     TRUE,		 // Fire at Startup
                     FALSE, 		 // Fire on Edge Change
                     1, 			// BankID
                     1, 			//Starting Addr
                     100,			//BlockSize
                     FALSE, 		 // Delta Value absolute change
                     0, 		 // Delta value	(0 - no need to trigger based on delta)
                     0, 			// BankID
                     0, 			//Starting Addr
                     0,			//BlockSize
                     5,		// Event Number
                     1		// Session this event belongs
                    );

    //Write Forces to the Tank
    AddBricknetEvent(SER_PORT1,
                     WRITE, 		// Cmds: READ, WRITE		  // ACTION
                     2,   // Unit Number							  // FROM WHICH RTU
                     TYPE_CHAR, // datatype							  // WHAT KIND OF DATA
                     2, 7, // Source:  BankID_Src,  Starting Address	  // SRC. RTU BANK ID/START ADDR
                     2, 7, // Dest:  BankID_Dst,  Starting Address	  // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                     4, // Block Size								  // # OF REGISTERS TO READ
                     5,  // Pollrate SECS 				  // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                     PING_TIME, 	 // Failed Probetime SECS				  //
                     TRUE,		 // Fire at Startup
                     FALSE, 		 // Fire on Edge Change
                     1, 			// BankID
                     1, 			//Starting Addr
                     100,			//BlockSize
                     FALSE, 		 // Delta Value absolute change
                     0, 		 // Delta value	(0 - no need to trigger based on delta)
                     0, 			// BankID
                     0, 			//Starting Addr
                     0,			//BlockSize
                     6,		// Event Number
                     1		// Session this event belongs
                    );



    for( bank = 1, ComSession[i].Triggers.TotalEdgeBlocks = 0; bank <= 2; bank++ )
    {
        // Find low, highs
        for(j = 1, LowestStartAddr = BANKSIZE, HighestEndAddr = 0, sa = 0, ea = 0; j <= ComSession[i].NumEvents; j++)
        {
            if( ComSession[i].Events[j].FireOnEdgeChange && ComSession[i].Events[j].EdgeBankID == bank )
            {
                if( LowestStartAddr > ComSession[i].Events[j].EdgeStartAddr ) LowestStartAddr = ComSession[i].Events[j].EdgeStartAddr;
                pq = (ComSession[i].Events[j].EdgeStartAddr + ComSession[i].Events[j].EdgeBlockSize - 1);
                if( HighestEndAddr  < pq ) HighestEndAddr = pq;

                if( !sa ) StartingAddrs[sa++] = ComSession[i].Events[j].EdgeStartAddr;	// First
                if( !ea ) EndingAddrs[ea++] = pq;	// First

                for(k = 0; k < sa; k++) if( StartingAddrs[k] == ComSession[i].Events[j].EdgeStartAddr ) break;
                if( k == sa ) StartingAddrs[sa++] = ComSession[i].Events[j].EdgeStartAddr;

                for(k = 0; k < ea; k++) if( EndingAddrs[k] == pq ) break;
                if( k == ea ) EndingAddrs[ea++] = pq;
            }
        }

        // sort starting
        qsort(StartingAddrs, sa, 2, qsortcmp);
        // sort ending
        qsort(EndingAddrs, ea, 2, qsortcmp);

        for(j = 0, k = 0, Start = LowestStartAddr; k < ea;)
        {
            End = EndingAddrs[k];
            n = ComSession[i].Triggers.TotalEdgeBlocks;
            ComSession[i].Triggers.EdgeBlock[n].Start = Start;
            ComSession[i].Triggers.EdgeBlock[n].End = ( ((j+1) < sa) && StartingAddrs[j+1] > Start && StartingAddrs[j+1] <= End ) ? StartingAddrs[j+1]-1 : End;
            ComSession[i].Triggers.EdgeBlock[n].BankId = bank;
            ComSession[i].Triggers.EdgeBlock[n].TotalEvents = 0;

            for(m = 1; m <= ComSession[i].NumEvents; m++)
            {
                if((ComSession[i].Events[m].EdgeStartAddr <= Start) && ((End <= ComSession[i].Events[m].EdgeStartAddr + ComSession[i].Events[m].EdgeBlockSize - 1)) &&
                        ComSession[i].Events[m].EdgeBankID == bank && ComSession[i].Events[m].FireOnEdgeChange )
                    ComSession[i].Triggers.EdgeBlock[n].EventNums[ComSession[i].Triggers.EdgeBlock[n].TotalEvents++] = m;
            }
            if( ComSession[i].Triggers.EdgeBlock[n].TotalEvents > 0 )
            {
                ComSession[i].Triggers.EdgeBlock[n].Enabled = TRUE;
                ComSession[i].Triggers.EdgeBlock[n].DebouceRegNumber = 0;
                if( ++ComSession[i].Triggers.TotalEdgeBlocks >= TOTAL_EDGE_BLOCKS )
                {
                    printf("\nArray index out of bounds...while adding EDGE blocks...Exiting");
                    exit(1);
                }
            }

            if( ((j+1) < sa) && StartingAddrs[j+1] > Start && StartingAddrs[j+1] <= End )
            {
                j++;
                Start = StartingAddrs[j];
            }
            else
            {
                Start = End + 1;
                while(1)
                {
                    if( StartingAddrs[j+1] <= Start ) j++;
                    else break;
                }
                k++;
            }
        }
    }

    for(j = 0; j < ComSession[i].Triggers.TotalEdgeBlocks; j++)
    {
        printf("\nBank: %d Block[%d]: Start: %d End: %d Events[%d]=>", ComSession[i].Triggers.EdgeBlock[j].BankId, j, ComSession[i].Triggers.EdgeBlock[j].Start,
               ComSession[i].Triggers.EdgeBlock[j].End, ComSession[i].Triggers.EdgeBlock[j].TotalEvents);
        for(k = 0; k < ComSession[i].Triggers.EdgeBlock[j].TotalEvents; k++) printf(" %d", ComSession[i].Triggers.EdgeBlock[j].EventNums[k]);
    }

    for( bank = 3, ComSession[i].Triggers.TotalDeltaBlocks = 0; bank <= 4; bank++ )
    {
        // Find low, highs
        for(j = 1, LowestStartAddr = BANKSIZE, HighestEndAddr = 0, sa = 0, ea = 0; j <= ComSession[i].NumEvents; j++)
        {
            if( ComSession[i].Events[j].ConstDelta && ComSession[i].Events[j].DeltaBankID == bank )
            {
                if( LowestStartAddr > ComSession[i].Events[j].DeltaStartAddr ) LowestStartAddr = ComSession[i].Events[j].DeltaStartAddr;
                pq = (ComSession[i].Events[j].DeltaStartAddr + ComSession[i].Events[j].DeltaBlockSize - 1);
                if( HighestEndAddr  < pq ) HighestEndAddr = pq;

                if( !sa ) StartingAddrs[sa++] = ComSession[i].Events[j].DeltaStartAddr;	// First
                if( !ea ) EndingAddrs[ea++] = pq;	// First

                for(k = 0; k < sa; k++) if( StartingAddrs[k] == ComSession[i].Events[j].DeltaStartAddr ) break;
                if( k == sa ) StartingAddrs[sa++] = ComSession[i].Events[j].DeltaStartAddr;

                for(k = 0; k < ea; k++) if( EndingAddrs[k] == pq ) break;
                if( k == ea ) EndingAddrs[ea++] = pq;
            }
        }

        // sort starting
        qsort(StartingAddrs, sa, 2, qsortcmp);
        // sort ending
        qsort(EndingAddrs, ea, 2, qsortcmp);

        for(j = 0, k = 0, Start = LowestStartAddr; k < ea;)
        {
            End = EndingAddrs[k];
            n = ComSession[i].Triggers.TotalDeltaBlocks;
            ComSession[i].Triggers.DeltaBlock[n].Start = Start;
            ComSession[i].Triggers.DeltaBlock[n].End = ( ((j+1) < sa) && StartingAddrs[j+1] > Start && StartingAddrs[j+1] <= End ) ? StartingAddrs[j+1]-1 : End;
            ComSession[i].Triggers.DeltaBlock[n].BankId = bank;
            ComSession[i].Triggers.DeltaBlock[n].TotalEvents = 0;

            for(m = 1, CntEventNums = 0; m <= ComSession[i].NumEvents; m++)
            {
                if((ComSession[i].Events[m].DeltaStartAddr <= Start) && ((End <= ComSession[i].Events[m].DeltaStartAddr + ComSession[i].Events[m].DeltaBlockSize - 1)) &&
                        ComSession[i].Events[m].DeltaBankID == bank && ComSession[i].Events[m].ConstDelta > 0)
                    EventNums[CntEventNums++] = m;
            }

            for(bExit = FALSE, b = 0; !bExit;)
            {
                for(m = 0, Delta = 0.0; m < CntEventNums; m++)
                {
                    if( EventNums[m] == 0 ) continue;
                    else if( Delta == 0.0 )
                    {
                        Delta = ComSession[i].Events[EventNums[m]].ConstDelta;
                        isDeltaAbsolute = ComSession[i].Events[EventNums[m]].IsDeltaValueAbsolute;
                    }

                    if( Delta == ComSession[i].Events[EventNums[m]].ConstDelta )
                    {
                        ComSession[i].Triggers.DeltaBlock[n].EventNums[ComSession[i].Triggers.DeltaBlock[n].TotalEvents++] = EventNums[m];
                        EventNums[m] = 0;
                        b++;
                    }
                }
                if( b == CntEventNums )bExit = TRUE;
                if( ComSession[i].Triggers.DeltaBlock[n].TotalEvents > 0 )
                {
                    ComSession[i].Triggers.DeltaBlock[n].IsDeltaValueAbsolute = isDeltaAbsolute;
                    ComSession[i].Triggers.DeltaBlock[n].Delta = Delta;
                    ComSession[i].Triggers.DeltaBlock[n].Enabled = TRUE;
                    n = ++ComSession[i].Triggers.TotalDeltaBlocks;
                    // Next block
                    ComSession[i].Triggers.DeltaBlock[n].Start = Start;
                    ComSession[i].Triggers.DeltaBlock[n].End = ( ((j+1) < sa) && StartingAddrs[j+1] > Start && StartingAddrs[j+1] <= End ) ? StartingAddrs[j+1]-1 : End;
                    ComSession[i].Triggers.DeltaBlock[n].BankId = bank;
                    ComSession[i].Triggers.DeltaBlock[n].TotalEvents = 0;
                    if( n >= TOTAL_DELTA_BLOCKS )
                    {
                        printf("\nArray index out of bounds...while adding DELTA blocks...Exiting");
                        exit(1);
                    }
                }
            }

            if( ((j+1) < sa) && StartingAddrs[j+1] > Start && StartingAddrs[j+1] <= End )
            {
                j++;
                Start = StartingAddrs[j];
            }
            else
            {
                Start = End + 1;
                while(1)
                {
                    if( StartingAddrs[j+1] <= Start ) j++;
                    else break;
                }
                k++;
            }
        }
    }
    for(j = 0; j < ComSession[i].Triggers.TotalDeltaBlocks; j++)
    {
        printf("\nBank: %d Block[%d]: Start: %d End: %d IsDeltaAbsolute: %d Delta: %.2f Events[%d]=>", ComSession[i].Triggers.DeltaBlock[j].BankId, j, ComSession[i].Triggers.DeltaBlock[j].Start,
               ComSession[i].Triggers.DeltaBlock[j].End, ComSession[i].Triggers.DeltaBlock[j].IsDeltaValueAbsolute, ComSession[i].Triggers.DeltaBlock[j].Delta, ComSession[i].Triggers.DeltaBlock[j].TotalEvents);
        for(k = 0; k < ComSession[i].Triggers.DeltaBlock[j].TotalEvents; k++) printf(" %d", ComSession[i].Triggers.DeltaBlock[j].EventNums[k]);
    }
}

// Added 01/27/2009
nodebug void DoEvents(int i)
{
    int j, k, m;
    for(k = 0; k < ComSession[i].Triggers.TotalEdgeBlocks; k++)
    {
        if( ComSession[i].Triggers.EdgeBlock[k].Enabled )
        {
            if( EdgeTrigger(i, k, ComSession[i].Triggers.EdgeBlock[k].BankId, ComSession[i].Triggers.EdgeBlock[k].Start,
                            ComSession[i].Triggers.EdgeBlock[k].End) )
                for( m = 0; m < ComSession[i].Triggers.EdgeBlock[k].TotalEvents; m++ )
                {
                    if( ComSession[i].Events[ComSession[i].Triggers.EdgeBlock[k].EventNums[m]].FireOnEdgeChange &&
                            ComSession[i].Events[ComSession[i].Triggers.EdgeBlock[k].EventNums[m]].sActive &&
                            ComSession[i].Events[ComSession[i].Triggers.EdgeBlock[k].EventNums[m]].eActive)
                        ComSession[i].Events[ComSession[i].Triggers.EdgeBlock[k].EventNums[m]].EdgeChanged = TRUE;
                }
        }
    }

    for(k = 0; k < ComSession[i].Triggers.TotalDeltaBlocks; k++)
    {
        if( ComSession[i].Triggers.DeltaBlock[k].Enabled )
        {
            if( DeltaTrigger(i, k, ComSession[i].Triggers.DeltaBlock[k].BankId, ComSession[i].Triggers.DeltaBlock[k].Start,
                             ComSession[i].Triggers.DeltaBlock[k].End, ComSession[i].Triggers.DeltaBlock[k].Delta,
                             ComSession[i].Triggers.DeltaBlock[k].IsDeltaValueAbsolute) )
                for( m = 0; m < ComSession[i].Triggers.DeltaBlock[k].TotalEvents; m++ )
                {
                    if( ComSession[i].Events[ComSession[i].Triggers.DeltaBlock[k].EventNums[m]].ConstDelta &&
                            ComSession[i].Events[ComSession[i].Triggers.DeltaBlock[k].EventNums[m]].sActive &&
                            ComSession[i].Events[ComSession[i].Triggers.DeltaBlock[k].EventNums[m]].eActive)
                        ComSession[i].Events[ComSession[i].Triggers.DeltaBlock[k].EventNums[m]].DeltaChanged = TRUE;
                }
        }
    }

    for( j = 1; j <= ComSession[i].NumEvents; j++ )
    {
        if(ComSession[i].Events[j].sActive && ComSession[i].Events[j].eActive && ComSession[i].Events[j].PollRate && !ComSession[i].Events[j].TimesUp)
        {
            ComSession[i].Events[j].TimesUp = ( ++ComSession[i].Events[j].TimeLastPolledSecs >= ComSession[i].Events[j].PollRate ) ? TRUE : FALSE;
            if(ComSession[i].Events[j].TimeLastPolledSecs >= 65535) ComSession[i].Events[j].TimeLastPolledSecs = 0;
        }
    }
}

nodebug void BricknetMaster(int PortNum)
{
    // If the session is not active or session is active but not the event then skip the event
    if( !ComSession[PortNum].Events[CommEvent[PortNum]].sActive || (ComSession[PortNum].Events[CommEvent[PortNum]].sActive && !ComSession[PortNum].Events[CommEvent[PortNum]].eActive) )
        MasterState[PortNum] = 3;

    switch( MasterState[PortNum] )
    {
    case 0:
        if(Ping[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] &&
                (PingPollTimer[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] >= ComSession[PortNum].Events[CommEvent[PortNum]].PollRate_Failed))
        {
            PrepareCmdPacket(PortNum, 0);
            MasterState[PortNum] = 1;
            //printf("\nPING [%ld] %d\r\n", timesecs, CommEvent[PortNum]);
        }
        else if(!Ping[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] &&
                (ComSession[PortNum].Events[CommEvent[PortNum]].FireAtStartup || ComSession[PortNum].Events[CommEvent[PortNum]].EdgeChanged ||
                 ComSession[PortNum].Events[CommEvent[PortNum]].DeltaChanged || ComSession[PortNum].Events[CommEvent[PortNum]].TimesUp))
        {
            PrepareCmdPacket(PortNum, CommEvent[PortNum]);
            MasterState[PortNum] = 1;
            //printf("\nSend-B: [%ld] %d\r\n", timesecs, CommEvent[PortNum]);
        }
        else
            MasterState[PortNum] = 3;

        if(MasterState[PortNum] == 1)
        {
            ResponseTimer[PortNum] =  0;
            GoodResponse[PortNum] = FALSE;
            BadCRC[PortNum] = FALSE;
        }
        break;

    case 1:
        if( ResponseTimer[PortNum] <= MAX_RESP_TIMEOUT )
        {
            if( GoodResponse[PortNum] )
            {
                RetryCounter[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] = 0;

                if(Ping[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber])
                    Ping[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] = FALSE;
                else
                {
                    ComSession[PortNum].Events[CommEvent[PortNum]].FireAtStartup = FALSE;
                    ComSession[PortNum].Events[CommEvent[PortNum]].EdgeChanged = FALSE;
                    ComSession[PortNum].Events[CommEvent[PortNum]].DeltaChanged = FALSE;
                    ComSession[PortNum].Events[CommEvent[PortNum]].TimesUp = FALSE;
                    ComSession[PortNum].Events[CommEvent[PortNum]].TimeLastPolledSecs = 0;
                }
                MasterState[PortNum] = 3;
            }
            else if( BadCRC[PortNum] )
            {
                MasterState[PortNum] = 2;
                if( DebugEnable[PortNum] )   printf("\r\n##########");
            }
        }
        else
        {
            MasterState[PortNum] = 2;
            if( DebugEnable[PortNum] )   printf("\r\n!!!!!!%u [%ld]!!!!!!", RxState[PortNum], timesecs);
        }
        break;

    case 2:
        if(RetryCounter[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] < MAX_RETRIES)
        {
            RetryCounter[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber]++;
            ComSession[PortNum].Events[CommEvent[PortNum]].TimeLastPolledSecs = 0;
        }
        else
        {
            if(!Ping[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber])
                Ping[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] = TRUE;
            PingPollTimer[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] = 0;
        }
        MasterState[PortNum] = 0;
        break;

    case 3:
        if(++CommEvent[PortNum] > ComSession[PortNum].NumEvents)
            CommEvent[PortNum] = 1;
        MasterState[PortNum] = 0;
        break;
    }
}

// Added on 06/16/2004 - Modified 10/15/09
// By Srinivasa
nodebug void SessionActivateDeactivate( char PortNum, char SesNum, char bVal )
{
    int i, j;
    char AllBelong2SameSes, AllElseDisabled;
    for(i = 1; i <= ComSession[PortNum].NumEvents; i++)
    {
        if( ComSession[PortNum].Events[i].SessionNum == SesNum )
        {
            ComSession[PortNum].Events[i].sActive = bVal;
            ComSession[PortNum].Events[i].FireAtStartup = bVal && ComSession[PortNum].EventStartUp[i];
        }
    }

    for(i = 0; i < ComSession[PortNum].Triggers.TotalEdgeBlocks; i++)
    {
        for(j = 0, AllBelong2SameSes = TRUE, AllElseDisabled = TRUE; j < ComSession[PortNum].Triggers.EdgeBlock[i].TotalEvents; j++)
        {
            if( ComSession[PortNum].Events[ComSession[PortNum].Triggers.EdgeBlock[i].EventNums[j]].SessionNum != SesNum )
            {
                AllBelong2SameSes = FALSE;
                if( ComSession[PortNum].Events[ComSession[PortNum].Triggers.EdgeBlock[i].EventNums[j]].sActive ) AllElseDisabled = FALSE;
            }
        }

        // Enable for edge changes
        if( AllBelong2SameSes ) ComSession[PortNum].Triggers.EdgeBlock[i].Enabled = bVal;
        else if( !AllElseDisabled ) ComSession[PortNum].Triggers.EdgeBlock[i].Enabled = TRUE;
        else ComSession[PortNum].Triggers.EdgeBlock[i].Enabled = bVal;
    }

    for(i = 0; i < ComSession[PortNum].Triggers.TotalDeltaBlocks; i++)
    {
        for(j = 0, AllBelong2SameSes = TRUE, AllElseDisabled = TRUE; j < ComSession[PortNum].Triggers.DeltaBlock[i].TotalEvents; j++)
        {
            if( ComSession[PortNum].Events[ComSession[PortNum].Triggers.DeltaBlock[i].EventNums[j]].SessionNum != SesNum )
            {
                AllBelong2SameSes = FALSE;
                if( ComSession[PortNum].Events[ComSession[PortNum].Triggers.DeltaBlock[i].EventNums[j]].sActive ) AllElseDisabled = FALSE;
            }
        }
        // Enable for delta changes
        if( AllBelong2SameSes ) ComSession[PortNum].Triggers.DeltaBlock[i].Enabled = bVal;
        else if( !AllElseDisabled ) ComSession[PortNum].Triggers.DeltaBlock[i].Enabled = TRUE;
        else ComSession[PortNum].Triggers.DeltaBlock[i].Enabled = bVal;
    }
}

// *********************************** Modbus CRC ******************************
// Modified by Srinivasa on 10/22/03
nodebug UWORD CalculateCRC( unsigned char *msg, unsigned int len)
{
    TUWord crc;
    char bLSB;
    unsigned char i;

    crc.uWord = 0xFFFF;

    while(len--)
    {
        crc.bytes.lsb ^= *msg++;
        for (i = 0; i < 8; i++)
        {
            bLSB = crc.bytes.lsb & 0x01;
            crc.uWord = crc.uWord >> 1;
            crc.uWord &= 0x7FFF;
            if(bLSB) crc.uWord ^= 0xA001;
        }
    }
    return(crc.uWord);
}

nodebug void TotalizeAins()
{
    int i;
    // Normalized Integration, i.e., One GPM -> multiply integration by Full Scale
    for(i = 0; i < 4; i++)
    {
        if(AinRead(i) < 0.0)
            AinWrite(i, 0.0);

        Gals[i] += AinRead(i) * 0.01666667 * msFraction;

        if( Gals[i] > 1000.0)
        {
            // Inc [KGals] and reset counter
            ++KGals[i];
            Gals[i] -= (int)Gals[i];
            if(Gals[i] < 0.0 || Gals[i] > 1000.0)
                Gals[i] = 0.0;
        }

        if( KGals[i] > 1000)
        {
            // Inc [MGals] and reset
            ++MGals[i];
            KGals[i] = 0;
        }
    }
}

nodebug void TotalizeDins()
{
    int i, j;

    // Pump Starts and RunHours (Din "FALSE to TRUE" Counts
    for(i = 2; i < 8; i++)
    {
        // Pump Run Hours (Khours, hours)
        if(DinRead(i))
            HRtimer[i] += msFraction;  // Start with Din13

        if(HRtimer[i] >= 3600.0)
        {
            HRtimer[i] -= 3600.0;
            if(++Hours[i] > 999)
            {
                Hours[i] = 0;
                ++KHours[i];
            }
        }

        // Starts
        if(DinRead(i) && !Din_last[i])
        {
            // Accumulate Starts
            if(++Din_Starts[i] > 999)
            {
                Din_Starts[i] = 0;
                ++Din_KStarts[i];
            }
        }
        Din_last[i] = DinRead(i);
    }

#define STATS_START_ADDR 60

    if(!toggle)
    {
        for(i = STATS_START_ADDR, j = 2; i < STATS_START_ADDR + 14;)
        {
            AinWrite(i++, Din_Starts[j]);
            AinWrite(i++, Din_KStarts[j++]);
        }

        for(j = 2; i < STATS_START_ADDR + 28; )
        {
            AinWrite(i++, Hours[j] + (2.778e-4 * HRtimer[j]));
            AinWrite(i++, KHours[j++]);
        }
    }
}

nodebug void ResetPort(int PortNum)
{
    int i;
    printf("\r\nReset port %d", PortNum);

    if( PortNum == 0 )
    {
        serCwrFlush();
        serCrdFlush();
        serCclose();
    }

    BNMCrcLsb[PortNum] = 0;
    BNRRoutePkt[PortNum] = 0;
    BNRFinalDst[PortNum] = 0;
    BNRSrc[PortNum] = 0;
    BNRSrcMsb[PortNum] = 0;
    TransactionID[PortNum] = 0;
    BNRPktLenMsb[PortNum] = 0;
    BNRByteCnt[PortNum] = 0;
    BNRPktLen[PortNum] = 0;
    BNMDstMSB[PortNum] = 0;
    BNMFinalDst[PortNum] = 0;
    BNMSrcMSB[PortNum] = 0;
    BNMSrc[PortNum] = 0;
    BNMDataGramLenMSB[PortNum] = 0;
    BNMDataGramLen[PortNum] = 0;
    BNMRespCmd[PortNum] = 0;
    BNMNumRegs[PortNum] = 0;
    BNMNumRegsCnt[PortNum] = 0;
    BNSDst[PortNum] = 0;
    BNSDstMsb[PortNum] = 0;
    BNSDataGramLen[PortNum] = 0;
    BNSDataGramLenMsb[PortNum] = 0;
    BNSReqBank[PortNum] = 0;
    BNSReqBankMsb[PortNum] = 0;
    MBMCmdReq[PortNum] = 0;
    MBMStartAddr[PortNum] = 0;
    RxState[PortNum] = 0;
    RxPrevState[PortNum] = 0;
    BNRxPrevByte[PortNum] = 0;
    BNMCrcMsb[PortNum] = 0;
    BNSCmdReq[PortNum] = 0;
    BNSStartAddr[PortNum] = 0;
    BNSNumRegs[PortNum] = 0;
    BNSNumRegsCnt[PortNum] = 0;
    BNSByteCount[PortNum] = 0;
    BNSDataType[PortNum] = 0;
    SPktTmr[PortNum] = 0;
    SRespAvail[PortNum] = 0;
    SPktIndex[PortNum] = 0;
    SRespLen[PortNum] = 0;
    BNMByteCnt[PortNum] = 0;
    BNMTempCnt[PortNum] = 0;
    SendState[PortNum] = 0;
    ChannelClearTime[PortNum] = 0;
    LeadLagTimer[PortNum] = 0;
    OnDelay[PortNum] = 50;
    OffDelay[PortNum] = 10;
    BNMTranxID[PortNum] = 0;
    CommEvent[PortNum] = 0;
    GoodResponse[PortNum] = 0;
    BadCRC[PortNum] = 0;
    MasterPacketReady[PortNum] = 0;

    memset(SRespPkt[PortNum], 0, sizeof(SRespPkt[PortNum]));
    memset(SPkt[PortNum], 0, sizeof(SPkt[PortNum]));
    memset(MPkt[PortNum], 0, sizeof(MPkt[PortNum]));
    memset(Unique_RTU_Nums[PortNum], 0, sizeof(Unique_RTU_Nums[PortNum]));
    memset(Ping[PortNum], 0, sizeof(Ping[PortNum]));
    memset(PingPollTimer[PortNum], 0, sizeof(PingPollTimer[PortNum]));
    memset(RetryCounter[PortNum], 0, sizeof(RetryCounter[PortNum]));
    ResponseTimer[PortNum] = 0;
    MPktLen[PortNum] = 0;
    Max_Rtu_Num[PortNum] = 0;
    ModbusMSecTmr[PortNum] = 0;
    SlaveAlgLkout[PortNum] = 0;
    MasterAlgLkout[PortNum] = 0;
    MasterState[PortNum] = 0;
    DataSize[PortNum] = 0;

    if( SerialPortProtocol[PortNum] == BRICKNET && SerialPortMaster[PortNum] )
        AddBricknetEvents(PortNum);
    else if( SerialPortProtocol[PortNum] == MODBUS && SerialPortMaster[PortNum] )
        AddModbusEvents(PortNum);

    if( PortNum == 0 )
    {
        serCopen( PORT1_BAUD_RATE );
        serCwrFlush();
        serCrdFlush();
        serCparity(PARAM_NOPARITY);
        serCdatabits(PARAM_8BIT);
        serCflowcontrolOff();
        BitWrPortI(PDDR, &PDDRShadow, 1, 4);
    }
    else if( PortNum == 1 )
    {
        serDopen( PORT2_BAUD_RATE );
        serDwrFlush();
        serDrdFlush();
        serDparity(PARAM_NOPARITY);
        serDdatabits(PARAM_8BIT);
        serDflowcontrolOff();
        BitWrPortI(PGDR, &PGDRShadow, 1, 0);
    }
}



// Added by Srinivasa on 04/16/09
nodebug unsigned char CalculateLRC( unsigned char *msg, unsigned int len)
{
    unsigned char uchLRC, i;
    for(uchLRC = 0, i = 0; i < len; i +=2)
        uchLRC += ByteFromASCII(msg + i);
    return((unsigned char)(-((char)uchLRC)));  // ~2s Complement
}

// Added by Srinivasa on 04/15/09
nodebug void ByteToASCII(char *str, unsigned char inbyte)
{
    int i;
    unsigned char nibble;

    for(i = 1, nibble = 0; i >= 0; i--)
    {
        nibble = inbyte & 0x0f;
        str[i] = (nibble < 10) ? (48 + nibble) : (65 + nibble - 10);
        inbyte = inbyte >> 4;
    }
}

// Added by Srinivasa on 04/15/09
nodebug unsigned char ByteFromASCII(char *ascii)
{
    int i;
    char byte;

    for(i = 0, byte = 0; i < 2; i++)
    {
        byte = byte << 4;
        byte += (ascii[i] >= '0' && ascii[i] <= '9') ? (ascii[i] - 48) : (ascii[i] - 65 + 10);
    }
    return byte;
}

// Added by Srinivasa on 04/16/09
nodebug void ModbusASCIIMasterReceiver(int PortNum, char Data, int EventNo)
{
    int i, DstAddr;
    unsigned int Word, Cmd;
    unsigned char HRbyte;

//   printf("\r\nPort %d, State %d %x", PortNum, RxState[PortNum], Data);

    RxPrevState[PortNum] = RxState[PortNum];

    if( RxState[PortNum] == 0 )
    {
        SPkt[PortNum][0] = Data;
        SPktIndex[PortNum] = 1;
        SPktTmr[PortNum] = 0;
        if(Data == 0x3A)
            ++RxState[PortNum];
    }
    else if( RxState[PortNum] == 1 )
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==2)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        if(ByteFromASCII(SPkt[PortNum]+1) == (ComSession[PortNum].Events[EventNo].UnitNumber & 0xFF))
            ++RxState[PortNum];
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum] == 3)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==4)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        Cmd = ByteFromASCII(SPkt[PortNum]+3);
        if(Cmd != ComSession[PortNum].Events[EventNo].Cmd)
        {
            RxState[PortNum] = 0;
            return;
        }

        if(Cmd >=1 && Cmd <= 4) // Read Response
            RxState[PortNum]++;
        else if(Cmd == 5 || Cmd == 6 || Cmd == 15 || Cmd == 16) // Write Single Coil (DO) or  Write Single  HR (AO) Response
            RxState[PortNum] = 100;                              // Write Blocks (DO/AO) Response
        else
            RxState[PortNum]=0; // Command Not Supported
    }
    else if(RxState[PortNum] == 5)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 6)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        BNMByteCnt[PortNum] = ByteFromASCII(SPkt[PortNum]+5);
        BNMByteCnt[PortNum] *= 2;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 7)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        if(--BNMByteCnt[PortNum] < 1)
            ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 8)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 9)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        if(CalculateLRC(SPkt[PortNum]+1, (6 + ByteFromASCII(SPkt[PortNum]+5) * 2) ) == ByteFromASCII(SPkt[PortNum]+(7 + (ByteFromASCII(SPkt[PortNum]+5) * 2))))
        {
            BNMNumRegs[PortNum] = (ComSession[PortNum].Events[EventNo].NumPtsHi << 8) | ComSession[PortNum].Events[EventNo].NumPtsLo;
            DstAddr = ComSession[PortNum].Events[EventNo].StartAddr;
            BNMNumRegsCnt[PortNum] = 0;
            DataSize[PortNum] = 0;

            switch(ComSession[PortNum].Events[EventNo].Cmd)
            {
            case 1:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    HRbyte = ByteFromASCII(SPkt[PortNum]+(7+DataSize[PortNum]));
                    for(i = 0; i < 8; ++i)  // Extract Each Bit
                    {
                        if( ComSession[PortNum].Events[EventNo].BankID_Dst == DO_BANK )
                            DoutWrite(DstAddr + BNMNumRegsCnt[PortNum], (HRbyte >> i) & 0x01);
                        else
                            DinWrite(DstAddr + BNMNumRegsCnt[PortNum], (HRbyte >> i) & 0x01);
                        if(++BNMNumRegsCnt[PortNum] >= BNMNumRegs[PortNum])
                            break;
                    }
                    DataSize[PortNum] += 2;
                }
                break;

            case 2:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    HRbyte = ByteFromASCII(SPkt[PortNum]+(7+DataSize[PortNum]));
                    for(i = 0; i < 8; ++i)  // Extract Each Bit
                    {
                        if( ComSession[PortNum].Events[EventNo].BankID_Dst == DI_BANK )
                            DinWrite(DstAddr + BNMNumRegsCnt[PortNum], (HRbyte >> i) & 0x01);
                        else
                            DoutWrite(DstAddr + BNMNumRegsCnt[PortNum], (HRbyte >> i) & 0x01);
                        if(++BNMNumRegsCnt[PortNum] >= BNMNumRegs[PortNum])
                            break;
                    }
                    DataSize[PortNum] += 2;
                }
                break;

            case 3:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    Word = (ByteFromASCII(SPkt[PortNum]+(7+DataSize[PortNum])) << 8) | ByteFromASCII(SPkt[PortNum]+(9+DataSize[PortNum]));
                    if( ComSession[PortNum].Events[EventNo].BankID_Dst == AO_BANK)
                        iAoutWrite(DstAddr + BNMNumRegsCnt[PortNum], Word);
                    else
                        iAinWrite(DstAddr + BNMNumRegsCnt[PortNum], Word);
                    DataSize[PortNum] += 4;
                    ++BNMNumRegsCnt[PortNum];
                }
                break;

            case 4:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    Word = (ByteFromASCII(SPkt[PortNum]+(7+DataSize[PortNum])) << 8) | ByteFromASCII(SPkt[PortNum]+(9+DataSize[PortNum]));
                    if( ComSession[PortNum].Events[EventNo].BankID_Dst == AI_BANK)
                        iAinWrite(DstAddr + BNMNumRegsCnt[PortNum], Word);
                    else
                        iAoutWrite(DstAddr + BNMNumRegsCnt[PortNum], Word);
                    DataSize[PortNum] += 4;
                    ++BNMNumRegsCnt[PortNum];
                }
                break;
            }
            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
            if( DebugEnable[PortNum] ) printf("\r\nGood modbus slave read response for event %d on port %d ", EventNo, PortNum);
            return;
        }
        else
        {
            printf("\nLRC Read Failed");
            if( DebugEnable[PortNum] ) printf("\r\nbad crc-1 for event %d on port %d ", EventNo, PortNum);
            BadCRC[PortNum] = TRUE;
            RxState[PortNum] = 0;
            return;
        }
    }
    else if(RxState[PortNum] >= 100 && RxState[PortNum] <= 108)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 109)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        RxState[PortNum] = 0;
        if(CalculateLRC(SPkt[PortNum]+1, 12) == ByteFromASCII(SPkt[PortNum]+13))
        {
            //printf("\nLRC Write Resp Passed");
            GoodResponse[PortNum] = TRUE;
            if( DebugEnable[PortNum] ) printf("\r\nGood modbus slave write response for event %d on port %d ", EventNo, PortNum);
            return;
        }
        else
        {
            if( DebugEnable[PortNum] ) printf("\r\nbad crc-2 for event %d on port %d ", EventNo, PortNum);
            BadCRC[PortNum] = TRUE;
            return;
        }
    }
    else
        RxState[PortNum] = 0;
}

// Added by Srinivasa on 04/16/09
nodebug void CreateMBASCIIPacket(int PortNum, int EventNo)
{
    int fnCode, ByteCount, RegCount, TotalRegs, j, Counter;
    unsigned char Byte;
    unsigned int Word, temp;

    fnCode = ComSession[PortNum].Events[EventNo].Cmd;

    MPkt[PortNum][0] = 0x3A;
    ByteToASCII(MPkt[PortNum]+1, ComSession[PortNum].Events[EventNo].UnitNumber & 0xFF);
    ByteToASCII(MPkt[PortNum]+3, ComSession[PortNum].Events[EventNo].Cmd);
    ByteToASCII(MPkt[PortNum]+5, ComSession[PortNum].Events[EventNo].StartAddrHi);
    ByteToASCII(MPkt[PortNum]+7, ComSession[PortNum].Events[EventNo].StartAddrLo);

    if(fnCode >= 1 && fnCode <= 6)
    {
        switch(fnCode)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            ByteToASCII(MPkt[PortNum]+9, ComSession[PortNum].Events[EventNo].NumPtsHi);
            ByteToASCII(MPkt[PortNum]+11, ComSession[PortNum].Events[EventNo].NumPtsLo);
            break;

        case 5:
            temp = ( ComSession[PortNum].Events[EventNo].BankID_Src == DO_BANK ) ?
                   (DoutRead(ComSession[PortNum].Events[EventNo].StartAddr)) :
                   (DinRead(ComSession[PortNum].Events[EventNo].StartAddr)) ;

            ByteToASCII(MPkt[PortNum]+9, temp ? 0xFF : 0x00);
            ByteToASCII(MPkt[PortNum]+11, 0x00);
            break;

        case 6:
            Word = ( ComSession[PortNum].Events[EventNo].BankID_Src == AO_BANK ) ?
                   (int)(AoutRead(ComSession[PortNum].Events[EventNo].StartAddr)) :
                   (int)(AinRead(ComSession[PortNum].Events[EventNo].StartAddr));
            ByteToASCII(MPkt[PortNum]+9, ((Word >> 8) & 0xFF));
            ByteToASCII(MPkt[PortNum]+11, (Word & 0xFF));
            break;
        }

        ByteToASCII(MPkt[PortNum]+13, CalculateLRC(MPkt[PortNum]+1, 12));
        MPkt[PortNum][15] = 0x0D;
        MPkt[PortNum][16] = 0x0A;
        MasterPacketReady[PortNum] = TRUE; // Ready to Send Packet
        MPktLen[PortNum] = 17;
//      printf("\r\nport %d %x %x %x %x %x %x lsb: %x msb: %x ", PortNum, MPkt[PortNum][0], MPkt[PortNum][1], MPkt[PortNum][2], MPkt[PortNum][3],
//      MPkt[PortNum][4], MPkt[PortNum][5], MPkt[PortNum][6], MPkt[PortNum][7]);
    }
    else if(fnCode == 15 || fnCode == 16)
    {
        ByteToASCII(MPkt[PortNum]+9, ComSession[PortNum].Events[EventNo].NumPtsHi);
        ByteToASCII(MPkt[PortNum]+11, ComSession[PortNum].Events[EventNo].NumPtsLo);

        TotalRegs = (ComSession[PortNum].Events[EventNo].NumPtsHi << 8) | ComSession[PortNum].Events[EventNo].NumPtsLo;

        ByteCount = Counter = RegCount = 0;

        if(fnCode == 15)
        {
            while(RegCount < TotalRegs)
            {
                Byte = 0;
                for(j = 0; j < 8; ++j)  // Bits
                {
                    temp = ( ComSession[PortNum].Events[EventNo].BankID_Src == DO_BANK ) ?
                           (DoutRead(ComSession[PortNum].Events[EventNo].StartAddr + 8 * ByteCount + j)) :
                           (DinRead(ComSession[PortNum].Events[EventNo].StartAddr + 8 * ByteCount + j)) ;

                    if( temp )   set(&Byte, j);

                    if( ++RegCount >= TotalRegs) break;
                }
                ByteToASCII(MPkt[PortNum]+(15+Counter), Byte);
                Counter += 2;
                ByteCount++;
            }
        }
        else
        {
            while(RegCount < TotalRegs)
            {
                Word = ( ComSession[PortNum].Events[EventNo].BankID_Src == AO_BANK ) ?
                       (int)(AoutRead(ComSession[PortNum].Events[EventNo].StartAddr + RegCount)) :
                       (int)(AinRead(ComSession[PortNum].Events[EventNo].StartAddr + RegCount)) ;

                ByteToASCII(MPkt[PortNum]+(15+Counter), ((Word >> 8) & 0xFF));
                ByteToASCII(MPkt[PortNum]+(17+Counter), (Word & 0xFF));
                Counter += 4;
                ++RegCount;
            }
        }

        ByteToASCII(MPkt[PortNum]+13, ((Counter / 2) & 0xFF));
        ByteToASCII(MPkt[PortNum]+(15+Counter), CalculateLRC(MPkt[PortNum]+1, 14 + Counter));
        MPkt[PortNum][15+Counter+2] = 0x0D;
        MPkt[PortNum][15+Counter+3] = 0x0A;
        MasterPacketReady[PortNum] = TRUE; // Ready to Send Packet
        MPktLen[PortNum] = 15 + Counter + 4;
//      printf("\r\nCmd %d port %d msb %x lsb %x", fnCode, PortNum, crc.bytes.msb, crc.bytes.lsb);
    }
}

// Added by Srinivasa on 04/15/09
nodebug void ModbusASCIISlave(int PortNum, unsigned char Data)
{
    unsigned int i, j, bytecount, HRword, Counter;
    unsigned char HRbyte;

//   printf("\r\nPort %d, State %d %c", PortNum, RxState[PortNum], Data);

    RxPrevState[PortNum] = RxState[PortNum];

    if( RxState[PortNum] == 0 )
    {
        SPkt[PortNum][0] = Data;
        SPktIndex[PortNum] = 1;
        SPktTmr[PortNum] = 0;
        if(Data == 0x3A)
            ++RxState[PortNum];
    }
    else if( RxState[PortNum] == 1 )
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if( RxState[PortNum] == 2 )
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        if(UnitAddr[PortNum] == ByteFromASCII(SPkt[PortNum]+1))
            ++RxState[PortNum];
        else
            RxState[PortNum] = 0;
    }
    else if(RxState[PortNum] == 3)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 4)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        MBMCmdReq[PortNum] = ByteFromASCII(SPkt[PortNum]+3);
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 5)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 6)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 7)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 8)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        MBMStartAddr[PortNum] = ((ByteFromASCII(SPkt[PortNum]+5) << 8) | ByteFromASCII(SPkt[PortNum]+7)) + 1;
        if(MBMStartAddr[PortNum] >= BANKSIZE)
            RxState[PortNum] = 0;
        else
            ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 9)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 10)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 11)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 12)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        BNMNumRegsCnt[PortNum] = ((ByteFromASCII(SPkt[PortNum]+9) << 8) | ByteFromASCII(SPkt[PortNum]+11));
        if(MBMCmdReq[PortNum] >=1 && MBMCmdReq[PortNum] <= 4) // Read Commands
            RxState[PortNum]=1000;
        else if(MBMCmdReq[PortNum]==5) // Write Single Coil (DO)
            RxState[PortNum]=200;
        else if(MBMCmdReq[PortNum]==6) // Write Single  HR (AO)
            RxState[PortNum]=300;
        else if(MBMCmdReq[PortNum]==15 || MBMCmdReq[PortNum]==16) // Write Blocks (DO/AO)
            RxState[PortNum]=400;
        else
            RxState[PortNum] = 0; // Command Not Supported

        //
        // We need to check for number of registers requested in a read/ number of registers
        // wrote in a write if the command is other than 5 or 6
        //
        if(MBMCmdReq[PortNum] == 1 || MBMCmdReq[PortNum] == 2 || MBMCmdReq[PortNum] == 15)  // Digital
        {
            if(((MBMStartAddr[PortNum]+BNMNumRegsCnt[PortNum]) >= BANKSIZE) || (BNMNumRegsCnt[PortNum] > 2008))     // 2008 = 251 * 8
                RxState[PortNum] = 0;
        }
        else if(MBMCmdReq[PortNum] == 3 || MBMCmdReq[PortNum] == 4 || MBMCmdReq[PortNum] == 16) // Analog
        {
            if(((MBMStartAddr[PortNum]+BNMNumRegsCnt[PortNum]) >= BANKSIZE) || (BNMNumRegsCnt[PortNum] > 125))     // 502 = 251 * 2
                RxState[PortNum] = 0;
        }
    }
    //==================================================Read Cmds
    else if(RxState[PortNum]==1000)  // Get CRC Lo byte
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1001)  // Get CRC Lo byte
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        if( ByteFromASCII(SPkt[PortNum]+13) == CalculateLRC(SPkt[PortNum]+1, 12))   // LRC Passed
        {
            BNMNumRegs[PortNum] = 0;
            bytecount = 0; // ByteCount
            Counter = 0;
            // Read Multiple Points
            if(MBMCmdReq[PortNum]==1 || MBMCmdReq[PortNum]==2)  // Boolean
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    HRbyte = 0;
                    for(j = 0; j < 8; ++j)  // Bits
                    {
                        if(MBMCmdReq[PortNum]==1)  // Dout (Coils)
                        {
                            if(DoutRead(MBMStartAddr[PortNum] + 8 * bytecount + j))
                                set(&HRbyte, j);
                        }
                        else  // Din
                        {
                            if(DinRead(MBMStartAddr[PortNum] + 8 * bytecount + j))
                                set(&HRbyte, j);

                        } // endif

                        if( ++BNMNumRegs[PortNum] >= BNMNumRegsCnt[PortNum])
                            break;

                    } // endfor

                    // Store
                    ByteToASCII(SRespPkt[PortNum]+(7+Counter), HRbyte); // Store Results
                    Counter += 2;
                    ++bytecount;
                } // end while
            }
            else if(MBMCmdReq[PortNum]==3 || MBMCmdReq[PortNum]==4)     //  ============  Read Analog I/O ================
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    if(MBMCmdReq[PortNum] == 3)                                             // Read AO
                        HRword = (int)(AoutRead(MBMStartAddr[PortNum] + BNMNumRegs[PortNum]));
                    else                                                      // Read AI
                        HRword = (int)(AinRead(MBMStartAddr[PortNum] + BNMNumRegs[PortNum]));

                    ByteToASCII(SRespPkt[PortNum]+(7+Counter), ((HRword >> 8) & 0xFF)); // Store Results
                    Counter += 2;
                    ByteToASCII(SRespPkt[PortNum]+(7+Counter), (HRword & 0xFF)); // Store Results
                    Counter += 2;

                    ++BNMNumRegs[PortNum];
                } // end while
            }
            for( i = 0; i < 5; i++ ) SRespPkt[PortNum][i] = SPkt[PortNum][i];

            ByteToASCII(SRespPkt[PortNum]+5, ((Counter / 2) & 0xFF));

            // Calulate LRC
            ByteToASCII(SRespPkt[PortNum]+(7 + Counter), CalculateLRC(SRespPkt[PortNum]+1, (6 + (Counter & 0xFF))));

            SRespPkt[PortNum][7 + Counter + 2] = 0x0D;
            SRespPkt[PortNum][7 + Counter + 3] = 0x0A;
            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 7 + Counter + 4;
            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
        }
        else
        {
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else if(RxState[PortNum]==200)  // Write Single Coil
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==201)  // BNMCrcMsb[PortNum], Check CRC
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        if( ByteFromASCII(SPkt[PortNum]+13) == CalculateLRC(SPkt[PortNum]+1, 12))   // LRC Passed
        {
            if(ByteFromASCII(SPkt[PortNum]+9) == 0xFF) // On
                DoutWrite(MBMStartAddr[PortNum], ON);
            else if(ByteFromASCII(SPkt[PortNum]+9) == 0) // Off
                DoutWrite(MBMStartAddr[PortNum], OFF);
            else
            {
                RxState[PortNum] = 0;
                return;
            }

            for( i = 0; i < 13; i++ ) SRespPkt[PortNum][i] = SPkt[PortNum][i];

            // Calulate LRC
            ByteToASCII(SRespPkt[PortNum]+13, CalculateLRC(SRespPkt[PortNum]+1, 12));
            SRespPkt[PortNum][15] = 0x0D;
            SRespPkt[PortNum][16] = 0x0A;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 17;

            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
        }
        else
        {
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else if(RxState[PortNum]==300)  // Write HR
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==301)  // BNMCrcMsb[PortNum], Check CRC
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        if( ByteFromASCII(SPkt[PortNum]+13) == CalculateLRC(SPkt[PortNum]+1, 12))   // LRC Passed
        {
            HRword = (ByteFromASCII(SPkt[PortNum]+9) << 8) | ByteFromASCII(SPkt[PortNum]+11);
            AoutWrite(MBMStartAddr[PortNum], (float)(HRword * 0.1));

            for( i = 0; i < 13; i++ ) SRespPkt[PortNum][i] = SPkt[PortNum][i];

            // Calulate LRC
            ByteToASCII(SRespPkt[PortNum]+13, CalculateLRC(SRespPkt[PortNum]+1, 12));
            SRespPkt[PortNum][15] = 0x0D;
            SRespPkt[PortNum][16] = 0x0A;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 17;

            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
        }
        else
        {
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else if(RxState[PortNum]==400)  // Write DO or AO
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==401)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        BNMByteCnt[PortNum] = ByteFromASCII(SPkt[PortNum]+13);
        BNMByteCnt[PortNum] *= 2;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==402)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data; //  get data
        if(--BNMByteCnt[PortNum] < 1)
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==403)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==404)
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        if( ByteFromASCII(SPkt[PortNum] + (15 + (ByteFromASCII(SPkt[PortNum]+13) * 2))) == CalculateLRC(SPkt[PortNum]+1, (14 + (ByteFromASCII(SPkt[PortNum]+13) * 2))))   // LRC Passed
        {
            BNMNumRegs[PortNum] = 0;
            Counter = 0;

            if(MBMCmdReq[PortNum]==15)  // Write Multiple DO
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    HRbyte = ByteFromASCII(SPkt[PortNum]+(15+Counter));
                    for(i = 0; (i < 8) && (BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum]); i++, BNMNumRegs[PortNum]++)
                        DoutWrite(MBMStartAddr[PortNum] + BNMNumRegs[PortNum], ((HRbyte >> i) & 0x01));
                    Counter += 2;
                }
            }
            else  // Write Multiple AO
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    HRword = (ByteFromASCII(SPkt[PortNum]+(15+Counter)) << 8 ) | ByteFromASCII(SPkt[PortNum]+(17+Counter));
                    AoutWrite(MBMStartAddr[PortNum] + BNMNumRegs[PortNum], (float)(HRword));
                    Counter += 4;
                    ++BNMNumRegs[PortNum];
                }
            }

            for( i = 0; i < 13; i++ ) SRespPkt[PortNum][i] = SPkt[PortNum][i];

            // Calulate LRC
            ByteToASCII(SRespPkt[PortNum]+13, CalculateLRC(SRespPkt[PortNum]+1, 12));
            SRespPkt[PortNum][15] = 0x0D;
            SRespPkt[PortNum][16] = 0x0A;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 17;

            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
        }
        else
        {
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else
        RxState[PortNum]=0;
}

nodebug void ModbusRTUSlave(int PortNum, char Data)
{
    unsigned int i, j, bytecount, HRword;
    unsigned char HRbyte;

//   printf("\r\nPort %d, State %d %x", PortNum, RxState[PortNum], Data);

    RxPrevState[PortNum] = RxState[PortNum];

    if( RxState[PortNum] == 0 )  // Unit Number
    {
        SPkt[PortNum][0] = Data;
        SPktIndex[PortNum] = 1;
        SPktTmr[PortNum] = 0;
        if(Data == UnitAddr[PortNum])
            ++RxState[PortNum]; // Modbus Addr Matches
    }
    else if(RxState[PortNum] == 1)
    {
        MBMCmdReq[PortNum] = SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==2)  // Start Addr MSB
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==3)  // Start Addr LSB
    {
        MBMStartAddr[PortNum] = ((SPkt[PortNum][2] << 8) | Data) + 1;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        if(MBMStartAddr[PortNum] >= BANKSIZE)
            RxState[PortNum] = 0;
        else
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==4)  // BNSNumRegs1 MSB
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==5)  // BNSNumRegs1 LSB
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        BNMNumRegsCnt[PortNum] = (SPkt[PortNum][4]<<8) | Data;

        if(MBMCmdReq[PortNum] >=1 && MBMCmdReq[PortNum] <= 4) // Read Commands
            RxState[PortNum]=1000;
        else if(MBMCmdReq[PortNum]==5) // Write Single Coil (DO)
            RxState[PortNum]=200;
        else if(MBMCmdReq[PortNum]==6) // Write Single  HR (AO)
            RxState[PortNum]=300;
        else if(MBMCmdReq[PortNum]==15 || MBMCmdReq[PortNum]==16) // Write Blocks (DO/AO)
            RxState[PortNum]=400;
        else
            RxState[PortNum] = 0; // Command Not Supported

        //
        // We need to check for number of registers requested in a read/ number of registers
        // wrote in a write if the command is other than 5 or 6
        //
        if(MBMCmdReq[PortNum] == 1 || MBMCmdReq[PortNum] == 2 || MBMCmdReq[PortNum] == 15)  // Digital
        {
            if(((MBMStartAddr[PortNum]+BNMNumRegsCnt[PortNum]) >= BANKSIZE) || (BNMNumRegsCnt[PortNum] > 2008))     // 2008 = 251 * 8
                RxState[PortNum] = 0;
        }
        else if(MBMCmdReq[PortNum] == 3 || MBMCmdReq[PortNum] == 4 || MBMCmdReq[PortNum] == 16) // Analog
        {
            if(((MBMStartAddr[PortNum]+BNMNumRegsCnt[PortNum]) >= BANKSIZE) || (BNMNumRegsCnt[PortNum] > 125))     // 502 = 251 * 2
                RxState[PortNum] = 0;
        }
    }
    //==================================================Read Cmds
    else if(RxState[PortNum]==1000)  // Get CRC Lo byte
    {
        BNMCrcLsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1001)  // BNMCrcMsb1, Check CRC
    {
        BNMCrcMsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], 6);

        if(  (crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]) )  // CRC Passed
        {
            BNMNumRegs[PortNum] = 0;

            bytecount = 0; // ByteCount
            // Read Multiple Points
            if(MBMCmdReq[PortNum]==1 || MBMCmdReq[PortNum]==2)  // Boolean
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    HRbyte = 0;
                    for(j = 0; j < 8; ++j)  // Bits
                    {
                        if(MBMCmdReq[PortNum]==1)  // Dout (Coils)
                        {
                            if(DoutRead(MBMStartAddr[PortNum] + 8*bytecount + j))
                                set(&HRbyte, j);
                        }
                        else  // Din
                        {
                            if(DinRead(MBMStartAddr[PortNum] + 8 * bytecount + j))
                                set(&HRbyte, j);

                        } // endif

                        if( ++BNMNumRegs[PortNum] >= BNMNumRegsCnt[PortNum])
                            break;

                    } // endfor

                    // Store
                    SRespPkt[PortNum][3+bytecount] = HRbyte; // Store Results
                    ++bytecount;
                } // end while
            }
            else if(MBMCmdReq[PortNum]==3 || MBMCmdReq[PortNum]==4)     //  ============  Read Analog I/O ================
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    if(MBMCmdReq[PortNum] == 3)                                             // Read AO
                        HRword = (int)(AoutRead(MBMStartAddr[PortNum] + BNMNumRegs[PortNum]) * 10);
                    else                                                      // Read AI
                        HRword = (int)(AinRead(MBMStartAddr[PortNum] + BNMNumRegs[PortNum]) * 10);

                    SRespPkt[PortNum][3+bytecount] = (HRword>>8) & 0xFF; // MSB
                    ++bytecount;
                    SRespPkt[PortNum][3+bytecount] = HRword & 0xFF;  // LSB
                    ++bytecount;

                    ++BNMNumRegs[PortNum];
                } // end while
            }
            // Finished Packing Data

            SRespPkt[PortNum][0] = SPkt[PortNum][0]; // Unit
            SRespPkt[PortNum][1] = SPkt[PortNum][1]; // MBMCmdReq[PortNum]
            SRespPkt[PortNum][2] = bytecount;

            // Calulate CRC and Send Data to Master
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], bytecount+3);
            SRespPkt[PortNum][3+bytecount] = crc_value.bytes.lsb;
            SRespPkt[PortNum][4+bytecount] = crc_value.bytes.msb;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = bytecount + 5;
            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
            if( DebugEnable[PortNum] ) printf("\r\nSlave read resp for port %d len %d", PortNum, SRespLen[PortNum]);
        }
        else
        {
            if( DebugEnable[PortNum] ) printf("\r\nBad CRC %x/%x-- %x/%x for slave read resp for port %d", crc_value.bytes.msb, BNMCrcMsb[PortNum], crc_value.bytes.lsb, BNMCrcLsb[PortNum], PortNum);
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else if(RxState[PortNum]==200)  // Write Single Coil
    {
        BNMCrcLsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==201)  // BNMCrcMsb[PortNum], Check CRC
    {
        BNMCrcMsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], 6);

        SPkt[PortNum][SPktIndex[PortNum]++] = crc_value.bytes.lsb;
        SPkt[PortNum][SPktIndex[PortNum]++] = crc_value.bytes.msb;

        if(  (crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]) && (MBMStartAddr[PortNum] < BANKSIZE)  )  // CRC Passed and Valid Start Addr
        {
            ++RxState[PortNum];
            if(SPkt[PortNum][4]==0xFF && SPkt[PortNum][5]==0) // On
                DoutWrite(MBMStartAddr[PortNum], ON);
            else if(SPkt[PortNum][4]==0 && SPkt[PortNum][5]==0) // Off
                DoutWrite(MBMStartAddr[PortNum], OFF);
            else
            {
                BadCRC[PortNum] = TRUE;
                RxState[PortNum]=0;
                return;
            }
            // Finished Packing Data

            SRespPkt[PortNum][0] = SPkt[PortNum][0]; // Unit
            SRespPkt[PortNum][1] = SPkt[PortNum][1]; // MBMCmdReq[PortNum]
            SRespPkt[PortNum][2] = SPkt[PortNum][2]; // Start
            SRespPkt[PortNum][3] = SPkt[PortNum][3]; //    Addr
            SRespPkt[PortNum][4] = SPkt[PortNum][4]; //  Data
            SRespPkt[PortNum][5] = SPkt[PortNum][5]; //   Data


            // Calulate CRC and Send Data to Master
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], 6);
            SRespPkt[PortNum][6] = crc_value.bytes.lsb;
            SRespPkt[PortNum][7] = crc_value.bytes.msb;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 8;

            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
            if( DebugEnable[PortNum] ) printf("\r\nSlave write coil for port %d len %d", PortNum, SRespLen[PortNum]);
        }
        else
        {
            if( DebugEnable[PortNum] ) printf("\r\nBad CRC %x/%x-- %x/%x for coil write on port %d", crc_value.bytes.msb, BNMCrcMsb[PortNum], crc_value.bytes.lsb, BNMCrcLsb[PortNum], PortNum);
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else if(RxState[PortNum]==300)  // Write HR
    {
        BNMCrcLsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==301)  // BNMCrcMsb[PortNum], Check CRC
    {
        BNMCrcMsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], 6);

        SPkt[PortNum][SPktIndex[PortNum]++] = crc_value.bytes.lsb;
        SPkt[PortNum][SPktIndex[PortNum]++] = crc_value.bytes.msb;

        if(  (crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]) && (MBMStartAddr[PortNum] < BANKSIZE)  ) // CRC Passed and Valid Start Addr
        {
            BNMNumRegs[PortNum] = 0;
            bytecount = 0; // ByteCount

            HRword = (SPkt[PortNum][4]<<8) | SPkt[PortNum][5];
            AoutWrite(MBMStartAddr[PortNum], (float)(HRword * 0.1));

            // Finished Packing Data

            SRespPkt[PortNum][0] = SPkt[PortNum][0]; // Unit
            SRespPkt[PortNum][1] = SPkt[PortNum][1]; // MBMCmdReq[PortNum]
            SRespPkt[PortNum][2] = SPkt[PortNum][2]; // Addr
            SRespPkt[PortNum][3] = SPkt[PortNum][3]; // Addr
            SRespPkt[PortNum][4] = SPkt[PortNum][4]; // Val
            SRespPkt[PortNum][5] = SPkt[PortNum][5]; // Val

            // Calulate CRC and Send Data to Master
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], 6);

            SRespPkt[PortNum][6] = crc_value.bytes.lsb;
            SRespPkt[PortNum][7] = crc_value.bytes.msb;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 8;

            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
            if( DebugEnable[PortNum] ) printf("\r\nSlave write word for port %d len %d", PortNum, SRespLen[PortNum]);
        }
        else
        {
            if( DebugEnable[PortNum] ) printf("\r\nBad CRC %x/%x-- %x/%x word write on port %d", crc_value.bytes.msb, BNMCrcMsb[PortNum], crc_value.bytes.lsb, BNMCrcLsb[PortNum], PortNum);
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else if(RxState[PortNum]==400)  // Write DO or AO
    {
        BNMByteCnt[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = BNMByteCnt[PortNum];
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==401)  // Write DO or AO
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data; //  get data
        if(--BNMByteCnt[PortNum] < 1)
            ++RxState[PortNum];
    }
    else if(RxState[PortNum]==402)  // Write DO or AO
    {
        BNMCrcLsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==403)  // BNMCrcMsb[PortNum], Check CRC
    {
        BNMCrcMsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], SPkt[PortNum][6] + 7);

        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) &&   (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            BNMNumRegs[PortNum] = 0;
            bytecount = 0; // ByteCount

            if(MBMCmdReq[PortNum]==15)  // Write Multiple DO
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    for(i = 0; (i < 8) && (BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum]); i++, BNMNumRegs[PortNum]++)
                        DoutWrite(MBMStartAddr[PortNum] + BNMNumRegs[PortNum], (SPkt[PortNum][7+bytecount]>>i) & 0x01);
                    ++bytecount;
                }
            }
            else  // Write Multiple AO
            {
                while(BNMNumRegs[PortNum] < BNMNumRegsCnt[PortNum])
                {
                    HRword = (SPkt[PortNum][7+bytecount]<<8) | SPkt[PortNum][8+bytecount];
                    AoutWrite(MBMStartAddr[PortNum] + BNMNumRegs[PortNum], (float)(HRword * 0.1));
                    bytecount = bytecount + 2;
                    ++BNMNumRegs[PortNum];
                }
            }
            SRespPkt[PortNum][0] = SPkt[PortNum][0]; // Unit
            SRespPkt[PortNum][1] = SPkt[PortNum][1]; // MBMCmdReq[PortNum]
            SRespPkt[PortNum][2] = SPkt[PortNum][2]; // Start Addr
            SRespPkt[PortNum][3] = SPkt[PortNum][3]; //   "    "
            SRespPkt[PortNum][4] = SPkt[PortNum][4]; // Npts
            SRespPkt[PortNum][5] = SPkt[PortNum][5]; //  "

            // Calulate CRC and Send Data to Master
            crc_value.uWord = CalculateCRC(SRespPkt[PortNum], 6);
            SRespPkt[PortNum][6] = crc_value.bytes.lsb;
            SRespPkt[PortNum][7] = crc_value.bytes.msb;

            // Send
            SRespAvail[PortNum] = TRUE;
            SRespLen[PortNum] = 8;

            if( DebugEnable[PortNum] ) printf("\r\nSlave block write resp for port %d len %d", PortNum, SRespLen[PortNum]);
            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
        }
        else
        {
            if( DebugEnable[PortNum] ) printf("\r\nBad CRC %x/%x-- %x/%x for block write on port %d", crc_value.bytes.msb, BNMCrcMsb[PortNum], crc_value.bytes.lsb, BNMCrcLsb[PortNum], PortNum);
            BadCRC[PortNum] = TRUE;
            RxState[PortNum]=0;
        }
    }
    else
        RxState[PortNum]=0;
}

nodebug void AddModbusEvents(int i)
{
    int j;

    CommEvent[i] = 1;

    ComSession[i].NumEvents = 0;

    for( j = 0; j < TOTAL_PORTS; j++ )
    {
        ComSession[i].Events[j].UnitNumber = 0;
        ComSession[i].Events[j].TimeLastPolledSecs = 0;
        ComSession[i].Events[j].PollRate = 0; // Secs
        ComSession[i].Events[j].PollRate_Failed = 0; // Secs
        ComSession[i].Events[j].BankID_Dst = 0;
        ComSession[i].Events[j].BankID_Src = 0;
        ComSession[i].Events[j].Cmd = 0;
        ComSession[i].Events[j].StartAddr = 0; // must match the BankId you are writing to.
    }

    ActivePortNumber = i;

    // PORT 1
    AddModbusEvent(SER_PORT1,
                   READ,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   4, 1, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   4, 1, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   25, // Block Size                          // # OF REGISTERS TO READ
                   1,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   7     // Failed Probetime SECS              //
                  );

    AddModbusEvent(SER_PORT1,
                   READ,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   4, 26, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   4, 26, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   25, // Block Size                          // # OF REGISTERS TO READ
                   1,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   7     // Failed Probetime SECS              //
                  );
    AddModbusEvent(SER_PORT1,
                   READ,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   4, 1, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   4, 51, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   25, // Block Size                          // # OF REGISTERS TO READ
                   1,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   7     // Failed Probetime SECS              //
                  );

    AddModbusEvent(SER_PORT1,
                   READ,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   4, 26, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   4, 76, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   25, // Block Size                          // # OF REGISTERS TO READ
                   1,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   7     // Failed Probetime SECS              //
                  );

    // PORT 2
    AddModbusEvent(SER_PORT2,
                   READ,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   1,1, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   1,385, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   96, // Block Size                          // # OF REGISTERS TO READ
                   5,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   10     // Failed Probetime SECS              //
                  );

    AddModbusEvent(SER_PORT2,
                   READ,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   3,31, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   3,401, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   20, // Block Size                          // # OF REGISTERS TO READ
                   5,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   10     // Failed Probetime SECS              //
                  );

    AddModbusEvent(SER_PORT2,
                   WRITE,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   1,1, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   2,1, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   1, // Block Size                          // # OF REGISTERS TO READ
                   5,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   10     // Failed Probetime SECS              //
                  );

    AddModbusEvent(SER_PORT2,
                   WRITE,       // Cmds: READ, WRITE        // ACTION
                   2,   // Unit Number                       // FROM WHICH RTU
                   3,1, // Source:  BankID_Src,  Starting Address     // SRC. RTU BANK ID/START ADDR
                   4,11, // Dest:  BankID_Dst,  Starting Address     // WHERE TO KEEP IN OUR MEMORY (BANK ID/START ADDR)
                   1, // Block Size                          // # OF REGISTERS TO READ
                   5,  // Pollrate SECS               // FOR EVERY HOW MANY # OF SECONDS, TRIGGER THIS EVENT
                   10     // Failed Probetime SECS              //
                  );
}

nodebug int AddModbusEvent(int PortNum, UINT8 cmd, int SlaveID, unsigned int bankid_src,  unsigned int startaddr_src, unsigned int bankid_dst,
                           unsigned int startaddr_dst, unsigned int blocksize, unsigned int pollrate, unsigned int pollratefailed)
{
    auto int m, FnCode;
    auto unsigned int Hi, Lo;

    if( ActivePortNumber != PortNum ) return;

    if(++ComSession[PortNum].NumEvents < MAXEVENTS)
    {
        //  ASSIGN FUNCTION CODES BASED ON cmd, bank, and blocksize
        if(cmd == READ)
        {
            switch(bankid_src)
            {
            case DO_BANK:
                FnCode = 0x01;
                break;
            case DI_BANK:
                FnCode = 0x02;
                break;
            case AO_BANK:
                FnCode = 0x03;
                break;
            case AI_BANK:
                FnCode = 0x04;
                break;
            }
        }
        else if(cmd == WRITE)
        {
            switch(bankid_dst)
            {
            case DO_BANK:
                FnCode = (blocksize == 1) ? 0x05 : 0x0F;
                break;
            case AO_BANK:
                FnCode = (blocksize == 1) ? 0x06 : 0x10;
                break;
            }
        }

        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].UnitNumber = SlaveID;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].Cmd = FnCode;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].BankID_Dst = bankid_dst;
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].BankID_Src = bankid_src;

        if(FnCode >= 1 && FnCode <= 4)
        {
            startaddr_src -= 1; //to make it compatible with Modbus protocol

            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddrHi = (startaddr_src >> 8) & 0xFF;
            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddrLo =  startaddr_src & 0xFF;

            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].NumPtsHi = (blocksize >> 8) & 0xFF;
            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].NumPtsLo =  blocksize & 0xFF;

            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddr = startaddr_dst;
        }
        else if(FnCode == 5 || FnCode == 6 || FnCode == 15 || FnCode == 16)
        {
            startaddr_dst -= 1; //to make it compatible with Modbus protocol

            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddrHi = (startaddr_dst >> 8) & 0xFF;
            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddrLo =  startaddr_dst & 0xFF;

            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].NumPtsHi = (blocksize >> 8) & 0xFF;
            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].NumPtsLo =  blocksize & 0xFF;

            ComSession[PortNum].Events[ComSession[PortNum].NumEvents].StartAddr = startaddr_src;
        }
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].PollRate = pollrate; // Secs
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].PollRate_Failed = pollratefailed; // Secs
        ComSession[PortNum].Events[ComSession[PortNum].NumEvents].TimeLastPolledSecs = 0;
    }
    // Exceeded the max allowable number of events
    else
    {
        printf("\r\nExceeded maximum events allowed....Expand MAXEVENTS events...Quitting here....");
        exit(2);
    }
}

nodebug void CreateMBRTUPacket(int PortNum, int EventNo)
{
    TUWord crc;
    int fnCode, ByteCount, RegCount, TotalRegs, j;
    unsigned char Byte;
    unsigned int Word, temp;

    fnCode = ComSession[PortNum].Events[EventNo].Cmd;

    MPkt[PortNum][0] = ComSession[PortNum].Events[EventNo].UnitNumber & 0xFF;
    MPkt[PortNum][1] = ComSession[PortNum].Events[EventNo].Cmd;
    MPkt[PortNum][2] = ComSession[PortNum].Events[EventNo].StartAddrHi;
    MPkt[PortNum][3] = ComSession[PortNum].Events[EventNo].StartAddrLo;

    if(fnCode >= 1 && fnCode <= 6)
    {
        switch(fnCode)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            MPkt[PortNum][4] = ComSession[PortNum].Events[EventNo].NumPtsHi;
            MPkt[PortNum][5] = ComSession[PortNum].Events[EventNo].NumPtsLo;
            break;

        case 5:
            temp = ( ComSession[PortNum].Events[EventNo].BankID_Src == DO_BANK ) ?
                   (DoutRead(ComSession[PortNum].Events[EventNo].StartAddr)) :
                   (DinRead(ComSession[PortNum].Events[EventNo].StartAddr)) ;

            if( temp )
            {
                MPkt[PortNum][4] = 0xFF;
                MPkt[PortNum][5] = 0x00;
            }
            else
            {
                MPkt[PortNum][4] = 0x00;
                MPkt[PortNum][5] = 0x00;
            }
            break;

        case 6:
            Word = ( ComSession[PortNum].Events[EventNo].BankID_Src == AO_BANK ) ?
                   (int)(AoutRead(ComSession[PortNum].Events[EventNo].StartAddr) * 10) :
                   (int)(AinRead(ComSession[PortNum].Events[EventNo].StartAddr) * 10);
            MPkt[PortNum][4] = (Word >> 8) & 0xFF;    //Hi
            MPkt[PortNum][5] =  Word & 0xFF;       //Lo
            break;
        }

        crc.uWord = CalculateCRC(MPkt[PortNum], 6);
        MPkt[PortNum][6] = crc.bytes.lsb;
        MPkt[PortNum][7] = crc.bytes.msb;
        MasterPacketReady[PortNum] = TRUE; // Ready to Send Packet
        MPktLen[PortNum] = 8;
//      printf("\r\nport %d %x %x %x %x %x %x lsb: %x msb: %x ", PortNum, MPkt[PortNum][0], MPkt[PortNum][1], MPkt[PortNum][2], MPkt[PortNum][3],
//      MPkt[PortNum][4], MPkt[PortNum][5], MPkt[PortNum][6], MPkt[PortNum][7]);
    }
    else if(fnCode == 15 || fnCode == 16)
    {
        MPkt[PortNum][4] = ComSession[PortNum].Events[EventNo].NumPtsHi;
        MPkt[PortNum][5] = ComSession[PortNum].Events[EventNo].NumPtsLo;

        TotalRegs = MPkt[PortNum][4] << 8 | MPkt[PortNum][5];

        ByteCount = RegCount = 0;

        if(fnCode == 15)
        {
            while(RegCount < TotalRegs)
            {
                Byte = 0;
                for(j = 0; j < 8; ++j)  // Bits
                {
                    temp = ( ComSession[PortNum].Events[EventNo].BankID_Src == DO_BANK ) ?
                           (DoutRead(ComSession[PortNum].Events[EventNo].StartAddr + 8 * ByteCount + j)) :
                           (DinRead(ComSession[PortNum].Events[EventNo].StartAddr + 8 * ByteCount + j)) ;
                    if( temp )
                        set(&Byte, j);

                    if( ++RegCount >= TotalRegs)
                        break;

                }
                MPkt[PortNum][7 + ByteCount] = Byte; // Store Results
                ++ByteCount;
            }
        }
        else
        {
            while(RegCount < TotalRegs)
            {
                Word = ( ComSession[PortNum].Events[EventNo].BankID_Src == AO_BANK ) ?
                       (int)(AoutRead(ComSession[PortNum].Events[EventNo].StartAddr + RegCount) * 10) :
                       (int)(AinRead(ComSession[PortNum].Events[EventNo].StartAddr + RegCount) * 10) ;

                MPkt[PortNum][7+ByteCount] = (Word >> 8) & 0xFF; // MSB
                ++ByteCount;
                MPkt[PortNum][7+ByteCount] = Word & 0xFF;  // LSB
                ++ByteCount;
                ++RegCount;
            }
        }

        MPkt[PortNum][6] = ByteCount;
        crc.uWord = CalculateCRC(MPkt[PortNum], 7+ByteCount);
        MPkt[PortNum][7+ByteCount] = crc.bytes.lsb;
        MPkt[PortNum][8+ByteCount] = crc.bytes.msb;
        MasterPacketReady[PortNum] = TRUE; // Ready to Send Packet
        MPktLen[PortNum] = 9 + ByteCount;
//      printf("\r\nCmd %d port %d msb %x lsb %x", fnCode, PortNum, crc.bytes.msb, crc.bytes.lsb);
    }
}

nodebug void ModbusMaster(int PortNum)
{
    int i;
    switch( MasterState[PortNum] )
    {
    case 0:
        for( i = 1; i <= ComSession[PortNum].NumEvents; i++ ) ComSession[PortNum].Events[i].TimeLastPolledSecs += ModbusMSecTmr[PortNum];
        ModbusMSecTmr[PortNum] = 0;
        if(ComSession[PortNum].Events[CommEvent[PortNum]].TimeLastPolledSecs >= ComSession[PortNum].Events[CommEvent[PortNum]].PollRate)
        {
#if MODBUS_PROTOCOL == MODBUS_ASCII
            CreateMBASCIIPacket(PortNum, CommEvent[PortNum]);
#else
            CreateMBRTUPacket(PortNum, CommEvent[PortNum]);
#endif
            MasterState[PortNum] = 1;
            if( DebugEnable[PortNum] )
                printf("\nSend-M: [%ld] %d %u %u port: %d", timesecs, CommEvent[PortNum], ComSession[PortNum].Events[CommEvent[PortNum]].TimeLastPolledSecs, ComSession[PortNum].Events[CommEvent[PortNum]].PollRate, PortNum);
        }
        else
            MasterState[PortNum] = 3;

        if(MasterState[PortNum] == 1)
        {
            ResponseTimer[PortNum] =  0;
            GoodResponse[PortNum] = FALSE;
            BadCRC[PortNum] = FALSE;
        }
        break;

    case 1:
        if( ResponseTimer[PortNum] <= MAX_RESP_TIMEOUT )
        {
            if( GoodResponse[PortNum] )
            {
                RetryCounter[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] = 0;
                ComSession[PortNum].Events[CommEvent[PortNum]].TimeLastPolledSecs = 0;
                MasterState[PortNum] = 3;
            }
            else if( BadCRC[PortNum] )
            {
                MasterState[PortNum] = 2;
                if( DebugEnable[PortNum] )   printf("\r\n##########");
            }
        }
        else
        {
            MasterState[PortNum] = 2;
            if( DebugEnable[PortNum] )   printf("\r\n!!!!!!%u [%ld]!!!!!!", RxState[PortNum], timesecs);
        }
        break;

    case 2:
        if(RetryCounter[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] < MAX_RETRIES)
        {
            if( DebugEnable[PortNum] ) printf("\r\nModbus RETRY on port %d", PortNum);
            RetryCounter[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber]++;
            MasterState[PortNum] = 0;
        }
        else
        {
            RetryCounter[PortNum][ComSession[PortNum].Events[CommEvent[PortNum]].UnitNumber] = 0;
            MasterState[PortNum] = 3;
        }
        ComSession[PortNum].Events[CommEvent[PortNum]].TimeLastPolledSecs = 0;
        break;

    case 3:
        if(++CommEvent[PortNum] > ComSession[PortNum].NumEvents)
            CommEvent[PortNum] = 1;
        MasterState[PortNum] = 0;
        break;
    }
}


nodebug void ModbusRTUMasterReceiver(int PortNum, char Data, int EventNo)
{
    int i, DstAddr;
    unsigned int Word, Cmd;

//   printf("\r\nPort %d, State %d %x", PortNum, RxState[PortNum], Data);

    RxPrevState[PortNum] = RxState[PortNum];

    if(RxState[PortNum]==0)  // From which Slave ??????
    {
        if(Data != (ComSession[PortNum].Events[EventNo].UnitNumber & 0xFF))
            return;

        SPkt[PortNum][0] = Data;
        SPktIndex[PortNum] = 1;
        SPktTmr[PortNum] = 0;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum]==1)  // Function Code
    {
        if(Data != ComSession[PortNum].Events[EventNo].Cmd)
        {
            RxState[PortNum] = 0;
            return;
        }

        Cmd = SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        if(Cmd >=1 && Cmd <= 4) // Read Response
            RxState[PortNum]++;
        else if(Cmd == 5 || Cmd == 6 || Cmd == 15 || Cmd == 16) // Write Single Coil (DO) or  Write Single  HR (AO) Response
            RxState[PortNum] = 100;                              // Write Blocks (DO/AO) Response
        else
            RxState[PortNum]=0; // Command Not Supported
    }
    else if(RxState[PortNum] == 2)  // Byte Count
    {
        BNMByteCnt[PortNum] = SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 3)  // Start receiving data
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        if(--BNMByteCnt[PortNum] < 1)
            ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 4)  // Read Response
    {
        BNMCrcLsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 5)  // crc_msb, Check CRC
    {
        BNMCrcMsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], SPkt[PortNum][2] + 3);

        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            BNMNumRegs[PortNum] = (ComSession[PortNum].Events[EventNo].NumPtsHi << 8) | ComSession[PortNum].Events[EventNo].NumPtsLo;
            DstAddr = ComSession[PortNum].Events[EventNo].StartAddr;
            BNMNumRegsCnt[PortNum] = 0;
            DataSize[PortNum] = 0;

            switch(ComSession[PortNum].Events[EventNo].Cmd)
            {
            case 1:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    for(i = 0; i < 8; ++i)  // Extract Each Bit
                    {
                        if( ComSession[PortNum].Events[EventNo].BankID_Dst == DO_BANK )
                            DoutWrite(DstAddr + BNMNumRegsCnt[PortNum], (SPkt[PortNum][3+DataSize[PortNum]] >> i) & 0x01);
                        else
                            DinWrite(DstAddr + BNMNumRegsCnt[PortNum], (SPkt[PortNum][3+DataSize[PortNum]] >> i) & 0x01);
                        if(++BNMNumRegsCnt[PortNum] >= BNMNumRegs[PortNum])
                            break;
                    }
                    ++DataSize[PortNum];
                }
                break;

            case 2:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    for(i = 0; i < 8; ++i)  // Extract Each Bit
                    {
                        if( ComSession[PortNum].Events[EventNo].BankID_Dst == DI_BANK )
                            DinWrite(DstAddr + BNMNumRegsCnt[PortNum], (SPkt[PortNum][3+DataSize[PortNum]] >> i) & 0x01);
                        else
                            DoutWrite(DstAddr + BNMNumRegsCnt[PortNum], (SPkt[PortNum][3+DataSize[PortNum]] >> i) & 0x01);
                        if(++BNMNumRegsCnt[PortNum] >= BNMNumRegs[PortNum])
                            break;
                    }
                    ++DataSize[PortNum];
                }
                break;

            case 3:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    Word = (SPkt[PortNum][3+DataSize[PortNum]] << 8) | SPkt[PortNum][4+DataSize[PortNum]];
                    if( ComSession[PortNum].Events[EventNo].BankID_Dst == AO_BANK)
                        AoutWrite(DstAddr + BNMNumRegsCnt[PortNum], (float)(Word * 0.1));
                    else
                        AinWrite(DstAddr + BNMNumRegsCnt[PortNum], (float)(Word * 0.1));
                    DataSize[PortNum] = DataSize[PortNum] + 2;
                    ++BNMNumRegsCnt[PortNum];
                }
                break;

            case 4:
                while(BNMNumRegsCnt[PortNum] < BNMNumRegs[PortNum])
                {
                    Word = (SPkt[PortNum][3+DataSize[PortNum]] << 8) | SPkt[PortNum][4+DataSize[PortNum]];
                    if( ComSession[PortNum].Events[EventNo].BankID_Dst == AI_BANK)
                        AinWrite(DstAddr + BNMNumRegsCnt[PortNum], (float)(Word * 0.1));
                    else
                        AoutWrite(DstAddr + BNMNumRegsCnt[PortNum], (float)(Word * 0.1));
                    DataSize[PortNum] = DataSize[PortNum] + 2;
                    ++BNMNumRegsCnt[PortNum];
                }
                break;
            }
            GoodResponse[PortNum] = TRUE;
            RxState[PortNum] = 0;
            if( DebugEnable[PortNum] ) printf("\r\nGood modbus slave read response for event %d on port %d ", EventNo, PortNum);
            return;
        }
        else
        {
            if( DebugEnable[PortNum] ) printf("\r\nbad crc-1 for event %d on port %d ", EventNo, PortNum);
            BadCRC[PortNum] = TRUE;
            RxState[PortNum] = 0;
            return;
        }
    }
    else if(RxState[PortNum] == 100 || RxState[PortNum] == 101 || RxState[PortNum] == 102 || RxState[PortNum] == 103)  // Write Single Coil (DO) Response
    {
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 104)
    {
        BNMCrcLsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;
        ++RxState[PortNum];
    }
    else if(RxState[PortNum] == 105)
    {
        BNMCrcMsb[PortNum] = Data;
        SPkt[PortNum][SPktIndex[PortNum]++] = Data;

        crc_value.uWord = CalculateCRC(SPkt[PortNum], 6);

        RxState[PortNum] = 0;

        if((crc_value.bytes.msb == BNMCrcMsb[PortNum]) && (crc_value.bytes.lsb == BNMCrcLsb[PortNum]))  // CRC Passed
        {
            GoodResponse[PortNum] = TRUE;
            if( DebugEnable[PortNum] ) printf("\r\nGood modbus slave write response for event %d on port %d ", EventNo, PortNum);
            return;
        }
        else
        {
            if( DebugEnable[PortNum] ) printf("\r\nbad crc-2 for event %d on port %d ", EventNo, PortNum);
            BadCRC[PortNum] = TRUE;
            return;
        }
    }
    else
        RxState[PortNum] = 0;
}

nodebug char ResetModem()
{
    unsigned char buffer[10];
    int txIndex;
    char resetComplete = 0;
    txIndex = 0;

    memset(buffer, 0, sizeof(buffer));
    switch (stateCaseResetModem)
    {
    case 0:
        ++stateCaseResetModem;
        break;

    case 1:
        ++stateCaseResetModem;
        break;

    case 2://Put Modem in Command Mode
        buffer[txIndex++] = '+';
        buffer[txIndex++] = '+';
        buffer[txIndex++] = '+';
        printf("Sending +++\r\n");
        MQTT_SendData(buffer,txIndex);
        ++stateCaseResetModem;
        break;

    case 3:
        if(receiveOK)//If we get OK then modem is in command mode
        {
            receiveOK = 0;
            printf("ReceivedOK\r\n");
            ++stateCaseResetModem;
        }
        else
        {
            ++rxTimeOuttimer;
            if(rxTimeOuttimer > 3)
            {
                stateCaseResetModem =  0;
                rxTimeOuttimer = 0;
                receiveOK = 0;
                printf("TimedOut\r\n");
                ++ResetRetries;
                if(ResetRetries >= 3)
                {
                    resetComplete = 1;
                    ResetRetries =  0;
                }
            }
        }
        break;

    case 4://Reset the modem with AT cmd FR
        memset(buffer, 0, sizeof(buffer));
        txIndex = 0;
        buffer[txIndex++] = 'A';
        buffer[txIndex++] = 'T';
        buffer[txIndex++] = 'S';
        buffer[txIndex++] = 'D';
        buffer[txIndex++] = '=';
        buffer[txIndex++] = '1';
        buffer[txIndex++] = 0x0d;

        printf("Resetting the modem\r\n");
        MQTT_SendData(buffer,txIndex);
        ++stateCaseResetModem;
        break;

    case 5:
        if(receiveOK)
        {
            printf("Modem Got the reset\r\n");
            stateCaseResetModem = 0;
            resetComplete = 1;
        }
        ++rxTimeOuttimer;
        if(rxTimeOuttimer > 30)
        {
            stateCaseResetModem =  0;
            rxTimeOuttimer = 0;
            receiveOK = 0;
            ++ResetRetries;
            if(ResetRetries >= 3)
            {
                resetComplete = 1;
                ResetRetries =  0;
            }
            printf("TimedOut\r\n");
        }
        break;
    }
    return resetComplete;
}

nodebug char ResetModemLogic(unsigned char isConnected)
{
    char StopPublishResetModem=0;
    if(!isConnected)
    {
        ++NotConnectedToBrokerTimer;
        if(NotConnectedToBrokerTimer >= NotConnectedToBrokerTimeOut)
        {
            StopPublishResetModem = 1;
        }
    }
    else
    {
        StopPublishResetModem = 0;
        NotConnectedToBrokerTimer =0;
    }
    printf("NotConnectedTimer:%d ResetModem:%d\r\n",NotConnectedToBrokerTimer,StopPublishResetModem);
    return StopPublishResetModem;
}