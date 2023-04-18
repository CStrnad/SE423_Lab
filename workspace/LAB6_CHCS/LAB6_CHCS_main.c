//#############################################################################
// FILE:   LAB6_CHCS_main.c
//
// TITLE:  Lab 6 C File for SE423 Mechatronics Lab
//
// Initials CH (Chris Hahn) and CJS (Chris Strnad)
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "F2837xD_SWPrioritizedIsrLevels.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define FEETINONEMETER 3.28083989501312
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI1_HighestPriority(void);
__interrupt void SWI2_MiddlePriority(void);
__interrupt void SWI3_LowestPriority(void);
__interrupt void ADCC_ISR(void);    //CJS ADCC Interrupt predef.
void setupSpib(void); // CH Setup SPIB function predef.
void PostSWI1(void);
void PostSWI3(void);


//CH predefinitions
void init_eQEPS(void);
float readEncLeft(void);
float readEncRight(void);
float readEncWheel(void);
void setupSpib(void);
void SPIB_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint32_t ADCC_ISR_count = 0;    //CJS ADCC ISR Counter Variable.
uint32_t SPIB_Interrupt_Count = 0;

//CJS Global variables for 2000th sum of data points. ADCC
float sum4Z = 0;
float sumZ = 0;
float sum4X = 0;
float sumX = 0;
float sumGyroZ = 0;
//CJS Global variables for 2000th average of sums.
float zero4Z = 0;
float zeroZ = 0;
float zero4X = 0;
float zeroX = 0;
float zeroGyroZ = 0;
//CJS Degrees per second global vars.
float X4degSec = 0;
float Z4DegSec = 0;
float XDegSec = 0;
float ZDegSec = 0;
//CH gyro and accel vars
int16_t gyroxraw = 0;
int16_t gyroyraw = 0;
int16_t gyrozraw = 0;
int16_t tempraw = 0;
int16_t accelxraw = 0;
int16_t accelyraw = 0;
int16_t accelzraw = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;
//CH ADCC gyro vars
float adcc0result = 0;
float adcc1result = 0;
float adcc2result = 0;
float adcc3result = 0;
float scaledAdccResult0 = 0;
float scaledAdccResult1 = 0;
float scaledAdccResult2 = 0;
float scaledAdccResult3 = 0;

//CH Lab 3 global variables ---asdf
uint16_t updown = 0;
float motorControl = 7;
uint16_t motorUpdown = 0;
float LeftWheel = 0;
float RightWheel = 0;
float encWheel = 0;
float Rwheeldist = 0;
float Lwheeldist = 0;
float radpft = 9.724;
float uLeft = 0;
float uRight = 0;
float v_left = 0;
float v_right = 0;
float Lwheeldistold = 0;
float Rwheeldistold = 0;
//CJS Position and Friction Coeff constants
float vPos = 2.447;
float vNeg = 2.463;
float cPos = 2.117;
float cNeg = -2.127;

//CJS Lab 5 Global Variables for closed-loop
float errLeftOld = 0;
float errRightOld = 0;
float errLeft = 0;
float errRight = 0;
float Vref = 1;
float IkLeft = 0;
float IkRight = 0;
float IkLeftOld = 0;
float IkRightOld = 0;
float Kp = 3.0;
float Ki = 25.0;
float Kp_turn = 3.0;
float turn = 0;
float errSteer = 0;
float bearingK = 0;
float bearingK4 = 0;
float bearingKOld = 0;
float bearingK4Old = 0;
float gyroZOld = 0;
float gyroZ4Old = 0;
float gyrozold = 0;
float bearingAngleOld = 0;
float Xpointold = 0;
float Ypointold = 0;

//CJS Global Defs from LADAR Lab 6 backup
uint32_t timecount = 0;
extern datapts ladar_data[228]; //distance data from LADAR
extern float printLV1;
extern float printLV2;
extern float LADARrightfront;
extern float LADARfront;
float LADARrightrear;
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern char G_command[]; //command for getting distance -120 to 120 degree
extern uint16_t G_len; //length of command
extern xy ladar_pts[228]; //xy data
extern uint16_t LADARpingpong;
extern float LADARxoffset;
extern float LADARyoffset;
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];
extern uint16_t NewLVData;
uint16_t LADARi = 0;
pose ROBOTps = {0,0,0}; //robot position
pose LADARps = {3.5/12.0,0,1};  // 3.5/12 for front mounting, theta is not used in this current code
float printLinux1 = 0;
float printLinux2 = 0;
//CH ex 4 variables inside LinuxCMDApp
float ref_right_wall = 1.0; // 1.25
float left_turn_Start_threshold = 1.3; // 1.9
float left_turn_Stop_threshold = 3.2; // 4.0
float Kp_right_wall = -2.0; // -1.2
float Kp_front_wall = -0.4; // -0.2
float front_turn_velocity = 0.4; // 0.6
float forward_velocity = 1; // 1.3
float turn_command_saturation = 2.0;
uint16_t right_wall_follow_state = 2;

//--------------------- SWI Defs----------------------
void PostSWI1(void){
    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; //CJS Manually interrupt SWI1
}
void PostSWI3(void){
    PieCtrlRegs.PIEIFR12.bit.INTx11 = 1; //CJS Manually interrupt SWI3
}



//CJS----------------------- Functions imported from Lab 3 -------------------------


void  setEPWM1A(float controleffort){
    //CH check input bounds
    if (controleffort > 10){
        controleffort = 10;
    } else if (controleffort < -10){
        controleffort = -10;
    }

    //CH adding 10 to the input gets rid of negative values, and then dividing by 20 gets us the desired duty cycle
    //CH which we can then assign to CMPA
    float fracValue = 10 + controleffort;
    float dutyCycle = fracValue/20.0;
    EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD * dutyCycle;
}

//CJS same thing as above, written differently
void setEPWM2A (float controlEffort) {
    if (controlEffort > 10) controlEffort = 10;
    if (controlEffort < -10) controlEffort = -10;
    EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.TBPRD * ((10+controlEffort)/20.0);
}

//CH provided code to read raw values of encoder and wheels
void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP2 pins for input
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO55 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP3 pins for input
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1; // Disable pull-up on GPIO54 (EQEP3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1; // Disable pull-up on GPIO55 (EQEP3B)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 5); // set GPIO6 and eQep2A
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 5); // set GPIO7 and eQep2B
    EQep3Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep3Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep3Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep3Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep3Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep3Regs.QPOSCNT = 0;
    EQep3Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(-2*PI/40000));// CH number of recordings per turn
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/40000)); // CH number of recordings per turn
}

float readEncWheel(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep3Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/4000));
}

//--------------------------------------------------------------------------------------




void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LS7366#1 CS
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LS7366#2 CS
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LS7366#3 CS
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LS7366#4 CS
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // WIZNET RST
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    //PushButton 1
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_INPUT, GPIO_PULLUP);

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //F28027 CS
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCC1_INT = &ADCC_ISR;                 //CJS ADCC PIE Entry
    PieVectTable.SPIB_RX_INT = &SPIB_isr; //CH SPIB PIE
    PieVectTable.EMIF_ERROR_INT = &SWI1_HighestPriority;
    PieVectTable.RAM_CORRECTABLE_ERROR_INT = &SWI2_MiddlePriority;
    PieVectTable.FLASH_CORRECTABLE_ERROR_INT = &SWI3_LowestPriority;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 100000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    DELAY_US(1000000);  // Delay 1 second giving LADAR Time to power on after system power on

   init_serialSCIA(&SerialA,115200);
   init_serialSCIB(&SerialB,19200);
   init_serialSCIC(&SerialC,19200);
   init_serialSCID(&SerialD,2083332);

   for (LADARi = 0; LADARi < 228; LADARi++) {
       ladar_data[LADARi].angle = ((3*LADARi+44)*0.3515625-135)*0.01745329; //0.017453292519943 is pi/180, multiplication is faster; 0.3515625 is 360/1024
   }

//    EALLOW;

    setupSpib(); //CH call setup SPIB function

    //CJS-------------------- EPWM1A and 2A Configurations from Lab 3 ------------------------
    //CJS EPwm1A Setup
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTR = 0;
    EPwm1Regs.TBPRD = 2500;
    EPwm1Regs.CMPA.bit.CMPA = 1250;
    EPwm1Regs.AQCTLA.bit.CAU = 1;
    EPwm1Regs.AQCTLA.bit.ZRO = 2;
    EPwm1Regs.TBPHS.bit.TBPHS = 0;

    //CJS EPwm2A Setup
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 1250;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;


            //CH EPWM1A GPIO0
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);

    //CH EPWM2A GPIO2
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    //CJS-------------------- END EPWM1A and 2A Configurations from Lab 3 --------------------

    //CJS-------------------- ADCC Configurations from Lab 4 ------------------------
    // CH EPWM4 Initializations

    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
//    EDIS;

    EALLOW;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;      //CJS set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;   //CJS Set pulse positions to late.
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //CJS Power up ADCC

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be B0
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC0
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be B1
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC1
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4; //SOC2 will convert Channel you choose Does not have to be B2
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC2
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5; //SOC3 will convert Channel you choose Does not have to be B3
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC3
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //CJS-------------------- END ADCC Configurations from Lab 4 --------------------
    EDIS;

    //Uncomment for Exercise 2----------------------asdf----------------------------
    //CH provided code for safety reasons
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1; // For EPWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWMA2
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    //CH call init_eQEPS()
    init_eQEPs();
    //-------------------------------------------------------------------------



    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; //CH SPIB PIE channel

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
//    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;   //CJS Disable original SWI
    //CJS Enable Group for ADCC1 at Group 1 Interrupt 3
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;
    // CH Enable SPIB_RX interrupt Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1; //SWI1
    PieCtrlRegs.PIEIER12.bit.INTx10 = 1; //SWI2
    PieCtrlRegs.PIEIER12.bit.INTx11 = 1; //SWI3  Lowest priority

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    char S_command[19] = "S1152000124000\n";//this change the baud rate to 115200
    uint16_t S_len = 19;
    serial_sendSCIC(&SerialC, S_command, S_len);


    DELAY_US(1000000);  // Delay letting LADAR change its Baud rate
    init_serialSCIC(&SerialC,115200);

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
//            serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
//            UART_printfLine(1,"Timer2 Calls %ld",CpuTimer2.InterruptCount);

            //Lab 3
//            UART_printfLine(2,"Wheel Enc: %.3f", encWheel); // CH print adjusted wheel encoder input value
//            UART_printfLine(1,"vl: %.2f vr %.2f", v_left,v_right); //CH print left and right wheel velocity in ft/s

//            UART_printfLine(2,"Vref: %.3f", Vref); // CH print adjusted wheel encoder input value
//            UART_printfLine(1,"vl: %.2f vr %.2f", v_left,v_right); //CH print left and right wheel velocity in ft/s

            //Lab 4
//            //CJS Print the Degrees/Second conversions to LCD.
//            UART_printfLine(1, "X4: %.2f  Z4: %.2f", X4degSec, Z4DegSec);
//            UART_printfLine(2, " X: %.2f   Z: %.2f", XDegSec, ZDegSec);
//
//            //CJS Print also to TerraTerm
//            serial_printf(&SerialA,"Z4: %6.2f \t Z: %6.2f \t ADCC Count: %ld\r\n", Z4DegSec, ZDegSec, ADCC_ISR_count);
//            serial_printf(&SerialA,"bear4Z: %6.2f \t bearZ: %6.2f \t\r\n", bearingK4, bearingK);

            //CH lab 6 print MPU gyroz, ADCC gyroz4, and left/right wheel velocities to LCD screen
            //CH print all 6 MPU sensor readings to tera term
//            UART_printfLine(1, "MPZ:%0.2f ACZ:%0.2f", gyroz, Z4DegSec);
//            UART_printfLine(2, "vl: %.2f vr: %.2f", v_left, v_right);
            //serial_printf(&SerialA, "GX: %.2f\t GY: %.2f\t GZ: %.2f\t AX: %.2f\t AY: %.2f\t AZ: %.2f\t\r\n", gyrox,gyroy,gyroz,accelx,accely,accelz);
            //UART_printfLine(1,"turn: %.2f LRR: %.2f",turn, LADARrightrear);
            //UART_printfLine(2,"F%.4f R%.4f",LADARfront,LADARrightfront); //good rightfront value at 1.29
            //UART_printfLine(1,"%.1f %.1f %.1f %.1f",Vref,turn,ref_right_wall,left_turn_Start_threshold);
            //UART_printfLine(2,"%.1f %.1f %.1f %.1f",left_turn_Stop_threshold,Kp_right_wall,Kp_front_wall,front_turn_velocity);
            //UART_printfLine(2,"%.1f %.1f",forward_velocity,turn_command_saturation);
            //UART_printfLine(1,"theta: %.2f",ROBOTps.theta);
            //UART_printfLine(2,"X: %.2f Y: %.2f",ROBOTps.x,ROBOTps.y);
            UART_printfLine(1,"LV1: %.2f LV2: %.2f",printLV1, printLV2);


            UARTPrint = 0;
        }
    }
}





// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%5) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    serial_sendSCIC(&SerialC, G_command, G_len);


    //Print to Serial Every 100ms
//    if ((CpuTimer1.InterruptCount % 100) == 0){
//        UARTPrint = 1;
//    }

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0) {
        //UARTPrint = 1;
    }
}

//CJS ADCC ISR
__interrupt void ADCC_ISR (void) {
    //CJS Read the Four result registers
    adcc0result = AdccResultRegs.ADCRESULT0;
    adcc1result = AdccResultRegs.ADCRESULT1;
    adcc2result = AdccResultRegs.ADCRESULT2;
    adcc3result = AdccResultRegs.ADCRESULT3;

    //CJS Get scaled results.
    scaledAdccResult0 = (adcc0result / 4095.0) * 3.0;
    scaledAdccResult1 = (adcc1result / 4095.0) * 3.0;
    scaledAdccResult2 = (adcc2result / 4095.0) * 3.0;
    scaledAdccResult3 = (adcc3result / 4095.0) * 3.0;

    //CH EX 5 read 3 accelerometer readings and 3 gyro readings, as well as the temp reading
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = (0x8000 | 0x3A00); //CH set read bit, at INT_STATUS, which is one register above the first accel register
    SpibRegs.SPITXBUF = 0x0000; //CH 3B ACCEL_XOUT
    SpibRegs.SPITXBUF = 0x0000; //CH 3D ACCEL_YOUT
    SpibRegs.SPITXBUF = 0x0000; //CH 3F ACCEL_ZOUT
    SpibRegs.SPITXBUF = 0x0000; //CH 41 TEMP_OUT
    SpibRegs.SPITXBUF = 0x0000; //CH 43 GYRO_XOUT
    SpibRegs.SPITXBUF = 0x0000; //CH 45 GYRO_YOUT
    SpibRegs.SPITXBUF = 0x0000; //CH 47 GYRO_ZOUT

//    //Print to Serial Every 100ms
//    if ((ADCC_ISR_count % 100) == 0){
//        UARTPrint = 1;
//    }
    ADCC_ISR_count++;

    //CJS PIECE CLEAR THING
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//CH SPIB provided interrupt function
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
int16_t spivalue4 = 0;
int16_t spivalue5 = 0;
int16_t spivalue6 = 0;
int16_t spivalue7 = 0;
int16_t spivalue8 = 0;

__interrupt void SPIB_isr(void){

    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. Again probably zero
    spivalue3 = SpibRegs.SPIRXBUF;
    spivalue4 = SpibRegs.SPIRXBUF;
    spivalue5 = SpibRegs.SPIRXBUF;
    spivalue6 = SpibRegs.SPIRXBUF;
    spivalue7 = SpibRegs.SPIRXBUF;
    spivalue8 = SpibRegs.SPIRXBUF;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 66 high to end Slave Select. Now to Scope. Later to deselect
    //MPU9250.

    //CH assign accel and gyro raw readings to SPI recieved values
    accelxraw = spivalue2;
    accelyraw = spivalue3;
    accelzraw = spivalue4;
    tempraw = spivalue5;
    gyroxraw = spivalue6;
    gyroyraw = spivalue7;
    gyrozraw = spivalue8;

    //CH adjust values with offsets

    //CH convert raw values to scaled - gyro in deg/s and accel in g
    accelx = accelxraw*4.0/32767.0;
    accely = accelyraw*4.0/32767.0;
    accelz = accelzraw*4.0/32767.0;
    gyrox = gyroxraw*250.0/32767.0;
    gyroy = gyroyraw*250.0/32767.0;
    gyroz = gyrozraw*250.0/32767.0; //MPU-9250 Z Gyro reading

    PostSWI1();


    // Later when actually communicating with the MPU9250 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

    if ((SPIB_Interrupt_Count % 100) == 0){
        UARTPrint = 1;
    }
    SPIB_Interrupt_Count++;

}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;

    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Also don’t forget to cut and
    //paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
    //-----------------------------------------------------------------------------------------------------------------

    //CH SPI Setup
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB r
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 50; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
     // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0; // Set delay between transmits to 0 spi clocks.
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more received into FIFO causes
    //interrupt. This is just the initial setting for the register. Will be changed below

    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    //sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x1300;
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0013;
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0200;
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0806;
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x2300;
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C;
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288;
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A;
    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0013); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0022); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00EC); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0012); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001E); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x000E); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

//
// Connected to PIEIER12_9 (use MINT12 and MG12_9 masks):
//
__interrupt void SWI1_HighestPriority(void)     // EMIF_ERROR
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_9;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    // Add declarations for tunable global variables to include:
    // ref_right_wall – desired distance from right wall. You will have
    // to figure out what distance is read by the LADAR [54] reading when the robot is
    // placed approximately 6 to 8 inches from the wall.
    // left_turn_Start_threshold When front distance less than this, switch to Left Turn state
    // left_turn_Stop_threshold When front distance greater than this, switch to Right follow
    // Kp_right_wall – proportional gain for controlling distance of robot to wall,
    // start with 0.1
    // Kp_front_wall – proportional gain for turning robot when front wall error is high,
    // start with 0.1
    // front_turn_velocity – velocity when the robot starts to turn to avoid
    // a front wall, use 0.4 to start
    // forward_velocity – velocity when robot is right wall following, use 1.0 to start
    // turn_command_saturation – maximum turn command to prevent robot from spinning quickly if
    // error jumps too high, start with 1.0
    // These are all ‘knobs’ to tune in lab!

//   //declare other globals that you will need
//    uint16_t right_wall_follow_state = 2;
//
//   //inside SWI1 before PI speed control
    switch (right_wall_follow_state) {
        case 1:
            //Left Turn
            turn = Kp_front_wall*(14.5 - LADARfront);
            Vref = front_turn_velocity;
            if (LADARfront > left_turn_Stop_threshold) {
                right_wall_follow_state = 2;
            }
            break;
        case 2:
            //Right Wall Follow
            Vref = forward_velocity;
            if ((fabs(ref_right_wall - LADARrightfront)) > 2.2){
                turn = Kp_right_wall*(ref_right_wall - LADARrightrear);

            } else {
                turn = Kp_right_wall*(ref_right_wall - LADARrightfront);
            }
            if (LADARfront < left_turn_Start_threshold) {
                right_wall_follow_state = 1;
            }

            break;
    }
    // Add code here to saturate the turn command so that it is not larger
    // than turn_command_saturation or less than –turn_command_saturation
    if (turn > turn_command_saturation){
        turn = turn_command_saturation;
    } else if (turn < -turn_command_saturation){
        turn = -turn_command_saturation;
    }

    uint16_t i = 0;//for loop

    //PI Control Code from Lab 5 - CJS
    if(timecount <= 1000){ //CJS For the first 1 second, do nothing.
        setEPWM1A(0);
        setEPWM2A(0);
        //Do nothing
    } else if (timecount > 1000 && timecount <= 3000){ //CJS For 2 seconds...
        setEPWM1A(0);
        setEPWM2A(0);
        sum4Z   += scaledAdccResult2; //CJS Add current value to global var.
        sumZ    += scaledAdccResult3;
        sum4X   += scaledAdccResult1;
        sumX    += scaledAdccResult0;
        //CH gyroz reading
        sumGyroZ += gyroz;
    } else if (timecount == 3001){ //CJS Get average (zero) baseline
        setEPWM1A(0);
        setEPWM2A(0);
        zero4Z  = sum4Z / 2000.0;
        zeroZ   = sumZ  / 2000.0;
        zero4X  = sum4X / 2000.0;
        zeroX   = sumX  / 2000.0;
        zeroGyroZ = sumGyroZ / 2000.0;
    }
    else { //CJS Set original converted variables to themselves minus the correction variables.


        X4degSec    = (scaledAdccResult1 - zero4X) * 100.0;// - 123.0;
        Z4DegSec    = (scaledAdccResult2 - zero4Z) * 100.0;// - 123.0;
        XDegSec     = (scaledAdccResult0 - zeroX)  * 400.0;// - 492.0;
        ZDegSec     = (scaledAdccResult3 - zeroZ)  * 400.0;// - 492.0;
        gyroz -= zeroGyroZ;

        //CH getting raw values for wheels and encoder
        LeftWheel = readEncLeft();
        RightWheel = readEncRight();
        encWheel = readEncWheel();

        //CH converting raw values to distance values via radpft conversion factor
        Lwheeldist = LeftWheel / radpft;
        Rwheeldist = RightWheel / radpft;

        //CH calculating velocity of wheels by taking delta x over delta t
        v_left = (Lwheeldist - Lwheeldistold)/0.001;
        v_right = (Rwheeldist - Rwheeldistold)/0.001;

        //        //CH assign reference velocity
        //        Vref = encWheel/20.0;

        //CH calculating error
        errLeft = Vref - v_left;
        errRight = Vref - v_right;

        //CH calculating integral
        IkRight = IkRightOld + ((errRight + errRightOld)/2) * 0.001;
        IkLeft = IkLeftOld + ((errLeft + errLeftOld)/2) * 0.001;

        //CH turning code Ex 4
        //turn = encWheel/20.0;
        errSteer = v_right - v_left + turn;
        errLeft = errLeft + errSteer*Kp_turn;
        errRight = errRight - errSteer*Kp_turn;

        //CH assign new wheel velocity from block diagram
        uLeft = Kp*errLeft + Ki*IkLeft;
        uRight = Kp*errRight  + Ki*IkRight;

        //Integral Wind-up correction
        if(fabs(uLeft) > 10) IkLeft = IkLeftOld;
        if(fabs(uRight) > 10) IkRight = IkRightOld;


        //        uLeft = encWheel;
        //        uRight = encWheel;
        //    uLeft  = 0;
        //    uRight = 0; //CJS set disp to 0 for friction test.

        // CH set bounds for uLeft and uRight
        if (uLeft >= 10){
            uLeft = 10;
        }
        if (uLeft <= -10){
            uLeft = -10;
        }
        if (uRight >= 10){
            uRight = 10;
        }
        if (uRight <= -10){
            uRight = -10;
        }

        //CJS Friction compensation for Left and Right motors.
        if (v_left > 0.0) uLeft = uLeft + vPos * v_left + cPos;
        else uLeft = uLeft + vNeg * v_left + cNeg;

        if (v_right > 0.0) uRight = uRight + vPos * v_right + cPos;
        else uRight = uRight + vNeg * v_right + cNeg;

        //CH Ex6 Integrating gyro
        bearingK = bearingKOld + ((ZDegSec + gyroZOld)/2)*.001;
        bearingK4 = bearingK4Old + ((Z4DegSec + gyroZ4Old)/2)*.001;
        //CH SPI bearing angle
        ROBOTps.theta = bearingAngleOld + ((gyroz + gyrozold)/2)*.001;

        //CH Integrating MPU-9250 Gyro
        ROBOTps.x = Xpointold + ((v_left + v_right)/2.0)*cos(ROBOTps.theta*(PI/180))*.001;
        ROBOTps.y = Ypointold + ((v_left + v_right)/2.0)*sin(ROBOTps.theta*(PI/180))*.001;

        // CH calling the functions to enable wheel function
        setEPWM1A(uLeft);
        setEPWM2A(-uRight);
        //CH set old values lab 5
        errLeftOld = errLeft;
        errRightOld = errRight;
        IkRightOld = IkRight;
        IkLeftOld = IkLeft;
        gyroZOld = ZDegSec;
        gyroZ4Old = Z4DegSec;
        bearingKOld = bearingK;
        bearingK4Old = bearingK4;
        gyrozold = gyroz;
        bearingAngleOld = ROBOTps.theta;
        Xpointold = ROBOTps.x;
        Ypointold = ROBOTps.y;


        //-----------------------------------------------------------------------------

        //CH setting old values as current values before loop exits
        Lwheeldistold = Lwheeldist;
        Rwheeldistold = Rwheeldist;

    }


    //##### END PI Control Code

    if (newLinuxCommands == 1) {
        newLinuxCommands = 0;
        Vref = LinuxCommands[0];
        turn = LinuxCommands[1];
        ref_right_wall = LinuxCommands[2];
        left_turn_Start_threshold = LinuxCommands[3];
        left_turn_Stop_threshold = LinuxCommands[4];
        Kp_right_wall = LinuxCommands[5];
        Kp_front_wall = LinuxCommands[6];
        front_turn_velocity = LinuxCommands[7];
        forward_velocity = LinuxCommands[8];
        turn_command_saturation = LinuxCommands[9];
        //value11 = LinuxCommands[10];
    }


    if (NewLVData == 1) {
        NewLVData = 0;
        printLV1 = fromLVvalues[0];
        printLV2 = fromLVvalues[1];
    }

    if((timecount%250) == 0) {
        DataToLabView.floatData[0] = ROBOTps.x;
        DataToLabView.floatData[1] = ROBOTps.y;
        DataToLabView.floatData[2] = (float)timecount;
        DataToLabView.floatData[3] = ROBOTps.theta;
        DataToLabView.floatData[4] = ROBOTps.theta;
        DataToLabView.floatData[5] = ROBOTps.theta;
        DataToLabView.floatData[6] = ROBOTps.theta;
        DataToLabView.floatData[7] = ROBOTps.theta;
        LVsenddata[0] = '*';  // header for LVdata
        LVsenddata[1] = '$';
        for (i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }


    timecount++;

    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    //##############################################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_10 (use MINT12 and MG12_10 masks):
//
__interrupt void SWI2_MiddlePriority(void)     // RAM_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_10;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    if (LADARpingpong == 1) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_ping;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_ping;
            }
        }
        //CH LADARrightrear is the min of dist 1, 2, 3, 4, 5, 6
        LADARrightrear = 19;
        for (LADARi = 30; LADARi <= 35; LADARi++){
            if (ladar_data[LADARi].distance_ping < LADARrightrear){
                LADARrightrear = ladar_data[LADARi].distance_ping;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_ping*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_ping*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
    } else if (LADARpingpong == 0) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_pong;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_pong;
            }
        }
        //CH LADARrightrear is the min of dist 1, 2, 3, 4, 5, 6
        LADARrightrear = 19;
        for (LADARi = 30; LADARi <= 35; LADARi++){
            if (ladar_data[LADARi].distance_pong < LADARrightrear){
                LADARrightrear = ladar_data[LADARi].distance_pong;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_pong*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_pong*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
}




    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_11 (use MINT12 and MG12_11 masks):
//
__interrupt void SWI3_LowestPriority(void)     // FLASH_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_11;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}
