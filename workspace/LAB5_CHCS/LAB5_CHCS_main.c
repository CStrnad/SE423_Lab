//#############################################################################
// FILE:   LAB5_CHCS_main.c
//
// TITLE:  Lab 5 C File for SE423 Mechatronics Lab
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
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCC_ISR(void);    //CJS ADCC Interrupt predef.

//CH predefinitions
void init_eQEPS(void);
float readEncLeft(void);
float readEncRight(void);
float readEncWheel(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint32_t ADCC_ISR_count = 0;    //CJS ADCC ISR Counter Variable.

//CJS Global variables for 2000th sum of data points. ADCC
float sum4Z = 0;
float sumZ = 0;
float sum4X = 0;
float sumX = 0;
//CJS Global variables for 2000th average of sums.
float zero4Z = 0;
float zeroZ = 0;
float zero4X = 0;
float zeroX = 0;
//CJS Degrees per second global vars.
float X4degSec = 0;
float Z4DegSec = 0;
float XDegSec = 0;
float ZDegSec = 0;

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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
//    init_serialSCIC(&SerialC,115200);
//    init_serialSCID(&SerialD,115200);

//    EALLOW;
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

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //CJS Enable Group for ADCC1 at Group 1 Interrupt 3
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
//            serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
//            UART_printfLine(1,"Timer2 Calls %ld",CpuTimer2.InterruptCount);

            //Lab 3
//            UART_printfLine(2,"Wheel Enc: %.3f", encWheel); // CH print adjusted wheel encoder input value
//            UART_printfLine(1,"vl: %.2f vr %.2f", v_left,v_right); //CH print left and right wheel velocity in ft/s

            UART_printfLine(2,"Vref: %.3f", Vref); // CH print adjusted wheel encoder input value
            UART_printfLine(1,"vl: %.2f vr %.2f", v_left,v_right); //CH print left and right wheel velocity in ft/s

            //Lab 4
//            //CJS Print the Degrees/Second conversions to LCD.
//            UART_printfLine(1, "X4: %.2f  Z4: %.2f", X4degSec, Z4DegSec);
//            UART_printfLine(2, " X: %.2f   Z: %.2f", XDegSec, ZDegSec);
//
//            //CJS Print also to TerraTerm
//            serial_printf(&SerialA,"Z4: %6.2f \t Z: %6.2f \t ADCC Count: %ld\r\n", Z4DegSec, ZDegSec, ADCC_ISR_count);
            serial_printf(&SerialA,"bear4Z: %6.2f \t bearZ: %6.2f \t\r\n", bearingK4, bearingK);


            UARTPrint = 0;
        }
    }
}



// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;
    
    DINT;

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



    //Print to Serial Every 100ms
    if ((CpuTimer1.InterruptCount % 100) == 0){
        UARTPrint = 1;
    }

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
    float adcc0result = AdccResultRegs.ADCRESULT0;
    float adcc1result = AdccResultRegs.ADCRESULT1;
    float adcc2result = AdccResultRegs.ADCRESULT2;
    float adcc3result = AdccResultRegs.ADCRESULT3;

    //CJS Get scaled results.
    float scaledAdccResult0 = (adcc0result / 4095.0) * 3.0;
    float scaledAdccResult1 = (adcc1result / 4095.0) * 3.0;
    float scaledAdccResult2 = (adcc2result / 4095.0) * 3.0;
    float scaledAdccResult3 = (adcc3result / 4095.0) * 3.0;

    if(ADCC_ISR_count <= 1000){ //CJS For the first 1 second, do nothing.
        setEPWM1A(0);
        setEPWM2A(0);
        //Do nothing
    } else if (ADCC_ISR_count > 1000 && ADCC_ISR_count <= 3000){ //CJS For 2 seconds...
        setEPWM1A(0);
        setEPWM2A(0);
        sum4Z   += scaledAdccResult2; //CJS Add current value to global var.
        sumZ    += scaledAdccResult3;
        sum4X   += scaledAdccResult1;
        sumX    += scaledAdccResult0;
    } else if (ADCC_ISR_count == 3001){ //CJS Get average (zero) baseline
        setEPWM1A(0);
        setEPWM2A(0);
        zero4Z  = sum4Z / 2000.0;
        zeroZ   = sumZ  / 2000.0;
        zero4X  = sum4X / 2000.0;
        zeroX   = sumX  / 2000.0;
    }
    else { //CJS Set original converted variables to themselves minus the correction variables.
        X4degSec    = (scaledAdccResult1 - zero4X) * 100.0;// - 123.0;
        Z4DegSec    = (scaledAdccResult2 - zero4Z) * 100.0;// - 123.0;
        XDegSec     = (scaledAdccResult0 - zeroX)  * 400.0;// - 492.0;
        ZDegSec     = (scaledAdccResult3 - zeroZ)  * 400.0;// - 492.0;

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
        turn = encWheel/20.0;
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

        //-----------------------------------------------------------------------------

        //CH setting old values as current values before loop exits
        Lwheeldistold = Lwheeldist;
        Rwheeldistold = Rwheeldist;

    }


//    //Print to Serial Every 100ms
//    if ((ADCC_ISR_count % 100) == 0){
//        UARTPrint = 1;
//    }
    ADCC_ISR_count++;

    //CJS PIECE CLEAR THING
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

