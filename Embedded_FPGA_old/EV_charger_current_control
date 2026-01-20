


//Program Overview:
//This embedded C program is designed for a Zynq-based system that manages a power conversion setup, including PWM control, current and voltage measurement via XADC, and safety features like short circuit detection. It initializes hardware peripherals such as GPIO, timers, and ADC, then runs a main loop where it monitors and controls power electronics, generating PWM signals, reading sensor data, and responding to fault conditions. The program also includes a simple user interface via buttons and debugging output via UART.


#include "math.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xgpiops.h"
#include "xadcps.h"
#include "xstatus.h"
#include "stdio.h"
#include "stdbool.h"
#include "unistd.h"
#include "xuartps.h"
#include "xscutimer.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xil_printf.h"

// Define device IDs and constants
#define TIMER_DEVICE_ID       XPAR_XSCUTIMER_0_DEVICE_ID
#define SCU_TIMER_DEVICE_ID   XPAR_SCUTIMER_DEVICE_ID
#define INTC_DEVICE_INT_ID    XPAR_SCUGIC_SINGLE_DEVICE_ID
#define XADC_DEVICE_ID        XPAR_XADCPS_0_DEVICE_ID

// Memory addresses
#define BRAM_MEMORY         0x40020000
#define DDR_MEMORY          0x20000000

// Buffer length
#define LENGTH 8192

// Pin definitions
#define legpin1 0x01 // GPIO pin IO00
#define legpin2 27   // GPIO pin IO02

// Safety thresholds (example values)
#define S100 0
#define S200 0
#define S101 128
#define S201 1
#define S110 1
#define S210 16

// Function prototypes
static int XAdcPolledPrintfExample(u16 XAdcDeviceId);
static int XAdcFractionToInt(float FloatNum);

// Global XADC driver instance
static XAdcPs XAdcInst;

/**
 * Main function:
 * Initializes hardware peripherals, configures control parameters,
 * and runs a control loop to manage power electronics, monitor sensors,
 * and handle fault detection.
 */
int main(void)
{
    // --- Variable declarations ---

    // PWM control variables
    int pwmCount;
    int pwmCountMax;
    int pwmRunCount;

    // Voltage and current measurements
    int VDC, VDCraw, VDC1;
    int iref, irefMax, fs, duty1, duty1prev, duty1Check;
    int duty2, groundCount, switchCount1, switchCount2, periodCount, fsCount;
    int btnCheck, VDCsimul, blankingMicro;

    // Thresholds and calibration
    int duty1Thresh;
    int IDCraw, VUT, deltaIDC, iref2, IDCcalibrate, VDCcalibrate, threshold;
    int ampereConvert, dutyAttenuate, currentFake, IDCfake, VDCfake, V1, V2;

    // Frequency control
    int freqRef, freqRefMargin;

    // Scalars for sensor calibration
    float IDCscalar1, IDCscalar2, IDCscalar3;
    float VDCscalar1, VDCscalar2, VDCscalar3;

    // Timing
    double timerPer;
    int CntValueNow, CntValuePrev;
    long CntDiff;
    int TsMicro, TsPrint;

    // GPIO peripherals
    XGpio leg1, leg2, btn;

    // Timer instance
    XScuTimer Timer;
    XScuTimer_Config *ConfigPtrTime;
    XScuTimer *TimerInstancePtr = &Timer;
    int Status;

    // Sensor measurements
    double IDCdouble, IDCamp, IDCfallCompensateAmp;
    int IDCfallCompensateMilliamp;

    // Flags and control variables
    bool scFlag, pwmRun, scCheck, negCurrentCheck, freqCorrect, tempCheck, simulRun;
    bool irefPotControl, overtempFlag, onlyPhase, backToBack, blankingBool, backToBackResistive, sensitivityAuto;
    bool meanCurrentCalc, negCurrentFlag, keepCurrentConstantBool;
    bool hass50Bool, hass100Bool, hass400Bool, chosenSensorBool, nonChosenSensorBool1, nonChosenSensorBool2;
    bool compensateCurrentFallBool;

    // Arrays for current measurement averaging
    int IDCarraySize = 5;
    int IDCarray[5] = {0, 0, 0, 0, 0};
    int IDCmean, IDCarraySum, k;

    // --- Hardware initialization ---

    // Timer configuration
    ConfigPtrTime = XScuTimer_LookupConfig(TIMER_DEVICE_ID);
    Status = XScuTimer_CfgInitialize(TimerInstancePtr, ConfigPtrTime, ConfigPtrTime->BaseAddr);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    XScuTimer_LoadTimer(TimerInstancePtr, XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2);
    XScuTimer_Start(TimerInstancePtr);
    timerPer = 1000.0 / 325.0; // Timer period in nanoseconds

    // GPIO setup for legs and buttons
    XGpio_Initialize(&leg1, XPAR_AXI_GPIO_1_DEVICE_ID);
    XGpio_SetDataDirection(&leg1, 1, 0x00000000);
    XGpio_DiscreteClear(&leg1, 1, legpin1);

    XGpio_Initialize(&leg2, XPAR_AXI_GPIO_0_DEVICE_ID);
    XGpio_SetDataDirection(&leg2, 1, 0x00000000);
    XGpio_DiscreteClear(&leg2, 1, legpin2);

    XGpio_Initialize(&btn, XPAR_AXI_GPIO_2_DEVICE_ID);
    XGpio_SetDataDirection(&btn, 1, 0xffffffff);

    // XADC configuration
    XAdcPs_Config *ConfigPtr;
    ConfigPtr = XAdcPs_LookupConfig(XADC_DEVICE_ID);
    if (ConfigPtr == NULL) return XST_FAILURE;

    XAdcPs_CfgInitialize(&XAdcInst, ConfigPtr, ConfigPtr->BaseAddress);
    XAdcPs_SetSequencerMode(&XAdcInst, XADCPS_SEQ_MODE_CONTINPASS);

    // --- Variable initializations ---

    // Calibration and measurement settings
    int calibrateSingleEnd = -505;
    int calibrateDiff = 0;
    scalar1SingleEnd = 1000 / 2.97;
    scalar2SingleEnd = 0.05 * 1000;
    scalar1Diff = 1000;
    scalar2Diff = 1 / 3.28;
    VDCcalibrate = calibrateDiff;
    threshold = 1500;
    VDCscalar1 = scalar1Diff;
    VDCscalar2 = scalar2Diff;
    IDCscalar1 = scalar1SingleEnd;
    IDCscalar2 = scalar2SingleEnd;

    // PWM parameters
    pwmStretch = 3;
    pwmCount = 0;
    pwmCountMax = 300;
    pwmRunCount = 0;

    // Power control parameters
    irefMax = 250;
    iref = 3000; // Example initial value
    freqRef = 5000; // Hz
    freqRefMargin = 200;
    IDCfake = 100E3;
    VDCfake = 40;
    V2 = 5;

    // Control timing
    fs = freqRef;
    fsCount = freqRef;
    VDC = VDCfake;
    L = 330E-6;
    R = 3;
    Ts = 1.0 / freqRef;
    ampereConvert = 1000;
    dutyAttenuate = 100;
    blankingMicro = 0;

    // Current measurement start times and amplitudes
    currentStartTime1 = 5;
    currentStartAmp1 = 100E3;
    currentStartTime2 = currentStartTime1 + 10;
    currentStartAmp2 = 10E3;
    currentStartTime3 = currentStartTime2 + 10;
    currentStartAmp3 = 10E3;

    // Thresholds
    scCurrentAbsoluteThresh = 150E3;
    negCurrentThresh = -100E3;
    instantCurrentThresh = 150E3;
    currentEndTime = 30 * 60; // 30 minutes

    // Sensor selection
    hass50Bool = false;
    hass100Bool = true;
    hass400Bool = false;
    chosenSensorBool = hass100Bool;
    if (chosenSensorBool == hass50Bool) {
        nonChosenSensorBool1 = hass100Bool;
        nonChosenSensorBool2 = hass400Bool;
    } else if (chosenSensorBool == hass100Bool) {
        nonChosenSensorBool1 = hass50Bool;
        nonChosenSensorBool2 = hass400Bool;
    }

    // Simulation variables
    IDCsim = 0;
    IDCsimPrev = IDCsim;
    VDCsimul = 500;

    // Small delay before starting main loop
    sleep(2);

    // --- Main control loop ---
    while (1)
    {
        // Set initial power states
        XGpio_DiscreteWrite(&leg1, 1, S100);
        XGpio_DiscreteWrite(&leg2, 1, S200);

        // Reset flags
        overtempFlag = false;
        scFlag = false;
        negCurrentFlag = false;

        // Read button state (simulate button press)
        btnCheck = XGpio_DiscreteRead(&leg1, 1);
        // For debugging, override button check
        btnCheck = 16255;
        printf("\r
Button state: %d\r
", btnCheck);

        usleep(200E3); // 200 ms delay

        // Check for button press to start PWM
        if ((btnCheck == 16255 || btnCheck == 16254 || btnCheck == 16239 || btnCheck == 16238) && pwmRunCount == 0)
        {
            sleep(1);
            pwmRun = true;
            duty2 = 50; // Set duty cycles
            duty1 = 50;
            duty1prev = duty1;
            periodCount = 0;
            iref = 0;
            IDCsim = 0;
            IDCsimDelta = 0;
            fs = 4000; // Hz
            // Initialize GPIO outputs
            XGpio_DiscreteClear(&leg1, 1, 0xFFFFFFFF);
            XGpio_SetDataDirection(&leg1, 1, 0x00000000); // input mode
            XGpio_DiscreteClear(&leg1, 1, 0x00000000);
            sleep(1);
            XGpio_DiscreteWrite(&leg1, 1, S100);
            XGpio_DiscreteWrite(&leg2, 1, S200);

            if (backToBack == false)
            {
                XGpio_DiscreteWrite(&leg2, 1, S200);
                scCheck = false;
            }
        }

        // PWM control loop
        while (pwmRun)
        {
            // Record timestamp at start of period
            if (pwmCount == 0)
            {
                CntValuePrev = XScuTimer_GetCounterValue(TimerInstancePtr);
            }

            // Simulation mode: update setpoints based on timing
            if (simulRun)
            {
                if (periodCount == fsCount)
                {
                    keepCurrentConstantBool = false;
                    iref = 1000;
                }
                if (periodCount == currentStartTime1 * fsCount)
                {
                    sensitivityAuto = true;
                    iref = currentStartAmp1;
                }
                if (periodCount == currentStartTime2 * fsCount)
                {
                    sensitivityAuto = false;
                    iref = currentStartAmp2;
                    sensitivityScalar = sensitivityScalar1;
                }
                if (periodCount == currentStartTime2 * fsCount + 1)
                {
                    if (!sensitivityAuto)
                        sensitivityScalar = sensitivityScalar2;
                }
                if (periodCount == currentStartTime3 * fsCount)
                {
                    keepCurrentConstantBool = false;
                    sensitivityAuto = false;
                    sensitivityScalar = sensitivityScalar1;
                    iref = currentStartAmp3;
                    duty1Thresh = duty1ThreshStart;
                }
                if (periodCount == currentEndTime * fsCount)
                {
                    iref = 1000;
                }
                if (periodCount == (currentEndTime + 1) * fsCount)
                {
                    scFlag = true; // End of simulation period
                }
            }

            // Calculate PWM signal switching points
            int groundCount = (int)(groundCountDuty * pwmCountMax / 100);
            int switchCount1 = (int)((100 - duty1) * pwmCountMax / 100);
            int switchCount2 = (int)((100 - duty2) * pwmCountMax / 100);

            // Generate PWM signals for back-to-back configuration
            if (backToBack)
            {
                // Leg 1 control
                if ((pwmCount == groundCount) && !keepCurrentConstantBool)
                {
                    XGpio_DiscreteWrite(&leg1, 1, S101);
                }

                // Leg 2 control
                if ((pwmCount == groundCount) && !keepCurrentConstantBool)
                {
                    XGpio_DiscreteWrite(&leg2, 1, S201);
                }

                // Sequence for leg 2 (main switching)
                if ((pwmCount == switchCount2) && (duty2 > 0) && !keepCurrentConstantBool)
                {
                    if (duty2 < 100)
                    {
                        XGpio_DiscreteWrite(&leg2, 1, S200);
                        if (blankingBool) usleep(blankingMicro);
                    }
                    XGpio_DiscreteWrite(&leg2, 1, S210);
                }

                // Sequence for leg 1 (main switching)
                if ((pwmCount == switchCount1) && !keepCurrentConstantBool)
                {
                    if (duty1 < 100)
                    {
                        XGpio_DiscreteWrite(&leg1, 1, S100);
                        if (blankingBool) usleep(blankingMicro);
                    }
                    XGpio_DiscreteWrite(&leg1, 1, S110);
                }
            }
            else
            {
                // Non back-to-back: operate only on leg 1
                XGpio_DiscreteWrite(&leg2, 1, S200); // Leg 2 off

                if ((pwmCount == 0) && (duty1 >= 0))
                {
                    XGpio_DiscreteWrite(&leg1, 1, S101);
                }

                if ((pwmCount == switchCount1) && (duty1 > 0))
                {
                    if (duty1 < 100)
                    {
                        XGpio_DiscreteWrite(&leg1, 1, S100);
                        if (blankingBool) usleep(blankingMicro);
                    }
                    XGpio_DiscreteWrite(&leg1, 1, S110);
                }
            }

            // --- Short circuit detection and measurements ---

            // Periodic ADC read for short circuit detection
            if (shortCircCount == 5) // Every 5 cycles
            {
                Vaux8_Raw = XAdcPs_GetAdcData(&XAdcInst, XADCPS_CH_AUX_MIN + 8);
                Vaux8_data = XAdcPs_RawToVoltage(Vaux8_Raw) * 1000; // mV
                IDCraw = (int)(Vaux8_data);

                // Convert raw ADC reading to current (example calibration)
                IDC = -237 * IDCraw + 310666;
                IDC = 2 * IDC; // Example scaling

                // Alternative calibration per sensor type
                if (chosenSensorBool && !nonChosenSensorBool1 && !nonChosenSensorBool2)
                {
                    if (chosenSensorBool == hass50Bool)
                    {
                        IDC = 232 * IDCraw - 386675; // Sensor 50 calibration
                    }
                    else if (chosenSensorBool == hass100Bool)
                    {
                        double IDCamp = -0.3293 * IDCraw + 688.3; // Sensor 100 calibration
                        IDC = IDCamp * 1000; // Convert to mA
                    }
                }
                else
                {
                    scFlag = true; // Fault if sensor config invalid
                }

                // Check for overcurrent conditions
                double deltaIDCAbs = fabs(deltaIDC);
                if ((IDC > scCurrentAbsoluteThresh) && (deltaIDC > 200E3) && scCheck)
                {
                    // Trigger short circuit response
                    duty1 = duty2;
                    scFlag = true; // Stop PWM
                }

                shortCircCount = 0; // Reset count
            }
            shortCircCount++;

            // --- PWM and timing updates ---

            pwmCount++;
            if (pwmCount == pwmCountMax)
            {
                // End of PWM cycle
                pwmCount = 0;
                periodCount++;

                // Save previous current value for filtering
                if (meanCurrentCalc)
                {
                    IDCprev5 = IDCprev4;
                    IDCprev4 = IDCprev3;
                    IDCprev3 = IDCprev2;
                    IDCprev2 = IDCprev;
                    IDCprev = IDC;
                }

                // Measure current at cycle end
                Vaux8_Raw = XAdcPs_GetAdcData(&XAdcInst, XADCPS_CH_AUX_MIN + 8);
                Vaux8_data = XAdcPs_RawToVoltage(Vaux8_Raw) * 1000;
                IDCraw = (int)(Vaux8_data);
                IDC = -237 * IDCraw + 310666; // Current calibration
                IDC = 2 * IDC;

                // Sensor-specific calibration
                if (chosenSensorBool && !nonChosenSensorBool1 && !nonChosenSensorBool2)
                {
                    if (chosenSensorBool == hass50Bool)
                        IDC = 232 * IDCraw - 386675;
                    else if (chosenSensorBool == hass100Bool)
                    {
                        double IDCamp = -0.3293 * IDCraw + 688.3;
                        IDC = IDCamp * 1000;
                    }
                }
                else
                {
                    scFlag = true;
                }

                // Calculate mean current
                if (meanCurrentCalc || negCurrentCheck)
                {
                    // Store in array for averaging
                    IDCarray[4] = IDC;
                    IDCarray[3] = IDCprev;
                    IDCarray[2] = IDCprev2;
                    IDCarray[1] = IDCprev3;
                    IDCarray[0] = IDCprev4;

                    // Compute mean
                    int sum = 0;
                    for (k = 0; k < IDCarraySize; k++)
                        sum += IDCarray[k];
                    IDCmean = sum / IDCarraySize;

                    if (IDCmean > -2000 && IDCmean < 1000)
                        IDCmean = 0;
                }

                // Negative current detection
                if (IDCmean < negCurrentThresh && negCurrentCheck)
                {
                    duty1 = duty2;
                    scFlag = true; // Stop PWM
                    negCurrentFlag = true;
                }

                // Change in current (delta)
                deltaIDC = IDC - IDCprev;
                duty1prev = duty1;

                // If no fault, compute duty ratio
                if (!scFlag)
                {
                    if (backToBack)
                    {
                        // Adjust duty based on feedback and control
                        if (freqCorrect == false)
                        {
                            if (compensateCurrentFallBool)
                            {
                                // Compensation calculations
                                IDCfallCompensateAmp = (VDC * groundCountDuty * TsPrint) / (L * 1E6);
                                IDCfallCompensateMilliamp = (int)(IDCfallCompensateAmp * 1000);
                                duty1 = 100 * (duty2 * VDC / 100 + sensitivityScalar * L * fs * (iref + IDCfallCompensateMilliamp - IDC) / 1000) / VDC;
                                duty1Check = duty1;
                            }
                            else
                            {
                                duty1 = 100 * (duty2 * VDC / 100 + sensitivityScalar * L * fs * (iref - IDC) / 1000) / VDC;
                            }
                        }
                        else
                        {
                            duty1 = duty2; // For stable frequency
                        }
                    }
                    else
                    {
                        // For normal DC operation
                        V1 = currentFake ? VDCfake : VDC;
                        duty1 = 100 * V2 / V1;
                    }
                }
                else
                {
                    // Fault detected: reduce duty to stop
                    duty2 -= 5;
                    duty1 = duty2;
                    if (duty2 <= 0)
                    {
                        XGpio_DiscreteClear(&leg1, 1, legpin1);
                        XGpio_DiscreteClear(&leg2, 1, legpin2);
                        duty2 = 0;
                        duty1 = 0;
                        sleep(1);
                    }
                }

                // Safety: limit duty ratios
                if (duty1 >= 100 - duty1Thresh)
                    duty1 = 100 - (duty1Thresh + 1);
                if (duty1 <= duty1Thresh)
                    duty1 = duty1Thresh + 1;

                // Set switching states based on duty
                if (duty2 > 0 && duty2 < 100 && !keepCurrentConstantBool)
                {
                    XGpio_DiscreteWrite(&leg2, 1, S200);
                }
                if (duty1 > 0 && duty1 < 100 && !keepCurrentConstantBool)
                {
                    XGpio_DiscreteWrite(&leg1, 1, S100);
                }
                if (blankingBool) usleep(blankingMicro);

                // Optional: print debug info periodically
                if (!(periodCount % (2 * fs)))
                {
                    printf("Voltage [V]: %d
", VDC);
                    printf("Frequency [Hz]: %d
", fs);
                    printf("Average current [mA]: %d
", IDCmean);
                }

                // Time measurement and frequency correction
                CntValueNow = XScuTimer_GetCounterValue(TimerInstancePtr);
                CntDiff = CntValueNow - CntValuePrev;
                CntDiff = -CntDiff * timerPer; // in seconds
                Ts = CntDiff;
                TsMicro = (int)(Ts * 1E6);
                TsPrint = (int)(Ts * 1E3);
                fs = (int)(1E9 / CntDiff);

                // Restart timer for next cycle
                XScuTimer_RestartTimer(TimerInstancePtr);

                // Frequency correction to match target
                if (freqCorrect && (periodCount > 1 * fs))
                {
                    if (fs < freqRef - freqRefMargin)
                        pwmCountMax--;
                    else if (fs > freqRef + freqRefMargin)
                        pwmCountMax++;
                    if (fs > freqRef - freqRefMargin && fs < freqRef + freqRefMargin)
                        freqCorrect = false;
                }
            } // end PWM cycle

        } // end pwmRun loop

        // --- End of PWM run, evaluate status ---

        printf("PWM running: %s
", pwmRun ? "true" : "false");
        printf("Short circuit detected: %s
", scFlag ? "true" : "false");
        printf("Negative current detected: %s
", negCurrentFlag ? "true" : "false");

        if (scFlag)
        {
            if (overtempFlag)
                printf("Processor temperature exceeded 70°C. Stopping.\n");
            else if (negCurrentFlag)
                printf("Negative current detected. Stopping.\n");
            else
                printf("Over current detected! Possible short circuit?\n");
        }

        // Increment run count
        pwmRunCount++;
        // Reset run flag for next cycle
        pwmRun = false;
    } // end main loop
} // end main



