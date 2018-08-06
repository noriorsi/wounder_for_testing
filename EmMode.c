
#include "EmMode.h"


/******************************************************************************************
 * 				STATIC FUNCTIONS
 *******************************************************************************************/


/**************************************************************************//**
 * @brief   Disable high frequency clocks
 *****************************************************************************/
static void disableHFClocks(void)
{
 // Disable High Frequency Peripheral Clocks
  CMU_ClockEnable(cmuClock_HFPER, false);
#if defined( CMU_HFPERCLKEN0_USART1 )
  CMU_ClockEnable(cmuClock_USART1, false);
#endif
  CMU_ClockEnable(cmuClock_USART0, false);
  CMU_ClockEnable(cmuClock_TIMER0, false);
  CMU_ClockEnable(cmuClock_TIMER1, false);
#if defined( CMU_HFPERCLKEN0_TIMER2 )
  CMU_ClockEnable(cmuClock_TIMER2, false);
#endif
  CMU_ClockEnable(cmuClock_ACMP0, false);
  CMU_ClockEnable(cmuClock_PRS, false);
  CMU_ClockEnable(cmuClock_IDAC0, false);
  CMU_ClockEnable(cmuClock_GPIO, false);
  CMU_ClockEnable(cmuClock_VCMP, false);
  CMU_ClockEnable(cmuClock_ADC0, false);
  CMU_ClockEnable(cmuClock_I2C0, false);

  // Disable High Frequency Core/Bus Clocks
  CMU_ClockEnable(cmuClock_AES, false);
  CMU_ClockEnable(cmuClock_DMA, false);
  CMU_ClockEnable(cmuClock_HFLE, false);
#if defined( CMU_HFCORECLKEN0_USB )
  CMU_ClockEnable(cmuClock_USB, false);
#endif
#if defined( CMU_HFCORECLKEN0_USBC )
  // Disable USBC clock by choosing unused oscillator (LFXO)
  CMU_ClockEnable(cmuClock_USBC, true);
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  CMU_ClockSelectSet(cmuClock_USBC, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_USBC, false);
#endif
}

/**************************************************************************//**
 * @brief   Disable low frequency clocks
 *****************************************************************************/
static void disableLFClocks(void)
{
    // Enable LFXO for Low Frequency Clock Disables
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  // Disable Low Frequency A Peripheral Clocks
  // Note: LFA clock must be sourced before modifying peripheral clock enables
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_RTC, false);
    CMU_ClockEnable(cmuClock_PCNT0, false);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_Disabled);

  // Disable Low Frequency B Peripheral Clocks
  // Note: LFB clock must be sourced before modifying peripheral clock enables
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_LEUART0, false);
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);

#if defined( _CMU_LFCCLKEN0_MASK )
  // Disable Low Frequency C Peripheral Clocks
  // Note: LFC clock must be sourced before modifying peripheral clock enables
    CMU_ClockSelectSet(cmuClock_LFC, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_USBLE, false);
    CMU_ClockSelectSet(cmuClock_LFC, cmuSelect_Disabled);
#endif

    // Disable Low Frequency Oscillator
    CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
}

/**************************************************************************//**
 * @brief   Disable all clocks to achieve lowest current consumption numbers.
 *****************************************************************************/
static void disableClocks(void)
{
    // Disable High Frequency Clocks
    disableHFClocks();

    // Disable Low Frequency Clocks
    disableLFClocks();
}


/***************************************************************
 * Enables all the clock that were initialle set up
 ***************************************************************/
static void EnableClocks(){
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_USART1, true);
	CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_ADC0, true);
}



/******************************************************************************************
 * 				 FUNCTIONS
 *******************************************************************************************/












/***************************************************************************//**
 * Enter EM0
 ******************************************************************************/
void em_EM0_Hfxo(void)
{
	EnableClocks();
}



/***************************************************************************//**
 * @brief
 *   Enter EM0 with HFRCO running at desired frequency.
 *
 * @param[in] band
 *   HFRCO band to activate (21, 14, 11, 6.6, 1.2 MHz).
 *
 * @details
 *   Parameter:
 *     EM0. No prescaling. Running prime number calculation code
 *     from Flash (Production test condition = 14MHz).@n
 *   Condition:
 *     Between 21 and 1.2 MHz HFRCO, all peripheral clocks disabled, VDD= 3.0V.@n
 *
 * @note
 *   To better understand disabling clocks and oscillators for specific modes,
 *   see Reference Manual section EMU-Energy Management Unit and Table 10.1.
 ******************************************************************************/
void em_EM0_Hfrco(CMU_HFRCOBand_TypeDef band)
{
  // Make sure clocks are disabled.
  disableClocks();

  // Set HFRCO for HF clock.
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

  // Set HFRCO frequency.
  CMU_HFRCOBandSet(band);

  // Make sure unwanted oscillators are disabled specifically for EM0 and HFRCO.
  CMU_OscillatorEnable(cmuOsc_HFXO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFRCO, false, true);
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, false, true);
#if defined(CMU_OSCENCMD_USHFRCOEN)
  CMU_OscillatorEnable(cmuOsc_USHFRCO, false, true);
#endif

}

/***************************************************************************//**
 * @brief
 *   Enter EM1 with HFXO running at 24MHz.
 *
 * @details
 *   Parameter:
       EM1 (Production test condition = 14 MHz).@n
 *   Condition:
 *     24 MHz HFXO, all peripheral clocks disabled, VDD= 3.0 V.@n
 *
 * @note
 *   To better understand disabling clocks and oscillators for specific modes,
 *   see Reference Manual section EMU-Energy Management Unit and Table 10.1.
 ******************************************************************************/
void em_EM1_Hfxo(void)
{
  // Make sure clocks are disabled.
  disableClocks();

  // Set HFXO for HF clock.
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  // Make sure unwanted oscillators are disabled specifically for EM1 and HFXO.
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFRCO, false, true);
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, false, true);
#if defined(CMU_OSCENCMD_USHFRCOEN)
  CMU_OscillatorEnable(cmuOsc_USHFRCO, false, true);
#endif

  // Enter EM1.
  EMU_EnterEM1();
}

/***************************************************************************//**
 * @brief
 *   Enter EM1 with HFRCO running at desired frequency.
 *
 * @param[in] band
 *   HFRCO band to activate (21, 14, 11, 6.6, 1.2 MHz).
 *
 * @details
 *   Parameter:
 *     EM1 (Production test condition = 14 MHz).@n
 *   Condition:
 *     Between 21 and 1.2 MHz HFRCO, all peripheral clocks disabled, VDD= 3.0 V.@n
 *
 * @note
 *   To better understand disabling clocks and oscillators for specific modes,
 *   see Reference Manual section EMU-Energy Management Unit and Table 10.1.
 ******************************************************************************/
void em_EM1_Hfrco(CMU_HFRCOBand_TypeDef band)
{
  // Make sure clocks are disabled.
  disableClocks();

  // Set HFRCO for HF clock.
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

  // Set HFRCO frequency.
  CMU_HFRCOBandSet(band);

  // Make sure unwanted oscillators are disabled specifically for EM1 and HFRCO.
  CMU_OscillatorEnable(cmuOsc_HFXO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFRCO, false, true);
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, false, true);
#if defined(CMU_OSCENCMD_USHFRCOEN)
  CMU_OscillatorEnable(cmuOsc_USHFRCO, false, true);
#endif

  // Enter EM1.
  EMU_EnterEM1();
}

/***************************************************************************//**
 * @brief
 *   Enter EM2 with RTC running with LFRCO.
 *
 * @details
 *   Parameter:
       EM2.@n
 *   Condition:
 *     RTC prescaled to 1 Hz, 32.768 kHz LFRCO, VDD= 3.0 V.@n
 *
 * @note
 *   To better understand disabling clocks and oscillators for specific modes,
 *   see Reference Manual section EMU-Energy Management Unit and Table 10.1.
 ******************************************************************************/
void em_EM2_LfrcoRTC(void)
{
  // Make sure clocks are disabled.
  disableClocks();

  // Route the LFRCO clock to RTC.
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);

  // Configure RTC to 1Hz.
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_32768);

  // Enable clock to the interface with low energy modules.
  CMU_ClockEnable(cmuClock_CORELE, true);

  // Setup RTC parameters.
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

  // Initialize RTC.
  RTC_Init(&rtcInit);

  // Make sure unwanted oscillators are disabled specifically for EM2 and LFRCO.
  CMU_OscillatorEnable(cmuOsc_LFXO, false, true);

  // Enter EM2.
  EMU_EnterEM2(false);
}

/***************************************************************************//**
 * @brief
 *   Enter EM3.
 *
 * @details
 *   Parameter:
       EM3.@n
 *   Condition:
 *     VDD= 3.0 V, ULFRCO enabled, LFRCO/LFXO disabled.@n
 *
 * @note
 *   To better understand disabling clocks and oscillators for specific modes,
 *   see Reference Manual section EMU-Energy Management Unit and Table 10.1.
 ******************************************************************************/
void EnterEM3(void)
{
	SetGPIO(MCULED1_PORT,MCULED1_PIN,0);
	SetGPIO(MCULED2_PORT,MCULED2_PIN,0);
	SetGPIO(MCULED3_PORT,MCULED3_PIN,0);
  // High and low frequency clocks are disabled in EM3.

  // ULFRCO is always running from EM0-EM3.

  // All unwanted oscillators are disabled in EM3.

  // Enter EM3.
  EMU_EnterEM3(false);
}

/***************************************************************************//**
 * @brief
 *   Enter EM4.
 *
 * @details
 *   Parameter:
       EM4.
 *   Condition:
 *     VDD= 3.0 V.
 *
 * @note
 *   To better understand disabling clocks and oscillators for specific modes,
 *   see Reference Manual section EMU-Energy Management Unit and Table 10.1.
 ******************************************************************************/
void EnterEM4(void)
{

	InitRFduinoUART();
	RFDuino_GiveIT();
	send_string("Entering EM4 (ULP) state.\n");
	SendEmpty(100);
	send_RFDuino_command(CMD[CMD_SLEEP]);
	SendEmpty(100);





  // High and low frequency clocks are disabled in EM4.

  // All unwanted oscillators are disabled in EM4.

  // Enter EM4.
  EMU_EnterEM4();
}
