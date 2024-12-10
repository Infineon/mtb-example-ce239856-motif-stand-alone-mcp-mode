/******************************************************************************
* File Name:   main.c
*
* Description: This code example demonstrate the MOTIF in stand alone multi
* channel mode. MOTIF is configured to generate the High side modulated pwm
* and low side is completely on.
* PWM multi channel pattern is updated every 1ms in hardware mode.
* High side pwm can be control using potentiometer.
* 1ms tcpwm timer used for PWM duty update and toggling the user LED.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/


/*******************************************************************************
* Global Variables
********************************************************************************/
/* The TCPWM patter update (PAT_UPDATE) interrupt configuration structure */
cy_stc_sysint_t patt_update_config =
{
    .intrSrc      = PAT_UPDATE_IRQ,
    .intrPriority = 1U
};

volatile uint32_t pot_value = 0u;       /* Hold the potentiometer value */

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */
/*******************************************************************************
* Function Prototypes
********************************************************************************/
void patterUpdate_intr_handler(void);
void PWMTimer_Init(void);
void PatternUpdateTimer_Init(void);
void TCPWM_Motif_Init(void);
void Interrupt_Config(void);
/*******************************************************************************
* Function Definition
********************************************************************************/
void PWMTimer_Init(void)
{
    /*PWM phase U Initialization*/
    Cy_TCPWM_PWM_Init(PWM_U_HW, PWM_U_NUM, &PWM_U_config);
    Cy_TCPWM_PWM_Enable(PWM_U_HW, PWM_U_NUM);

    /*PWM phase V Initialization*/
    Cy_TCPWM_PWM_Init(PWM_V_HW, PWM_V_NUM, &PWM_V_config);
    Cy_TCPWM_PWM_Enable(PWM_V_HW, PWM_V_NUM);

    /*PWM phase W Initialization*/
    Cy_TCPWM_PWM_Init(PWM_W_HW, PWM_W_NUM, &PWM_W_config);
    Cy_TCPWM_PWM_Enable(PWM_W_HW, PWM_W_NUM);
}

void PatternUpdateTimer_Init(void)
{
    /*Pattern update timer Initialization*/
    Cy_TCPWM_Counter_Init(PAT_UPDATE_HW,PAT_UPDATE_NUM,&PAT_UPDATE_config);
    Cy_TCPWM_Counter_Enable(PAT_UPDATE_HW,PAT_UPDATE_NUM);
}

void TCPWM_Motif_Init(void)
{
   /*Initialize Motif in stand alone multi channel mode*/
    Cy_TCPWM_MOTIF_MCP_Init(MOTIF0_HW,&MOTIF0_mcp_config);
    Cy_TCPWM_MOTIF_Update_MLUT(MOTIF0_HW,&mcp_mlut_config);
    Cy_TCPWM_MOTIF_Enable(MOTIF0_HW);
}


/* Interrupt handler configuration */
void Interrupt_Config(void)
{
    NVIC_ClearPendingIRQ((IRQn_Type)patt_update_config.intrSrc);
    Cy_SysInt_Init(&patt_update_config, patterUpdate_intr_handler);  /* Sets up the interrupt handler */
    NVIC_EnableIRQ((IRQn_Type)patt_update_config.intrSrc);           /* Enable Pattern update TC ISR */
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Start the HPPASS autonomous controller (AC) from state 0*/
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, 0U))
    {
        CY_ASSERT(0);
    }

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*Initialize Pattern update Timer*/
    PatternUpdateTimer_Init();

    /*Initialize MOTIF*/
    TCPWM_Motif_Init();

    Cy_TCPWM_MOTIF_Update_Multi_Channel_Pattern_Immediately(MOTIF0_HW); /*Immediate MCP update*/

    /*Initialize Phase U, Phase V and Phase W PWMs*/
    PWMTimer_Init();

    /* Interrupt handler Initialization */
    Interrupt_Config();

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("******************\r\n\n"
           "Peripherals Initialized\r\n\n"
           "MOTIF started\r\n\n"
           "****************** \r\n\n"
           "Observe the modulated PWM pattern on following Pins\r\n\n"
           "P4_0, P4_1, P4_2, P4_3, P4_4 and P4_5\r\n\n"
            "****************** \r\n\n"
            "PWM pattern update interval is 1ms\r\n\n"
            "Use potentiometer to change the PWM duty cycle\r\n\n"
              "****************** \r\n\n");

    Cy_TCPWM_MOTIF_Start(MOTIF0_HW); /*Start MOTIF*/

    for (;;)
    {

    }
}

/*******************************************************************************
* Function Name: patterUpdate_intr_handler
********************************************************************************
* Summary:
* This is the pattern update interrupt handler. This ISR executed every 1 ms.
* This ISR updating the PWM duty cycle using potentiometer.
* Toggling user LED to indicate execution of ISR.
*
* Parameters:
*  void
*
* Return:
*  void
*******************************************************************************/
void patterUpdate_intr_handler(void)
{
    /*Clear Interrupt*/
    Cy_TCPWM_ClearInterrupt(PAT_UPDATE_HW, PAT_UPDATE_NUM, CY_TCPWM_INT_ON_TC);

    /* Invert the USER LED state */
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

    volatile uint32_t compare_value = PWM_U_config.period0 >> 1U; //50% duty

    /* Read the potentiometer Value */
    pot_value    = Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_12_IDX); //AN_B4

    compare_value = PWM_U_config.period0 - ((pot_value * PWM_U_config.period0)/4095u);

    /*Limit the max duty count*/
    if(compare_value > PWM_U_config.period0){
        compare_value = PWM_U_config.period0;
    }

    /* Update compare value to the TCPWM ,Compare register*/
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_U_HW, PWM_U_NUM, compare_value);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_V_HW, PWM_V_NUM, compare_value);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_W_HW, PWM_W_NUM, compare_value);

    NVIC_ClearPendingIRQ((IRQn_Type)patt_update_config.intrSrc);

}

/* [] END OF FILE */
