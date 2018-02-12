/**
  ******************************************************************************
  * File Name          : OPAMP.c
  * Description        : This file provides code for the configuration
  *                      of the OPAMP instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "opamp.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef HAL_OPAMPEx_SelfCalibrateAll_50VDDA(OPAMP_HandleTypeDef *hopamp1, OPAMP_HandleTypeDef *hopamp2, OPAMP_HandleTypeDef *hopamp3, OPAMP_HandleTypeDef *hopamp4);

/* USER CODE END 0 */

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;
OPAMP_HandleTypeDef hopamp4;

/* OPAMP1 init function */
void MX_OPAMP1_Init(void)
{

  hopamp1.Instance = OPAMP1;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_USER;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* OPAMP2 init function */
void MX_OPAMP2_Init(void)
{

  hopamp2.Instance = OPAMP2;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_USER;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* OPAMP3 init function */
void MX_OPAMP3_Init(void)
{

  hopamp3.Instance = OPAMP3;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_USER;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* OPAMP4 init function */
void MX_OPAMP4_Init(void)
{

  hopamp4.Instance = OPAMP4;
  hopamp4.Init.Mode = OPAMP_PGA_MODE;
  hopamp4.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO3;
  hopamp4.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp4.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp4.Init.PgaGain = OPAMP_PGA_GAIN_16;
  hopamp4.Init.UserTrimming = OPAMP_TRIMMING_USER;
  if (HAL_OPAMP_Init(&hopamp4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* opampHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (opampHandle->Instance == OPAMP1)
  {
    /* USER CODE BEGIN OPAMP1_MspInit 0 */

    /* USER CODE END OPAMP1_MspInit 0 */

    /**OPAMP1 GPIO Configuration
    PA2     ------> OPAMP1_VOUT
    PA3     ------> OPAMP1_VINP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN OPAMP1_MspInit 1 */

    /* USER CODE END OPAMP1_MspInit 1 */
  }
  else if (opampHandle->Instance == OPAMP2)
  {
    /* USER CODE BEGIN OPAMP2_MspInit 0 */

    /* USER CODE END OPAMP2_MspInit 0 */

    /**OPAMP2 GPIO Configuration
    PA6     ------> OPAMP2_VOUT
    PA7     ------> OPAMP2_VINP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN OPAMP2_MspInit 1 */

    /* USER CODE END OPAMP2_MspInit 1 */
  }
  else if (opampHandle->Instance == OPAMP3)
  {
    /* USER CODE BEGIN OPAMP3_MspInit 0 */

    /* USER CODE END OPAMP3_MspInit 0 */

    /**OPAMP3 GPIO Configuration
    PA1     ------> OPAMP3_VINP
    PB1     ------> OPAMP3_VOUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN OPAMP3_MspInit 1 */

    /* USER CODE END OPAMP3_MspInit 1 */
  }
  else if (opampHandle->Instance == OPAMP4)
  {
    /* USER CODE BEGIN OPAMP4_MspInit 0 */

    /* USER CODE END OPAMP4_MspInit 0 */

    /**OPAMP4 GPIO Configuration
    PB11     ------> OPAMP4_VINP
    PB12     ------> OPAMP4_VOUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN OPAMP4_MspInit 1 */

    /* USER CODE END OPAMP4_MspInit 1 */
  }
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* opampHandle)
{

  if (opampHandle->Instance == OPAMP1)
  {
    /* USER CODE BEGIN OPAMP1_MspDeInit 0 */

    /* USER CODE END OPAMP1_MspDeInit 0 */

    /**OPAMP1 GPIO Configuration
    PA2     ------> OPAMP1_VOUT
    PA3     ------> OPAMP1_VINP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

    /* USER CODE BEGIN OPAMP1_MspDeInit 1 */

    /* USER CODE END OPAMP1_MspDeInit 1 */
  }
  else if (opampHandle->Instance == OPAMP2)
  {
    /* USER CODE BEGIN OPAMP2_MspDeInit 0 */

    /* USER CODE END OPAMP2_MspDeInit 0 */

    /**OPAMP2 GPIO Configuration
    PA6     ------> OPAMP2_VOUT
    PA7     ------> OPAMP2_VINP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7);

    /* USER CODE BEGIN OPAMP2_MspDeInit 1 */

    /* USER CODE END OPAMP2_MspDeInit 1 */
  }
  else if (opampHandle->Instance == OPAMP3)
  {
    /* USER CODE BEGIN OPAMP3_MspDeInit 0 */

    /* USER CODE END OPAMP3_MspDeInit 0 */

    /**OPAMP3 GPIO Configuration
    PA1     ------> OPAMP3_VINP
    PB1     ------> OPAMP3_VOUT
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* USER CODE BEGIN OPAMP3_MspDeInit 1 */

    /* USER CODE END OPAMP3_MspDeInit 1 */
  }
  else if (opampHandle->Instance == OPAMP4)
  {
    /* USER CODE BEGIN OPAMP4_MspDeInit 0 */

    /* USER CODE END OPAMP4_MspDeInit 0 */

    /**OPAMP4 GPIO Configuration
    PB11     ------> OPAMP4_VINP
    PB12     ------> OPAMP4_VOUT
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11 | GPIO_PIN_12);

    /* USER CODE BEGIN OPAMP4_MspDeInit 1 */

    /* USER CODE END OPAMP4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void OPAMP_Calibration(void)
{

  if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp2) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp3) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp4) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  HAL_OPAMPEx_SelfCalibrateAll_50VDDA(&hopamp1, &hopamp2, &hopamp3, &hopamp4);
}


void OPAMP_Start(void)
{
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_OPAMP_Start(&hopamp4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

HAL_StatusTypeDef HAL_OPAMPEx_SelfCalibrateAll_50VDDA(OPAMP_HandleTypeDef *hopamp1, OPAMP_HandleTypeDef *hopamp2, OPAMP_HandleTypeDef *hopamp3, OPAMP_HandleTypeDef *hopamp4)
{
  HAL_StatusTypeDef status = HAL_OK;

  uint32_t trimmingvaluen1 = 0U;
  uint32_t trimmingvaluep1 = 0U;
  uint32_t trimmingvaluen2 = 0U;
  uint32_t trimmingvaluep2 = 0U;
  uint32_t trimmingvaluen3 = 0U;
  uint32_t trimmingvaluep3 = 0U;
  uint32_t trimmingvaluen4 = 0U;
  uint32_t trimmingvaluep4 = 0U;

  uint32_t delta;

  if ((hopamp1 == NULL) || (hopamp1->State == HAL_OPAMP_STATE_BUSYLOCKED) || \
      (hopamp2 == NULL) || (hopamp2->State == HAL_OPAMP_STATE_BUSYLOCKED) || \
      (hopamp3 == NULL) || (hopamp3->State == HAL_OPAMP_STATE_BUSYLOCKED) || \
      (hopamp4 == NULL) || (hopamp4->State == HAL_OPAMP_STATE_BUSYLOCKED))
  {
    status = HAL_ERROR;
  }

  if (status == HAL_OK)
  {
    /* Check if OPAMP in calibration mode and calibration not yet enable */
    if ((hopamp1->State ==  HAL_OPAMP_STATE_READY) && (hopamp2->State ==  HAL_OPAMP_STATE_READY) && \
        (hopamp3->State ==  HAL_OPAMP_STATE_READY) && (hopamp4->State ==  HAL_OPAMP_STATE_READY))
    {
      /* Check the parameter */
      assert_param(IS_OPAMP_ALL_INSTANCE(hopamp1->Instance));
      assert_param(IS_OPAMP_ALL_INSTANCE(hopamp2->Instance));
      assert_param(IS_OPAMP_ALL_INSTANCE(hopamp3->Instance));
      assert_param(IS_OPAMP_ALL_INSTANCE(hopamp4->Instance));

      /* Set Calibration mode */
      /* Non-inverting input connected to calibration reference voltage. */
      SET_BIT(hopamp1->Instance->CSR, OPAMP_CSR_FORCEVP);
      SET_BIT(hopamp2->Instance->CSR, OPAMP_CSR_FORCEVP);
      SET_BIT(hopamp3->Instance->CSR, OPAMP_CSR_FORCEVP);
      SET_BIT(hopamp4->Instance->CSR, OPAMP_CSR_FORCEVP);

      /*  user trimming values are used for offset calibration */
      SET_BIT(hopamp1->Instance->CSR, OPAMP_CSR_USERTRIM);
      SET_BIT(hopamp2->Instance->CSR, OPAMP_CSR_USERTRIM);
      SET_BIT(hopamp3->Instance->CSR, OPAMP_CSR_USERTRIM);
      SET_BIT(hopamp4->Instance->CSR, OPAMP_CSR_USERTRIM);

      /* Enable calibration */
      SET_BIT (hopamp1->Instance->CSR, OPAMP_CSR_CALON);
      SET_BIT (hopamp2->Instance->CSR, OPAMP_CSR_CALON);
      SET_BIT (hopamp3->Instance->CSR, OPAMP_CSR_CALON);
      SET_BIT (hopamp4->Instance->CSR, OPAMP_CSR_CALON);

      /* 1st calibration - N */
      /* Select 90U% VREF */
      MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);
      MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);
      MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);
      MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);

      /* Enable the opamps */
      SET_BIT (hopamp1->Instance->CSR, OPAMP_CSR_OPAMPxEN);
      SET_BIT (hopamp2->Instance->CSR, OPAMP_CSR_OPAMPxEN);
      SET_BIT (hopamp3->Instance->CSR, OPAMP_CSR_OPAMPxEN);
      SET_BIT (hopamp4->Instance->CSR, OPAMP_CSR_OPAMPxEN);

      /* Init trimming counter */
      /* Medium value */
      trimmingvaluen1 = 16U;
      trimmingvaluen2 = 16U;
      trimmingvaluen3 = 16U;
      trimmingvaluen4 = 16U;
      delta = 8U;

      while (delta != 0U)
      {
        /* Set candidate trimming */
        MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen1 << OPAMP_INPUT_INVERTING);
        MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen2 << OPAMP_INPUT_INVERTING);
        MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen3 << OPAMP_INPUT_INVERTING);
        MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen4 << OPAMP_INPUT_INVERTING);

        /* OFFTRIMmax delay 2 ms as per datasheet (electrical characteristics */
        /* Offset trim time: during calibration, minimum time needed between */
        /* two steps to have 1 mV accuracy */
        HAL_Delay(2U);

        if ((hopamp1->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluen1 += delta;
        }
        else
        {
          /* OPAMP_CSR_OUTCAL is LOW try lower trimming */
          trimmingvaluen1 -= delta;
        }

        if ((hopamp2->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluen2 += delta;
        }
        else
        {
          /* OPAMP_CSR_OUTCAL is LOW try lower trimming */
          trimmingvaluen2 -= delta;
        }

        if ((hopamp3->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluen3 += delta;
        }
        else
        {
          /* OPAMP_CSR_OUTCAL is LOW try lower trimming */
          trimmingvaluen3 -= delta;
        }

        if ((hopamp4->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluen4 += delta;
        }
        else
        {
          /* OPAMP_CSR_OUTCAL is LOW try lower trimming */
          trimmingvaluen4 -= delta;
        }

        delta >>= 1U;
      }

      /* Still need to check if righ calibration is current value or un step below */
      /* Indeed the first value that causes the OUTCAL bit to change from 1 to 0U */
      MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen1 << OPAMP_INPUT_INVERTING);
      MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen2 << OPAMP_INPUT_INVERTING);
      MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen3 << OPAMP_INPUT_INVERTING);
      MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen4 << OPAMP_INPUT_INVERTING);

      /* OFFTRIMmax delay 2 ms as per datasheet (electrical characteristics */
      /* Offset trim time: during calibration, minimum time needed between */
      /* two steps to have 1 mV accuracy */
      HAL_Delay(2U);

      if ((hopamp1->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* OPAMP_CSR_OUTCAL is actually one value more */
        trimmingvaluen1++;
        /* Set right trimming */
        MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen1 << OPAMP_INPUT_INVERTING);
      }

      if ((hopamp2->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* OPAMP_CSR_OUTCAL is actually one value more */
        trimmingvaluen2++;
        /* Set right trimming */
        MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen2 << OPAMP_INPUT_INVERTING);
      }

      if ((hopamp3->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* OPAMP_CSR_OUTCAL is actually one value more */
        trimmingvaluen3++;
        /* Set right trimming */
        MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen3 << OPAMP_INPUT_INVERTING);
      }

      if ((hopamp4->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* OPAMP_CSR_OUTCAL is actually one value more */
        trimmingvaluen4++;
        /* Set right trimming */
        MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen4 << OPAMP_INPUT_INVERTING);
      }

      /* 2nd calibration - P */
      /* Select 10U% VREF */
      MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);
      MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);
      MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);
      MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_CALSEL, OPAMP_VREF_50VDDA);

      /* Init trimming counter */
      /* Medium value */
      trimmingvaluep1 = 16U;
      trimmingvaluep2 = 16U;
      trimmingvaluep3 = 16U;
      trimmingvaluep4 = 16U;

      delta = 8U;

      while (delta != 0U)
      {
        /* Set candidate trimming */
        MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep1 << OPAMP_INPUT_NONINVERTING);
        MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep2 << OPAMP_INPUT_NONINVERTING);
        MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep3 << OPAMP_INPUT_NONINVERTING);
        MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep4 << OPAMP_INPUT_NONINVERTING);

        /* OFFTRIMmax delay 2 ms as per datasheet (electrical characteristics */
        /* Offset trim time: during calibration, minimum time needed between */
        /* two steps to have 1 mV accuracy */
        HAL_Delay(2U);

        if ((hopamp1->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluep1 += delta;
        }
        else
        {
          trimmingvaluep1 -= delta;
        }

        if ((hopamp2->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluep2 += delta;
        }
        else
        {
          trimmingvaluep2 -= delta;
        }

        if ((hopamp3->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluep3 += delta;
        }
        else
        {
          trimmingvaluep3 -= delta;
        }

        if ((hopamp4->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
        {
          /* OPAMP_CSR_OUTCAL is HIGH try higher trimming */
          trimmingvaluep4 += delta;
        }
        else
        {
          trimmingvaluep4 -= delta;
        }

        delta >>= 1U;
      }

      /* Still need to check if righ calibration is current value or un step below */
      /* Indeed the first value that causes the OUTCAL bit to change from 1 to 0U */
      /* Set candidate trimming */
      MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep1 << OPAMP_INPUT_NONINVERTING);
      MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep2 << OPAMP_INPUT_NONINVERTING);
      MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep3 << OPAMP_INPUT_NONINVERTING);
      MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep4 << OPAMP_INPUT_NONINVERTING);

      /* OFFTRIMmax delay 2 ms as per datasheet (electrical characteristics */
      /* Offset trim time: during calibration, minimum time needed between */
      /* two steps to have 1 mV accuracy */
      HAL_Delay(2U);

      if ((hopamp1->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* Trimming value is actually one value more */
        trimmingvaluep1++;
        /* Set right trimming */
        MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep1 << OPAMP_INPUT_NONINVERTING);
      }

      if ((hopamp2->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* Trimming value is actually one value more */
        trimmingvaluep2++;
        /* Set right trimming */
        MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep2 << OPAMP_INPUT_NONINVERTING);
      }

      if ((hopamp3->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* Trimming value is actually one value more */
        trimmingvaluep3++;
        /* Set right trimming */
        MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep3 << OPAMP_INPUT_NONINVERTING);
      }

      if ((hopamp4->Instance->CSR & OPAMP_CSR_OUTCAL) != RESET)
      {
        /* Trimming value is actually one value more */
        trimmingvaluep4++;
        /* Set right trimming */
        MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep4 << OPAMP_INPUT_NONINVERTING);
      }

      /* Disable calibration */
      CLEAR_BIT (hopamp1->Instance->CSR, OPAMP_CSR_CALON);
      CLEAR_BIT (hopamp2->Instance->CSR, OPAMP_CSR_CALON);
      CLEAR_BIT (hopamp3->Instance->CSR, OPAMP_CSR_CALON);
      CLEAR_BIT (hopamp4->Instance->CSR, OPAMP_CSR_CALON);

      /* Disable the OPAMPs */
      CLEAR_BIT (hopamp1->Instance->CSR, OPAMP_CSR_OPAMPxEN);
      CLEAR_BIT (hopamp2->Instance->CSR, OPAMP_CSR_OPAMPxEN);
      CLEAR_BIT (hopamp3->Instance->CSR, OPAMP_CSR_OPAMPxEN);
      CLEAR_BIT (hopamp4->Instance->CSR, OPAMP_CSR_OPAMPxEN);

      /* Set normal operating mode back */
      CLEAR_BIT(hopamp1->Instance->CSR, OPAMP_CSR_FORCEVP);
      CLEAR_BIT(hopamp2->Instance->CSR, OPAMP_CSR_FORCEVP);
      CLEAR_BIT(hopamp3->Instance->CSR, OPAMP_CSR_FORCEVP);
      CLEAR_BIT(hopamp4->Instance->CSR, OPAMP_CSR_FORCEVP);

      /* Self calibration is successful  */
      /* Store calibration(user timming) results in init structure. */
      /* Select user timming mode */

      /* Write calibration result N */
      hopamp1->Init.TrimmingValueN = trimmingvaluen1;
      hopamp2->Init.TrimmingValueN = trimmingvaluen2;
      hopamp3->Init.TrimmingValueN = trimmingvaluen3;
      hopamp4->Init.TrimmingValueN = trimmingvaluen4;

      /* Write calibration result P */
      hopamp1->Init.TrimmingValueP = trimmingvaluep1;
      hopamp2->Init.TrimmingValueP = trimmingvaluep2;
      hopamp3->Init.TrimmingValueP = trimmingvaluep3;
      hopamp4->Init.TrimmingValueP = trimmingvaluep4;

      /* Select user timming mode */
      /* And updated with calibrated settings */
      hopamp1->Init.UserTrimming = OPAMP_TRIMMING_USER;
      hopamp2->Init.UserTrimming = OPAMP_TRIMMING_USER;
      hopamp3->Init.UserTrimming = OPAMP_TRIMMING_USER;
      hopamp4->Init.UserTrimming = OPAMP_TRIMMING_USER;

      MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen1 << OPAMP_INPUT_INVERTING);
      MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen2 << OPAMP_INPUT_INVERTING);
      MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen3 << OPAMP_INPUT_INVERTING);
      MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETN, trimmingvaluen4 << OPAMP_INPUT_INVERTING);

      MODIFY_REG(hopamp1->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep1 << OPAMP_INPUT_NONINVERTING);
      MODIFY_REG(hopamp2->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep2 << OPAMP_INPUT_NONINVERTING);
      MODIFY_REG(hopamp3->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep3 << OPAMP_INPUT_NONINVERTING);
      MODIFY_REG(hopamp4->Instance->CSR, OPAMP_CSR_TRIMOFFSETP, trimmingvaluep4 << OPAMP_INPUT_NONINVERTING);

    }

    else
    {
      /* At least one OPAMP can not be calibrated */
      status = HAL_ERROR;
    }
  }

  return status;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
