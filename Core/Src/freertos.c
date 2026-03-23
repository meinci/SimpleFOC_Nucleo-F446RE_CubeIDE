/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  *
  * HINWEIS: MX_FREERTOS_Init() wird NICHT von main.c aufgerufen!
  * Alle echten Tasks (FOC_Task, Log_Task) werden direkt in main.c
  * mit osThreadNew() erstellt und verwaltet.
  *
  * StartFOC_Task / StartLog_Task sind leere CubeMX-Stubs – sie laufen nicht.
  * defaultTask läuft als reiner Idle-Fallback.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for FOC_Task (CubeMX-Stub – wird NICHT gestartet) */
osThreadId_t FOC_TaskHandle;
const osThreadAttr_t FOC_Task_attributes = {
  .name = "FOC_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Definitions for Log_Task (CubeMX-Stub – wird NICHT gestartet) */
osThreadId_t Log_TaskHandle;
const osThreadAttr_t Log_Task_attributes = {
  .name = "Log_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

void StartDefaultTask(void *argument);
void StartFOC_Task(void *argument);
void StartLog_Task(void *argument);

void MX_FREERTOS_Init(void)
{
  /* HINWEIS: Diese Funktion wird von main.c NICHT aufgerufen!
   * Tasks werden direkt in main.c erstellt.
   * Nur defaultTask wird hier für den Fall registriert, dass
   * MX_FREERTOS_Init() doch einmal aufgerufen wird. */

  /* USER CODE BEGIN RTOS_THREADS */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  /* FOC_TaskHandle  = osThreadNew(StartFOC_Task,  NULL, &FOC_Task_attributes);  // NICHT starten! */
  /* Log_TaskHandle  = osThreadNew(StartLog_Task,  NULL, &Log_Task_attributes);  // NICHT starten! */
  /* USER CODE END RTOS_THREADS */
}

/* ── defaultTask: Idle-Fallback, läuft nur wenn alle anderen Tasks blockiert ── */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* ── StartFOC_Task: CubeMX-Stub, wird NICHT gestartet ── */
void StartFOC_Task(void *argument)
{
  /* USER CODE BEGIN StartFOC_Task */
  /* Leer – echter FOC-Task läuft in main.c */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartFOC_Task */
}

/* ── StartLog_Task: CubeMX-Stub, wird NICHT gestartet ── */
void StartLog_Task(void *argument)
{
  /* USER CODE BEGIN StartLog_Task */
  /* Leer – echter Log-Task läuft in main.c */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartLog_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
