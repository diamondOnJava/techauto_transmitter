/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "LoRa.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_BUFFER_MAX_LENGTH 64
#define TIME_BUFFER_LENGTH 2

#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA

#define DEBUG_ENABLED 1 // Установить на 1 для включения отладки

#if DEBUG_ENABLED
#define DEBUG_MSG(msg)                                              \
  do                                                                \
  {                                                                 \
    HAL_UART_Transmit(&huart2, (uint8_t *)(msg), strlen(msg), 100); \
  } while (0)
#define DEBUG_PRINTF(fmt, ...)                                    \
  do                                                              \
  {                                                               \
    char buf[64];                                                 \
    snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__);               \
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 100); \
  } while (0)
#else
#define DEBUG_MSG(msg)
#define DEBUG_PRINTF(fmt, ...)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef struct
{
  uint8_t rank;
  uint8_t network_id;
  size_t buffer_size;
  char **buffer;
} Node;

typedef enum
{
  STATE_SLEEP = 0,         // Сон до следующего события
  STATE_MEASUREMENT,       // Выполнение измерения
  STATE_WAIT_DATA,         // Ожидание данных от предыдущих узлов
  STATE_TRANSMIT,          // Передача данных
  STATE_RECEIVE_TIMEOUT,   // Таймаут приема
  STATE_WAIT_SYNC,         // Ожидание синхронизации времени от приемника
  STATE_WAIT_CONFIGURATION // Ожидание конфигурации устройства при первой загрузке
} DeviceState;

typedef struct
{
  uint8_t rank;                  // Ранг устройства (нумерация начинается с единицы!!!)
  uint8_t network_id;            // Идентификатор сети
  uint8_t buffer_size;           // Размер буфера для пакета
  uint16_t wait_timeout;         // Таймаут ожидания от предыдущих узлов
  uint16_t sync_wait_timeout;    // Таймаут ожидания сообщения с временем от приемника
  uint8_t default_alarm_hours;   // Часы будильника по умолчанию
  uint8_t default_alarm_minutes; // Минуты будильника по умолчанию
  uint16_t wakeup_timeout;       // Время пробуждения по таймеру RTC в миллисекундах
  // DeviceState mode;              // Режим работы устройства (синхронизированный или же нет)
} DeviceConfig;

Node *this_transmitter;
LoRa lora_module;
DeviceConfig device_config;
DeviceState current_state;
size_t packet_size = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

float read_voltage_ads1115(void);
void transmit_data(Node *node);
void reset_lora();
void check_spi_config();
void SystemClock_Config(void);
void enter_to_sleep(void);
void ads1115_init(void);
void RTC_Init(uint16_t timeout_ms);
Node *configure_node(uint8_t rank, uint8_t network_id, size_t buffer_size);
int push_to_packet(Node *transmitter, uint8_t rank, char *buffer);
void receive_transmit();
void sync_time_with_receiver(uint16_t sync_wait_timeout);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*********************  Функции для работы с Node и обработки пакетов *********/

Node *configure_node(uint8_t rank, uint8_t network_id, size_t buffer_size)
{
  Node *transmitter = (Node *)malloc(sizeof(Node));
  if (!transmitter)
  {
    DEBUG_MSG("Error with Node initialization! \r\n");
    return NULL;
  }

  transmitter->rank = rank;
  transmitter->network_id = network_id;
  transmitter->buffer_size = buffer_size;
  transmitter->buffer = (char **)malloc(rank * sizeof(char *));

  if (!transmitter->buffer)
  {
    DEBUG_MSG("Error with Node's buffer initialization! \r\n");
    free(transmitter);
    return NULL;
  }

  for (size_t i = 0; i < rank; i++)
  {
    transmitter->buffer[i] = (char *)malloc(buffer_size + 1);
    if (!transmitter->buffer[i])
    {
      DEBUG_MSG("Error allocating buffer row! \r\n");
      // Освобождаем уже выделенную память
      for (size_t j = 0; j < i; j++)
      {
        free(transmitter->buffer[j]);
      }
      free(transmitter->buffer);
      free(transmitter);
      return NULL;
    }
    transmitter->buffer[i][0] = '\0';
  }

  // Правильная отладочная информация
  char msg[50];
  snprintf(msg, sizeof(msg), "Configured Node with params: N%dD%d \r\n",
           transmitter->network_id, transmitter->rank);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 200);

  return transmitter;
}


int push_to_packet(Node *transmitter, uint8_t rank, char *buffer)
{
  if (!transmitter || !buffer || rank > transmitter->rank)
  {
    return -1;
  }
  uint8_t index = rank - 1;

  // strncpy автоматически добавляет null-terminator
  // (если строка меньше, чем max_size)
  strncpy(transmitter->buffer[index], buffer,
          transmitter->buffer_size - 1);

  // Гарантируем null-terminator в конце
  transmitter->buffer[index][transmitter->buffer_size - 1] = '\0';

  return 0;
}

size_t serialize(Node *node, uint8_t *buffer, size_t buffer_len)
{
  if (!node || !buffer)
  {
    DEBUG_MSG("ERROR: Invalid parameters\r\n");
    return 0;
  }

  size_t offset = 0;

  if (offset + sizeof(node->network_id) > buffer_len)
  {
    DEBUG_MSG("ERROR: Buffer overflow at network_id\r\n");
    return 0;
  }

  memcpy(buffer + offset, &node->network_id, sizeof(node->network_id));
  offset += sizeof(node->network_id);

  if (offset + sizeof(node->rank) > buffer_len)
  {
    DEBUG_MSG("ERROR: Buffer overflow at rank\r\n");
    return 0;
  }

  memcpy(buffer + offset, &node->rank, sizeof(node->rank));
  offset += sizeof(node->rank);

  for (uint8_t i = 0; i < node->rank; i++)
  {
    if (!node->buffer || !node->buffer[i])
    {
      char warn[80];
      snprintf(warn, sizeof(warn),
               "WARNING: Empty buffer at slot %d\r\n", i);
      DEBUG_MSG(warn);
      continue;
    }

    size_t len = strlen(node->buffer[i]);

    // Проверяем, влезают ли данные в буфер
    if (offset + len + 1 > buffer_len)
    {
      char overflow[80];
      snprintf(overflow, sizeof(overflow),
               "WARNING: Not enough space for string %d (need %u, have %u)\r\n",
               i, (unsigned int)len,
               (unsigned int)(buffer_len - offset));
      DEBUG_MSG(overflow);
      break;
    }

    memcpy(buffer + offset, node->buffer[i], len + 1);
    offset += len + 1;

    char str_msg[50];
    snprintf(str_msg, sizeof(str_msg),
             "String %d: '%s' (len=%u), offset now=%u\r\n",
             i, node->buffer[i], (unsigned int)len, (unsigned int)offset);
    DEBUG_MSG(str_msg);
  }

  char header[50];
  snprintf(header, sizeof(header), "\r\nSerialized packet - %u bytes\r\n",
           (unsigned int)offset);
  DEBUG_MSG(header);

  return offset;
}

void deserialize(uint8_t *buffer, Node *node, size_t buffer_len)
{
  if (!node || !buffer)
    return;

  size_t offset = 0;
  uint8_t rank = 0;
  uint8_t network_id = 0;

  // Восстанавливаем network_id
  memcpy(&network_id, buffer + offset, sizeof(network_id));
  offset += sizeof(network_id);

  // Восстанавливаем rank
  memcpy(&rank, buffer + offset, sizeof(rank));
  offset += sizeof(rank);

  if (network_id != node->network_id || rank >= node->rank)
  {
    char msg[100];
    snprintf(msg, sizeof(msg), "Received message from device: Network ID=%d, Rank=%d - packet ignored\r\n",
             network_id, rank);
    DEBUG_MSG(msg);
    return;
  }

  // Восстанавливаем строки
  for (uint8_t i = 0; i < (node->rank - 1); i++)
  {
    if (offset >= buffer_len)
      break;

    // Копируем строку с null-terminator
    size_t len = strlen((char *)(buffer + offset));

    if (len + 1 > node->buffer_size)
    {
      len = node->buffer_size - 1; // Обрезаем, если слишком длинная
    }

    memcpy(node->buffer[i], buffer + offset, len + 1);
    offset += len + 1;
  }
}

size_t get_node_size(Node *node)
{
  size_t size = sizeof(node->network_id) + sizeof(node->rank);
  for (uint8_t i = 0; i < node->rank; i++)
  {
    size += strlen(node->buffer[i]) + 1; // +1 для null-terminator
  }
  return size;
}

void debug_node_contents(Node *node)
{
  char msg[50];
  snprintf(msg, sizeof(msg), "\n=== Node Debug: ID=%d, Rank=%d ===\r\n",
           node->network_id, node->rank);
  DEBUG_MSG(msg);

  for (uint8_t i = 0; i < node->rank; i++)
  {
    size_t len = strlen(node->buffer[i]);
    snprintf(msg, sizeof(msg), "Buffer[%d] (len=%zu): ", i, len);
    DEBUG_MSG(msg);
    HAL_UART_Transmit(&huart2, (uint8_t *)node->buffer[i], len, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
  }
}

/*************************  Функции для АЦП и отправки данных */

float read_voltage_ads1115(void)
{
  uint16_t config[2] = {0};
  int16_t raw_value;
  float voltage;

  HAL_StatusTypeDef isReady = HAL_I2C_IsDeviceReady(&hi2c1, 0x90, 5, 100);
  if (isReady != HAL_OK)
  {
    DEBUG_MSG("ADS not ready! \r\n");
  }

  // Запрос данных от АЦП
  HAL_StatusTypeDef status =
      HAL_I2C_Master_Transmit(&hi2c1, 0x90, (uint8_t[]){0x00}, 1, 100);
  if (status != HAL_OK)
  {
    DEBUG_MSG("Error during sending conv reg to ADS! \r\n");
  }

  status = HAL_I2C_Master_Receive(&hi2c1, 0x90, (uint8_t *)&config, 2, 100);
  if (status != HAL_OK)
  {
    DEBUG_MSG("Error during receiving data from ADS! \r\n");
  }

  // Конвертация
  raw_value = ((config[0] << 8) | config[1]);
  voltage = (raw_value * 6.144) / 32768.0;

  // Выводим в консоль
  char msg[32];
  sprintf(msg, "Read voltage: %.3f \r\n", voltage);
  DEBUG_MSG(msg);

  return voltage;
}

void transmit_data(Node *node)
{
  /* Измеряем значение суммарного потенциала и передаем по LoRa */
  if (!node)
  {
    DEBUG_MSG("Error: Node is NULL\r\n");
    return;
  }

  ads1115_init();
  float voltage = read_voltage_ads1115();

  char err_code[3] = "E0";

  if (voltage < -3.5 || voltage > 3.5)
  {
    strcpy(err_code, "E1");
  }

  char packet[128];
  snprintf(packet, sizeof(packet), "N%dD%dV%.3f%s\r\n",
           node->network_id,
           node->rank,
           voltage,
           err_code);

  // Заполняем буфер
  if (push_to_packet(node, node->rank, packet) != 0)
  {
    DEBUG_MSG("Error: Failed to push packet to node\r\n");
    return;
  }

  // Проверяем содержимое узла
  debug_node_contents(node);

  // Вычисляем размер структуры
  packet_size = get_node_size(node);

  // Создаем и заполняем буфер u8 перед отправкой
  uint8_t serialized_packet[node->buffer_size];
  if (!serialized_packet)
  {
    DEBUG_MSG("Error: Failed to allocate serialization buffer\r\n");
    return;
  }

  serialize(node, &serialized_packet, packet_size);

  // Выводим сериализованный пакет целиком
  HAL_UART_Transmit(&huart2, serialized_packet, sizeof(serialized_packet), 200);
  HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);

  // Отправляем пакет
  uint8_t status = LoRa_transmit(&lora_module, serialized_packet, (uint8_t)packet_size, 200);
  if (status != 1)
  {
    DEBUG_MSG("LoRa can't send data! \r\n");
  }
  else
  {
    DEBUG_MSG("Packet was sent! \r\n");
  }

  // Освобождаем память
  free(serialized_packet);
}

void receive_transmit()
{
  /* Функция приема-передачи сообщений по запросу приемника */

  uint8_t *raw_packet = {0};

  /* Если ранг устройства не крайний в подсети, он ретранслятор */
  if (device_config.rank > 1)
  {

    /* Настраиваем таймер для ожидания сообщения от предыдущего устройства */
    TIM16_Init(device_config.wait_timeout * (device_config.rank - 1));
    HAL_TIM_Base_Start_IT(&htim16);

    /* Включаем режим приема сообщений у модуля */
    LoRa_startReceiving(&lora_module);

    /* Пока не получим сообщение предыдущего и не истечет таймаут */
    while (current_state == STATE_WAIT_DATA)
    {
      uint8_t packet_size = LoRa_receive(&lora_module, raw_packet, RX_BUFFER_MAX_LENGTH);
      HAL_Delay(50);
      if (packet_size > 0)
      {
        deserialize(raw_packet, this_transmitter, RX_BUFFER_MAX_LENGTH);
        current_state = STATE_TRANSMIT;

        debug_node_contents(this_transmitter);
        break;
      }
      LoRa_startReceiving(&lora_module);
    }

    /* Таймаут истек, передаем только свое сообщение */
    if (current_state == STATE_RECEIVE_TIMEOUT)
    {
      current_state = STATE_TRANSMIT;
      DEBUG_MSG("Timeout for receiving previous packets occurred!\r\n");
    }

    /* Теперь нужно остановить таймер */
    HAL_TIM_Base_Stop_IT(&htim16);
  }
  /* Если ранг = 1, это крайний в подсети передатчик
   * Сразу переводим в режим передачи пакета */
  else
  {
    current_state = STATE_TRANSMIT;
  }

  /* Отправляем показания текущего устройства, записав их в буфер*/
  if (current_state == STATE_TRANSMIT)
  {
    transmit_data(this_transmitter);
  }
}

void ads1115_init()
{
  uint16_t config = (0x01 << 15) | // OS: выполняем конвертацию
                    (0x00 << 12) | // MUX: A0-A1 (дифф режим)
                    (0x00 << 9) |  // PGA: ±6.144V
                    (0x01 << 8) |  // MODE: разовое чтение single-shot
                    (0x04 << 5) |  // DR: data rate (128 SPS)
                    (0x03 << 0);   // COMP_QUE: компаратор встроенный

  uint8_t config_data[3] = {0x01, config >> 8, config & 0xFF};
  HAL_StatusTypeDef isAdsReady = HAL_I2C_IsDeviceReady(&hi2c1, 0x90, 5, 1000);
  if (isAdsReady != HAL_OK)
  {
    DEBUG_MSG("Error during ADS initialization! \r\n");
  }

  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, 0x90, config_data, 3, 1000);
  if (status != HAL_OK)
  {
    DEBUG_MSG("Error during ADS configuration! \r\n");
  }
}

void sync_time_with_receiver(uint16_t sync_wait_timeout)
{
  TIM16_Init(sync_wait_timeout);
  HAL_TIM_Base_Start_IT(&htim16);

  LoRa_startReceiving(&lora_module);
  while (current_state == STATE_WAIT_DATA)
  {
    uint8_t rx_buffer[TIME_BUFFER_LENGTH];
    int packet_size = LoRa_receive(&lora_module, &rx_buffer, RX_BUFFER_MAX_LENGTH);
    if (packet_size == TIME_BUFFER_LENGTH)
    {
      DEBUG_MSG("Received synchro time: \r\n");
      HAL_UART_Transmit(&huart2, rx_buffer, sizeof(rx_buffer), 100);

      LoRa_transmit(&lora_module, rx_buffer, packet_size, 100);

      Alarm_Init(rx_buffer[0], rx_buffer[1]);
      current_state = STATE_MEASUREMENT;
      return;
    }
  }
  if (current_state == STATE_RECEIVE_TIMEOUT)
  {
    DEBUG_MSG("Timeout waiting for synchro time! \r\n");
    current_state = STATE_MEASUREMENT;
    Alarm_Init(device_config.default_alarm_hours, device_config.default_alarm_minutes);
  }
  HAL_TIM_Base_Stop_IT(&htim16);
}

void RTC_Init(uint16_t timeout_ms)
{
  HAL_NVIC_SetPriority(RTC_TAMP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_TAMP_IRQn);
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, timeout_ms, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    DEBUG_MSG("RTC can't set wakeup timer! \r\n");
  }
}

void RTC_Stop()
{
  if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
  {
    DEBUG_MSG("RTC can't deactivate wakeup timer! \r\n");
  }
}

void TIM16_Init(uint16_t timeout_ms)
{
  __HAL_TIM_CLEAR_IT(&htim16, TIM_IT_UPDATE);

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 16000 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = timeout_ms - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  __HAL_RCC_TIM16_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM16_IRQn);
  HAL_TIM_Base_Init(&htim16);
}

void Alarm_Init(uint8_t hours, uint8_t minutes)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  // Устанавливаем текущее время в 00:00:00
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  // Задаем будильник
  sAlarm.AlarmTime.Hours = hours;
  sAlarm.AlarmTime.Minutes = minutes;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  
  // Отключаем предыдущий alarm перед установкой нового
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
  HAL_Delay(10);
  
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_GPIO_DeInit()
{
  /* Переводим все ноги в аналоговый режим */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_13 | GPIO_PIN_14);
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void enter_to_sleep()
{
  DEBUG_MSG("Sleep\r\n");

  HAL_I2C_DeInit(&hi2c1);
  HAL_SPI_DeInit(&hspi1);
  HAL_UART_DeInit(&huart2);
  MX_GPIO_DeInit();

  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_I2C1_CLK_DISABLE();

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

  HAL_SuspendTick();

  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  SystemClock_Config();
  HAL_ResumeTick();

  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  check_spi_config();
  MX_I2C1_Init();

  reset_lora();
  uint16_t lora_status = LoRa_init(&lora_module);
  if (lora_status != 200)
  {
    DEBUG_MSG("LoRa init failed\r\n");
  }
}

void reset_lora()
{
  HAL_GPIO_WritePin(lora_module.reset_port, lora_module.reset_pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(lora_module.reset_port, lora_module.reset_pin, GPIO_PIN_SET);
  HAL_Delay(100);
}

void check_spi_config()
{
  // Переинициализация SPI
  if (hspi1.Instance->CR1 != SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE |
      SPI_CR1_BR_0 | SPI_CR1_MSTR)
  {
    MX_SPI1_Init();
  }
}

/* Обработчики прерываний */

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  if (current_state == STATE_WAIT_CONFIGURATION)
  {
    // DEBUG
    DEBUG_MSG("RTC wake up triggered: exiting configuration wait state \r\n");

    // Проверям сообщение от приемника
    uint8_t rx_buffer[RX_BUFFER_MAX_LENGTH];
    int packet_size = LoRa_receive(&lora_module, &rx_buffer, RX_BUFFER_MAX_LENGTH);
    if (packet_size == TIME_BUFFER_LENGTH)
    {
      // Debug message
      DEBUG_MSG("Received synchro time during configuration: \r\n");
      HAL_UART_Transmit(&huart2, rx_buffer, sizeof(rx_buffer), 100);

      Alarm_Init(rx_buffer[0], rx_buffer[1]);
      current_state = STATE_MEASUREMENT;
    }
    else
    {
      DEBUG_MSG("No synchro time received during configuration, keep sleep and waiting \r\n");
      current_state = STATE_WAIT_CONFIGURATION;
    }
  }
  else
  {
    current_state = STATE_MEASUREMENT;
  }
}

/* Обработчики прерываний по таймеру (ожидание приема/отправки) */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM16)
  {
    current_state = STATE_RECEIVE_TIMEOUT;
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  DEBUG_MSG("Wake up from Alarm: start retranslation\r\n");
  current_state = STATE_WAIT_DATA;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  lora_module = newLoRa();
  lora_module.CS_port = NSS_GPIO_Port;
  lora_module.CS_pin = NSS_Pin;
  lora_module.reset_port = RST_GPIO_Port;
  lora_module.reset_pin = RST_Pin;
  lora_module.DIO0_port = DIO0_GPIO_Port;
  lora_module.DIO0_pin = DIO0_Pin;
  lora_module.hSPIx = &hspi1;

  uint16_t lora_status = LoRa_init(&lora_module);
  if (lora_status != 200)
  {
    DEBUG_MSG("LoRa init failed \n\r");
  }

  device_config.rank = 1;
  device_config.network_id = 1;
  device_config.buffer_size = 20;
  device_config.wait_timeout = 5000;       // 5 sec
  device_config.sync_wait_timeout = 30000; // 30 sec
  device_config.default_alarm_hours = 0;
  device_config.default_alarm_minutes = 1;
  device_config.wakeup_timeout = 20428; // 10 sec

  this_transmitter = configure_node(device_config.rank, device_config.network_id, device_config.buffer_size);

  current_state = STATE_WAIT_CONFIGURATION;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch (current_state)
    {
    case STATE_MEASUREMENT:
      current_state = STATE_SLEEP;
      break;
    case STATE_RECEIVE_TIMEOUT:
      // Обрабатывается в функции receive_transmit()
      break;
    case STATE_SLEEP:
      RTC_Init(device_config.wakeup_timeout);
      enter_to_sleep();
      RTC_Stop();
      break;
    case STATE_TRANSMIT:
      transmit_data(this_transmitter);
      current_state = STATE_SLEEP;
      break;
    case STATE_WAIT_DATA:
      DEBUG_MSG("Retranslation\r\n");
      HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
      HAL_Delay(20);
      RTC_Stop();
      receive_transmit();
      current_state = STATE_WAIT_SYNC;
      break;
    case STATE_WAIT_SYNC:
    	// Механизм переписать!!!
      sync_time_with_receiver(device_config.sync_wait_timeout);
      current_state = STATE_MEASUREMENT;
      break;
    case STATE_WAIT_CONFIGURATION:
      LoRa_startReceiving(&lora_module);
      DEBUG_MSG("Wait config\r\n");

      RTC_Init(device_config.wakeup_timeout);
      enter_to_sleep();
      RTC_Stop();
      // После пробуждения current_state уже установлено в callback
      break;
    default:
      current_state = STATE_SLEEP;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 1;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20428, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0); // Приоритет 1 (ниже RTC)
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_Pin|NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin NSS_Pin */
  GPIO_InitStruct.Pin = RST_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  char *err_msg = "Undefined error with peripherals! \r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)err_msg, strlen(err_msg), 100);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
