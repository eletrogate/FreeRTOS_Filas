/**
 * @file main.cpp
 * @author Evandro Teixeira
 * @brief 
 * @version 0.1
 * @date 14-01-2022
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define COLOR_BLACK         "\e[0;30m"
#define COLOR_RED           "\e[0;31m"
#define COLOR_GREEN         "\e[0;32m"
#define COLOR_YELLOW        "\e[0;33m"
#define COLOR_BLUE          "\e[0;34m"
#define COLOR_PURPLE        "\e[0;35m"
#define COLOR_CYAN          "\e[0;36m"
#define COLOR_WRITE         "\e[0;37m"
#define COLOR_RESET         "\e[0m"
#define MESSAGE_SIZE        32    // Tamanho da mensagem
#define NUMBER_OF_ELEMENTS  8     // Número de elementos na fila
#define BUTTON              15    // Pino do botão 
#define LED_BOARD           2     // Pino do LED
#define DEBOUNCE_BUTTON     1000  // Tempo do debounce do botão

// Estruta dados mensagem 
typedef struct
{
  uint8_t Sts;
  char  Txt[MESSAGE_SIZE];
}DataMsg_t;

// Prototipo das tarefas 
void Tarefa_A(void *parameters);
void Tarefa_B(void *parameters);
void Tarefa_LED(void *parameters);

// Cria Handle da Mensagem de tarefa de A para tarefa B
QueueHandle_t Msg_A_para_B;
// Cria Handle da Mensagem da interrupção do botão para a tarefa LED
QueueHandle_t Button_para_LED;

/**
 * @brief Função da interrupção botão
 */
void IRAM_ATTR Button_ISR()
{
  // tempo da ultima leitura do botão
  static uint32_t last_time = 0; 
  static DataMsg_t Msg = {
    .Sts = 0,
    .Txt = {0}
  }; 

  // Algoritmo de debounce do botão
  if( (millis() - last_time) >= DEBOUNCE_BUTTON)
  {
    last_time = millis();
    if(Msg.Sts == 1)
    {
      // Prepara dado para ser publicado na fila
      Msg.Sts = 0;
      sprintf(Msg.Txt,"LED BOARD: OFF");
    }
    else 
    {
      // Prepara dado para ser publicado na fila
      Msg.Sts = 1;
      sprintf(Msg.Txt,"LED BOARD: ON");
    }

    // Publica dado na fila
    xQueueSendFromISR(Button_para_LED,&Msg,pdFALSE);
  }
}

/**
 * @brief 
 */
void setup() 
{
  // Inicializa a Serial 
  Serial.begin(115200);
  Serial.printf("\n\rFreeRTOS - Fila\n\r");

  // Inicializa pino 15 como entra e inicializa interrupção do botão
  pinMode(BUTTON, INPUT);
  attachInterrupt(BUTTON, Button_ISR, RISING);

  // Inicializa pino do LED on Board
  pinMode(LED_BOARD,OUTPUT);
  digitalWrite(LED_BOARD,LOW);

  // Cria Fila de mensagem para comunicação entre as Tarefas A e B
  Msg_A_para_B = xQueueCreate( NUMBER_OF_ELEMENTS, (MESSAGE_SIZE * sizeof(char)) );
  if(Msg_A_para_B == NULL)
  {
    Serial.printf("\n\rFalha em criar a fila Msg_A_para_B");
  }
  // Cria Fila de mensagem para função da inturrupção do botão para a Tarefa LED
  Button_para_LED = xQueueCreate( NUMBER_OF_ELEMENTS, sizeof(DataMsg_t) );
  if(Button_para_LED == NULL)
  {
    Serial.printf("\n\rFalha em criar a fila Button_para_LED");
  }

  // Cria as tarefas da aplicação
  xTaskCreate(Tarefa_A, "Tarefa_A", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(Tarefa_B, "Tarefa_B", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(Tarefa_LED, "Tarefa_LED", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
  
}
/**
 * @brief 
 */
void loop() 
{
  Serial.printf("\n\rDeleta tarefa LOOP");
  vTaskSuspend(NULL);
}

/**
 * @brief Tarefa A
 * 
 * @param parameters 
 */
void Tarefa_A(void *parameters)
{
  // Variavel locais
  char txt[MESSAGE_SIZE] = {0};
  uint8_t msg_counter = 0;

  // Imprime informação da tarefa no barramento serial
  Serial.print(COLOR_GREEN);
  Serial.printf("\n\r%s", pcTaskGetTaskName(NULL) );
  Serial.print(COLOR_RESET); 

  while (1)
  {  
    // Prepara dado para ser publicado na fila
    sprintf(txt,"Msg number: %3d, task A to B",msg_counter++);

    // Imprime o conteudo a ser publicado no barramento serial
    Serial.print(COLOR_GREEN);
    Serial.printf("\n\r%s Envia  -> %s", pcTaskGetTaskName(NULL), txt);
    Serial.print(COLOR_RESET); 

    // Publica dado na fila
    if( xQueueSend( Msg_A_para_B, (void*)&txt, (TickType_t)1000 ) != pdTRUE)
    {
      Serial.print(COLOR_GREEN);
      Serial.printf("\n\rFalha em enviar os dados da fila");
      Serial.print(COLOR_RESET); 
    }
    
    // Pausa a Tarefa A por 05 segundos 
    vTaskDelay(5000/portTICK_PERIOD_MS);
  }
}

/**
 * @brief Tarefa B
 * 
 * @param parameters 
 */
void Tarefa_B(void *parameters)
{
  // Variavel locais
  char txt[MESSAGE_SIZE] = {0};

  // Imprime informação da tarefa 
  Serial.print(COLOR_YELLOW);
  Serial.printf("\n\r%s", pcTaskGetTaskName(NULL) );
  Serial.print(COLOR_RESET); 

  while (1)
  {
    // Checa se há dados na fila - e pausa a tarefa por 01 segundo
    if( xQueueReceive(Msg_A_para_B,&txt,(TickType_t)1000 ) == pdPASS)
    {
      // Imprime conteudo da fila no barramento serial
      Serial.print(COLOR_YELLOW);
      Serial.printf("\n\r%s Recebe -> %s", pcTaskGetTaskName(NULL),txt );
      Serial.print(COLOR_RESET); 
    }
    
    // Imprime informação da tarefa no barramento serial
    Serial.print(COLOR_YELLOW);
    Serial.printf("\n\r%s - time: %d s", pcTaskGetTaskName(NULL),(uint)(millis()/1000) );
    Serial.print(COLOR_RESET); 
  }
}

/**
 * @brief Tarefa LED
 * 
 * @param parameters 
 */
void Tarefa_LED(void *parameters)
{
  DataMsg_t Msg;

  // Imprime informação da tarefa 
  Serial.print(COLOR_RED);
  Serial.printf("\n\r%s", pcTaskGetTaskName(NULL) );
  Serial.print(COLOR_RESET); 

  while (1)
  {
    // Checa se há dados na fila - e suspende a tarefa enquato a fila estiver vazia
    if( xQueueReceive(Button_para_LED, &Msg, portMAX_DELAY) == pdPASS)
    {
      // Imprime conteudo da fila no barramento serial
      Serial.print(COLOR_RED);
      Serial.printf("\n\r%s Recebe -> %s | Sts: %d", pcTaskGetTaskName(NULL),Msg.Txt,Msg.Sts);
      Serial.print(COLOR_RESET); 

      // Altera o valor do pino do LED on Board
      digitalWrite(LED_BOARD,Msg.Sts);
    }

    // Imprime informação da tarefa no barramento serial
    Serial.print(COLOR_RED);
    Serial.printf("\n\r%s - time: %d s", pcTaskGetTaskName(NULL),(uint)(millis()/1000) );
    Serial.print(COLOR_RESET); 
  }
}