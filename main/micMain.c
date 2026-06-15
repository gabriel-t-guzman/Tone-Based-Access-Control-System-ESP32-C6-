/* ********************************************************************************************************************************* 
 * Microphone test - ADC in continuous mode and time-domain BP filtering 
 * Paulo Pedreiras, Pedro Fonseca, Luis Moutinho 2026/Apr.
 * 
 * Tested:
 *  ESP32-C6 DevKitC-1
 * 
 * - Basic use of the ADC to get and process sound samples.
 * - Uses continuous mode ADC operation, FEITO POR GRABIEL to allow higher frequencies#
 * - Signal is processed by a Band-Pass filter, in the time-domain, to identify defined frequencies 
 *  
 * Microphone is a MEMS Adafruit Silicon MEMS Microphone Breakout - SPW2430.
 *     Supplied with 3.3-5V, output at DC pin has a 0.7 V and a 100 mVpp "when talking near". 
 *      In my case I had around 1 V. So the attenuation cannot be 0 dB. 
 *      I have used 2.5 dB (vref/0.7), to get 1.3 to 1.5 volts for Vref+ and avoid saturation
 *      Check other mics to see if this is normal.  
 * 
 *  
 * Bibliography: 
 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/index.html
 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/adc_continuous.html 
 *      https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-library.html      
 * 
 * Based on the sample code  provided by EspressIF:
 *      https://github.com/espressif/esp-idf/tree/47faecc3/examples/peripherals/adc/continuous_read 
 * 
 * NOTE: must run idf.py add-dependency "espressif/esp-dsp" when creating a new project using dsp functionality
 ***********************************************************************************************************************************/ 

/* ********************************* 
 * Includes
 ***********************************/
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"      // FreeRTOS includes
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h" // For ESP ADC
#include "esp_dsp.h"                // For ESP DSP functions, conv in the case
#include "esp_private/esp_clk.h"    // For ESP clock functions
#include "driver/gpio.h"


/* ********************************
 * Global defines 
 **********************************/
#define MICEX_ADC_UNIT                    ADC_UNIT_1
#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5            // Use Vref/0.75, 1.3 ... 1.5 V
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH   // 12 bits resolution (maximum)

#define MICEX_ADC_FRAME_SIZE             512                           /* ADC frame size, in bytes */
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE)    /* Internal buffer, should an integer multiple of the frame size to avoid incomplete frames */
#define MICEX_ADC_SAMPLE_FREQ            (20 * 1000)                   /* Sample frequency, in Hz. Notice that there are lower and higher bounds*/

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048 /* IMPORTANT: If FFT is to be used, must be must be a power of two */
                                              /* For time-domain conv. filters there is no such restriction */
                                              
#define MAX_FILT_IR_LEN                 200     /* Maximum IR filter length */
#define LED_GPIO GPIO_NUM_11

/* Global variable declarations */
static adc_channel_t channel[1] = {ADC_CHANNEL_3};  // Mic on ADC channel 3
static TaskHandle_t s_task_handle;

static const char *TAG = "MIC_EXAMPLE";

/* ADC - Variables to hold data acquisition and parsing */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0}; // Buffer where the results of a continuous read are placed   
__attribute__((aligned(16))) adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES]; // Buffer where frame parsed data is placed 

/* FreeRTOS tasks and IPC */
#define PROCESSOR_TASK_STACK_SIZE       8192            // Accomodate calls to dsp functions, log, user vars, ...
#define PROCESSOR_TASK_PRIORITY	( tskIDLE_PRIORITY + 4 )
QueueHandle_t XQ;    /* Queue handle */

/* Impulse reponse filter and related variables */
__attribute__((aligned(16))) float hbpf_2000[]={0.000000159, 0.000000627, 0.000001342, 0.000001992, 0.000002021, 0.000000811, -0.000001951, -0.000005803, -0.000009211, -0.000009771, -0.000005126, 0.000005530, 0.000019968, 0.000032393, 0.000034728, 0.000019953, -0.000013366, -0.000057344, -0.000094328, -0.000101598, -0.000061027, 0.000029216, 0.000146267, 0.000243335, 0.000263148, 0.000162446, -0.000060416, -0.000347390, -0.000584891, -0.000636656, -0.000400138, 0.000131931, 0.001417364, 0.002300985, 0.002467127, 0.001590301, -0.000342366, -0.002850801, -0.004964817, -0.005514172, -0.003664154, 0.000519957, 0.005794255, 0.009994873, 0.010855580, 0.007102806, -0.000673877, -0.009874717, -0.016695600, -0.017674894, -0.011358970,0.000698320, 0.014181650, 0.023565343, 0.024413948, 0.015457390, -0.000532528, -0.017575347, -0.028794290, -0.029285509, -0.018308976, 0.000199747, 0.019096140, 0.030905128, 0.030905128, 0.019096140, 0.000199747, -0.018308976, -0.029285509, -0.028794290, -0.017575347, -0.000532528, 0.015457390, 0.024413948, 0.023565343, 0.014181650, 0.000698320, -0.011358970, -0.017674894, -0.016695600, -0.009874717, -0.000673877, 0.007102806, 0.010855580, 0.009994873, 0.005794255, 0.000519957, -0.003664154, -0.005514172, -0.004964817, -0.002850801, -0.000342366, 0.001590301, 0.002467127, 0.002300985, 0.001417364, 0.000131931, -0.000400138, -0.000636656, -0.000584891, -0.000347390, -0.000060416, 0.000162446, 0.000263148, 0.000243335, 0.000146267, 0.000029216, -0.000061027, -0.000101598, -0.000094328, -0.000057344, -0.000013366, 0.000019953, 0.000034728, 0.000032393, 0.000019968, 0.000005530, -0.000005126, -0.000009771, -0.000009211, -0.000005803, -0.000001951, 0.000000811, 0.000002021, 0.000001992, 0.000001342, 0.000000627, 0.000000159};
__attribute__((aligned(16))) float hbpf_2720[]={0.000000011, -0.000000300, -0.000000337, 0.000000417, 0.000001761, 0.000002475, 0.000000949, -0.000003197, -0.000007591, -0.000007571, 0.000000361, 0.000013852, 0.000023072, 0.000015941, -0.000011010, -0.000043917, -0.000055054, -0.000021794, 0.000049136, 0.000112629, 0.000107557, 0.000004862, -0.000150829, -0.000246951, -0.000172189, 0.000084845, 0.000381876, 0.000479938, 0.000207117, -0.000368266, -0.000898043, -0.000932597, -0.000275679, 0.001528519, 0.002653181, 0.002004354, -0.000550969, -0.003640798, -0.004888088, -0.002544430, 0.002736750, 0.007572032, 0.007818908, 0.001872767, -0.007212707, -0.012981865, -0.010038631, 0.001286196, 0.013937580, 0.018415082, 0.009845251, -0.007395249, -0.021627314, -0.021829109, -0.006153866, 0.015589616, 0.028100448, 0.021519961, -0.000795603, -0.023815971, -0.031164443, -0.016998127, 0.009339093, 0.029613040, 0.029613040, 0.009339093, -0.016998127, -0.031164443, -0.023815971, -0.000795603, 0.021519961, 0.028100448, 0.015589616, -0.006153866, -0.021829109, -0.021627314, -0.007395249, 0.009845251, 0.018415082, 0.013937580, 0.001286196, -0.010038631, -0.012981865, -0.007212707, 0.001872767, 0.007818908, 0.007572032, 0.002736750, -0.002544430, -0.004888088, -0.003640798, -0.000550969, 0.002004354, 0.002653181, 0.001528519, -0.000275679, -0.000932597, -0.000898043, -0.000368266, 0.000207117, 0.000479938, 0.000381876, 0.000084845, -0.000172189, -0.000246951, -0.000150829, 0.000004862, 0.000107557, 0.000112629, 0.000049136, -0.000021794, -0.000055054, -0.000043917, -0.000011010, 0.000015941, 0.000023072, 0.000013852, 0.000000361, -0.000007571, -0.000007591, -0.000003197, 0.000000949, 0.000002475, 0.000001761, 0.000000417, -0.000000337, -0.000000300, 0.000000011};
__attribute__((aligned(16))) float hbpf_3440[]={0.000000491, 0.000000094, -0.000001118, -0.000001511, 0.000000428, 0.000003471, 0.000003483, -0.000002080, -0.000008782, -0.000007034, 0.000006553, 0.000019664, 0.000012611, -0.000016938, -0.000040032, -0.000020203, 0.000038568, 0.000075611, 0.000028759, -0.000080329, -0.000134581, -0.000035082, 0.000156797, 0.000228397, 0.000031596, -0.000292150, -0.000372749, -0.000000261, 0.000530080, 0.000584197, -0.000137995, -0.001079888, -0.001796318, 0.000234705, 0.002507987, 0.002410103, -0.000764652, -0.004178289, -0.003593029, 0.001745673, 0.006792180, 0.005087701, -0.003434380, -0.010294310, -0.006593436, 0.005970834, 0.014438656, 0.007760218, -0.009347290, -0.018819161, -0.008262927, 0.013383570, 0.022931641, 0.007875000, -0.017736350, -0.026258208, -0.006523384, 0.021944336, 0.028358183, 0.004312012, -0.025502019, -0.028947214, -0.001507949, 0.027947791, 0.027947791, -0.001507949, -0.028947214, -0.025502019, 0.004312012, 0.028358183, 0.021944336, -0.006523384, -0.026258208, -0.017736350, 0.007875000, 0.022931641, 0.013383570, -0.008262927, -0.018819161, -0.009347290, 0.007760218, 0.014438656, 0.005970834, -0.006593436, -0.010294310,-0.003434380, 0.005087701, 0.006792180, 0.001745673, -0.003593029, -0.004178289, -0.000764652, 0.002410103, 0.002507987, 0.000234705, -0.001796318, -0.001079888, -0.000137995, 0.000584197, 0.000530080, -0.000000261, -0.000372749, -0.000292150, 0.000031596, 0.000228397, 0.000156797, -0.000035082, -0.000134581, -0.000080329, 0.000028759, 0.000075611, 0.000038568, -0.000020203, -0.000040032, -0.000016938, 0.000012611, 0.000019664, 0.000006553, -0.000007034, -0.000008782, -0.000002080, 0.000003483, 0.000003471, 0.000000428, -0.000001511, -0.000001118, 0.000000094, 0.000000491};


/* *************************************************************** 
 * Function prototypes 
 *****************************************************************/
/* Inits the ADC for continuous mode (channels, attenuation, frequency, handles, ...)*/
 static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
 /* Callback of ADC driver. Executed whenever a new frame is available */
static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
/* Task called to process one full buffer of data. A queue + blocking read is used for synchronization and data passing */
static void pv_processor_task(void *pvParam);

/******************************************************************* 
 * The main task 
 *******************************************************************/
void app_main(void)
{
    /* Variable declarations */
    esp_err_t ret;          // Generic return code variable
    esp_err_t parse_ret;    // return code of ADC frame parse function 
    uint32_t ret_num = 0;   // Length of bytes return by a read operation
    uint32_t sb_count = 0;   // For counting the number of acquired samples    
    uint32_t num_parsed_samples = 0;    // To count the number of parsed samples
    
    adc_continuous_evt_cbs_t cbs;   // Variable for setting callback type (internal poll full, or frame conversion completed)    
    adc_continuous_handle_t handle = NULL;  //Handle for ADC          

    float * sound_samp_buf_ADC;   // Buffer to hold sound samples. Sound buffers are float because conv() function requires float parameters - avoid conversions 
    
    /* Variable inits */
    memset(result, 0x00, MICEX_ADC_FRAME_SIZE); // Init frame buffer     
    sound_samp_buf_ADC = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);     

    s_task_handle = xTaskGetCurrentTaskHandle();    // Get handle of the current task

    cbs.on_conv_done = s_conv_done_cb;  // Callback called when one conversion frame is done     
    cbs.on_pool_ovf = NULL;          // Don't set callback for internbal buffer overflow         

    /* Set log level */
    /* Debug allow to see variable values */
    /* Info only shows the decision */
    /* Verbose shows a trace of calls an some additional vars*/
    esp_log_level_set(TAG,ESP_LOG_DEBUG);

    /* Processor task and Queue inits */
    XQ=xQueueCreate(1, sizeof(float)*MICEX_SOUND_SAMPLES_BUF_SIZE); // Create queue to store one full sample period of sound
    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL );

    /* Init ADC */
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle); // Call init function
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));   // Regiter callbacks
    ESP_ERROR_CHECK(adc_continuous_start(handle));                                  // Start the ADC

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0); // Começa fechada (LED desligado)

    /* Infinite loop - wait for data and process it */
    /* Synchronization with ADC is obtained via the ulTaskNotifyTake(pdTRUE, portMAX_DELAY); call */
    /*     that assures that processing does not proceed until a notification that a frame was acquired*/
    while (1) {        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a new frame

        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGV(TAG, "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);                
                /* One frame received. Extract samples from frame and put them on sound sample buffer*/
                parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
                if (parse_ret == ESP_OK) {
                    
                    for (int i = 0; i < num_parsed_samples; i++) {
                        sound_samp_buf_ADC[sb_count] = (float) parsed_data[i].raw_data;                           
                        sb_count+=1;
                        if(sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) { // The sound buffer is full. Process it ... */
                            ESP_LOGD(TAG, "sound buffer acquired. Time to process ...\n");                
                            xQueueSend(XQ,(void *)sound_samp_buf_ADC,0);     // Places the sound buffer in the queue. If the queue is full skip it (ticksTo Wait set to 0)
                                                                        // The consumer/processing task is automatically waked if blocked in the Queue
                            sb_count = 0;
                        }
                    }

                } else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }
                /*                  
                 * To avoid a task watchdog timeout, add a delay here. 
                 */
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}


/* **********************************************************************************************
 * Task activated when there is a full buffer of sound samples data available
 * The task reads a queue in blocking mode. This wait it awakes whenever the ADC processing code
 *      (the app_main taks in the case) delivers a new full buffer of data. 
 * Note that the use of a Queue and two separate buffers (ADC and processing) decouples the 
 *      acquisition from processing. I.e., processing can take as much time as needed without race conditions
 *      or any other sort of interference. The cost is overhead ...
 ************************************************************************************************/

void pv_processor_task(void *pvParam)
{           
    float * sound_samp_buf_proc;       
    sound_samp_buf_proc = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);         
    
    /* 2. AS TUAS VARIÁVEIS (Filtros e Máquina de Estados) */
    // O tamanho do array de saída da convolução é N + M - 1
    // N = 2048 (buffer de som), M = 127 (tamanho do filtro gerado no Octave)
    int out_len = MICEX_SOUND_SAMPLES_BUF_SIZE + 127 - 1;
    
    float * out_2000 = heap_caps_malloc(sizeof(float) * out_len, MALLOC_CAP_DMA);
    float * out_2720 = heap_caps_malloc(sizeof(float) * out_len, MALLOC_CAP_DMA);
    float * out_3440 = heap_caps_malloc(sizeof(float) * out_len, MALLOC_CAP_DMA);
    
    int sequencia_atual[4] = {-1, -1, -1, -1};
    int seq_idx = 0;
    int ultimo_simbolo = -1; // Guarda o estado anterior para evitar leituras repetidas
    
    // Definir password
    int seq_abrir[4] = {0, 1, 2, 0}; 
    int seq_fechar[4] = {2, 0, 1, 2};

    /* Infinite processing loop */
    for(;;) {
        /* Waits for new data */
        xQueueReceive(XQ,(void *)sound_samp_buf_proc,portMAX_DELAY); 

        /* 3. IMPRESSÃO DE DEBUG (CÓDIGO ORIGINAL) */        
        /*printf("\nFirst 100 samples of the sound frame:----------- ");
        for(n=0; n < 100; n++) {            
            if(n%10 == 0) {
                printf("\n[%d to %d]:", n,n+9);
            }
            printf("%5d ", (int)sound_samp_buf_proc[n]);
        }
        printf("\n---------------------\n"); */

        /* 4. O TEU CÓDIGO (Filtragem e Lógica da Porta) */
        
        /* 4. O TEU CÓDIGO (Filtragem e Lógica da Porta) */
        
        // 4.0. REMOVER O OFFSET DC (O Segredo para a convolução funcionar)
        float media = 0;
        for(int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
            media += sound_samp_buf_proc[i];
        }
        media /= MICEX_SOUND_SAMPLES_BUF_SIZE;
        
        for(int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
            sound_samp_buf_proc[i] -= media; // Puxa o sinal (ex: 3276) para o 0
        }

        // 4.1. Aplicar os Filtros FIR
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, hbpf_2000, 127, out_2000);
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, hbpf_2720, 127, out_2720);
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, hbpf_3440, 127, out_3440);
        
        // 4.2. Calcular Energias (Incluindo a Energia Total do Som)
        float energy_2000 = 0, energy_2720 = 0, energy_3440 = 0;
        float energy_total = 0;

        // Energia de TODO o som captado (depois de tirar o DC)
        for(int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
            energy_total += sound_samp_buf_proc[i] * sound_samp_buf_proc[i];
        }

        // Energia filtrada (ignorando as pontas ruidosas da convolução)
        for(int i = 127; i < out_len - 127; i++) {
            energy_2000 += out_2000[i] * out_2000[i];
            energy_2720 += out_2720[i] * out_2720[i];
            energy_3440 += out_3440[i] * out_3440[i];
        }
        
        // IMPRESSÃO DE DEBUG - VITAL PARA AFINAR!
        printf("Energias -> TOT: %.0f | 0: %.0f | 1: %.0f | 2: %.0f\n", energy_total, energy_2000, energy_2720, energy_3440);

        // 4.3. Deteção Dinâmica por Rácio de Energia

        // 1. Define o teu novo limiar de energia
        // Como o teu pico é ~20000 e o ruído é ~50, 5000 é um bom limite de segurança.
       float THRESHOLD = 18000.0f; 
       
        // 2. Variável para guardar a entrada atual (-1 significa silêncio/ruído)
        int entrada_actual = -1; 

        // 3. Lógica "Winner-Takes-All"
        // Verifica se o 2000 Hz passou o limiar E se é mais forte que os outros dois
        if (energy_2000 > THRESHOLD && energy_2000 > energy_2720 && energy_2000 > energy_3440) {
            entrada_actual = 0;
        }
        // Verifica se o 2720 Hz passou o limiar E se é mais forte que os outros dois
        else if (energy_2720 > THRESHOLD && energy_2720 > energy_2000 && energy_2720 > energy_3440) {
            entrada_actual = 1;
        } 
        // Verifica se o 3440 Hz passou o limiar E se é mais forte que os outros dois
        else if (energy_3440 > THRESHOLD && energy_3440 > energy_2000 && energy_3440 > energy_2720) {
            entrada_actual = 2;
        }
          /*  gpio_set_level(LED_GPIO, 0);

         4. (Opcional) Imprimir o resultado para testares no terminal
        if (entrada_actual != -1) {
            printf("Detetado o simbolo: %d\n", entrada_actual);
            gpio_set_level(LED_GPIO, 1);

        }
        */
        // 5 MAQUINA DE ESTADOS // ---------------------------------------------------------
// LÓGICA DE VALIDACIÓN Y MÁQUINA DE ESTADOS
// ---------------------------------------------------------
        // Variables para la máquina de estados y contadores
// 5 MÁQUINA DE ESTADOS // ---------------------------------------------------------
/// Adicionamos a variável ruta_actual
static int estado_maquina = 0;
static int ruta_actual = 0; // 0 = Indefinido, 1 = Abrir, 2 = Fechar
static int contador_simbolo = 0;
static int contador_silencio = 0;
static int simbolo_previo = -1;
       int state = gpio_get_level(LED_GPIO);
if (entrada_actual != -1) {
    contador_silencio = 0; 

    if (entrada_actual == simbolo_previo) {
        contador_simbolo++;
    } else {
        contador_simbolo = 1;
        simbolo_previo = entrada_actual;
    }

    if (contador_simbolo == 2) { 
        printf("Símbolo validado de forma estável!: %d\n", entrada_actual);

        // --- INÍCIO DA MÁQUINA DE ESTADOS ---
        
        // CASO A: Estamos no Estado 0 (Aguardando o primeiro dígito)
        if (estado_maquina == 0) {
            if (entrada_actual == seq_abrir[0]) {
                ruta_actual = 1; // Bloqueamos a máquina na rota de ABRIR
                estado_maquina = 1;
                printf("Iniciando sequência para ABRIR...\n");
            } 
            else if (entrada_actual == seq_fechar[0]) {
                ruta_actual = 2; // Bloqueamos a máquina na rota de FECHAR
                estado_maquina = 1;
                printf("Iniciando sequência para FECHAR...\n");
            } 
            else {
                printf("Símbolo incorreto para começar. Continuamos em 0.\n");
                for(int i = 0; i < 5; i++) {
                    gpio_set_level(LED_GPIO, 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(LED_GPIO, 0);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                gpio_set_level(LED_GPIO, state);
                
                
                estado_maquina = 0; // Mantém-se em 0
            }
        }
        
        // CASO B: Já estamos avançando em uma das duas rotas (Estado 1, 2 ou 3)
        else {
            // Descobrimos qual símbolo deveríamos estar esperando segundo a nossa rota
            int simbolo_esperado = -1;
            if (ruta_actual == 1) {
                simbolo_esperado = seq_abrir[estado_maquina];
            } else if (ruta_actual == 2) {
                simbolo_esperado = seq_fechar[estado_maquina];
            }

            // Comparamos a entrada com o símbolo estrito da nossa rota
            if (entrada_actual == simbolo_esperado) {
                estado_maquina++;
                printf("Correto! Avançando para o estado %d\n", estado_maquina);

                // Chegamos ao final da senha? (Estado 4)
                if (estado_maquina == 4) {
                    if (ruta_actual == 1) {
                        printf("SENHA DE ABRIR CORRETA! Acendendo LED...\n");
                        gpio_set_level(LED_GPIO, 1); 
                    } else if (ruta_actual == 2) {
                        printf("SENHA DE FECHAR CORRETA! Apagando LED...\n");
                        gpio_set_level(LED_GPIO, 0); 
                    }
                    
                    // Missão cumprida. Resetamos tudo para a próxima tentativa
                    estado_maquina = 0;
                    ruta_actual = 0; 
                }
            } 
            else {
                // Errou em algum passo intermediário
                printf("Erro na senha. Reiniciando para 0...\n");

                for(int i = 0; i < 5; i++) {
                    gpio_set_level(LED_GPIO, 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(LED_GPIO, 0);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                gpio_set_level(LED_GPIO, state);


                estado_maquina = 0;
                ruta_actual = 0; // Soltamos a rota
            }
        }
        // --- FIM DA MÁQUINA DE ESTADOS ---
    }
} 
else {
    contador_simbolo = 0;
    simbolo_previo = -1;
    contador_silencio++;

    if (contador_silencio == 8) {
        if (estado_maquina > 0 && estado_maquina <= 4) {
            printf("Timeout. Reiniciando a máquina e soltando a rota...\n");

            for(int i = 0; i < 5; i++) {
                    gpio_set_level(LED_GPIO, 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(LED_GPIO, 0);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            gpio_set_level(LED_GPIO, state);

            
            estado_maquina = 0; 
            ruta_actual = 0; // Importante resetar a rota no timeout 
        }
    }
}

    }
}

/* ADC Callback - called when one frame was acquired */
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

/* ADC init function */
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = MICEX_ADC_BUF_SIZE,
        .conv_frame_size = MICEX_ADC_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = MICEX_ADC_SAMPLE_FREQ,
        .conv_mode = MICEX_ADC_CONV_MODE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = MICEX_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = MICEX_ADC_UNIT;
        adc_pattern[i].bit_width = MICEX_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}
