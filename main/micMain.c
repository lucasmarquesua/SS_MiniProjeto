/* ********************************************************************************************************************************* 
 * Microphone test - ADC in continuous mode and time-domain BP filtering 
 * Paulo Pedreiras, Pedro Fonseca, Luis Moutinho 2026/Apr.
 * 
 * Tested:
 *  ESP32-C6 DevKitC-1
 * 
 * - Basic use of the ADC to get and process sound samples.
 * - Uses continuous mode ADC operation, to allow higher frequencies
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

#include "driver/gpio.h"  // para configuração dos leds GPIO11
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"      // FreeRTOS includes
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h" // For ESP ADC
#include "esp_dsp.h"                // For ESP DSP functions, conv in the case
#include "esp_private/esp_clk.h"    // For ESP clock functions

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

//Macros da LED11 GPIO11
#define LED_PIN         11
#define LED_PIN_SEL     (1ULL << LED_PIN)


#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048 /* IMPORTANT: If FFT is to be used, must be must be a power of two */
                                              /* For time-domain conv. filters there is no such restriction */
                                              
#define MAX_FILT_IR_LEN                 200     /* Maximum IR filter length */

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
__attribute__((aligned(16))) float hbpf2k[]={0.000000139618742, 0.000000255721385, 0.000000140050607, -0.000000328918009, -0.000001079871671, -0.000001790994587, -0.000001943975411, -0.000001031882007, 0.000001111877099, 0.000004001347557, 0.000006413035946, 0.000006707873993, 0.000003574260112, -0.000003048702149, -0.000011330185703, -0.000017688431396, -0.000017969871244, -0.000009435702605, 0.000007215862062, 0.000026871405806, 0.000040972415077, 0.000040614490911, 0.000020897907315, -0.000015285006327, -0.000056126887733, -0.000083842525959, -0.000081437626075, -0.000041101266796, 0.000029421836936, 0.000106148839227, 0.000155757573851, 0.000148773462501, 0.000074351529935, -0.000050052623102, -0.000180121406766, -0.000259437345082, -0.000243952642519, -0.000126937980784, 0.000047492844400, 0.000191919289194, 0.000580032052973, 0.000667352843244, 0.000306111499321, -0.000446298721323, -0.001342483904490, -0.001978250973038, -0.001912439692934,-0.000868574004532, 0.001047898364964, 0.003236203885917, 0.004708723047573, 0.004458080398213, 0.001979374693417, -0.002286980559547, -0.006863696045905, -0.009659616406773, -0.008836721670391, -0.003790852332498, 0.004230959606325, 0.012280364027838, 0.016725773018749, 0.014820278024314, 0.006161630397857, -0.006683261470121, -0.018835296877081, -0.024940029322091, -0.021501888240714, -0.008700432996997, 0.009216737308611, 0.025316468417048, 0.032708573103906, 0.027529205755171, 0.010869128872094, -0.011297781514959, -0.030304625508737, -0.038281206662112, -0.031498011959705, -0.012107397246638, 0.012536609979868, 0.032771101414432, 0.040552804004055, 0.032771101414432, 0.012536609979868, -0.012107397246638, -0.031498011959704, -0.038281206662113, -0.030304625508737, -0.011297781514959, 0.010869128872094, 0.027529205755171, 0.032708573103906, 0.025316468417048, 0.009216737308611, -0.008700432996997, -0.021501888240714, -0.024940029322091, -0.018835296877081, -0.006683261470121, 0.006161630397857, 0.014820278024314, 0.016725773018749, 0.012280364027838,0.004230959606325, -0.003790852332499, -0.008836721670391, -0.009659616406773, -0.006863696045905, -0.002286980559547, 0.001979374693417, 0.004458080398213, 0.004708723047573, 0.003236203885917, 0.001047898364964, -0.000868574004532, -0.001912439692934, -0.001978250973038, -0.001342483904490, -0.000446298721323, 0.000306111499321, 0.000667352843244, 0.000580032052973, 0.000191919289194, 0.000047492844400, -0.000126937980784, -0.000243952642519, -0.000259437345082, -0.000180121406766, -0.000050052623102, 0.000074351529935, 0.000148773462501, 0.000155757573851, 0.000106148839227, 0.000029421836936, -0.000041101266796, -0.000081437626075, -0.000083842525959, -0.000056126887733, -0.000015285006327, 0.000020897907315, 0.000040614490911, 0.000040972415077, 0.000026871405806, 0.000007215862062, -0.000009435702605, -0.000017969871244, -0.000017688431396, -0.000011330185703, -0.000003048702149, 0.000003574260112,0.000006707873993, 0.000006413035946, 0.000004001347557, 0.000001111877099, -0.000001031882007, -0.000001943975411, -0.000001790994587, -0.000001079871671, -0.000000328918009, 0.000000140050607, 0.000000255721385, 0.000000139618742};

/*Resposta impulsional do Filtro 2780hz */
__attribute__((aligned(16))) float hbpf2780[]={0.000371006265024 , 0.000535957222600 , 0.000314785189046 , -0.000180044399332 , -0.000614802815259 , -0.000646190970129 , -0.000177349231429 , 0.000521800575340 , 0.000948714725925 , 0.000710467501562 , -0.000144256576119 , -0.001067046638772 , -0.001335975567477 , -0.000594698096936 , 0.000780834138988 , 0.001824654726544 , 0.001631479293878 , 0.000099136576193 , -0.001821592030802 , -0.002677152371626 , -0.001579165793040 , 0.000972658766020 , 0.003227118850369 , 0.003349327684956 , 0.000868169994483 , -0.002710770335322 , -0.004763524426618 , -0.003438733198102 , 0.000758323946119 , 0.004997473948752 , 0.005994464671357 , 0.002515061828099 , -0.003371508247342 , -0.007450968868279 , -0.006352451627409 , -0.000268782778418 , 0.006748810290179 , 0.009449818338191 , 0.005284416486731 , -0.003334163470783 , -0.010336472958416 , -0.010250601524176 , -0.002437472253430 , 0.007946103866345 , 0.013313816034712 , 0.009179842335529 , -0.002170388025862 , -0.012827547462845 , -0.014760071066816 , -0.005851789178898 , 0.008054693516192 , 0.016956422149223 , 0.013889924405083 , 0.000346752321375 , -0.014288755051389 , -0.019243888824933 , -0.010296023354048 , 0.006712294942763 , 0.019660282803105 , 0.018807886650624 , 0.004125911380361 , -0.014223267527405 , -0.022932810508033 , -0.015231335755175 , 0.003867619952926 , 0.020808334498693 , 0.023149061887291 , 0.008728577111230 , -0.012431825112578 , -0.025114544292257 , -0.019894662101526 , -0.000163852015858 , 0.020060396199201 , 0.026136759379495 , 0.013445572252750 , -0.009095451855979 , -0.025324209445413 , -0.023476210877688 , -0.004751128676454 , 0.017461599055728 , 0.027202172655433 , 0.017461599055728 , -0.004751128676454 , -0.023476210877688 , -0.025324209445413 , -0.009095451855979 , 0.013445572252750 , 0.026136759379495 , 0.020060396199201 , -0.000163852015858 , -0.019894662101526 , -0.025114544292257 , -0.012431825112578 , 0.008728577111230 , 0.023149061887291 , 0.020808334498693 , 0.003867619952926 , -0.015231335755175 , -0.022932810508033 , -0.014223267527405 , 0.004125911380361 , 0.018807886650624 , 0.019660282803105 , 0.006712294942763 , -0.010296023354048 , -0.019243888824933 , -0.014288755051389 , 0.000346752321375 , 0.013889924405083 , 0.016956422149223 , 0.008054693516192 , -0.005851789178898 , -0.014760071066816 , -0.012827547462845 , -0.002170388025862 , 0.009179842335529 , 0.013313816034712 , 0.007946103866345 , -0.002437472253430 , -0.010250601524176 , -0.010336472958416 , -0.003334163470783 , 0.005284416486731 , 0.009449818338191 , 0.006748810290179 , -0.000268782778418 , -0.006352451627409 , -0.007450968868279 , -0.003371508247342 , 0.002515061828099 , 0.005994464671357 , 0.004997473948752 , 0.000758323946119 , -0.003438733198102 , -0.004763524426618 , -0.002710770335322 , 0.000868169994483 , 0.003349327684956 , 0.003227118850369 , 0.000972658766020 , -0.001579165793040 , -0.002677152371626 , -0.001821592030802 , 0.000099136576193 , 0.001631479293878 , 0.001824654726544 , 0.000780834138988 , -0.000594698096936 , -0.001335975567477 , -0.001067046638772 , -0.000144256576119 , 0.000710467501562 , 0.000948714725925 , 0.000521800575340 , -0.000177349231429 , -0.000646190970129 , -0.000614802815259 , -0.000180044399332 , 0.000314785189046 , 0.000535957222600 , 0.000371006265024};

/*Resposta impulsional do Filtro 3560hz */
__attribute__((aligned(16))) float hbpf3560[]={0.000031970915394 , 0.000499572812978 , 0.000429933459549 , -0.000168930248969 , -0.000658068764335 , -0.000426468811140 , 0.000372385763921 , 0.000865202751142 , 0.000382892901619 , -0.000676387406818 , -0.001115308159728 , -0.000253515422015 , 0.001111374910067 , 0.001379198004707 , -0.000021359192361 , -0.001692474309375 , -0.001600267152221 , 0.000505891383202 , 0.002408040447011 , 0.001695896647418 , -0.001255383525038 , -0.003210861886966 , -0.001564857936403 , 0.002301141102823 , 0.004014253286978 , 0.001100385400210 , -0.003636074531311 , -0.004694654681400 , -0.000207516668021 , 0.005203795361183 , 0.005101410989820 , -0.001177628469988 , -0.006893702757947 , -0.005073253564041 , 0.003069581194352 , 0.008543853027450 , 0.004459825933335 , -0.005419408089248 , -0.009952340524814 , -0.003145580175890 , 0.008107579978607 , 0.010896640273784 , 0.001072694815959 , -0.010945872197435 , -0.011159061563516 , 0.001740538928916 , 0.013689170025837 , 0.010555342217080 , -0.005189085187894 , -0.016056651548710 , -0.008962661644444 , 0.009080661141384 , 0.017760393583616 , 0.006343105385259 , -0.013146475220809 , -0.018538211262696 , -0.002758940844605 , 0.017062803368128 , 0.018186695435760 , -0.001623055422327 , -0.020481452951397 , -0.016590100414693 , 0.006539648036700 , 0.023065809233667 , 0.013741028663049 , -0.011651395300101 , -0.024528205258350 , -0.009749741433683 , 0.016573272686334 , 0.024663957245319 , 0.004840289422467 , -0.020912366833842 , -0.023377641067573 , 0.000666674927425 , 0.024308273502550 , 0.020698037017654 , -0.006382801043282 , -0.026471333110575 , -0.016779534957216 , 0.011889488357555 , 0.027213987575656 , 0.011889488357555 , -0.016779534957216 , -0.026471333110575 , -0.006382801043282 , 0.020698037017654 , 0.024308273502550 , 0.000666674927425 , -0.023377641067573 , -0.020912366833842 , 0.004840289422467 , 0.024663957245319 , 0.016573272686334 , -0.009749741433683 , -0.024528205258350 , -0.011651395300101 , 0.013741028663049 , 0.023065809233667 , 0.006539648036700 , -0.016590100414693 , -0.020481452951397 , -0.001623055422327 , 0.018186695435760 , 0.017062803368128 , -0.002758940844605 , -0.018538211262696 , -0.013146475220809 , 0.006343105385259 , 0.017760393583616 , 0.009080661141384 , -0.008962661644444 , -0.016056651548710 , -0.005189085187894 , 0.010555342217080 , 0.013689170025837 , 0.001740538928916 , -0.011159061563516 , -0.010945872197435 , 0.001072694815959 , 0.010896640273784 , 0.008107579978607 , -0.003145580175890 , -0.009952340524814 , -0.005419408089248 , 0.004459825933335 , 0.008543853027450 , 0.003069581194352 , -0.005073253564041 , -0.006893702757947 , -0.001177628469988 , 0.005101410989820 , 0.005203795361183 , -0.000207516668021 , -0.004694654681400 , -0.003636074531311 , 0.001100385400210 , 0.004014253286978 , 0.002301141102823 , -0.001564857936403 , -0.003210861886966 , -0.001255383525038 , 0.001695896647418 , 0.002408040447011 , 0.000505891383202 , -0.001600267152221 , -0.001692474309375 , -0.000021359192361 , 0.001379198004707 , 0.001111374910067 , -0.000253515422015 , -0.001115308159728 , -0.000676387406818 , 0.000382892901619 , 0.000865202751142 , 0.000372385763921 , -0.000426468811140 , -0.000658068764335 , -0.000168930248969 , 0.000429933459549 , 0.000499572812978 , 0.000031970915394}; 

/* Enumerado das máquinas de estado: Mais simples de resolver. */
typedef enum {
    ESTADO_ESPERA_SEQUENCIA,
    ESTADO_ERRO_PISCAR
    } t_estado;



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

    /* Configuração da LED no PORTO GPIO11 como saida digital */

    gpio_config_t io_conf = {
        .pin_bit_mask = LED_PIN_SEL,        // Seleciona o GPIO11
        .mode = GPIO_MODE_OUTPUT,           // Configura como Saída
        .pull_up_en = GPIO_PULLUP_DISABLE,  // Desativa Pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Desativa Pull-down
        .intr_type = GPIO_INTR_DISABLE      // Sem interrupções
    };
    gpio_config(&io_conf);
    
    // Garantir que o LED começa desligado (assumindo lógica ativa em HIGH)
    gpio_set_level(LED_PIN, 0);

    


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
   /* Local vars, for auxiliary computations */    
    int n;        
    float * sound_samp_buf_proc; // Buffer para os samples originais
    
    // 1. Descobrir o tamanho do filtro fornecido (2000 Hz)
    int n_fir_2k = sizeof(hbpf2k) / sizeof(hbpf2k[0]);
    int n_fir_2780 = sizeof(hbpf2780) / sizeof(hbpf2780[0]);
    int n_fir_3560 = sizeof(hbpf3560) / sizeof(hbpf3560[0]);

    // O tamanho do sinal de saída de uma convolução é (Tamanho_Sinal + Tamanho_Filtro - 1)
    int out_len = MICEX_SOUND_SAMPLES_BUF_SIZE + n_fir_2k - 1;
    int out_len2780 = MICEX_SOUND_SAMPLES_BUF_SIZE + n_fir_2780 - 1;
    int out_len3560 = MICEX_SOUND_SAMPLES_BUF_SIZE + n_fir_3560 - 1;

    // Buffer para guardar o sinal DEPOIS de filtrado
    float * sinal_filtrado = heap_caps_malloc(sizeof(float) * out_len, MALLOC_CAP_DMA);       
    float * sinal_filtrado2780 = heap_caps_malloc(sizeof(float) * out_len2780, MALLOC_CAP_DMA); 
    float * sinal_filtrado3560 = heap_caps_malloc(sizeof(float) * out_len3560, MALLOC_CAP_DMA); 
    /* Variable inits */
    sound_samp_buf_proc = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);
    
    //Sinais de controlo da máquina de estados
    //Sequencia de abertura da porta: 2000 Hz -> 2780 Hz -> 3560 Hz
     int SEQ_ABRIR[4]  = {0, 1, 2, 0}; 
     int SEQ_FECHAR[4] = {2, 1, 0, 2};

    t_estado estado_atual = ESTADO_ESPERA_SEQUENCIA;
    int digito_idx = 0; // Representa em que "bola" (estado 0 a 3) estamos atualmente
    int sequencia_capturada[4] = {-1, -1, -1, -1};
    int ultimo_tom_processado = -1;
    uint32_t tempo_inicio_erro = 0;
    int blink_contador = 0;     



    /* Infinite processing loop */
    for(;;) {
        /* Waits for new data */
        xQueueReceive(XQ,(void *)sound_samp_buf_proc,portMAX_DELAY);
        
        // PASSO 1: Aplicar o filtro (Convolução)
        // A função dsps_conv_f32 filtra o som capturado usando os coeficientes dos 2000 Hz
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, hbpf2k, n_fir_2k, sinal_filtrado);
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, hbpf2780, n_fir_2780, sinal_filtrado2780);
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, hbpf3560, n_fir_3560, sinal_filtrado3560);


        // PASSO 2: Calcular a "Força" (Energia) do sinal filtrado
        // Somamos o quadrado de todas as amostras. Se houver muito som nos 2000Hz, este valor será enorme.
        float energia_2k = 0;
        float energia_2780 = 0;
        float energia_3560 = 0;
        for(n = 0; n < out_len; n++) {
            energia_2k += sinal_filtrado[n] * sinal_filtrado[n];
            energia_2780 += sinal_filtrado2780[n] * sinal_filtrado2780[n];
            energia_3560 += sinal_filtrado3560[n] * sinal_filtrado3560[n];

        }

        // PASSO 3: Imprimir a energia para podermos definir um Limiar (Threshold)
        // Por agora vamos apenas ler este valor no terminal para perceber os níveis do microfone
        
        //ESP_LOGI(TAG, "Energia nos 2000 Hz: %f", energia_2k);
        //ESP_LOGI(TAG, "Energia nos 2780 Hz: %f", energia_2780);
        //ESP_LOGI(TAG, "Energia nos 3560 Hz: %f", energia_3560);

        //LIMIARES 

    //Linha base de ruido na zona de testes (sem som, mas com o microfone ligado e captando o ambiente)
     float RUIDO_BASE_0 = 1270000.0f; // Canal 2000 Hz 
     float RUIDO_BASE_1 =  626000.0f; // Canal 2780 Hz
     float RUIDO_BASE_2 =  396000.0f; // Canal 3560 Hz


     float LIMIAR_UTIL_0 =   60000.0f; // Margem para 2000 Hz
     float LIMIAR_UTIL_1 =  300000.0f; // Margem para 2780 Hz
     float LIMIAR_UTIL_2 =  300000.0f; // Margem para 3560 Hz


            // EXTRAÇÃO DO SINAL ÚTIL (Subtração do Ruído Fundo)
            // Coloquei o microfone aqui em silencio aqui. e deu esses valores, usei o codigo que
            // está comentado na linha 286,287 e 289. para saber como ia fazer as contas pra tirar o ruído.
            // Deve ter um jeito mais profissional de fazer, mas funciona bem assim.
     float liq_0 = energia_2k - RUIDO_BASE_0;
     float liq_1 = energia_2780 - RUIDO_BASE_1;
     float liq_2 = energia_3560 - RUIDO_BASE_2;

    if (liq_0 < 0) liq_0 = 0;
    if (liq_1 < 0) liq_1 = 0;
    if (liq_2 < 0) liq_2 = 0;  

        //Aqui resolve a situação dos sons que eu expliquei lá em cima.
      // ------------------------------------------------------------------
        // DETEÇÃO POR RÁCIO SINAL/RUÍDO (PUREZA DO TOM)
        int tom_detetado = -1;
        
        // Multiplicador de Pureza (3.0 significa que o tom tem de ser 3x mais forte que o lixo)
        float FATOR_PUREZA = 3.0f; 

        if (liq_0 > LIMIAR_UTIL_0 && liq_0 > FATOR_PUREZA * (liq_1 + liq_2)) {
            tom_detetado = 0;
        } 
        else if (liq_1 > LIMIAR_UTIL_1 && liq_1 > FATOR_PUREZA * (liq_0 + liq_2)) {
            tom_detetado = 1;
        } 
        else if (liq_2 > LIMIAR_UTIL_2 && liq_2 > FATOR_PUREZA * (liq_0 + liq_1)) {
            tom_detetado = 2;
        } else {
            // Se cair aqui, é silêncio OU um som complexo com várias frequências ativas
            tom_detetado = -1;
        }

    
        int tom_efetivo = tom_detetado;
        if (tom_detetado == ultimo_tom_processado && tom_detetado != -1) {
            tom_efetivo = -1; 
        }
        ultimo_tom_processado = tom_detetado;

        /*
        * Foi criado um enumerado para simplificar a visualização da maquina de estado, também daria 
        para fazer de outro jeito, mas achei mais simples e acessível para nós.

        Os maiores problemas que encontrei na maquina de estados foram:
            * 1.) Quando transmitimos o som certo por 1, por exemplo - temos fs_microfone = 20Khz 
            * periodo_amostragem = 1/fs_microfone  = 50x10^(-6) s.
            * A FIFO enche a cada 50x10^(-6) * 2048 = 102x10^(-3);
            * Se tocarmos um som durante 1 s temos 1000ms / 102ms = 10 FIFOS com o mesmo som,
            * Assim a maquina iria achar que seria 10 simbolos e iria causar um erro instantaneo
             
            
        
        
        */
        switch (estado_atual) {

            case ESTADO_ESPERA_SEQUENCIA:
                if (tom_efetivo != -1) { 
                    sequencia_capturada[digito_idx] = tom_efetivo;
                    ESP_LOGI(TAG, "Estado %d -> Ouviu símbolo %d", digito_idx, tom_efetivo);
                    digito_idx++;

                    // Valida se a sequência em curso ainda corresponde a "Abrir" ou "Fechar"
                    bool caminho_abrir_valido = true;
                    bool caminho_fechar_valido = true;
                    
                    for(int i = 0; i < digito_idx; i++) {
                        if (sequencia_capturada[i] != SEQ_ABRIR[i]) caminho_abrir_valido = false;
                        if (sequencia_capturada[i] != SEQ_FECHAR[i]) caminho_fechar_valido = false;
                    }

                    // Se não for nem abrir nem fechar, Erro! Volta ao início.
                    if (!caminho_abrir_valido && !caminho_fechar_valido) {
                        ESP_LOGW(TAG, "Som errado! Sequência abortada. A iniciar Erro (5s)...");
                        digito_idx = 0;
                        tempo_inicio_erro = xTaskGetTickCount(); //isso daqui é para guardar o tempo atual 
                        estado_atual = ESTADO_ERRO_PISCAR;
                    } 
                    // Se chegou à bola 4 (último dígito) com sucesso
                    else if (digito_idx == 4) {
                        if (caminho_abrir_valido) {
                            ESP_LOGI(TAG, ">>> SEQUÊNCIA COMPLETA: ABRIR PORTA (LED ON) <<<");
                            gpio_set_level(LED_PIN, 1);
                        } else if (caminho_fechar_valido) {
                            ESP_LOGI(TAG, ">>> SEQUÊNCIA COMPLETA: FECHAR PORTA (LED OFF) <<<");
                            gpio_set_level(LED_PIN, 0);
                        }
                        
                        // Retorna para o Estado Inicial (Aguardar nova sequência)
                        digito_idx = 0;
                    }
                }
                // Se tom_efetivo == -1 (Ruído/Silêncio), permanece no estado infinitamente (não faz nada)
                break;

            case ESTADO_ERRO_PISCAR:
                // Pisca o LED de forma não-bloqueante a cada bloco recebido
                blink_contador++;
                gpio_set_level(LED_PIN, (blink_contador % 2 == 0));

                // Verifica se já passaram 5000 milissegundos
                if ((xTaskGetTickCount() - tempo_inicio_erro) >= pdMS_TO_TICKS(5000)) {
                    gpio_set_level(LED_PIN, 0); // Garante que o LED fica desligado no fim
                    ESP_LOGI(TAG, "Fim do Erro. Sistema reiniciado para o Estado Inicial.");
                    estado_atual = ESTADO_ESPERA_SEQUENCIA; // Volta a ficar pronto para ouvir
                }
                break;
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
