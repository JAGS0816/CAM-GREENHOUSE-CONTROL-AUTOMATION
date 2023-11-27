#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "freertos/event_groups.h"
#include "driver/adc.h"
#include <math.h>

// MQTT Libraries
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
// WiFi libraries
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "driver/uart.h"



// MQTT Constants
#define MQTT_SERVER 			"mqtt://192.168.0.100:1883"

// Published MQTT TOPICS
#define MQTT_TOPIC_HUMIDITY1 	"E3/Hum1"
#define MQTT_TOPIC_HUMIDITY2 	"E3/Hum2"
#define MQTT_TOPIC_HUMIDITY3	"E3/Hum3"
#define MQTT_TOPIC_HUMIDITY4	"E3/Hum4"

#define MQTT_TOPIC_FLOW1		"E3/Flu1"
#define MQTT_TOPIC_FLOW2		"E3/Flu2"

#define MQTT_TOPIC_MODE			"E3/Mode"
#define MQTT_TOPIC_ONOFF1		"E3/Onoff1"
#define MQTT_TOPIC_ONOFF2		"E3/Onoff2"

#define MQTT_TOPIC_TIME1		"E3/T1"
#define MQTT_TOPIC_TIME2		"E3/T2"

// Subscribed MQTT TOPICS
#define MQTT_TOPIC_HUMDES1  	"E3/Hdes1"
#define MQTT_TOPIC_HUMDES2 		"E3/Hdes2"

#define MQTT_TOPIC_TIME_RECIEVED1		"E3/TR1"
#define MQTT_TOPIC_TIME_RECIEVED2		"E3/TR2"

#define AUTO	0
#define MANUAL	1
#define TIMER	2

#define TOLERANCE .05

#define SENS_HUM_1 ADC1_CHANNEL_6
#define SENS_HUM_2 ADC1_CHANNEL_7
#define SENS_HUM_3 ADC1_CHANNEL_4
#define SENS_HUM_4 ADC1_CHANNEL_5

#define ATM1 GPIO_NUM_25
#define ATM2 GPIO_NUM_26

#define BUTTON_P5 GPIO_NUM_23
#define BUTTON_M5 GPIO_NUM_22

#define TX0_GPIO GPIO_NUM_1

#define FLOW1_GPIO GPIO_NUM_21
#define FLOW2_GPIO GPIO_NUM_19

#define ELECTRO_VALVE1 GPIO_NUM_18
#define ELECTRO_VALVE2 GPIO_NUM_5

#define LINE1_BUTTON GPIO_NUM_4
#define LINE2_BUTTON GPIO_NUM_2

#define MAX_TIME 60
//Commands
#define COMMAND_TP 'a'
#define COMMAND_TM 'b'
#define COMMAND_HUMEDAD_1 'h'
#define COMMAND_HUMEDAD_2 'i'
#define COMMAND_TIME_1	  't'
#define COMMAND_TIME_2	  'u'

// Functions definitions for MQTT
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, long int event_id, void *event_data);
static void mqtt_app_start(void);
static void mqtt_init(void);

#define UART_NUM UART_NUM_2
#define TXD_PIN (GPIO_NUM_17)
#define BAUD_RATE 115200

esp_mqtt_client_handle_t client;

//
float humidity = 0;
float ref_value = 0;

//Delays
int delay_humidity = 1000;
int delay_flow = 1000;
int delay_control = 1000;
int delay_time = 60000;
int delay_mqtt = 10000;
int delay_time_data = 300;
int delay_uart = 4000;

// Control Variables
float error = 0;
float acc_error = 0;
float diff_error = 0;
float last_error = 0;
float control = 0;
int output_signal = 0;

// Flags
int time_interrupt_flag = 0;

struct new_control{
	float error;
	float ref_humidity;
	float avg_humidity;
};

struct new_timer{
	int time;
};

struct new_line{
	int flow_gpio;
	int electrovalve_gpio;
	int humidity1_gpio;
	int humidity2_gpio;
	int button_gpio;

	int onoff;
	float humidity1;
	float humidity2;
	float flow;

	int pulse_count;

	int button;
	int mode;

	struct new_control control;
	struct new_timer timer;
};

struct new_line line1;
struct new_line line2;

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);
}


void send_command(char type, int number) {
    char comando[5];

    snprintf(comando, sizeof(comando), "%c%d", type, number);

    uart_write_bytes(UART_NUM, comando, strlen(comando));

}

void init_new_line(int flow, int electrovalve, int humidity1, int humidity2, int button, gpio_isr_t isr_handler, struct new_line* line) {

    line->flow_gpio = flow;
    line->electrovalve_gpio = electrovalve;
    line->humidity1_gpio = humidity1;
    line->humidity2_gpio = humidity2;
    line->button_gpio = button;

    line->onoff = 0;
    line->humidity1 = 0.0;
    line->humidity2 = 0.0;
    line->flow = 0.0;

    line->pulse_count = 0;

    line->button = 0;
    line->mode = 0;

    line->control.error = 0.0;
    line->control.ref_humidity = 0.0;
    line->control.avg_humidity = 0.0;

	line->timer.time = 0.0;


    //Flow init
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << flow),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
	gpio_isr_handler_add(flow, isr_handler, (void*) line);

    //Electrovalve init
    gpio_reset_pin(electrovalve);
    gpio_set_direction(electrovalve, GPIO_MODE_OUTPUT);

    //Humidity init
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(humidity1, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(humidity2, ADC_ATTEN_DB_11);

	//Button
	gpio_reset_pin(button);
	gpio_set_direction(button, GPIO_MODE_INPUT);


	gpio_config(&io_conf);

}

void control_humidity_task(void* arg);
void flow_isr_handler(void* arg);
void calculate_flow_task(void* arg);
void calculate_humidity_task(void* arg);
void control_humidity_task(void* arg);
void change_time_task(void* arg);
void send_data_mqtt_task(void* arg);
void init_buttons();
void init_line_mode(struct new_line* line);
void send_time_to_mqtt();
void send_time_data_task(void* arg);
void send_data_to_uart(void* arg);

//Task han-dlers
TaskHandle_t send_time_data_handle = NULL;

void app_main()
{

	// Init MQTT Communication

	mqtt_init();
	mqtt_app_start();


    //Line 1
    init_new_line(FLOW1_GPIO, ELECTRO_VALVE1, SENS_HUM_1, SENS_HUM_2, LINE1_BUTTON, flow_isr_handler, &line1);

    //Line 2
	init_new_line(FLOW2_GPIO, ELECTRO_VALVE2, SENS_HUM_3, SENS_HUM_4, LINE2_BUTTON, flow_isr_handler, &line2);

	//Buttons
	init_buttons();

	//Mode

	init_line_mode(&line1);
	init_line_mode(&line2);

	//Init uart
	init_uart();

    xTaskCreate(calculate_flow_task, "calculate_flow_task", 4096, NULL, 6, NULL);
    xTaskCreate(calculate_humidity_task, "calculate_humidity_task", 4096, NULL, 2, NULL);
    xTaskCreate(control_humidity_task, "control_humidity_task", 4096, NULL, 3, NULL);
    xTaskCreate(change_time_task, "change_time_task", 4096, NULL, 4, NULL);
    xTaskCreate(send_data_mqtt_task, "send_data_mqtt_task", 4096, NULL, 5, NULL);
    xTaskCreate(send_time_data_task, "send_time_data_task", 4096, NULL, 5, &send_time_data_handle);
    xTaskCreate(send_data_to_uart, "send_data_to_uart", 4096, NULL, 5, NULL);





    while(1){

    	vTaskDelay( 10000/portTICK_PERIOD_MS);
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR flow_isr_handler(void* arg) {
	struct new_line* line = (struct new_line*)arg;
    line->pulse_count++;
}

void change_line_time(struct new_line* line, int delta_time){
	int new_time = line->timer.time;
	if (line->button){
		new_time += delta_time;
	}
	new_time = (new_time > MAX_TIME) ? MAX_TIME: new_time;
	line->timer.time = (new_time < 0) ? 0: new_time;
}

void IRAM_ATTR time_button_isr_handler(void* arg) {
	if (line1.mode == TIMER){
		int pin = (int)arg;
		int delta_time = 0;
		if (pin == BUTTON_P5) {
			delta_time = 5;
			time_interrupt_flag = 1;
		} else if (pin == BUTTON_M5) {
			delta_time = -5;
			time_interrupt_flag = 0;
		}


		change_line_time(&line1, delta_time);
		change_line_time(&line2, delta_time);
		xTaskResumeFromISR(send_time_data_handle);
	}
}

void change_line_mode(struct new_line* line){
	int atm1_state = gpio_get_level(ATM1);
	int atm2_state = gpio_get_level(ATM2);
	int mode = line->mode;
	// Timer
	if (!atm1_state && !atm2_state) {
		mode = AUTO;
	}
	// Auto
	else if (atm1_state && !atm2_state) {
		mode = MANUAL;
	}
	// Manual
	else if (!atm1_state && atm2_state) {
		mode = TIMER;
	}

	line->mode = mode;
}

void IRAM_ATTR mode_button_isr_handler(void* arg) {
	change_line_mode(&line1);
	change_line_mode(&line2);
}

void check_line_button(struct new_line* line){
	int level = gpio_get_level(line->button_gpio);
	line->button = level;
}

void IRAM_ATTR line_button_isr_handler(void* arg) {
	check_line_button(&line1);
	check_line_button(&line2);
}

void init_buttons(){
    gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << BUTTON_P5) | (1ULL << BUTTON_M5),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
	};

    gpio_config(&io_conf);

    gpio_isr_handler_add(BUTTON_P5, time_button_isr_handler, (void*)(int)BUTTON_P5);
	gpio_isr_handler_add(BUTTON_M5, time_button_isr_handler, (void*)(int)BUTTON_M5);

	gpio_config_t io_conf1 = {
		        .pin_bit_mask = (1ULL << ATM1) | (1ULL << ATM2),
		        .mode = GPIO_MODE_INPUT,
		        .intr_type = GPIO_INTR_ANYEDGE,
		        .pull_up_en = GPIO_PULLUP_DISABLE,
		        .pull_down_en = GPIO_PULLDOWN_DISABLE,
		    };
	gpio_config(&io_conf1);

	gpio_isr_handler_add(ATM1, mode_button_isr_handler, (void*)ATM1);
	gpio_isr_handler_add(ATM2, mode_button_isr_handler, (void*)ATM2);

	gpio_config_t io_conf2 = {
			        .pin_bit_mask = (1ULL << LINE1_BUTTON) | (1ULL << LINE2_BUTTON),
			        .mode = GPIO_MODE_INPUT,
			        .intr_type = GPIO_INTR_ANYEDGE,
			        .pull_up_en = GPIO_PULLUP_DISABLE,
			        .pull_down_en = GPIO_PULLDOWN_DISABLE,
			    };
	gpio_config(&io_conf2);

	gpio_isr_handler_add(LINE1_BUTTON, line_button_isr_handler, (void*)LINE1_BUTTON);
	gpio_isr_handler_add(LINE2_BUTTON, line_button_isr_handler, (void*)LINE2_BUTTON);
}

void get_flow_rate(struct new_line* line, int elapsed_time){

	float frequency = (float)line->pulse_count / ((float)elapsed_time / ( portTICK_PERIOD_MS ));
	float flow_rate = -14.68 + 7.78*frequency;
	flow_rate = (flow_rate < 0) ? 0 : flow_rate;

	line->pulse_count = 0;
	line->flow = flow_rate;

	printf("flow: %.2f\n", flow_rate);
}

void calculate_flow_task(void* arg) {
    TickType_t last_time = xTaskGetTickCount();

    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        TickType_t elapsed_time = current_time - last_time;
        last_time = current_time;

        //Line 1
        get_flow_rate(&line1, elapsed_time);

        //Line 2
        get_flow_rate(&line2, elapsed_time);
		printf("Flow1: %.2f \t Flow2: %.2f\n",line1.flow, line2.flow);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void get_humidity(struct new_line* line){

	float sensor1_raw = 4095 - adc1_get_raw(line->humidity1_gpio);
	float sensor2_raw = 4095 - adc1_get_raw(line->humidity2_gpio);

	float humidity1 = pow(M_E, ((float)sensor1_raw - 165.72)/825.26);
	float humidity2 = pow(M_E, ((float)sensor2_raw - 165.72)/825.26);

	humidity1 = (humidity1 > 99) ? 99: humidity1;
	humidity2 = (humidity2 > 99) ? 99: humidity2;

	line->humidity1 = humidity1;
	line->humidity2 = humidity2;
}

void calculate_humidity_task(void* arg) {
    while (1) {

    	//Line 1
    	get_humidity(&line1);
    	//Line 2
		get_humidity(&line2);
		printf("Humidity calculated\n");

        vTaskDelay(delay_humidity / portTICK_PERIOD_MS);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void line_set_level(struct new_line* line, int onoff){
	gpio_set_level(line->electrovalve_gpio, onoff);
	line->onoff=onoff;
}

void control_line(struct new_line* line){
	float ref_humidity = line->control.ref_humidity;
	float avg_humidity = (line->humidity1 + line->humidity2)/2;
	float error = (ref_humidity - avg_humidity)/ref_humidity;
	int output_signal = line->onoff;
	line->control.error = error;

	if (error < 0){
		output_signal = 0;
	}
	else if (error > TOLERANCE){
		output_signal = 1;
	}
	line_set_level(line, output_signal);
}

void manual_control_line(struct new_line* line){
	int onoff = line->button;
	gpio_set_level(line->electrovalve_gpio, onoff);
	line->onoff=onoff;
}

void timer_control_line(struct new_line* line){

	int onoff = (line->timer.time != 0);
	gpio_set_level(line->electrovalve_gpio, onoff);
	line->onoff=onoff;
}

void control_humidity_task(void* arg){

	while(1){
		switch (line1.mode){
			case AUTO:
				control_line(&line1);
				control_line(&line2);
				break;
			case MANUAL:
				manual_control_line(&line1);
				manual_control_line(&line2);
				break;
			case TIMER:
				timer_control_line(&line1);
				timer_control_line(&line2);

				printf("Time \tLine1: %d \t Line2: %d \n", line1.timer.time, line2.timer.time);
				printf("Button \tLine1: %d \t Line2: %d \n", line1.button, line2.button);
				break;
		}
		printf("Humidity controled\n");

		vTaskDelay(delay_control / portTICK_PERIOD_MS);
	}
}


void change_time_task(void* arg){

	while(1){
		change_line_time(&line1, -1);
		change_line_time(&line2, -1);
		printf("Time changed\n");

		send_time_to_mqtt();

		vTaskDelay(delay_time / portTICK_PERIOD_MS);
	}
}

//--------------------------------------------------------
// MQTT App Start
//--------------------------------------------------------
static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
       .broker.address.uri = MQTT_SERVER,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

//--------------------------------------------------------
// MQTT Init
//--------------------------------------------------------
static void mqtt_init(void)
{
	printf("[APP] Startup..\n");
    printf("[APP] Free memory: %ld bytes\n", esp_get_free_heap_size());
    printf("[APP] IDF version: %s\n", esp_get_idf_version());

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    example_connect();
}

void send_time_to_mqtt(){
	char time_line1[10];
	char time_line2[10];

	float time1 = line1.timer.time;
	float time2 = line2.timer.time;

	sprintf(time_line1, "%.2f", time1);
	sprintf(time_line2, "%.2f", time2);

	esp_mqtt_client_publish(client, MQTT_TOPIC_TIME1, time_line1, 0, 1, 0);
	esp_mqtt_client_publish(client, MQTT_TOPIC_TIME2, time_line2, 0, 1, 0);
}

void send_data_mqtt_task(void* arg) {

    char str_humidity1[10];
    char str_humidity2[10];
    char str_humidity3[10];
    char str_humidity4[10];
    char str_flow1[10];
    char str_flow2[10];
    char str_mode[10];
    char str_onoff1[10];
    char str_onoff2[10];


	while(1){
	    float humidity1 = line1.humidity1;
	    float humidity2 = line1.humidity2;
	    float humidity3 = line2.humidity1;
	    float humidity4 = line2.humidity2;

		float flow1 = line1.flow;
		float flow2 = line2.flow;

		int mode = line1.mode;

		int onoff1 = line1.onoff;
		int onoff2 = line2.onoff;

		sprintf(str_humidity1, "%.2f", humidity1);
		sprintf(str_humidity2, "%.2f", humidity2);
		sprintf(str_humidity3, "%.2f", humidity3);
		sprintf(str_humidity4, "%.2f", humidity4);
		sprintf(str_flow1, "%.2f", flow1);
		sprintf(str_flow2, "%.2f", flow2);
		sprintf(str_mode, "%d", mode);
		sprintf(str_onoff1, "%d", onoff1);
		sprintf(str_onoff2, "%d", onoff2);


		printf("Humidity: %.2f\n", humidity1);
		printf("Humidity2: %.2f\n", humidity2);
		printf("Humidity3: %.2f\n", humidity3);
		printf("Humidity4: %.2f\n", humidity4);
		printf("flow1: %.2f\n", flow1);
		printf("flow2: %.2f\n", flow2);
		printf("mode: %d\n", mode);
		printf("onoff1: %d\n", onoff1);
		printf("onoff: %d\n", onoff2);

		printf("----------- Data sent to MQTT Broker ------------\n");

		esp_mqtt_client_publish(client, MQTT_TOPIC_HUMIDITY1, str_humidity1, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_HUMIDITY2, str_humidity2, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_HUMIDITY3, str_humidity3, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_HUMIDITY4, str_humidity4, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_FLOW1, str_flow1, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_FLOW2, str_flow2, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_MODE, str_mode, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_ONOFF1, str_onoff1, 0, 1, 0);

		esp_mqtt_client_publish(client, MQTT_TOPIC_ONOFF2, str_onoff2, 0, 1, 0);

		vTaskDelay(delay_mqtt/portTICK_PERIOD_MS);
	}
}


void init_line_mode(struct new_line* line){
	check_line_button(line);
	change_line_mode(line);
}

void send_time_data_task(void* args){
	while(1){
		vTaskSuspend(NULL);
		send_time_to_mqtt();
		if (time_interrupt_flag){
			send_command(COMMAND_TP, 0);
		}
		else{
			send_command(COMMAND_TM, 0);
		}
	}
}

void send_data_to_uart(void* arg){
	while(1){
		int humidity1 = (line1.humidity1 + line1.humidity2)/2;
		humidity1 = (humidity1>99) ? 99: humidity1;
		int humidity2 = (line2.humidity1 + line2.humidity2)/2;
		humidity2 = (humidity2>99) ? 99: humidity2;

		int time1 = line1.timer.time;
		int time2 = line2.timer.time;

		send_command(COMMAND_HUMEDAD_1, humidity1);
		send_command(COMMAND_HUMEDAD_2, humidity2);
		send_command(COMMAND_TIME_1, time1);
		send_command(COMMAND_TIME_2, time2);

		printf("Valores enviados por UART \n");

		vTaskDelay( delay_uart/portTICK_PERIOD_MS );
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////7
//--------------------------------------------------------
// MQTT Event Handler
//--------------------------------------------------------

void line_desired_humidity(struct new_line* line, int humidity){
	line->control.ref_humidity = humidity;
}

void absolut_time_change(struct new_line* line, int new_time){
	line->timer.time = new_time;
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char topic[80];
    char data[20];

    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
        	printf("MQTT_EVENT_CONNECTED\n");
			msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_HUMDES1, 0);
			printf("Sent subscribe successful to %s, msg_id=%d\n", MQTT_TOPIC_HUMDES1, msg_id);
			msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_HUMDES2, 0);
			printf("Sent subscribe successful to %s, msg_id=%d\n", MQTT_TOPIC_HUMDES2, msg_id);
			msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_TIME_RECIEVED1, 0);
			printf("Sent subscribe successful to %s, msg_id=%d\n", MQTT_TOPIC_TIME_RECIEVED1, msg_id);
			msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_TIME_RECIEVED2, 0);
			printf("Sent subscribe successful to %s, msg_id=%d\n", MQTT_TOPIC_TIME_RECIEVED2, msg_id);
			break;
        case MQTT_EVENT_DISCONNECTED:
        	printf("MQTT_EVENT_DISCONNECTED\n");
            break;
        case MQTT_EVENT_SUBSCRIBED:
        	printf("MQTT_EVENT_SUBSCRIBED, msg_id=%d\n", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            printf("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d\n", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
        	printf("MQTT_EVENT_PUBLISHED, msg_id=%d\n", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
        	printf("----------- Data received from MQTT Broker ------------\n");
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
			printf("DATA=%.*s\r\n", event->data_len, event->data);
			strncpy(topic,event->topic,event->topic_len);
			topic[event->topic_len]=0;
			strncpy(data,event->data,event->data_len);
			data[event->data_len]=0;

            // If topic detected is on/off control
            if (strcmp(topic,MQTT_TOPIC_HUMDES1)==0){
        		printf("Detected topic: %s\n",MQTT_TOPIC_HUMDES1);
            	int humdes1 = atoi(data);
            	line_desired_humidity(&line1, humdes1);
            }

            // If topic detected is on/off control
			if (strcmp(topic,MQTT_TOPIC_HUMDES2)==0){
				printf("Detected topic: %s\n",MQTT_TOPIC_HUMDES2);
				int humdes2 = atoi(data);
				line_desired_humidity(&line2, humdes2);
			}

			if (strcmp(topic,MQTT_TOPIC_TIME_RECIEVED1)==0 && line1.mode==TIMER){
				printf("Detected topic: %s\n",MQTT_TOPIC_TIME_RECIEVED1);
				int new_time = atoi(data);
				absolut_time_change(&line1, new_time);
			}

			if (strcmp(topic,MQTT_TOPIC_TIME_RECIEVED2)==0 && line1.mode==TIMER){
				printf("Detected topic: %s\n",MQTT_TOPIC_TIME_RECIEVED2);
				int new_time = atoi(data);
				absolut_time_change(&line2, new_time);
			}

			break;
        case MQTT_EVENT_ERROR:
        	printf("MQTT_EVENT_ERROR\n");
            break;
        default:
        	printf("Other event id:%d\n", event->event_id);
            break;
    }
    return ESP_OK;
}

//--------------------------------------------------------
// MQTT Event Handler
//--------------------------------------------------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, long int event_id, void *event_data)
{
    printf("Event dispatched from event loop base=%s, event_id=%ld\n", base, event_id);
    mqtt_event_handler_cb(event_data);
}


