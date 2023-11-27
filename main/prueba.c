#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "freertos/event_groups.h"
#include "driver/adc.h"
#include <math.h>

#define SENSOR_PIN GPIO_NUM_34
#define RELAY_GPIO 26

//
float humidity = 0;
float ref_value = 30;

//Delays
int delay_humidity = 1000;
int delay_flow = 1000;
int delay_control = 1000;


// Control Variables
float error = 0;
float acc_error = 0;
float diff_error = 0;
float last_error = 0;
float control = 0;
int output_signal = 0;

float Kp = 20;
float Ki = 30;
float Kd = 5;

static volatile int pulse_count = 0;

struct new_line{
	int onoff;
	float humidity;
	float flow;
	int pin;
};

void IRAM_ATTR gpio_isr_handler(void* arg) {
    pulse_count++;
}

void calculate_frequency_task(void* arg) {
    TickType_t last_time = xTaskGetTickCount(); // Obtiene el tiempo actual

    while (1) {
        TickType_t current_time = xTaskGetTickCount(); // Obtiene el tiempo actual
        TickType_t elapsed_time = current_time - last_time; // Calcula el tiempo transcurrido
        last_time = current_time; // Actualiza el tiempo anterior

        float frequency = (float)pulse_count / ((float)elapsed_time / ( portTICK_PERIOD_MS ));
        float flow_rate = -14.68 + 7.78*frequency;// Calcula la frecuencia en Hz

        flow_rate = (flow_rate < 60) ? 0 : flow_rate;

        pulse_count = 0; // Reinicia el contador de pulsos

        printf("Flow rate: %.2f L/H\n", flow_rate);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera 1 segundo antes de calcular la frecuencia nuevamente
    }
}

void calculate_humidity_task(void* arg) {
    while (1) {
        // Leer valor del sensor de humedad (asumiendo que utilizas un ADC)
        int sensor_value = 4095 - adc1_get_raw(ADC1_CHANNEL_4); // Cambia el canal ADC según tu configuración

        /*
        float res = (4095 - (float)sensor_value)*13 / (float)sensor_value;

        res = res > 80 ? 80 : res;

        // Calcular la humedad en base al valor del sensor
        humidity = (0.016098 * res*res + -0.153206 * res); // Suponiendo un rango de 0-100% de humedad
        */
        humidity = pow(M_E, ((float)sensor_value - 165.72)/825.26);

        printf("Humidity: %.2f \tSens: %d \n", humidity, sensor_value);

        vTaskDelay(delay_humidity / portTICK_PERIOD_MS); // Esperar 1 segundo antes de volver a leer el sensor
    }
}

void control_humidity_task(void* arg){
	while(1){
		float dt = delay_humidity/1000;
		float error = ref_value - humidity;

		acc_error += error*dt;

		diff_error = (error - last_error)/dt;

		control = Kp*error + Ki*acc_error + Kd*diff_error;

		printf("Control: %.2f \n", control);

		if (control < .5){
			output_signal = 0;
		}
		else{
			output_signal = 1;
		}
		last_error = error;
		gpio_set_level(RELAY_GPIO, output_signal);

		vTaskDelay(delay_control / portTICK_PERIOD_MS);
	}
}

void app_main()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE, // Detecta cualquier cambio en el pin
    };

    gpio_config(&io_conf);

    gpio_reset_pin(RELAY_GPIO);
	gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    gpio_install_isr_service(0); // Instala el servicio de interrupciones
    gpio_isr_handler_add(SENSOR_PIN, gpio_isr_handler, (void*) SENSOR_PIN); // Asigna el manejador de interrupciones al pin

    adc1_config_width(ADC_WIDTH_BIT_12); // Configurar resolución del ADC a 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // Configurar atenuación del canal del ADC

    xTaskCreate(calculate_frequency_task, "calculate_frequency_task", 4096, NULL, 10, NULL);
    xTaskCreate(calculate_humidity_task, "calculate_humidity_task", 4096, NULL, 10, NULL);
    xTaskCreate(control_humidity_task, "calculate_humidity_task", 4096, NULL, 10, NULL);
}


