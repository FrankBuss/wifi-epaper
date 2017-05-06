/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "epd.h"

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "192.168.11.27"
#define WEB_PORT 80
#define WEB_URL "/image.bin"

static const char *TAG = "wifi-epaper";

static const char *REQUEST = "GET " WEB_URL " HTTP/1.0\r\n"
                             "Host: "WEB_SERVER"\r\n"
                             "User-Agent: wifi-epaper/1.0 esp32\r\n"
                             "Connection: close\r\n"
                             "\r\n";

#define IMAGE_SIZE (400*300/8*2)
static uint8_t image[IMAGE_SIZE];
static char recv_buf[2048];

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
		case SYSTEM_EVENT_STA_START:
			esp_wifi_connect();
			break;
		case SYSTEM_EVENT_STA_GOT_IP:
			xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			/* This is a workaround as ESP32 WiFi libs don't currently
			   auto-reassociate. */
			esp_wifi_connect();
			xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
			break;
		default:
			break;
	}
	return ESP_OK;
}

static void initialise_wifi(void)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_WIFI_SSID,
			.password = EXAMPLE_WIFI_PASS,
		},
	};
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
	ESP_ERROR_CHECK( esp_wifi_start() );
}

static void http_get_task(void *pvParameters)
{
	const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct addrinfo *res;
	struct in_addr *addr;
	int s, r;

	while(1) {
		/* Wait for the callback to set the CONNECTED_BIT in the
		   event group.
		*/
		xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
		                    false, true, portMAX_DELAY);
		ESP_LOGI(TAG, "Connected to AP");

		int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

		if(err != 0 || res == NULL) {
			ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}

		/* Code to print the resolved IP.

		   Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
		addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
		ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

		s = socket(res->ai_family, res->ai_socktype, 0);
		if(s < 0) {
			ESP_LOGE(TAG, "... Failed to allocate socket.");
			freeaddrinfo(res);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG, "... allocated socket\r\n");

		if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
			ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
			close(s);
			freeaddrinfo(res);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}

		ESP_LOGI(TAG, "... connected");
		freeaddrinfo(res);

		if (write(s, REQUEST, strlen(REQUEST)) < 0) {
			ESP_LOGE(TAG, "... socket send failed");
			close(s);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG, "... socket send success");

		/* Read HTTP response */
		int pos = 0;
		do {
			r = read(s, recv_buf, sizeof(recv_buf)-1);
			if (r > 0) {
				ESP_LOGI(TAG, "%d bytes read\r\n", r);
				if (pos + r > IMAGE_SIZE) {
					int delta = pos + r - IMAGE_SIZE;
					memmove(image, image + delta, IMAGE_SIZE - delta);
					pos = IMAGE_SIZE - r;
				}
				memcpy(image + pos, recv_buf, r);
				pos += r;
			}
		} while(r > 0);
		ESP_LOGI(TAG, "bytes in buffer: %d bytes\r\n", pos);

		ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
		close(s);

		epdShowImage(image);

		ESP_LOGI(TAG, "sleeping now");
		esp_deep_sleep_start();
		/*
		    for(int countdown = 10; countdown >= 0; countdown--) {
		        ESP_LOGI(TAG, "%d... ", countdown);
		        vTaskDelay(1000 / portTICK_PERIOD_MS);
		    }
		    ESP_LOGI(TAG, "Starting again!");*/
	}
}

void app_main()
{
	ESP_ERROR_CHECK( nvs_flash_init() );
	initialise_wifi();
	epdInitHardware();
	xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
}
