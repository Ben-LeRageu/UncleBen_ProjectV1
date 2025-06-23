#include "http_server.h"
#include "air_fryer.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"

static const char *HTTP_TAG = "HTTP";

static esp_err_t status_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "Temp", fryer.temp);
    cJSON_AddStringToObject(root, "Time", fryer.time_display);
    cJSON_AddStringToObject(root, "State", fryer.state);
    cJSON_AddStringToObject(root, "Mode", fryer.mode);

    const char *json_str = cJSON_PrintUnformatted(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    cJSON_Delete(root);
    free((void *)json_str);

    return ESP_OK;
}

static esp_err_t update_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);  // pour avoir de la place pour '\0'
    if (ret <= 0)
    {
      ESP_LOGW("HTTP", "Échec de réception POST /update : %d", ret);
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erreur réception");
      return ESP_FAIL;
    }

    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) return ESP_FAIL;

    cJSON *temp_json = cJSON_GetObjectItem(json, "temp");
    cJSON *time_json = cJSON_GetObjectItem(json, "time");

    if (cJSON_IsNumber(temp_json)) {
        fryer.temp = temp_json->valueint;
        //display_temperature(fryer.temp);
    }

    if (cJSON_IsNumber(time_json)) {
        fryer.total_minutes = time_json->valueint;
        //update_time_display(&fryer);
    }

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    char response[64];
    snprintf(response, sizeof(response), "ESP32 AirFryer: temp = %d°F", fryer.temp);
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

esp_err_t configure_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;

    buf[ret] = '\0';
    ESP_LOGI("CONFIGURE", "Données reçues : %s", buf);

    // Ici, tu peux parser le JSON avec cJSON
    // Pour extraire ssid / password et les stocker ou les utiliser

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t update_uri = {
            .uri       = "/update",
            .method    = HTTP_POST,
            .handler   = update_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &update_uri);

        httpd_uri_t configure_uri = {
        .uri       = "/configure",
        .method    = HTTP_POST,
        .handler   = configure_handler,
        .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &configure_uri);

        ESP_LOGI("HTTP", "Serveur HTTP démarré avec /, /status et /update");
    }
    else
    {
        ESP_LOGE("HTTP", "Échec du démarrage du serveur HTTP");
    }
}
