#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_timer.h>
#include <esp_vfs_fat.h>
#include <freertos/ringbuf.h>
#include <esp_timer.h>
#include <esp_http_client.h>
#include <esp_log.h>
#include <esp_vfs.h>
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "../component/DS3231/ds3231.h"
#include "time.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

// Include I2S driver
#include <driver/i2s.h>

#include "esp_http_server.h"

// Incluse SD card driver
#include "../component/FileManager/sdcard.h"


// Incluse MAX30102 driver
#include "max30102.h"

// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

#define bufferCount 6
#define bufferLen 32
#define receiveBufferLen ((bufferLen * 32 / 8)  * bufferCount / 2) 

// Buffers to store data read from dma buffers
int16_t buffer16[receiveBufferLen / 4] = {0};
uint8_t buffer32[receiveBufferLen] = {0};

// Buffer for data to save to SD card
RingbufHandle_t buf_handle_max;
RingbufHandle_t buf_handle_inm;

// Data buffer to send to ringbuffer
static char data_max[400] = "";
static char data_inm[receiveBufferLen / 4 * 6] = ""; // Should not be to big. For some reason, I set its size 1536B and it fails ???

TaskHandle_t readMAXTask_handle = NULL;
TaskHandle_t readINMTask_handle = NULL;
TaskHandle_t saveToSDTask_handle = NULL;
TaskHandle_t controlPillow_handle = NULL;

i2c_dev_t ds3231_device;

// namefile save into sd card
char nameFilePCG[15];
char nameFilePPG[15];

esp_mqtt_client_handle_t client;

// RTC

#define CONFIG_RTC_I2C_PORT 0
#define  CONFIG_RTC_PIN_NUM_SDA 26
#define CONFIG_RTC_PIN_NUM_SCL 27

#define DS3231_ADDR_TIME    0x00
#define DS3231_12HOUR_FLAG  0x40
#define DS3231_12HOUR_MASK  0x1f
#define DS3231_PM_FLAG      0x20
#define DS3231_MONTH_MASK   0x1f

#define SECONDS_PER_MIN             60U
#define SECONDS_PER_HOUR            3600U
#define SECONDS_PER_DAY             86400U
#define SECONDS_PER_MON             2629743U
#define SECONDS_PER_YEAR            31556926U
#define SECONDS_FROM_1970_TO_2023   1672506000U    // Unixtime for 2023-01-01 00:00:00
static const char *timeFormat = "%d:%d:%d,%d-%d-%d";
// %d-%d-%d
static const char *timeFormat2 = "%d-%d-%d";

static const int month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

int currentDay;


#define CHECK_ARG(ARG) do { if (!ARG) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}
esp_err_t ds3231_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = DS3231_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if defined(CONFIG_IDF_TARGET_ESP32)
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    i2c_dev_create_mutex(dev);

    return ESP_OK;
}


esp_err_t ds3231_set_time(i2c_dev_t *dev, struct tm *time)
{
    CHECK_ARG(dev);
    CHECK_ARG(time);

    uint8_t data[7];

    /* time/date data */
    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);
    /* The week data must be in the range 1 to 7, and to keep the start on the
     * same day as for tm_wday have it start at 1 on Sunday. */
    data[3] = dec2bcd(time->tm_wday + 1);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_mon + 1);
    data[6] = dec2bcd(time->tm_year - 2000);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, DS3231_ADDR_TIME, data, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ds3231_initialize(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    esp_err_t error;
    error = ds3231_init_desc(dev, port, sda_gpio, scl_gpio);
    if (error != ESP_OK)
    {
        ESP_LOGE(__func__, "DS3231 module initialize fail.");
        return error;
    }
    ESP_LOGI(__func__, "DS3231 module initialize success.");
    struct tm currentTime;
    ds3231_get_time(dev, &currentTime);
    currentDay = currentTime.tm_yday;
    return error;
}
esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time)
{
    CHECK_ARG(dev);
    CHECK_ARG(time);

    uint8_t data[7];

    /* read time */
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, DS3231_ADDR_TIME, data, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    /* convert to unix time structure */
    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & DS3231_12HOUR_FLAG)
    {
        /* 12H */
        time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
        /* AM/PM? */
        if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
    }
    else time->tm_hour = bcd2dec(data[2]); /* 24H */
    time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
    time->tm_year = bcd2dec(data[6]) + 2000;
    time->tm_isdst = 0;

    // apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
    //applyTZ(time);

    return ESP_OK;
}
esp_err_t ds3231_convertTimeToString(i2c_dev_t *dev, char* timeString, const unsigned int lenghtString)
{
    struct tm time;
    ds3231_get_time(dev, &time);
    memset(timeString, 0, lenghtString);
    int lenght = 0;
    lenght = sprintf(timeString, timeFormat2, time.tm_hour, time.tm_min, time.tm_sec, time.tm_year, time.tm_mon, time.tm_mday);
    if (lenght)
    {
        ESP_LOGI(__func__, "Convert time to string success.");
        return ESP_OK;
    }

    ESP_LOGE(__func__, "Convert time to string fail.");
    return ESP_FAIL;

}

esp_err_t ds3231_setTime(i2c_dev_t *dev, struct tm *time)
{
    return ds3231_set_time(dev, time);
}

esp_err_t ds3231_getEpochTime(i2c_dev_t *dev, uint32_t *epochTime)
{
    struct tm currentTime = {0};
    *epochTime = 0;
    esp_err_t err_code = ESP_OK;
    err_code = ds3231_get_time(dev, &currentTime);
    if (err_code != ESP_OK)
    {
        ESP_LOGE(__func__, "DS3231 get UnixTime fail(%s).", esp_err_to_name(err_code));
        return ESP_FAIL;
    }

    for (size_t i = 0; i < (currentTime.tm_mon - 1); i++)
    {
        currentTime.tm_yday += month[i];
        if (i == 1) currentTime.tm_yday += ((currentTime.tm_year % 4) == 0);
    }
    currentTime.tm_yday += (currentTime.tm_mday - 1);

    *epochTime += SECONDS_FROM_1970_TO_2023;
    *epochTime += ((currentTime.tm_year - 2023) * SECONDS_PER_YEAR);
    *epochTime += (currentTime.tm_yday * SECONDS_PER_DAY);
    *epochTime += (currentTime.tm_hour * SECONDS_PER_HOUR);
    *epochTime += (currentTime.tm_min  * SECONDS_PER_MIN);
    *epochTime += currentTime.tm_sec;
    ESP_LOGI(__func__, "DS3231 get EpochTime success.");
    
    return ESP_OK;
}

bool ds3231_isNewDay(i2c_dev_t *dev)
{
    struct tm currentTime;
    ds3231_get_time(dev, &currentTime);
    return (currentDay == currentTime.tm_yday);
}



// Web Server
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN + 17)

/* Scratch buffer size */
#define SCRATCH_BUFSIZE  8192

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};
static const char base_path[] = MOUNT_POINT;

#define CHUNK_SIZE 1024
esp_err_t index_html_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);  // Response body can be empty
    return ESP_OK;
}

/* Handler to respond with an icon file embedded in flash.
 * Browsers expect to GET website icon at URI /favicon.ico.
 * This can be overridden by uploading file with same name */
esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}

/* Send HTTP response with a run-time generated html consisting of
 * a list of all files and folders under the requested path.
 * In case of SPIFFS this returns empty list when path is any
 * string other than '/', since SPIFFS doesn't support directories */
esp_err_t http_response_dir_html(httpd_req_t *req, const char *dirpath)
{
    char entrypath[FILE_PATH_MAX];
    char entrysize[16];
    const char *entrytype;

    struct dirent *entry;
    struct stat entry_stat;

    DIR *dir = opendir(dirpath);
    const size_t dirpath_len = strlen(dirpath);

    /* Retrieve the base path of file storage to construct the full path */
    strlcpy(entrypath, dirpath, sizeof(entrypath));

    if (!dir) {
        ESP_LOGE(__func__, "Failed to stat dir : %s", dirpath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
        return ESP_FAIL;
    }

    /* Send HTML file header */
    httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><body>");

    /* Get handle to embedded file upload script */
    extern const unsigned char upload_script_start[] asm("_binary_upload_script_html_start");
    extern const unsigned char upload_script_end[]   asm("_binary_upload_script_html_end");
    const size_t upload_script_size = (upload_script_end - upload_script_start);

    /* Add file upload form and script which on execution sends a POST request to /upload */
    httpd_resp_send_chunk(req, (const char *)upload_script_start, upload_script_size);

    /* Send file-list table definition and column labels */
    httpd_resp_sendstr_chunk(req,
        "<table class=\"fixed\" border=\"1\">"
        "<col width=\"800px\" /><col width=\"300px\" /><col width=\"300px\" /><col width=\"100px\" />"
        "<thead><tr><th>Name</th><th>Type</th><th>Size (Bytes)</th><th>Delete</th></tr></thead>"
        "<tbody>");

    /* Iterate over all files / folders and fetch their names and sizes */
    while ((entry = readdir(dir)) != NULL) {
        entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

        strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
        if (stat(entrypath, &entry_stat) == -1) {
            ESP_LOGE(__func__, "Failed to stat %s : %s", entrytype, entry->d_name);
            continue;
        }
        sprintf(entrysize, "%ld", entry_stat.st_size);
        ESP_LOGI(__func__, "Found %s : %s (%s bytes)", entrytype, entry->d_name, entrysize);

        /* Send chunk of HTML file containing table entries with file name and size */
        httpd_resp_sendstr_chunk(req, "<tr><td><a href=\"");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        if (entry->d_type == DT_DIR) {
            httpd_resp_sendstr_chunk(req, "/");
        }
        httpd_resp_sendstr_chunk(req, "\">");
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "</a></td><td>");
        httpd_resp_sendstr_chunk(req, entrytype);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, entrysize);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, "<form method=\"post\" action=\"/delete");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "\"><button type=\"submit\">Delete</button></form>");
        httpd_resp_sendstr_chunk(req, "</td></tr>\n");
    }
    closedir(dir);

    /* Finish the file list table */
    httpd_resp_sendstr_chunk(req, "</tbody></table>");

    /* Send remaining chunk of HTML file to complete it */
    httpd_resp_sendstr_chunk(req, "</body></html>");

    /* Send empty chunk to signal HTTP response completion */
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

/* Set HTTP response content type according to file extension */
esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

/* Handler to download a file kept on the server */
esp_err_t download_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri, sizeof(filepath));
    if (!filename) {
        ESP_LOGE(__func__, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/') {
        return http_response_dir_html(req, filepath);
    }

    if (stat(filepath, &file_stat) == -1) {
        /* If file not present on SPIFFS check if URI
         * corresponds to one of the hardcoded paths */
        if (strcmp(filename, "/index.html") == 0) {
            return index_html_get_handler(req);
        } else if (strcmp(filename, "/favicon.ico") == 0) {
            return favicon_get_handler(req);
        }
        ESP_LOGE(__func__, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(__func__, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(__func__, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(fd);
                ESP_LOGE(__func__, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    ESP_LOGI(__func__, "File sending complete");

    /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Handler to delete a file from the server */
esp_err_t delete_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;
    char pathFile[64];
    
    /* Skip leading "/delete" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri  + sizeof("/delete") - 1, sizeof(filepath));
    if (!filename) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(__func__, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(__func__, "File does not exist : %s", filename);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist");
        return ESP_FAIL;
    }

    ESP_LOGI(__func__, "Deleting file : %s", filename);
    /* Delete file */
    unlink(filepath);

    // 
    // sprintf(pathFile, "%s%s.txt", mount_point, filename);
    // ESP_LOGE(__func__, "File :%s\n", pathFile);
    // FILE *file = fopen(pathFile, "w");
    // if (file == NULL) {
    //     ESP_LOGE(__func__, "Failed to create file");
    //     return 0;
    // }
    // fclose(file);


    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_sendstr(req, "File deleted successfully");
    return ESP_OK;
}

/* Function to start the file server */
esp_err_t start_file_server(const char *base_path)
{
    static struct file_server_data *server_data = NULL;

    if (server_data) {
        ESP_LOGE(__func__, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate memory for server data */
    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data) {
        ESP_LOGE(__func__, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path,
            sizeof(server_data->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(__func__, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(__func__, "Failed to start file server!");
        return ESP_FAIL;
    }

    /* URI handler for getting uploaded files */
    httpd_uri_t file_download = {
        .uri       = "/*",  // Match all URIs of type /path/to/file
        .method    = HTTP_GET,
        .handler   = download_get_handler,
        .user_ctx  = server_data    // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_download);

    /* URI handler for deleting files from server */
    httpd_uri_t file_delete = {
        .uri       = "/delete/*",   // Match all URIs of type /delete/path/to/file
        .method    = HTTP_POST,
        .handler   = delete_post_handler,
        .user_ctx  = server_data    // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_delete);

    return ESP_OK;
}




void publish_message(const char* topic, const char*nameFile) {
    // Tạo đối tượng JSON
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(__func__, "Failed to create JSON object");
        return;
    }

    // Thêm các trường vào JSON
    cJSON_AddStringToObject(root, "namefile", nameFile);
    
   
    // Chuyển đổi đối tượng JSON thành chuỗi
    char *json_string = cJSON_Print(root);
    if (json_string == NULL) {
        ESP_LOGE(__func__, "Failed to print JSON string");
        cJSON_Delete(root);
        return;
    }

    // Publish thông điệp JSON
    int msg_id = esp_mqtt_client_publish(client, topic, json_string, 0, 0, 0);
    ESP_LOGI(__func__, "Sent publish successful, msg_id=%d", msg_id);

    // Giải phóng bộ nhớ
    cJSON_Delete(root);
    free(json_string);
}


/**
 * @brief Read data from MAX30102 and send to ring buffer
 * 
 * @param pvParameters 
 */
void max30102_test(void* parameter)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(max30102_initDesc(&dev, 0, 21, 22));

    struct max30102_record record;
    struct max30102_data data;

    if (max30102_readPartID(&dev) == ESP_OK) {
        ESP_LOGI(__func__, "Found MAX30102!");
    }
    else {
        ESP_LOGE(__func__, "Not found MAX30102");
    }

    if (max30102_init(0x1F, 4, 2, 1000, 118, 4096, &record, &dev) == ESP_OK) {
        ESP_LOGI(__func__, "Init MAX30102 successful");
    }
    else {
        ESP_LOGE(__func__, "Init fail!");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    

    uint16_t samplesTaken = 0;
    char data_temp[16] = "";
    unsigned long red;
    unsigned long ir;
    

    TickType_t startTime = xTaskGetTickCount(); // Get the current tick count
    //ESP_LOGE(__func__, "Start time MAX30102 = %ld", startTime);
    int number_samples = 0;

    struct tm timeTemp = { 0 };
    ds3231_get_time(&ds3231_device, &timeTemp);
    sprintf(nameFilePPG,"%s_%d_%d_%d", "PPG",timeTemp.tm_hour,timeTemp.tm_min,timeTemp.tm_sec);
    while (1)
    {
        number_samples = max30102_check(&record, &dev); //Check the sensor, read up to 3 samples
        while (max30102_available(&record)) //do we have new data?
        {
            int numberOfSamples = record.head - record.tail;

            samplesTaken++;
            red = max30102_getFIFORed(&record);
            ir = max30102_getFIFOIR(&record);
            memset(data_temp, 0, sizeof(data_temp));
            sprintf(data_temp, "%lu,%lu\n", red, ir);
            strcat(data_max, data_temp);
            max30102_nextSample(&record); //We're finished with this sample so move to next sample
        }
        //ESP_LOGI(__func__, "Print data_max = %s", data_max);
        if (samplesTaken >= 25) 
        {
            xRingbufferSend(buf_handle_max, data_max, sizeof(data_max), pdMS_TO_TICKS(5));
            samplesTaken = 0;
            memset(data_max, 0, sizeof(data_max));
        }
        TickType_t currentTime = xTaskGetTickCount(); // Get the current tick count
        TickType_t elapsedTime = currentTime - startTime; // Calculate the elapsed time
        if (elapsedTime >= pdMS_TO_TICKS(15000)) // Check if 10 seconds have passed
        { 
            TickType_t start2 = xTaskGetTickCount(); 
            publish_message("message/nameFilePPG", nameFilePPG);
            memset(nameFilePPG,0,sizeof(nameFilePPG));
            ds3231_get_time(&ds3231_device, &timeTemp);
            sprintf(nameFilePPG,"%s_%d_%d_%d", "PPG",timeTemp.tm_hour,timeTemp.tm_min,timeTemp.tm_sec);
            TickType_t end2 = xTaskGetTickCount(); 
            ESP_LOGI(__func__, "Get new namefile %ld\n", end2 - start2);
            startTime = xTaskGetTickCount();
            ESP_LOGI(__func__, "Get data MAX30102 start file %s\n", nameFilePPG);
            //i++;
            
            //break; // Exit the loop
        }
        //ESP_LOGI(__func__, "End of MAX30102 while loop");
    }
    //ESP_LOGI(__func__, "End of MAX30102 task");
    //vTaskDelete(NULL);
}

int s_retry_num = 0;
int status;  // variable to save status message from MQTT to control pillow
char str[30];  // variable to save message
bool check = false;
static void initialize_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(__func__, "retry to connect to the AP");
        }
        ESP_LOGI(__func__,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(__func__, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        start_file_server(base_path);
        s_retry_num = 0;
    }
}


void WIFI_initSTA(void)
{

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Bug",
            .password = "12345678",
            /* Authmode threshold resets to WPA2 ass default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(__func__, "wifi_init_sta finished.");
}
enum functionControl {
    top,
    left,
    right,
    bottom
};
//                            top// left//right/bottom

int indexControl = -1;
int LOW = 0;

void controlPillow(void* parameter){
    ESP_LOGI(__func__, "Status to control pillow: %d\n", status);
    //gpio_pad_select_gpio(2);
  
    
    //config maybom1 - 2// may2 -4 // may3-16 // thoatkhi1 - 17 / thoatkhi2 - 3 / thoatkhi3 - 1

    while(true){
        if(status == 0){
            break;
        }
        indexControl++;
        indexControl = indexControl % 4;
        //ESP_LOGI(__func__, "Top: %d %d\n", indexControl, top);
        if(indexControl == top){
            ESP_LOGI(__func__, "Status to control top\n");
            gpio_set_level(2, status); // control may bom 1
            gpio_set_level(4, status); // control may bom 2
            gpio_set_level(16, status); // control may bom 3
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            gpio_set_level(2, LOW); // stop may bom 1
            gpio_set_level(4, LOW); // stop may bom 2
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            gpio_set_level(16, LOW); // stop may bom 3
            break;
                
        } 
        if(indexControl == left){
            ESP_LOGI(__func__, "Status to control left\n");
            gpio_set_level(2, status); // bom 1
            gpio_set_level(3, status); // xa van 2
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            gpio_set_level(2, LOW); 
            gpio_set_level(3, LOW); 
            break;
                
        } 
        if(indexControl == right){
            ESP_LOGI(__func__, "Status to control right\n");
            gpio_set_level(17, status); // xa van 1
            gpio_set_level(4, status); // bom 2
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            gpio_set_level(17, LOW); 
            gpio_set_level(4, LOW); 
            break;
                
        } 
        if(indexControl == bottom){
            ESP_LOGI(__func__, "Status to control bottom\n");
            gpio_set_level(3, status); // xa van 2
            gpio_set_level(1, status); // xa van 3
            vTaskDelay(15000 / portTICK_PERIOD_MS);
            gpio_set_level(3, LOW); 
            gpio_set_level(1, LOW); 
            break;
                
        } 
        
    }
    
    // while(left == indexControl && status == 1){
    //     ESP_LOGI(__func__, "Status to control left\n");
    //     TickType_t startTime = xTaskGetTickCount();
    //     gpio_set_level(2, status); //  bom 1
    //     gpio_set_level(8, status); //  open van 2
    //     TickType_t endTime = xTaskGetTickCount();
            
    //     if(endTime - startTime >= 5000){
    //         gpio_set_level(2, 0); // stop may bom 1
    //         gpio_set_level(8, 0); // close van 2
    //         break;
    //     }
    // }
    // while(right == indexControl && status == 1){
    //     ESP_LOGI(__func__, "Status to control right\n");
    //     TickType_t startTime = xTaskGetTickCount();
    //     gpio_set_level(5, status); // open van 1
    //     gpio_set_level(4, status); // bom 2
    //     TickType_t endTime = xTaskGetTickCount();
            
    //     if(endTime - startTime >= 10000){
    //         gpio_set_level(5, 0); // close van 1
    //         gpio_set_level(4, 0); // stop may bom 2
    //         break;
    //     }
    // }
    // while(bottom == indexControl && status == 1){
    //     ESP_LOGI(__func__, "Status to control bottom\n");
    //     TickType_t startTime = xTaskGetTickCount();
    //     gpio_set_level(5, status); // open 1
    //     gpio_set_level(18, status); // open 2
    //     gpio_set_level(19, status); // open 3
    //     TickType_t endTime = xTaskGetTickCount();
            
    //     if(endTime - startTime >= 15000){
    //         gpio_set_level(5, status); // close 1
    //         gpio_set_level(18, status); // close 2
    //         gpio_set_level(19, status); // close 3
    //     }
    
    // }
    
    vTaskDelete(NULL);    
    
}


static void log_error_if_nonzero(const char * message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(__func__, "Last error %s: 0x%x", message, error_code);
    }
}




static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(__func__, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, "pillow/control", 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(__func__, "MQTT_EVENT_DISCONNECTED");
            break;

        // case MQTT_EVENT_SUBSCRIBED:
        //     ESP_LOGI(__func__, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        //     msg_id = esp_mqtt_client_subscribe(client, "pillow/control", 1);
        //     // ESP_LOGI(__func__, "sent publish successful, msg_id=%d", msg_id);
        //     break;
        // case MQTT_EVENT_UNSUBSCRIBED:
        //     ESP_LOGI(__func__, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        //     break;
        // case MQTT_EVENT_PUBLISHED:
        //     ESP_LOGI(__func__, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        //     break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(__func__, "MQTT_EVENT_DATA");
            int index = 0;
            check = true;
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            sprintf(str, "%.*s\r\n", event->data_len, event->data);
            //printf("DATA=%.*s\r\n", event->data_len, event->data);
            while(str[index] != ':'){
                index++;
            }
            status = str[index+1] - '0';
            printf("Status: %s\n", str);
            xTaskCreatePinnedToCore(controlPillow, "controlPillow", 1024 * 5,NULL,15,&controlPillow_handle, 0);
            //controlPillow(status);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(__func__, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(__func__, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
            break;
        default:
            ESP_LOGI(__func__, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}



static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(__func__, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://test.mosquitto.org:1883",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}




// Set up I2S Processor configuration
void i2s_install() {
  // Set up I2S Processor configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 4000, // or 44100 if you like
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Ground the L/R pin on the INMP441.
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S |I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = bufferCount,
    .dma_buf_len = bufferLen,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if(err == ESP_OK){
    ESP_LOGI(__func__, "Initinalize I2S successfull");
  }
  else{
    ESP_LOGE(__func__, "Initinalize I2S fail");
  }
}

//TickType_t startTimeSwitchTask; // variable of time to switch between readINMPTask and sendDataToServer

// Set I2S pin configuration
void i2s_setpin() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    i2s_set_pin(I2S_PORT, &pin_config);
}

/**
 * @brief Read data from INMP441 and send to ring buffer
 * 
 * @param pvParameters 
 */

void readINMP441Task(void* parameter) {
    // Set up I2C
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    size_t bytesRead = 0;
    char data_temp[8] = ""; 
    

    // FILE *file = fopen("/sdcard/pcg.txt", "w");
    // if (file != NULL) {
    //     fprintf(file, "11111");
    //     fclose(file);
    // } else {
    //     printf("Failed to open file for writing.");
    // }
    
    TickType_t startTime = xTaskGetTickCount(); // Get the current tick count
    //ESP_LOGE(__func__, "Start time INMP441 = %ld", startTime);
    int take_time_inm = 0;
    struct tm timeTemp = { 0 };
    ds3231_get_time(&ds3231_device, &timeTemp);
    ESP_LOGI(__func__, "Time %.2d:%.2d:%.2d|%.2d|%.2d|%d.\n", timeTemp.tm_hour,
                                                                timeTemp.tm_min,
                                                                timeTemp.tm_sec,
                                                                timeTemp.tm_mday,
                                                                timeTemp.tm_mon,
                                                                timeTemp.tm_year);
    sprintf(nameFilePCG,"%s_%d_%d_%d", "PCG",timeTemp.tm_hour,timeTemp.tm_min,timeTemp.tm_sec);
    
    while (1)
    {
        vTaskDelay(1); // Feed for watchdog, if not watchdog timer will be triggered!
        //int32_t start_time = esp_timer_get_time();
        i2s_read(I2S_PORT, &buffer32, (sizeof(buffer32)), &bytesRead, 100);
        //ESP_LOGW(__func__,"time i2s_read = %lld, byteRead = %d", esp_timer_get_time() - start_time, bytesRead);
        int samplesRead = bytesRead / 4;
        // if(take_time_inm == 0 || take_time_inm == 1 || take_time_inm == 2)
        // {
        //     ESP_LOGI(__func__, "Start taking data from INMP441 = %lld and number sample read = %d", esp_timer_get_time(), samplesRead);
        //     take_time_inm += 1;
        // }
        for (uint8_t i = 0; i < samplesRead; i++) {
            uint8_t mid = buffer32[i * 4 + 2];
            uint8_t msb = buffer32[i * 4 + 3];
            uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
            memcpy(&buffer16[i], &raw, sizeof(raw)); // Copy so sign bits aren't interfered with somehow.
            // printf("%d %d %d\n", 3000, -3000, buffer16[i]);
            
            memset(data_temp, 0, sizeof(data_temp));
            sprintf(data_temp, "%d\n", buffer16[i]);
            strcat(data_inm, data_temp);
            
        }
        bool res = pdFALSE;
        while (res != pdTRUE)
        {
            res = xRingbufferSend(buf_handle_inm, data_inm, sizeof(data_inm), pdMS_TO_TICKS(10));
        }
        memset(data_inm, 0, sizeof(data_inm));
        
        TickType_t currentTime = xTaskGetTickCount(); // Get the current tick count
        TickType_t elapsedTime = currentTime - startTime; // Calculate the elapsed time
        if (elapsedTime >= pdMS_TO_TICKS(15000)) { // Check if 10 seconds have passed
            
            
            publish_message("message/nameFilePCG", nameFilePCG);
            memset(nameFilePCG,0,sizeof(nameFilePCG));
            ds3231_get_time(&ds3231_device, &timeTemp);
            sprintf(nameFilePCG,"%s_%d_%d_%d", "PCG",timeTemp.tm_hour,timeTemp.tm_min,timeTemp.tm_sec);
            startTime = xTaskGetTickCount();

            ESP_LOGI(__func__, "Get data INMP start of file %s\n", nameFilePCG);
            
            // j++;
            //break; // Exit the loop
        }
    }
    // ESP_LOGI(__func__, "End of INMP441 task");
    // vTaskDelete(NULL); // Delete the task
}

/**
 * @brief Receive data from 2 ring buffers and save them to SD card
 * 
 * @param parameter 
 */
void saveINMPAndMAXToSDTask(void *parameter) {
    while(1) {
        size_t item_size1;
        size_t item_size2;
        //Receive an item from no-split INMP441 ring buffer
        char *item1 = (char *)xRingbufferReceive(buf_handle_inm, &item_size1, 1);
        
        if (item1 != NULL) {
            //Return Item
            // Serial.println("r");
            vRingbufferReturnItem(buf_handle_inm, (void *)item1);
            //TickType_t startTime = xTaskGetTickCount(); // Get the current tick count
            sdcard_writeDataToFile_noArgument(nameFilePCG, item1);
            //TickType_t currentTime = xTaskGetTickCount(); // Get the current tick count
            //ESP_LOGE(__func__,"Time save data: %ld\n", currentTime - startTime);
        } 

        //Receive an item from no-split MAX30102 ring buffer
        char *item2 = (char *)xRingbufferReceive(buf_handle_max, &item_size2, 1);
        
        if (item2 != NULL) {
        //     //Return Item
        //     // Serial.println("rev");
            vRingbufferReturnItem(buf_handle_max, (void *)item2);
            sdcard_writeDataToFile_noArgument(nameFilePPG, item2);
        } 
    }
}
void sntp_init_func()
{
    ESP_LOGI(__func__, "Initializing SNTP.");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_init();
}

esp_err_t sntp_setTime(struct tm *timeInfo, time_t *timeNow)
{
    for (size_t i = 0; (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET); i++)
    {
        ESP_LOGI(__func__, "Waiting for system time to be set...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    time(timeNow);
    localtime_r(timeNow, timeInfo);

    char timeString[64];

    // Set timezone to VietNam Standard Time
    setenv("TZ", "GMT-07", 1);
    tzset();
    localtime_r(timeNow, timeInfo);
    strftime(timeString, sizeof(timeString), "%c", timeInfo);
    ESP_LOGI(__func__, "The current date/time in Viet Nam is: %s ", timeString);
    return ESP_OK;
}

void app_main(void)
{
    // Initialize SPI Bus
    
    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t mount_config_t = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t host_t = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));

    // Initialise ring buffers
    buf_handle_max = xRingbufferCreate(1028 * 6, RINGBUF_TYPE_NOSPLIT);
    buf_handle_inm = xRingbufferCreate(1028 * 15, RINGBUF_TYPE_NOSPLIT);

    if(buf_handle_inm == NULL) 
    {
        ESP_LOGE(__func__, "Ring buffers create fail");
    }
    else
    {
        ESP_LOGI(__func__, "Ring buffers create OK");
    }

    ESP_LOGI(__func__, "Initialize nvs partition.");
    initialize_nvs();
    // Wait 1 second for memory initialization
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    WIFI_initSTA();
    
    // initinal mqtt
    mqtt_app_start();

    ESP_LOGI(__func__, "Set up output control pillow\n");
    gpio_set_direction(2, GPIO_MODE_OUTPUT);    // bom 1
    gpio_set_direction(4, GPIO_MODE_OUTPUT);    // bom 2
    gpio_set_direction(16, GPIO_MODE_OUTPUT);   // bom 3

    gpio_set_direction(17, GPIO_MODE_OUTPUT);    // xa 1
    gpio_set_direction(3, GPIO_MODE_OUTPUT);    // xa 2
    gpio_set_direction(1, GPIO_MODE_OUTPUT);   // xa 3
    
    // update time use sntp
    time_t timeNow = 0;
    struct tm timeInfo = { 0 };
    sntp_init_func();
    sntp_setTime(&timeInfo, & timeNow);
    mktime(&timeInfo);

    ESP_LOGI(__func__, "Initialize DS3231 module(I2C/Wire%d).", CONFIG_RTC_I2C_PORT);
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    memset(&ds3231_device, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_initialize(&ds3231_device, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
    timeInfo.tm_sec += 2;
    timeInfo.tm_mon += 1;
    timeInfo.tm_year = 2024;

    ds3231_setTime(&ds3231_device, &timeInfo);

    // Create tasks
    xTaskCreatePinnedToCore(max30102_test, "max30102_test", 1024 * 5,NULL,6, &readMAXTask_handle, 0);
    xTaskCreatePinnedToCore(readINMP441Task, "readINM411", 1024 * 15, NULL, 10, &readINMTask_handle, 0);  // ?? Make max30102 task and inm task have equal priority can make polling cycle of max3012 shorter ??
    xTaskCreatePinnedToCore(saveINMPAndMAXToSDTask, "saveToSD", 1024 * 10,NULL,  10, &saveToSDTask_handle, 1);


    //xTaskCreatePinnedToCore(sendDataToServer, "sendDataToServer", 1024 * 10,NULL,  10, &sendDataToServer_handle, 0);
    //xTaskCreatePinnedToCore(listenFromMQTT, "listenFromMQTT", 1024 * 3,NULL,  5, &listenFromMQTT_handle, 0);
}
