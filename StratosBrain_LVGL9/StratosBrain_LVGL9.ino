/*
 * Waveshare ESP32-S3-Touch-AMOLED-1.64
 * CO5300 + FT3168 + LVGL 9
 *
 * Current goals:
 * - Keep the display path stable using the official Arduino_GFX offsets
 * - Add a simple FT3168 touch driver over Wire
 * - Create a menu-based UI to move the project forward
 */

#define STRATOSBRAIN_BUILD_WEBCONFIG 0

#if STRATOSBRAIN_BUILD_WEBCONFIG == 0

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <NetworkServer.h>
#include <NetworkClient.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Arduino_GFX_Library.h>
#include <bme68xLibrary.h>
#include <esp32-hal-rgb-led.h>
#include <esp_heap_caps.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <lvgl.h>
#include <WiFi.h>
#include <Preferences.h>

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// Forward declarations for functions used early (types not yet known)
struct ImuState;
struct BlackboxState;
struct Bme688State;
struct GpsState;
struct LoraState;
enum AppScreenId : uint8_t;
enum OrientationMode : uint8_t;
static bool ensureEfisCanvasBuffer();
static void drawEfisHorizon(const ImuState &imu);
static void refreshSensorCaches();
static void sendHttpTextDownload(NetworkClient &client, const char *filename, const char *content);
static void sendHttpActiveLogDownload(NetworkClient &client);
static void efisSetLevelEventCb(lv_event_t *e);
static void applyEfisViewMode();
static void efisToggleViewEventCb(lv_event_t *e);
// NOTE: loadScreenById(AppScreenId) declared after AppScreenId enum below
static void saveWifiPrefs();
static void loadWifiPrefs();

static constexpr int LCD_CS = 9;
static constexpr int LCD_CLK = 10;
static constexpr int LCD_D0 = 11;
static constexpr int LCD_D1 = 12;
static constexpr int LCD_D2 = 13;
static constexpr int LCD_D3 = 14;
static constexpr int LCD_RST = 21;

static constexpr int I2C_SDA = 47;
static constexpr int I2C_SCL = 48;
static constexpr int BOOT_BUTTON_PIN = 0;
static constexpr int RGB_LED_PIN = 45;
// GP10 breakout wiring used by this project:
// GP10 TXD -> ESP32 GPIO1 (RX)
// GP10 RXD -> ESP32 GPIO2 (TX)
static constexpr int GPS_UART_TX_PIN = 2;
static constexpr int GPS_UART_RX_PIN = 1;
static constexpr uint32_t GPS_UART_BAUD = 9600UL;
static constexpr int LORA_UART_TX_PIN = 17;
static constexpr int LORA_UART_RX_PIN = 18;
static constexpr int LORA_AUX_PIN = 6;
static constexpr int LORA_M0_PIN = 7;
static constexpr int LORA_M1_PIN = 8;
static constexpr uint32_t LORA_UART_BAUD = 9600UL;

static constexpr int SDCARD_CS_PIN = 38;
static constexpr int SDCARD_MOSI_PIN = 39;
static constexpr int SDCARD_MISO_PIN = 40;
static constexpr int SDCARD_SCK_PIN = 41;

static constexpr int LCD_W = 280;
static constexpr int LCD_H = 456;

static constexpr int LCD_COL_OFFSET_1 = 20;
static constexpr int LCD_ROW_OFFSET_1 = 0;
static constexpr int LCD_COL_OFFSET_2 = 180;
static constexpr int LCD_ROW_OFFSET_2 = 24;

static constexpr uint8_t TOUCH_ADDR = 0x38;
static constexpr uint8_t TOUCH_REG_MODE = 0x00;
static constexpr uint8_t TOUCH_REG_POINTS = 0x02;
static constexpr uint8_t TOUCH_REG_COORDS = 0x03;
static constexpr uint8_t TOUCH_REG_CHIP_ID = 0xA3;

static constexpr bool TOUCH_SWAP_XY = false;
static constexpr bool TOUCH_INVERT_X = false;
static constexpr bool TOUCH_INVERT_Y = false;

static constexpr uint8_t QMI8658_ADDR_PRIMARY = 0x6B;
static constexpr uint8_t QMI8658_ADDR_SECONDARY = 0x6A;
static constexpr uint8_t QMI8658_REG_WHO_AM_I = 0x00;
static constexpr uint8_t QMI8658_REG_CTRL1 = 0x02;
static constexpr uint8_t QMI8658_REG_CTRL2 = 0x03;
static constexpr uint8_t QMI8658_REG_CTRL3 = 0x04;
static constexpr uint8_t QMI8658_REG_CTRL5 = 0x06;
static constexpr uint8_t QMI8658_REG_CTRL7 = 0x08;
static constexpr uint8_t QMI8658_REG_CTRL8 = 0x09;
static constexpr uint8_t QMI8658_REG_TEMP_L = 0x33;
static constexpr uint8_t QMI8658_REG_AX_L = 0x35;

static constexpr uint8_t QMI8658_CHIP_ID = 0x05;
static constexpr float QMI8658_ACC_LSB_8G = 4096.0f;
static constexpr float QMI8658_GYRO_LSB_1024DPS = 32.0f;
static constexpr float GRAVITY_MS2 = 9.80665f;
static constexpr float IMU_COMPLEMENTARY_ALPHA = 0.985f;
static constexpr float IMU_PIXELS_PER_DEG = 2.3f;
static constexpr float EFIS_DISPLAY_BLEND = 0.18f;
static constexpr uint32_t IMU_TASK_PERIOD_MS = 10;
static constexpr uint32_t IMU_RETRY_PERIOD_MS = 1200;
static constexpr uint16_t EFIS_HORIZON_W = 220;
static constexpr uint16_t EFIS_HORIZON_H = 220;
static constexpr size_t EFIS_HORIZON_PIXELS = EFIS_HORIZON_W * EFIS_HORIZON_H;
static constexpr float DEG_TO_RAD_F = 0.01745329252f;
static constexpr float RAD_TO_DEG_F = 57.2957795f;
static constexpr uint8_t BME688_ADDR_PRIMARY = 0x77;
static constexpr uint8_t BME688_ADDR_SECONDARY = 0x76;
static constexpr uint16_t BME688_HEATER_TEMP_C = 300;
static constexpr uint16_t BME688_HEATER_TIME_MS = 100;
static constexpr uint32_t BME688_POLL_PERIOD_MS = 2000U;
static constexpr uint32_t GPS_STALE_MS = 4000U;
static constexpr uint32_t LORA_STALE_MS = 4000U;
static constexpr float STANDARD_SEA_LEVEL_HPA = 1013.25f;

static constexpr bool IMU_SWAP_XY = false;
static constexpr bool IMU_INVERT_X = false;
static constexpr bool IMU_INVERT_Y = false;
static constexpr bool IMU_INVERT_Z = false;
static constexpr bool IMU_INVERT_PITCH_SIGN = false;
static constexpr bool IMU_INVERT_ROLL_SIGN = true;

enum MeteoThemeMode : uint8_t
{
  METEO_THEME_OPEN = 0,
  METEO_THEME_CLOUDY = 1,
  METEO_THEME_RAIN = 2,
  METEO_THEME_WAITING = 3
};

static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xFFFF;
static constexpr uint16_t COLOR_RED = 0xF800;
static constexpr uint16_t COLOR_GREEN = 0x07E0;
static constexpr uint16_t COLOR_BLUE = 0x001F;
static constexpr uint16_t COLOR_CYAN = 0x07FF;
static constexpr uint16_t COLOR_YELLOW = 0xFFE0;

static constexpr uint32_t LV_TICK_PERIOD_US = 2000;
static constexpr uint16_t LV_BUF_LINES = 24;
static constexpr size_t LV_BUF_PIXELS = LCD_W * LV_BUF_LINES;
static constexpr uint32_t SD_SPI_FREQUENCY = 10000000UL;
static constexpr uint32_t BLACKBOX_LOG_PERIOD_MS = 30000U;
static constexpr uint32_t BLACKBOX_MOUNT_RETRY_MS = 3000U;
static constexpr uint32_t BLACKBOX_FLUSH_PERIOD_MS = 5000U;
static constexpr uint32_t SD_PURGE_CONFIRM_MS = 8000U;
static constexpr uint8_t BLACKBOX_TAIL_LINE_COUNT = 5;
static constexpr size_t BLACKBOX_TAIL_LINE_LEN = 220;
static constexpr uint8_t ACTIVITY_HISTORY_LINE_COUNT = 12;
static constexpr size_t ACTIVITY_HISTORY_LINE_LEN = 140;
static constexpr uint8_t GPS_ACTIVITY_LINE_COUNT = 10;
static constexpr size_t GPS_ACTIVITY_LINE_LEN = 140;
static constexpr uint8_t LORA_HISTORY_LINE_COUNT = 16;
static constexpr size_t LORA_HISTORY_LINE_LEN = 140;
static constexpr uint32_t BOOT_BUTTON_GUARD_MS = 2500U;
static constexpr uint32_t BOOT_BUTTON_DEBOUNCE_MS = 40U;
static constexpr uint32_t BOOT_BUTTON_SHORT_PRESS_MS = 700U;
static constexpr bool ENABLE_DISPLAY_SMOKE_TEST = false;
static constexpr size_t BLACKBOX_PATH_LEN = 48;
static constexpr size_t BLACKBOX_NOTE_LEN = 96;
static constexpr char WIFI_AP_SSID[] = "StratosBrain-S3";
static constexpr char WIFI_AP_PASSWORD[] = "stratos123";
static constexpr uint8_t WIFI_AP_CHANNEL = 6;
static constexpr bool WIFI_AP_HIDDEN = false;
static constexpr uint8_t WIFI_AP_MAX_CONNECTIONS = 4;
static constexpr uint32_t WIFI_STATUS_SERIAL_MS = 15000U;
static constexpr bool ENABLE_AUTO_ROTATION_UI = false;
static constexpr uint16_t WIFI_WEB_PORT = 80;
static constexpr uint32_t WIFI_CLIENT_TIMEOUT_MS = 350U;
static constexpr size_t WIFI_STA_SSID_LEN = 33;
static constexpr size_t WIFI_STA_PASSWORD_LEN = 65;
static constexpr char WIFI_STA_DEFAULT_SSID[] = "";
static constexpr char WIFI_STA_DEFAULT_PASSWORD[] = "";

static Arduino_DataBus *g_bus = nullptr;
static Arduino_CO5300 *g_panel = nullptr;

static lv_display_t *g_lv_display = nullptr;
static lv_indev_t *g_touch_indev = nullptr;
static esp_timer_handle_t g_lv_tick_timer = nullptr;
static lv_color_t *g_buf1 = nullptr;
static lv_color_t *g_buf2 = nullptr;
static lv_color_t *g_efis_canvas_buf = nullptr;

static lv_obj_t *g_screen_main = nullptr;
static lv_obj_t *g_screen_efis = nullptr;
static lv_obj_t *g_screen_meteo = nullptr;
static lv_obj_t *g_screen_gps = nullptr;
static lv_obj_t *g_screen_lora = nullptr;
static lv_obj_t *g_screen_config = nullptr;

enum AppScreenId : uint8_t
{
  SCREEN_HOME = 0,
  SCREEN_EFIS,
  SCREEN_METEO,
  SCREEN_GPS,
  SCREEN_LORA,
  SCREEN_CONFIG
};

// Forward declaration requiring AppScreenId (enum defined above)
static void loadScreenById(AppScreenId screen_id);

enum OrientationMode : uint8_t
{
  ORIENTATION_MODE_PORTRAIT = 0,
  ORIENTATION_MODE_LANDSCAPE,
  ORIENTATION_MODE_AUTO
};

static lv_obj_t *g_lbl_uptime = nullptr;
static lv_obj_t *g_lbl_i2c = nullptr;
static lv_obj_t *g_lbl_touch = nullptr;
static lv_obj_t *g_touch_dot = nullptr;
static lv_obj_t *g_efis_horizon = nullptr;
static lv_obj_t *g_efis_info_panel = nullptr;
static lv_obj_t *g_efis_horizon_panel = nullptr;
static lv_obj_t *g_lbl_efis_pitch = nullptr;
static lv_obj_t *g_lbl_efis_roll = nullptr;
static lv_obj_t *g_lbl_efis_status = nullptr;
static lv_obj_t *g_lbl_efis_calibration = nullptr;
static lv_obj_t *g_lbl_efis_toggle = nullptr;
static lv_obj_t *g_lbl_plane_altitude = nullptr;
static lv_obj_t *g_lbl_plane_vario = nullptr;
static lv_obj_t *g_lbl_plane_heading = nullptr;
static lv_obj_t *g_lbl_plane_speed = nullptr;
static lv_obj_t *g_lbl_plane_skydive = nullptr;
static lv_obj_t *g_lbl_efis_horizon_alt = nullptr;
static lv_obj_t *g_lbl_efis_horizon_speed = nullptr;
static lv_obj_t *g_meteo_hero = nullptr;
static lv_obj_t *g_meteo_panel = nullptr;
static lv_obj_t *g_lbl_meteo_theme = nullptr;
static lv_obj_t *g_lbl_meteo_status = nullptr;
static lv_obj_t *g_lbl_meteo_cards = nullptr;
static lv_obj_t *g_lbl_meteo_altitude = nullptr;
static lv_obj_t *g_lbl_meteo_pressure = nullptr;
static lv_obj_t *g_lbl_meteo_temperature = nullptr;
static lv_obj_t *g_lbl_meteo_humidity = nullptr;
static lv_obj_t *g_lbl_comms_status = nullptr;
static lv_obj_t *g_lbl_comms_wifi = nullptr;
static lv_obj_t *g_lbl_comms_ble = nullptr;
static lv_obj_t *g_lbl_comms_lora = nullptr;
static lv_obj_t *g_lbl_comms_gps = nullptr;
static lv_obj_t *g_lbl_comms_mode = nullptr;
static lv_obj_t *g_lbl_comms_wifi_btn = nullptr;
static lv_obj_t *g_lbl_comms_sta_btn = nullptr;
static lv_obj_t *g_lbl_comms_lora_btn = nullptr;
static lv_obj_t *g_lbl_comms_rescan_btn = nullptr;
static lv_obj_t *g_lbl_lora_status = nullptr;
static lv_obj_t *g_lbl_lora_radio = nullptr;
static lv_obj_t *g_lbl_lora_test = nullptr;
static lv_obj_t *g_lbl_lora_toggle_btn = nullptr;
static lv_obj_t *g_lbl_home_hint = nullptr;
static lv_obj_t *g_lbl_config_orientation = nullptr;
static lv_obj_t *g_lbl_config_rotation = nullptr;
static lv_obj_t *g_lbl_config_net = nullptr;
static lv_obj_t *g_lbl_config_sensors = nullptr;
static lv_obj_t *g_lbl_config_blackbox = nullptr;
static lv_obj_t *g_lbl_config_note = nullptr;
static lv_obj_t *g_lbl_config_scan_raw = nullptr;
static lv_obj_t *g_lbl_config_refresh = nullptr;
static lv_obj_t *g_lbl_config_blackbox_btn = nullptr;
static lv_obj_t *g_lbl_config_format_btn = nullptr;
static lv_obj_t *g_lbl_config_wifi_btn = nullptr;
static lv_obj_t *g_lbl_config_lora_btn = nullptr;
static lv_obj_t *g_lbl_config_brightness = nullptr;
static lv_obj_t *g_lbl_config_cdc = nullptr;

static uint16_t g_touch_x = 0;
static uint16_t g_touch_y = 0;
static bool g_touch_pressed = false;
static bool g_touch_prev_pressed = false;
static uint32_t g_touch_press_count = 0;

static SemaphoreHandle_t g_i2c_mutex = nullptr;
static TaskHandle_t g_imu_task_handle = nullptr;
static TaskHandle_t g_blackbox_task_handle = nullptr;
static uint8_t g_qmi_addr = QMI8658_ADDR_PRIMARY;
static AppScreenId g_current_screen_id = SCREEN_HOME;
static OrientationMode g_orientation_mode = ORIENTATION_MODE_PORTRAIT;
static lv_display_rotation_t g_display_rotation = LV_DISPLAY_ROTATION_0;
static lv_display_rotation_t g_auto_candidate_rotation = LV_DISPLAY_ROTATION_0;
static uint32_t g_auto_candidate_since_ms = 0;
static bool g_rotation_change_pending = false;
static lv_display_rotation_t g_pending_rotation = LV_DISPLAY_ROTATION_0;
static AppScreenId g_pending_screen_id = SCREEN_HOME;
static String g_sensor_summary_cache;
static String g_sensor_raw_scan_cache;
static String g_sensor_compact_cache;
static String g_sensor_scan_compact_cache;
static String g_sensor_bosch_cache;
static String g_sensor_route_cache;
static String g_wifi_scan_summary = "Scan Wi-Fi ainda nao executado.";
static uint16_t g_efis_horizon_w = EFIS_HORIZON_W;
static uint16_t g_efis_horizon_h = EFIS_HORIZON_H;
static float g_pitch_trim_deg = 0.0f;
static float g_roll_trim_deg = 0.0f;
static volatile bool g_blackbox_enabled_target = true;
static volatile bool g_blackbox_remount_requested = false;
static volatile bool g_sd_purge_requested = false;
static bool g_blackbox_resume_after_wifi = false;
static bool g_sd_purge_armed = false;
static uint32_t g_sd_purge_arm_ms = 0;
static SPIClass g_sd_spi(HSPI);
static File g_blackbox_file;
static IPAddress g_wifi_ap_ip(0, 0, 0, 0);
static IPAddress g_wifi_sta_ip(0, 0, 0, 0);
static esp_netif_t *g_wifi_ap_netif = nullptr;
static esp_netif_t *g_wifi_sta_netif = nullptr;
static NetworkServer g_web_server(WIFI_WEB_PORT);
static bool g_wifi_stack_ready = false;
static bool g_wifi_ap_started = false;
static bool g_wifi_sta_started = false;
static bool g_wifi_sta_connected = false;
static bool g_wifi_web_started = false;
static bool g_wifi_events_registered = false;
static uint32_t g_wifi_web_hits = 0;
static uint32_t g_wifi_last_client_ms = 0;
static uint32_t g_wifi_last_status_ms = 0;
static uint32_t g_wifi_recover_request_ms = 0;
static bool g_wifi_mode_requested = false;
static bool g_wifi_ap_requested = false;
static bool g_wifi_sta_requested = false;
static bool g_wifi_apply_pending = false;
static bool g_lora_enabled = false;
static bool g_lora_uart_ready = false;
static bool g_gps_uart_ready = false;
static bool g_lora_auto_tx_enabled = false;
static uint32_t g_lora_auto_tx_interval_ms = 30000UL;
static uint32_t g_lora_auto_tx_last_ms = 0;
static char g_lora_auto_tx_mode[24] = "summary";
static uint32_t g_plane_last_update_ms = 0;
static float g_plane_smooth_alt = 0.0f;
static float g_plane_smooth_vs = 0.0f;
static float g_plane_smooth_spd = 0.0f;
static bool g_runtime_services_light = false;
static bool g_efis_focus_horizon = false;
static char g_wifi_diag_note[BLACKBOX_NOTE_LEN] = "AP desligado. Ligue em COMMS ou CONFIG.";
static char g_wifi_sta_ssid[WIFI_STA_SSID_LEN] = "";
static char g_wifi_sta_password[WIFI_STA_PASSWORD_LEN] = "";
static char g_wifi_sta_note[BLACKBOX_NOTE_LEN] = "LAN desligada";
static uint8_t g_wifi_scan_count = 0;
static uint32_t g_wifi_scan_ms = 0;
static char g_blackbox_tail_lines[BLACKBOX_TAIL_LINE_COUNT][BLACKBOX_TAIL_LINE_LEN] = {};
static uint8_t g_blackbox_tail_count = 0;
static uint8_t g_blackbox_tail_head = 0;
static char g_activity_history_lines[ACTIVITY_HISTORY_LINE_COUNT][ACTIVITY_HISTORY_LINE_LEN] = {};
static uint8_t g_activity_history_count = 0;
static uint8_t g_activity_history_head = 0;
static char g_gps_activity_lines[GPS_ACTIVITY_LINE_COUNT][GPS_ACTIVITY_LINE_LEN] = {};
static uint8_t g_gps_activity_count = 0;
static uint8_t g_gps_activity_head = 0;
static char g_lora_history_lines[LORA_HISTORY_LINE_COUNT][LORA_HISTORY_LINE_LEN] = {};
static uint8_t g_lora_history_count = 0;
static uint8_t g_lora_history_head = 0;
static float g_efis_display_pitch_deg = 0.0f;
static float g_efis_display_roll_deg = 0.0f;
static bool g_efis_display_seeded = false;
static uint8_t g_display_brightness = 255;
static bool g_boot_button_last_raw_pressed = false;
static bool g_boot_button_stable_pressed = false;
static uint32_t g_boot_button_last_change_ms = 0;
static uint32_t g_boot_button_press_ms = 0;
static char g_gps_line_buffer[128] = {};
static size_t g_gps_line_len = 0;
static char g_lora_line_buffer[128] = {};
static size_t g_lora_line_len = 0;

struct HttpJsonScratch
{
  char ip_text[20];
  char ap_ip_text[20];
  char sta_ip_text[20];
  char wifi_scan_summary[240];
  char blackbox_file[80];
  char blackbox_name[40];
  char blackbox_note[80];
  char blackbox_tail[1200];
  char blackbox_files[1400];
  char wifi_note[96];
  char wifi_sta_note[96];
  char wifi_sta_ssid[96];
  char bme_note[96];
  char meteo_summary[160];
  char meteo_theme[64];
  char sensor_summary[220];
  char sensor_known[220];
  char sensor_scan_raw[220];
  char sensor_bosch[240];
  char sensor_routes[220];
  char gps_note[96];
  char gps_sentence[112];
  char gps_config_note[96];
  char gps_last_command[48];
  char gps_map_osm[220];
  char gps_map_google[220];
  char gps_map_hint[160];
  char gps_power_hint[120];
  char gps_wiring_hint[160];
  char gps_history[1500];
  char lora_note[96];
  char lora_message[112];
  char lora_last_rx[112];
  char lora_last_tx[112];
  char lora_auto_mode[32];
  char lora_history[1800];
  char lora_export[2000];
  char activity_history[1800];
  char plane_altitude_ft_text[24];
  char plane_speed_kmh_text[24];
};

static HttpJsonScratch g_http_json_scratch = {};

struct ImuState
{
  bool connected;
  bool healthy;
  bool has_solution;
  float acc_mps2[3];
  float gyro_dps[3];
  float pitch_deg;
  float roll_deg;
  float temperature_c;
  uint32_t sample_count;
  uint32_t last_update_ms;
};

struct ImuSample
{
  float acc_mps2[3];
  float gyro_dps[3];
  float temperature_c;
};

struct BlackboxState
{
  bool mounted;
  bool logging_enabled;
  bool file_open;
  bool last_write_ok;
  uint32_t records_written;
  uint32_t last_log_ms;
  uint64_t card_size_bytes;
  char file_path[BLACKBOX_PATH_LEN];
  char note[BLACKBOX_NOTE_LEN];
};

struct Bme688State
{
  bool connected;
  bool initialized;
  bool reading_ok;
  bool has_data;
  uint8_t i2c_addr;
  float temperature_c;
  float humidity_pct;
  float pressure_hpa;
  float altitude_m;
  float gas_ohms;
  uint32_t last_update_ms;
  char note[80];
};

struct GpsSat
{
  uint8_t prn;
  uint8_t elv;
  uint16_t az;
  uint8_t snr;
  char type;
};

struct GpsState
{
  GpsSat sats_info[32];
  uint8_t sats_count;
  bool uart_ready;
  bool receiving;
  bool has_fix;
  bool has_location;
  bool sentence_seen;
  uint8_t sats;
  uint8_t update_rate_hz;
  uint32_t rx_bytes;
  uint32_t line_count;
  uint32_t last_rx_ms;
  uint32_t last_fix_ms;
  float latitude_deg;
  float longitude_deg;
  float altitude_m;
  float speed_kmh;
  char utc_time[16];
  char utc_date[16];
  char last_sentence[96];
  char note[96];
  char config_note[96];
  char last_command[48];
};

struct LoraState
{
  bool uart_ready;
  bool enabled;
  bool aux_high;
  bool receiving;
  bool auto_tx_enabled;
  uint32_t rx_bytes;
  uint32_t tx_bytes;
  uint32_t last_rx_ms;
  uint32_t last_tx_ms;
  uint32_t auto_tx_interval_ms;
  uint32_t last_auto_tx_ms;
  char last_message[96];
  char last_rx_message[96];
  char last_tx_payload[96];
  char note[96];
  char auto_tx_mode[24];
};

static portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_blackbox_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_bme688_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_gps_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_lora_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_activity_mux = portMUX_INITIALIZER_UNLOCKED;
static ImuState g_imu_state = {};
static BlackboxState g_blackbox_state = {};
static Bme68x g_bme688;
static Bme688State g_bme688_state = {};
static GpsState g_gps_state = {};
static LoraState g_lora_state = {};
static uint8_t g_bme688_addr = 0;
static bool g_bme688_initialized = false;
static String scanI2C();
static const IPAddress WIFI_AP_LOCAL_IP(192, 168, 4, 1);
static const IPAddress WIFI_AP_GATEWAY(192, 168, 4, 1);
static const IPAddress WIFI_AP_SUBNET(255, 255, 255, 0);

// Explicit prototypes to stop the Arduino preprocessor from generating
// broken forward declarations for functions that use sketch-defined types.
static void storeImuState(const ImuState &state);
static ImuState copyImuState();
static void storeBlackboxState(const BlackboxState &state);
static BlackboxState copyBlackboxState();
static void storeBme688State(const Bme688State &state);
static Bme688State copyBme688State();
static void storeGpsState(const GpsState &state);
static GpsState copyGpsState();
static void storeLoraState(const LoraState &state);
static LoraState copyLoraState();
static uint8_t classifyMeteoTheme(const Bme688State &bme, char *theme_out, size_t theme_len, char *summary_out, size_t summary_len);
static const char *orientationModeLabel(OrientationMode mode);
static const char *screenIdSlug(AppScreenId screen_id);
static const char *screenIdLabel(AppScreenId screen_id);
static bool parseScreenIdFromSlug(const char *slug, AppScreenId *screen_out);
static void requestScreenLoad(AppScreenId screen_id, const char *origin);
static bool buildLoraTelemetryPayload(const char *mode, char *payload, size_t payload_len);
static void requestLoraAutoTxConfig(bool enabled, const char *mode, const char *origin);
static void handleLoraAutoTxTick(uint32_t now_ms);

static void fatalStop(const char *message)
{
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

static bool lockI2C(TickType_t timeout_ticks = pdMS_TO_TICKS(20))
{
  if (!g_i2c_mutex) {
    return true;
  }

  return xSemaphoreTake(g_i2c_mutex, timeout_ticks) == pdTRUE;
}

static void unlockI2C()
{
  if (g_i2c_mutex) {
    xSemaphoreGive(g_i2c_mutex);
  }
}

static void storeImuState(const ImuState &state)
{
  portENTER_CRITICAL(&g_imu_mux);
  g_imu_state = state;
  portEXIT_CRITICAL(&g_imu_mux);
}

static ImuState copyImuState()
{
  ImuState snapshot;

  portENTER_CRITICAL(&g_imu_mux);
  snapshot = g_imu_state;
  portEXIT_CRITICAL(&g_imu_mux);

  return snapshot;
}

static void storeBlackboxState(const BlackboxState &state)
{
  portENTER_CRITICAL(&g_blackbox_mux);
  g_blackbox_state = state;
  portEXIT_CRITICAL(&g_blackbox_mux);
}

static BlackboxState copyBlackboxState()
{
  BlackboxState snapshot;

  portENTER_CRITICAL(&g_blackbox_mux);
  snapshot = g_blackbox_state;
  portEXIT_CRITICAL(&g_blackbox_mux);

  return snapshot;
}

static void storeBme688State(const Bme688State &state)
{
  portENTER_CRITICAL(&g_bme688_mux);
  g_bme688_state = state;
  portEXIT_CRITICAL(&g_bme688_mux);
}

static Bme688State copyBme688State()
{
  Bme688State snapshot;

  portENTER_CRITICAL(&g_bme688_mux);
  snapshot = g_bme688_state;
  portEXIT_CRITICAL(&g_bme688_mux);

  return snapshot;
}

static void storeGpsState(const GpsState &state)
{
  portENTER_CRITICAL(&g_gps_mux);
  g_gps_state = state;
  portEXIT_CRITICAL(&g_gps_mux);
}

static GpsState copyGpsState()
{
  GpsState snapshot;

  portENTER_CRITICAL(&g_gps_mux);
  snapshot = g_gps_state;
  portEXIT_CRITICAL(&g_gps_mux);

  return snapshot;
}

static void storeLoraState(const LoraState &state)
{
  portENTER_CRITICAL(&g_lora_mux);
  g_lora_state = state;
  portEXIT_CRITICAL(&g_lora_mux);
}

static LoraState copyLoraState()
{
  LoraState snapshot;

  portENTER_CRITICAL(&g_lora_mux);
  snapshot = g_lora_state;
  portEXIT_CRITICAL(&g_lora_mux);

  return snapshot;
}

static void clearBlackboxTail()
{
  portENTER_CRITICAL(&g_blackbox_mux);
  g_blackbox_tail_count = 0;
  g_blackbox_tail_head = 0;
  for (uint8_t i = 0; i < BLACKBOX_TAIL_LINE_COUNT; ++i) {
    g_blackbox_tail_lines[i][0] = '\0';
  }
  portEXIT_CRITICAL(&g_blackbox_mux);
}

static void appendBlackboxTailLine(const char *line)
{
  if (!line || !line[0]) {
    return;
  }

  char clean_line[BLACKBOX_TAIL_LINE_LEN];
  snprintf(clean_line, sizeof(clean_line), "%s", line);
  size_t len = strlen(clean_line);
  while (len > 0 && (clean_line[len - 1] == '\r' || clean_line[len - 1] == '\n')) {
    clean_line[--len] = '\0';
  }

  portENTER_CRITICAL(&g_blackbox_mux);
  snprintf(
      g_blackbox_tail_lines[g_blackbox_tail_head],
      BLACKBOX_TAIL_LINE_LEN,
      "%s",
      clean_line);
  g_blackbox_tail_head = static_cast<uint8_t>((g_blackbox_tail_head + 1U) % BLACKBOX_TAIL_LINE_COUNT);
  if (g_blackbox_tail_count < BLACKBOX_TAIL_LINE_COUNT) {
    g_blackbox_tail_count += 1U;
  }
  portEXIT_CRITICAL(&g_blackbox_mux);
}

static void formatBlackboxTailJson(char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  buffer[0] = '\0';
  size_t out = 0;

  portENTER_CRITICAL(&g_blackbox_mux);
  const uint8_t count = g_blackbox_tail_count;
  const uint8_t head = g_blackbox_tail_head;

  for (uint8_t i = 0; i < count && out < (buffer_len - 1U); ++i) {
    const uint8_t index = static_cast<uint8_t>((head + BLACKBOX_TAIL_LINE_COUNT - count + i) % BLACKBOX_TAIL_LINE_COUNT);
    const char *line = g_blackbox_tail_lines[index];
    for (size_t j = 0; line[j] != '\0' && out < (buffer_len - 1U); ++j) {
      char c = line[j];
      if (c == '"' || c == '\\') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = c;
      } else if (c == '\r' || c == '\n') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = 'n';
      } else if (c < 32) {
        buffer[out++] = ' ';
      } else {
        buffer[out++] = c;
      }
    }

    if (i + 1U < count && out + 2U < buffer_len) {
      buffer[out++] = '\\';
      buffer[out++] = 'n';
    }
  }
  portEXIT_CRITICAL(&g_blackbox_mux);

  buffer[out] = '\0';
}

static void formatBlackboxFilesJson(char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  buffer[0] = '\0';
  if (!SD.exists("/logs")) {
    snprintf(buffer, buffer_len, "Sem pasta /logs");
    return;
  }

  File dir = SD.open("/logs");
  if (!dir || !dir.isDirectory()) {
    snprintf(buffer, buffer_len, "Nao foi possivel abrir /logs");
    return;
  }

  size_t out = 0;
  uint8_t count = 0;
  for (File entry = dir.openNextFile(); entry && count < 8; entry = dir.openNextFile()) {
    if (entry.isDirectory()) {
      entry.close();
      continue;
    }

    const char *name = entry.name();
    const size_t size_bytes = static_cast<size_t>(entry.size());
    char line[160];
    snprintf(line, sizeof(line), "%s | %u B", name ? name : "arquivo", static_cast<unsigned>(size_bytes));
    entry.close();

    for (size_t j = 0; line[j] != '\0' && out < (buffer_len - 1U); ++j) {
      const char c = line[j];
      if (c == '"' || c == '\\') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = c;
      } else if (c == '\n' || c == '\r') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = 'n';
      } else {
        buffer[out++] = c;
      }
    }

    ++count;
    if (count < 8 && out + 2U < buffer_len) {
      buffer[out++] = '\\';
      buffer[out++] = 'n';
    }
  }

  if (count == 0) {
    snprintf(buffer, buffer_len, "Nenhum arquivo em /logs");
  } else {
    buffer[out] = '\0';
  }
  dir.close();
}

static void clearActivityHistory()
{
  portENTER_CRITICAL(&g_activity_mux);
  g_activity_history_count = 0;
  g_activity_history_head = 0;
  for (uint8_t i = 0; i < ACTIVITY_HISTORY_LINE_COUNT; ++i) {
    g_activity_history_lines[i][0] = '\0';
  }
  portEXIT_CRITICAL(&g_activity_mux);
}

static void appendActivityLine(const char *tag, const char *payload)
{
  if ((!tag || !tag[0]) && (!payload || !payload[0])) {
    return;
  }

  char line[ACTIVITY_HISTORY_LINE_LEN];
  snprintf(
      line,
      sizeof(line),
      "%lus | %s | %s",
      static_cast<unsigned long>(millis() / 1000UL),
      (tag && tag[0]) ? tag : "?",
      (payload && payload[0]) ? payload : "-");

  portENTER_CRITICAL(&g_activity_mux);
  snprintf(
      g_activity_history_lines[g_activity_history_head],
      ACTIVITY_HISTORY_LINE_LEN,
      "%s",
      line);
  g_activity_history_head = static_cast<uint8_t>((g_activity_history_head + 1U) % ACTIVITY_HISTORY_LINE_COUNT);
  if (g_activity_history_count < ACTIVITY_HISTORY_LINE_COUNT) {
    g_activity_history_count += 1U;
  }
  portEXIT_CRITICAL(&g_activity_mux);
}

static void formatActivityHistoryJson(char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  buffer[0] = '\0';
  size_t out = 0;

  portENTER_CRITICAL(&g_activity_mux);
  const uint8_t count = g_activity_history_count;
  const uint8_t head = g_activity_history_head;

  for (uint8_t i = 0; i < count && out < (buffer_len - 1U); ++i) {
    const uint8_t index = static_cast<uint8_t>((head + ACTIVITY_HISTORY_LINE_COUNT - count + i) % ACTIVITY_HISTORY_LINE_COUNT);
    const char *line = g_activity_history_lines[index];
    for (size_t j = 0; line[j] != '\0' && out < (buffer_len - 1U); ++j) {
      const char c = line[j];
      if (c == '"' || c == '\\') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = c;
      } else if (c == '\n' || c == '\r') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = 'n';
      } else {
        buffer[out++] = c;
      }
    }
    if ((i + 1U) < count && out < (buffer_len - 2U)) {
      buffer[out++] = '\\';
      buffer[out++] = 'n';
    }
  }
  portEXIT_CRITICAL(&g_activity_mux);

  buffer[out] = '\0';
}

static void clearGpsActivityHistory()
{
  portENTER_CRITICAL(&g_activity_mux);
  g_gps_activity_count = 0;
  g_gps_activity_head = 0;
  for (uint8_t i = 0; i < GPS_ACTIVITY_LINE_COUNT; ++i) {
    g_gps_activity_lines[i][0] = '\0';
  }
  portEXIT_CRITICAL(&g_activity_mux);
}

static void appendGpsActivityLine(const char *tag, const char *payload)
{
  if ((!tag || !tag[0]) && (!payload || !payload[0])) {
    return;
  }

  char line[GPS_ACTIVITY_LINE_LEN];
  snprintf(
      line,
      sizeof(line),
      "%lus | %s | %s",
      static_cast<unsigned long>(millis() / 1000UL),
      (tag && tag[0]) ? tag : "GPS",
      (payload && payload[0]) ? payload : "-");

  portENTER_CRITICAL(&g_activity_mux);
  snprintf(
      g_gps_activity_lines[g_gps_activity_head],
      GPS_ACTIVITY_LINE_LEN,
      "%s",
      line);
  g_gps_activity_head = static_cast<uint8_t>((g_gps_activity_head + 1U) % GPS_ACTIVITY_LINE_COUNT);
  if (g_gps_activity_count < GPS_ACTIVITY_LINE_COUNT) {
    g_gps_activity_count += 1U;
  }
  portEXIT_CRITICAL(&g_activity_mux);
}

static void formatGpsActivityJson(char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  buffer[0] = '\0';
  size_t out = 0;

  portENTER_CRITICAL(&g_activity_mux);
  const uint8_t count = g_gps_activity_count;
  const uint8_t head = g_gps_activity_head;

  for (uint8_t i = 0; i < count && out < (buffer_len - 1U); ++i) {
    const uint8_t index = static_cast<uint8_t>((head + GPS_ACTIVITY_LINE_COUNT - count + i) % GPS_ACTIVITY_LINE_COUNT);
    const char *line = g_gps_activity_lines[index];
    for (size_t j = 0; line[j] != '\0' && out < (buffer_len - 1U); ++j) {
      const char c = line[j];
      if (c == '"' || c == '\\') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = c;
      } else if (c == '\n' || c == '\r') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = 'n';
      } else {
        buffer[out++] = c;
      }
    }
    if ((i + 1U) < count && out < (buffer_len - 2U)) {
      buffer[out++] = '\\';
      buffer[out++] = 'n';
    }
  }
  portEXIT_CRITICAL(&g_activity_mux);

  buffer[out] = '\0';
}

static void clearLoraHistory()
{
  portENTER_CRITICAL(&g_lora_mux);
  g_lora_history_count = 0;
  g_lora_history_head = 0;
  for (uint8_t i = 0; i < LORA_HISTORY_LINE_COUNT; ++i) {
    g_lora_history_lines[i][0] = '\0';
  }
  portEXIT_CRITICAL(&g_lora_mux);
}

static void appendLoraHistoryLine(const char *direction, const char *payload)
{
  if ((!direction || !direction[0]) && (!payload || !payload[0])) {
    return;
  }

  char line[LORA_HISTORY_LINE_LEN];
  snprintf(
      line,
      sizeof(line),
      "%s | %s",
      (direction && direction[0]) ? direction : "?",
      (payload && payload[0]) ? payload : "-");

  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
    line[--len] = '\0';
  }

  portENTER_CRITICAL(&g_lora_mux);
  snprintf(
      g_lora_history_lines[g_lora_history_head],
      LORA_HISTORY_LINE_LEN,
      "%s",
      line);
  g_lora_history_head = static_cast<uint8_t>((g_lora_history_head + 1U) % LORA_HISTORY_LINE_COUNT);
  if (g_lora_history_count < LORA_HISTORY_LINE_COUNT) {
    g_lora_history_count += 1U;
  }
  portEXIT_CRITICAL(&g_lora_mux);
}

static void formatLoraHistoryJson(char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  buffer[0] = '\0';
  size_t out = 0;

  portENTER_CRITICAL(&g_lora_mux);
  const uint8_t count = g_lora_history_count;
  const uint8_t head = g_lora_history_head;

  for (uint8_t i = 0; i < count && out < (buffer_len - 1U); ++i) {
    const uint8_t index = static_cast<uint8_t>((head + LORA_HISTORY_LINE_COUNT - count + i) % LORA_HISTORY_LINE_COUNT);
    const char *line = g_lora_history_lines[index];
    for (size_t j = 0; line[j] != '\0' && out < (buffer_len - 1U); ++j) {
      const char c = line[j];
      if (c == '"' || c == '\\') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = c;
      } else if (c == '\r' || c == '\n') {
        if (out + 2U >= buffer_len) {
          break;
        }
        buffer[out++] = '\\';
        buffer[out++] = 'n';
      } else if (c < 32) {
        buffer[out++] = ' ';
      } else {
        buffer[out++] = c;
      }
    }

    if (i + 1U < count && out + 2U < buffer_len) {
      buffer[out++] = '\\';
      buffer[out++] = 'n';
    }
  }
  portEXIT_CRITICAL(&g_lora_mux);

  buffer[out] = '\0';
}

static void formatLoraExportText(char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  buffer[0] = '\0';
  size_t out = 0;

  portENTER_CRITICAL(&g_lora_mux);
  const uint8_t count = g_lora_history_count;
  const uint8_t head = g_lora_history_head;

  for (uint8_t i = 0; i < count && out < (buffer_len - 1U); ++i) {
    const uint8_t index = static_cast<uint8_t>((head + LORA_HISTORY_LINE_COUNT - count + i) % LORA_HISTORY_LINE_COUNT);
    const char *line = g_lora_history_lines[index];
    for (size_t j = 0; line[j] != '\0' && out < (buffer_len - 1U); ++j) {
      buffer[out++] = line[j];
    }
    if (out < (buffer_len - 1U)) {
      buffer[out++] = '\n';
    }
  }
  portEXIT_CRITICAL(&g_lora_mux);

  buffer[out] = '\0';
}

static bool ipAddressValid(const IPAddress &ip)
{
  return ip != IPAddress(0, 0, 0, 0);
}

static void updateWifiRequestedFlag()
{
  g_wifi_mode_requested = g_wifi_ap_requested || g_wifi_sta_requested;
}

static float pressureToAltitudeMeters(float pressure_hpa, float sea_level_hpa = STANDARD_SEA_LEVEL_HPA)
{
  if (pressure_hpa <= 0.0f || sea_level_hpa <= 0.0f) {
    return 0.0f;
  }

  return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.1903f));
}

static uint8_t classifyMeteoTheme(const Bme688State &bme, char *theme_out, size_t theme_len, char *summary_out, size_t summary_len)
{
  auto setTexts = [&](const char *theme, const char *summary, uint8_t mode) -> uint8_t {
    if (theme_out && theme_len > 0) {
      snprintf(theme_out, theme_len, "%s", theme ? theme : "");
    }
    if (summary_out && summary_len > 0) {
      snprintf(summary_out, summary_len, "%s", summary ? summary : "");
    }
    return mode;
  };

  if (!bme.connected || !bme.has_data) {
    return setTexts(
        "AGUARDANDO",
        "Sem leitura valida do BME688. Assim que houver dados, o sistema estima aberto, nublado ou chuva provavel.",
        METEO_THEME_WAITING);
  }

  const float pressure = bme.pressure_hpa;
  const float humidity = bme.humidity_pct;

  if (pressure <= 1006.0f || (pressure <= 1009.0f && humidity >= 82.0f)) {
    return setTexts(
        "CHUVA PROVAVEL",
        "Pressao baixa e umidade alta sugerem instabilidade e chance maior de chuva.",
        METEO_THEME_RAIN);
  }

  if (pressure >= 1018.0f && humidity <= 65.0f) {
    return setTexts(
        "ABERTO",
        "Pressao alta e ar mais seco sugerem tempo firme e ceu mais aberto.",
        METEO_THEME_OPEN);
  }

  if (pressure >= 1014.0f && humidity <= 75.0f) {
    return setTexts(
        "ABERTO",
        "Pressao acima da media e umidade moderada sugerem tempo mais estavel.",
        METEO_THEME_OPEN);
  }

  return setTexts(
      "NUBLADO",
      "Umidade elevada ou pressao mais baixa sugerem mais nuvens e mudanca de tempo.",
      METEO_THEME_CLOUDY);
}

static float parseNmeaCoordinate(const char *value, const char hemisphere)
{
  if (!value || !value[0]) {
    return NAN;
  }

  const float raw = atof(value);
  if (raw == 0.0f) {
    return 0.0f;
  }

  const char *dot = strchr(value, '.');
  const size_t whole_digits = dot ? static_cast<size_t>(dot - value) : strlen(value);
  const size_t degree_digits = whole_digits > 4 ? 3 : 2;
  if (whole_digits <= degree_digits) {
    return NAN;
  }

  char degrees_text[4] = {};
  memcpy(degrees_text, value, degree_digits);
  const float degrees = static_cast<float>(atoi(degrees_text));
  const float minutes = atof(value + degree_digits);
  float coordinate = degrees + (minutes / 60.0f);
  if (hemisphere == 'S' || hemisphere == 'W') {
    coordinate = -coordinate;
  }
  return coordinate;
}

static void formatNmeaUtcTime(const char *raw, char *output, size_t output_len)
{
  if (!output || output_len == 0) {
    return;
  }

  output[0] = '\0';
  if (!raw || strlen(raw) < 6) {
    return;
  }

  char digits[7] = {};
  memcpy(digits, raw, 6);
  digits[6] = '\0';
  snprintf(output, output_len, "%.2s:%.2s:%.2s", digits, digits + 2, digits + 4);
}

static void formatNmeaUtcDate(const char *raw, char *output, size_t output_len)
{
  if (!output || output_len == 0) {
    return;
  }

  output[0] = '\0';
  if (!raw || strlen(raw) < 6) {
    return;
  }

  char digits[7] = {};
  memcpy(digits, raw, 6);
  digits[6] = '\0';
  snprintf(output, output_len, "20%.2s-%.2s-%.2s", digits + 4, digits + 2, digits);
}

static uint8_t computeNmeaChecksum(const char *payload)
{
  uint8_t checksum = 0U;
  if (!payload) {
    return checksum;
  }

  while (*payload) {
    checksum ^= static_cast<uint8_t>(*payload++);
  }
  return checksum;
}

static bool sendGpsPcasCommand(const char *payload, const char *label, const char *origin)
{
  if (!payload || !payload[0] || !g_gps_uart_ready) {
    GpsState state = copyGpsState();
    snprintf(state.config_note, sizeof(state.config_note), "Falha ao enviar %s", label ? label : "comando");
    storeGpsState(state);
    Serial.printf("[GPS] Falha ao enviar %s via %s\n", label ? label : "comando", origin ? origin : "sistema");
    return false;
  }

  const uint8_t checksum = computeNmeaChecksum(payload);
  char command[96];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", payload, checksum);
  const size_t written = Serial1.print(command);

  GpsState state = copyGpsState();
  snprintf(state.last_command, sizeof(state.last_command), "%s", label ? label : payload);
  snprintf(
      state.config_note,
      sizeof(state.config_note),
      "Comando %s enviado via %s",
      label ? label : payload,
      origin ? origin : "sistema");
  storeGpsState(state);
  appendGpsActivityLine("CMD", label ? label : payload);
  appendActivityLine("GPS", label ? label : payload);

  Serial.printf(
      "[GPS] CMD via %s | %s | raw=%s | bytes=%u\n",
      origin ? origin : "sistema",
      label ? label : payload,
      command,
      static_cast<unsigned>(written));
  return written > 0U;
}

static void requestGpsRateHz(uint8_t rate_hz, const char *origin)
{
  const char *payload = nullptr;
  switch (rate_hz) {
    case 1:
      payload = "PCAS02,1000";
      break;
    case 2:
      payload = "PCAS02,500";
      break;
    case 4:
      payload = "PCAS02,250";
      break;
    case 5:
      payload = "PCAS02,200";
      break;
    case 10:
      payload = "PCAS02,100";
      break;
    default:
      return;
  }

  if (sendGpsPcasCommand(payload, rate_hz == 1 ? "rate 1Hz" : rate_hz == 2 ? "rate 2Hz"
                                                   : rate_hz == 4   ? "rate 4Hz"
                                                   : rate_hz == 5   ? "rate 5Hz"
                                                                    : "rate 10Hz",
                         origin)) {
    GpsState state = copyGpsState();
    state.update_rate_hz = rate_hz;
    snprintf(state.config_note, sizeof(state.config_note), "Update rate configurado para %uHz", static_cast<unsigned>(rate_hz));
    storeGpsState(state);
  }
}

static void requestGpsRestartMode(uint8_t mode, const char *origin)
{
  const char *payload = nullptr;
  const char *label = nullptr;
  switch (mode) {
    case 0:
      payload = "PCAS10,0";
      label = "hot start";
      break;
    case 1:
      payload = "PCAS10,1";
      label = "warm start";
      break;
    case 2:
      payload = "PCAS10,2";
      label = "cold start";
      break;
    case 3:
      payload = "PCAS10,3";
      label = "factory start";
      break;
    default:
      return;
  }

  sendGpsPcasCommand(payload, label, origin);
}

static void requestGpsConstellationMode(uint8_t mode, const char *origin)
{
  const char *payload = nullptr;
  const char *label = nullptr;
  switch (mode) {
    case 1:
      payload = "PCAS04,1";
      label = "GPS";
      break;
    case 3:
      payload = "PCAS04,3";
      label = "GPS+BDS";
      break;
    case 5:
      payload = "PCAS04,5";
      label = "GPS+GLONASS";
      break;
    case 7:
      payload = "PCAS04,7";
      label = "GPS+BDS+GLONASS";
      break;
    default:
      return;
  }

  sendGpsPcasCommand(payload, label, origin);
}

static void requestGpsAllNmea(bool enable, const char *origin)
{
  sendGpsPcasCommand(
      enable ? "PCAS03,1,1,1,1,1,1,1,1,1,1,,,1,1" : "PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0",
      enable ? "all NMEA on" : "all NMEA off",
      origin);
}

static void finalizeGpsSentence(char *sentence)
{
  if (!sentence || sentence[0] == '\0' || sentence[0] != '$') {
    return;
  }

  GpsState state = copyGpsState();
  const bool had_sentence = state.sentence_seen;
  const bool had_fix = state.has_fix;
  const uint32_t now = millis();
  state.uart_ready = g_gps_uart_ready;
  state.sentence_seen = true;
  state.line_count += 1U;
  state.last_rx_ms = now;
  snprintf(state.last_sentence, sizeof(state.last_sentence), "%s", sentence);

  char work[128];
  snprintf(work, sizeof(work), "%s", sentence);
  char *checksum = strchr(work, '*');
  if (checksum) {
    *checksum = '\0';
  }

  char *save = nullptr;
  char *fields[20] = {};
  size_t field_count = 0;
  char *token = strtok_r(work, ",", &save);
  while (token && field_count < 20) {
    fields[field_count++] = token;
    token = strtok_r(nullptr, ",", &save);
  }

  if (field_count == 0 || !fields[0]) {
    storeGpsState(state);
    return;
  }

  const bool is_gga = strstr(fields[0], "GGA") != nullptr;
  const bool is_rmc = strstr(fields[0], "RMC") != nullptr;
  const bool is_gsv = strstr(fields[0], "GSV") != nullptr;

  if (is_gga && field_count >= 10) {
    formatNmeaUtcTime(fields[1], state.utc_time, sizeof(state.utc_time));
    const int fix_quality = fields[6] ? atoi(fields[6]) : 0;
    const uint8_t sats = fields[7] ? static_cast<uint8_t>(atoi(fields[7])) : 0U;
    const float lat = parseNmeaCoordinate(fields[2], fields[3] ? fields[3][0] : 'N');
    const float lon = parseNmeaCoordinate(fields[4], fields[5] ? fields[5][0] : 'E');
    const float alt_m = fields[9] ? atof(fields[9]) : 0.0f;
    state.sats = sats;
    if (!isnan(lat) && !isnan(lon)) {
      state.latitude_deg = lat;
      state.longitude_deg = lon;
      state.has_location = true;
    }
    state.altitude_m = alt_m;
    state.has_fix = fix_quality > 0;
    if (state.has_fix) {
      state.last_fix_ms = now;
    }
  } else if (is_rmc && field_count >= 8) {
    formatNmeaUtcTime(fields[1], state.utc_time, sizeof(state.utc_time));
    if (field_count >= 10) {
      formatNmeaUtcDate(fields[9], state.utc_date, sizeof(state.utc_date));
    }
    const bool valid = fields[2] && fields[2][0] == 'A';
    const float lat = parseNmeaCoordinate(fields[3], fields[4] ? fields[4][0] : 'N');
    const float lon = parseNmeaCoordinate(fields[5], fields[6] ? fields[6][0] : 'E');
    const float speed_knots = fields[7] ? atof(fields[7]) : 0.0f;
    if (!isnan(lat) && !isnan(lon)) {
      state.latitude_deg = lat;
      state.longitude_deg = lon;
      state.has_location = true;
    }
    state.speed_kmh = speed_knots * 1.852f;
    if (state.speed_kmh < 3.0f) {
      state.speed_kmh = 0.0f;
    }
    if (valid) {
      state.has_fix = true;
      state.last_fix_ms = now;
    }
  } else if (is_gsv && field_count >= 4) {
    const uint8_t msg_num = atoi(fields[2]);
    if (msg_num == 1) { 
        if (fields[0][1] == 'G' && fields[0][2] == 'P') state.sats_count = 0; 
    }
    char s_type = 'G';
    if (fields[0][1] == 'G' && fields[0][2] == 'L') s_type = 'R'; 
    else if (fields[0][1] == 'G' && (fields[0][2] == 'B' || fields[0][2] == 'Q')) s_type = 'B'; 
    else if (fields[0][1] == 'G' && fields[0][2] == 'A') s_type = 'E'; 
    else if (fields[0][1] == 'B' && fields[0][2] == 'D') s_type = 'B'; 

    for (int i = 0; i < 4; i++) {
        int base = 4 + i * 4;
        if (base + 3 < field_count && fields[base] && fields[base][0] && state.sats_count < 32) {
            state.sats_info[state.sats_count].prn = atoi(fields[base]);
            state.sats_info[state.sats_count].elv = atoi(fields[base + 1]);
            state.sats_info[state.sats_count].az = atoi(fields[base + 2]);
            state.sats_info[state.sats_count].snr = fields[base + 3][0] ? atoi(fields[base + 3]) : 0;
            state.sats_info[state.sats_count].type = s_type;
            state.sats_count++;
        }
    }
  }

  state.receiving = true;
  if (!had_sentence) {
    appendGpsActivityLine("NMEA", "primeira sentenca recebida");
    appendActivityLine("GPS", "primeira sentenca NMEA");
  }
  if (!had_fix && state.has_fix) {
    char fix_line[96];
    snprintf(
        fix_line,
        sizeof(fix_line),
        "fix ganho | %u sats | %.5f %.5f",
        static_cast<unsigned>(state.sats),
        state.latitude_deg,
        state.longitude_deg);
    appendGpsActivityLine("FIX", fix_line);
    appendActivityLine("GPS", "fix valido adquirido");
  } else if (had_fix && !state.has_fix) {
    appendGpsActivityLine("FIX", "fix perdido");
    appendActivityLine("GPS", "fix perdido");
  }
  if (state.has_fix) {
    snprintf(
        state.note,
        sizeof(state.note),
        "Fix OK | %u sats | %.5f %.5f",
        static_cast<unsigned>(state.sats),
        state.latitude_deg,
        state.longitude_deg);
  } else {
    snprintf(
        state.note,
        sizeof(state.note),
        "NMEA RX | linhas %lu | aguardando fix",
        static_cast<unsigned long>(state.line_count));
  }
  storeGpsState(state);
}

static void pollGpsUart()
{
  GpsState state = copyGpsState();
  const uint32_t now = millis();
  state.uart_ready = g_gps_uart_ready;

  while (g_gps_uart_ready && Serial1.available() > 0) {
    const int raw = Serial1.read();
    if (raw < 0) {
      break;
    }

    const char c = static_cast<char>(raw);
    state.rx_bytes += 1U;
    state.last_rx_ms = now;
    state.receiving = true;

    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (g_gps_line_len > 0) {
        g_gps_line_buffer[g_gps_line_len] = '\0';
        finalizeGpsSentence(g_gps_line_buffer);
        g_gps_line_len = 0;
        g_gps_line_buffer[0] = '\0';
        state = copyGpsState();
      }
      continue;
    }

    if (g_gps_line_len < (sizeof(g_gps_line_buffer) - 1U)) {
      g_gps_line_buffer[g_gps_line_len++] = c;
    } else {
      g_gps_line_buffer[g_gps_line_len] = '\0';
      finalizeGpsSentence(g_gps_line_buffer);
      g_gps_line_len = 0;
      g_gps_line_buffer[0] = '\0';
      state = copyGpsState();
    }
  }

  state.receiving = state.uart_ready && state.rx_bytes > 0U && (now - state.last_rx_ms) < GPS_STALE_MS;
  if (!state.uart_ready) {
    snprintf(state.note, sizeof(state.note), "UART GPS indisponivel");
  } else if (!state.receiving && !state.sentence_seen) {
    snprintf(state.note, sizeof(state.note), "UART pronta | sem trafego NMEA");
  } else if (!state.has_fix && state.receiving) {
    snprintf(
        state.note,
        sizeof(state.note),
        "Recebendo NMEA | linhas %lu | fix pendente",
        static_cast<unsigned long>(state.line_count));
  } else if (!state.receiving && state.sentence_seen && !state.has_fix) {
    snprintf(state.note, sizeof(state.note), "Sem trafego recente | ultimo fix pendente");
  }
  storeGpsState(state);
}

static void finalizeLoraMessage(char *message)
{
  if (!message || message[0] == '\0') {
    return;
  }

  LoraState state = copyLoraState();
  state.receiving = true;
  snprintf(state.last_message, sizeof(state.last_message), "%s", message);
  snprintf(state.last_rx_message, sizeof(state.last_rx_message), "%s", message);
  snprintf(state.note, sizeof(state.note), "RX %lu bytes | ultima msg pronta", static_cast<unsigned long>(state.rx_bytes));
  storeLoraState(state);
  appendLoraHistoryLine("RX", message);
  appendActivityLine("LORA", "payload RX");
  Serial.printf("[LORA] RX: %s\n", message);
}

static void pollLoraUart()
{
  LoraState state = copyLoraState();
  const uint32_t now = millis();
  state.uart_ready = g_lora_uart_ready;
  state.enabled = g_lora_enabled;
  state.aux_high = g_lora_uart_ready ? (digitalRead(LORA_AUX_PIN) == HIGH) : false;

  while (g_lora_uart_ready && Serial2.available() > 0) {
    const int raw = Serial2.read();
    if (raw < 0) {
      break;
    }

    const char c = static_cast<char>(raw);
    state.rx_bytes += 1U;
    state.last_rx_ms = now;
    state.receiving = true;

    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (g_lora_line_len > 0) {
        g_lora_line_buffer[g_lora_line_len] = '\0';
        finalizeLoraMessage(g_lora_line_buffer);
        g_lora_line_len = 0;
        g_lora_line_buffer[0] = '\0';
        state = copyLoraState();
      }
      continue;
    }

    const char safe_char = (c >= 32 && c <= 126) ? c : '.';
    if (g_lora_line_len < (sizeof(g_lora_line_buffer) - 1U)) {
      g_lora_line_buffer[g_lora_line_len++] = safe_char;
    } else {
      g_lora_line_buffer[g_lora_line_len] = '\0';
      finalizeLoraMessage(g_lora_line_buffer);
      g_lora_line_len = 0;
      g_lora_line_buffer[0] = '\0';
      state = copyLoraState();
    }
  }

  if (g_lora_line_len > 0 && state.last_rx_ms > 0 && (now - state.last_rx_ms) > 120U) {
    g_lora_line_buffer[g_lora_line_len] = '\0';
    finalizeLoraMessage(g_lora_line_buffer);
    g_lora_line_len = 0;
    g_lora_line_buffer[0] = '\0';
    state = copyLoraState();
  }

  state.receiving = state.uart_ready && state.rx_bytes > 0U && (now - state.last_rx_ms) < LORA_STALE_MS;
  if (!state.uart_ready) {
    snprintf(state.note, sizeof(state.note), "UART LoRa indisponivel");
  } else if (!state.enabled) {
    snprintf(state.note, sizeof(state.note), "LoRa desligado");
  } else if (state.receiving) {
    snprintf(
        state.note,
        sizeof(state.note),
        "LoRa RX ativo | AUX %s | %lu bytes",
        state.aux_high ? "HIGH" : "LOW",
        static_cast<unsigned long>(state.rx_bytes));
  } else {
    snprintf(
        state.note,
        sizeof(state.note),
        "LoRa pronto | AUX %s | sem trafego",
        state.aux_high ? "HIGH" : "LOW");
  }
  storeLoraState(state);
}

static void requestLoraTestSend(const char *payload, const char *origin)
{
  if (!payload || !payload[0]) {
    return;
  }

  LoraState state = copyLoraState();
  if (!g_lora_uart_ready || !g_lora_enabled) {
    snprintf(state.note, sizeof(state.note), "Envio bloqueado: LoRa OFF");
    storeLoraState(state);
    Serial.printf("[LORA] Envio ignorado via %s: modulo OFF\n", origin ? origin : "sistema");
    return;
  }

  Serial2.print(payload);
  Serial2.print("\r\n");
  state.tx_bytes += static_cast<uint32_t>(strlen(payload) + 2U);
  state.last_tx_ms = millis();
  snprintf(state.last_message, sizeof(state.last_message), "TX: %s", payload);
  snprintf(state.last_tx_payload, sizeof(state.last_tx_payload), "%s", payload);
  state.auto_tx_enabled = g_lora_auto_tx_enabled;
  state.auto_tx_interval_ms = g_lora_auto_tx_interval_ms;
  state.last_auto_tx_ms = g_lora_auto_tx_last_ms;
  snprintf(state.auto_tx_mode, sizeof(state.auto_tx_mode), "%s", g_lora_auto_tx_mode);
  snprintf(
      state.note,
      sizeof(state.note),
      "TX %lu bytes | ultima carga: %s",
      static_cast<unsigned long>(state.tx_bytes),
      payload);
  storeLoraState(state);
  appendLoraHistoryLine("TX", payload);
  appendActivityLine("LORA", payload);
  Serial.printf("[LORA] Envio via %s: %s\n", origin ? origin : "sistema", payload);
}

static bool buildLoraTelemetryPayload(const char *mode, char *payload, size_t payload_len)
{
  if (!payload || payload_len == 0) {
    return false;
  }

  payload[0] = '\0';
  const char *resolved_mode = (mode && mode[0]) ? mode : "summary";

  if (strcmp(resolved_mode, "meteo") == 0) {
    const Bme688State bme = copyBme688State();
    if (!bme.connected || !bme.has_data) {
      snprintf(payload, payload_len, "METEO:SEM_DADOS");
      return true;
    }

    snprintf(
        payload,
        payload_len,
        "METEO,T=%.1f,H=%.1f,P=%.1f,ALT=%.1f",
        bme.temperature_c,
        bme.humidity_pct,
        bme.pressure_hpa,
        bme.altitude_m);
    return true;
  }

  if (strcmp(resolved_mode, "gps") == 0) {
    const GpsState gps = copyGpsState();
    if (!gps.receiving) {
      return false;
    }

    if (!gps.has_location) {
      snprintf(
          payload,
          payload_len,
          "GPS,NMEA,sats=%u,fix=%u",
          static_cast<unsigned>(gps.sats),
          gps.has_fix ? 1U : 0U);
    } else {
      snprintf(
          payload,
          payload_len,
          "GPS,fix=%u,sats=%u,lat=%.5f,lon=%.5f,alt=%.1f,spd=%.1f",
          gps.has_fix ? 1U : 0U,
          static_cast<unsigned>(gps.sats),
          gps.latitude_deg,
          gps.longitude_deg,
          gps.altitude_m,
          gps.speed_kmh);
    }
    return true;
  }

  const Bme688State bme = copyBme688State();
  const GpsState gps = copyGpsState();
  const BlackboxState blackbox = copyBlackboxState();

  if (!bme.connected || !bme.has_data) {
    snprintf(payload, payload_len, "AUTO,SEM_METEO");
    return true;
  }

  if (gps.has_location) {
    snprintf(
        payload,
        payload_len,
        "AUTO,T=%.1f,H=%.1f,P=%.1f,AB=%.1f,AG=%.1f,F=%u,S=%u,LAT=%.5f,LON=%.5f,SPD=%.1f,R=%lu",
        bme.temperature_c,
        bme.humidity_pct,
        bme.pressure_hpa,
        bme.altitude_m,
        gps.has_fix ? gps.altitude_m : 0.0f,
        gps.has_fix ? 1U : 0U,
        static_cast<unsigned>(gps.sats),
        gps.latitude_deg,
        gps.longitude_deg,
        gps.receiving ? gps.speed_kmh : 0.0f,
        static_cast<unsigned long>(blackbox.records_written));
  } else {
    snprintf(
        payload,
        payload_len,
        "AUTO,T=%.1f,H=%.1f,P=%.1f,AB=%.1f,AG=%.1f,F=%u,S=%u,SPD=%.1f,R=%lu",
        bme.temperature_c,
        bme.humidity_pct,
        bme.pressure_hpa,
        bme.altitude_m,
        gps.has_fix ? gps.altitude_m : 0.0f,
        gps.has_fix ? 1U : 0U,
        static_cast<unsigned>(gps.sats),
        gps.receiving ? gps.speed_kmh : 0.0f,
        static_cast<unsigned long>(blackbox.records_written));
  }
  return true;
}

static void requestLoraTelemetrySend(const char *mode, const char *origin)
{
  char payload[160];
  if (buildLoraTelemetryPayload(mode, payload, sizeof(payload)) && payload[0]) {
    requestLoraTestSend(payload, origin);
  }
}

static void requestLoraAutoTxConfig(bool enabled, const char *mode, const char *origin)
{
  if (mode && mode[0]) {
    snprintf(g_lora_auto_tx_mode, sizeof(g_lora_auto_tx_mode), "%s", mode);
  }

  g_lora_auto_tx_enabled = enabled;
  if (!enabled) {
    g_lora_auto_tx_last_ms = 0;
  }

  LoraState state = copyLoraState();
  state.auto_tx_enabled = g_lora_auto_tx_enabled;
  state.auto_tx_interval_ms = g_lora_auto_tx_interval_ms;
  state.last_auto_tx_ms = g_lora_auto_tx_last_ms;
  snprintf(state.auto_tx_mode, sizeof(state.auto_tx_mode), "%s", g_lora_auto_tx_mode);
  snprintf(
      state.note,
      sizeof(state.note),
      enabled ? "Auto TX 30s ativo | modo %s" : "Auto TX desligado | modo %s",
      state.auto_tx_mode[0] ? state.auto_tx_mode : "summary");
  storeLoraState(state);
  appendActivityLine("LORA", enabled ? "auto TX ON" : "auto TX OFF");
  Serial.printf(
      "[LORA] Auto TX via %s: %s | modo=%s | periodo=%lums\n",
      origin ? origin : "sistema",
      enabled ? "ON" : "OFF",
      g_lora_auto_tx_mode,
      static_cast<unsigned long>(g_lora_auto_tx_interval_ms));
}

static void handleLoraAutoTxTick(uint32_t now_ms)
{
  if (!g_lora_auto_tx_enabled || !g_lora_enabled || !g_lora_uart_ready) {
    return;
  }

  if ((now_ms - g_lora_auto_tx_last_ms) < g_lora_auto_tx_interval_ms) {
    return;
  }

  char payload[160];
  if (!buildLoraTelemetryPayload(g_lora_auto_tx_mode, payload, sizeof(payload)) || !payload[0]) {
    return;
  }

  requestLoraTestSend(payload, "auto30s");
  g_lora_auto_tx_last_ms = now_ms;
  LoraState state = copyLoraState();
  state.auto_tx_enabled = g_lora_auto_tx_enabled;
  state.auto_tx_interval_ms = g_lora_auto_tx_interval_ms;
  state.last_auto_tx_ms = g_lora_auto_tx_last_ms;
  snprintf(state.auto_tx_mode, sizeof(state.auto_tx_mode), "%s", g_lora_auto_tx_mode);
  storeLoraState(state);
}

static void setImuTrim(float pitch_trim_deg, float roll_trim_deg)
{
  portENTER_CRITICAL(&g_imu_mux);
  g_pitch_trim_deg = pitch_trim_deg;
  g_roll_trim_deg = roll_trim_deg;
  portEXIT_CRITICAL(&g_imu_mux);
}

static void getImuTrim(float *pitch_trim_deg, float *roll_trim_deg)
{
  portENTER_CRITICAL(&g_imu_mux);
  if (pitch_trim_deg) {
    *pitch_trim_deg = g_pitch_trim_deg;
  }
  if (roll_trim_deg) {
    *roll_trim_deg = g_roll_trim_deg;
  }
  portEXIT_CRITICAL(&g_imu_mux);
}

static float clampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }

  if (value > max_value) {
    return max_value;
  }

  return value;
}

static int32_t uiWidth()
{
  return g_lv_display ? lv_display_get_horizontal_resolution(g_lv_display) : LCD_W;
}

static int32_t uiHeight()
{
  return g_lv_display ? lv_display_get_vertical_resolution(g_lv_display) : LCD_H;
}

static bool isLandscapeUI()
{
  return uiWidth() > uiHeight();
}

static const char *orientationModeLabel(OrientationMode mode)
{
  switch (mode) {
    case ORIENTATION_MODE_LANDSCAPE:
      return "Horizontal";
    case ORIENTATION_MODE_AUTO:
      return "Auto";
    default:
      return "Vertical";
  }
}

static const char *rotationLabel(lv_display_rotation_t rotation)
{
  switch (rotation) {
    case LV_DISPLAY_ROTATION_90:
      return "Horizontal dir";
    case LV_DISPLAY_ROTATION_180:
      return "Vertical invertido";
    case LV_DISPLAY_ROTATION_270:
      return "Horizontal esq";
    default:
      return "Vertical";
  }
}

static uint8_t clampBrightness(int value)
{
  if (value < 24) {
    return 24;
  }
  if (value > 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

static void applyPanelBrightness(uint8_t value)
{
  g_display_brightness = clampBrightness(value);
  if (g_panel) {
    g_panel->setBrightness(g_display_brightness);
  }
}

static const char *cardTypeLabel(sdcard_type_t card_type)
{
  switch (card_type) {
    case CARD_MMC:
      return "MMC";
    case CARD_SD:
      return "SDSC";
    case CARD_SDHC:
      return "SDHC";
    default:
      return "desconhecido";
  }
}

static void formatStorageSize(uint64_t bytes, char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  if (bytes == 0) {
    snprintf(buffer, buffer_len, "--");
    return;
  }

  const uint64_t megabytes = bytes / (1024ULL * 1024ULL);
  if (megabytes >= 1024ULL) {
    const float gigabytes = static_cast<float>(bytes) / (1024.0f * 1024.0f * 1024.0f);
    snprintf(buffer, buffer_len, "%.1f GB", gigabytes);
    return;
  }

  snprintf(buffer, buffer_len, "%llu MB", static_cast<unsigned long long>(megabytes));
}

static const char *baseFileName(const char *path)
{
  if (!path || !path[0]) {
    return "-";
  }

  const char *slash = strrchr(path, '/');
  return slash ? (slash + 1) : path;
}

static void startWifiPortal();
static void stopWifiPortal();
static void handleWifiPortal();

static void formatIpAddress(const IPAddress &ip, char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return;
  }

  snprintf(
      buffer,
      buffer_len,
      "%u.%u.%u.%u",
      static_cast<unsigned>(ip[0]),
      static_cast<unsigned>(ip[1]),
      static_cast<unsigned>(ip[2]),
      static_cast<unsigned>(ip[3]));
}

static const char *wifiStateLabel()
{
  if (g_wifi_ap_started) {
    return "ativo";
  }

  if (g_wifi_ap_requested) {
    return "subindo";
  }

  return "off";
}

static const char *wifiModeLabel()
{
  if (g_wifi_ap_started && g_wifi_sta_connected) {
    return "AP+LAN";
  }
  if (g_wifi_ap_started && g_wifi_sta_requested) {
    return "AP+STA";
  }
  if (g_wifi_ap_started) {
    return "AP";
  }
  if (g_wifi_sta_connected) {
    return "LAN";
  }
  if (g_wifi_sta_requested) {
    return "STA";
  }
  return "off";
}

static uint16_t wifiClientCount()
{
  if (!g_wifi_ap_started) {
    return 0U;
  }

  wifi_sta_list_t sta_list = {};
  if (esp_wifi_ap_get_sta_list(&sta_list) != ESP_OK) {
    return 0U;
  }

  return static_cast<uint16_t>(sta_list.num);
}

static void formatWifiApIp(char *buffer, size_t buffer_len)
{
  if (g_wifi_ap_started && ipAddressValid(g_wifi_ap_ip)) {
    formatIpAddress(g_wifi_ap_ip, buffer, buffer_len);
    return;
  }

  formatIpAddress(WIFI_AP_LOCAL_IP, buffer, buffer_len);
}

static void formatWifiStaIp(char *buffer, size_t buffer_len)
{
  if (g_wifi_sta_connected && ipAddressValid(g_wifi_sta_ip)) {
    formatIpAddress(g_wifi_sta_ip, buffer, buffer_len);
    return;
  }

  snprintf(buffer, buffer_len, "--");
}

static void formatWifiVisibleIp(char *buffer, size_t buffer_len)
{
  if (g_wifi_sta_connected && ipAddressValid(g_wifi_sta_ip)) {
    formatIpAddress(g_wifi_sta_ip, buffer, buffer_len);
    return;
  }

  formatWifiApIp(buffer, buffer_len);
}

static void printWifiStatus(const char *reason)
{
  char ip_text[20];
  char ap_ip_text[20];
  char sta_ip_text[20];
  char mac_text[24];
  formatWifiVisibleIp(ip_text, sizeof(ip_text));
  formatWifiApIp(ap_ip_text, sizeof(ap_ip_text));
  formatWifiStaIp(sta_ip_text, sizeof(sta_ip_text));
  wifi_mode_t mode = WIFI_MODE_NULL;
  uint8_t mac[6] = {0};
  esp_wifi_get_mode(&mode);
  if (g_wifi_ap_started) {
    esp_wifi_get_mac(WIFI_IF_AP, mac);
  }
  snprintf(
      mac_text,
      sizeof(mac_text),
      "%02X:%02X:%02X:%02X:%02X:%02X",
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5]);
  Serial.printf(
      "[WIFI] %s | mode=%s(%d) | req_ap=%s req_sta=%s | ap=%s ip=%s | sta=%s ip=%s ssid=%s | primary=%s | ap_ssid=%s | pass=%s | clients=%u | hits=%lu | mac=%s | heap=%lu | psram=%lu\n",
      reason ? reason : "Status",
      wifiModeLabel(),
      static_cast<int>(mode),
      g_wifi_ap_requested ? "on" : "off",
      g_wifi_sta_requested ? "on" : "off",
      g_wifi_ap_started ? "on" : "off",
      ap_ip_text,
      g_wifi_sta_connected ? "on" : "off",
      sta_ip_text,
      g_wifi_sta_ssid[0] ? g_wifi_sta_ssid : "-",
      ip_text,
      WIFI_AP_SSID,
      WIFI_AP_PASSWORD,
      static_cast<unsigned>(wifiClientCount()),
      static_cast<unsigned long>(g_wifi_web_hits),
      mac_text,
      static_cast<unsigned long>(ESP.getFreeHeap()),
      static_cast<unsigned long>(ESP.getFreePsram()));
}

static void releaseEfisCanvasBuffer()
{
  if (!g_efis_canvas_buf) {
    return;
  }

  heap_caps_free(g_efis_canvas_buf);
  g_efis_canvas_buf = nullptr;
  Serial.println("[EFIS] Buffer do horizonte liberado para modo leve");
}

static void restoreEfisCanvasBufferIfNeeded()
{
  if (!g_efis_horizon || g_efis_canvas_buf) {
    return;
  }

  if (!ensureEfisCanvasBuffer()) {
    Serial.println("[EFIS] Nao foi possivel restaurar o buffer do horizonte");
    return;
  }

  lv_canvas_set_buffer(
      g_efis_horizon,
      g_efis_canvas_buf,
      g_efis_horizon_w,
      g_efis_horizon_h,
      LV_COLOR_FORMAT_RGB565);
  drawEfisHorizon(copyImuState());
  Serial.println("[EFIS] Buffer do horizonte restaurado");
}

static void updateWifiLed()
{
  static uint32_t last_blink_ms = 0;
  static bool blink_on = false;

  if (g_wifi_ap_started) {
    const uint32_t now = millis();
    if ((now - last_blink_ms) >= 400U) {
      last_blink_ms = now;
      blink_on = !blink_on;
      rgbLedWrite(RGB_LED_PIN, 0, blink_on ? 48 : 0, 0);
    }
    return;
  }

  if (g_wifi_mode_requested) {
    rgbLedWrite(RGB_LED_PIN, 0, 24, 0);
    return;
  }

  blink_on = false;
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
}

static void updateRuntimeServiceMode()
{
  const bool light_mode = g_wifi_ap_requested || g_wifi_ap_started;
  if (light_mode == g_runtime_services_light) {
    return;
  }

  g_runtime_services_light = light_mode;
  if (g_runtime_services_light) {
    releaseEfisCanvasBuffer();
    if (g_blackbox_enabled_target) {
      g_blackbox_enabled_target = false;
      g_blackbox_resume_after_wifi = true;
      Serial.println("[BLACKBOX] Logger pausado enquanto Wi-Fi AP estiver ativo");
    }
    g_efis_display_seeded = false;
    Serial.println("[MODE] Wi-Fi ativo: runtime em modo leve, METEO permanece disponivel");
  } else {
    restoreEfisCanvasBufferIfNeeded();
    if (g_blackbox_resume_after_wifi) {
      g_blackbox_enabled_target = true;
      g_blackbox_resume_after_wifi = false;
      Serial.println("[BLACKBOX] Logger reativado ao sair do modo Wi-Fi");
    }
    Serial.println("[MODE] Wi-Fi desligado: cockpit pesado liberado novamente");
  }
}

static void onWifiEvent(void *, esp_event_base_t event_base, int32_t event_id, void *)
{
  if (event_base != WIFI_EVENT) {
    return;
  }

  if (event_id == WIFI_EVENT_STA_START) {
    if (g_wifi_sta_requested && g_wifi_sta_ssid[0]) {
      esp_wifi_connect();
      snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "Conectando em %s...", g_wifi_sta_ssid);
    }
  } else if (event_id == WIFI_EVENT_AP_START) {
    g_wifi_ap_started = true;
    refreshWifiApIp();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP ativo em http://192.168.4.1");
    appendActivityLine("WIFI", "AP_START");
    Serial.println("[WIFI] Evento AP_START");
  } else if (event_id == WIFI_EVENT_AP_STOP) {
    g_wifi_ap_started = false;
    g_wifi_web_started = false;
    appendActivityLine("WIFI", "AP_STOP");
    if (g_wifi_ap_requested) {
      g_wifi_apply_pending = true;
      g_wifi_recover_request_ms = millis();
      snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP caiu. Tentando recuperar...");
      Serial.println("[WIFI] Evento AP_STOP, agendando recuperacao");
    } else {
      Serial.println("[WIFI] Evento AP_STOP");
    }
  } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
    g_wifi_sta_started = true;
    snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "Associado ao roteador. Aguardando IP...");
  } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
    g_wifi_sta_connected = false;
    g_wifi_sta_ip = IPAddress(0, 0, 0, 0);
    if (g_wifi_sta_requested && g_wifi_sta_ssid[0]) {
      esp_wifi_connect();
      snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "STA desconectado. Tentando reconectar...");
    } else {
      snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "LAN desligada");
    }
  }
}

static void onWifiIpEvent(void *, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base != IP_EVENT || event_id != IP_EVENT_STA_GOT_IP || !event_data) {
    return;
  }

  const ip_event_got_ip_t *got_ip = static_cast<const ip_event_got_ip_t *>(event_data);
  const uint32_t addr = got_ip->ip_info.ip.addr;
  g_wifi_sta_ip = IPAddress(
      static_cast<uint8_t>(addr & 0xFF),
      static_cast<uint8_t>((addr >> 8) & 0xFF),
      static_cast<uint8_t>((addr >> 16) & 0xFF),
      static_cast<uint8_t>((addr >> 24) & 0xFF));
  g_wifi_sta_connected = true;
  g_wifi_sta_started = true;
  char ip_text[20];
  formatIpAddress(g_wifi_sta_ip, ip_text, sizeof(ip_text));
  snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "LAN OK em http://%s", ip_text);
  
  if (g_wifi_ap_requested) {
    g_wifi_ap_requested = false;
    g_wifi_apply_pending = true;
    updateWifiRequestedFlag();
  }
}

static bool ensureWifiStack()
{
  if (g_wifi_stack_ready) {
    return true;
  }

  esp_err_t err = esp_netif_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[WIFI] esp_netif_init falhou: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[WIFI] esp_event_loop_create_default falhou: %s\n", esp_err_to_name(err));
    return false;
  }

  if (!g_wifi_ap_netif) {
    g_wifi_ap_netif = esp_netif_create_default_wifi_ap();
    if (!g_wifi_ap_netif) {
      Serial.println("[WIFI] Falha ao criar esp_netif do AP");
      return false;
    }
  }

  if (!g_wifi_sta_netif) {
    g_wifi_sta_netif = esp_netif_create_default_wifi_sta();
    if (!g_wifi_sta_netif) {
      Serial.println("[WIFI] Falha ao criar esp_netif do STA");
      return false;
    }
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&cfg);
  if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) {
    Serial.printf("[WIFI] esp_wifi_init falhou: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (err != ESP_OK) {
    Serial.printf("[WIFI] esp_wifi_set_storage falhou: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_wifi_set_ps(WIFI_PS_NONE);
  if (err != ESP_OK) {
    Serial.printf("[WIFI] esp_wifi_set_ps falhou: %s\n", esp_err_to_name(err));
    return false;
  }

  if (!g_wifi_events_registered) {
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, nullptr);
    if (err != ESP_OK) {
      Serial.printf("[WIFI] Registro do handler WIFI_EVENT falhou: %s\n", esp_err_to_name(err));
      return false;
    }

    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiIpEvent, nullptr);
    if (err != ESP_OK) {
      Serial.printf("[WIFI] Registro do handler IP_EVENT falhou: %s\n", esp_err_to_name(err));
      return false;
    }

    g_wifi_events_registered = true;
  }

  g_wifi_stack_ready = true;
  return true;
}

static bool refreshWifiApIp()
{
  if (!g_wifi_ap_netif) {
    g_wifi_ap_ip = WIFI_AP_LOCAL_IP;
    return false;
  }

  esp_netif_ip_info_t ip_info = {};
  if (esp_netif_get_ip_info(g_wifi_ap_netif, &ip_info) != ESP_OK) {
    g_wifi_ap_ip = WIFI_AP_LOCAL_IP;
    return false;
  }

  const uint32_t addr = ip_info.ip.addr;
  g_wifi_ap_ip = IPAddress(
      static_cast<uint8_t>(addr & 0xFF),
      static_cast<uint8_t>((addr >> 8) & 0xFF),
      static_cast<uint8_t>((addr >> 16) & 0xFF),
      static_cast<uint8_t>((addr >> 24) & 0xFF));
  return true;
}

static bool refreshWifiStaIp()
{
  if (!g_wifi_sta_netif) {
    g_wifi_sta_ip = IPAddress(0, 0, 0, 0);
    return false;
  }

  esp_netif_ip_info_t ip_info = {};
  if (esp_netif_get_ip_info(g_wifi_sta_netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
    g_wifi_sta_ip = IPAddress(0, 0, 0, 0);
    return false;
  }

  const uint32_t addr = ip_info.ip.addr;
  g_wifi_sta_ip = IPAddress(
      static_cast<uint8_t>(addr & 0xFF),
      static_cast<uint8_t>((addr >> 8) & 0xFF),
      static_cast<uint8_t>((addr >> 16) & 0xFF),
      static_cast<uint8_t>((addr >> 24) & 0xFF));
  return true;
}

static const char *wifiModeLabel(wifi_mode_t mode)
{
  switch (mode) {
    case WIFI_MODE_STA:
      return "STA";
    case WIFI_MODE_AP:
      return "AP";
    case WIFI_MODE_APSTA:
      return "APSTA";
    default:
      return "NULL";
  }
}

static void printWifiScanDiag(const char *stage, esp_err_t err, wifi_mode_t mode)
{
  Serial.printf(
      "[WIFI][SCAN] %s | err=%s | mode=%s(%d) | ap=%s | sta_req=%s | sta_conn=%s | heap=%u\n",
      stage ? stage : "-",
      esp_err_to_name(err),
      wifiModeLabel(mode),
      static_cast<int>(mode),
      g_wifi_ap_started ? "on" : "off",
      g_wifi_sta_requested ? "on" : "off",
      g_wifi_sta_connected ? "on" : "off",
      static_cast<unsigned>(ESP.getFreeHeap()));
}

static esp_err_t prepareWifiModeForScan(wifi_mode_t *original_mode_out, wifi_mode_t *scan_mode_out, bool *restore_mode_out)
{
  if (!original_mode_out || !scan_mode_out || !restore_mode_out) {
    return ESP_ERR_INVALID_ARG;
  }

  *original_mode_out = WIFI_MODE_NULL;
  *scan_mode_out = WIFI_MODE_NULL;
  *restore_mode_out = false;

  wifi_mode_t mode = WIFI_MODE_NULL;
  esp_err_t err = esp_wifi_get_mode(&mode);
  if (err != ESP_OK) {
    printWifiScanDiag("get_mode_fail", err, mode);
    return err;
  }

  *original_mode_out = mode;
  *scan_mode_out = mode;
  printWifiScanDiag("begin", ESP_OK, mode);

  if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
    return ESP_OK;
  }

  if (mode == WIFI_MODE_AP) {
    err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
      printWifiScanDiag("promote_apsta_fail", err, mode);
      return err;
    }
    delay(40);
    *scan_mode_out = WIFI_MODE_APSTA;
    *restore_mode_out = true;
    printWifiScanDiag("promote_apsta_ok", ESP_OK, WIFI_MODE_APSTA);
    return ESP_OK;
  }

  printWifiScanDiag("unsupported_mode", ESP_ERR_WIFI_MODE, mode);
  return ESP_ERR_WIFI_MODE;
}

static void restoreWifiModeAfterScan(wifi_mode_t original_mode, bool restore_mode)
{
  if (!restore_mode) {
    return;
  }

  const esp_err_t err = esp_wifi_set_mode(original_mode);
  if (err != ESP_OK) {
    printWifiScanDiag("restore_fail", err, original_mode);
    return;
  }

  delay(40);
  printWifiScanDiag("restore_ok", ESP_OK, original_mode);
  if (original_mode == WIFI_MODE_AP) {
    refreshWifiApIp();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP ativo em http://192.168.4.1");
  }
}

static int performWifiNetworkScan(char *summary_out, size_t summary_len)
{
  if (!summary_out || summary_len == 0) {
    return -1;
  }

  summary_out[0] = '\0';
  wifi_mode_t original_mode = WIFI_MODE_NULL;
  wifi_mode_t scan_mode = WIFI_MODE_NULL;
  bool restore_mode = false;
  esp_err_t err = prepareWifiModeForScan(&original_mode, &scan_mode, &restore_mode);
  if (err != ESP_OK) {
    snprintf(
        summary_out,
        summary_len,
        err == ESP_ERR_WIFI_MODE
            ? "Scan requer Wi-Fi ativo em AP ou LAN."
            : "Falha ao preparar scan: %s",
        esp_err_to_name(err));
    return (err == ESP_ERR_WIFI_MODE) ? -2 : -1;
  }

  wifi_scan_config_t scan_cfg = {};
  scan_cfg.show_hidden = true;
  scan_cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;

  err = esp_wifi_scan_start(&scan_cfg, true);
  if (err != ESP_OK) {
    printWifiScanDiag("scan_start_fail", err, scan_mode);
    restoreWifiModeAfterScan(original_mode, restore_mode);
    snprintf(summary_out, summary_len, "Falha ao iniciar scan: %s", esp_err_to_name(err));
    return -3;
  }

  uint16_t ap_count = 0;
  err = esp_wifi_scan_get_ap_num(&ap_count);
  if (err != ESP_OK) {
    printWifiScanDiag("get_ap_num_fail", err, scan_mode);
    restoreWifiModeAfterScan(original_mode, restore_mode);
    snprintf(summary_out, summary_len, "Falha ao obter total de redes");
    return -4;
  }

  if (ap_count == 0) {
    printWifiScanDiag("done_empty", ESP_OK, scan_mode);
    restoreWifiModeAfterScan(original_mode, restore_mode);
    snprintf(summary_out, summary_len, "Nenhuma rede encontrada agora.");
    return 0;
  }

  wifi_ap_record_t records[12];
  uint16_t fetch_count = ap_count;
  if (fetch_count > 12) {
    fetch_count = 12;
  }

  err = esp_wifi_scan_get_ap_records(&fetch_count, records);
  if (err != ESP_OK) {
    printWifiScanDiag("get_records_fail", err, scan_mode);
    restoreWifiModeAfterScan(original_mode, restore_mode);
    snprintf(summary_out, summary_len, "Falha ao listar redes: %s", esp_err_to_name(err));
    return -5;
  }

  size_t out = 0;
  for (uint16_t i = 0; i < fetch_count && out < (summary_len - 1U); ++i) {
    const char *ssid = reinterpret_cast<const char *>(records[i].ssid);
    if (i > 0 && out < (summary_len - 1U)) {
      summary_out[out++] = '\n';
    }
    const int written = snprintf(
        summary_out + out,
        summary_len - out,
        "%s | %d dBm | %s",
        (ssid && ssid[0]) ? ssid : "(oculta)",
        static_cast<int>(records[i].rssi),
        records[i].authmode == WIFI_AUTH_OPEN ? "aberta" : "senha");
    if (written < 0) {
      break;
    }
    const size_t used = static_cast<size_t>(written);
    if (used >= (summary_len - out)) {
      out = summary_len - 1U;
      break;
    }
    out += used;
  }

  printWifiScanDiag("done_ok", ESP_OK, scan_mode);
  restoreWifiModeAfterScan(original_mode, restore_mode);
  return static_cast<int>(ap_count);
}

static void startWifiPortal()
{
  updateWifiRequestedFlag();
  if (!g_wifi_mode_requested) {
    stopWifiPortal();
    return;
  }

  if (g_wifi_sta_requested && !g_wifi_sta_ssid[0]) {
    if (!g_wifi_ap_requested) {
      g_wifi_sta_requested = false;
      updateWifiRequestedFlag();
      snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "LAN sem SSID configurado");
      snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "Defina SSID e senha para a rede local");
      g_wifi_apply_pending = false;
      return;
    }
    snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "LAN ignorada: sem SSID configurado");
  }

  Serial.println("[WIFI] Aplicando modo AP/STA...");
  if (!ensureWifiStack()) {
    g_wifi_ap_requested = false;
    g_wifi_sta_requested = false;
    updateWifiRequestedFlag();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "Falha no stack Wi-Fi. Veja o serial.");
    g_wifi_apply_pending = false;
    return;
  }

  delay(100);
  esp_wifi_stop();
  g_wifi_ap_started = false;
  g_wifi_sta_started = false;
  g_wifi_sta_connected = false;
  g_wifi_web_started = false;
  g_wifi_ap_ip = IPAddress(0, 0, 0, 0);
  g_wifi_sta_ip = IPAddress(0, 0, 0, 0);

  const bool want_ap = g_wifi_ap_requested;
  const bool want_sta = g_wifi_sta_requested && g_wifi_sta_ssid[0];
  if (want_ap) {
    Serial.printf(
        "[WIFI] AP alvo | SSID: %s | Senha: %s | IP: 192.168.4.1 | Canal: %u\n",
        WIFI_AP_SSID,
        WIFI_AP_PASSWORD,
        static_cast<unsigned>(WIFI_AP_CHANNEL));

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = static_cast<uint32_t>(WIFI_AP_LOCAL_IP);
    ip_info.gw.addr = static_cast<uint32_t>(WIFI_AP_GATEWAY);
    ip_info.netmask.addr = static_cast<uint32_t>(WIFI_AP_SUBNET);
    esp_netif_dhcps_stop(g_wifi_ap_netif);
    esp_netif_set_ip_info(g_wifi_ap_netif, &ip_info);
    esp_netif_dhcps_start(g_wifi_ap_netif);
  }

  wifi_config_t ap_config = {};
  if (want_ap) {
    strncpy(reinterpret_cast<char *>(ap_config.ap.ssid), WIFI_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    strncpy(reinterpret_cast<char *>(ap_config.ap.password), WIFI_AP_PASSWORD, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.channel = WIFI_AP_CHANNEL;
    ap_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
    ap_config.ap.ssid_hidden = WIFI_AP_HIDDEN ? 1 : 0;
    ap_config.ap.authmode = strlen(WIFI_AP_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
  }

  wifi_config_t sta_config = {};
  if (want_sta) {
    strncpy(reinterpret_cast<char *>(sta_config.sta.ssid), g_wifi_sta_ssid, sizeof(sta_config.sta.ssid) - 1);
    strncpy(reinterpret_cast<char *>(sta_config.sta.password), g_wifi_sta_password, sizeof(sta_config.sta.password) - 1);
    sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
  }

  const wifi_mode_t mode = want_ap ? WIFI_MODE_APSTA : WIFI_MODE_STA;

  esp_err_t err = esp_wifi_set_mode(mode);
  if (err == ESP_OK && want_ap) {
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
  }
  if (err == ESP_OK && want_sta) {
    err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
  }
  if (err == ESP_OK) {
    err = esp_wifi_start();
  }
  if (err == ESP_OK && want_sta) {
    err = esp_wifi_connect();
  }
  if (err == ESP_OK) {
    err = esp_wifi_set_ps(WIFI_PS_NONE);
  }

  if (err != ESP_OK) {
    g_wifi_ap_started = false;
    g_wifi_sta_started = false;
    g_wifi_sta_connected = false;
    g_wifi_web_started = false;
    updateRuntimeServiceMode();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "Falha ao iniciar Wi-Fi. Veja o serial.");
    Serial.printf("[WIFI] Falha ao iniciar modo AP/STA: %s\n", esp_err_to_name(err));
    g_wifi_apply_pending = false;
    printWifiStatus("Falha ao iniciar");
    return;
  }

  delay(250);
  if (want_ap) {
    refreshWifiApIp();
  }
  if (want_sta) {
    refreshWifiStaIp();
  }

  g_wifi_ap_started = want_ap;
  g_wifi_sta_started = want_sta;
  updateWifiRequestedFlag();
  g_web_server.begin();
  g_web_server.setNoDelay(true);
  g_wifi_web_started = true;
  g_wifi_last_status_ms = millis();
  updateRuntimeServiceMode();

  char ap_ip_text[20];
  char sta_ip_text[20];
  formatWifiApIp(ap_ip_text, sizeof(ap_ip_text));
  formatWifiStaIp(sta_ip_text, sizeof(sta_ip_text));
  if (want_ap && want_sta) {
    snprintf(
        g_wifi_diag_note,
        sizeof(g_wifi_diag_note),
        "AP em http://%s | LAN %s",
        ap_ip_text,
        g_wifi_sta_connected ? sta_ip_text : "conectando");
  } else if (want_ap) {
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP ativo em http://%s", ap_ip_text);
  } else {
    snprintf(
        g_wifi_diag_note,
        sizeof(g_wifi_diag_note),
        g_wifi_sta_connected ? "LAN ativa em http://%s" : "LAN conectando em %s",
        g_wifi_sta_connected ? sta_ip_text : g_wifi_sta_ssid);
  }
  printWifiStatus("Wi-Fi ativo");
  g_wifi_apply_pending = false;
}

static void stopWifiPortal()
{
  wifi_mode_t mode = WIFI_MODE_NULL;
  esp_wifi_get_mode(&mode);
  if (!g_wifi_ap_started && !g_wifi_sta_started && !g_wifi_web_started && mode == WIFI_MODE_NULL) {
    updateRuntimeServiceMode();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "Wi-Fi desligado. Ligue em COMMS ou CONFIG.");
    return;
  }

  Serial.println("[WIFI] Desligando radio/AP/STA...");
  g_wifi_ap_requested = false;
  g_wifi_sta_requested = false;
  updateWifiRequestedFlag();
  g_web_server.stop();
  esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
  g_wifi_ap_started = false;
  g_wifi_sta_started = false;
  g_wifi_sta_connected = false;
  g_wifi_web_started = false;
  g_wifi_ap_ip = IPAddress(0, 0, 0, 0);
  g_wifi_sta_ip = IPAddress(0, 0, 0, 0);
  g_wifi_web_hits = 0;
  g_wifi_last_client_ms = 0;
  g_wifi_last_status_ms = 0;
  updateRuntimeServiceMode();
  snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "Wi-Fi desligado. Cockpit completo liberado.");
  snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "LAN desligada");
  g_wifi_apply_pending = false;
  printWifiStatus("Wi-Fi desligado");
}

static bool readHttpRequestLine(NetworkClient &client, char *buffer, size_t buffer_len)
{
  if (!buffer || buffer_len == 0) {
    return false;
  }

  size_t idx = 0;
  const uint32_t start_ms = millis();

  while ((millis() - start_ms) < WIFI_CLIENT_TIMEOUT_MS) {
    while (client.available()) {
      const char c = static_cast<char>(client.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        buffer[idx] = '\0';
        return idx > 0;
      }
      if (idx < (buffer_len - 1)) {
        buffer[idx++] = c;
      }
    }
    delay(1);
  }

  buffer[idx] = '\0';
  return idx > 0;
}

static void discardHttpHeaders(NetworkClient &client)
{
  const uint32_t start_ms = millis();
  bool blank_line = false;

  while ((millis() - start_ms) < WIFI_CLIENT_TIMEOUT_MS) {
    while (client.available()) {
      const char c = static_cast<char>(client.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        if (blank_line) {
          return;
        }
        blank_line = true;
      } else {
        blank_line = false;
      }
    }
    delay(1);
  }
}

static void sanitizeJsonText(const char *src, char *dst, size_t dst_len)
{
  if (!dst || dst_len == 0) {
    return;
  }

  if (!src) {
    dst[0] = '\0';
    return;
  }

  size_t out = 0;
  for (size_t i = 0; src[i] != '\0' && out < (dst_len - 1); ++i) {
    char c = src[i];
    if (c == '"' || c == '\\') {
      c = '\'';
    } else if (c == '\r' || c == '\n' || c == '\t') {
      c = ' ';
    }
    dst[out++] = c;
  }

  dst[out] = '\0';
}

static const char *screenIdSlug(AppScreenId screen_id)
{
  switch (screen_id) {
    case SCREEN_EFIS:
      return "plane";
    case SCREEN_METEO:
      return "meteo";
    case SCREEN_GPS:
      return "comms";
    case SCREEN_LORA:
      return "lora";
    case SCREEN_CONFIG:
      return "config";
    default:
      return "home";
  }
}

static const char *screenIdLabel(AppScreenId screen_id)
{
  switch (screen_id) {
    case SCREEN_EFIS:
      return "PLANE";
    case SCREEN_METEO:
      return "METEO";
    case SCREEN_GPS:
      return "COMMS";
    case SCREEN_LORA:
      return "LORA";
    case SCREEN_CONFIG:
      return "CONFIG";
    default:
      return "HOME";
  }
}

static bool parseHttpPath(const char *request_line, char *path_out, size_t path_out_len)
{
  if (!request_line || !path_out || path_out_len == 0) {
    return false;
  }

  const char *start = strstr(request_line, "GET ");
  if (!start) {
    return false;
  }
  start += 4;

  const char *end = strstr(start, " HTTP/");
  if (!end || end <= start) {
    return false;
  }

  size_t len = static_cast<size_t>(end - start);
  if (len >= path_out_len) {
    len = path_out_len - 1;
  }

  memcpy(path_out, start, len);
  path_out[len] = '\0';
  return true;
}

static int hexNibble(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return 10 + (c - 'A');
  }
  if (c >= 'a' && c <= 'f') {
    return 10 + (c - 'a');
  }
  return -1;
}

static void urlDecodeInPlace(char *text)
{
  if (!text) {
    return;
  }

  size_t read_idx = 0;
  size_t write_idx = 0;
  while (text[read_idx] != '\0') {
    char c = text[read_idx];
    if (c == '+') {
      text[write_idx++] = ' ';
      read_idx += 1;
      continue;
    }

    if (c == '%' && text[read_idx + 1] != '\0' && text[read_idx + 2] != '\0') {
      const int hi = hexNibble(text[read_idx + 1]);
      const int lo = hexNibble(text[read_idx + 2]);
      if (hi >= 0 && lo >= 0) {
        text[write_idx++] = static_cast<char>((hi << 4) | lo);
        read_idx += 3;
        continue;
      }
    }

    text[write_idx++] = c;
    read_idx += 1;
  }

  text[write_idx] = '\0';
}

static bool httpQueryValue(const char *path, const char *key, char *value_out, size_t value_out_len)
{
  if (!path || !key || !value_out || value_out_len == 0) {
    return false;
  }

  const char *query = strchr(path, '?');
  if (!query) {
    return false;
  }
  ++query;

  const size_t key_len = strlen(key);
  while (*query) {
    const char *segment_end = strchr(query, '&');
    if (!segment_end) {
      segment_end = path + strlen(path);
    }

    const char *eq = static_cast<const char *>(memchr(query, '=', segment_end - query));
    if (eq && static_cast<size_t>(eq - query) == key_len && strncmp(query, key, key_len) == 0) {
      size_t value_len = static_cast<size_t>(segment_end - eq - 1);
      if (value_len >= value_out_len) {
        value_len = value_out_len - 1;
      }
      memcpy(value_out, eq + 1, value_len);
      value_out[value_len] = '\0';
      urlDecodeInPlace(value_out);
      return true;
    }

    if (*segment_end == '\0') {
      break;
    }
    query = segment_end + 1;
  }

  return false;
}

static bool parseScreenIdFromSlug(const char *slug, AppScreenId *screen_out)
{
  if (!slug || !screen_out) {
    return false;
  }

  if (strcmp(slug, "home") == 0) {
    *screen_out = SCREEN_HOME;
    return true;
  }
  if (strcmp(slug, "plane") == 0) {
    *screen_out = SCREEN_EFIS;
    return true;
  }
  if (strcmp(slug, "meteo") == 0) {
    *screen_out = SCREEN_METEO;
    return true;
  }
  if (strcmp(slug, "comms") == 0) {
    *screen_out = SCREEN_GPS;
    return true;
  }
  if (strcmp(slug, "lora") == 0) {
    *screen_out = SCREEN_LORA;
    return true;
  }
  if (strcmp(slug, "config") == 0) {
    *screen_out = SCREEN_CONFIG;
    return true;
  }

  return false;
}

static void requestWifiMode(bool enable, const char *origin)
{
  g_wifi_ap_requested = enable;
  if (enable) {
    g_sd_purge_armed = false;
  }

  // Persiste a alteracao de modo
  saveWifiPrefs();

  updateWifiRequestedFlag();
  g_wifi_apply_pending = true;
  Serial.printf("[WIFI] Pedido via %s: %s\n", origin ? origin : "sistema", enable ? "ON" : "OFF");
  updateRuntimeServiceMode();
}

static void requestWifiStaMode(bool enable, const char *origin)
{
  g_wifi_sta_requested = enable;
  
  // Persiste a alteracao de modo
  saveWifiPrefs();

  updateWifiRequestedFlag();
  g_wifi_apply_pending = true;
  appendActivityLine("WIFI", enable ? "STA ON" : "STA OFF");
  Serial.printf("[WIFI] STA via %s: %s\n", origin ? origin : "sistema", enable ? "ON" : "OFF");
  updateRuntimeServiceMode();
}
static void saveWifiPrefs()
{
  Preferences prefs;
  if (!prefs.begin("wifi_cfg", false)) {
    Serial.println("[NVS] ERRO: nao foi possivel abrir wifi_cfg para escrita");
    return;
  }
  prefs.putString("ssid",     g_wifi_sta_ssid);
  prefs.putString("password", g_wifi_sta_password);
  prefs.putBool  ("ap_on",    g_wifi_ap_requested);
  prefs.putBool  ("sta_on",   g_wifi_sta_requested);
  prefs.end();
  Serial.printf("[NVS] Salvo: SSID=%s | AP=%d | STA=%d\n",
                g_wifi_sta_ssid, g_wifi_ap_requested, g_wifi_sta_requested);
}

static void loadWifiPrefs()
{
  Preferences prefs;
  if (!prefs.begin("wifi_cfg", true)) {
    Serial.println("[NVS] Sem dados Wi-Fi salvos, usando padrao AP");
    return;
  }
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("password", "");
  bool   ap   = prefs.getBool  ("ap_on",  true);   // padrão: AP ligado
  bool   sta  = prefs.getBool  ("sta_on", false);  // padrão: STA desligado
  prefs.end();

  if (ssid.length() > 0) {
    snprintf(g_wifi_sta_ssid,      sizeof(g_wifi_sta_ssid),     "%s", ssid.c_str());
    snprintf(g_wifi_sta_password,  sizeof(g_wifi_sta_password), "%s", pass.c_str());
  }
  g_wifi_ap_requested  = ap;
  g_wifi_sta_requested = sta && (ssid.length() > 0); // STA só se tiver SSID
  Serial.printf("[NVS] Carregado: SSID=%s | AP=%d | STA=%d\n",
                g_wifi_sta_ssid[0] ? g_wifi_sta_ssid : "-", ap, sta);
}

static void requestWifiStaCredentials(const char *ssid, const char *password, const char *origin)
{
  if (ssid) {
    snprintf(g_wifi_sta_ssid, sizeof(g_wifi_sta_ssid), "%s", ssid);
  }
  if (password) {
    snprintf(g_wifi_sta_password, sizeof(g_wifi_sta_password), "%s", password);
  }

  // Persiste na flash NVS para sobreviver ao reboot
  saveWifiPrefs();

  if (g_wifi_sta_ssid[0]) {
    snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "Credenciais LAN prontas para %s", g_wifi_sta_ssid);
    appendActivityLine("WIFI", g_wifi_sta_ssid);
  } else {
    snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "SSID LAN limpo");
    appendActivityLine("WIFI", "SSID limpo");
  }
  Serial.printf("[WIFI] Credenciais STA atualizadas via %s | SSID=%s\n", origin ? origin : "sistema", g_wifi_sta_ssid[0] ? g_wifi_sta_ssid : "-");
}

static void requestWifiNetworkScan(const char *origin)
{
  g_wifi_scan_summary = "Preparando scan...";
  g_wifi_scan_count = 0;
  g_wifi_scan_ms = millis();

  if (!ensureWifiStack()) {
    g_wifi_scan_summary = "Falha ao preparar stack Wi-Fi.";
    Serial.printf("[WIFI] Scan via %s falhou: stack indisponivel\n", origin ? origin : "sistema");
    return;
  }

  char summary[720];
  const int n = performWifiNetworkScan(summary, sizeof(summary));
  if (n <= 0) {
    g_wifi_scan_summary = summary;
    g_wifi_scan_count = 0;
    appendActivityLine("WIFI", "scan vazio");
    Serial.printf("[WIFI] Scan via %s sem redes | note=%s\n", origin ? origin : "sistema", summary);
    return;
  }

  g_wifi_scan_summary = summary;
  g_wifi_scan_count = static_cast<uint8_t>((n > 255) ? 255 : n);
  appendActivityLine("WIFI", "scan OK");
  Serial.printf("[WIFI] Scan via %s concluiu | redes=%d\n", origin ? origin : "sistema", n);
}

static void requestLoraMode(bool enable, const char *origin)
{
  g_lora_enabled = enable;
  if (g_lora_uart_ready) {
    digitalWrite(LORA_M0_PIN, LOW);
    digitalWrite(LORA_M1_PIN, LOW);
  }
  LoraState state = copyLoraState();
  state.enabled = enable;
  state.uart_ready = g_lora_uart_ready;
  state.aux_high = g_lora_uart_ready ? (digitalRead(LORA_AUX_PIN) == HIGH) : false;
  state.auto_tx_enabled = g_lora_auto_tx_enabled;
  state.auto_tx_interval_ms = g_lora_auto_tx_interval_ms;
  state.last_auto_tx_ms = g_lora_auto_tx_last_ms;
  snprintf(state.auto_tx_mode, sizeof(state.auto_tx_mode), "%s", g_lora_auto_tx_mode);
  snprintf(
      state.note,
      sizeof(state.note),
      enable ? "LoRa habilitado | aguardando trafego" : "LoRa desabilitado");
  storeLoraState(state);
  appendActivityLine("LORA", enable ? "ON" : "OFF");
  Serial.printf("[LORA] Pedido via %s: %s\n", origin ? origin : "sistema", enable ? "ON" : "OFF");
}

static void requestCommsRescan(const char *origin)
{
  g_blackbox_remount_requested = true;
  refreshSensorCaches();
  appendActivityLine("SYS", "rescan solicitado");
  Serial.printf("[COMMS] Rescan solicitado via %s\n", origin ? origin : "sistema");
  printWifiStatus("Rescan");
}

static void requestLoggerToggle(const char *origin)
{
  g_blackbox_enabled_target = !g_blackbox_enabled_target;
  appendActivityLine("SD", g_blackbox_enabled_target ? "logger ON" : "logger OFF");
  Serial.printf("[BLACKBOX] Toggle via %s: %s\n", origin ? origin : "sistema", g_blackbox_enabled_target ? "ON" : "OFF");
}

static void requestEfisLevelSet(const char *origin)
{
  const ImuState imu = copyImuState();
  float pitch_trim_deg = 0.0f;
  float roll_trim_deg = 0.0f;
  getImuTrim(&pitch_trim_deg, &roll_trim_deg);
  setImuTrim(pitch_trim_deg + imu.pitch_deg, roll_trim_deg + imu.roll_deg);
  Serial.printf("[EFIS] Nivel ajustado via %s\n", origin ? origin : "sistema");
}

static void requestScreenLoad(AppScreenId screen_id, const char *origin)
{
  if (g_current_screen_id != screen_id) {
    loadScreenById(screen_id);
    Serial.printf("[UI] Tela alterada via %s para %s\n", origin ? origin : "sistema", screenIdLabel(screen_id));
  }
}

static void handleHttpActionPath(const char *path)
{
  if (!path) {
    return;
  }

  char value[96];
  char sta_ssid[WIFI_STA_SSID_LEN];
  char sta_pass[WIFI_STA_PASSWORD_LEN];
  const bool has_sta_ssid = httpQueryValue(path, "sta_ssid", sta_ssid, sizeof(sta_ssid));
  const bool has_sta_pass = httpQueryValue(path, "sta_pass", sta_pass, sizeof(sta_pass));
  if (has_sta_ssid || has_sta_pass) {
    requestWifiStaCredentials(has_sta_ssid ? sta_ssid : nullptr, has_sta_pass ? sta_pass : nullptr, "web");
  }

  if (httpQueryValue(path, "screen", value, sizeof(value))) {
    AppScreenId screen_id = SCREEN_HOME;
    if (parseScreenIdFromSlug(value, &screen_id)) {
      requestScreenLoad(screen_id, "web");
    }
  }

  if (httpQueryValue(path, "wifi", value, sizeof(value))) {
    if (strcmp(value, "on") == 0) {
      requestWifiMode(true, "web");
    } else if (strcmp(value, "off") == 0) {
      requestWifiMode(false, "web");
    } else if (strcmp(value, "toggle") == 0) {
      requestWifiMode(!g_wifi_ap_requested, "web");
    }
  }

  if (httpQueryValue(path, "sta", value, sizeof(value))) {
    if (strcmp(value, "on") == 0) {
      requestWifiStaMode(true, "web");
    } else if (strcmp(value, "off") == 0) {
      requestWifiStaMode(false, "web");
    } else if (strcmp(value, "toggle") == 0) {
      requestWifiStaMode(!g_wifi_sta_requested, "web");
    } else if (strcmp(value, "forget") == 0) {
      requestWifiStaCredentials("", "", "web");
      requestWifiStaMode(false, "web");
    }
  }

  if (httpQueryValue(path, "lora", value, sizeof(value))) {
    if (strcmp(value, "on") == 0) {
      requestLoraMode(true, "web");
    } else if (strcmp(value, "off") == 0) {
      requestLoraMode(false, "web");
    } else if (strcmp(value, "toggle") == 0) {
      requestLoraMode(!g_lora_enabled, "web");
    }
  }

  if (httpQueryValue(path, "gps_rate", value, sizeof(value))) {
    if (strcmp(value, "1") == 0) {
      requestGpsRateHz(1, "web");
    } else if (strcmp(value, "2") == 0) {
      requestGpsRateHz(2, "web");
    } else if (strcmp(value, "4") == 0) {
      requestGpsRateHz(4, "web");
    } else if (strcmp(value, "5") == 0) {
      requestGpsRateHz(5, "web");
    } else if (strcmp(value, "10") == 0) {
      requestGpsRateHz(10, "web");
    }
  }

  if (httpQueryValue(path, "gps_restart", value, sizeof(value))) {
    if (strcmp(value, "hot") == 0) {
      requestGpsRestartMode(0, "web");
    } else if (strcmp(value, "warm") == 0) {
      requestGpsRestartMode(1, "web");
    } else if (strcmp(value, "cold") == 0) {
      requestGpsRestartMode(2, "web");
    } else if (strcmp(value, "factory") == 0) {
      requestGpsRestartMode(3, "web");
    }
  }

  if (httpQueryValue(path, "gps_constellation", value, sizeof(value))) {
    if (strcmp(value, "gps") == 0) {
      requestGpsConstellationMode(1, "web");
    } else if (strcmp(value, "gps_bds") == 0) {
      requestGpsConstellationMode(3, "web");
    } else if (strcmp(value, "gps_glo") == 0) {
      requestGpsConstellationMode(5, "web");
    } else if (strcmp(value, "all") == 0) {
      requestGpsConstellationMode(7, "web");
    }
  }

  if (httpQueryValue(path, "gps_nmea", value, sizeof(value))) {
    if (strcmp(value, "all_on") == 0) {
      requestGpsAllNmea(true, "web");
    } else if (strcmp(value, "all_off") == 0) {
      requestGpsAllNmea(false, "web");
    }
  }

  if (httpQueryValue(path, "lora_tx", value, sizeof(value))) {
    if (strcmp(value, "ping") == 0) {
      requestLoraTestSend("PING STRATOSBRAIN", "web");
    } else if (strcmp(value, "status") == 0) {
      requestLoraTestSend("STATUS?", "web");
    } else if (strcmp(value, "summary") == 0) {
      requestLoraTelemetrySend("summary", "web");
    } else if (strcmp(value, "meteo") == 0) {
      requestLoraTelemetrySend("meteo", "web");
    } else if (strcmp(value, "gps") == 0) {
      requestLoraTelemetrySend("gps", "web");
    }
  }

  if (httpQueryValue(path, "lora_auto", value, sizeof(value))) {
    if (strcmp(value, "on") == 0) {
      requestLoraAutoTxConfig(true, nullptr, "web");
    } else if (strcmp(value, "off") == 0) {
      requestLoraAutoTxConfig(false, nullptr, "web");
    } else if (strcmp(value, "toggle") == 0) {
      requestLoraAutoTxConfig(!g_lora_auto_tx_enabled, nullptr, "web");
    }
  }

  if (httpQueryValue(path, "lora_auto_mode", value, sizeof(value))) {
    if (strcmp(value, "summary") == 0 || strcmp(value, "meteo") == 0 || strcmp(value, "gps") == 0) {
      requestLoraAutoTxConfig(g_lora_auto_tx_enabled, value, "web");
    }
  }

  if (httpQueryValue(path, "lora_payload", value, sizeof(value))) {
    requestLoraTestSend(value, "web");
  }

  if (httpQueryValue(path, "lora_clear", value, sizeof(value))) {
    if (strcmp(value, "1") == 0) {
      clearLoraHistory();
      LoraState state = copyLoraState();
      snprintf(state.note, sizeof(state.note), "Console LoRa limpo via web");
      storeLoraState(state);
      appendActivityLine("LORA", "console limpo");
      Serial.println("[LORA] Console web limpo");
    }
  }

  if (httpQueryValue(path, "logger", value, sizeof(value))) {
    if (strcmp(value, "toggle") == 0) {
      requestLoggerToggle("web");
    }
  }

  if (httpQueryValue(path, "rescan", value, sizeof(value))) {
    if (strcmp(value, "1") == 0) {
      requestCommsRescan("web");
    }
  }

  if (httpQueryValue(path, "sd", value, sizeof(value))) {
    if (strcmp(value, "remount") == 0) {
      g_blackbox_remount_requested = true;
      appendActivityLine("SD", "remount solicitado");
      Serial.println("[BLACKBOX] Remount solicitado via web");
    }
  }

  if (httpQueryValue(path, "level", value, sizeof(value))) {
    if (strcmp(value, "1") == 0) {
      requestEfisLevelSet("web");
    }
  }
}

static void sendHttpJsonStatus(NetworkClient &client)
{
  const ImuState imu = copyImuState();
  const BlackboxState blackbox = copyBlackboxState();
  const Bme688State bme = copyBme688State();
  const GpsState gps = copyGpsState();
  const LoraState lora = copyLoraState();
  HttpJsonScratch &scratch = g_http_json_scratch;
  formatWifiVisibleIp(scratch.ip_text, sizeof(scratch.ip_text));
  formatWifiApIp(scratch.ap_ip_text, sizeof(scratch.ap_ip_text));
  formatWifiStaIp(scratch.sta_ip_text, sizeof(scratch.sta_ip_text));
  sanitizeJsonText(blackbox.file_path[0] ? blackbox.file_path : "-", scratch.blackbox_file, sizeof(scratch.blackbox_file));
  sanitizeJsonText(baseFileName(blackbox.file_path), scratch.blackbox_name, sizeof(scratch.blackbox_name));
  sanitizeJsonText(blackbox.note, scratch.blackbox_note, sizeof(scratch.blackbox_note));
  formatBlackboxTailJson(scratch.blackbox_tail, sizeof(scratch.blackbox_tail));
  formatBlackboxFilesJson(scratch.blackbox_files, sizeof(scratch.blackbox_files));
  sanitizeJsonText(g_wifi_diag_note, scratch.wifi_note, sizeof(scratch.wifi_note));
  sanitizeJsonText(g_wifi_sta_note, scratch.wifi_sta_note, sizeof(scratch.wifi_sta_note));
  sanitizeJsonText(g_wifi_sta_ssid, scratch.wifi_sta_ssid, sizeof(scratch.wifi_sta_ssid));
  sanitizeJsonText(g_wifi_scan_summary.c_str(), scratch.wifi_scan_summary, sizeof(scratch.wifi_scan_summary));
  sanitizeJsonText(bme.note, scratch.bme_note, sizeof(scratch.bme_note));
  sanitizeJsonText(gps.note, scratch.gps_note, sizeof(scratch.gps_note));
  sanitizeJsonText(gps.last_sentence, scratch.gps_sentence, sizeof(scratch.gps_sentence));
  sanitizeJsonText(gps.config_note, scratch.gps_config_note, sizeof(scratch.gps_config_note));
  sanitizeJsonText(gps.last_command, scratch.gps_last_command, sizeof(scratch.gps_last_command));
  formatGpsActivityJson(scratch.gps_history, sizeof(scratch.gps_history));
  sanitizeJsonText(lora.note, scratch.lora_note, sizeof(scratch.lora_note));
  sanitizeJsonText(lora.last_message, scratch.lora_message, sizeof(scratch.lora_message));
  sanitizeJsonText(lora.last_rx_message, scratch.lora_last_rx, sizeof(scratch.lora_last_rx));
  sanitizeJsonText(lora.last_tx_payload, scratch.lora_last_tx, sizeof(scratch.lora_last_tx));
  sanitizeJsonText(lora.auto_tx_mode, scratch.lora_auto_mode, sizeof(scratch.lora_auto_mode));
  formatLoraHistoryJson(scratch.lora_history, sizeof(scratch.lora_history));
  formatLoraExportText(scratch.lora_export, sizeof(scratch.lora_export));
  formatActivityHistoryJson(scratch.activity_history, sizeof(scratch.activity_history));
  classifyMeteoTheme(bme, scratch.meteo_theme, sizeof(scratch.meteo_theme), scratch.meteo_summary, sizeof(scratch.meteo_summary));
  sanitizeJsonText(g_sensor_compact_cache.c_str(), scratch.sensor_summary, sizeof(scratch.sensor_summary));
  sanitizeJsonText(g_sensor_summary_cache.c_str(), scratch.sensor_known, sizeof(scratch.sensor_known));
  sanitizeJsonText(g_sensor_scan_compact_cache.c_str(), scratch.sensor_scan_raw, sizeof(scratch.sensor_scan_raw));
  sanitizeJsonText(g_sensor_bosch_cache.c_str(), scratch.sensor_bosch, sizeof(scratch.sensor_bosch));
  sanitizeJsonText(g_sensor_route_cache.c_str(), scratch.sensor_routes, sizeof(scratch.sensor_routes));

  const bool plane_has_altitude = gps.has_fix;
  const bool plane_has_speed = gps.receiving || gps.has_fix;
  const float plane_altitude_ft = plane_has_altitude ? (gps.altitude_m * 3.28084f) : 0.0f;
  const float plane_speed_kmh = plane_has_speed ? gps.speed_kmh : 0.0f;
  snprintf(scratch.plane_altitude_ft_text, sizeof(scratch.plane_altitude_ft_text), plane_has_altitude ? "%.1f" : "null", plane_altitude_ft);
  snprintf(scratch.plane_speed_kmh_text, sizeof(scratch.plane_speed_kmh_text), plane_has_speed ? "%.1f" : "null", plane_speed_kmh);

  if (gps.has_location) {
    snprintf(
        scratch.gps_map_osm,
        sizeof(scratch.gps_map_osm),
        "https://www.openstreetmap.org/?mlat=%.6f&mlon=%.6f#map=16/%.6f/%.6f",
        gps.latitude_deg,
        gps.longitude_deg,
        gps.latitude_deg,
        gps.longitude_deg);
    snprintf(
        scratch.gps_map_google,
        sizeof(scratch.gps_map_google),
        "https://www.google.com/maps?q=%.6f,%.6f",
        gps.latitude_deg,
        gps.longitude_deg);
    snprintf(scratch.gps_map_hint, sizeof(scratch.gps_map_hint), "Mapa pronto. Abra no celular quando houver internet na rede.");
  } else if (gps.receiving) {
    snprintf(scratch.gps_map_osm, sizeof(scratch.gps_map_osm), "");
    snprintf(scratch.gps_map_google, sizeof(scratch.gps_map_google), "");
    snprintf(scratch.gps_map_hint, sizeof(scratch.gps_map_hint), "GPS recebendo NMEA, mas ainda sem fix. Teste com visao aberta do ceu.");
  } else {
    snprintf(scratch.gps_map_osm, sizeof(scratch.gps_map_osm), "");
    snprintf(scratch.gps_map_google, sizeof(scratch.gps_map_google), "");
    snprintf(scratch.gps_map_hint, sizeof(scratch.gps_map_hint), "Sem posicao valida ainda. Confira alimentacao, TX/RX e visao do ceu.");
  }
  snprintf(scratch.gps_power_hint, sizeof(scratch.gps_power_hint), "GP10 modulo nu = 3V3. Breakout DX-PJ17 = VCC 5V. WAKE alto ou solto.");
  snprintf(scratch.gps_wiring_hint, sizeof(scratch.gps_wiring_hint), "TXD GP10 -> GPIO1 | RXD GP10 -> GPIO2 | GND comum | 9600 8N1 | 1PPS opcional.");

  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: application/json; charset=utf-8\r\n"));
  client.print(F("Cache-Control: no-store\r\n"));
  client.print(F("Connection: close\r\n\r\n"));
  client.printf(
      "{\"device\":\"StratosBrain S3\",\"screen\":\"%s\",\"screen_label\":\"%s\",\"runtime_light\":%s,",
      screenIdSlug(g_current_screen_id),
      screenIdLabel(g_current_screen_id),
      g_runtime_services_light ? "true" : "false");
  client.printf(
      "\"wifi\":{\"mode\":\"%s\",\"ap\":%s,\"requested\":%s,\"ap_requested\":%s,\"ssid\":\"%s\",\"password\":\"%s\",\"ap_ip\":\"%s\",\"sta_requested\":%s,\"sta_connected\":%s,\"sta_ssid\":\"%s\",\"sta_ip\":\"%s\",\"ip\":\"%s\",\"clients\":%u,\"web_hits\":%lu,\"scan_count\":%u,\"note\":\"%s\",\"sta_note\":\"%s\",\"scan_summary\":\"%s\"},",
      wifiModeLabel(),
      g_wifi_ap_started ? "true" : "false",
      g_wifi_mode_requested ? "true" : "false",
      g_wifi_ap_requested ? "true" : "false",
      WIFI_AP_SSID,
      WIFI_AP_PASSWORD,
      scratch.ap_ip_text,
      g_wifi_sta_requested ? "true" : "false",
      g_wifi_sta_connected ? "true" : "false",
      scratch.wifi_sta_ssid,
      scratch.sta_ip_text,
      scratch.ip_text,
      static_cast<unsigned>(wifiClientCount()),
      static_cast<unsigned long>(g_wifi_web_hits),
      static_cast<unsigned>(g_wifi_scan_count),
      scratch.wifi_note,
      scratch.wifi_sta_note,
      scratch.wifi_scan_summary);
  client.printf("\"uptime_s\":%lu,", static_cast<unsigned long>(millis() / 1000UL));
  client.printf(
      "\"touch\":{\"pressed\":%s,\"x\":%u,\"y\":%u,\"count\":%lu},",
      g_touch_pressed ? "true" : "false",
      static_cast<unsigned>(g_touch_x),
      static_cast<unsigned>(g_touch_y),
      static_cast<unsigned long>(g_touch_press_count));
  client.printf(
      "\"imu\":{\"connected\":%s,\"healthy\":%s,\"pitch_deg\":%.1f,\"roll_deg\":%.1f,\"temperature_c\":%.1f},",
      imu.connected ? "true" : "false",
      imu.healthy ? "true" : "false",
      imu.pitch_deg,
      imu.roll_deg,
      imu.temperature_c);
  client.printf(
      "\"plane\":{\"avg_altitude_m\":%.1f,\"avg_vs_ms\":%.2f,\"avg_speed_kmh\":%.1f},",
      g_plane_smooth_alt,
      g_plane_smooth_vs,
      g_plane_smooth_spd);
  client.printf(
      "\"meteo\":{\"theme\":\"%s\",\"summary\":\"%s\",\"connected\":%s,\"has_data\":%s,\"temperature_c\":%.1f,\"humidity_pct\":%.1f,\"pressure_hpa\":%.1f,\"altitude_m\":%.1f,\"gas_ohms\":%.0f,\"last_update_ms\":%lu,\"note\":\"%s\"},",
      scratch.meteo_theme,
      scratch.meteo_summary,
      bme.connected ? "true" : "false",
      bme.has_data ? "true" : "false",
      bme.temperature_c,
      bme.humidity_pct,
      bme.pressure_hpa,
      bme.altitude_m,
      bme.gas_ohms,
      static_cast<unsigned long>(bme.last_update_ms),
      scratch.bme_note);
  client.print("\"sats_arr\":[");
  for (uint8_t i = 0; i < gps.sats_count; i++) {
    client.printf("{\"p\":%u,\"e\":%u,\"a\":%u,\"s\":%u,\"t\":\"%c\"}%s", 
      gps.sats_info[i].prn, gps.sats_info[i].elv, gps.sats_info[i].az, gps.sats_info[i].snr, gps.sats_info[i].type,
      (i == gps.sats_count - 1) ? "" : ",");
  }
  client.print("],");
  client.printf(
      "\"gps\":{\"uart_ready\":%s,\"receiving\":%s,\"fix\":%s,\"has_location\":%s,\"sats\":%u,\"rx_bytes\":%lu,\"line_count\":%lu,\"last_rx_ms\":%lu,\"last_fix_ms\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"alt_m\":%.1f,\"speed_kmh\":%.1f,\"update_hz\":%u,\"last_sentence\":\"%s\",\"note\":\"%s\",\"config_note\":\"%s\",\"last_command\":\"%s\",\"power_hint\":\"%s\",\"wiring_hint\":\"%s\",\"map_osm\":\"%s\",\"map_google\":\"%s\",\"map_hint\":\"%s\",\"history\":\"%s\"},",
      gps.uart_ready ? "true" : "false",
      gps.receiving ? "true" : "false",
      gps.has_fix ? "true" : "false",
      gps.has_location ? "true" : "false",
      static_cast<unsigned>(gps.sats),
      static_cast<unsigned long>(gps.rx_bytes),
      static_cast<unsigned long>(gps.line_count),
      static_cast<unsigned long>(gps.last_rx_ms),
      static_cast<unsigned long>(gps.last_fix_ms),
      gps.latitude_deg,
      gps.longitude_deg,
      gps.altitude_m,
      gps.speed_kmh,
      static_cast<unsigned>(gps.update_rate_hz),
      scratch.gps_sentence,
      scratch.gps_note,
      scratch.gps_config_note,
      scratch.gps_last_command,
      scratch.gps_power_hint,
      scratch.gps_wiring_hint,
      scratch.gps_map_osm,
      scratch.gps_map_google,
      scratch.gps_map_hint,
      scratch.gps_history);
  client.printf(
      "\"lora\":{\"enabled\":%s,\"uart_ready\":%s,\"aux_high\":%s,\"receiving\":%s,\"rx_bytes\":%lu,\"tx_bytes\":%lu,\"last_rx_ms\":%lu,\"last_tx_ms\":%lu,\"last_message\":\"%s\",\"last_rx_message\":\"%s\",\"last_tx_payload\":\"%s\",\"auto_tx_enabled\":%s,\"auto_tx_interval_ms\":%lu,\"last_auto_tx_ms\":%lu,\"auto_tx_mode\":\"%s\",\"note\":\"%s\",\"history\":\"%s\",\"export\":\"%s\",\"console_note\":\"Console de payload UART LoRa. Nao e analisador SDR/RF.\"},",
      lora.enabled ? "true" : "false",
      lora.uart_ready ? "true" : "false",
      lora.aux_high ? "true" : "false",
      lora.receiving ? "true" : "false",
      static_cast<unsigned long>(lora.rx_bytes),
      static_cast<unsigned long>(lora.tx_bytes),
      static_cast<unsigned long>(lora.last_rx_ms),
      static_cast<unsigned long>(lora.last_tx_ms),
      scratch.lora_message,
      scratch.lora_last_rx,
      scratch.lora_last_tx,
      lora.auto_tx_enabled ? "true" : "false",
      static_cast<unsigned long>(lora.auto_tx_interval_ms),
      static_cast<unsigned long>(lora.last_auto_tx_ms),
      scratch.lora_auto_mode,
      scratch.lora_note,
      scratch.lora_history,
      scratch.lora_export);
  client.printf("\"activity\":{\"recent\":\"%s\"},", scratch.activity_history);
  client.printf(
      "\"comms\":{\"lora_enabled\":%s,\"lora_uart_ready\":%s,\"gps_fix\":%s,\"gps_uart_ready\":%s,\"gps_receiving\":%s},",
      g_lora_enabled ? "true" : "false",
      g_lora_uart_ready ? "true" : "false",
      gps.has_fix ? "true" : "false",
      g_gps_uart_ready ? "true" : "false",
      gps.receiving ? "true" : "false");
  client.printf(
      "\"lab\":{\"bme_ok\":%s,\"gps_ok\":%s,\"lora_ok\":%s},",
      bme.has_data ? "true" : "false",
      gps.receiving ? "true" : "false",
      lora.uart_ready ? "true" : "false");
  client.printf(
      "\"blackbox\":{\"mounted\":%s,\"logging_enabled\":%s,\"file_open\":%s,\"last_write_ok\":%s,\"records\":%lu,\"last_log_ms\":%lu,\"card_size_bytes\":%llu,\"file\":\"%s\",\"file_name\":\"%s\",\"note\":\"%s\",\"tail\":\"%s\",\"files\":\"%s\"},",
      blackbox.mounted ? "true" : "false",
      blackbox.logging_enabled ? "true" : "false",
      blackbox.file_open ? "true" : "false",
      blackbox.last_write_ok ? "true" : "false",
      static_cast<unsigned long>(blackbox.records_written),
      static_cast<unsigned long>(blackbox.last_log_ms),
      static_cast<unsigned long long>(blackbox.card_size_bytes),
      scratch.blackbox_file,
      scratch.blackbox_name,
      scratch.blackbox_note,
      scratch.blackbox_tail,
      scratch.blackbox_files);
  client.printf(
      "\"ui\":{\"rotation\":\"%s\",\"orientation\":\"%s\"},",
      rotationLabel(g_display_rotation),
      orientationModeLabel(g_orientation_mode));
  client.printf(
      "\"sensors\":{\"summary\":\"%s\",\"known\":\"%s\",\"scan\":\"%s\",\"bosch\":\"%s\",\"routes\":\"%s\"}}\n",
      scratch.sensor_summary,
      scratch.sensor_known,
      scratch.sensor_scan_raw,
      scratch.sensor_bosch,
      scratch.sensor_routes);
}

static void sendHttpDashboard(NetworkClient &client)
{
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
  client.print(F(R"SBWEB(<!DOCTYPE html><html translate="no"><head><meta charset='utf-8'><meta name="google" content="notranslate">
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>StratosBrain S3 Hub</title>
<style>
:root{--bg:#071019;--panel:#0d1823;--line:#21445f;--text:#eef6ff;--muted:#9ab2c7;--accent:#6fd6ff;--ok:#41d39b;--warn:#ffb357;--danger:#ff7a59}
*{box-sizing:border-box}body{margin:0;font-family:Arial,sans-serif;background:linear-gradient(180deg,#06111a,#09131d 38%,#05090e);color:var(--text);overscroll-behavior:none}
.wrap{padding:16px;max-width:1180px;margin:0 auto}
.hero{background:linear-gradient(180deg,#102235,#0b1622);border:1px solid #1d5b86;border-radius:20px;padding:18px;margin-bottom:14px;box-shadow:0 10px 30px rgba(0,0,0,.28)}
.hero h1{margin:6px 0 8px;font-size:34px}.lead{color:var(--muted);font-size:15px;line-height:1.5}
.grid,.hero-grid,.split{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.mini,.card{background:var(--panel);border:1px solid #21364a;border-radius:16px;padding:14px;transition:opacity .2s}
.mini b,.card h3{margin:0 0 8px;font-size:13px;color:var(--accent);letter-spacing:.08em;text-transform:uppercase}
.actions,.tabs{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}
.btn{appearance:none;border:1px solid #255f85;background:#112131;color:#eef6ff;border-radius:12px;padding:10px 14px;font-weight:700;cursor:pointer}
.btn.alt{border-color:#386749;background:#11261c}.btn.warn{border-color:#87592d;background:#2b1d12}.btn.small{padding:8px 12px;font-size:13px}
.tab{background:#0c1722;border:1px solid #20384d}.tab.active{background:#143048;border-color:#4abfff}
.section{display:none}.section.active{display:block}.big{font-size:30px;font-weight:700;margin:4px 0 6px}.muted{color:var(--muted);font-size:14px}.mono{font-family:monospace}
.pills{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
.pill{padding:5px 10px;border-radius:999px;border:1px solid #275779;background:#123046;font-size:12px;font-weight:700}
.pill.ok{border-color:#2f7b5a;background:#10281d;color:#d8ffe9}.pill.warn{border-color:#8a6a34;background:#2d2110;color:#ffe7b5}
input{width:100%;background:#08131d;border:1px solid #21445f;border-radius:10px;color:#eef6ff;padding:10px;margin-top:8px}
pre{margin:0;white-space:pre-wrap;word-break:break-word;background:#08131d;border:1px solid #1d3245;border-radius:12px;padding:12px;min-height:100px;color:#d7edf7}
#skyplot{background:#070f15;border-radius:50%;width:100%;max-width:300px;aspect-ratio:1;display:block;margin:10px auto;border:1px solid #21364a}
.ttip{position:relative;display:inline-block;cursor:help;border-bottom:1px dotted #6fd6ff}
</style></head><body><div class='wrap'>
<div class='hero'><h1>StratosBrain Hub</h1><div class='lead'>Estacao de telemetria AJAX suave.</div>
<div class='hero-grid'>
<div class='mini'><b>Wi-Fi AP</b><span id='heroAp'>--</span></div><div class='mini'><b>LAN Local</b><span id='heroLan'>--</span></div>
<div class='mini'><b>IP</b><span class='mono' id='heroIp'>--</span></div>
</div><div class='actions'><button class='btn alt' onclick="act('wifi=toggle')">Wi-Fi AP</button><button class='btn small' onclick="act('logger=toggle')">Logger SD</button><button class='btn warn' onclick="act('lora=toggle')">LoRa RX</button></div></div>
<div class='tabs'>
<button class='btn tab active' data-tab='lab' onclick="setTab('lab')">Dashboard</button>
<button class='btn tab' data-tab='gps' onclick="setTab('gps')">GPS</button>
<button class='btn tab' data-tab='sd' onclick="setTab('sd')">Cartão SD</button>
<button class='btn tab' data-tab='meteo' onclick="setTab('meteo')">Meteorologia</button>
<button class='btn tab' data-tab='comms' onclick="setTab('comms')">Rede</button>
<button class='btn tab' data-tab='lora' onclick="setTab('lora')">LoRa</button>
<button class='btn tab' data-tab='config' onclick="setTab('config')">Configuração</button>
<button class='btn tab' data-tab='plane' onclick="setTab('plane')">Aviação</button>
<button class='btn tab' data-tab='overview' onclick="setTab('overview')">Visão geral</button>
</div>)SBWEB"));
  client.print(F(R"SBWEB(
<section class='section active' id='section-lab'><div class='split'>
<div class='card'><h3>BME688 Status</h3><div class='big' id='labBmeState'>--</div><div class='muted'>Pitch/Roll: <span id='labImuPitch'>--</span> / <span id='labImuRoll'>--</span><br>SD: <span id='labSdLogger'>--</span></div></div>
<div class='card'><h3>GPS NMEA</h3><div class='big' id='labGpsState'>--</div><div class='muted'>Fix: <span id='labGpsFix'>--</span><br>Sats: <span id='labGpsSats'>--</span><br>Lat/Lon: <span class='mono' id='labGpsPos'>--</span><br>Ultima: <span class='mono' id='labGpsSentence'>--</span></div></div>
<div class='card'><h3>LoRa Status</h3><div class='big' id='labLoraState'>--</div><div class='muted'>Tráfego RX: <span id='lrRx2'>--</span> | TX: <span id='lrTx2'>--</span><br>Pinos: TX17 RX18 AUX6<br>Ultima lida: <span class='mono' id='lrMsg2'>--</span></div><div class='actions'><button class='btn small' onclick="act('lora_tx=ping')">Polo Ping</button></div></div>
</div></section>
<section class='section' id='section-plane'><div class='split'>
<div class='card'><h3>Cockpit Virtual</h3><div class='muted'>Altitude e velocidade combinadas BME688 + GPS com filtro EMA.</div></div>
<div class='card'><h3>Alt / ASL media</h3><div class='big' style='font-size:42px;color:#FFD34D' id='pltAlt'>--</div></div>
<div class='card'><h3>Vel. Vertical</h3><div class='big' style='font-size:42px;color:#45D1FF' id='pltVs'>--</div></div>
<div class='card'><h3>Velocidade (GPS)</h3><div class='big' style='font-size:42px;color:#FFB357' id='pltSpd'>--</div></div>
<div class='card'><h3>Horizonte Artificial (Web)</h3><canvas id='ahCanvas' style='display:block;margin:0 auto;border-radius:12px;background:#0a1520;border:1px solid #21364a' width='220' height='120'></canvas><div class='muted' style='margin-top:6px;text-align:center'>Pitch: <span id='ahPitch'>--</span> &nbsp; Roll: <span id='ahRoll'>--</span></div></div>
</div></section>
<section class='section' id='section-gps'><div class='split'>
<div class='card'><h3>Skyplot Constelacoes</h3><div style='display:flex;gap:8px;flex-wrap:wrap;margin-bottom:6px'><span class='pill ok' id='skpTotal'>-- sats</span><span class='pill' style='border-color:#41d39b;color:#41d39b' id='skpGps'>GPS 0</span><span class='pill' style='border-color:#c872ff;color:#c872ff' id='skpGlo'>GLO 0</span><span class='pill' style='border-color:#ff7a59;color:#ff7a59' id='skpBds'>BDS 0</span><span class='pill' style='border-color:#6fd6ff;color:#6fd6ff' id='skpGal'>GAL 0</span></div><canvas id='skyplot'></canvas></div>
<div class='card'><h3>Dados do Voo</h3><div class='big' id='gpsSpeed'>--</div><div class='muted'>Loc: <span class='mono' id='gpsLatLon'>--</span><br>Alt: <span id='gpsAlt'>--</span><br>Atualizacao: <span id='gpsRate'>--</span><br>Ultimo RX: <span id='gpsLastRx'>--</span></div><div class='actions'><a class='btn small' id='gpsOsmLink' href='#' target='_blank'>OpenStreetMap</a><a class='btn small' id='gpsGoogleLink' href='#' target='_blank'>Google Maps</a></div></div>
<div class='card'><h3>Controles GP10 (CASIC)</h3><div class='actions'>
<button class='btn small' onclick="act('gps_restart=cold')" title="Cold Start: apaga almanaques, reinicia busca cega. Demora achar FIX.">Partida a frio</button>
<button class='btn small' onclick="act('gps_restart=hot')" title="Hot Start: aproveita memórias, fix super rápido se moveu pouco.">Inicio Rapido</button>
<button class='btn small' onclick="act('gps_rate=1')" title="1 atualização de posição por segundo.">1Hz</button>
<button class='btn small' onclick="act('gps_rate=5')" title="5 atualizações por segundo. (Atenção ao tráfego UART)">5Hz</button>
</div><div class='actions' style='margin-top:6px;'>
<button class='btn small' onclick="act('gps_constellation=all')" title="Habilita GPS + BDS + GLONASS. Máquina consome +energia.">Todos os GNSS</button>
<button class='btn small' onclick="act('gps_constellation=gps_bds')" title="Habilita GPS + BeiDou apenas. Ideal na nossa região.">GPS+BDS</button>
<button class='btn small' onclick="act('gps_nmea=all_on')" title="Volta a emitir todas de GSV/GSA.">NMEA Full</button>
</div><div class='muted' style='margin-top:10px'>Comandos despachados ao modulo UART1.</div></div>
<div class='card'><h3>Atividade recente GPS</h3><pre id='gpsHistory'>--</pre></div>
<div class='card'><h3>Terminal Serial GPS</h3><pre id='gpsSentence'>--</pre></div>
</div></section>
<section class='section' id='section-sd'><div class='split'>
<div class='card'><h3>Status do SD</h3><div class='big' id='sdMounted'>--</div><div class='muted'>Logger: <span id='sdLogger'>--</span><br>Arquivo ativo: <span class='mono' id='sdFile'>--</span><br>Registros: <span id='sdRecords'>--</span><br>Capacidade: <span id='sdSize'>--</span></div><div class='actions'><button class='btn small' onclick="act('sd=remount')">Remount SD</button><a class='btn small' href='/download/blackbox' target='_blank'>Baixar log ativo</a></div></div>
<div class='card'><h3>Arquivos do SD</h3><pre id='sdFiles'>--</pre></div>
<div class='card' style='grid-column:1/-1'><h3>Ultimas linhas</h3><pre id='sdTail'>--</pre></div>
</div></section>
<section class='section' id='section-meteo'><div class='split'>
<div class='card' style='grid-column:1/-1'><h3>Clima</h3><div class='big' id='mtTheme'>--</div><div class='muted' id='mtSummary'>--</div><div class='footer'>Gás bruto: <span id='mtGas'>--</span></div></div>
<div class='card'><h3>Altitude</h3><div class='big' id='mtAlt'>--</div><div class='muted'>estimada por pressão</div></div>
<div class='card'><h3>Pressão</h3><div class='big' id='mtPress'>--</div><div class='muted'>barometria</div></div>
<div class='card'><h3>Temperatura</h3><div class='big' id='mtTemp'>--</div><div class='muted'>ambiente</div></div>
<div class='card'><h3>Umidade</h3><div class='big' id='mtHum'>--</div><div class='muted'>relativa</div></div>
</div></section>
<section class='section' id='section-lora'><div class='split'>
<div class='card'><h3>LoRa Escuta (Sniffer)</h3><div class='big' id='lrState'>--</div><div class='muted'>Interceptando pacotes sem filtro.<br>Ultima msg decodificada: <span class='mono' id='lrMsg'>--</span></div><div class='actions'><button class='btn warn' onclick="act('lora=toggle')">Ligar/Desligar LoRa</button><button class='btn small' onclick="act('lora_tx=status')">Despachar STATUS</button></div></div>
<div class='card'><h3>Envelopes RF TX</h3><input id='lrPayload' type='text' placeholder='Ex: ALVO=ALPHA,T=33'><button class='btn small' style='margin-top:8px' onclick="sendLoraPayload()">Fogo</button><div class='actions'><button class='btn small' onclick="act('lora_tx=meteo')">Enviar METEO</button><button class='btn small' onclick="act('lora_tx=gps')">Enviar GPS</button><a class='btn small' href='/download/lora' target='_blank'>Exportar console</a></div></div>
<div class='card'><h3>Resumo RX/TX</h3><div class='muted'>RX: <span id='lrRxBytes'>--</span><br>TX: <span id='lrTxBytes'>--</span><br>Nota: <span id='lrNote'>--</span></div></div>
<div class='card' style='grid-column:1/-1'><h3>Terminal Interceptacao</h3><pre id='lrConsole'>--</pre></div>
</div></section>
<section class='section' id='section-comms'><div class='split'>
<div class='card'><h3>Status Wi-Fi</h3><div class='muted' style='color:#ffb357'>Aviso: ao conectar no Wi-Fi residencial, o AP (192.168.4.1) sera desligado. Acesse pelo novo IP!</div><div class='muted' style='margin-top:8px'>AP Clients: <span id='cmClients'>--</span> | Hits: <span id='cmHits'>--</span></div></div>
<div class='card'><h3>Redes Disponiveis</h3><button class='btn small' onclick='scanWifi()'>Varrer Redes Wi-Fi</button><div id='wifiList' style='margin-top:10px;display:flex;flex-direction:column;gap:6px'><div class='muted'>Clique em Varrer para listar redes.</div></div></div>
<div class='card'><h3>Conectar na Rede</h3><input id='cmStaSsid' type='text' placeholder='SSID (ou clique na lista acima)'><input id='cmStaPass' type='password' placeholder='Senha'><div class='actions'><button class='btn alt' onclick='connectSta()'>Conectar</button><button class='btn small warn' onclick="act('sta=off')">Desconectar</button></div></div>
</div></section>
<section class='section' id='section-config'><div class='split'>
<div class='card'><h3>Configuração</h3><div class='muted'>Tela ativa: <span id='cfScreen'>--</span><br>Modo: <span id='cfMode'>--</span><br>Orientação: <span id='cfOrientation'>--</span><br>Rotação: <span id='cfRotation'>--</span></div></div>
<div class='card'><h3>SD e sensores</h3><div class='muted'>Arquivo: <span class='mono' id='cfFile'>--</span><br>Registros: <span id='cfRecords'>--</span><br>Nota SD: <span id='cfNote'>--</span><br>Sensores: <span id='cfSensors'>--</span><br>Scan I2C: <span class='mono' id='cfScan'>--</span></div></div>
</div></section>
<section class='section' id='section-overview'><div class='grid'>
<div class='card'><h3>Uptime</h3><div class='big' id='ovUptime'>--</div><div class='muted'>Hits web: <span id='ovHits'>--</span><br>Clientes AP: <span id='ovClients'>--</span></div></div>
<div class='card'><h3>IMU</h3><div class='big' id='ovImu'>--</div><div class='muted'>Pitch: <span id='ovPitch'>--</span><br>Roll: <span id='ovRoll'>--</span><br>Temp: <span id='ovTemp'>--</span></div></div>
<div class='card'><h3>Touch</h3><div class='big' id='ovTouch'>--</div><div class='muted'>X/Y: <span id='ovTouchXY'>--</span><br>Toques: <span id='ovTouchCount'>--</span></div></div>
<div class='card'><h3>Blackbox</h3><div class='big' id='ovSd'>--</div><div class='muted'>Modo: <span id='ovLogger'>--</span><br>Arquivo: <span class='mono' id='ovFile'>--</span><br>Registros: <span id='ovRecords'>--</span></div></div>
<div class='card'><h3>Atividade recente</h3><pre id='ovActivity'>--</pre></div>
</div></section>)SBWEB"));

  client.print(F(R"SBWEB(
<script>
const $=i=>document.getElementById(i), txt=(v,d='--')=>(v===undefined||v===null||v==='')?d:v, toText=v=>String(txt(v));
const put=(i,v)=>{const e=$(i); if(!e)return; const next=toText(v); if(e.textContent!==next)e.textContent=next;};
const val=(i,v)=>{const e=$(i); const next=toText(v); if(e&&document.activeElement!==e&&(!e.value||e.dataset.user!=='1')&&e.value!==next)e.value=next;};
const num=(v,s='')=>(v===undefined||v===null||Number.isNaN(v))?'--':String(v)+s, yes=(v,a='ON',b='OFF')=>v?a:b;
function linkify(id,href,label){const e=$(id); if(!e)return; const ok=!!(href&&href!==''&&href!=='--'); e.href=ok?href:'#'; e.style.pointerEvents=ok?'auto':'none'; e.style.opacity=ok?'1':'.5'; if(label&&e.textContent!==label)e.textContent=label;}
function setTab(n){document.querySelectorAll('.section').forEach(s=>s.classList.toggle('active',s.id==='section-'+n));document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('active',b.dataset.tab===n));}
function act(q){fetch('/api/action?'+q).then(()=>fetchData()).catch(e=>console.log("Action Fetch Err",e));}
function connectSta(){const ssid=$('cmStaSsid').value.trim(), pass=$('cmStaPass').value; if(ssid) act('sta_ssid='+encodeURIComponent(ssid)+'&sta_pass='+encodeURIComponent(pass)+'&sta=on');}
function sendLoraPayload(){const p=$('lrPayload').value.trim(); if(p) act('lora_payload='+encodeURIComponent(p));}
['cmStaSsid','cmStaPass'].forEach(id=>{const e=$(id); if(e)e.addEventListener('input',()=>e.dataset.user='1');});
function scanWifi(){
  const box=$('wifiList'); if(box)box.innerHTML="<div class='muted'>Varrendo...</div>";
  fetch('/api/scan').then(r=>r.json()).then(d=>{
    if(!d.networks||!d.networks.length){if(box)box.innerHTML="<div class='muted'>Nenhuma rede encontrada.</div>";return;}
    if(box)box.innerHTML=d.networks.map(n=>{
      const bars=n.rssi>-60?'▂▄▆█':n.rssi>-75?'▂▄▆':n.rssi>-85?'▂▄':'▂';
      const lock=n.enc?'🔒':'';
      return `<div style='display:flex;justify-content:space-between;align-items:center;padding:7px 10px;background:#0b1a28;border:1px solid #21364a;border-radius:10px;cursor:pointer' onclick='selectNetwork("${n.ssid.replace(/"/g,'&quot;')}")'>` +
        `<span style='font-weight:700'>${n.ssid}</span><span style='color:var(--muted);font-size:13px'>${lock} ${bars} ${n.rssi}dBm</span></div>`;
    }).join('');
  }).catch(()=>{if(box)box.innerHTML="<div class='muted'>Erro ao varrer.</div>";});}
function selectNetwork(ssid){const e=$('cmStaSsid');if(e){e.value=ssid;e.dataset.user='1';}$('cmStaPass').focus();}

function plotSky(sats){const cvs=$('skyplot'); if(!cvs)return; const r=cvs.width=cvs.offsetWidth, h=cvs.height=cvs.offsetHeight, ctx=cvs.getContext('2d'), cx=r/2, cy=h/2, maxR=cx*0.9;
ctx.clearRect(0,0,r,h); ctx.strokeStyle='#21445f'; ctx.lineWidth=1;
[1,0.66,0.33].forEach(f=>{ctx.beginPath(); ctx.arc(cx,cy,maxR*f,0,2*Math.PI); ctx.stroke();});
for(let i=0;i<12;i++){ctx.beginPath();const a=i*30*Math.PI/180; ctx.moveTo(cx,cy); ctx.lineTo(cx+Math.cos(a)*maxR, cy+Math.sin(a)*maxR); ctx.stroke();}
if(!sats||!sats.length)return;
sats.forEach(s=>{
  const el=Math.max(0,Math.min(90,s.e)), az=s.a||0, dist=maxR*(90-el)/90, ang=(az-90)*Math.PI/180, px=cx+Math.cos(ang)*dist, py=cy+Math.sin(ang)*dist;
  let c='#41d39b'; if(s.t==='R')c='#c872ff'; else if(s.t==='B')c='#ff7a59'; else if(s.t==='E')c='#6fd6ff';
  ctx.beginPath(); ctx.arc(px,py,s.s>0?6:4,0,2*Math.PI); ctx.fillStyle=s.s>0?c:'rgba(150,150,150,0.5)'; ctx.fill(); ctx.strokeStyle='#000'; ctx.stroke();
  ctx.fillStyle='#fff'; ctx.font='10px Arial'; ctx.textAlign='center'; ctx.textBaseline='middle'; ctx.fillText(s.p, px, py-10);
});}

function drawAH(pitch, roll){
  const cvs=document.getElementById('ahCanvas'); if(!cvs)return;
  const w=cvs.width,h=cvs.height,ctx=cvs.getContext('2d'),cx=w/2,cy=h/2;
  ctx.save(); ctx.clearRect(0,0,w,h);
  ctx.translate(cx,cy); ctx.rotate(-roll*Math.PI/180);
  const pitchPx=pitch*2;
  // Sky
  ctx.fillStyle='#1a6fb5'; ctx.fillRect(-w,-h/2+pitchPx,w*2,h);
  // Ground
  ctx.fillStyle='#7a4a1e'; ctx.fillRect(-w,pitchPx,w*2,h);
  // Horizon line
  ctx.strokeStyle='#fff'; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(-w,pitchPx); ctx.lineTo(w,pitchPx); ctx.stroke();
  // Bank angle ticks
  [30,60].forEach(a=>{
    ctx.save(); ctx.rotate(a*Math.PI/180);
    ctx.strokeStyle='rgba(255,255,255,0.5)'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(0,-cy+10); ctx.lineTo(0,-cy+20); ctx.stroke(); ctx.restore();
    ctx.save(); ctx.rotate(-a*Math.PI/180);
    ctx.beginPath(); ctx.moveTo(0,-cy+10); ctx.lineTo(0,-cy+20); ctx.stroke(); ctx.restore();
  });
  ctx.restore();
  // Center marker
  ctx.strokeStyle='#FFD34D'; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(cx-30,cy); ctx.lineTo(cx-10,cy); ctx.lineTo(cx-10,cy+8); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx+30,cy); ctx.lineTo(cx+10,cy); ctx.lineTo(cx+10,cy+8); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx,cy,4,0,2*Math.PI); ctx.fillStyle='#FFD34D'; ctx.fill();
}

let g_ah_pitch=0,g_ah_roll=0;

function fetchData(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    put('heroAp',yes(d.wifi.ap,'AP Ligado','AP Inativo')); put('heroLan',d.wifi.sta_connected?'WIFI OK @ '+txt(d.wifi.sta_ip):'Sem LAN'); put('heroIp','http://'+txt(d.wifi.ip));
    put('labBmeState',d.meteo.connected?'Ligado':'Off'); put('labImuPitch',num(d.imu.pitch_deg,' deg')); put('labImuRoll',num(d.imu.roll_deg,' deg')); put('labSdLogger',yes(d.blackbox.logging_enabled,'Gravando','Pausado'));
    put('labGpsState',d.gps.fix?'3D Fix':(d.gps.receiving?'RX...':'Wait')); put('labGpsFix',txt(d.gps.sats)+' sats'); put('labGpsSats',txt(d.gps.sats)); put('labGpsPos',d.gps.has_location?(num(d.gps.lat,'')+', '+num(d.gps.lon,'')):'--'); put('labGpsMotion',num(d.gps.speed_kmh,' km/h')); put('labGpsSentence',txt(d.gps.last_sentence));
    put('labLoraState',yes(d.lora.enabled,'Sniffer ON','Off')); put('lrRx2',txt(d.lora.rx_bytes)); put('lrTx2',txt(d.lora.tx_bytes)); put('lrMsg2',txt(d.lora.last_message));
    put('gpsSpeed',num(d.gps.speed_kmh,' km/h')); put('gpsLatLon',d.gps.has_location?(num(d.gps.lat,'')+', '+num(d.gps.lon,'')):'--'); put('gpsAlt',num(d.gps.alt_m,' m')); put('gpsRate',txt(d.gps.update_hz)+' Hz'); put('gpsLastRx',txt(d.gps.last_rx_ms)+' ms'); put('gpsSentence',txt(d.gps.last_sentence)); put('gpsHistory',txt(d.gps.history).replaceAll('\\n','\n'));
    linkify('gpsOsmLink',txt(d.gps.map_osm,''),'OpenStreetMap'); linkify('gpsGoogleLink',txt(d.gps.map_google,''),'Google Maps');
    if(d.sats_arr){
      plotSky(d.sats_arr);
      const total=d.sats_arr.length;
      const gps=d.sats_arr.filter(s=>s.t==='G').length;
      const glo=d.sats_arr.filter(s=>s.t==='R').length;
      const bds=d.sats_arr.filter(s=>s.t==='B').length;
      const gal=d.sats_arr.filter(s=>s.t==='E').length;
      put('skpTotal',total+' sats visíveis'); put('skpGps','GPS '+gps); put('skpGlo','GLO '+glo); put('skpBds','BDS '+bds); put('skpGal','GAL '+gal);
    }
    put('sdMounted',yes(d.blackbox.mounted,'montado','sem cartao')); put('sdLogger',yes(d.blackbox.logging_enabled,'ligado','pausado')); put('sdFile',txt(d.blackbox.file_name)); put('sdRecords',txt(d.blackbox.records)); put('sdSize',txt(d.blackbox.card_size_bytes)); put('sdTail',txt(d.blackbox.tail).replaceAll('\\n','\n')); put('sdFiles',txt(d.blackbox.files).replaceAll('\\n','\n'));
    put('lrState',yes(d.lora.enabled,'LIGADO','DESLIGADO')); put('lrMsg',txt(d.lora.last_message)); put('lrConsole',txt(d.lora.history).replaceAll('\\n','\n')); put('lrRxBytes',txt(d.lora.rx_bytes)); put('lrTxBytes',txt(d.lora.tx_bytes)); put('lrNote',txt(d.lora.note));
    put('cmClients',txt(d.wifi.clients)); put('cmHits',txt(d.wifi.web_hits)); val('cmStaSsid',txt(d.wifi.sta_ssid,''));
    put('mtTheme',txt(d.meteo.theme)); put('mtSummary',txt(d.meteo.summary)); put('mtTemp',num(d.meteo.temperature_c,' C')); put('mtHum',num(d.meteo.humidity_pct,' %')); put('mtPress',num(d.meteo.pressure_hpa,' hPa')); put('mtAlt',num(d.meteo.altitude_m,' m')); put('mtGas',num(d.meteo.gas_ohms,' ohms'));
    put('cfScreen',txt(d.screen_label)); put('cfMode',d.runtime_light?'leve':'normal'); put('cfOrientation',txt(d.ui.orientation)); put('cfRotation',txt(d.ui.rotation)); put('cfFile',txt(d.blackbox.file)); put('cfRecords',txt(d.blackbox.records)); put('cfNote',txt(d.blackbox.note)); put('cfSensors',txt(d.sensors.known)); put('cfScan',txt(d.sensors.scan));
    put('ovUptime',num(d.uptime_s,' s')); put('ovHits',txt(d.wifi.web_hits)); put('ovClients',txt(d.wifi.clients)); put('ovImu',d.imu.connected?'OK':'OFF'); put('ovPitch',num(d.imu.pitch_deg,' deg')); put('ovRoll',num(d.imu.roll_deg,' deg')); put('ovTemp',num(d.imu.temperature_c,' C')); put('ovTouch',yes(d.touch.pressed,'ON','OFF')); put('ovTouchXY',txt(d.touch.x)+','+txt(d.touch.y)); put('ovTouchCount',txt(d.touch.count)); put('ovSd',yes(d.blackbox.mounted,'montado','sem cartao')); put('ovLogger',yes(d.blackbox.logging_enabled,'ligado','pausado')); put('ovFile',txt(d.blackbox.file_name)); put('ovRecords',txt(d.blackbox.records)); put('ovActivity',txt(d.activity.recent).replaceAll('\\n','\n'));
    
    // Plane (Smoothed) + Artificial Horizon
    if(d.plane) {
        put('pltAlt', num(d.plane.avg_altitude_m, ' m'));
        put('pltVs', num(d.plane.avg_vs_ms, ' m/s'));
        put('pltSpd', num(d.plane.avg_speed_kmh, ' km/h'));
    }
    if(d.imu){
        const p=d.imu.pitch_deg||0, r=d.imu.roll_deg||0;
        put('ahPitch', num(p,' °')); put('ahRoll', num(r,' °'));
        drawAH(p,r);
    }
  }).catch(()=>{});
}
setTab('lab'); fetchData(); setInterval(fetchData, 1800);
</script></div></body></html>)SBWEB"));
}

static void sendHttpDashboardV2(NetworkClient &client)
{
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
  client.print(F(R"STRATOSWEB(<!DOCTYPE html><html translate="no"><head><meta charset='utf-8'><meta name="google" content="notranslate">
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>StratosBrain Hub</title>
<style>
:root{--bg:#071019;--panel:#0d1823;--panel-2:#0b1520;--line:#21445f;--text:#eef6ff;--muted:#9ab2c7;--accent:#6fd6ff;--ok:#41d39b;--warn:#ffb357;--danger:#ff7a59}
*{box-sizing:border-box}body{margin:0;font-family:Arial,sans-serif;background:linear-gradient(180deg,#06111a,#09131d 38%,#05090e);color:var(--text);overscroll-behavior:none}
.wrap{padding:16px;max-width:1220px;margin:0 auto}.hero{background:linear-gradient(180deg,#102235,#0b1622);border:1px solid #1d5b86;border-radius:20px;padding:18px;margin-bottom:14px;box-shadow:0 10px 30px rgba(0,0,0,.28)}
.hero h1{margin:6px 0 8px;font-size:34px}.lead{color:var(--muted);font-size:15px;line-height:1.5}
.hero-grid,.section-grid,.split,.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:12px}
.mini,.card{background:var(--panel);border:1px solid #21364a;border-radius:16px;padding:14px;transition:opacity .2s}
.mini b,.card h3{display:block;margin:0 0 8px;font-size:13px;color:var(--accent);letter-spacing:.08em;text-transform:uppercase}
.toolbar,.tabs,.actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}
.btn{appearance:none;border:1px solid #255f85;background:#112131;color:#eef6ff;border-radius:12px;padding:10px 14px;font-weight:700;cursor:pointer;text-decoration:none}
.btn.alt{border-color:#386749;background:#11261c}.btn.warn{border-color:#87592d;background:#2b1d12}.btn.danger{border-color:#823843;background:#2b1418}.btn.small{padding:8px 12px;font-size:13px}
.tab{background:#0c1722;border:1px solid #20384d}.tab.active{background:#143048;border-color:#4abfff}.tab[aria-selected='true']{background:#143048;border-color:#4abfff}
.section{display:none}.section.active{display:block}.big{font-size:30px;font-weight:700;margin:4px 0 6px}.muted{color:var(--muted);font-size:14px;line-height:1.5}.mono{font-family:Consolas,monospace}
.pill{display:inline-flex;align-items:center;padding:5px 10px;border-radius:999px;border:1px solid #275779;background:#123046;font-size:12px;font-weight:700}.pill.ok{border-color:#2f7b5a;background:#10281d;color:#d8ffe9}.pill.warn{border-color:#8a6a34;background:#2d2110;color:#ffe7b5}
.notice{margin-top:10px;padding:10px 12px;border-radius:12px;border:1px solid #3a586f;background:#0d1b28;color:#cfe7f7;font-size:13px}.notice.warn{border-color:#8a6a34;background:#2d2110;color:#ffe7b5}
.listbox{display:flex;flex-direction:column;gap:8px;margin-top:10px}
.network-item{display:flex;justify-content:space-between;align-items:center;gap:10px;width:100%;padding:10px 12px;border:1px solid #21364a;border-radius:12px;background:#0b1a28;color:#eef6ff;text-align:left;cursor:pointer}.network-item:hover{border-color:#4abfff}.network-meta{color:var(--muted);font-size:12px}
.kv{display:grid;grid-template-columns:120px 1fr;gap:6px 10px;align-items:start}.kv b{color:#b8dfff;font-size:12px;letter-spacing:.05em;text-transform:uppercase}
.full{grid-column:1/-1}
label{display:block;margin-top:10px;color:#b8dfff;font-size:12px;font-weight:700;letter-spacing:.05em;text-transform:uppercase}
input{width:100%;background:#08131d;border:1px solid #21445f;border-radius:10px;color:#eef6ff;padding:10px;margin-top:6px}
pre{margin:0;white-space:pre-wrap;word-break:break-word;background:#08131d;border:1px solid #1d3245;border-radius:12px;padding:12px;min-height:110px;color:#d7edf7}
#skyplot{background:#070f15;border-radius:50%;width:100%;max-width:300px;aspect-ratio:1;display:block;margin:10px auto;border:1px solid #21364a}
#ahCanvas{display:block;margin:0 auto;border-radius:12px;background:#0a1520;border:1px solid #21364a}
a.btn{display:inline-flex;align-items:center;justify-content:center}
</style></head><body><div class='wrap'>
<div class='hero'>
  <h1>StratosBrain Hub</h1>
  <div class='lead'>Painel web do ESP32 para telemetria, log local e enlaces LoRa/Wi-Fi.</div>
  <div class='hero-grid'>
    <div class='mini'><b>Wi-Fi AP</b><span id='heroAp'>--</span></div>
    <div class='mini'><b>LAN Local</b><span id='heroLan'>--</span></div>
    <div class='mini'><b>URL ativa</b><span class='mono' id='heroIp'>--</span></div>
    <div class='mini'><b>Runtime</b><span id='heroMode'>--</span></div>
  </div>
  <div class='toolbar'>
    <button class='btn alt' type='button' onclick="act('wifi=toggle')">Wi-Fi AP</button>
    <button class='btn small' type='button' onclick="act('logger=toggle')">Logger SD</button>
    <button class='btn warn' type='button' onclick="act('lora=toggle')">LoRa UART</button>
  </div>
</div>
<div class='tabs' role='tablist' aria-label='Abas do painel web'>
  <button class='btn tab active' type='button' role='tab' aria-selected='true' data-tab='lab' onclick="setTab('lab')">Dashboard</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='gps' onclick="setTab('gps')">GPS</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='sd' onclick="setTab('sd')">Cartao SD</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='meteo' onclick="setTab('meteo')">Meteorologia</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='comms' onclick="setTab('comms')">Rede</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='lora' onclick="setTab('lora')">LoRa</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='config' onclick="setTab('config')">Configuracao</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='plane' onclick="setTab('plane')">Aviacao</button>
  <button class='btn tab' type='button' role='tab' aria-selected='false' data-tab='overview' onclick="setTab('overview')">Visao geral</button>
</div>
<section class='section active' id='section-lab'><div class='section-grid'>
  <div class='card'><h3>BME688</h3><div class='big' id='labBmeState'>--</div><div class='muted'>Pitch/Roll: <span id='labImuPitch'>--</span> / <span id='labImuRoll'>--</span><br>Logger SD: <span id='labSdLogger'>--</span></div></div>
  <div class='card'><h3>GPS NMEA</h3><div class='big' id='labGpsState'>--</div><div class='muted'>Fix: <span id='labGpsFix'>--</span><br>Satelites: <span id='labGpsSats'>--</span><br>Lat/Lon: <span class='mono' id='labGpsPos'>--</span><br>Ultima: <span class='mono' id='labGpsSentence'>--</span></div></div>
  <div class='card'><h3>LoRa UART</h3><div class='big' id='labLoraState'>--</div><div class='muted'>RX: <span id='lrRx2'>--</span> | TX: <span id='lrTx2'>--</span><br>AUX: <span id='lrAux2'>--</span><br>Ultimo trafego: <span class='mono' id='lrMsg2'>--</span></div><div class='toolbar'><button class='btn small' type='button' onclick="act('lora_tx=summary')">Enviar resumo</button></div></div>
</div></section>
<section class='section' id='section-plane'><div class='section-grid'>
  <div class='card'><h3>Cockpit virtual</h3><div class='muted'>Altitude e velocidade filtradas a partir do Bosch + GPS.</div></div>
  <div class='card'><h3>Altitude media</h3><div class='big' style='font-size:42px;color:#FFD34D' id='pltAlt'>--</div></div>
  <div class='card'><h3>Velocidade vertical</h3><div class='big' style='font-size:42px;color:#45D1FF' id='pltVs'>--</div></div>
  <div class='card'><h3>Velocidade GPS</h3><div class='big' style='font-size:42px;color:#FFB357' id='pltSpd'>--</div></div>
  <div class='card'><h3>Horizonte artificial (web)</h3><canvas id='ahCanvas' width='220' height='120'></canvas><div class='muted' style='margin-top:6px;text-align:center'>Pitch: <span id='ahPitch'>--</span> | Roll: <span id='ahRoll'>--</span></div></div>
</div></section>
<section class='section' id='section-gps'><div class='section-grid'>
  <div class='card'><h3>Skyplot constelacoes</h3><div style='display:flex;gap:8px;flex-wrap:wrap;margin-bottom:6px'><span class='pill ok' id='skpTotal'>-- sats</span><span class='pill' style='border-color:#41d39b;color:#41d39b' id='skpGps'>GPS 0</span><span class='pill' style='border-color:#c872ff;color:#c872ff' id='skpGlo'>GLO 0</span><span class='pill' style='border-color:#ff7a59;color:#ff7a59' id='skpBds'>BDS 0</span><span class='pill' style='border-color:#6fd6ff;color:#6fd6ff' id='skpGal'>GAL 0</span></div><canvas id='skyplot'></canvas></div>
  <div class='card'><h3>Dados do voo</h3><div class='big' id='gpsSpeed'>--</div><div class='muted'>Loc: <span class='mono' id='gpsLatLon'>--</span><br>Alt: <span id='gpsAlt'>--</span><br>Atualizacao: <span id='gpsRate'>--</span><br>Ultimo RX: <span id='gpsLastRx'>--</span></div><div class='toolbar'><a class='btn small' id='gpsOsmLink' href='#' target='_blank'>OpenStreetMap</a><a class='btn small' id='gpsGoogleLink' href='#' target='_blank'>Google Maps</a></div></div>
  <div class='card'><h3>Controles GP10</h3><div class='toolbar'><button class='btn small' type='button' onclick="act('gps_restart=cold')">Partida a frio</button><button class='btn small' type='button' onclick="act('gps_restart=hot')">Inicio rapido</button><button class='btn small' type='button' onclick="act('gps_rate=1')">1Hz</button><button class='btn small' type='button' onclick="act('gps_rate=5')">5Hz</button></div><div class='toolbar' style='margin-top:6px'><button class='btn small' type='button' onclick="act('gps_constellation=all')">Todos os GNSS</button><button class='btn small' type='button' onclick="act('gps_constellation=gps_bds')">GPS+BDS</button><button class='btn small' type='button' onclick="act('gps_nmea=all_on')">NMEA full</button></div><div class='muted' style='margin-top:10px'>Aba mantida estavel para diagnostico do GPS.</div></div>
  <div class='card'><h3>Atividade recente GPS</h3><pre id='gpsHistory'>--</pre></div>
  <div class='card'><h3>Terminal serial GPS</h3><pre id='gpsSentence'>--</pre></div>
</div></section>
<section class='section' id='section-sd'><div class='section-grid'>
  <div class='card'><h3>Status do SD</h3><div class='big' id='sdMounted'>--</div><div class='muted'>Logger: <span id='sdLogger'>--</span><br>Arquivo ativo: <span class='mono' id='sdFile'>--</span><br>Registros: <span id='sdRecords'>--</span><br>Capacidade: <span id='sdSize'>--</span></div><div class='toolbar'><button class='btn small' type='button' onclick="act('sd=remount')">Remount SD</button><a class='btn small' href='/download/blackbox' target='_blank'>Baixar log ativo</a></div></div>
  <div class='card'><h3>Arquivos do SD</h3><pre id='sdFiles'>--</pre></div>
  <div class='card full'><h3>Ultimas linhas</h3><pre id='sdTail'>--</pre></div>
</div></section>
<section class='section' id='section-meteo'><div class='section-grid'>
  <div class='card full'><h3>Clima</h3><div class='big' id='mtTheme'>--</div><div class='muted' id='mtSummary'>--</div><div class='notice'>Gas bruto: <span id='mtGas'>--</span></div></div>
  <div class='card'><h3>Altitude Bosch</h3><div class='big' id='mtAlt'>--</div><div class='muted'>estimada por pressao</div></div>
  <div class='card'><h3>Pressao</h3><div class='big' id='mtPress'>--</div><div class='muted'>barometria</div></div>
  <div class='card'><h3>Temperatura</h3><div class='big' id='mtTemp'>--</div><div class='muted'>ambiente</div></div>
  <div class='card'><h3>Umidade</h3><div class='big' id='mtHum'>--</div><div class='muted'>relativa</div></div>
</div></section>
<section class='section' id='section-lora'><div class='section-grid'>
  <div class='card'><h3>Recepcao UART LoRa</h3><div class='big' id='lrState'>--</div><div class='kv'><b>UART</b><span id='lrUart'>--</span><b>AUX</b><span id='lrAux'>--</span><b>RX agora</b><span id='lrRxState'>--</span><b>Ultimo RX</b><span class='mono' id='lrLastRx'>--</span></div><div class='toolbar'><button class='btn warn' type='button' onclick="act('lora=toggle')">Ligar/Desligar LoRa</button><button class='btn small' type='button' onclick="act('lora_clear=1')">Limpar console</button></div></div>
  <div class='card'><h3>Transmissao manual</h3><label for='lrPayload'>Payload manual</label><input id='lrPayload' name='lrPayload' type='text' placeholder='Ex: STATUS? ou ALVO=ALPHA,T=33'><div class='toolbar'><button class='btn small' type='button' onclick="sendLoraPayload()">Enviar texto</button><button class='btn small' type='button' onclick="act('lora_tx=status')">Enviar STATUS?</button><button class='btn small' type='button' onclick="act('lora_tx=summary')">Enviar resumo</button><button class='btn small' type='button' onclick="act('lora_tx=meteo')">Enviar METEO</button><button class='btn small' type='button' onclick="act('lora_tx=gps')">Enviar GPS</button><a class='btn small' href='/download/lora' target='_blank'>Exportar console</a></div></div>
  <div class='card'><h3>Auto TX 30s</h3><div class='big' id='lrAutoState'>--</div><div class='kv'><b>Modo</b><span id='lrAutoMode'>--</span><b>Intervalo</b><span id='lrAutoInterval'>--</span><b>Ultimo envio</b><span id='lrAutoLast'>--</span></div><div class='toolbar'><button class='btn alt' type='button' onclick="act('lora_auto=toggle')">Ligar/Desligar Auto TX</button><button class='btn small' type='button' onclick="act('lora_auto_mode=summary')">Modo resumo</button><button class='btn small' type='button' onclick="act('lora_auto_mode=meteo')">Modo METEO</button><button class='btn small' type='button' onclick="act('lora_auto_mode=gps')">Modo GPS</button></div></div>
  <div class='card'><h3>Resumo RX/TX</h3><div class='kv'><b>RX bytes</b><span id='lrRxBytes'>--</span><b>TX bytes</b><span id='lrTxBytes'>--</span><b>Ultimo TX</b><span class='mono' id='lrLastTx'>--</span><b>Nota</b><span id='lrNote'>--</span></div></div>
  <div class='card full'><h3>Console RX/TX UART</h3><div class='notice'>Este painel mostra payloads recebidos e enviados pela UART do modulo LoRa. Nao e SDR nem analise bruta de RF.</div><pre id='lrConsole'>--</pre></div>
</div></section>
<section class='section' id='section-comms'><div class='section-grid'>
  <div class='card'><h3>Modo de rede</h3><div class='big' id='cmMode'>--</div><div class='kv'><b>AP</b><span id='cmApState'>--</span><b>LAN</b><span id='cmLanState'>--</span><b>IP LAN</b><span class='mono' id='cmStaIp'>--</span><b>Clientes AP</b><span id='cmClients'>--</span><b>Hits web</b><span id='cmHits'>--</span></div><div class='toolbar'><button class='btn alt' type='button' onclick="act('wifi=toggle')">Ligar/Desligar AP</button><button class='btn small' id='btnScanWifi' type='button' onclick='scanWifi()'>Varrer redes</button><button class='btn small warn' type='button' onclick="act('sta=off')">Desconectar LAN</button></div></div>
  <div class='card'><h3>Redes detectadas</h3><div class='muted'>Toque em uma rede para preencher o SSID automaticamente.</div><div id='wifiList' class='listbox'><div class='muted'>Clique em Varrer redes para listar SSIDs visiveis.</div></div><div class='notice' id='cmScanNote'>--</div></div>
  <div class='card'><h3>Conectar LAN</h3><label for='cmStaSsid'>SSID</label><input id='cmStaSsid' name='ssid' type='text' autocomplete='username' placeholder='Selecione uma rede ou digite o SSID'><label for='cmStaPass'>Senha</label><input id='cmStaPass' name='password' type='password' autocomplete='current-password' placeholder='Senha da rede Wi-Fi'><div class='toolbar'><button class='btn alt' type='button' onclick='connectSta()'>Conectar via DHCP</button><button class='btn small' type='button' onclick="act('sta=forget')">Limpar credenciais</button></div><div class='notice warn'>Quando a LAN conectar com sucesso, o AP pode ser desligado e o acesso passa para o novo IP da rede local.</div></div>
  <div class='card'><h3>Diagnostico Wi-Fi</h3><div class='kv'><b>Nota AP</b><span id='cmWifiNote'>--</span><b>Nota LAN</b><span id='cmLanNote'>--</span><b>Ultimo scan</b><span id='cmScanSummary'>--</span></div></div>
</div></section>
<section class='section' id='section-config'><div class='section-grid'>
  <div class='card'><h3>Estado local</h3><div class='kv'><b>Tela ativa</b><span id='cfScreen'>--</span><b>Runtime</b><span id='cfMode'>--</span><b>Orientacao</b><span id='cfOrientation'>--</span><b>Rotacao</b><span id='cfRotation'>--</span><b>Wi-Fi</b><span id='cfWifi'>--</span><b>LoRa</b><span id='cfLora'>--</span><b>Logger</b><span id='cfLogger'>--</span></div></div>
  <div class='card'><h3>SD e sensores</h3><div class='kv'><b>Arquivo</b><span class='mono' id='cfFile'>--</span><b>Registros</b><span id='cfRecords'>--</span><b>Nota SD</b><span id='cfNote'>--</span><b>Sensores</b><span id='cfSensors'>--</span><b>Scan I2C</b><span class='mono' id='cfScan'>--</span></div></div>
  <div class='card'><h3>Acoes rapidas</h3><div class='toolbar'><button class='btn alt' type='button' onclick="act('wifi=toggle')">Wi-Fi AP</button><button class='btn small' type='button' onclick="act('logger=toggle')">Logger SD</button><button class='btn small' type='button' onclick="act('sd=remount')">Remount SD</button><button class='btn small' type='button' onclick="act('rescan=1')">Rescan sensores</button><button class='btn warn' type='button' onclick="act('lora=toggle')">LoRa UART</button></div></div>
  <div class='card'><h3>Abrir no dispositivo</h3><div class='toolbar'><button class='btn small' type='button' onclick="act('screen=home')">HOME local</button><button class='btn small' type='button' onclick="act('screen=plane')">PLANE local</button><button class='btn small' type='button' onclick="act('screen=meteo')">METEO local</button><button class='btn small' type='button' onclick="act('screen=lora')">LoRa local</button><button class='btn small' type='button' onclick="act('screen=config')">CONFIG local</button></div></div>
</div></section>
<section class='section' id='section-overview'><div class='section-grid'>
  <div class='card'><h3>Sistema</h3><div class='big' id='ovUptime'>--</div><div class='kv'><b>Tela local</b><span id='ovScreen'>--</span><b>Runtime</b><span id='ovRuntime'>--</span><b>URL ativa</b><span class='mono' id='ovIp'>--</span></div></div>
  <div class='card'><h3>Rede</h3><div class='kv'><b>Modo</b><span id='ovNetMode'>--</span><b>AP</b><span id='ovAp'>--</span><b>LAN</b><span id='ovLan'>--</span><b>Clientes</b><span id='ovClients'>--</span><b>Hits</b><span id='ovHits'>--</span></div></div>
  <div class='card'><h3>GPS e meteo</h3><div class='kv'><b>GPS</b><span id='ovGps'>--</span><b>Satelites</b><span id='ovSats'>--</span><b>Temperatura</b><span id='ovTemp'>--</span><b>Umidade</b><span id='ovHum'>--</span><b>Pressao</b><span id='ovPress'>--</span></div></div>
  <div class='card'><h3>LoRa e SD</h3><div class='kv'><b>LoRa</b><span id='ovLora'>--</span><b>RX/TX</b><span id='ovLoraFlow'>--</span><b>SD</b><span id='ovSd'>--</span><b>Logger</b><span id='ovLogger'>--</span><b>Arquivo</b><span class='mono' id='ovFile'>--</span><b>Registros</b><span id='ovRecords'>--</span></div></div>
  <div class='card full'><h3>Atividade recente</h3><pre id='ovActivity'>--</pre></div>
</div></section>)STRATOSWEB"));
  client.print(F(R"WEBJS(<script>
const $=i=>document.getElementById(i), txt=(v,d='--')=>(v===undefined||v===null||v==='')?d:v, toText=v=>String(txt(v));
const put=(i,v)=>{const e=$(i); if(!e)return; const next=toText(v); if(e.textContent!==next)e.textContent=next;};
const val=(i,v)=>{const e=$(i); const next=toText(v); if(e&&document.activeElement!==e&&(!e.value||e.dataset.user!=='1')&&e.value!==next)e.value=next;};
const num=(v,s='')=>(v===undefined||v===null||Number.isNaN(v))?'--':String(v)+s, yes=(v,a='ON',b='OFF')=>v?a:b;
function linkify(id,href,label){const e=$(id); if(!e)return; const ok=!!(href&&href!==''&&href!=='--'); e.href=ok?href:'#'; e.style.pointerEvents=ok?'auto':'none'; e.style.opacity=ok?'1':'.5'; if(label&&e.textContent!==label)e.textContent=label;}
function setTab(name,writeHash=true){document.querySelectorAll('.section').forEach(s=>s.classList.toggle('active',s.id==='section-'+name));document.querySelectorAll('.tab').forEach(b=>{const active=b.dataset.tab===name; b.classList.toggle('active',active); b.setAttribute('aria-selected',active?'true':'false');}); if(writeHash) location.hash='tab='+name;}
function activeTabFromHash(){const raw=(location.hash||'').replace(/^#/,''); return raw.startsWith('tab=')?raw.slice(4):'lab';}
function act(q){return fetch('/api/action?'+q).then(r=>r.json()).then(()=>fetchData()).catch(err=>console.log('Action error',err));}
function connectSta(){const ssid=$('cmStaSsid').value.trim(), pass=$('cmStaPass').value; if(!ssid){put('cmScanNote','Informe um SSID para conectar.'); return;} put('cmLanNote','Pedido de conexao enviado. Aguarde DHCP e veja o novo IP.'); act('sta_ssid='+encodeURIComponent(ssid)+'&sta_pass='+encodeURIComponent(pass)+'&sta=on');}
function sendLoraPayload(){const p=$('lrPayload').value.trim(); if(!p){put('lrNote','Informe um payload manual antes de enviar.'); return;} act('lora_payload='+encodeURIComponent(p));}
['cmStaSsid','cmStaPass'].forEach(id=>{const e=$(id); if(e)e.addEventListener('input',()=>e.dataset.user='1');});
function renderWifiNetworks(networks){const box=$('wifiList'); if(!box)return; box.innerHTML=''; if(!networks||!networks.length){const empty=document.createElement('div'); empty.className='muted'; empty.textContent='Nenhuma rede encontrada neste scan.'; box.appendChild(empty); return;} networks.forEach(n=>{const btn=document.createElement('button'); btn.type='button'; btn.className='network-item'; btn.addEventListener('click',()=>selectNetwork(n.ssid||'')); const left=document.createElement('span'); left.textContent=(n.ssid&&n.ssid.length)?n.ssid:'(oculta)'; const right=document.createElement('span'); right.className='network-meta'; right.textContent=(n.enc?'Senha':'Aberta')+' | '+n.rssi+' dBm'; btn.appendChild(left); btn.appendChild(right); box.appendChild(btn);});}
function scanWifi(){const btn=$('btnScanWifi'); if(btn) btn.disabled=true; renderWifiNetworks([]); put('cmScanNote','Varrendo redes Wi-Fi...'); fetch('/api/scan').then(r=>r.json()).then(d=>{renderWifiNetworks(d.networks||[]); put('cmScanNote',txt(d.note,d.ok?'Scan concluido':'Falha no scan')); put('cmScanSummary',txt(d.note,d.ok?'Scan concluido':'Falha no scan'));}).catch(()=>{put('cmScanNote','Erro ao executar o scan.'); put('cmScanSummary','Erro ao executar o scan.');}).finally(()=>{if(btn) btn.disabled=false;});}
function selectNetwork(ssid){const e=$('cmStaSsid'); if(e){e.value=ssid; e.dataset.user='1';} const p=$('cmStaPass'); if(p) p.focus();}
function plotSky(sats){const cvs=$('skyplot'); if(!cvs)return; const r=cvs.width=cvs.offsetWidth, h=cvs.height=cvs.offsetHeight, ctx=cvs.getContext('2d'), cx=r/2, cy=h/2, maxR=cx*0.9; ctx.clearRect(0,0,r,h); ctx.strokeStyle='#21445f'; ctx.lineWidth=1; [1,0.66,0.33].forEach(f=>{ctx.beginPath(); ctx.arc(cx,cy,maxR*f,0,2*Math.PI); ctx.stroke();}); for(let i=0;i<12;i++){ctx.beginPath();const a=i*30*Math.PI/180; ctx.moveTo(cx,cy); ctx.lineTo(cx+Math.cos(a)*maxR, cy+Math.sin(a)*maxR); ctx.stroke();} if(!sats||!sats.length)return; sats.forEach(s=>{const el=Math.max(0,Math.min(90,s.e)), az=s.a||0, dist=maxR*(90-el)/90, ang=(az-90)*Math.PI/180, px=cx+Math.cos(ang)*dist, py=cy+Math.sin(ang)*dist; let c='#41d39b'; if(s.t==='R')c='#c872ff'; else if(s.t==='B')c='#ff7a59'; else if(s.t==='E')c='#6fd6ff'; ctx.beginPath(); ctx.arc(px,py,s.s>0?6:4,0,2*Math.PI); ctx.fillStyle=s.s>0?c:'rgba(150,150,150,0.5)'; ctx.fill(); ctx.strokeStyle='#000'; ctx.stroke(); ctx.fillStyle='#fff'; ctx.font='10px Arial'; ctx.textAlign='center'; ctx.textBaseline='middle'; ctx.fillText(s.p, px, py-10);});}
function drawAH(pitch,roll){const cvs=$('ahCanvas'); if(!cvs)return; const w=cvs.width,h=cvs.height,ctx=cvs.getContext('2d'),cx=w/2,cy=h/2; ctx.save(); ctx.clearRect(0,0,w,h); ctx.translate(cx,cy); ctx.rotate(-roll*Math.PI/180); const pitchPx=pitch*2; ctx.fillStyle='#1a6fb5'; ctx.fillRect(-w,-h/2+pitchPx,w*2,h); ctx.fillStyle='#7a4a1e'; ctx.fillRect(-w,pitchPx,w*2,h); ctx.strokeStyle='#fff'; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(-w,pitchPx); ctx.lineTo(w,pitchPx); ctx.stroke(); [30,60].forEach(a=>{ctx.save(); ctx.rotate(a*Math.PI/180); ctx.strokeStyle='rgba(255,255,255,0.5)'; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(0,-cy+10); ctx.lineTo(0,-cy+20); ctx.stroke(); ctx.restore(); ctx.save(); ctx.rotate(-a*Math.PI/180); ctx.beginPath(); ctx.moveTo(0,-cy+10); ctx.lineTo(0,-cy+20); ctx.stroke(); ctx.restore();}); ctx.restore(); ctx.strokeStyle='#FFD34D'; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(cx-30,cy); ctx.lineTo(cx-10,cy); ctx.lineTo(cx-10,cy+8); ctx.stroke(); ctx.beginPath(); ctx.moveTo(cx+30,cy); ctx.lineTo(cx+10,cy); ctx.lineTo(cx+10,cy+8); ctx.stroke(); ctx.beginPath(); ctx.arc(cx,cy,4,0,2*Math.PI); ctx.fillStyle='#FFD34D'; ctx.fill();}
function fetchData(){fetch('/api/status').then(r=>r.json()).then(d=>{put('heroAp',yes(d.wifi.ap,'AP ligado','AP desligado')); put('heroLan',d.wifi.sta_connected?('LAN OK @ '+txt(d.wifi.sta_ip)):(d.wifi.sta_requested?'Tentando LAN':'Sem LAN')); put('heroIp','http://'+txt(d.wifi.ip)); put('heroMode',d.runtime_light?'Modo leve':'Modo normal'); put('labBmeState',d.meteo.connected?'Ligado':'Off'); put('labImuPitch',num(d.imu.pitch_deg,' deg')); put('labImuRoll',num(d.imu.roll_deg,' deg')); put('labSdLogger',yes(d.blackbox.logging_enabled,'Gravando','Pausado')); put('labGpsState',d.gps.fix?'Fix OK':(d.gps.receiving?'NMEA RX':'Aguardando')); put('labGpsFix',d.gps.fix?'Valido':'Sem fix'); put('labGpsSats',txt(d.gps.sats)); put('labGpsPos',d.gps.has_location?(num(d.gps.lat,'')+', '+num(d.gps.lon,'')):'--'); put('labGpsSentence',txt(d.gps.last_sentence)); put('labLoraState',yes(d.lora.enabled,'UART ativa','UART desligada')); put('lrRx2',txt(d.lora.rx_bytes)); put('lrTx2',txt(d.lora.tx_bytes)); put('lrAux2',d.lora.aux_high?'HIGH':'LOW'); put('lrMsg2',txt(d.lora.last_message)); put('gpsSpeed',num(d.gps.speed_kmh,' km/h')); put('gpsLatLon',d.gps.has_location?(num(d.gps.lat,'')+', '+num(d.gps.lon,'')):'--'); put('gpsAlt',num(d.gps.alt_m,' m')); put('gpsRate',txt(d.gps.update_hz)+' Hz'); put('gpsLastRx',txt(d.gps.last_rx_ms)+' ms'); put('gpsSentence',txt(d.gps.last_sentence)); put('gpsHistory',txt(d.gps.history).replaceAll('\\n','\n')); linkify('gpsOsmLink',txt(d.gps.map_osm,''),'OpenStreetMap'); linkify('gpsGoogleLink',txt(d.gps.map_google,''),'Google Maps'); if(d.sats_arr){plotSky(d.sats_arr); const total=d.sats_arr.length, gpsCount=d.sats_arr.filter(s=>s.t==='G').length, glo=d.sats_arr.filter(s=>s.t==='R').length, bds=d.sats_arr.filter(s=>s.t==='B').length, gal=d.sats_arr.filter(s=>s.t==='E').length; put('skpTotal',total+' sats visiveis'); put('skpGps','GPS '+gpsCount); put('skpGlo','GLO '+glo); put('skpBds','BDS '+bds); put('skpGal','GAL '+gal);} put('sdMounted',yes(d.blackbox.mounted,'montado','sem cartao')); put('sdLogger',yes(d.blackbox.logging_enabled,'ligado','pausado')); put('sdFile',txt(d.blackbox.file_name)); put('sdRecords',txt(d.blackbox.records)); put('sdSize',txt(d.blackbox.card_size_bytes)); put('sdTail',txt(d.blackbox.tail).replaceAll('\\n','\n')); put('sdFiles',txt(d.blackbox.files).replaceAll('\\n','\n')); put('mtTheme',txt(d.meteo.theme)); put('mtSummary',txt(d.meteo.summary)); put('mtTemp',num(d.meteo.temperature_c,' C')); put('mtHum',num(d.meteo.humidity_pct,' %')); put('mtPress',num(d.meteo.pressure_hpa,' hPa')); put('mtAlt',num(d.meteo.altitude_m,' m')); put('mtGas',num(d.meteo.gas_ohms,' ohms')); put('lrState',yes(d.lora.enabled,'ATIVO','DESLIGADO')); put('lrUart',yes(d.lora.uart_ready,'OK','OFF')); put('lrAux',d.lora.aux_high?'HIGH':'LOW'); put('lrRxState',d.lora.receiving?'recebendo':'idle'); put('lrLastRx',txt(d.lora.last_rx_message)); put('lrLastTx',txt(d.lora.last_tx_payload)); put('lrRxBytes',txt(d.lora.rx_bytes)); put('lrTxBytes',txt(d.lora.tx_bytes)); put('lrNote',txt(d.lora.note)); put('lrAutoState',d.lora.auto_tx_enabled?'ATIVO':'DESLIGADO'); put('lrAutoMode',txt(d.lora.auto_tx_mode,'summary')); put('lrAutoInterval',num((d.lora.auto_tx_interval_ms||0)/1000,' s')); put('lrAutoLast',num(d.lora.last_auto_tx_ms,' ms')); put('lrConsole',txt(d.lora.history).replaceAll('\\n','\n')); put('cmMode',txt(d.wifi.mode)); put('cmApState',yes(d.wifi.ap,'Ligado','Desligado')); put('cmLanState',d.wifi.sta_connected?'Conectado':(d.wifi.sta_requested?'Tentando':'Desligado')); put('cmStaIp',txt(d.wifi.sta_ip)); put('cmClients',txt(d.wifi.clients)); put('cmHits',txt(d.wifi.web_hits)); put('cmWifiNote',txt(d.wifi.note)); put('cmLanNote',txt(d.wifi.sta_note)); put('cmScanNote',txt(d.wifi.scan_summary)); put('cmScanSummary',txt(d.wifi.scan_summary)); val('cmStaSsid',txt(d.wifi.sta_ssid,'')); put('cfScreen',txt(d.screen_label)); put('cfMode',d.runtime_light?'leve':'normal'); put('cfOrientation',txt(d.ui.orientation)); put('cfRotation',txt(d.ui.rotation)); put('cfWifi',d.wifi.ap?('AP '+txt(d.wifi.mode)):txt(d.wifi.mode)); put('cfLora',yes(d.lora.enabled,'Ligado','Desligado')); put('cfLogger',yes(d.blackbox.logging_enabled,'Ligado','Pausado')); put('cfFile',txt(d.blackbox.file)); put('cfRecords',txt(d.blackbox.records)); put('cfNote',txt(d.blackbox.note)); put('cfSensors',txt(d.sensors.known)); put('cfScan',txt(d.sensors.scan)); put('ovUptime',num(d.uptime_s,' s')); put('ovScreen',txt(d.screen_label)); put('ovRuntime',d.runtime_light?'Modo leve':'Modo normal'); put('ovIp','http://'+txt(d.wifi.ip)); put('ovNetMode',txt(d.wifi.mode)); put('ovAp',yes(d.wifi.ap,'Ligado','Desligado')); put('ovLan',d.wifi.sta_connected?txt(d.wifi.sta_ip):txt(d.wifi.sta_note)); put('ovClients',txt(d.wifi.clients)); put('ovHits',txt(d.wifi.web_hits)); put('ovGps',d.gps.fix?'Fix OK':(d.gps.receiving?'NMEA RX':'Sem GPS')); put('ovSats',txt(d.gps.sats)); put('ovTemp',num(d.meteo.temperature_c,' C')); put('ovHum',num(d.meteo.humidity_pct,' %')); put('ovPress',num(d.meteo.pressure_hpa,' hPa')); put('ovLora',yes(d.lora.enabled,'Ligado','Desligado')); put('ovLoraFlow',txt(d.lora.rx_bytes)+' / '+txt(d.lora.tx_bytes)+' bytes'); put('ovSd',yes(d.blackbox.mounted,'montado','sem cartao')); put('ovLogger',yes(d.blackbox.logging_enabled,'ligado','pausado')); put('ovFile',txt(d.blackbox.file_name)); put('ovRecords',txt(d.blackbox.records)); put('ovActivity',txt(d.activity.recent).replaceAll('\\n','\n')); if(d.plane){put('pltAlt',num(d.plane.avg_altitude_m,' m')); put('pltVs',num(d.plane.avg_vs_ms,' m/s')); put('pltSpd',num(d.plane.avg_speed_kmh,' km/h'));} if(d.imu){const p=d.imu.pitch_deg||0, r=d.imu.roll_deg||0; put('ahPitch',num(p,' deg')); put('ahRoll',num(r,' deg')); drawAH(p,r);}}).catch(()=>{put('heroLan','Sem resposta do servidor'); put('cmScanNote','Falha ao atualizar o painel web.'); put('ovActivity','Sem resposta do servidor web neste momento.');});}
window.addEventListener('hashchange',()=>setTab(activeTabFromHash(),false));
setTab(activeTabFromHash(),false); fetchData(); setInterval(fetchData,1800);
</script></div></body></html>)WEBJS"));
}

static void sendHttpTextDownload(NetworkClient &client, const char *filename, const char *content)
{
  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: text/plain; charset=utf-8\r\n"));
  client.print(F("Cache-Control: no-store\r\n"));
  client.print(F("Connection: close\r\n"));
  client.print(F("Content-Disposition: attachment; filename=\""));
  client.print(filename ? filename : "download.txt");
  client.print(F("\"\r\n\r\n"));
  client.print(content ? content : "");
}

static void sendHttpActiveLogDownload(NetworkClient &client)
{
  const BlackboxState blackbox = copyBlackboxState();
  if (!blackbox.mounted || !blackbox.file_path[0] || !SD.exists(blackbox.file_path)) {
    sendHttpTextDownload(client, "sd_unavailable.txt", "SD indisponivel ou sem arquivo ativo.\n");
    return;
  }

  File file = SD.open(blackbox.file_path, FILE_READ);
  if (!file) {
    sendHttpTextDownload(client, "sd_open_error.txt", "Falha ao abrir o arquivo ativo do SD.\n");
    return;
  }

  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: text/csv; charset=utf-8\r\n"));
  client.print(F("Cache-Control: no-store\r\n"));
  client.print(F("Connection: close\r\n"));
  client.print(F("Content-Disposition: attachment; filename=\""));
  client.print(baseFileName(blackbox.file_path));
  client.print(F("\"\r\n\r\n"));

  while (file.available()) {
    uint8_t chunk[256];
    const size_t read_n = file.read(chunk, sizeof(chunk));
    if (read_n == 0) {
      break;
    }
    client.write(chunk, read_n);
  }
  file.close();
}

static void handleWifiPortal()
{
  const uint32_t now = millis();

  updateWifiRequestedFlag();
  if (g_wifi_ap_requested && !g_wifi_ap_started && !g_wifi_apply_pending && (now - g_wifi_recover_request_ms) > 1200U) {
    g_wifi_apply_pending = true;
    g_wifi_recover_request_ms = now;
    Serial.println("[WIFI] AP solicitado, mas inativo. Tentando subir novamente...");
  }
  if (g_wifi_apply_pending) {
    if (g_wifi_mode_requested) {
      startWifiPortal();
    } else {
      stopWifiPortal();
    }
  }

  if (g_wifi_sta_requested && !g_wifi_sta_connected && g_wifi_sta_ssid[0] && (now - g_wifi_last_status_ms) >= WIFI_STATUS_SERIAL_MS) {
    esp_wifi_connect();
  }

  if (!g_wifi_web_started) {
    return;
  }

  if ((now - g_wifi_last_status_ms) >= WIFI_STATUS_SERIAL_MS) {
    g_wifi_last_status_ms = now;
    printWifiStatus("Heartbeat");
  }

  NetworkClient client = g_web_server.accept();
  if (!client) {
    return;
  }

  char request_line[96];
  if (!readHttpRequestLine(client, request_line, sizeof(request_line))) {
    client.stop();
    return;
  }

  discardHttpHeaders(client);
  g_wifi_web_hits += 1;
  g_wifi_last_client_ms = millis();

  char request_path[160];
  if (!parseHttpPath(request_line, request_path, sizeof(request_path))) {
    client.stop();
    return;
  }

  Serial.printf(
      "[WEB] Cliente %u | path=%s | heap=%lu\n",
      static_cast<unsigned>(wifiClientCount()),
      request_path,
      static_cast<unsigned long>(ESP.getFreeHeap()));

  if (strncmp(request_path, "/api/action", 11) == 0) {
    handleHttpActionPath(request_path);
    sendHttpJsonStatus(client);
  } else if (strncmp(request_path, "/api/status", 11) == 0) {
    sendHttpJsonStatus(client);
  } else if (strncmp(request_path, "/download/blackbox", 18) == 0) {
    sendHttpActiveLogDownload(client);
  } else if (strncmp(request_path, "/download/lora", 14) == 0) {
    char export_text[2000];
    formatLoraExportText(export_text, sizeof(export_text));
    sendHttpTextDownload(client, "lora_console.txt", export_text);
  } else if (strncmp(request_path, "/api/scan", 9) == 0) {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json; charset=utf-8\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n"));
    wifi_mode_t original_mode = WIFI_MODE_NULL;
    wifi_mode_t scan_mode = WIFI_MODE_NULL;
    bool restore_mode = false;
    esp_err_t mode_err = ensureWifiStack() ? prepareWifiModeForScan(&original_mode, &scan_mode, &restore_mode) : ESP_FAIL;
    char scan_summary[720] = {};
    int n = -1;
    uint16_t fetch_count = 0;
    wifi_ap_record_t records[12] = {};
    if (mode_err == ESP_OK) {
      wifi_scan_config_t scan_cfg = {};
      scan_cfg.show_hidden = true;
      scan_cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
      esp_err_t scan_err = esp_wifi_scan_start(&scan_cfg, true);
      if (scan_err == ESP_OK) {
        uint16_t ap_count = 0;
        scan_err = esp_wifi_scan_get_ap_num(&ap_count);
        if (scan_err == ESP_OK) {
          n = static_cast<int>(ap_count);
          fetch_count = static_cast<uint16_t>(ap_count > 12 ? 12 : ap_count);
          if (fetch_count > 0) {
            scan_err = esp_wifi_scan_get_ap_records(&fetch_count, records);
          }
        }
      }
      restoreWifiModeAfterScan(original_mode, restore_mode);
      if (scan_err != ESP_OK) {
        snprintf(scan_summary, sizeof(scan_summary), "Falha no scan: %s", esp_err_to_name(scan_err));
        printWifiScanDiag("api_scan_fail", scan_err, scan_mode);
        n = -1;
      } else if (n == 0) {
        snprintf(scan_summary, sizeof(scan_summary), "Nenhuma rede encontrada agora.");
        printWifiScanDiag("api_scan_empty", ESP_OK, scan_mode);
      } else {
        snprintf(scan_summary, sizeof(scan_summary), "Scan concluido com %d rede(s)", n);
        printWifiScanDiag("api_scan_ok", ESP_OK, scan_mode);
      }
    }
    g_wifi_scan_summary = scan_summary[0] ? scan_summary : ((mode_err == ESP_OK) ? "Scan concluido" : "Falha ao preparar scan");
    g_wifi_scan_count = (n > 0) ? static_cast<uint8_t>((n > 255) ? 255 : n) : 0;
    g_wifi_scan_ms = millis();
    client.print(F("{\"ok\":"));
    client.print((mode_err == ESP_OK && n >= 0) ? F("true") : F("false"));
    client.print(F(",\"count\":"));
    client.print((n > 0) ? n : 0);
    client.print(F(",\"networks\":["));
    if (n > 0 && fetch_count > 0) {
      for (uint16_t i = 0; i < fetch_count; i++) {
        char ssid_esc[66] = {};
        const char *raw = reinterpret_cast<const char *>(records[i].ssid);
        size_t eo = 0;
        for (size_t si = 0; raw && raw[si] != '\0' && eo < 63; si++) {
          char c = raw[si];
          if (c == '"' || c == '\\') { ssid_esc[eo++] = '\\'; }
          if (eo < 63) ssid_esc[eo++] = c;
        }
        client.printf(
            "{\"ssid\":\"%s\",\"rssi\":%d,\"enc\":%s}%s",
            ssid_esc[0] ? ssid_esc : "(oculta)",
            static_cast<int>(records[i].rssi),
            records[i].authmode == WIFI_AUTH_OPEN ? "false" : "true",
            (i + 1U < fetch_count) ? "," : "");
      }
    }
    client.print(F("],\"mode\":\""));
    switch (scan_mode) {
      case WIFI_MODE_AP:
        client.print(F("AP"));
        break;
      case WIFI_MODE_STA:
        client.print(F("STA"));
        break;
      case WIFI_MODE_APSTA:
        client.print(F("APSTA"));
        break;
      default:
        client.print(F("NULL"));
        break;
    }
    client.print(F("\",\"note\":\""));
    if (mode_err != ESP_OK) {
      client.print(F("Falha ao preparar scan Wi-Fi"));
      Serial.printf("[WIFI] Scan falhou | get_mode=%s\n", esp_err_to_name(mode_err));
    } else if (n < 0) {
      client.print(scan_summary);
      Serial.printf("[WIFI] Scan retornou erro | %s\n", scan_summary);
    } else if (n == 0) {
      client.print(scan_summary);
      Serial.printf("[WIFI] Scan sem redes | %s\n", scan_summary);
    } else {
      client.print(F("Scan concluido"));
      Serial.printf("[WIFI] Scan concluiu | redes=%d | mode=%s\n", n, wifiModeLabel(scan_mode));
    }
    client.print(F("\"}"));
  } else {
    sendHttpDashboardV2(client);
  }

  delay(2);
  client.stop();
}


static lv_obj_t *screenFromId(AppScreenId screen_id)
{
  switch (screen_id) {
    case SCREEN_EFIS:
      return g_screen_efis;
    case SCREEN_METEO:
      return g_screen_meteo;
    case SCREEN_GPS:
      return g_screen_gps;
    case SCREEN_LORA:
      return g_screen_lora;
    case SCREEN_CONFIG:
      return g_screen_config;
    default:
      return g_screen_main;
  }
}

static bool i2cAddressPresent(uint8_t addr)
{
  if (!lockI2C(pdMS_TO_TICKS(30))) {
    return false;
  }

  Wire.beginTransmission(addr);
  const bool ok = (Wire.endTransmission() == 0);
  unlockI2C();
  return ok;
}

static bool initBme688Sensor()
{
  Bme688State state = {};
  state.i2c_addr = 0;

  if (i2cAddressPresent(BME688_ADDR_PRIMARY)) {
    state.i2c_addr = BME688_ADDR_PRIMARY;
  } else if (i2cAddressPresent(BME688_ADDR_SECONDARY)) {
    state.i2c_addr = BME688_ADDR_SECONDARY;
  }

  state.connected = state.i2c_addr != 0;
  state.initialized = false;
  state.reading_ok = false;
  state.has_data = false;
  state.last_update_ms = millis();

  if (!state.connected) {
    snprintf(state.note, sizeof(state.note), "BME688 nao detectado");
    g_bme688_addr = 0;
    g_bme688_initialized = false;
    storeBme688State(state);
    return false;
  }

  if (!lockI2C(pdMS_TO_TICKS(80))) {
    snprintf(state.note, sizeof(state.note), "BME688 aguardando mutex I2C");
    storeBme688State(state);
    return false;
  }

  g_bme688.begin(state.i2c_addr, Wire);
  const int8_t begin_status = g_bme688.checkStatus();
  if (begin_status == BME68X_ERROR) {
    unlockI2C();
    snprintf(state.note, sizeof(state.note), "Erro init: %s", g_bme688.statusString().c_str());
    g_bme688_addr = 0;
    g_bme688_initialized = false;
    storeBme688State(state);
    Serial.printf("[BME688] Falha ao inicializar em 0x%02X: %s\n", state.i2c_addr, state.note);
    return false;
  }

  g_bme688.setTPH();
  g_bme688.setHeaterProf(BME688_HEATER_TEMP_C, BME688_HEATER_TIME_MS);
  unlockI2C();

  state.initialized = true;
  g_bme688_addr = state.i2c_addr;
  g_bme688_initialized = true;

  if (begin_status == BME68X_WARNING) {
    snprintf(state.note, sizeof(state.note), "Init com aviso @0x%02X", state.i2c_addr);
  } else {
    snprintf(state.note, sizeof(state.note), "BME688 pronto @0x%02X", state.i2c_addr);
  }

  storeBme688State(state);
  Serial.printf("[BME688] Inicializado em 0x%02X\n", state.i2c_addr);
  return true;
}

static void pollBme688Sensor()
{
  static uint32_t last_poll_ms = 0;
  static uint8_t consecutive_failures = 0;
  const uint32_t now = millis();

  if ((now - last_poll_ms) < BME688_POLL_PERIOD_MS) {
    return;
  }
  last_poll_ms = now;

  if (!g_bme688_initialized) {
    initBme688Sensor();
    return;
  }

  uint32_t meas_dur_us = 0;
  if (!lockI2C(pdMS_TO_TICKS(80))) {
    return;
  }

  g_bme688.setOpMode(BME68X_FORCED_MODE);
  meas_dur_us = g_bme688.getMeasDur();
  unlockI2C();

  if (meas_dur_us > 20000U) {
    delay((meas_dur_us + 999U) / 1000U);
  } else {
    delayMicroseconds(meas_dur_us + 1000U);
  }

  bme68xData data = {};
  bool ok = false;
  uint8_t fetch_count = 0;
  int8_t lib_status = BME68X_OK;
  String status_text;
  if (lockI2C(pdMS_TO_TICKS(80))) {
    fetch_count = g_bme688.fetchData();
    lib_status = g_bme688.checkStatus();
    if (fetch_count > 0 && lib_status != BME68X_ERROR) {
      g_bme688.getData(data);
      ok = true;
    }
    status_text = g_bme688.statusString();
    unlockI2C();
  }

  Bme688State state = copyBme688State();
  state.connected = g_bme688_addr != 0;
  state.initialized = g_bme688_initialized;
  state.i2c_addr = g_bme688_addr;
  state.last_update_ms = now;
  state.reading_ok = ok;

  if (ok) {
    consecutive_failures = 0;
    state.has_data = true;
    state.temperature_c = data.temperature;
    state.humidity_pct = data.humidity;
    state.pressure_hpa = data.pressure / 100.0f;
    state.altitude_m = pressureToAltitudeMeters(state.pressure_hpa);
    state.gas_ohms = static_cast<float>(data.gas_resistance);
    snprintf(
        state.note,
        sizeof(state.note),
        "T %.1fC | H %.1f%% | P %.1fhPa",
        state.temperature_c,
        state.humidity_pct,
        state.pressure_hpa);
    Serial.printf(
        "[BME688] OK @0x%02X | T=%.1fC H=%.1f%% P=%.1fhPa Alt=%.1fm Gas=%.0fohm | fields=%u\n",
        state.i2c_addr,
        state.temperature_c,
        state.humidity_pct,
        state.pressure_hpa,
        state.altitude_m,
        state.gas_ohms,
        static_cast<unsigned>(fetch_count));
  } else {
    if (consecutive_failures < 255U) {
      consecutive_failures += 1U;
    }
    snprintf(
        state.note,
        sizeof(state.note),
        fetch_count == 0
            ? (status_text.length() ? "Sem dado novo: %s" : "Sem dado novo")
            : (status_text.length() ? "Leitura falhou: %s" : "Leitura falhou"),
        status_text.c_str());
    Serial.printf(
        "[BME688] FAIL @0x%02X | fetch=%u | status=%d | msg=%s\n",
        state.i2c_addr,
        static_cast<unsigned>(fetch_count),
        static_cast<int>(lib_status),
        status_text.length() ? status_text.c_str() : "sem mensagem");

    if (consecutive_failures >= 3U) {
      g_bme688_initialized = false;
      state.initialized = false;
      snprintf(state.note, sizeof(state.note), "BME688 falhou repetido. Tentando reinicializar...");
      Serial.println("[BME688] Muitas falhas seguidas. Reinicializacao agendada.");
      consecutive_failures = 0;
    }
  }

  storeBme688State(state);
}

static String formatKnownSensorSummary()
{
  struct KnownSensor
  {
    const char *name;
    uint8_t primary;
    uint8_t secondary;
  };

  static const KnownSensor known_sensors[] = {
      {"FT3168 Touch", 0x38, 0xFF},
      {"QMI8658 IMU", 0x6B, 0x6A},
      {"BME688", 0x77, 0x76},
      {"BMP581", 0x47, 0x46},
      {"BMM350", 0x14, 0xFF}};

  String result;
  result.reserve(320);

  for (const KnownSensor &sensor : known_sensors) {
    const bool primary_ok = i2cAddressPresent(sensor.primary);
    const bool secondary_ok = (sensor.secondary != 0xFF) ? i2cAddressPresent(sensor.secondary) : false;
    const bool found = primary_ok || secondary_ok;

    result += sensor.name;
    result += ": ";
    result += found ? "OK " : "-- ";

    if (primary_ok) {
      char tmp[8];
      snprintf(tmp, sizeof(tmp), "0x%02X", sensor.primary);
      result += tmp;
    } else if (secondary_ok) {
      char tmp[8];
      snprintf(tmp, sizeof(tmp), "0x%02X", sensor.secondary);
      result += tmp;
    } else {
      char tmp[16];
      if (sensor.secondary != 0xFF) {
        snprintf(tmp, sizeof(tmp), "0x%02X/0x%02X", sensor.primary, sensor.secondary);
      } else {
        snprintf(tmp, sizeof(tmp), "0x%02X", sensor.primary);
      }
      result += tmp;
    }

    result += '\n';
  }

  return result;
}

static bool scanHasAddress(const String &scan, uint8_t addr)
{
  char token[8];
  snprintf(token, sizeof(token), "0x%02X", addr);
  return scan.indexOf(token) >= 0;
}

static String compactScanLine(const String &scan)
{
  if (scan.length() <= 30) {
    return scan;
  }

  return scan.substring(0, 30) + "...";
}

static String formatCompactSensorSummary(const String &scan)
{
  const bool touch_ok = scanHasAddress(scan, 0x38);
  const bool imu_ok = scanHasAddress(scan, 0x6B) || scanHasAddress(scan, 0x6A);
  const bool bme_ok = scanHasAddress(scan, 0x77) || scanHasAddress(scan, 0x76);
  const bool bmp_ok = scanHasAddress(scan, 0x47) || scanHasAddress(scan, 0x46);
  const bool bmm_ok = scanHasAddress(scan, 0x14);

  char buffer[128];
  snprintf(
      buffer,
      sizeof(buffer),
      "PLANE IMU %s BMP %s MAG %s\nMETEO BME %s",
      imu_ok ? "OK" : "--",
      bmp_ok ? "OK" : "--",
      bmm_ok ? "OK" : "--",
      bme_ok ? "OK" : "--");
  return String(buffer);
}

static String formatBoschSensorMap(const String &scan)
{
  const bool bme_ok = scanHasAddress(scan, 0x77) || scanHasAddress(scan, 0x76);
  const bool bmp_ok = scanHasAddress(scan, 0x47) || scanHasAddress(scan, 0x46);
  const bool bmm_ok = scanHasAddress(scan, 0x14);

  char buffer[384];
  snprintf(
      buffer,
      sizeof(buffer),
      "BME688 %s -> METEO / ar\nBMP581 %s -> METEO + PLANE\nBMM350 %s -> PLANE / heading",
      bme_ok ? "OK" : "--",
      bmp_ok ? "OK" : "--",
      bmm_ok ? "OK" : "--");
  return String(buffer);
}

static String formatRouteIntegrationSummary(const String &scan)
{
  const bool touch_ok = scanHasAddress(scan, 0x38);
  const bool imu_ok = scanHasAddress(scan, 0x6B) || scanHasAddress(scan, 0x6A);

  char buffer[320];
  snprintf(
      buffer,
      sizeof(buffer),
      "Touch %s -> UI\nPLANE = GPS + sensores\nGPS UART1/2 -> COMMS + PLANE\nLoRa UART17/18 M0/1=7/8 AUX6 -> COMMS + LORA\nCDC USB/JTAG -> CONFIG",
      touch_ok ? "OK" : "--");
  return String(buffer);
}

static void refreshSensorCaches()
{
  g_sensor_summary_cache = formatKnownSensorSummary();
  g_sensor_raw_scan_cache = scanI2C();
  g_sensor_compact_cache = formatCompactSensorSummary(g_sensor_raw_scan_cache);
  g_sensor_scan_compact_cache = compactScanLine(g_sensor_raw_scan_cache);
  g_sensor_bosch_cache = formatBoschSensorMap(g_sensor_raw_scan_cache);
  g_sensor_route_cache = formatRouteIntegrationSummary(g_sensor_raw_scan_cache);
}

static String scanI2C()
{
  String result;
  uint8_t found = 0;

  if (!lockI2C(pdMS_TO_TICKS(50))) {
    return "Barramento I2C ocupado";
  }

  for (uint8_t addr = 1; addr < 127; ++addr) {
    // Skip reserved I2C ranges to avoid reporting bus-control addresses as sensors.
    if (addr <= 0x07 || addr >= 0x78) {
      continue;
    }

    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (found > 0) {
        result += ' ';
      }

      char tmp[8];
      snprintf(tmp, sizeof(tmp), "0x%02X", addr);
      result += tmp;
      ++found;
    }
  }

  if (found == 0) {
    result = "Nenhum dispositivo I2C encontrado";
  }

  unlockI2C();

  return result;
}

static void closeBlackboxFile()
{
  if (g_blackbox_file) {
    g_blackbox_file.flush();
    g_blackbox_file.close();
  }
}

static void unmountBlackboxStorage()
{
  closeBlackboxFile();
  SD.end();
  g_sd_spi.end();
  clearBlackboxTail();
}

static void chooseBlackboxPath(char *path, size_t path_len)
{
  if (!path || path_len == 0) {
    return;
  }

  for (uint16_t index = 1; index < 10000; ++index) {
    snprintf(path, path_len, "/logs/flight_%04u.csv", static_cast<unsigned>(index));
    if (!SD.exists(path)) {
      return;
    }
  }

  snprintf(path, path_len, "/logs/flight_overflow.csv");
}

static bool mountBlackboxStorage(BlackboxState *state)
{
  if (!state) {
    return false;
  }

  pinMode(SDCARD_CS_PIN, OUTPUT);
  digitalWrite(SDCARD_CS_PIN, HIGH);

  if (!g_sd_spi.begin(SDCARD_SCK_PIN, SDCARD_MISO_PIN, SDCARD_MOSI_PIN, SDCARD_CS_PIN)) {
    state->mounted = false;
    state->file_open = false;
    state->card_size_bytes = 0;
    snprintf(state->note, sizeof(state->note), "Falha ao iniciar SPI do SD");
    return false;
  }

  if (!SD.begin(SDCARD_CS_PIN, g_sd_spi, SD_SPI_FREQUENCY)) {
    state->mounted = false;
    state->file_open = false;
    state->card_size_bytes = 0;
    snprintf(state->note, sizeof(state->note), "Sem cartao SD ou falha de montagem");
    SD.end();
    g_sd_spi.end();
    return false;
  }

  const sdcard_type_t card_type = SD.cardType();
  if (card_type == CARD_NONE) {
    state->mounted = false;
    state->file_open = false;
    state->card_size_bytes = 0;
    snprintf(state->note, sizeof(state->note), "Slot SD sem cartao");
    SD.end();
    g_sd_spi.end();
    return false;
  }

  state->mounted = true;
  state->file_open = false;
  state->card_size_bytes = SD.cardSize();
  snprintf(
      state->note,
      sizeof(state->note),
      "Cartao %s montado. Pronto para gravar",
      cardTypeLabel(card_type));
  appendActivityLine("SD", "cartao montado");
  return true;
}

static bool openBlackboxLog(BlackboxState *state)
{
  if (!state || !state->mounted) {
    return false;
  }

  if (!SD.exists("/logs")) {
    SD.mkdir("/logs");
  }

  char path[BLACKBOX_PATH_LEN] = {0};
  chooseBlackboxPath(path, sizeof(path));
  const bool new_file = !SD.exists(path);

  closeBlackboxFile();
  g_blackbox_file = SD.open(path, FILE_WRITE);
  if (!g_blackbox_file) {
    state->file_open = false;
    snprintf(state->note, sizeof(state->note), "Falha ao abrir arquivo no SD");
    return false;
  }

  if (new_file || g_blackbox_file.size() == 0) {
    g_blackbox_file.println(
        "data_utc,hora_utc,uptime_s,temperatura_c,umidade_pct,pressao_hpa,altitude_bosch_m,altitude_gps_m,altitude_gps_km,latitude_gps_deg,longitude_gps_deg,localizacao_gps,gps_fix,gps_satelites");
    g_blackbox_file.flush();
  }

  snprintf(state->file_path, sizeof(state->file_path), "%s", path);
  state->file_open = true;
  state->records_written = 0;
  clearBlackboxTail();
  snprintf(state->note, sizeof(state->note), "Gravando BME688 + GPS a cada 30s.");
  appendActivityLine("SD", baseFileName(path));
  return true;
}

static bool appendBlackboxRecord(BlackboxState *state, uint32_t now_ms)
{
  if (!state || !state->mounted || !state->file_open || !g_blackbox_file) {
    return false;
  }

  const GpsState gps = copyGpsState();
  const Bme688State bme = copyBme688State();
  char line[BLACKBOX_TAIL_LINE_LEN];
  char location_text[40];
  if (gps.has_location) {
    snprintf(
        location_text,
        sizeof(location_text),
        "%.6f%s %.6f%s",
        fabsf(gps.latitude_deg),
        gps.latitude_deg < 0.0f ? "S" : "N",
        fabsf(gps.longitude_deg),
        gps.longitude_deg < 0.0f ? "W" : "E");
  } else {
    snprintf(location_text, sizeof(location_text), "--");
  }

  const int line_len = snprintf(
      line,
      sizeof(line),
      "%s,%s,%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f,%.6f,%.6f,%s,%u,%u\n",
      gps.utc_date[0] ? gps.utc_date : "--",
      gps.utc_time[0] ? gps.utc_time : "--:--:--",
      static_cast<unsigned long>(now_ms / 1000U),
      bme.has_data ? bme.temperature_c : 0.0f,
      bme.has_data ? bme.humidity_pct : 0.0f,
      bme.has_data ? bme.pressure_hpa : 0.0f,
      bme.has_data ? bme.altitude_m : 0.0f,
      gps.has_fix ? gps.altitude_m : 0.0f,
      gps.has_fix ? (gps.altitude_m / 1000.0f) : 0.0f,
      gps.has_location ? gps.latitude_deg : 0.0f,
      gps.has_location ? gps.longitude_deg : 0.0f,
      location_text,
      gps.has_fix ? 1U : 0U,
      static_cast<unsigned>(gps.sats));

  if (line_len <= 0 || static_cast<size_t>(line_len) >= sizeof(line)) {
    snprintf(state->note, sizeof(state->note), "Falha ao montar linha CSV");
    return false;
  }

  const size_t written = g_blackbox_file.write(
      reinterpret_cast<const uint8_t *>(line),
      static_cast<size_t>(line_len));

  if (written != static_cast<size_t>(line_len)) {
    snprintf(state->note, sizeof(state->note), "Falha ao escrever no cartao SD");
    return false;
  }

  state->records_written += 1;
  state->last_log_ms = now_ms;
  state->last_write_ok = true;
  appendBlackboxTailLine(line);
  return true;
}

static uint16_t clearBlackboxLogs()
{
  if (!SD.exists("/logs")) {
    return 0;
  }

  File dir = SD.open("/logs");
  if (!dir || !dir.isDirectory()) {
    return 0;
  }

  uint16_t removed = 0;
  for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
    char entry_path[96];
    const char *entry_name = entry.name();

    if (entry_name && entry_name[0] == '/') {
      snprintf(entry_path, sizeof(entry_path), "%s", entry_name);
    } else {
      snprintf(entry_path, sizeof(entry_path), "/logs/%s", entry_name ? entry_name : "");
    }

    entry.close();

    if (entry_path[0] != '\0' && SD.exists(entry_path) && SD.remove(entry_path)) {
      ++removed;
    }
  }

  dir.close();
  return removed;
}


static void drawHardwareSmokeTest()
{
  g_panel->fillScreen(COLOR_BLACK);

  g_panel->drawRect(0, 0, LCD_W, LCD_H, COLOR_WHITE);
  g_panel->fillRoundRect(8, 8, LCD_W - 16, 40, 8, COLOR_CYAN);

  g_panel->setTextSize(2);
  g_panel->setTextColor(COLOR_BLACK);
  g_panel->setCursor(20, 22);
  g_panel->println("CO5300 DIRECT TEST");

  g_panel->fillRect(18, 72, LCD_W - 36, 40, COLOR_RED);
  g_panel->fillRect(18, 122, LCD_W - 36, 40, COLOR_GREEN);
  g_panel->fillRect(18, 172, LCD_W - 36, 40, COLOR_BLUE);

  g_panel->setTextColor(COLOR_WHITE);
  g_panel->setCursor(18, 240);
  g_panel->println("Display OK.");
  g_panel->setCursor(18, 264);
  g_panel->println("Agora vamos testar");
  g_panel->setCursor(18, 288);
  g_panel->println("touch e menus em LVGL.");

  g_panel->drawFastHLine(18, 328, LCD_W - 36, COLOR_YELLOW);
  g_panel->setCursor(18, 350);
  g_panel->println("Carregando interface...");

  delay(1600);
}

static void initDisplay()
{
  Serial.println("[DISPLAY] Iniciando QSPI...");

  g_bus = new Arduino_ESP32QSPI(
      LCD_CS,
      LCD_CLK,
      LCD_D0,
      LCD_D1,
      LCD_D2,
      LCD_D3);

  g_panel = new Arduino_CO5300(
      g_bus,
      LCD_RST,
      0,
      LCD_W,
      LCD_H,
      LCD_COL_OFFSET_1,
      LCD_ROW_OFFSET_1,
      LCD_COL_OFFSET_2,
      LCD_ROW_OFFSET_2);

  if (!g_panel->begin()) {
    fatalStop("[DISPLAY] Falha em g_panel->begin()");
  }

  applyPanelBrightness(255);
  g_panel->fillScreen(COLOR_BLACK);

  Serial.println("[DISPLAY] begin() OK");
  if (ENABLE_DISPLAY_SMOKE_TEST) {
    Serial.println("[DISPLAY] Teste direto em hardware...");
    drawHardwareSmokeTest();
    Serial.println("[DISPLAY] Teste direto concluido");
  } else {
    Serial.println("[DISPLAY] Smoke test visual desativado");
  }
}

static bool touchWriteReg(uint8_t reg, uint8_t value)
{
  if (!lockI2C()) {
    return false;
  }

  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(reg);
  Wire.write(value);
  const bool ok = (Wire.endTransmission() == 0);
  unlockI2C();
  return ok;
}

static bool touchReadRegs(uint8_t reg, uint8_t *buffer, size_t len)
{
  if (!lockI2C()) {
    return false;
  }

  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    unlockI2C();
    return false;
  }

  const size_t read = Wire.requestFrom(static_cast<int>(TOUCH_ADDR), static_cast<int>(len));
  if (read != len) {
    while (Wire.available()) {
      Wire.read();
    }
    unlockI2C();
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }

  unlockI2C();
  return true;
}

static void normalizeTouchPoint(uint16_t &x, uint16_t &y)
{
  if (TOUCH_SWAP_XY) {
    const uint16_t tmp = x;
    x = y;
    y = tmp;
  }

  if (TOUCH_INVERT_X) {
    x = (LCD_W - 1) - x;
  }

  if (TOUCH_INVERT_Y) {
    y = (LCD_H - 1) - y;
  }

  if (x >= LCD_W) {
    x = LCD_W - 1;
  }

  if (y >= LCD_H) {
    y = LCD_H - 1;
  }
}

static bool readTouchPoint(uint16_t *x, uint16_t *y)
{
  uint8_t points = 0;
  if (!touchReadRegs(TOUCH_REG_POINTS, &points, 1)) {
    return false;
  }

  points &= 0x0F;
  if (points == 0) {
    return false;
  }

  uint8_t buf[4] = {0};
  if (!touchReadRegs(TOUCH_REG_COORDS, buf, sizeof(buf))) {
    return false;
  }

  uint16_t raw_x = static_cast<uint16_t>(((buf[0] & 0x0F) << 8) | buf[1]);
  uint16_t raw_y = static_cast<uint16_t>(((buf[2] & 0x0F) << 8) | buf[3]);

  normalizeTouchPoint(raw_x, raw_y);

  *x = raw_x;
  *y = raw_y;
  return true;
}

static void initTouch()
{
  Serial.println("[TOUCH] Inicializando FT3168...");

  const bool locked = lockI2C();
  Wire.beginTransmission(TOUCH_ADDR);
  const bool ack = (Wire.endTransmission() == 0);
  if (locked) {
    unlockI2C();
  }
  Serial.printf("[TOUCH] Endereco 0x%02X: %s\n", TOUCH_ADDR, ack ? "OK" : "falhou");

  if (!touchWriteReg(TOUCH_REG_MODE, 0x00)) {
    Serial.println("[TOUCH] Aviso: nao consegui escrever modo normal");
  }

  uint8_t chip_id = 0;
  if (touchReadRegs(TOUCH_REG_CHIP_ID, &chip_id, 1)) {
    Serial.printf("[TOUCH] Registro 0xA3 = 0x%02X\n", chip_id);
  } else {
    Serial.println("[TOUCH] Aviso: nao consegui ler registro 0xA3");
  }

  Serial.println("[TOUCH] Driver pronto");
}

static void applyImuAxisTransform(float &x, float &y, float &z)
{
  if (IMU_SWAP_XY) {
    const float tmp = x;
    x = y;
    y = tmp;
  }

  if (IMU_INVERT_X) {
    x = -x;
  }

  if (IMU_INVERT_Y) {
    y = -y;
  }

  if (IMU_INVERT_Z) {
    z = -z;
  }
}

static bool qmiWriteReg(uint8_t reg, uint8_t value)
{
  if (!lockI2C()) {
    return false;
  }

  Wire.beginTransmission(g_qmi_addr);
  Wire.write(reg);
  Wire.write(value);
  const bool ok = (Wire.endTransmission() == 0);
  unlockI2C();
  return ok;
}

static bool qmiReadRegs(uint8_t reg, uint8_t *buffer, size_t len)
{
  if (!lockI2C()) {
    return false;
  }

  Wire.beginTransmission(g_qmi_addr);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    unlockI2C();
    return false;
  }

  const size_t read = Wire.requestFrom(static_cast<int>(g_qmi_addr), static_cast<int>(len));
  if (read != len) {
    while (Wire.available()) {
      Wire.read();
    }
    unlockI2C();
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }

  unlockI2C();
  return true;
}

static bool detectQmi8658(uint8_t *detected_addr, uint8_t *chip_id)
{
  const uint8_t addresses[] = {QMI8658_ADDR_PRIMARY, QMI8658_ADDR_SECONDARY};

  for (uint8_t addr : addresses) {
    g_qmi_addr = addr;

    uint8_t id = 0;
    if (!qmiReadRegs(QMI8658_REG_WHO_AM_I, &id, 1)) {
      continue;
    }

    if (id == QMI8658_CHIP_ID) {
      if (detected_addr) {
        *detected_addr = addr;
      }

      if (chip_id) {
        *chip_id = id;
      }

      return true;
    }
  }

  return false;
}

static bool initQmi8658()
{
  uint8_t detected_addr = 0;
  uint8_t chip_id = 0;

  if (!detectQmi8658(&detected_addr, &chip_id)) {
    Serial.println("[IMU] QMI8658 nao encontrado no barramento I2C");
    return false;
  }

  g_qmi_addr = detected_addr;
  Serial.printf("[IMU] QMI8658 detectado em 0x%02X (chip 0x%02X)\n", g_qmi_addr, chip_id);

  if (!qmiWriteReg(QMI8658_REG_CTRL7, 0x00)) {
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(10));

  if (!qmiWriteReg(QMI8658_REG_CTRL1, 0x78)) {
    return false;
  }

  if (!qmiWriteReg(QMI8658_REG_CTRL8, 0xC0)) {
    return false;
  }

  if (!qmiWriteReg(QMI8658_REG_CTRL2, 0x25)) {
    return false;
  }

  if (!qmiWriteReg(QMI8658_REG_CTRL3, 0x65)) {
    return false;
  }

  if (!qmiWriteReg(QMI8658_REG_CTRL5, 0x00)) {
    return false;
  }

  if (!qmiWriteReg(QMI8658_REG_CTRL7, 0x03)) {
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(20));
  Serial.println("[IMU] QMI8658 pronto para leitura");
  return true;
}

static bool readQmi8658Sample(ImuSample *sample)
{
  if (!sample) {
    return false;
  }

  uint8_t buf[14] = {0};
  if (!qmiReadRegs(QMI8658_REG_TEMP_L, buf, sizeof(buf))) {
    return false;
  }

  const int16_t temp_raw = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  const int16_t ax_raw = static_cast<int16_t>((buf[3] << 8) | buf[2]);
  const int16_t ay_raw = static_cast<int16_t>((buf[5] << 8) | buf[4]);
  const int16_t az_raw = static_cast<int16_t>((buf[7] << 8) | buf[6]);
  const int16_t gx_raw = static_cast<int16_t>((buf[9] << 8) | buf[8]);
  const int16_t gy_raw = static_cast<int16_t>((buf[11] << 8) | buf[10]);
  const int16_t gz_raw = static_cast<int16_t>((buf[13] << 8) | buf[12]);

  sample->temperature_c = static_cast<float>(temp_raw) / 256.0f;
  sample->acc_mps2[0] = static_cast<float>(ax_raw) * GRAVITY_MS2 / QMI8658_ACC_LSB_8G;
  sample->acc_mps2[1] = static_cast<float>(ay_raw) * GRAVITY_MS2 / QMI8658_ACC_LSB_8G;
  sample->acc_mps2[2] = static_cast<float>(az_raw) * GRAVITY_MS2 / QMI8658_ACC_LSB_8G;
  sample->gyro_dps[0] = static_cast<float>(gx_raw) / QMI8658_GYRO_LSB_1024DPS;
  sample->gyro_dps[1] = static_cast<float>(gy_raw) / QMI8658_GYRO_LSB_1024DPS;
  sample->gyro_dps[2] = static_cast<float>(gz_raw) / QMI8658_GYRO_LSB_1024DPS;

  applyImuAxisTransform(sample->acc_mps2[0], sample->acc_mps2[1], sample->acc_mps2[2]);
  applyImuAxisTransform(sample->gyro_dps[0], sample->gyro_dps[1], sample->gyro_dps[2]);
  return true;
}

static void imuTask(void *parameter)
{
  (void)parameter;

  ImuState state = {};
  bool sensor_ready = false;
  bool filter_seeded = false;
  uint8_t failure_count = 0;
  float fused_pitch = 0.0f;
  float fused_roll = 0.0f;
  uint32_t last_sample_us = 0;

  for (;;) {
    if (!sensor_ready) {
      state.connected = false;
      state.healthy = false;
      state.has_solution = false;
      state.last_update_ms = millis();
      storeImuState(state);

      if (!initQmi8658()) {
        vTaskDelay(pdMS_TO_TICKS(IMU_RETRY_PERIOD_MS));
        continue;
      }

      sensor_ready = true;
      filter_seeded = false;
      failure_count = 0;
      last_sample_us = micros();
      state.connected = true;
      state.healthy = true;
      state.last_update_ms = millis();
      storeImuState(state);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    ImuSample sample = {};
    if (!readQmi8658Sample(&sample)) {
      ++failure_count;
      state.connected = true;
      state.healthy = false;
      state.has_solution = false;
      state.last_update_ms = millis();
      storeImuState(state);

      if (failure_count >= 5) {
        Serial.println("[IMU] Leitura falhou repetidamente, reinicializando...");
        sensor_ready = false;
      }

      vTaskDelay(pdMS_TO_TICKS(g_runtime_services_light ? 90U : 40U));
      continue;
    }

    failure_count = 0;

    const uint32_t now_us = micros();
    float dt = (last_sample_us == 0) ? 0.01f : static_cast<float>(now_us - last_sample_us) / 1000000.0f;
    last_sample_us = now_us;
    if (dt <= 0.0f || dt > 0.25f) {
      dt = static_cast<float>(IMU_TASK_PERIOD_MS) / 1000.0f;
    }

    const float ax = sample.acc_mps2[0];
    const float ay = sample.acc_mps2[1];
    const float az = sample.acc_mps2[2];
    const float gx = sample.gyro_dps[0];
    const float gy = sample.gyro_dps[1];
    const float denom = sqrtf((ay * ay) + (az * az));

    const float acc_roll = atan2f(ay, az) * RAD_TO_DEG_F;
    const float acc_pitch = atan2f(-ax, denom) * RAD_TO_DEG_F;

    if (!filter_seeded) {
      fused_roll = acc_roll;
      fused_pitch = acc_pitch;
      filter_seeded = true;
    } else {
      fused_roll = (IMU_COMPLEMENTARY_ALPHA * (fused_roll + (gx * dt))) +
                   ((1.0f - IMU_COMPLEMENTARY_ALPHA) * acc_roll);
      fused_pitch = (IMU_COMPLEMENTARY_ALPHA * (fused_pitch + (gy * dt))) +
                    ((1.0f - IMU_COMPLEMENTARY_ALPHA) * acc_pitch);
    }

    state.connected = true;
    state.healthy = true;
    state.has_solution = filter_seeded;
    state.acc_mps2[0] = sample.acc_mps2[0];
    state.acc_mps2[1] = sample.acc_mps2[1];
    state.acc_mps2[2] = sample.acc_mps2[2];
    state.gyro_dps[0] = sample.gyro_dps[0];
    state.gyro_dps[1] = sample.gyro_dps[1];
    state.gyro_dps[2] = sample.gyro_dps[2];

    float pitch_trim_deg = 0.0f;
    float roll_trim_deg = 0.0f;
    getImuTrim(&pitch_trim_deg, &roll_trim_deg);

    float pitch_deg = fused_pitch - pitch_trim_deg;
    float roll_deg = fused_roll - roll_trim_deg;

    if (IMU_INVERT_PITCH_SIGN) {
      pitch_deg = -pitch_deg;
    }

    if (IMU_INVERT_ROLL_SIGN) {
      roll_deg = -roll_deg;
    }

    if (fabsf(pitch_deg) < 0.6f) pitch_deg = 0.0f;
    if (fabsf(roll_deg) < 0.6f) roll_deg = 0.0f;

    state.pitch_deg = clampFloat(pitch_deg, -89.0f, 89.0f);
    state.roll_deg = clampFloat(roll_deg, -180.0f, 180.0f);
    state.temperature_c = sample.temperature_c;
    state.sample_count += 1;
    state.last_update_ms = millis();

    storeImuState(state);
    vTaskDelay(pdMS_TO_TICKS(g_runtime_services_light ? 40U : IMU_TASK_PERIOD_MS));
  }
}

static void blackboxTask(void *parameter)
{
  (void)parameter;

  BlackboxState state = {};
  uint32_t last_mount_attempt_ms = millis() - BLACKBOX_MOUNT_RETRY_MS;
  uint32_t last_flush_ms = 0;

  state.logging_enabled = g_blackbox_enabled_target;
  snprintf(state.note, sizeof(state.note), "Aguardando cartao SD");
  storeBlackboxState(state);

  for (;;) {
    const uint32_t now = millis();

    if (g_sd_purge_requested) {
      g_sd_purge_requested = false;
      closeBlackboxFile();
      clearBlackboxTail();
      state.file_open = false;
      state.records_written = 0;
      state.file_path[0] = '\0';

      if (!state.mounted) {
        mountBlackboxStorage(&state);
      }

      if (state.mounted) {
        const uint16_t removed = clearBlackboxLogs();
        snprintf(
            state.note,
            sizeof(state.note),
            "Logs apagados: %u arquivo(s). Cartao pronto para nova missao",
            static_cast<unsigned>(removed));
      } else {
        snprintf(state.note, sizeof(state.note), "Insira um SD para limpar os logs");
      }

      storeBlackboxState(state);
    }

    if (g_blackbox_remount_requested) {
      g_blackbox_remount_requested = false;
      unmountBlackboxStorage();
      state.mounted = false;
      state.file_open = false;
      state.card_size_bytes = 0;
      state.records_written = 0;
      state.last_write_ok = false;
      state.file_path[0] = '\0';
      snprintf(state.note, sizeof(state.note), "Refazendo montagem do SD...");
      storeBlackboxState(state);
    }

    const bool target_enabled = g_blackbox_enabled_target;
    if (state.logging_enabled != target_enabled) {
      state.logging_enabled = target_enabled;

      if (!state.logging_enabled) {
        closeBlackboxFile();
        state.file_open = false;
        snprintf(
            state.note,
            sizeof(state.note),
            state.mounted ? "Logger pausado. SD continua montado" : "Logger pausado. Insira um cartao SD");
      } else {
        snprintf(
            state.note,
            sizeof(state.note),
            state.mounted ? "Logger armado. Abrindo novo arquivo..." : "Logger armado. Tentando montar SD");
      }

      storeBlackboxState(state);
    }

    if (!state.mounted && (now - last_mount_attempt_ms) >= BLACKBOX_MOUNT_RETRY_MS) {
      last_mount_attempt_ms = now;

      if (mountBlackboxStorage(&state)) {
        Serial.println("[BLACKBOX] Cartao SD montado");
        if (!state.logging_enabled) {
          snprintf(state.note, sizeof(state.note), "SD montado. Logger pausado pelo modo atual");
        }
      }

      storeBlackboxState(state);
    }

    if (state.mounted && state.logging_enabled && !state.file_open) {
      if (openBlackboxLog(&state)) {
        Serial.printf("[BLACKBOX] Arquivo ativo: %s\n", state.file_path);
        last_flush_ms = now;
      }

      storeBlackboxState(state);
    }

    if (state.mounted && state.logging_enabled && state.file_open &&
        (now - state.last_log_ms) >= BLACKBOX_LOG_PERIOD_MS) {
      if (!appendBlackboxRecord(&state, now)) {
        Serial.println("[BLACKBOX] Falha de escrita. Reiniciando armazenamento");
        unmountBlackboxStorage();
        state.mounted = false;
        state.file_open = false;
        state.card_size_bytes = 0;
        state.last_write_ok = false;
        state.file_path[0] = '\0';
        last_mount_attempt_ms = now;
      } else if ((now - last_flush_ms) >= BLACKBOX_FLUSH_PERIOD_MS) {
        g_blackbox_file.flush();
        last_flush_ms = now;
      }

      storeBlackboxState(state);
    }

    vTaskDelay(pdMS_TO_TICKS(120));
  }
}

static void lvglFlushCb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
  const uint32_t width = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  const uint32_t height = static_cast<uint32_t>(area->y2 - area->y1 + 1);

  g_panel->draw16bitRGBBitmap(
      area->x1,
      area->y1,
      reinterpret_cast<uint16_t *>(px_map),
      width,
      height);

  lv_display_flush_ready(display);
}

static void lvTickCb(void *arg)
{
  (void)arg;
  lv_tick_inc(LV_TICK_PERIOD_US / 1000U);
}

static void touchReadCb(lv_indev_t *indev, lv_indev_data_t *data)
{
  (void)indev;

  uint16_t x = g_touch_x;
  uint16_t y = g_touch_y;
  const bool pressed = readTouchPoint(&x, &y);

  if (pressed) {
    lv_point_t point = {
        static_cast<lv_coord_t>(x),
        static_cast<lv_coord_t>(y)};

    if (g_lv_display) {
      lv_display_rotate_point(g_lv_display, &point);
    }

    g_touch_x = static_cast<uint16_t>(point.x);
    g_touch_y = static_cast<uint16_t>(point.y);
    g_touch_pressed = true;
    data->point.x = point.x;
    data->point.y = point.y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    g_touch_pressed = false;
    data->point.x = g_touch_x;
    data->point.y = g_touch_y;
    data->state = LV_INDEV_STATE_RELEASED;
  }

  if (pressed && !g_touch_prev_pressed) {
    ++g_touch_press_count;
  }

  g_touch_prev_pressed = pressed;
}

static bool allocateLvglBuffers()
{
  const size_t bytes = LV_BUF_PIXELS * sizeof(lv_color_t);

  g_buf1 = static_cast<lv_color_t *>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_buf2 = static_cast<lv_color_t *>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

  if (!g_buf1) {
    Serial.println("[LVGL] PSRAM indisponivel para buf1, tentando RAM interna");
    g_buf1 = static_cast<lv_color_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
  }

  if (!g_buf2) {
    Serial.println("[LVGL] Sem buf2 em PSRAM, seguindo com buffer unico se necessario");
    g_buf2 = static_cast<lv_color_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
  }

  return g_buf1 != nullptr;
}

static void initLVGL()
{
  Serial.println("[LVGL] Inicializando...");
  lv_init();

  if (!allocateLvglBuffers()) {
    fatalStop("[LVGL] Nao foi possivel alocar buffers");
  }

  g_lv_display = lv_display_create(LCD_W, LCD_H);
  lv_display_set_default(g_lv_display);
  lv_display_set_flush_cb(g_lv_display, lvglFlushCb);
  lv_display_set_buffers(
      g_lv_display,
      g_buf1,
      g_buf2,
      LV_BUF_PIXELS * sizeof(lv_color_t),
      LV_DISPLAY_RENDER_MODE_PARTIAL);

#if defined(LV_COLOR_FORMAT_RGB565)
  lv_display_set_color_format(g_lv_display, LV_COLOR_FORMAT_RGB565);
#endif

  g_touch_indev = lv_indev_create();
  lv_indev_set_type(g_touch_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_display(g_touch_indev, g_lv_display);
  lv_indev_set_read_cb(g_touch_indev, touchReadCb);

  const esp_timer_create_args_t timer_args = {
      .callback = lvTickCb,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "lvgl_tick"};

  if (esp_timer_create(&timer_args, &g_lv_tick_timer) != ESP_OK) {
    fatalStop("[LVGL] Falha ao criar esp_timer");
  }

  if (esp_timer_start_periodic(g_lv_tick_timer, LV_TICK_PERIOD_US) != ESP_OK) {
    fatalStop("[LVGL] Falha ao iniciar esp_timer");
  }

  Serial.printf(
      "[LVGL] Buffer 1: %u bytes | Buffer 2: %u\n",
      static_cast<unsigned>(LV_BUF_PIXELS * sizeof(lv_color_t)),
      static_cast<unsigned>(g_buf2 ? LV_BUF_PIXELS * sizeof(lv_color_t) : 0));
  Serial.println("[LVGL] OK");
}

static void styleScreen(lv_obj_t *screen)
{
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x05070B), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(screen, lv_color_hex(0xFFFFFF), 0);
}

static lv_obj_t *createHeader(lv_obj_t *parent, const char *title, const char *subtitle)
{
  const int32_t header_w = uiWidth() - 16;
  const bool show_subtitle = subtitle && subtitle[0] != '\0';
  const int32_t header_h = show_subtitle ? (isLandscapeUI() ? 50 : 58) : (isLandscapeUI() ? 40 : 46);

  lv_obj_t *header = lv_obj_create(parent);
  lv_obj_remove_style_all(header);
  lv_obj_set_size(header, header_w, header_h);
  lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 8);
  lv_obj_set_style_bg_color(header, lv_color_hex(0x0F1825), 0);
  lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(header, 14, 0);
  lv_obj_set_style_pad_all(header, 12, 0);

  lv_obj_t *title_label = lv_label_create(header);
  lv_label_set_text(title_label, title);
  lv_obj_set_width(title_label, header_w - 24);
  lv_label_set_long_mode(title_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(title_label, lv_color_hex(0x00D6FF), 0);
  lv_obj_align(title_label, show_subtitle ? LV_ALIGN_TOP_LEFT : LV_ALIGN_LEFT_MID, 0, 0);

  if (show_subtitle) {
    lv_obj_t *subtitle_label = lv_label_create(header);
    lv_label_set_text(subtitle_label, subtitle);
    lv_label_set_long_mode(subtitle_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(subtitle_label, header_w - 24);
    lv_obj_set_style_text_color(subtitle_label, lv_color_hex(0xB9C2CF), 0);
    lv_obj_align(subtitle_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  }

  return header;
}

static void loadScreenById(AppScreenId screen_id)
{
  lv_obj_t *target = screenFromId(screen_id);
  if (!target) {
    return;
  }

  g_current_screen_id = screen_id;
  lv_screen_load(target);
}

static void navigateEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  const AppScreenId screen_id = static_cast<AppScreenId>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));

  loadScreenById(screen_id);
}

static void backToMenuEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  g_sd_purge_armed = false;
  if (g_current_screen_id != SCREEN_HOME) {
    loadScreenById(SCREEN_HOME);
  }
}

static lv_obj_t *createBackButton(
    lv_obj_t *parent,
    lv_align_t align,
    lv_coord_t x_ofs,
    lv_coord_t y_ofs,
    lv_coord_t width)
{
  lv_obj_t *back_btn = lv_button_create(parent);
  lv_obj_set_size(back_btn, width, 42);
  lv_obj_align(back_btn, align, x_ofs, y_ofs);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x1A2638), 0);
  lv_obj_set_style_radius(back_btn, 14, 0);
  lv_obj_add_event_cb(back_btn, backToMenuEventCb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "Voltar");
  lv_obj_center(back_label);
  return back_btn;
}

static lv_obj_t *createNavButton(
    lv_obj_t *parent,
    const char *title,
    const char *subtitle,
    uint32_t color,
    AppScreenId target,
    lv_coord_t width,
    lv_coord_t height)
{
  const bool show_subtitle = subtitle && subtitle[0] != '\0' && height >= 72 && width >= 150;
  lv_obj_t *button = lv_button_create(parent);
  lv_obj_set_size(button, width, height);
  lv_obj_set_style_bg_color(button, lv_color_hex(color), 0);
  lv_obj_set_style_bg_grad_color(button, lv_color_hex(0x101722), 0);
  lv_obj_set_style_bg_grad_dir(button, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_radius(button, 16, 0);
  lv_obj_set_style_pad_all(button, 12, 0);
  lv_obj_add_event_cb(button, navigateEventCb, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(target)));

  lv_obj_t *title_label = lv_label_create(button);
  lv_label_set_text(title_label, title);
  lv_obj_set_width(title_label, width - 24);
  lv_label_set_long_mode(title_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), 0);
  if (!show_subtitle) {
    lv_obj_center(title_label);
  } else {
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);
  }

  if (show_subtitle) {
    lv_obj_t *subtitle_label = lv_label_create(button);
    lv_label_set_text(subtitle_label, subtitle);
    lv_label_set_long_mode(subtitle_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(subtitle_label, width - 24);
    lv_obj_set_style_text_color(subtitle_label, lv_color_hex(0xD7DFE8), 0);
    lv_obj_align(subtitle_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  }

  return button;
}

static lv_obj_t *createSectionScreen(
    const char *title,
    const char *subtitle,
    const char *body,
    uint32_t accent)
{
  const int32_t panel_y = isLandscapeUI() ? 78 : 92;
  const int32_t back_h = 46;
  const int32_t panel_h = uiHeight() - panel_y - back_h - 36;

  lv_obj_t *screen = lv_obj_create(nullptr);
  styleScreen(screen);
  createHeader(screen, title, subtitle);

  lv_obj_t *panel = lv_obj_create(screen);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(panel, uiWidth() - 16, panel_h);
  lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, panel_y);
  lv_obj_set_style_bg_color(panel, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(panel, lv_color_hex(accent), 0);
  lv_obj_set_style_border_width(panel, 2, 0);
  lv_obj_set_style_radius(panel, 16, 0);
  lv_obj_set_style_pad_all(panel, 14, 0);

  lv_obj_t *body_label = lv_label_create(panel);
  lv_label_set_text(body_label, body);
  lv_label_set_long_mode(body_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(body_label, uiWidth() - 48);
  lv_obj_set_style_text_color(body_label, lv_color_hex(0xE7EDF5), 0);
  lv_obj_align(body_label, LV_ALIGN_TOP_LEFT, 0, 0);

  createBackButton(screen, LV_ALIGN_BOTTOM_MID, 0, -10, uiWidth() - 32);

  return screen;
}

static lv_obj_t *createDataCard(
    lv_obj_t *parent,
    lv_coord_t width,
    lv_coord_t height,
    lv_align_t align,
    lv_coord_t x_ofs,
    lv_coord_t y_ofs,
    uint32_t accent,
    const char *title,
    const char *value,
    lv_obj_t **value_label_out)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(card, width, height);
  lv_obj_align(card, align, x_ofs, y_ofs);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(card, lv_color_hex(accent), 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_radius(card, 16, 0);
  lv_obj_set_style_pad_all(card, height <= 40 ? 6 : 10, 0);

  lv_obj_t *title_label = lv_label_create(card);
  lv_label_set_text(title_label, title);
  lv_obj_set_style_text_color(title_label, lv_color_hex(0xAFC2D8), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *value_label = lv_label_create(card);
  lv_label_set_text(value_label, value);
  lv_obj_set_style_text_color(value_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(value_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  if (value_label_out) {
    *value_label_out = value_label;
  }

  return card;
}

static lv_obj_t *createStatusCard(
    lv_obj_t *parent,
    lv_coord_t width,
    lv_coord_t height,
    lv_align_t align,
    lv_coord_t x_ofs,
    lv_coord_t y_ofs,
    uint32_t accent,
    const char *title,
    lv_obj_t **body_label_out)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(card, width, height);
  lv_obj_align(card, align, x_ofs, y_ofs);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(card, lv_color_hex(accent), 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_radius(card, 16, 0);
  lv_obj_set_style_pad_all(card, 10, 0);

  lv_obj_t *title_label = lv_label_create(card);
  lv_label_set_text(title_label, title);
  lv_obj_set_style_text_color(title_label, lv_color_hex(0xAFC2D8), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *body_label = lv_label_create(card);
  lv_label_set_text(body_label, "");
  lv_label_set_long_mode(body_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(body_label, width - 20);
  lv_obj_set_style_text_color(body_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(body_label, LV_ALIGN_TOP_LEFT, 0, 22);

  if (body_label_out) {
    *body_label_out = body_label;
  }

  return card;
}

static lv_obj_t *createActionButton(
    lv_obj_t *parent,
    lv_coord_t width,
    lv_coord_t height,
    lv_align_t align,
    lv_coord_t x_ofs,
    lv_coord_t y_ofs,
    uint32_t color,
    const char *label_text,
    lv_event_cb_t callback,
    lv_obj_t **label_out)
{
  lv_obj_t *button = lv_button_create(parent);
  lv_obj_set_size(button, width, height);
  lv_obj_align(button, align, x_ofs, y_ofs);
  lv_obj_set_style_bg_color(button, lv_color_hex(color), 0);
  lv_obj_set_style_radius(button, 14, 0);
  lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *label = lv_label_create(button);
  lv_label_set_text(label, label_text);
  lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
  lv_obj_set_width(label, width - 18);
  lv_obj_center(label);

  if (label_out) {
    *label_out = label;
  }

  return button;
}

static lv_obj_t *createLocalScrollBody(lv_obj_t *parent, lv_coord_t top_y, lv_coord_t bottom_reserved)
{
  lv_coord_t body_h = uiHeight() - top_y - bottom_reserved;
  if (body_h < 96) {
    body_h = 96;
  }

  lv_obj_t *body = lv_obj_create(parent);
  lv_obj_set_size(body, uiWidth() - 8, body_h);
  lv_obj_align(body, LV_ALIGN_TOP_MID, 0, top_y);
  lv_obj_set_style_bg_opa(body, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(body, 0, 0);
  lv_obj_set_style_radius(body, 0, 0);
  lv_obj_set_style_pad_left(body, 0, 0);
  lv_obj_set_style_pad_right(body, 0, 0);
  lv_obj_set_style_pad_top(body, 0, 0);
  lv_obj_set_style_pad_bottom(body, 10, 0);
  lv_obj_set_scroll_dir(body, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(body, LV_SCROLLBAR_MODE_AUTO);
  return body;
}

static lv_obj_t *createLocalMenuCard(
    lv_obj_t *parent,
    lv_coord_t width,
    lv_coord_t height,
    lv_coord_t y,
    uint32_t accent,
    const char *title)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(card, width, height);
  lv_obj_align(card, LV_ALIGN_TOP_MID, 0, y);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(card, lv_color_hex(accent), 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_radius(card, 16, 0);
  lv_obj_set_style_pad_all(card, 10, 0);

  lv_obj_t *title_label = lv_label_create(card);
  lv_label_set_text(title_label, title);
  lv_obj_set_style_text_color(title_label, lv_color_hex(0xE7EDF5), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);
  return card;
}

static bool ensureEfisCanvasBuffer()
{
  if (g_efis_canvas_buf) {
    return true;
  }

  const size_t bytes = EFIS_HORIZON_PIXELS * sizeof(lv_color_t);
  g_efis_canvas_buf = static_cast<lv_color_t *>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

  if (!g_efis_canvas_buf) {
    Serial.println("[EFIS] Canvas em PSRAM indisponivel, tentando RAM interna");
    g_efis_canvas_buf = static_cast<lv_color_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
  }

  if (!g_efis_canvas_buf) {
    Serial.println("[EFIS] Falha ao alocar buffer do horizonte artificial");
    return false;
  }

  return true;
}

static lv_point_precise_t makePoint(float x, float y)
{
  lv_point_precise_t point;
  point.x = x;
  point.y = y;
  return point;
}

static void drawCanvasTriangle(
    lv_layer_t *layer,
    lv_color_t color,
    lv_opa_t opa,
    lv_point_precise_t p1,
    lv_point_precise_t p2,
    lv_point_precise_t p3)
{
  lv_draw_triangle_dsc_t dsc;
  lv_draw_triangle_dsc_init(&dsc);
  dsc.color = color;
  dsc.opa = opa;
  dsc.p[0] = p1;
  dsc.p[1] = p2;
  dsc.p[2] = p3;
  lv_draw_triangle(layer, &dsc);
}

static void drawCanvasLine(
    lv_layer_t *layer,
    lv_color_t color,
    int32_t width,
    lv_opa_t opa,
    lv_point_precise_t p1,
    lv_point_precise_t p2)
{
  lv_draw_line_dsc_t dsc;
  lv_draw_line_dsc_init(&dsc);
  dsc.color = color;
  dsc.width = width;
  dsc.opa = opa;
  dsc.round_start = 1;
  dsc.round_end = 1;
  dsc.p1 = p1;
  dsc.p2 = p2;
  lv_draw_line(layer, &dsc);
}

static void drawCanvasRect(
    lv_layer_t *layer,
    lv_color_t color,
    lv_opa_t opa,
    int32_t x1,
    int32_t y1,
    int32_t x2,
    int32_t y2,
    int32_t radius = 0)
{
  lv_draw_rect_dsc_t dsc;
  lv_draw_rect_dsc_init(&dsc);
  dsc.bg_color = color;
  dsc.bg_opa = opa;
  dsc.radius = radius;
  dsc.border_width = 0;

  lv_area_t area;
  area.x1 = x1;
  area.y1 = y1;
  area.x2 = x2;
  area.y2 = y2;
  lv_draw_rect(layer, &dsc, &area);
}

static void drawEfisHorizon(const ImuState &imu)
{
  if (!g_efis_horizon || !g_efis_canvas_buf) {
    return;
  }

  lv_canvas_fill_bg(g_efis_horizon, lv_color_hex(0x020406), LV_OPA_COVER);

  lv_layer_t layer;
  lv_canvas_init_layer(g_efis_horizon, &layer);

  const float horizon_w = static_cast<float>(g_efis_horizon_w);
  const float horizon_h = static_cast<float>(g_efis_horizon_h);
  const float cx = horizon_w * 0.5f;
  const float cy = horizon_h * 0.5f;
  const float pitch_px = clampFloat(imu.pitch_deg, -35.0f, 35.0f) * IMU_PIXELS_PER_DEG;
  const float roll_rad = imu.roll_deg * DEG_TO_RAD_F;
  const float dx = cosf(roll_rad);
  const float dy = sinf(roll_rad);
  const float nx = -sinf(roll_rad);
  const float ny = cosf(roll_rad);
  const float span = (horizon_w > horizon_h ? horizon_w : horizon_h) * 1.2f;
  const float fill = (horizon_w > horizon_h ? horizon_w : horizon_h) * 1.5f;
  const float base_x = cx + (nx * pitch_px);
  const float base_y = cy + (ny * pitch_px);

  const lv_point_precise_t horizon_a = makePoint(base_x - (dx * span), base_y - (dy * span));
  const lv_point_precise_t horizon_b = makePoint(base_x + (dx * span), base_y + (dy * span));

  const lv_color_t sky_color = lv_color_hex(imu.connected ? 0x4C93FF : 0x293342);
  const lv_color_t ground_color = lv_color_hex(imu.connected ? 0x8A4D22 : 0x2E231C);
  const lv_color_t ladder_color = lv_color_hex(0xF1F5FA);
  const lv_color_t accent_color = lv_color_hex(0xFFD34D);
  const float bank_outer_r = (horizon_w < horizon_h ? horizon_w : horizon_h) * 0.44f;
  const float bank_inner_r_major = bank_outer_r - 16.0f;
  const float bank_inner_r_minor = bank_outer_r - 10.0f;
  const float wing_outer = (horizon_w < horizon_h ? horizon_w : horizon_h) * 0.26f;
  const float wing_inner = wing_outer * 0.32f;

  const lv_point_precise_t sky_a = horizon_a;
  const lv_point_precise_t sky_b = horizon_b;
  const lv_point_precise_t sky_c = makePoint(horizon_b.x - (nx * fill), horizon_b.y - (ny * fill));
  const lv_point_precise_t sky_d = makePoint(horizon_a.x - (nx * fill), horizon_a.y - (ny * fill));
  const lv_point_precise_t ground_a = horizon_a;
  const lv_point_precise_t ground_b = horizon_b;
  const lv_point_precise_t ground_c = makePoint(horizon_b.x + (nx * fill), horizon_b.y + (ny * fill));
  const lv_point_precise_t ground_d = makePoint(horizon_a.x + (nx * fill), horizon_a.y + (ny * fill));

  drawCanvasTriangle(&layer, sky_color, LV_OPA_COVER, sky_a, sky_b, sky_c);
  drawCanvasTriangle(&layer, sky_color, LV_OPA_COVER, sky_a, sky_c, sky_d);
  drawCanvasTriangle(&layer, ground_color, LV_OPA_COVER, ground_a, ground_b, ground_c);
  drawCanvasTriangle(&layer, ground_color, LV_OPA_COVER, ground_a, ground_c, ground_d);

  for (int deg = -30; deg <= 30; deg += 10) {
    if (deg == 0) {
      continue;
    }

    const float ladder_offset = pitch_px - (static_cast<float>(deg) * IMU_PIXELS_PER_DEG);
    const float line_cx = cx + (nx * ladder_offset);
    const float line_cy = cy + (ny * ladder_offset);
    const float half_len = (deg % 20 == 0) ? 34.0f : 26.0f;

    const lv_point_precise_t line_start = makePoint(line_cx - (dx * half_len), line_cy - (dy * half_len));
    const lv_point_precise_t line_end = makePoint(line_cx + (dx * half_len), line_cy + (dy * half_len));

    drawCanvasLine(&layer, ladder_color, 2, LV_OPA_80, line_start, line_end);
  }

  drawCanvasLine(&layer, lv_color_hex(0xFFFFFF), 4, LV_OPA_COVER, horizon_a, horizon_b);
  drawCanvasLine(&layer, lv_color_hex(0xA5B2C5), 2, LV_OPA_60, sky_d, sky_c);

  const int8_t bank_marks[] = {-60, -45, -30, 0, 30, 45, 60};
  for (int8_t angle_deg : bank_marks) {
    const float angle_rad = (static_cast<float>(angle_deg) - 90.0f) * DEG_TO_RAD_F;
    const float outer_r = bank_outer_r;
    const float inner_r = (angle_deg == 0) ? bank_inner_r_major : bank_inner_r_minor;

    const lv_point_precise_t p_outer = makePoint(cx + (cosf(angle_rad) * outer_r), cy + (sinf(angle_rad) * outer_r));
    const lv_point_precise_t p_inner = makePoint(cx + (cosf(angle_rad) * inner_r), cy + (sinf(angle_rad) * inner_r));
    drawCanvasLine(&layer, lv_color_hex(0xFFFFFF), 2, LV_OPA_80, p_inner, p_outer);
  }

  drawCanvasTriangle(
      &layer,
      accent_color,
      LV_OPA_COVER,
      makePoint(cx, 14.0f),
      makePoint(cx - 9.0f, 2.0f),
      makePoint(cx + 9.0f, 2.0f));

  drawCanvasLine(&layer, accent_color, 4, LV_OPA_COVER, makePoint(cx - wing_outer, cy), makePoint(cx - wing_inner, cy));
  drawCanvasLine(&layer, accent_color, 4, LV_OPA_COVER, makePoint(cx + wing_inner, cy), makePoint(cx + wing_outer, cy));
  drawCanvasLine(&layer, accent_color, 3, LV_OPA_COVER, makePoint(cx - wing_inner, cy), makePoint(cx - 6.0f, cy + 10.0f));
  drawCanvasLine(&layer, accent_color, 3, LV_OPA_COVER, makePoint(cx + wing_inner, cy), makePoint(cx + 6.0f, cy + 10.0f));
  drawCanvasRect(&layer, accent_color, LV_OPA_COVER, static_cast<int32_t>(cx - 6), static_cast<int32_t>(cy - 4), static_cast<int32_t>(cx + 6), static_cast<int32_t>(cy + 8), 3);
  drawCanvasLine(&layer, lv_color_hex(0xFFFFFF), 2, LV_OPA_70, makePoint(cx - 14.0f, cy + 18.0f), makePoint(cx + 14.0f, cy + 18.0f));

  lv_canvas_finish_layer(g_efis_horizon, &layer);
}

static void createEfisScreen()
{
  const bool landscape = isLandscapeUI();
  const int32_t top_y = landscape ? 64 : 70;
  const lv_coord_t screen_w = uiWidth();
  const lv_coord_t screen_h = uiHeight();
  const lv_coord_t summary_h = landscape ? 84 : 88;
  const lv_coord_t action_h = 42;
  const lv_coord_t bottom_reserved = landscape ? 64 : 114;
  const lv_coord_t info_y = top_y + summary_h + 8;
  const lv_coord_t info_h = screen_h - info_y - bottom_reserved;
  const lv_coord_t info_panel_w = screen_w - 16;
  g_efis_horizon_w = 0;
  g_efis_horizon_h = 0;
  g_efis_horizon = nullptr;
  g_efis_focus_horizon = false;

  g_screen_efis = lv_obj_create(nullptr);
  styleScreen(g_screen_efis);
  createHeader(g_screen_efis, "PLANE", "voo e dados");

  lv_obj_t *summary_panel = lv_obj_create(g_screen_efis);
  lv_obj_clear_flag(summary_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(summary_panel, screen_w - 16, summary_h);
  lv_obj_align(summary_panel, LV_ALIGN_TOP_MID, 0, top_y);
  lv_obj_set_style_bg_color(summary_panel, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(summary_panel, lv_color_hex(0x2E7DD1), 0);
  lv_obj_set_style_border_width(summary_panel, 2, 0);
  lv_obj_set_style_radius(summary_panel, 16, 0);
  lv_obj_set_style_pad_all(summary_panel, 10, 0);

  g_lbl_efis_pitch = lv_label_create(summary_panel);
  lv_label_set_text(g_lbl_efis_pitch, "Pitch --.-");
  lv_obj_set_style_text_color(g_lbl_efis_pitch, lv_color_hex(0xFFD34D), 0);
  lv_obj_align(g_lbl_efis_pitch, LV_ALIGN_TOP_LEFT, 0, 0);

  g_lbl_efis_roll = lv_label_create(summary_panel);
  lv_label_set_text(g_lbl_efis_roll, "Roll --.-");
  lv_obj_set_style_text_color(g_lbl_efis_roll, lv_color_hex(0x9BDAFF), 0);
  lv_obj_align(g_lbl_efis_roll, LV_ALIGN_TOP_RIGHT, 0, 0);

  g_lbl_efis_status = lv_label_create(summary_panel);
  lv_label_set_text(g_lbl_efis_status, "Altitude, velocidade, rumo e razao vertical.");
  lv_label_set_long_mode(g_lbl_efis_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_efis_status, screen_w - 40);
  lv_obj_set_style_text_color(g_lbl_efis_status, lv_color_hex(0xE7EDF5), 0);
  lv_obj_align(g_lbl_efis_status, LV_ALIGN_TOP_LEFT, 0, 28);


  g_efis_info_panel = lv_obj_create(g_screen_efis);
  lv_obj_clear_flag(g_efis_info_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(g_efis_info_panel, info_panel_w, info_h);
  lv_obj_align(g_efis_info_panel, LV_ALIGN_TOP_MID, 0, info_y);
  lv_obj_set_style_bg_color(g_efis_info_panel, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(g_efis_info_panel, lv_color_hex(0x2E7DD1), 0);
  lv_obj_set_style_border_width(g_efis_info_panel, 2, 0);
  lv_obj_set_style_radius(g_efis_info_panel, 16, 0);
  lv_obj_set_style_pad_all(g_efis_info_panel, 8, 0);

  const lv_coord_t plane_panel_w = info_panel_w - 20;
  const lv_coord_t half_w = (plane_panel_w - 8) / 2;
  const lv_coord_t card_h = (info_h - 26) / 2;
  const lv_coord_t row2_y = card_h + 8;
  createDataCard(g_efis_info_panel, half_w, card_h, LV_ALIGN_TOP_LEFT, 0, 0, 0xFFD34D, "ALT", "---- m", &g_lbl_plane_altitude);
  createDataCard(g_efis_info_panel, half_w, card_h, LV_ALIGN_TOP_RIGHT, 0, 0, 0x45D1FF, "V/S", "---- m/s", &g_lbl_plane_vario);
  createDataCard(g_efis_info_panel, half_w, card_h, LV_ALIGN_TOP_LEFT, 0, row2_y, 0x53C2A3, "HDG", "--- dg", &g_lbl_plane_heading);
  createDataCard(g_efis_info_panel, half_w, card_h, LV_ALIGN_TOP_RIGHT, 0, row2_y, 0xFFB357, "SPD", "-- km/h", &g_lbl_plane_speed);

  g_efis_horizon_panel = lv_obj_create(g_screen_efis);
  lv_obj_clear_flag(g_efis_horizon_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(g_efis_horizon_panel, info_panel_w, info_h);
  lv_obj_align(g_efis_horizon_panel, LV_ALIGN_TOP_MID, 0, info_y);
  lv_obj_set_style_bg_color(g_efis_horizon_panel, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(g_efis_horizon_panel, lv_color_hex(0x2E7DD1), 0);
  lv_obj_set_style_border_width(g_efis_horizon_panel, 2, 0);
  lv_obj_set_style_radius(g_efis_horizon_panel, 16, 0);
  lv_obj_set_style_pad_all(g_efis_horizon_panel, 8, 0);

  const lv_coord_t horizon_size = (info_panel_w - 24) < (info_h - 86) ? (info_panel_w - 24) : (info_h - 86);
  lv_coord_t clamped_horizon = horizon_size;
  if (clamped_horizon < 120) {
    clamped_horizon = 120;
  }
  if (clamped_horizon > static_cast<lv_coord_t>(EFIS_HORIZON_W)) {
    clamped_horizon = EFIS_HORIZON_W;
  }
  g_efis_horizon_w = static_cast<uint16_t>(clamped_horizon);
  g_efis_horizon_h = g_efis_horizon_w;
  if (ensureEfisCanvasBuffer()) {
    g_efis_horizon = lv_canvas_create(g_efis_horizon_panel);
    lv_canvas_set_buffer(
        g_efis_horizon,
        g_efis_canvas_buf,
        g_efis_horizon_w,
        g_efis_horizon_h,
        LV_COLOR_FORMAT_RGB565);
    lv_obj_align(g_efis_horizon, LV_ALIGN_TOP_MID, 0, 4);
  }

  g_lbl_efis_horizon_alt = lv_label_create(g_efis_horizon_panel);
  lv_label_set_text(g_lbl_efis_horizon_alt, "ALT ---- m");
  lv_obj_set_style_text_color(g_lbl_efis_horizon_alt, lv_color_hex(0xFFD34D), 0);
  lv_obj_align(g_lbl_efis_horizon_alt, LV_ALIGN_BOTTOM_LEFT, 4, -8);

  g_lbl_efis_horizon_speed = lv_label_create(g_efis_horizon_panel);
  lv_label_set_text(g_lbl_efis_horizon_speed, "SPD -- km/h");
  lv_obj_set_style_text_color(g_lbl_efis_horizon_speed, lv_color_hex(0x45D1FF), 0);
  lv_obj_align(g_lbl_efis_horizon_speed, LV_ALIGN_BOTTOM_RIGHT, -4, -8);

  lv_obj_add_flag(g_efis_horizon_panel, LV_OBJ_FLAG_HIDDEN);

  if (g_efis_horizon) {
    drawEfisHorizon(copyImuState());
  }

  if (landscape) {
    const lv_coord_t action_w = (screen_w - 48) / 3;

    lv_obj_t *toggle_btn = lv_button_create(g_screen_efis);
    lv_obj_set_size(toggle_btn, action_w, action_h);
    lv_obj_align(toggle_btn, LV_ALIGN_BOTTOM_LEFT, 12, -10);
    lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x1E4B66), 0);
    lv_obj_set_style_radius(toggle_btn, 12, 0);
    lv_obj_add_event_cb(toggle_btn, efisToggleViewEventCb, LV_EVENT_CLICKED, nullptr);
    g_lbl_efis_toggle = lv_label_create(toggle_btn);
    lv_label_set_text(g_lbl_efis_toggle, "Modo horizonte");
    lv_obj_center(g_lbl_efis_toggle);

    lv_obj_t *calibrate_btn = lv_button_create(g_screen_efis);
    lv_obj_set_size(calibrate_btn, action_w, action_h);
    lv_obj_align(calibrate_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(calibrate_btn, lv_color_hex(0x24384A), 0);
    lv_obj_set_style_radius(calibrate_btn, 12, 0);
    lv_obj_add_event_cb(calibrate_btn, efisSetLevelEventCb, LV_EVENT_CLICKED, nullptr);
    g_lbl_efis_calibration = lv_label_create(calibrate_btn);
    lv_label_set_text(g_lbl_efis_calibration, "Setar nivel");
    lv_obj_center(g_lbl_efis_calibration);

    createBackButton(g_screen_efis, LV_ALIGN_BOTTOM_RIGHT, -12, -10, action_w);
  } else {
    const lv_coord_t half_btn_w = (screen_w - 24) / 2;

    lv_obj_t *toggle_btn = lv_button_create(g_screen_efis);
    lv_obj_set_size(toggle_btn, half_btn_w, action_h);
    lv_obj_align(toggle_btn, LV_ALIGN_BOTTOM_LEFT, 8, -58);
    lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x1E4B66), 0);
    lv_obj_set_style_radius(toggle_btn, 12, 0);
    lv_obj_add_event_cb(toggle_btn, efisToggleViewEventCb, LV_EVENT_CLICKED, nullptr);
    g_lbl_efis_toggle = lv_label_create(toggle_btn);
    lv_label_set_text(g_lbl_efis_toggle, "Modo horizonte");
    lv_obj_center(g_lbl_efis_toggle);

    lv_obj_t *calibrate_btn = lv_button_create(g_screen_efis);
    lv_obj_set_size(calibrate_btn, half_btn_w, action_h);
    lv_obj_align(calibrate_btn, LV_ALIGN_BOTTOM_RIGHT, -8, -58);
    lv_obj_set_style_bg_color(calibrate_btn, lv_color_hex(0x24384A), 0);
    lv_obj_set_style_radius(calibrate_btn, 12, 0);
    lv_obj_add_event_cb(calibrate_btn, efisSetLevelEventCb, LV_EVENT_CLICKED, nullptr);
    g_lbl_efis_calibration = lv_label_create(calibrate_btn);
    lv_label_set_text(g_lbl_efis_calibration, "Setar nivel");
    lv_obj_center(g_lbl_efis_calibration);

    createBackButton(g_screen_efis, LV_ALIGN_BOTTOM_MID, 0, -10, screen_w - 32);
  }

  applyEfisViewMode();
}

static void applyEfisViewMode()
{
  const bool horizon_ready = g_efis_horizon_panel != nullptr;
  const bool show_horizon = g_efis_focus_horizon && horizon_ready;

  if (g_efis_info_panel) {
    if (show_horizon) {
      lv_obj_add_flag(g_efis_info_panel, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_remove_flag(g_efis_info_panel, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (g_efis_horizon_panel) {
    if (show_horizon) {
      lv_obj_remove_flag(g_efis_horizon_panel, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(g_efis_horizon_panel, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (g_lbl_efis_toggle) {
    lv_label_set_text(g_lbl_efis_toggle, show_horizon ? "Modo cards" : "Modo horizonte");
  }
}

static void efisToggleViewEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  if (!g_efis_horizon_panel) {
    if (g_lbl_efis_status) {
      lv_label_set_text(g_lbl_efis_status, "Horizonte indisponivel nesta build");
    }
    return;
  }

  g_efis_focus_horizon = !g_efis_focus_horizon;
  applyEfisViewMode();
}

static lv_display_rotation_t autoRotationFromImu(const ImuState &imu)
{
  const float ax = imu.acc_mps2[0];
  const float ay = imu.acc_mps2[1];

  if (fabsf(ax) > fabsf(ay) + 2.0f) {
    return (ax >= 0.0f) ? LV_DISPLAY_ROTATION_90 : LV_DISPLAY_ROTATION_270;
  }

  if (fabsf(ay) > fabsf(ax) + 2.0f) {
    return (ay >= 0.0f) ? LV_DISPLAY_ROTATION_180 : LV_DISPLAY_ROTATION_0;
  }

  return g_display_rotation;
}

static lv_display_rotation_t rotationForMode(OrientationMode mode)
{
  if (mode == ORIENTATION_MODE_LANDSCAPE) {
    return LV_DISPLAY_ROTATION_90;
  }

  if (mode == ORIENTATION_MODE_AUTO && ENABLE_AUTO_ROTATION_UI) {
    const ImuState imu = copyImuState();
    if (imu.connected && imu.healthy) {
      return autoRotationFromImu(imu);
    }
  }

  return LV_DISPLAY_ROTATION_0;
}

static void resetUiPointers()
{
  g_screen_main = nullptr;
  g_screen_efis = nullptr;
  g_screen_meteo = nullptr;
  g_screen_gps = nullptr;
  g_screen_lora = nullptr;
  g_screen_config = nullptr;
  g_lbl_uptime = nullptr;
  g_lbl_i2c = nullptr;
  g_lbl_touch = nullptr;
  g_touch_dot = nullptr;
  g_efis_horizon = nullptr;
  g_efis_info_panel = nullptr;
  g_efis_horizon_panel = nullptr;
  g_lbl_efis_pitch = nullptr;
  g_lbl_efis_roll = nullptr;
  g_lbl_efis_status = nullptr;
  g_lbl_efis_calibration = nullptr;
  g_lbl_efis_toggle = nullptr;
  g_lbl_plane_altitude = nullptr;
  g_lbl_plane_vario = nullptr;
  g_lbl_plane_heading = nullptr;
  g_lbl_plane_speed = nullptr;
  g_lbl_plane_skydive = nullptr;
  g_lbl_efis_horizon_alt = nullptr;
  g_lbl_efis_horizon_speed = nullptr;
  g_meteo_hero = nullptr;
  g_meteo_panel = nullptr;
  g_lbl_meteo_theme = nullptr;
  g_lbl_meteo_status = nullptr;
  g_lbl_meteo_cards = nullptr;
  g_lbl_meteo_altitude = nullptr;
  g_lbl_meteo_pressure = nullptr;
  g_lbl_meteo_temperature = nullptr;
  g_lbl_meteo_humidity = nullptr;
  g_lbl_comms_status = nullptr;
  g_lbl_comms_wifi = nullptr;
  g_lbl_comms_ble = nullptr;
  g_lbl_comms_lora = nullptr;
  g_lbl_comms_gps = nullptr;
  g_lbl_comms_mode = nullptr;
  g_lbl_comms_wifi_btn = nullptr;
  g_lbl_comms_sta_btn = nullptr;
  g_lbl_comms_lora_btn = nullptr;
  g_lbl_comms_rescan_btn = nullptr;
  g_lbl_lora_status = nullptr;
  g_lbl_lora_radio = nullptr;
  g_lbl_lora_test = nullptr;
  g_lbl_lora_toggle_btn = nullptr;
  g_lbl_home_hint = nullptr;
  g_lbl_config_orientation = nullptr;
  g_lbl_config_rotation = nullptr;
  g_lbl_config_net = nullptr;
  g_lbl_config_sensors = nullptr;
  g_lbl_config_blackbox = nullptr;
  g_lbl_config_note = nullptr;
  g_lbl_config_scan_raw = nullptr;
  g_lbl_config_refresh = nullptr;
  g_lbl_config_blackbox_btn = nullptr;
  g_lbl_config_format_btn = nullptr;
  g_lbl_config_wifi_btn = nullptr;
  g_lbl_config_lora_btn = nullptr;
  g_lbl_config_brightness = nullptr;
  g_lbl_config_cdc = nullptr;
}

static void createConfigScreen();
static void createMeteoScreen();
static void createCommsScreen();
static void createLoraScreen();
static void createUI(AppScreenId initial_screen_id);

static void rebuildUI(AppScreenId target_screen_id)
{
  lv_obj_t *old_main = g_screen_main;
  lv_obj_t *old_efis = g_screen_efis;
  lv_obj_t *old_meteo = g_screen_meteo;
  lv_obj_t *old_gps = g_screen_gps;
  lv_obj_t *old_lora = g_screen_lora;
  lv_obj_t *old_config = g_screen_config;

  resetUiPointers();
  createUI(target_screen_id);

  if (old_main) {
    lv_obj_delete(old_main);
  }
  if (old_efis) {
    lv_obj_delete(old_efis);
  }
  if (old_meteo) {
    lv_obj_delete(old_meteo);
  }
  if (old_gps) {
    lv_obj_delete(old_gps);
  }
  if (old_lora) {
    lv_obj_delete(old_lora);
  }
  if (old_config) {
    lv_obj_delete(old_config);
  }
}

static void applyDisplayRotation(lv_display_rotation_t rotation, AppScreenId keep_screen_id)
{
  if (rotation == g_display_rotation && g_screen_main != nullptr) {
    return;
  }

  g_display_rotation = rotation;

  if (g_panel) {
    g_panel->setRotation(static_cast<uint8_t>(rotation));
    g_panel->fillScreen(COLOR_BLACK);
  }

  if (g_lv_display) {
    lv_display_set_rotation(g_lv_display, rotation);
  }

  rebuildUI(keep_screen_id);
}

static void applyOrientationMode(OrientationMode mode, AppScreenId keep_screen_id)
{
  g_orientation_mode = mode;
  const lv_display_rotation_t next_rotation = rotationForMode(mode);
  applyDisplayRotation(next_rotation, keep_screen_id);
}

static void refreshSensorsEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  refreshSensorCaches();
}

static void toggleWifiEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  const bool next_ap_state = !g_wifi_ap_requested;
  if (next_ap_state) {
    g_sd_purge_armed = false;
    loadScreenById(SCREEN_GPS);
    Serial.println("[WIFI] Pedido para ligar o AP via UI");
  } else {
    Serial.println("[WIFI] Pedido para desligar o AP via UI");
  }
  requestWifiMode(next_ap_state, "ui");
}

static void toggleWifiStaEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  if (!g_wifi_sta_requested && !g_wifi_sta_ssid[0]) {
    Serial.println("[WIFI] LAN salva indisponivel: defina SSID/senha pela web primeiro");
    snprintf(g_wifi_sta_note, sizeof(g_wifi_sta_note), "Defina SSID/senha pela web antes de ligar a LAN");
    return;
  }

  requestWifiStaMode(!g_wifi_sta_requested, "ui");
}

static void toggleLoraEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  requestLoraMode(!g_lora_enabled, "ui");
}

static void openLoraScreenEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  loadScreenById(SCREEN_LORA);
}

static void backToCommsEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  g_sd_purge_armed = false;
  if (g_current_screen_id != SCREEN_GPS) {
    loadScreenById(SCREEN_GPS);
  }
}

static void rescanCommsEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  requestWifiNetworkScan("ui");
  Serial.println("[COMMS] Scan Wi-Fi solicitado pela UI");
}

static void toggleBlackboxEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  g_blackbox_enabled_target = !g_blackbox_enabled_target;
}

static void brightnessDownEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  applyPanelBrightness(static_cast<uint8_t>(g_display_brightness > 40 ? g_display_brightness - 24 : 24));
  Serial.printf("[DISPLAY] Brilho ajustado para %u\n", static_cast<unsigned>(g_display_brightness));
}

static void brightnessUpEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  applyPanelBrightness(static_cast<uint8_t>(g_display_brightness < 231 ? g_display_brightness + 24 : 255));
  Serial.printf("[DISPLAY] Brilho ajustado para %u\n", static_cast<unsigned>(g_display_brightness));
}

static void purgeSdLogsEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  const uint32_t now = millis();
  if (!g_sd_purge_armed || (now - g_sd_purge_arm_ms) > SD_PURGE_CONFIRM_MS) {
    g_sd_purge_armed = true;
    g_sd_purge_arm_ms = now;
    Serial.println("[BLACKBOX] Confirmacao de limpeza armada. Toque novamente para apagar /logs");
    return;
  }

  g_sd_purge_armed = false;
  g_sd_purge_requested = true;
  Serial.println("[BLACKBOX] Limpeza dos logs do SD solicitada");
}

static void efisSetLevelEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  const ImuState imu = copyImuState();
  float pitch_trim_deg = 0.0f;
  float roll_trim_deg = 0.0f;
  getImuTrim(&pitch_trim_deg, &roll_trim_deg);
  setImuTrim(pitch_trim_deg + imu.pitch_deg, roll_trim_deg + imu.roll_deg);
}

static void createConfigScreen()
{
  const lv_coord_t top_y = isLandscapeUI() ? 54 : 60;
  const lv_coord_t footer_h = 44;
  const lv_coord_t footer_gap = 16;
  const lv_coord_t body_bottom_reserved = footer_h + footer_gap + 12;
  const lv_coord_t card_w = uiWidth() - 24;
  const lv_coord_t half_btn_w = (card_w - 12) / 2;
  const lv_coord_t footer_btn_w = (uiWidth() - 40) / 2;

  g_screen_config = lv_obj_create(nullptr);
  styleScreen(g_screen_config);
  createHeader(g_screen_config, "CONFIG", "");

  lv_obj_t *content = createLocalScrollBody(g_screen_config, top_y, body_bottom_reserved);
  lv_coord_t y = 0;

  lv_obj_t *display_card = createLocalMenuCard(content, card_w, 176, y, 0xA44CB4, "Tela");
  g_lbl_config_orientation = lv_label_create(display_card);
  lv_label_set_text(g_lbl_config_orientation, "Modo: Vertical");
  lv_obj_set_style_text_color(g_lbl_config_orientation, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(g_lbl_config_orientation, LV_ALIGN_TOP_LEFT, 0, 24);

  g_lbl_config_rotation = lv_label_create(display_card);
  lv_label_set_text(g_lbl_config_rotation, "Ativa: Vertical");
  lv_obj_set_style_text_color(g_lbl_config_rotation, lv_color_hex(0xC9D4E5), 0);
  lv_obj_align(g_lbl_config_rotation, LV_ALIGN_TOP_LEFT, 0, 46);

  g_lbl_config_brightness = lv_label_create(display_card);
  lv_label_set_text(g_lbl_config_brightness, "Brilho: 255");
  lv_obj_set_style_text_color(g_lbl_config_brightness, lv_color_hex(0xFFF2C2), 0);
  lv_obj_align(g_lbl_config_brightness, LV_ALIGN_TOP_LEFT, 0, 68);

  g_lbl_config_cdc = lv_label_create(display_card);
  lv_label_set_text(g_lbl_config_cdc, "USB/JTAG 115200");
  lv_label_set_long_mode(g_lbl_config_cdc, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_cdc, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_config_cdc, lv_color_hex(0xC9D4E5), 0);
  lv_obj_align(g_lbl_config_cdc, LV_ALIGN_TOP_LEFT, 0, 84);

  lv_obj_t *brightness_down_btn = lv_button_create(display_card);
  lv_obj_set_size(brightness_down_btn, half_btn_w, 34);
  lv_obj_align(brightness_down_btn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(brightness_down_btn, lv_color_hex(0x31404F), 0);
  lv_obj_set_style_radius(brightness_down_btn, 12, 0);
  lv_obj_add_event_cb(brightness_down_btn, brightnessDownEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *brightness_down_label = lv_label_create(brightness_down_btn);
  lv_label_set_text(brightness_down_label, "Brilho -");
  lv_obj_center(brightness_down_label);

  lv_obj_t *brightness_up_btn = lv_button_create(display_card);
  lv_obj_set_size(brightness_up_btn, half_btn_w, 34);
  lv_obj_align(brightness_up_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(brightness_up_btn, lv_color_hex(0x765529), 0);
  lv_obj_set_style_radius(brightness_up_btn, 12, 0);
  lv_obj_add_event_cb(brightness_up_btn, brightnessUpEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *brightness_up_label = lv_label_create(brightness_up_btn);
  lv_label_set_text(brightness_up_label, "Brilho +");
  lv_obj_center(brightness_up_label);

  y += 186;

  lv_obj_t *net_card = createLocalMenuCard(content, card_w, 168, y, 0x46D8A8, "Rede e links");
  g_lbl_config_net = lv_label_create(net_card);
  lv_label_set_text(g_lbl_config_net, "");
  lv_label_set_long_mode(g_lbl_config_net, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_net, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_config_net, lv_color_hex(0xEAFBF4), 0);
  lv_obj_align(g_lbl_config_net, LV_ALIGN_TOP_LEFT, 0, 24);

  lv_obj_t *wifi_btn = lv_button_create(net_card);
  lv_obj_set_size(wifi_btn, half_btn_w, 34);
  lv_obj_align(wifi_btn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(wifi_btn, lv_color_hex(0x285C92), 0);
  lv_obj_set_style_radius(wifi_btn, 12, 0);
  lv_obj_add_event_cb(wifi_btn, toggleWifiEventCb, LV_EVENT_CLICKED, nullptr);
  g_lbl_config_wifi_btn = lv_label_create(wifi_btn);
  lv_label_set_text(g_lbl_config_wifi_btn, "Wi-Fi AP");
  lv_obj_center(g_lbl_config_wifi_btn);

  lv_obj_t *lora_btn = lv_button_create(net_card);
  lv_obj_set_size(lora_btn, half_btn_w, 34);
  lv_obj_align(lora_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(lora_btn, lv_color_hex(0x8B3E2A), 0);
  lv_obj_set_style_radius(lora_btn, 12, 0);
  lv_obj_add_event_cb(lora_btn, toggleLoraEventCb, LV_EVENT_CLICKED, nullptr);
  g_lbl_config_lora_btn = lv_label_create(lora_btn);
  lv_label_set_text(g_lbl_config_lora_btn, "LoRa");
  lv_obj_center(g_lbl_config_lora_btn);

  y += 178;

  lv_obj_t *blackbox_card = createLocalMenuCard(content, card_w, 174, y, 0xFFD86B, "Armazenamento");
  g_lbl_config_blackbox = lv_label_create(blackbox_card);
  lv_label_set_text(g_lbl_config_blackbox, "");
  lv_label_set_long_mode(g_lbl_config_blackbox, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_blackbox, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_config_blackbox, lv_color_hex(0xFFF4CC), 0);
  lv_obj_align(g_lbl_config_blackbox, LV_ALIGN_TOP_LEFT, 0, 24);

  g_lbl_config_note = lv_label_create(blackbox_card);
  lv_label_set_text(g_lbl_config_note, "Logs no SD");
  lv_label_set_long_mode(g_lbl_config_note, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_note, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_config_note, lv_color_hex(0xFFD86B), 0);
  lv_obj_align(g_lbl_config_note, LV_ALIGN_TOP_LEFT, 0, 82);

  lv_obj_t *logger_btn = lv_button_create(blackbox_card);
  lv_obj_set_size(logger_btn, half_btn_w, 34);
  lv_obj_align(logger_btn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(logger_btn, lv_color_hex(0x2A2232), 0);
  lv_obj_set_style_radius(logger_btn, 12, 0);
  lv_obj_add_event_cb(logger_btn, toggleBlackboxEventCb, LV_EVENT_CLICKED, nullptr);
  g_lbl_config_blackbox_btn = lv_label_create(logger_btn);
  lv_label_set_text(g_lbl_config_blackbox_btn, "Logger");
  lv_obj_center(g_lbl_config_blackbox_btn);

  lv_obj_t *format_btn = lv_button_create(blackbox_card);
  lv_obj_set_size(format_btn, half_btn_w, 34);
  lv_obj_align(format_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(format_btn, lv_color_hex(0x4B2721), 0);
  lv_obj_set_style_radius(format_btn, 12, 0);
  lv_obj_add_event_cb(format_btn, purgeSdLogsEventCb, LV_EVENT_CLICKED, nullptr);
  g_lbl_config_format_btn = lv_label_create(format_btn);
  lv_label_set_text(g_lbl_config_format_btn, "Limpar SD");
  lv_obj_center(g_lbl_config_format_btn);

  y += 184;

  lv_obj_t *sensor_card = createLocalMenuCard(content, card_w, 108, y, 0x4E92FF, "Sensores");
  g_lbl_config_sensors = lv_label_create(sensor_card);
  lv_label_set_text(g_lbl_config_sensors, "");
  lv_label_set_long_mode(g_lbl_config_sensors, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_sensors, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_config_sensors, lv_color_hex(0xE7EDF5), 0);
  lv_obj_align(g_lbl_config_sensors, LV_ALIGN_TOP_LEFT, 0, 24);

  g_lbl_config_scan_raw = lv_label_create(sensor_card);
  lv_label_set_text(g_lbl_config_scan_raw, "");
  lv_label_set_long_mode(g_lbl_config_scan_raw, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_scan_raw, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_config_scan_raw, lv_color_hex(0x7EC8FF), 0);
  lv_obj_align(g_lbl_config_scan_raw, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  lv_obj_t *refresh_btn = lv_button_create(g_screen_config);
  lv_obj_set_size(refresh_btn, footer_btn_w, footer_h);
  lv_obj_align(refresh_btn, LV_ALIGN_BOTTOM_LEFT, 12, -10);
  lv_obj_set_style_bg_color(refresh_btn, lv_color_hex(0x1A2638), 0);
  lv_obj_set_style_radius(refresh_btn, 14, 0);
  lv_obj_add_event_cb(refresh_btn, refreshSensorsEventCb, LV_EVENT_CLICKED, nullptr);
  g_lbl_config_refresh = lv_label_create(refresh_btn);
  lv_label_set_text(g_lbl_config_refresh, "Atualizar");
  lv_obj_center(g_lbl_config_refresh);

  lv_obj_t *back_btn = lv_button_create(g_screen_config);
  lv_obj_set_size(back_btn, footer_btn_w, footer_h);
  lv_obj_align(back_btn, LV_ALIGN_BOTTOM_RIGHT, -12, -10);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x1A2638), 0);
  lv_obj_set_style_radius(back_btn, 14, 0);
  lv_obj_add_event_cb(back_btn, backToMenuEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "Voltar");
  lv_obj_center(back_label);
}

static void createMeteoScreen()
{
  const bool landscape = isLandscapeUI();
  const int32_t top_y = landscape ? 64 : 70;
  const lv_coord_t hero_h = landscape ? 98 : 106;
  const lv_coord_t panel_y = top_y + hero_h + 8;
  const lv_coord_t panel_h = uiHeight() - panel_y - 60;
  const lv_coord_t panel_w = uiWidth() - 16;

  g_screen_meteo = lv_obj_create(nullptr);
  styleScreen(g_screen_meteo);
  lv_obj_set_style_bg_color(g_screen_meteo, lv_color_hex(0x07111A), 0);
  lv_obj_set_style_bg_grad_color(g_screen_meteo, lv_color_hex(0x15395A), 0);
  lv_obj_set_style_bg_grad_dir(g_screen_meteo, LV_GRAD_DIR_VER, 0);
  createHeader(g_screen_meteo, "METEO", "tempo e clima");

  g_meteo_hero = lv_obj_create(g_screen_meteo);
  lv_obj_clear_flag(g_meteo_hero, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(g_meteo_hero, uiWidth() - 16, hero_h);
  lv_obj_align(g_meteo_hero, LV_ALIGN_TOP_MID, 0, top_y);
  lv_obj_set_style_bg_color(g_meteo_hero, lv_color_hex(0x174C77), 0);
  lv_obj_set_style_bg_grad_color(g_meteo_hero, lv_color_hex(0x03131E), 0);
  lv_obj_set_style_bg_grad_dir(g_meteo_hero, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_border_color(g_meteo_hero, lv_color_hex(0x6FD9FF), 0);
  lv_obj_set_style_border_width(g_meteo_hero, 2, 0);
  lv_obj_set_style_radius(g_meteo_hero, 18, 0);
  lv_obj_set_style_pad_all(g_meteo_hero, 14, 0);

g_lbl_meteo_theme = lv_label_create(g_meteo_hero);
lv_label_set_text(g_lbl_meteo_theme, "AGUARDANDO");
  lv_label_set_long_mode(g_lbl_meteo_theme, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_meteo_theme, uiWidth() - 48);
  lv_obj_set_style_text_color(g_lbl_meteo_theme, lv_color_hex(0xF2FAFF), 0);
  lv_obj_align(g_lbl_meteo_theme, LV_ALIGN_TOP_LEFT, 0, 0);

g_lbl_meteo_status = lv_label_create(g_meteo_hero);
lv_label_set_text(g_lbl_meteo_status, "BME688 preparando a inferencia local do clima.");
  lv_label_set_long_mode(g_lbl_meteo_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_meteo_status, uiWidth() - 48);
  lv_obj_set_style_text_color(g_lbl_meteo_status, lv_color_hex(0xEAF7FF), 0);
  lv_obj_align(g_lbl_meteo_status, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  g_meteo_panel = lv_obj_create(g_screen_meteo);
  lv_obj_set_size(g_meteo_panel, panel_w, panel_h);
  lv_obj_align(g_meteo_panel, LV_ALIGN_TOP_MID, 0, panel_y);
  lv_obj_set_style_bg_color(g_meteo_panel, lv_color_hex(0x09131C), 0);
  lv_obj_set_style_border_color(g_meteo_panel, lv_color_hex(0x3CCB9A), 0);
  lv_obj_set_style_border_width(g_meteo_panel, 2, 0);
  lv_obj_set_style_radius(g_meteo_panel, 18, 0);
  lv_obj_set_style_pad_all(g_meteo_panel, 14, 0);

  g_lbl_meteo_cards = lv_label_create(g_meteo_panel);
  lv_label_set_text(g_lbl_meteo_cards, "CLIMA");
  lv_obj_add_flag(g_lbl_meteo_cards, LV_OBJ_FLAG_HIDDEN);

  const lv_coord_t gap = 8;
  const lv_coord_t card_w = (panel_w - 28 - gap) / 2;
  const lv_coord_t card_h = landscape ? 76 : 92;
  const lv_coord_t row2_y = card_h + gap;

  createDataCard(g_meteo_panel, card_w, card_h, LV_ALIGN_TOP_LEFT, 0, 0, 0x7BD7FF, "ALTITUDE", "--- m", &g_lbl_meteo_altitude);
  createDataCard(g_meteo_panel, card_w, card_h, LV_ALIGN_TOP_RIGHT, 0, 0, 0xF7C768, "PRESSAO", "---- hPa", &g_lbl_meteo_pressure);
  createDataCard(g_meteo_panel, card_w, card_h, LV_ALIGN_TOP_LEFT, 0, row2_y, 0xFF8A65, "TEMP", "--.- C", &g_lbl_meteo_temperature);
  createDataCard(g_meteo_panel, card_w, card_h, LV_ALIGN_TOP_RIGHT, 0, row2_y, 0x6BE4C8, "UMIDADE", "--.- %", &g_lbl_meteo_humidity);

  createBackButton(g_screen_meteo, LV_ALIGN_BOTTOM_MID, 0, -10, uiWidth() - 32);
}

static void createCommsScreen()
{
  const lv_coord_t top_y = isLandscapeUI() ? 58 : 64;
  const lv_coord_t footer_h = 44;
  const lv_coord_t body_bottom_reserved = footer_h + 24;
  const lv_coord_t card_w = uiWidth() - 24;
  const lv_coord_t half_btn_w = (card_w - 12) / 2;

  g_screen_gps = lv_obj_create(nullptr);
  styleScreen(g_screen_gps);
  lv_obj_set_style_bg_color(g_screen_gps, lv_color_hex(0x090C12), 0);
  lv_obj_set_style_bg_grad_color(g_screen_gps, lv_color_hex(0x1D1111), 0);
  lv_obj_set_style_bg_grad_dir(g_screen_gps, LV_GRAD_DIR_VER, 0);
  createHeader(g_screen_gps, "COMMS", "wifi, lora e gps");

  lv_obj_t *content = createLocalScrollBody(g_screen_gps, top_y, body_bottom_reserved);
  lv_coord_t y = 0;

  lv_obj_t *hero = createLocalMenuCard(content, card_w, 88, y, 0xFFB347, "Estado geral");
  g_lbl_comms_status = lv_label_create(hero);
  lv_label_set_text(g_lbl_comms_status, "AP ON | LAN OFF | 0 cli");
  lv_label_set_long_mode(g_lbl_comms_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_status, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_comms_status, lv_color_hex(0xFFF2DA), 0);
  lv_obj_align(g_lbl_comms_status, LV_ALIGN_TOP_LEFT, 0, 24);

  g_lbl_comms_mode = lv_label_create(hero);
  lv_label_set_text(g_lbl_comms_mode, "Scan: 0 redes");
  lv_label_set_long_mode(g_lbl_comms_mode, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_mode, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_comms_mode, lv_color_hex(0xFFD699), 0);
  lv_obj_align(g_lbl_comms_mode, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  y += 98;

  lv_obj_t *wifi_card = createLocalMenuCard(content, card_w, 102, y, 0xFFB347, "Wi-Fi / Web");
  g_lbl_comms_wifi = lv_label_create(wifi_card);
  lv_label_set_text(g_lbl_comms_wifi, "");
  lv_label_set_long_mode(g_lbl_comms_wifi, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_wifi, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_comms_wifi, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(g_lbl_comms_wifi, LV_ALIGN_TOP_LEFT, 0, 24);

  y += 112;

  lv_obj_t *lan_card = createLocalMenuCard(content, card_w, 84, y, 0x63C7FF, "LAN / DHCP");
  g_lbl_comms_ble = lv_label_create(lan_card);
  lv_label_set_text(g_lbl_comms_ble, "");
  lv_label_set_long_mode(g_lbl_comms_ble, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_ble, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_comms_ble, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(g_lbl_comms_ble, LV_ALIGN_TOP_LEFT, 0, 24);

  y += 94;

  lv_obj_t *gps_card = createLocalMenuCard(content, card_w, 84, y, 0x53C2A3, "GPS");
  g_lbl_comms_gps = lv_label_create(gps_card);
  lv_label_set_text(g_lbl_comms_gps, "");
  lv_label_set_long_mode(g_lbl_comms_gps, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_gps, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_comms_gps, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(g_lbl_comms_gps, LV_ALIGN_TOP_LEFT, 0, 24);

  y += 94;

  lv_obj_t *lora_card = createLocalMenuCard(content, card_w, 84, y, 0xFF7A59, "LoRa");
  g_lbl_comms_lora = lv_label_create(lora_card);
  lv_label_set_text(g_lbl_comms_lora, "");
  lv_label_set_long_mode(g_lbl_comms_lora, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_lora, card_w - 24);
  lv_obj_set_style_text_color(g_lbl_comms_lora, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(g_lbl_comms_lora, LV_ALIGN_TOP_LEFT, 0, 24);

  y += 94;

  lv_obj_t *actions_card = createLocalMenuCard(content, card_w, 126, y, 0x7B6CFF, "Acoes");
  createActionButton(actions_card, half_btn_w, 38, LV_ALIGN_TOP_LEFT, 0, 28, 0x285C92, "Wi-Fi AP", toggleWifiEventCb, &g_lbl_comms_wifi_btn);
  createActionButton(actions_card, half_btn_w, 38, LV_ALIGN_TOP_RIGHT, 0, 28, 0x2A5E4C, "Scan Wi-Fi", rescanCommsEventCb, &g_lbl_comms_rescan_btn);
  createActionButton(actions_card, half_btn_w, 38, LV_ALIGN_BOTTOM_LEFT, 0, 0, 0x356A9A, "LAN salva", toggleWifiStaEventCb, &g_lbl_comms_sta_btn);
  createActionButton(actions_card, half_btn_w, 38, LV_ALIGN_BOTTOM_RIGHT, 0, 0, 0x8B3E2A, "Menu LoRa", openLoraScreenEventCb, &g_lbl_comms_lora_btn);

  lv_obj_t *back_btn = lv_button_create(g_screen_gps);
  lv_obj_set_size(back_btn, uiWidth() - 32, footer_h);
  lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x2B4C73), 0);
  lv_obj_set_style_radius(back_btn, 14, 0);
  lv_obj_add_event_cb(back_btn, backToMenuEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "Voltar");
  lv_obj_center(back_label);
}

static void createLoraScreen()
{
  const bool landscape = isLandscapeUI();
  const int32_t top_y = landscape ? 64 : 70;
  const lv_coord_t screen_w = uiWidth();
  const lv_coord_t card_w = screen_w - 16;
  const lv_coord_t hero_h = landscape ? 82 : 88;
  const lv_coord_t info_h = landscape ? 86 : 94;
  const lv_coord_t test_h = landscape ? 74 : 82;
  const lv_coord_t second_y = top_y + hero_h + 8;
  const lv_coord_t third_y = second_y + info_h + 8;
  const lv_coord_t action_y = third_y + test_h + 12;
  const lv_coord_t action_w = (screen_w - 28) / 2;

  g_screen_lora = lv_obj_create(nullptr);
  styleScreen(g_screen_lora);
  lv_obj_set_style_bg_color(g_screen_lora, lv_color_hex(0x090B10), 0);
  lv_obj_set_style_bg_grad_color(g_screen_lora, lv_color_hex(0x25130D), 0);
  lv_obj_set_style_bg_grad_dir(g_screen_lora, LV_GRAD_DIR_VER, 0);
  createHeader(g_screen_lora, "LORA", "radio e telemetria");

  lv_obj_t *hero = lv_obj_create(g_screen_lora);
  lv_obj_clear_flag(hero, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(hero, card_w, hero_h);
  lv_obj_align(hero, LV_ALIGN_TOP_MID, 0, top_y);
  lv_obj_set_style_bg_color(hero, lv_color_hex(0x141821), 0);
  lv_obj_set_style_border_color(hero, lv_color_hex(0xFF7A59), 0);
  lv_obj_set_style_border_width(hero, 2, 0);
  lv_obj_set_style_radius(hero, 18, 0);
  lv_obj_set_style_pad_all(hero, 14, 0);

  g_lbl_lora_status = lv_label_create(hero);
  lv_label_set_text(g_lbl_lora_status, "LoRa UART OFF\nPinos 17/18/6/7/8 prontos");
  lv_label_set_long_mode(g_lbl_lora_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_lora_status, card_w - 28);
  lv_obj_set_style_text_color(g_lbl_lora_status, lv_color_hex(0xFFF2DA), 0);
  lv_obj_align(g_lbl_lora_status, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *radio_card = lv_obj_create(g_screen_lora);
  lv_obj_clear_flag(radio_card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(radio_card, card_w, info_h);
  lv_obj_align(radio_card, LV_ALIGN_TOP_MID, 0, second_y);
  lv_obj_set_style_bg_color(radio_card, lv_color_hex(0x0F1218), 0);
  lv_obj_set_style_border_color(radio_card, lv_color_hex(0xD48E5C), 0);
  lv_obj_set_style_border_width(radio_card, 2, 0);
  lv_obj_set_style_radius(radio_card, 18, 0);
  lv_obj_set_style_pad_all(radio_card, 14, 0);

  g_lbl_lora_radio = lv_label_create(radio_card);
  lv_label_set_text(g_lbl_lora_radio, "Radio: E220 UART\nTX=17 RX=18 | M0=7 M1=8\nAUX=6 | modo inicial = normal");
  lv_label_set_long_mode(g_lbl_lora_radio, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_lora_radio, card_w - 28);
  lv_obj_set_style_text_color(g_lbl_lora_radio, lv_color_hex(0xE7EDF5), 0);
  lv_obj_align(g_lbl_lora_radio, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *test_card = lv_obj_create(g_screen_lora);
  lv_obj_clear_flag(test_card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(test_card, card_w, test_h);
  lv_obj_align(test_card, LV_ALIGN_TOP_MID, 0, third_y);
  lv_obj_set_style_bg_color(test_card, lv_color_hex(0x0B1118), 0);
  lv_obj_set_style_border_color(test_card, lv_color_hex(0x53C2A3), 0);
  lv_obj_set_style_border_width(test_card, 2, 0);
  lv_obj_set_style_radius(test_card, 18, 0);
  lv_obj_set_style_pad_all(test_card, 14, 0);

  g_lbl_lora_test = lv_label_create(test_card);
  lv_label_set_text(g_lbl_lora_test, "Proximo passo:\n1. M0=0 M1=0\n2. ligar dois modulos\n3. validar TX/RX via UART");
  lv_label_set_long_mode(g_lbl_lora_test, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_lora_test, card_w - 28);
  lv_obj_set_style_text_color(g_lbl_lora_test, lv_color_hex(0xD5F3EA), 0);
  lv_obj_align(g_lbl_lora_test, LV_ALIGN_TOP_LEFT, 0, 0);

  createActionButton(g_screen_lora, action_w, 42, LV_ALIGN_TOP_LEFT, 8, action_y, 0x8B3E2A, "Ligar LoRa", toggleLoraEventCb, &g_lbl_lora_toggle_btn);

  lv_obj_t *back_btn = lv_button_create(g_screen_lora);
  lv_obj_set_size(back_btn, action_w, 42);
  lv_obj_align(back_btn, LV_ALIGN_TOP_RIGHT, -8, action_y);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x2B4C73), 0);
  lv_obj_set_style_radius(back_btn, 14, 0);
  lv_obj_add_event_cb(back_btn, backToCommsEventCb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "Voltar COMMS");
  lv_obj_center(back_label);
}

static void createMainScreen()
{
  const bool landscape = isLandscapeUI();
  const int32_t button_grid_y = landscape ? 58 : 58;
  const int32_t button_grid_h = landscape ? 170 : 300;

  g_screen_main = lv_obj_create(nullptr);
  styleScreen(g_screen_main);
  lv_obj_set_style_bg_color(g_screen_main, lv_color_hex(0x05080D), 0);
  lv_obj_set_style_bg_grad_color(g_screen_main, lv_color_hex(0x0E1724), 0);
  lv_obj_set_style_bg_grad_dir(g_screen_main, LV_GRAD_DIR_VER, 0);
  createHeader(g_screen_main, "StratosBrain S3", "");

  lv_obj_t *button_grid = lv_obj_create(g_screen_main);
  lv_obj_clear_flag(button_grid, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(button_grid, uiWidth() - 16, button_grid_h);
  lv_obj_align(button_grid, LV_ALIGN_TOP_MID, 0, button_grid_y);
  lv_obj_set_style_bg_color(button_grid, lv_color_hex(0x081018), 0);
  lv_obj_set_style_radius(button_grid, 16, 0);
  lv_obj_set_style_pad_all(button_grid, 10, 0);
  lv_obj_set_style_border_width(button_grid, 0, 0);

  if (landscape) {
    lv_obj_t *btn_efis = createNavButton(button_grid, "PLANE", "voo e instrumentos", 0x164B7A, SCREEN_EFIS, 122, 134);
    lv_obj_align(btn_efis, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *btn_meteo = createNavButton(button_grid, "METEO", "tempo", 0x226B52, SCREEN_METEO, 126, 60);
    lv_obj_align(btn_meteo, LV_ALIGN_TOP_RIGHT, 0, 0);

    lv_obj_t *btn_gps = createNavButton(button_grid, "COMMS", "radio", 0x6A4A16, SCREEN_GPS, 126, 60);
    lv_obj_align(btn_gps, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *btn_config = createNavButton(button_grid, "CONFIG", "ajustes", 0x5A245F, SCREEN_CONFIG, 126, 60);
    lv_obj_align(btn_config, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  } else {
    const lv_coord_t grid_w = uiWidth() - 16;
    const lv_coord_t card_w = (grid_w - 28) / 2;
    const lv_coord_t card_h = (button_grid_h - 28) / 2;

    lv_obj_t *btn_efis = createNavButton(button_grid, "PLANE", "", 0x164B7A, SCREEN_EFIS, card_w, card_h);
    lv_obj_align(btn_efis, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *btn_meteo = createNavButton(button_grid, "METEO", "", 0x226B52, SCREEN_METEO, card_w, card_h);
    lv_obj_align(btn_meteo, LV_ALIGN_TOP_RIGHT, 0, 0);

    lv_obj_t *btn_gps = createNavButton(button_grid, "COMMS", "", 0x6A4A16, SCREEN_GPS, card_w, card_h);
    lv_obj_align(btn_gps, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    lv_obj_t *btn_config = createNavButton(button_grid, "CONFIG", "", 0x5A245F, SCREEN_CONFIG, card_w, card_h);
    lv_obj_align(btn_config, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  }

  lv_obj_t *overview = lv_obj_create(g_screen_main);
  lv_obj_clear_flag(overview, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(overview, uiWidth() - 16, landscape ? 42 : 48);
  lv_obj_align(overview, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_bg_color(overview, lv_color_hex(0x0E141C), 0);
  lv_obj_set_style_border_color(overview, lv_color_hex(0x243D5C), 0);
  lv_obj_set_style_border_width(overview, 2, 0);
  lv_obj_set_style_radius(overview, 16, 0);
  lv_obj_set_style_pad_all(overview, 8, 0);

  g_lbl_home_hint = lv_label_create(overview);
  lv_label_set_text(g_lbl_home_hint, "SD -- | BME -- | WEB off");
  lv_obj_set_width(g_lbl_home_hint, uiWidth() - 40);
  lv_label_set_long_mode(g_lbl_home_hint, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(g_lbl_home_hint, lv_color_hex(0xD7DFE8), 0);
  lv_obj_align(g_lbl_home_hint, LV_ALIGN_TOP_LEFT, 0, 0);
}

static void createUI(AppScreenId initial_screen_id)
{
  refreshSensorCaches();
  createEfisScreen();
  createMeteoScreen();
  createCommsScreen();
  createLoraScreen();
  createConfigScreen();

  createMainScreen();
  g_current_screen_id = initial_screen_id;
  lv_screen_load(screenFromId(initial_screen_id));
}

static void refreshTouchUI()
{
  if (g_lbl_touch) {
    lv_label_set_text_fmt(
        g_lbl_touch,
        "Touch: %s | X:%u Y:%u | Toques:%lu",
        g_touch_pressed ? "PRESSIONADO" : "solto",
        static_cast<unsigned>(g_touch_x),
        static_cast<unsigned>(g_touch_y),
        static_cast<unsigned long>(g_touch_press_count));
  }

  if (!g_touch_dot) {
    return;
  }

  if (lv_screen_active() == g_screen_main && g_touch_pressed) {
    const int32_t pos_x = static_cast<int32_t>(g_touch_x) - 7;
    const int32_t pos_y = static_cast<int32_t>(g_touch_y) - 7;
    lv_obj_set_pos(g_touch_dot, pos_x, pos_y);
    lv_obj_clear_flag(g_touch_dot, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(g_touch_dot, LV_OBJ_FLAG_HIDDEN);
  }
}

static void refreshEfisUI()
{
  if (g_current_screen_id != SCREEN_EFIS) {
    return;
  }

  const ImuState imu = copyImuState();
  const GpsState gps = copyGpsState();
  const bool bmp_ok = scanHasAddress(g_sensor_raw_scan_cache, 0x47) || scanHasAddress(g_sensor_raw_scan_cache, 0x46);
  const bool bmm_ok = scanHasAddress(g_sensor_raw_scan_cache, 0x14);
  ImuState display_imu = imu;
  float pitch_trim_deg = 0.0f;
  float roll_trim_deg = 0.0f;
  getImuTrim(&pitch_trim_deg, &roll_trim_deg);

  if (imu.has_solution) {
    if (!g_efis_display_seeded) {
      g_efis_display_pitch_deg = imu.pitch_deg;
      g_efis_display_roll_deg = imu.roll_deg;
      g_efis_display_seeded = true;
    } else {
      g_efis_display_pitch_deg += (imu.pitch_deg - g_efis_display_pitch_deg) * EFIS_DISPLAY_BLEND;
      g_efis_display_roll_deg += (imu.roll_deg - g_efis_display_roll_deg) * EFIS_DISPLAY_BLEND;
    }

    display_imu.pitch_deg = g_efis_display_pitch_deg;
    display_imu.roll_deg = g_efis_display_roll_deg;
  } else {
    g_efis_display_seeded = false;
  }

  if (g_lbl_efis_pitch) {
    if (display_imu.has_solution) {
      lv_label_set_text_fmt(g_lbl_efis_pitch, "Pitch %+.1f", display_imu.pitch_deg);
    } else {
      lv_label_set_text(g_lbl_efis_pitch, "Pitch --.-");
    }
  }

  if (g_lbl_efis_roll) {
    if (display_imu.has_solution) {
      lv_label_set_text_fmt(g_lbl_efis_roll, "Roll %+.1f", display_imu.roll_deg);
    } else {
      lv_label_set_text(g_lbl_efis_roll, "Roll --.-");
    }
  }

  if (g_lbl_plane_altitude) {
    lv_label_set_text_fmt(g_lbl_plane_altitude, "%.1f m", g_plane_smooth_alt);
  }

  if (g_lbl_plane_vario) {
    lv_label_set_text_fmt(g_lbl_plane_vario, "%+.1f m/s", g_plane_smooth_vs);
  }

  if (g_lbl_plane_heading) {
    lv_label_set_text(g_lbl_plane_heading, "--- dg");
  }

  if (g_lbl_plane_speed) {
    lv_label_set_text_fmt(g_lbl_plane_speed, "%.1f km/h", g_plane_smooth_spd);
  }

  if (g_lbl_efis_horizon_alt) {
    lv_label_set_text_fmt(g_lbl_efis_horizon_alt, "ALT %.1f m", g_plane_smooth_alt);
  }

  if (g_lbl_efis_horizon_speed) {
    lv_label_set_text_fmt(g_lbl_efis_horizon_speed, "SPD %.1f km/h", g_plane_smooth_spd);
  }

  if (g_efis_horizon && g_efis_canvas_buf && !g_runtime_services_light) {
    drawEfisHorizon(display_imu);
  }


  if (g_lbl_efis_status) {
    lv_label_set_text_fmt(
        g_lbl_efis_status,
        "GPS SPD %.1f km/h | BMP %s | MAG %s | Wi-Fi %s",
        g_plane_smooth_spd,
        bmp_ok ? "OK" : "--",
        bmm_ok ? "OK" : "--",
        g_wifi_ap_started ? "ON" : "OFF");
  }

  if (g_lbl_efis_calibration) {
    lv_label_set_text(g_lbl_efis_calibration, "Setar nivel");
  }
}

static void refreshConfigUI()
{
  const bool screen_home = g_current_screen_id == SCREEN_HOME;
  const bool screen_meteo = g_current_screen_id == SCREEN_METEO;
  const bool screen_comms = g_current_screen_id == SCREEN_GPS;
  const bool screen_lora = g_current_screen_id == SCREEN_LORA;
  const bool screen_config = g_current_screen_id == SCREEN_CONFIG;
  const BlackboxState blackbox = copyBlackboxState();
  char ip_text[20];
  char ap_ip_text[20];
  char sta_ip_text[20];
  formatWifiVisibleIp(ip_text, sizeof(ip_text));
  formatWifiApIp(ap_ip_text, sizeof(ap_ip_text));
  formatWifiStaIp(sta_ip_text, sizeof(sta_ip_text));

  if (g_sd_purge_armed && (millis() - g_sd_purge_arm_ms) > SD_PURGE_CONFIRM_MS) {
    g_sd_purge_armed = false;
  }

  if (screen_config && g_lbl_config_orientation) {
    lv_label_set_text_fmt(g_lbl_config_orientation, "Modo: %s", orientationModeLabel(g_orientation_mode));
  }

  if (screen_config && g_lbl_config_rotation) {
    lv_label_set_text_fmt(g_lbl_config_rotation, "Ativa: %s", rotationLabel(g_display_rotation));
  }

  if (screen_config && g_lbl_config_brightness) {
    lv_label_set_text_fmt(g_lbl_config_brightness, "Brilho: %u / 255", static_cast<unsigned>(g_display_brightness));
  }

  if (screen_config && g_lbl_config_cdc) {
    const GpsState gps = copyGpsState();
    lv_label_set_text_fmt(
        g_lbl_config_cdc,
        "USB/JTAG 115200\nAP %s | LAN %s\nLoRa %s | GPS %s",
        g_wifi_ap_requested ? "ON" : "OFF",
        g_wifi_sta_connected ? "OK" : (g_wifi_sta_requested ? "..." : "--"),
        g_lora_enabled ? "ON" : "OFF",
        gps.receiving ? "RX" : "--");
  }

  if (screen_config && g_lbl_config_net) {
    char net_text[224];
    snprintf(
        net_text,
        sizeof(net_text),
        "AP %s\nIP AP %s\nLAN %s | %s\nSSID %s\nCli %u",
        wifiStateLabel(),
        ap_ip_text,
        g_wifi_sta_connected ? "OK" : (g_wifi_sta_requested ? "..." : "--"),
        g_wifi_sta_connected ? sta_ip_text : "--",
        g_wifi_sta_ssid[0] ? g_wifi_sta_ssid : "-",
        static_cast<unsigned>(wifiClientCount()));
    lv_label_set_text(g_lbl_config_net, net_text);
  }

  if (screen_config && g_lbl_config_sensors) {
    lv_label_set_text(g_lbl_config_sensors, g_sensor_compact_cache.c_str());
  }

  if (screen_config && g_lbl_config_note) {
    char note_text[128];
    if (g_sd_purge_armed) {
      const uint32_t remaining_ms = SD_PURGE_CONFIRM_MS - (millis() - g_sd_purge_arm_ms);
      snprintf(
          note_text,
          sizeof(note_text),
          "Toque de novo em\nLimpar SD (%lus)",
          static_cast<unsigned long>((remaining_ms + 999U) / 1000U));
    } else {
      snprintf(
          note_text,
          sizeof(note_text),
          "COMMS: Wi-Fi e LoRa\nBOOT: Home");
    }
    lv_label_set_text(g_lbl_config_note, note_text);
  }

  if (screen_config && g_lbl_config_blackbox) {
    char size_text[24];
    char status_text[192];

    formatStorageSize(blackbox.card_size_bytes, size_text, sizeof(size_text));
    snprintf(
        status_text,
        sizeof(status_text),
        "SD %s | %s\nReg %lu | %s\n%s%s",
        blackbox.mounted ? "OK" : "--",
        size_text,
        static_cast<unsigned long>(blackbox.records_written),
        blackbox.logging_enabled ? (blackbox.file_open ? "gravando" : "armado") : "pausado",
        blackbox.file_path[0] ? baseFileName(blackbox.file_path) : blackbox.note,
        g_runtime_services_light ? "\nWi-Fi pausando logger" : "");
    lv_label_set_text(g_lbl_config_blackbox, status_text);
  }

  if (screen_config && g_lbl_config_scan_raw) {
    lv_label_set_text_fmt(g_lbl_config_scan_raw, "I2C %s", g_sensor_scan_compact_cache.c_str());
  }

  if (screen_config && g_lbl_config_blackbox_btn) {
    lv_label_set_text(
        g_lbl_config_blackbox_btn,
        blackbox.logging_enabled ? "Pausar" : "Ligar");
  }

  if (screen_config && g_lbl_config_format_btn) {
    lv_label_set_text(g_lbl_config_format_btn, "Limpar SD");
  }

  if (screen_config && g_lbl_config_wifi_btn) {
    lv_label_set_text(g_lbl_config_wifi_btn, g_wifi_ap_requested ? "Desligar AP" : "Ligar AP");
  }

  if (screen_config && g_lbl_config_lora_btn) {
    lv_label_set_text(g_lbl_config_lora_btn, g_lora_enabled ? "Desligar LoRa" : "Ligar LoRa");
  }

  if (screen_home && g_lbl_home_hint) {
    const Bme688State bme_home = copyBme688State();
    char home_text[160];
    snprintf(
        home_text,
        sizeof(home_text),
        "SD %s | BME %s\nWiFi %s @ %s | LoRa %s",
        blackbox.mounted ? "OK" : "--",
        bme_home.connected ? "OK" : "--",
        wifiModeLabel(),
        ip_text,
        g_lora_enabled ? "on" : "off");
    lv_label_set_text(g_lbl_home_hint, home_text);
  }

  if (screen_meteo && g_lbl_meteo_theme) {
    const Bme688State bme_weather = copyBme688State();
    char meteo_theme[48];
    char meteo_summary[160];
    const uint8_t weather_mode = classifyMeteoTheme(
        bme_weather,
        meteo_theme,
        sizeof(meteo_theme),
        meteo_summary,
        sizeof(meteo_summary));

    switch (weather_mode) {
      case 0:
        lv_label_set_text(g_lbl_meteo_theme, meteo_theme);
        if (g_lbl_meteo_status) {
          lv_label_set_text(g_lbl_meteo_status, meteo_summary);
        }
        if (g_screen_meteo) {
          lv_obj_set_style_bg_color(g_screen_meteo, lv_color_hex(0x081B2D), 0);
          lv_obj_set_style_bg_grad_color(g_screen_meteo, lv_color_hex(0x1D5B89), 0);
        }
        if (g_meteo_hero) {
          lv_obj_set_style_bg_color(g_meteo_hero, lv_color_hex(0x2377B5), 0);
          lv_obj_set_style_bg_grad_color(g_meteo_hero, lv_color_hex(0x0F2D45), 0);
        }
        if (g_meteo_panel) {
          lv_obj_set_style_border_color(g_meteo_panel, lv_color_hex(0x6FD9FF), 0);
        }
        break;
      case 1:
        lv_label_set_text(g_lbl_meteo_theme, meteo_theme);
        if (g_lbl_meteo_status) {
          lv_label_set_text(g_lbl_meteo_status, meteo_summary);
        }
        if (g_screen_meteo) {
          lv_obj_set_style_bg_color(g_screen_meteo, lv_color_hex(0x0A121A), 0);
          lv_obj_set_style_bg_grad_color(g_screen_meteo, lv_color_hex(0x324252), 0);
        }
        if (g_meteo_hero) {
          lv_obj_set_style_bg_color(g_meteo_hero, lv_color_hex(0x4A5A67), 0);
          lv_obj_set_style_bg_grad_color(g_meteo_hero, lv_color_hex(0x1B252D), 0);
        }
        if (g_meteo_panel) {
          lv_obj_set_style_border_color(g_meteo_panel, lv_color_hex(0xA7C2D6), 0);
        }
        break;
      case 2:
        lv_label_set_text(g_lbl_meteo_theme, meteo_theme);
        if (g_lbl_meteo_status) {
          lv_label_set_text(g_lbl_meteo_status, meteo_summary);
        }
        if (g_screen_meteo) {
          lv_obj_set_style_bg_color(g_screen_meteo, lv_color_hex(0x090C12), 0);
          lv_obj_set_style_bg_grad_color(g_screen_meteo, lv_color_hex(0x2F213B), 0);
        }
        if (g_meteo_hero) {
          lv_obj_set_style_bg_color(g_meteo_hero, lv_color_hex(0x3A2B4D), 0);
          lv_obj_set_style_bg_grad_color(g_meteo_hero, lv_color_hex(0x151B28), 0);
        }
        if (g_meteo_panel) {
          lv_obj_set_style_border_color(g_meteo_panel, lv_color_hex(0xF8CA5B), 0);
        }
        break;
      default:
        lv_label_set_text(g_lbl_meteo_theme, meteo_theme);
        if (g_lbl_meteo_status) {
          lv_label_set_text(g_lbl_meteo_status, meteo_summary);
        }
        if (g_screen_meteo) {
          lv_obj_set_style_bg_color(g_screen_meteo, lv_color_hex(0x08111B), 0);
          lv_obj_set_style_bg_grad_color(g_screen_meteo, lv_color_hex(0x163246), 0);
        }
        if (g_meteo_hero) {
          lv_obj_set_style_bg_color(g_meteo_hero, lv_color_hex(0x294B63), 0);
          lv_obj_set_style_bg_grad_color(g_meteo_hero, lv_color_hex(0x0A131C), 0);
        }
        if (g_meteo_panel) {
          lv_obj_set_style_border_color(g_meteo_panel, lv_color_hex(0x6FD9FF), 0);
        }
        break;
    }
  }

  if (screen_meteo) {
    const Bme688State bme = copyBme688State();
    if (g_lbl_meteo_altitude) {
      if (bme.has_data) {
        lv_label_set_text_fmt(g_lbl_meteo_altitude, "%.1f m", bme.altitude_m);
      } else {
        lv_label_set_text(g_lbl_meteo_altitude, "--- m");
      }
    }
    if (g_lbl_meteo_pressure) {
      if (bme.has_data) {
        lv_label_set_text_fmt(g_lbl_meteo_pressure, "%.1f hPa", bme.pressure_hpa);
      } else {
        lv_label_set_text(g_lbl_meteo_pressure, "---- hPa");
      }
    }
    if (g_lbl_meteo_temperature) {
      if (bme.has_data) {
        lv_label_set_text_fmt(g_lbl_meteo_temperature, "%.1f C", bme.temperature_c);
      } else {
        lv_label_set_text(g_lbl_meteo_temperature, "--.- C");
      }
    }
    if (g_lbl_meteo_humidity) {
      if (bme.has_data) {
        lv_label_set_text_fmt(g_lbl_meteo_humidity, "%.1f %%", bme.humidity_pct);
      } else {
        lv_label_set_text(g_lbl_meteo_humidity, "--.- %");
      }
    }
  }

  if (screen_comms && g_lbl_comms_wifi) {
    char wifi_text[220];
    snprintf(
        wifi_text,
        sizeof(wifi_text),
        "AP %s | %s\nLAN %s | %s\nCli %u | Hits %lu",
        wifiStateLabel(),
        ap_ip_text,
        g_wifi_sta_connected ? "OK" : (g_wifi_sta_requested ? "..." : "--"),
        g_wifi_sta_connected ? sta_ip_text : "--",
        static_cast<unsigned>(wifiClientCount()),
        static_cast<unsigned long>(g_wifi_web_hits));
    lv_label_set_text(g_lbl_comms_wifi, wifi_text);
  }

  if (screen_comms && g_lbl_comms_mode) {
    char wifi_scan_text[300];
    String scan_brief = g_wifi_scan_summary;
    const int scan_break = scan_brief.indexOf('\n');
    if (scan_break >= 0) {
      scan_brief = scan_brief.substring(0, scan_break);
    }
    if (scan_brief.isEmpty()) {
      scan_brief = "sem scan recente";
    }
    snprintf(
        wifi_scan_text,
        sizeof(wifi_scan_text),
        "Scan: %u redes\n%s",
        static_cast<unsigned>(g_wifi_scan_count),
        scan_brief.c_str());
    lv_label_set_text(g_lbl_comms_mode, wifi_scan_text);
  }

  if (screen_comms && g_lbl_comms_wifi_btn) {
    lv_label_set_text(
        g_lbl_comms_wifi_btn,
        isLandscapeUI()
            ? (g_wifi_ap_requested ? "Desligar Wi-Fi AP" : "Ligar Wi-Fi AP")
            : "Wi-Fi");
  }

  if (screen_comms && g_lbl_comms_lora_btn) {
    lv_label_set_text(
        g_lbl_comms_lora_btn,
        isLandscapeUI() ? "Abrir menu LoRa" : "Menu LoRa");
  }

  if (screen_comms && g_lbl_comms_rescan_btn) {
    lv_label_set_text(g_lbl_comms_rescan_btn, isLandscapeUI() ? "Scan Wi-Fi" : "Scan");
  }

  if (screen_comms && g_lbl_comms_sta_btn) {
    lv_label_set_text(
        g_lbl_comms_sta_btn,
        g_wifi_sta_requested ? "Desligar LAN" : (g_wifi_sta_ssid[0] ? "Ligar LAN" : "Sem SSID"));
  }

  if (screen_comms && g_lbl_comms_ble) {
    lv_label_set_text_fmt(
        g_lbl_comms_ble,
        "LAN %s\nSSID %s\n%s",
        g_wifi_sta_connected ? "OK" : (g_wifi_sta_requested ? "..." : "OFF"),
        g_wifi_sta_ssid[0] ? g_wifi_sta_ssid : "--",
        g_wifi_sta_note);
  }

  if (screen_comms && g_lbl_comms_lora) {
    const LoraState lora = copyLoraState();
    lv_label_set_text_fmt(
        g_lbl_comms_lora,
        "LoRa %s\nAUX %s\nRX %lu | TX %lu",
        g_lora_enabled ? "ON" : "OFF",
        lora.aux_high ? "HIGH" : "LOW",
        static_cast<unsigned long>(lora.rx_bytes),
        static_cast<unsigned long>(lora.tx_bytes));
  }

  if (screen_comms && g_lbl_comms_gps) {
    const GpsState gps = copyGpsState();
    lv_label_set_text_fmt(
        g_lbl_comms_gps,
        "GPS %s\nFix %s | %u sats\nUART 1/2",
        gps.receiving ? "NMEA RX" : "aguardando",
        gps.has_fix ? "OK" : "--",
        static_cast<unsigned>(gps.sats));
  }

  if (screen_comms && g_lbl_comms_status) {
    const GpsState gps = copyGpsState();
    lv_label_set_text_fmt(
        g_lbl_comms_status,
        "AP %s | LAN %s | %u cli\nGPS %s | LoRa %s",
        g_wifi_ap_started ? "ON" : "OFF",
        g_wifi_sta_connected ? "OK" : (g_wifi_sta_requested ? "..." : "OFF"),
        static_cast<unsigned>(wifiClientCount()),
        gps.has_fix ? "OK" : "--",
        g_lora_enabled ? "ON" : "OFF");
  }

  if (screen_lora && g_lbl_lora_status) {
    const LoraState lora = copyLoraState();
    lv_label_set_text_fmt(
        g_lbl_lora_status,
        "LoRa UART %s\nPinos: TX17 RX18 M0=7 M1=8 AUX=6\nAUX %s | RX %lu",
        g_lora_enabled ? "ON" : "OFF",
        lora.aux_high ? "HIGH" : "LOW",
        static_cast<unsigned long>(lora.rx_bytes));
  }

  if (screen_lora && g_lbl_lora_radio) {
    const LoraState lora = copyLoraState();
    lv_label_set_text_fmt(
        g_lbl_lora_radio,
        "Radio: E220 UART\nTX=17 RX=18 | M0=7 M1=8\nAUX=6 | %s | TX %lu",
        g_lora_enabled ? "ligado" : "desligado",
        static_cast<unsigned long>(lora.tx_bytes));
  }

  if (screen_lora && g_lbl_lora_test) {
    const LoraState lora = copyLoraState();
    lv_label_set_text(
        g_lbl_lora_test,
        g_lora_enabled
            ? lora.note
            : "LoRa UART desligado.\nLigue aqui para iniciar os testes do modulo serial.");
  }

  if (screen_lora && g_lbl_lora_toggle_btn) {
    lv_label_set_text(g_lbl_lora_toggle_btn, g_lora_enabled ? "Desligar LoRa" : "Ligar LoRa");
  }
}

static void handleAutoRotation()
{
  if (!ENABLE_AUTO_ROTATION_UI) {
    return;
  }

  if (g_orientation_mode != ORIENTATION_MODE_AUTO) {
    return;
  }

  const ImuState imu = copyImuState();
  if (!imu.connected || !imu.healthy) {
    return;
  }

  const lv_display_rotation_t candidate = autoRotationFromImu(imu);
  const uint32_t now = millis();

  if (candidate != g_auto_candidate_rotation) {
    g_auto_candidate_rotation = candidate;
    g_auto_candidate_since_ms = now;
    return;
  }

  if (candidate != g_display_rotation && (now - g_auto_candidate_since_ms) >= 700U) {
    applyDisplayRotation(candidate, SCREEN_HOME);
  }
}

static void initBootButton()
{
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  const bool pressed = digitalRead(BOOT_BUTTON_PIN) == LOW;
  g_boot_button_last_raw_pressed = pressed;
  g_boot_button_stable_pressed = pressed;
  g_boot_button_last_change_ms = millis();
  g_boot_button_press_ms = pressed ? millis() : 0;
}

static void handleBootButton()
{
  const uint32_t now = millis();
  if (now < BOOT_BUTTON_GUARD_MS) {
    return;
  }

  const bool pressed = digitalRead(BOOT_BUTTON_PIN) == LOW;
  if (pressed != g_boot_button_last_raw_pressed) {
    g_boot_button_last_raw_pressed = pressed;
    g_boot_button_last_change_ms = now;
    return;
  }

  if ((now - g_boot_button_last_change_ms) < BOOT_BUTTON_DEBOUNCE_MS) {
    return;
  }

  if (pressed != g_boot_button_stable_pressed) {
    g_boot_button_stable_pressed = pressed;

    if (pressed) {
      g_boot_button_press_ms = now;
    } else {
      const uint32_t press_ms = now - g_boot_button_press_ms;
      if (press_ms <= BOOT_BUTTON_SHORT_PRESS_MS) {
        g_sd_purge_armed = false;
        if (g_current_screen_id != SCREEN_HOME) {
          loadScreenById(SCREEN_HOME);
        }
      }
    }
  }
}

static void startImuTask()
{
  if (g_imu_task_handle) {
    return;
  }

  BaseType_t result = xTaskCreatePinnedToCore(
      imuTask,
      "imu_task",
      8192,
      nullptr,
      1,
      &g_imu_task_handle,
      0);

  if (result != pdPASS) {
    Serial.println("[IMU] Falha ao criar tarefa do QMI8658");
    g_imu_task_handle = nullptr;
    return;
  }

  Serial.println("[IMU] Tarefa do QMI8658 iniciada no core 0");
}

static void startBlackboxTask()
{
  if (g_blackbox_task_handle) {
    return;
  }

  BaseType_t result = xTaskCreatePinnedToCore(
      blackboxTask,
      "blackbox_task",
      10240,
      nullptr,
      1,
      &g_blackbox_task_handle,
      0);

  if (result != pdPASS) {
    Serial.println("[BLACKBOX] Falha ao criar tarefa do logger");
    g_blackbox_task_handle = nullptr;
    return;
  }

  Serial.println("[BLACKBOX] Tarefa da caixa-preta iniciada no core 0");
}

void setup()
{
  Serial.begin(115200);
  delay(600);

  Serial.println();
  Serial.println("=== ESP32-S3 Touch AMOLED 1.64 ===");
  Serial.println("Boot...");
  initBootButton();

  g_i2c_mutex = xSemaphoreCreateMutex();
  if (!g_i2c_mutex) {
    fatalStop("[I2C] Falha ao criar mutex");
  }

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL, 400000UL);
  delay(50);
  initBme688Sensor();
  Serial1.begin(GPS_UART_BAUD, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
  g_gps_uart_ready = true;
  clearActivityHistory();
  clearGpsActivityHistory();
  GpsState gps_state = {};
  gps_state.uart_ready = true;
  gps_state.update_rate_hz = 1U;
  snprintf(
      gps_state.note,
      sizeof(gps_state.note),
      "UART GPS pronta | RX=%d TX=%d | aguardando NMEA",
      GPS_UART_RX_PIN,
      GPS_UART_TX_PIN);
  snprintf(
      gps_state.config_note,
      sizeof(gps_state.config_note),
      "GP10 padrao: 9600 8N1 | 1Hz | WAKE alto/solto");
  snprintf(gps_state.last_command, sizeof(gps_state.last_command), "boot default");
  storeGpsState(gps_state);
  appendActivityLine("BOOT", "sistema iniciado");
  appendGpsActivityLine("BOOT", "GPS UART pronta");
  Serial.printf(
      "[GPS] UART pronta | RX=%d TX=%d | baud=%lu\n",
      GPS_UART_RX_PIN,
      GPS_UART_TX_PIN,
      static_cast<unsigned long>(GPS_UART_BAUD));
  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  pinMode(LORA_AUX_PIN, INPUT_PULLUP);
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);
  Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_UART_RX_PIN, LORA_UART_TX_PIN);
  g_lora_uart_ready = true;
  clearLoraHistory();
  LoraState lora_state = {};
  lora_state.uart_ready = true;
  lora_state.enabled = g_lora_enabled;
  lora_state.aux_high = digitalRead(LORA_AUX_PIN) == HIGH;
  lora_state.auto_tx_enabled = g_lora_auto_tx_enabled;
  lora_state.auto_tx_interval_ms = g_lora_auto_tx_interval_ms;
  lora_state.last_auto_tx_ms = g_lora_auto_tx_last_ms;
  snprintf(lora_state.auto_tx_mode, sizeof(lora_state.auto_tx_mode), "%s", g_lora_auto_tx_mode);
  snprintf(
      lora_state.note,
      sizeof(lora_state.note),
      "UART LoRa pronta | RX=%d TX=%d | AUX=%d",
      LORA_UART_RX_PIN,
      LORA_UART_TX_PIN,
      LORA_AUX_PIN);
  storeLoraState(lora_state);
  Serial.printf("[LORA] UART pronta | TX=%d RX=%d M0=%d M1=%d AUX=%d | baud=%lu\n", LORA_UART_TX_PIN, LORA_UART_RX_PIN, LORA_M0_PIN, LORA_M1_PIN, LORA_AUX_PIN, static_cast<unsigned long>(LORA_UART_BAUD));

  initDisplay();
  initTouch();
  initLVGL();
  createUI(SCREEN_HOME);
  startImuTask();
  startBlackboxTask();
  g_wifi_stack_ready = false;
  loadWifiPrefs();  // Restaura AP/STA/SSID/senha da NVS
  g_wifi_mode_requested = g_wifi_ap_requested || g_wifi_sta_requested;
  
  if (!g_wifi_ap_requested && !g_wifi_sta_requested) {
    g_wifi_ap_requested = true;  // Fallback: se nada salvo, sobe AP
    g_wifi_mode_requested = true;
  }

  g_wifi_ap_started = false;
  g_wifi_sta_started = false;
  g_wifi_sta_connected = false;
  g_wifi_web_started = false;
  g_wifi_ap_ip = IPAddress(0, 0, 0, 0);
  g_wifi_sta_ip = IPAddress(0, 0, 0, 0);
  g_wifi_web_hits = 0;
  g_wifi_last_client_ms = 0;
  g_wifi_last_status_ms = 0;

  // Carregar credenciais Wi-Fi STA persistidas na NVS
  // (Removendo chamada antiga pois loadWifiPrefs ja carregou)

  g_wifi_recover_request_ms = millis();
  updateRuntimeServiceMode();
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
  Serial.println("[WIFI] AP integrado configurado para subir automaticamente no boot");
  printWifiStatus("Boot");

  Serial.println("[BOOT] Sistema pronto");
}

void loop()
{
  lv_timer_handler();
  handleWifiPortal();
  delay(5);

  static uint32_t last_uptime = 0;
  static uint32_t last_touch_ui = 0;
  static uint32_t last_efis_ui = 0;
  static uint32_t last_config_ui = 0;
  static uint32_t last_auto_rotate = 0;
  static uint32_t last_stack_diag = 0;
  const uint32_t now = millis();

  handleBootButton();
  updateWifiLed();
  pollBme688Sensor();
  pollGpsUart();
  pollLoraUart();
  handleLoraAutoTxTick(now);

  if (now - last_uptime >= 1000U) {
    last_uptime = now;
    const ImuState imu = copyImuState();
    const Bme688State bme = copyBme688State();
    const GpsState gps = copyGpsState();
    const LoraState lora = copyLoraState();

    if (g_plane_last_update_ms == 0) {
      g_plane_smooth_alt = gps.altitude_m;
      g_plane_smooth_spd = gps.speed_kmh;
      g_plane_last_update_ms = now;
    } else {
      float dt = (now - g_plane_last_update_ms) / 1000.0f;
      g_plane_last_update_ms = now;
      float alt_sum = 0;
      int alt_c = 0;
      if (bme.connected && bme.has_data) { alt_sum += bme.altitude_m; alt_c++; }
      if (gps.has_fix) { alt_sum += gps.altitude_m; alt_c++; }
      float raw_alt = alt_c > 0 ? (alt_sum / alt_c) : g_plane_smooth_alt;
      float raw_spd = gps.has_fix ? gps.speed_kmh : g_plane_smooth_spd;
      
      if (dt > 0.0f) {
        float alt_diff = raw_alt - g_plane_smooth_alt;
        float current_vs = alt_diff / dt;
        g_plane_smooth_vs = (current_vs * 0.15f) + (g_plane_smooth_vs * 0.85f);
        g_plane_smooth_alt = (raw_alt * 0.2f) + (g_plane_smooth_alt * 0.8f);
        g_plane_smooth_spd = (raw_spd * 0.2f) + (g_plane_smooth_spd * 0.8f);
      }
    }

    if (g_lbl_uptime) {
      lv_label_set_text_fmt(g_lbl_uptime, "Uptime: %lus", now / 1000UL);
    }

    Serial.printf(
        "Uptime: %lus | Touch: %s | X:%u Y:%u | Toques:%lu | IMU:%s | Pitch:%+.1f Roll:%+.1f\n",
        now / 1000UL,
        g_touch_pressed ? "ON" : "OFF",
        static_cast<unsigned>(g_touch_x),
        static_cast<unsigned>(g_touch_y),
        static_cast<unsigned long>(g_touch_press_count),
        imu.connected ? (imu.healthy ? "OK" : "WARN") : "OFF",
        imu.pitch_deg,
        imu.roll_deg);

    if (bme.connected) {
      if (bme.has_data) {
        Serial.printf(
            "[SERIAL][BME688] addr=0x%02X | T=%.1fC | H=%.1f%% | P=%.1fhPa | Alt=%.1fm | Gas=%.0fohm | note=%s\n",
            bme.i2c_addr,
            bme.temperature_c,
            bme.humidity_pct,
            bme.pressure_hpa,
            bme.altitude_m,
            bme.gas_ohms,
            bme.note[0] ? bme.note : "OK");
      } else {
        Serial.printf(
            "[SERIAL][BME688] addr=0x%02X | sem leitura valida | note=%s\n",
            bme.i2c_addr,
            bme.note[0] ? bme.note : "--");
      }
    } else {
      Serial.printf("[SERIAL][BME688] OFF | note=%s\n", bme.note[0] ? bme.note : "nao detectado");
    }

    if (gps.uart_ready) {
      if (gps.receiving || gps.sentence_seen) {
        Serial.printf(
            "[SERIAL][GPS] RX=%lu | linhas=%lu | fix=%s | sats=%u | lat=%.6f | lon=%.6f | alt=%.1fm | spd=%.1fkmh\n",
            static_cast<unsigned long>(gps.rx_bytes),
            static_cast<unsigned long>(gps.line_count),
            gps.has_fix ? "OK" : "--",
            static_cast<unsigned>(gps.sats),
            gps.latitude_deg,
            gps.longitude_deg,
            gps.altitude_m,
            gps.speed_kmh);
        Serial.printf(
            "[SERIAL][GPS] ultima=%s | note=%s\n",
            gps.last_sentence[0] ? gps.last_sentence : "--",
            gps.note[0] ? gps.note : "--");
      } else {
        Serial.printf("[SERIAL][GPS] UART pronta | sem trafego NMEA ainda | note=%s\n", gps.note[0] ? gps.note : "--");
      }
    } else {
      Serial.println("[SERIAL][GPS] UART OFF");
    }

    Serial.printf(
        "[SERIAL][LORA] estado=%s | uart=%s | AUX=%s | RX=%lu | TX=%lu | last_rx=%lu | last_tx=%lu\n",
        lora.enabled ? "ON" : "OFF",
        lora.uart_ready ? "OK" : "OFF",
        lora.aux_high ? "HIGH" : "LOW",
        static_cast<unsigned long>(lora.rx_bytes),
        static_cast<unsigned long>(lora.tx_bytes),
        static_cast<unsigned long>(lora.last_rx_ms),
        static_cast<unsigned long>(lora.last_tx_ms));
    Serial.printf(
        "[SERIAL][LORA] ultima=%s | note=%s\n",
        lora.last_message[0] ? lora.last_message : "--",
        lora.note[0] ? lora.note : "--");

    const bool gps_rx_ok = gps.receiving || gps.rx_bytes > 0 || gps.line_count > 0;
    const bool gps_fix_ok = gps.has_fix && gps.has_location;
    const bool lora_rx_ok = lora.receiving || lora.rx_bytes > 0;
    const bool lora_tx_ok = lora.tx_bytes > 0;

    Serial.printf(
        "[UART] GPS uart=%s rx=%s fix=%s | LORA uart=%s rx=%s tx=%s aux=%s\n",
        gps.uart_ready ? "OK" : "OFF",
        gps_rx_ok ? "OK" : "OFF",
        gps_fix_ok ? "OK" : "OFF",
        lora.uart_ready ? "OK" : "OFF",
        lora_rx_ok ? "OK" : "OFF",
        lora_tx_ok ? "OK" : "OFF",
        lora.aux_high ? "HIGH" : "LOW");

    if (!gps.uart_ready) {
      Serial.println("[UART][GPS] UART OFF: revisar Serial1, boot e inicializacao.");
    } else if (!gps_rx_ok) {
  Serial.println("[UART][GPS] sem bytes NMEA: revisar VCC, GND, TXD->GPIO1, RXD->GPIO2, WAKE solto e vista do ceu.");
    } else if (!gps_fix_ok) {
      Serial.println("[UART][GPS] NMEA OK, mas sem fix: levar para area aberta e aguardar satelites.");
    } else {
      Serial.println("[UART][GPS] fix valido: mapa e velocidade do GPS devem ficar ativos.");
    }

    if (!lora.uart_ready) {
      Serial.println("[UART][LORA] UART OFF: revisar Serial2 e pinos TX17/RX18.");
    } else if (!lora.enabled) {
      Serial.println("[UART][LORA] modulo desligado por software.");
    } else if (!lora_rx_ok && !lora_tx_ok) {
      Serial.println("[UART][LORA] sem trafego: revisar segunda ponta, baud, M0/M1 LOW e mesma configuracao.");
    } else {
      Serial.println("[UART][LORA] UART ativa: validar payloads RX/TX no console web.");
    }
  }

  if (now - last_touch_ui >= 40U) {
    last_touch_ui = now;
    refreshTouchUI();
  }

  if (now - last_efis_ui >= 100U) {
    last_efis_ui = now;
    refreshEfisUI();
  }

  if (now - last_config_ui >= 500U) {
    last_config_ui = now;
    refreshConfigUI();
  }

  if (now - last_stack_diag >= 15000U) {
    last_stack_diag = now;
    const UBaseType_t imu_hw = g_imu_task_handle ? uxTaskGetStackHighWaterMark(g_imu_task_handle) : 0;
    const UBaseType_t blackbox_hw = g_blackbox_task_handle ? uxTaskGetStackHighWaterMark(g_blackbox_task_handle) : 0;
    Serial.printf(
        "[STACK] loop heap=%lu | imu_hw=%lu words | blackbox_hw=%lu words | wifi_clients=%u\n",
        static_cast<unsigned long>(ESP.getFreeHeap()),
        static_cast<unsigned long>(imu_hw),
        static_cast<unsigned long>(blackbox_hw),
        static_cast<unsigned>(wifiClientCount()));
  }

  if (now - last_auto_rotate >= 250U) {
    last_auto_rotate = now;
    handleAutoRotation();
  }

  if (g_rotation_change_pending) {
    g_rotation_change_pending = false;
    applyDisplayRotation(g_pending_rotation, g_pending_screen_id);
  }
}

#endif  // STRATOSBRAIN_BUILD_WEBCONFIG
