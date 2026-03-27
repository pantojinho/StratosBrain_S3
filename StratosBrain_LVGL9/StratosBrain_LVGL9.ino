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

static constexpr bool IMU_SWAP_XY = false;
static constexpr bool IMU_INVERT_X = false;
static constexpr bool IMU_INVERT_Y = false;
static constexpr bool IMU_INVERT_Z = false;
static constexpr bool IMU_INVERT_PITCH_SIGN = false;
static constexpr bool IMU_INVERT_ROLL_SIGN = true;

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
static constexpr uint32_t BLACKBOX_LOG_PERIOD_MS = 1000U;
static constexpr uint32_t BLACKBOX_MOUNT_RETRY_MS = 3000U;
static constexpr uint32_t BLACKBOX_FLUSH_PERIOD_MS = 5000U;
static constexpr uint32_t SD_PURGE_CONFIRM_MS = 8000U;
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
static lv_obj_t *g_screen_config = nullptr;

enum AppScreenId
{
  SCREEN_HOME = 0,
  SCREEN_EFIS,
  SCREEN_METEO,
  SCREEN_GPS,
  SCREEN_CONFIG
};

enum OrientationMode
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
static lv_obj_t *g_lbl_efis_pitch = nullptr;
static lv_obj_t *g_lbl_efis_roll = nullptr;
static lv_obj_t *g_lbl_efis_status = nullptr;
static lv_obj_t *g_lbl_efis_calibration = nullptr;
static lv_obj_t *g_lbl_plane_altitude = nullptr;
static lv_obj_t *g_lbl_plane_vario = nullptr;
static lv_obj_t *g_lbl_plane_heading = nullptr;
static lv_obj_t *g_lbl_plane_speed = nullptr;
static lv_obj_t *g_lbl_plane_skydive = nullptr;
static lv_obj_t *g_lbl_meteo_theme = nullptr;
static lv_obj_t *g_lbl_meteo_cards = nullptr;
static lv_obj_t *g_lbl_comms_status = nullptr;
static lv_obj_t *g_lbl_comms_wifi = nullptr;
static lv_obj_t *g_lbl_comms_ble = nullptr;
static lv_obj_t *g_lbl_comms_lora = nullptr;
static lv_obj_t *g_lbl_comms_gps = nullptr;
static lv_obj_t *g_lbl_comms_mode = nullptr;
static lv_obj_t *g_lbl_comms_wifi_btn = nullptr;
static lv_obj_t *g_lbl_comms_lora_btn = nullptr;
static lv_obj_t *g_lbl_comms_rescan_btn = nullptr;
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
static esp_netif_t *g_wifi_ap_netif = nullptr;
static NetworkServer g_web_server(WIFI_WEB_PORT);
static bool g_wifi_stack_ready = false;
static bool g_wifi_ap_started = false;
static bool g_wifi_web_started = false;
static uint32_t g_wifi_web_hits = 0;
static uint32_t g_wifi_last_client_ms = 0;
static uint32_t g_wifi_last_status_ms = 0;
static bool g_wifi_mode_requested = false;
static bool g_lora_enabled = false;
static bool g_runtime_services_light = false;
static char g_wifi_diag_note[BLACKBOX_NOTE_LEN] = "AP desligado. Ligue em COMMS ou CONFIG.";
static float g_efis_display_pitch_deg = 0.0f;
static float g_efis_display_roll_deg = 0.0f;
static bool g_efis_display_seeded = false;
static bool g_boot_button_last_raw_pressed = false;
static bool g_boot_button_stable_pressed = false;
static uint32_t g_boot_button_last_change_ms = 0;
static uint32_t g_boot_button_press_ms = 0;

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

static portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_blackbox_mux = portMUX_INITIALIZER_UNLOCKED;
static ImuState g_imu_state = {};
static BlackboxState g_blackbox_state = {};
static String scanI2C();
static const IPAddress WIFI_AP_LOCAL_IP(192, 168, 4, 1);
static const IPAddress WIFI_AP_GATEWAY(192, 168, 4, 1);
static const IPAddress WIFI_AP_SUBNET(255, 255, 255, 0);

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

  if (g_wifi_mode_requested) {
    return "subindo";
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

static void formatWifiVisibleIp(char *buffer, size_t buffer_len)
{
  if (g_wifi_ap_started) {
    formatIpAddress(g_wifi_ap_ip, buffer, buffer_len);
    return;
  }

  formatIpAddress(WIFI_AP_LOCAL_IP, buffer, buffer_len);
}

static void printWifiStatus(const char *reason)
{
  char ip_text[20];
  char mac_text[24];
  formatWifiVisibleIp(ip_text, sizeof(ip_text));
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
      "[WIFI] %s | req=%s | ap=%s | mode=%d | ssid=%s | pass=%s | ip=%s | ch=%u | hidden=%u | clients=%u | hits=%lu | mac=%s | heap=%lu | psram=%lu\n",
      reason ? reason : "Status",
      g_wifi_mode_requested ? "on" : "off",
      g_wifi_ap_started ? "on" : "off",
      static_cast<int>(mode),
      WIFI_AP_SSID,
      WIFI_AP_PASSWORD,
      ip_text,
      static_cast<unsigned>(WIFI_AP_CHANNEL),
      static_cast<unsigned>(WIFI_AP_HIDDEN ? 1 : 0),
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
  const bool light_mode = g_wifi_mode_requested || g_wifi_ap_started;
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
    if (g_current_screen_id == SCREEN_METEO) {
      loadScreenById(SCREEN_GPS);
    }
    Serial.println("[MODE] Wi-Fi ativo: cockpit pesado em modo leve");
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

static void startWifiPortal()
{
  if (g_wifi_ap_started) {
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP ja ativo em http://192.168.4.1");
    printWifiStatus("AP ja ativo");
    return;
  }

  Serial.println("[WIFI] Iniciando SoftAP...");
  if (!ensureWifiStack()) {
    g_wifi_mode_requested = false;
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "Falha no stack Wi-Fi. Veja o serial.");
    return;
  }

  delay(100);
  esp_wifi_stop();
  Serial.printf(
      "[WIFI] Config alvo | SSID: %s | Senha: %s | IP: 192.168.4.1 | Canal: %u\n",
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

  wifi_config_t ap_config = {};
  strncpy(reinterpret_cast<char *>(ap_config.ap.ssid), WIFI_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
  ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
  strncpy(reinterpret_cast<char *>(ap_config.ap.password), WIFI_AP_PASSWORD, sizeof(ap_config.ap.password) - 1);
  ap_config.ap.channel = WIFI_AP_CHANNEL;
  ap_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
  ap_config.ap.ssid_hidden = WIFI_AP_HIDDEN ? 1 : 0;
  ap_config.ap.authmode = strlen(WIFI_AP_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

  esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
  if (err == ESP_OK) {
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
  }
  if (err == ESP_OK) {
    err = esp_wifi_start();
  }
  if (err == ESP_OK) {
    err = esp_wifi_set_ps(WIFI_PS_NONE);
  }

  if (err != ESP_OK) {
    g_wifi_mode_requested = false;
    g_wifi_ap_started = false;
    g_wifi_web_started = false;
    updateRuntimeServiceMode();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "Falha ao iniciar AP. Veja o serial.");
    Serial.printf("[WIFI] Falha ao iniciar SoftAP: %s\n", esp_err_to_name(err));
    printWifiStatus("Falha ao iniciar");
    return;
  }

  delay(250);
  refreshWifiApIp();
  if (g_wifi_ap_ip == IPAddress(0, 0, 0, 0)) {
    g_wifi_mode_requested = false;
    g_wifi_ap_started = false;
    g_wifi_web_started = false;
    updateRuntimeServiceMode();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP respondeu sem IP valido.");
    Serial.println("[WIFI] AP respondeu, mas o IP veio 0.0.0.0");
    printWifiStatus("AP sem IP");
    return;
  }

  g_wifi_mode_requested = true;
  g_wifi_ap_started = true;
  g_web_server.begin();
  g_web_server.setNoDelay(true);
  g_wifi_web_started = true;
  g_wifi_last_status_ms = millis();
  updateRuntimeServiceMode();

  char ip_text[20];
  formatWifiVisibleIp(ip_text, sizeof(ip_text));
  snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP ativo em http://%s", ip_text);
  Serial.printf(
      "[WIFI] AP ativo | SSID: %s | Senha: %s | IP: %s | URL: http://%s\n",
      WIFI_AP_SSID,
      WIFI_AP_PASSWORD,
      ip_text,
      ip_text);
  printWifiStatus("AP ativo");
}

static void stopWifiPortal()
{
  g_wifi_mode_requested = false;

  wifi_mode_t mode = WIFI_MODE_NULL;
  esp_wifi_get_mode(&mode);
  if (!g_wifi_ap_started && !g_wifi_web_started && mode == WIFI_MODE_NULL) {
    updateRuntimeServiceMode();
    snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP desligado. Ligue em COMMS ou CONFIG.");
    return;
  }

  Serial.println("[WIFI] Desligando SoftAP...");
  g_web_server.stop();
  esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
  g_wifi_ap_started = false;
  g_wifi_web_started = false;
  g_wifi_ap_ip = IPAddress(0, 0, 0, 0);
  g_wifi_web_hits = 0;
  g_wifi_last_client_ms = 0;
  g_wifi_last_status_ms = 0;
  updateRuntimeServiceMode();
  snprintf(g_wifi_diag_note, sizeof(g_wifi_diag_note), "AP desligado. Cockpit completo liberado.");
  printWifiStatus("AP desligado");
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

static void sendHttpJsonStatus(NetworkClient &client)
{
  const ImuState imu = copyImuState();
  const BlackboxState blackbox = copyBlackboxState();
  char ip_text[20];
  formatIpAddress(g_wifi_ap_ip, ip_text, sizeof(ip_text));

  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: application/json; charset=utf-8\r\n"));
  client.print(F("Cache-Control: no-store\r\n"));
  client.print(F("Connection: close\r\n\r\n"));
  client.printf(
      "{\"device\":\"StratosBrain S3\",\"wifi_ap\":%s,\"ssid\":\"%s\",\"ip\":\"%s\",\"web_hits\":%lu,"
      "\"uptime_s\":%lu,\"touch\":{\"pressed\":%s,\"x\":%u,\"y\":%u,\"count\":%lu},"
      "\"imu\":{\"connected\":%s,\"healthy\":%s,\"pitch_deg\":%.1f,\"roll_deg\":%.1f},"
      "\"blackbox\":{\"mounted\":%s,\"logging_enabled\":%s,\"records\":%lu,\"file\":\"%s\"}}\n",
      g_wifi_ap_started ? "true" : "false",
      WIFI_AP_SSID,
      ip_text,
      static_cast<unsigned long>(g_wifi_web_hits),
      static_cast<unsigned long>(millis() / 1000UL),
      g_touch_pressed ? "true" : "false",
      static_cast<unsigned>(g_touch_x),
      static_cast<unsigned>(g_touch_y),
      static_cast<unsigned long>(g_touch_press_count),
      imu.connected ? "true" : "false",
      imu.healthy ? "true" : "false",
      imu.pitch_deg,
      imu.roll_deg,
      blackbox.mounted ? "true" : "false",
      blackbox.logging_enabled ? "true" : "false",
      static_cast<unsigned long>(blackbox.records_written),
      blackbox.file_path[0] ? blackbox.file_path : "");
}

static void sendHttpDashboard(NetworkClient &client)
{
  const ImuState imu = copyImuState();
  const BlackboxState blackbox = copyBlackboxState();
  char ip_text[20];
  char sd_size_text[24];
  formatIpAddress(g_wifi_ap_ip, ip_text, sizeof(ip_text));
  formatStorageSize(blackbox.card_size_bytes, sd_size_text, sizeof(sd_size_text));

  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: text/html; charset=utf-8\r\n"));
  client.print(F("Cache-Control: no-store\r\n"));
  client.print(F("Connection: close\r\n\r\n"));
  client.print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"));
  client.print(F("<meta name='viewport' content='width=device-width,initial-scale=1'>"));
  client.print(F("<meta http-equiv='refresh' content='3'>"));
  client.print(F("<title>StratosBrain S3</title>"));
  client.print(F("<style>body{margin:0;font-family:Arial,sans-serif;background:#070b11;color:#eef4ff;}"));
  client.print(F(".wrap{padding:16px;max-width:760px;margin:0 auto;}"));
  client.print(F(".hero{background:linear-gradient(180deg,#0f2034,#09121c);border:1px solid #235c8f;border-radius:18px;padding:16px;margin-bottom:14px;}"));
  client.print(F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;}"));
  client.print(F(".card{background:#0c141d;border:1px solid #223344;border-radius:16px;padding:14px;}"));
  client.print(F(".k{color:#84d7ff;font-size:12px;text-transform:uppercase;letter-spacing:.08em;}"));
  client.print(F(".v{font-size:24px;font-weight:700;margin-top:8px;}"));
  client.print(F(".s{color:#b9c9d9;font-size:14px;line-height:1.4;}a{color:#7ed1ff;}</style></head><body>"));
  client.print(F("<div class='wrap'><div class='hero'><div class='k'>StratosBrain S3</div>"));
  client.printf("<div class='v'>AP %s</div>", g_wifi_ap_started ? "ativo" : "offline");
  client.printf("<div class='s'>SSID: %s<br>Senha: %s<br>IP: <a href='http://%s'>http://%s</a><br>JSON: <a href='/api/status'>/api/status</a></div>",
                WIFI_AP_SSID,
                WIFI_AP_PASSWORD,
                ip_text,
                ip_text);
  client.print(F("</div><div class='grid'>"));
  client.printf("<div class='card'><div class='k'>Uptime</div><div class='v'>%lus</div><div class='s'>Requisicoes web: %lu</div></div>",
                static_cast<unsigned long>(millis() / 1000UL),
                static_cast<unsigned long>(g_wifi_web_hits));
  client.printf("<div class='card'><div class='k'>IMU</div><div class='v'>%s</div><div class='s'>Pitch: %+.1f deg<br>Roll: %+.1f deg</div></div>",
                imu.connected ? (imu.healthy ? "OK" : "WARN") : "OFF",
                imu.pitch_deg,
                imu.roll_deg);
  client.printf("<div class='card'><div class='k'>Touch</div><div class='v'>%s</div><div class='s'>X:%u Y:%u<br>Toques: %lu</div></div>",
                g_touch_pressed ? "ON" : "OFF",
                static_cast<unsigned>(g_touch_x),
                static_cast<unsigned>(g_touch_y),
                static_cast<unsigned long>(g_touch_press_count));
  client.printf("<div class='card'><div class='k'>SD / Blackbox</div><div class='v'>%s</div><div class='s'>Modo: %s<br>Capacidade: %s<br>Arquivo: %s<br>Registros: %lu</div></div>",
                blackbox.mounted ? "montado" : "sem cartao",
                blackbox.logging_enabled ? "armado" : "pausado",
                sd_size_text,
                blackbox.file_path[0] ? blackbox.file_path : "-",
                static_cast<unsigned long>(blackbox.records_written));
  client.print(F("</div></div></body></html>"));
}

static void handleWifiPortal()
{
  const uint32_t now = millis();

  if (g_wifi_mode_requested && !g_wifi_ap_started) {
    startWifiPortal();
  } else if (!g_wifi_mode_requested && g_wifi_ap_started) {
    stopWifiPortal();
  }

  if (!g_wifi_ap_started || !g_wifi_web_started) {
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

  if (strncmp(request_line, "GET /api/status", 15) == 0) {
    sendHttpJsonStatus(client);
  } else {
    sendHttpDashboard(client);
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
      {"BMM350", 0x14, 0xFF},
      {"LTR390UV", 0x53, 0xFF},
      {"MAX17048", 0x36, 0xFF}};

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
  const bool uv_ok = scanHasAddress(scan, 0x53);
  const bool bat_ok = scanHasAddress(scan, 0x36);
  const uint8_t meteo_ok = (bme_ok ? 1U : 0U) + (bmp_ok ? 1U : 0U) + (bmm_ok ? 1U : 0U) + (uv_ok ? 1U : 0U);

  char buffer[128];
  snprintf(
      buffer,
      sizeof(buffer),
      "Touch %s | IMU %s\nMeteo %u/4 | Bat %s",
      touch_ok ? "OK" : "--",
      imu_ok ? "OK" : "--",
      static_cast<unsigned>(meteo_ok),
      bat_ok ? "OK" : "--");
  return String(buffer);
}

static void refreshSensorCaches()
{
  g_sensor_summary_cache = formatKnownSensorSummary();
  g_sensor_raw_scan_cache = scanI2C();
  g_sensor_compact_cache = formatCompactSensorSummary(g_sensor_raw_scan_cache);
  g_sensor_scan_compact_cache = compactScanLine(g_sensor_raw_scan_cache);
}

static String scanI2C()
{
  String result;
  uint8_t found = 0;

  if (!lockI2C(pdMS_TO_TICKS(50))) {
    return "Barramento I2C ocupado";
  }

  for (uint8_t addr = 1; addr < 127; ++addr) {
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
        "ms,imu_ok,pitch_deg,roll_deg,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,temp_c,touch_count,gps_fix,gps_lat,gps_lon,gps_alt_m,gps_speed_kmh,gps_sats");
    g_blackbox_file.flush();
  }

  snprintf(state->file_path, sizeof(state->file_path), "%s", path);
  state->file_open = true;
  state->records_written = 0;
  snprintf(state->note, sizeof(state->note), "Gravando IMU. GPS reservado para proxima etapa");
  return true;
}

static bool appendBlackboxRecord(BlackboxState *state, uint32_t now_ms)
{
  if (!state || !state->mounted || !state->file_open || !g_blackbox_file) {
    return false;
  }

  const ImuState imu = copyImuState();
  const bool imu_ok = imu.connected && imu.healthy && imu.has_solution;
  char line[256];
  const int line_len = snprintf(
      line,
      sizeof(line),
      "%lu,%u,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%lu,0,,,,,\n",
      static_cast<unsigned long>(now_ms),
      imu_ok ? 1U : 0U,
      imu.pitch_deg,
      imu.roll_deg,
      imu.acc_mps2[0],
      imu.acc_mps2[1],
      imu.acc_mps2[2],
      imu.gyro_dps[0],
      imu.gyro_dps[1],
      imu.gyro_dps[2],
      imu.temperature_c,
      static_cast<unsigned long>(g_touch_press_count));

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

  g_panel->setBrightness(255);
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

    if (!state.mounted && state.logging_enabled && (now - last_mount_attempt_ms) >= BLACKBOX_MOUNT_RETRY_MS) {
      last_mount_attempt_ms = now;

      if (mountBlackboxStorage(&state)) {
        Serial.println("[BLACKBOX] Cartao SD montado");
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

  if (g_runtime_services_light && screen_id == SCREEN_METEO) {
    Serial.println("[MODE] Saindo do modo Wi-Fi para abrir o painel METEO completo");
    g_wifi_mode_requested = false;
    stopWifiPortal();
  }

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
  const lv_coord_t summary_h = landscape ? 88 : 94;
  const lv_coord_t action_h = 42;
  const lv_coord_t bottom_reserved = landscape ? 58 : 104;
  const lv_coord_t info_y = top_y + summary_h + 8;
  const lv_coord_t info_h = screen_h - info_y - bottom_reserved;
  const lv_coord_t info_panel_w = screen_w - 16;
  g_efis_horizon_w = 0;
  g_efis_horizon_h = 0;
  g_efis_horizon = nullptr;

  g_screen_efis = lv_obj_create(nullptr);
  styleScreen(g_screen_efis);
  createHeader(g_screen_efis, "PLANE", "dados essenciais");

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
  lv_label_set_text(g_lbl_efis_status, "IMU aguardando");
  lv_label_set_long_mode(g_lbl_efis_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_efis_status, screen_w - 40);
  lv_obj_set_style_text_color(g_lbl_efis_status, lv_color_hex(0xE7EDF5), 0);
  lv_obj_align(g_lbl_efis_status, LV_ALIGN_TOP_LEFT, 0, 28);

  g_lbl_plane_skydive = lv_label_create(summary_panel);
  lv_label_set_text(g_lbl_plane_skydive, "SKYDIVE: aguardando altimetro");
  lv_label_set_long_mode(g_lbl_plane_skydive, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_plane_skydive, screen_w - 40);
  lv_obj_set_style_text_color(g_lbl_plane_skydive, lv_color_hex(0xFFCF8A), 0);
  lv_obj_align(g_lbl_plane_skydive, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  lv_obj_t *info_panel = lv_obj_create(g_screen_efis);
  lv_obj_clear_flag(info_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(info_panel, info_panel_w, info_h);
  lv_obj_align(info_panel, LV_ALIGN_TOP_MID, 0, info_y);
  lv_obj_set_style_bg_color(info_panel, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(info_panel, lv_color_hex(0x2E7DD1), 0);
  lv_obj_set_style_border_width(info_panel, 2, 0);
  lv_obj_set_style_radius(info_panel, 16, 0);
  lv_obj_set_style_pad_all(info_panel, 8, 0);

  const lv_coord_t plane_panel_w = info_panel_w - 20;
  const lv_coord_t half_w = (plane_panel_w - 8) / 2;
  const lv_coord_t card_h = (info_h - 26) / 2;
  const lv_coord_t row2_y = card_h + 8;
  createDataCard(info_panel, half_w, card_h, LV_ALIGN_TOP_LEFT, 0, 0, 0xFFD34D, "ALT", "---- ft", &g_lbl_plane_altitude);
  createDataCard(info_panel, half_w, card_h, LV_ALIGN_TOP_RIGHT, 0, 0, 0x45D1FF, "V/S", "---- fpm", &g_lbl_plane_vario);
  createDataCard(info_panel, half_w, card_h, LV_ALIGN_TOP_LEFT, 0, row2_y, 0x53C2A3, "HDG", "--- dg", &g_lbl_plane_heading);
  createDataCard(info_panel, half_w, card_h, LV_ALIGN_TOP_RIGHT, 0, row2_y, 0xFFB357, "SPD", "-- km/h", &g_lbl_plane_speed);

  const lv_coord_t action_w = landscape ? (screen_w - 36) / 2 : screen_w - 32;
  lv_obj_t *calibrate_btn = lv_button_create(g_screen_efis);
  lv_obj_set_size(calibrate_btn, action_w, action_h);
  lv_obj_align(
      calibrate_btn,
      landscape ? LV_ALIGN_BOTTOM_LEFT : LV_ALIGN_BOTTOM_MID,
      landscape ? 12 : 0,
      landscape ? -10 : -58);
  lv_obj_set_style_bg_color(calibrate_btn, lv_color_hex(0x24384A), 0);
  lv_obj_set_style_radius(calibrate_btn, 12, 0);
  lv_obj_add_event_cb(calibrate_btn, efisSetLevelEventCb, LV_EVENT_CLICKED, nullptr);

  g_lbl_efis_calibration = lv_label_create(calibrate_btn);
  lv_label_set_text(g_lbl_efis_calibration, "Setar nivel");
  lv_obj_center(g_lbl_efis_calibration);

  if (landscape) {
    createBackButton(g_screen_efis, LV_ALIGN_BOTTOM_RIGHT, -12, -10, action_w);
  } else {
    createBackButton(g_screen_efis, LV_ALIGN_BOTTOM_MID, 0, -10, screen_w - 32);
  }
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
  g_screen_config = nullptr;
  g_lbl_uptime = nullptr;
  g_lbl_i2c = nullptr;
  g_lbl_touch = nullptr;
  g_touch_dot = nullptr;
  g_efis_horizon = nullptr;
  g_lbl_efis_pitch = nullptr;
  g_lbl_efis_roll = nullptr;
  g_lbl_efis_status = nullptr;
  g_lbl_efis_calibration = nullptr;
  g_lbl_plane_altitude = nullptr;
  g_lbl_plane_vario = nullptr;
  g_lbl_plane_heading = nullptr;
  g_lbl_plane_speed = nullptr;
  g_lbl_plane_skydive = nullptr;
  g_lbl_meteo_theme = nullptr;
  g_lbl_meteo_cards = nullptr;
  g_lbl_comms_status = nullptr;
  g_lbl_comms_wifi = nullptr;
  g_lbl_comms_ble = nullptr;
  g_lbl_comms_lora = nullptr;
  g_lbl_comms_gps = nullptr;
  g_lbl_comms_mode = nullptr;
  g_lbl_comms_wifi_btn = nullptr;
  g_lbl_comms_lora_btn = nullptr;
  g_lbl_comms_rescan_btn = nullptr;
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
}

static void createConfigScreen();
static void createMeteoScreen();
static void createCommsScreen();
static void createUI(AppScreenId initial_screen_id);

static void rebuildUI(AppScreenId target_screen_id)
{
  lv_obj_t *old_main = g_screen_main;
  lv_obj_t *old_efis = g_screen_efis;
  lv_obj_t *old_meteo = g_screen_meteo;
  lv_obj_t *old_gps = g_screen_gps;
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

  g_wifi_mode_requested = !g_wifi_mode_requested;
  if (g_wifi_mode_requested) {
    g_sd_purge_armed = false;
    loadScreenById(SCREEN_GPS);
    Serial.println("[WIFI] Pedido para ligar o AP via UI");
  } else {
    Serial.println("[WIFI] Pedido para desligar o AP via UI");
  }
  updateRuntimeServiceMode();
}

static void toggleLoraEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  g_lora_enabled = !g_lora_enabled;
  Serial.printf(
      "[LORA] Estado alterado para %s (backend de radio ainda pendente nesta etapa)\n",
      g_lora_enabled ? "ON" : "OFF");
}

static void rescanCommsEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  g_blackbox_remount_requested = true;
  refreshSensorCaches();
  Serial.println("[COMMS] Rescan solicitado: sensores e SD");
  printWifiStatus("Rescan UI");
}

static void toggleBlackboxEventCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  g_blackbox_enabled_target = !g_blackbox_enabled_target;
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
  const bool landscape = isLandscapeUI();
  const int32_t top_y = landscape ? 52 : 60;
  const lv_coord_t full_w = uiWidth() - 16;
  const lv_coord_t bottom_btn_h = 42;
  const lv_coord_t net_h = landscape ? 140 : 148;

  g_screen_config = lv_obj_create(nullptr);
  styleScreen(g_screen_config);
  lv_obj_add_flag(g_screen_config, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(g_screen_config, LV_SCROLLBAR_MODE_AUTO);
  createHeader(g_screen_config, "CONFIG", "");

  const lv_coord_t left_w = landscape ? 172 : full_w;
  const lv_coord_t right_w = landscape ? (uiWidth() - left_w - 32) : full_w;
  const lv_coord_t system_h = 96;
  const lv_coord_t storage_h = landscape ? 168 : 128;
  const lv_coord_t sensor_h = landscape ? 72 : 88;
  const lv_coord_t portrait_net_y = top_y + system_h + 8;
  const lv_coord_t portrait_storage_y = portrait_net_y + net_h + 8;
  const lv_coord_t portrait_sensor_y = portrait_storage_y + storage_h + 8;
  const lv_coord_t portrait_buttons_y = portrait_sensor_y + sensor_h + 10;

  lv_obj_t *display_card = lv_obj_create(g_screen_config);
  lv_obj_set_size(display_card, left_w, system_h);
  lv_obj_align(display_card, LV_ALIGN_TOP_LEFT, 8, top_y);
  lv_obj_set_style_bg_color(display_card, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(display_card, lv_color_hex(0xA44CB4), 0);
  lv_obj_set_style_border_width(display_card, 2, 0);
  lv_obj_set_style_radius(display_card, 16, 0);
  lv_obj_set_style_pad_all(display_card, 10, 0);

  lv_obj_t *display_title = lv_label_create(display_card);
  lv_label_set_text(display_title, "Tela");
  lv_obj_set_style_text_color(display_title, lv_color_hex(0xEFD9FF), 0);
  lv_obj_align(display_title, LV_ALIGN_TOP_LEFT, 0, 0);

  g_lbl_config_orientation = lv_label_create(display_card);
  lv_label_set_text(g_lbl_config_orientation, "Modo: Vertical");
  lv_obj_set_style_text_color(g_lbl_config_orientation, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(g_lbl_config_orientation, LV_ALIGN_TOP_LEFT, 0, 22);

  g_lbl_config_rotation = lv_label_create(display_card);
  lv_label_set_text(g_lbl_config_rotation, "Ativa: Vertical");
  lv_obj_set_style_text_color(g_lbl_config_rotation, lv_color_hex(0xC9D4E5), 0);
  lv_obj_align(g_lbl_config_rotation, LV_ALIGN_TOP_LEFT, 0, 42);

  lv_obj_t *orientation_box = lv_obj_create(display_card);
  lv_obj_set_size(orientation_box, left_w - 20, 34);
  lv_obj_align(orientation_box, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(orientation_box, lv_color_hex(0x1A2638), 0);
  lv_obj_set_style_border_width(orientation_box, 0, 0);
  lv_obj_set_style_radius(orientation_box, 12, 0);
  lv_obj_clear_flag(orientation_box, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *portrait_label = lv_label_create(orientation_box);
  lv_label_set_text(portrait_label, "Rotacao travada");
  lv_obj_center(portrait_label);

  lv_obj_t *net_card = lv_obj_create(g_screen_config);
  lv_obj_set_size(net_card, right_w, net_h);
  if (landscape) {
    lv_obj_align(net_card, LV_ALIGN_TOP_RIGHT, -8, top_y);
  } else {
    lv_obj_align(net_card, LV_ALIGN_TOP_MID, 0, portrait_net_y);
  }
  lv_obj_set_style_bg_color(net_card, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(net_card, lv_color_hex(0x46D8A8), 0);
  lv_obj_set_style_border_width(net_card, 2, 0);
  lv_obj_set_style_radius(net_card, 16, 0);
  lv_obj_set_style_pad_all(net_card, 10, 0);

  lv_obj_t *net_title = lv_label_create(net_card);
  lv_label_set_text(net_title, "Rede e links");
  lv_obj_set_style_text_color(net_title, lv_color_hex(0xB7FFE3), 0);
  lv_obj_align(net_title, LV_ALIGN_TOP_LEFT, 0, 0);

  g_lbl_config_net = lv_label_create(net_card);
  lv_label_set_text(g_lbl_config_net, "");
  lv_label_set_long_mode(g_lbl_config_net, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_net, right_w - 20);
  lv_obj_set_style_text_color(g_lbl_config_net, lv_color_hex(0xEAFBF4), 0);
  lv_obj_align(g_lbl_config_net, LV_ALIGN_TOP_LEFT, 0, 22);

  const lv_coord_t net_btn_w = (right_w - 28) / 2;

  lv_obj_t *wifi_btn = lv_button_create(net_card);
  lv_obj_set_size(wifi_btn, net_btn_w, 34);
  lv_obj_align(wifi_btn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(wifi_btn, lv_color_hex(0x285C92), 0);
  lv_obj_set_style_radius(wifi_btn, 12, 0);
  lv_obj_add_event_cb(wifi_btn, toggleWifiEventCb, LV_EVENT_CLICKED, nullptr);

  g_lbl_config_wifi_btn = lv_label_create(wifi_btn);
  lv_label_set_text(g_lbl_config_wifi_btn, "Wi-Fi AP");
  lv_obj_center(g_lbl_config_wifi_btn);

  lv_obj_t *lora_btn = lv_button_create(net_card);
  lv_obj_set_size(lora_btn, net_btn_w, 34);
  lv_obj_align(lora_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(lora_btn, lv_color_hex(0x8B3E2A), 0);
  lv_obj_set_style_radius(lora_btn, 12, 0);
  lv_obj_add_event_cb(lora_btn, toggleLoraEventCb, LV_EVENT_CLICKED, nullptr);

  g_lbl_config_lora_btn = lv_label_create(lora_btn);
  lv_label_set_text(g_lbl_config_lora_btn, "LoRa");
  lv_obj_center(g_lbl_config_lora_btn);

  lv_obj_t *blackbox_card = lv_obj_create(g_screen_config);
  lv_obj_set_size(blackbox_card, right_w, storage_h);
  if (landscape) {
    lv_obj_align(blackbox_card, LV_ALIGN_TOP_RIGHT, -8, top_y + net_h + 8);
  } else {
    lv_obj_align(blackbox_card, LV_ALIGN_TOP_MID, 0, portrait_storage_y);
  }
  lv_obj_set_style_bg_color(blackbox_card, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(blackbox_card, lv_color_hex(0xFFD86B), 0);
  lv_obj_set_style_border_width(blackbox_card, 2, 0);
  lv_obj_set_style_radius(blackbox_card, 16, 0);
  lv_obj_set_style_pad_all(blackbox_card, 10, 0);

  lv_obj_t *blackbox_title = lv_label_create(blackbox_card);
  lv_label_set_text(blackbox_title, "Armazenamento");
  lv_obj_set_style_text_color(blackbox_title, lv_color_hex(0xFFD86B), 0);
  lv_obj_align(blackbox_title, LV_ALIGN_TOP_LEFT, 0, 0);

  g_lbl_config_blackbox = lv_label_create(blackbox_card);
  lv_label_set_text(g_lbl_config_blackbox, "");
  lv_label_set_long_mode(g_lbl_config_blackbox, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_blackbox, right_w - 20);
  lv_obj_set_style_text_color(g_lbl_config_blackbox, lv_color_hex(0xFFF4CC), 0);
  lv_obj_align(g_lbl_config_blackbox, LV_ALIGN_TOP_LEFT, 0, 22);

  g_lbl_config_note = lv_label_create(blackbox_card);
  lv_label_set_text(g_lbl_config_note, "Logger local e limpeza dos logs");
  lv_label_set_long_mode(g_lbl_config_note, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_note, right_w - 20);
  lv_obj_set_style_text_color(g_lbl_config_note, lv_color_hex(0xFFD86B), 0);
  lv_obj_align(g_lbl_config_note, LV_ALIGN_TOP_LEFT, 0, landscape ? 92 : 74);

  const lv_coord_t storage_btn_w = (right_w - 28) / 2;

  lv_obj_t *logger_btn = lv_button_create(blackbox_card);
  lv_obj_set_size(logger_btn, storage_btn_w, 34);
  lv_obj_align(logger_btn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(logger_btn, lv_color_hex(0x2A2232), 0);
  lv_obj_set_style_radius(logger_btn, 12, 0);
  lv_obj_add_event_cb(logger_btn, toggleBlackboxEventCb, LV_EVENT_CLICKED, nullptr);

  g_lbl_config_blackbox_btn = lv_label_create(logger_btn);
  lv_label_set_text(g_lbl_config_blackbox_btn, "Logger");
  lv_obj_center(g_lbl_config_blackbox_btn);

  lv_obj_t *format_btn = lv_button_create(blackbox_card);
  lv_obj_set_size(format_btn, storage_btn_w, 34);
  lv_obj_align(format_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(format_btn, lv_color_hex(0x4B2721), 0);
  lv_obj_set_style_radius(format_btn, 12, 0);
  lv_obj_add_event_cb(format_btn, purgeSdLogsEventCb, LV_EVENT_CLICKED, nullptr);

  g_lbl_config_format_btn = lv_label_create(format_btn);
  lv_label_set_text(g_lbl_config_format_btn, "Limpar SD");
  lv_obj_center(g_lbl_config_format_btn);

  lv_obj_t *sensor_card = lv_obj_create(g_screen_config);
  lv_obj_set_size(sensor_card, left_w, sensor_h);
  if (landscape) {
    lv_obj_align(sensor_card, LV_ALIGN_TOP_LEFT, 8, top_y + system_h + 8);
  } else {
    lv_obj_align(sensor_card, LV_ALIGN_TOP_MID, 0, portrait_sensor_y);
  }
  lv_obj_set_style_bg_color(sensor_card, lv_color_hex(0x0B111A), 0);
  lv_obj_set_style_border_color(sensor_card, lv_color_hex(0x4E92FF), 0);
  lv_obj_set_style_border_width(sensor_card, 2, 0);
  lv_obj_set_style_radius(sensor_card, 16, 0);
  lv_obj_set_style_pad_all(sensor_card, 10, 0);

  lv_obj_t *sensor_title = lv_label_create(sensor_card);
  lv_label_set_text(sensor_title, "Sensores");
  lv_obj_set_style_text_color(sensor_title, lv_color_hex(0xD7E6FF), 0);
  lv_obj_align(sensor_title, LV_ALIGN_TOP_LEFT, 0, 0);

  g_lbl_config_sensors = lv_label_create(sensor_card);
  lv_label_set_text(g_lbl_config_sensors, "");
  lv_label_set_long_mode(g_lbl_config_sensors, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_config_sensors, left_w - 20);
  lv_obj_set_style_text_color(g_lbl_config_sensors, lv_color_hex(0xE7EDF5), 0);
  lv_obj_align(g_lbl_config_sensors, LV_ALIGN_TOP_LEFT, 0, 20);

  g_lbl_config_scan_raw = lv_label_create(sensor_card);
  lv_label_set_text(g_lbl_config_scan_raw, "");
  lv_label_set_long_mode(g_lbl_config_scan_raw, LV_LABEL_LONG_CLIP);
  lv_obj_set_width(g_lbl_config_scan_raw, left_w - 20);
  lv_obj_set_style_text_color(g_lbl_config_scan_raw, lv_color_hex(0x7EC8FF), 0);
  lv_obj_align(g_lbl_config_scan_raw, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  lv_obj_t *refresh_btn = lv_button_create(g_screen_config);
  lv_obj_set_size(refresh_btn, landscape ? left_w : (uiWidth() - 40) / 2, bottom_btn_h);
  if (landscape) {
    lv_obj_align(refresh_btn, LV_ALIGN_BOTTOM_LEFT, 8, -10);
  } else {
    lv_obj_align(refresh_btn, LV_ALIGN_TOP_LEFT, 12, portrait_buttons_y);
  }
  lv_obj_set_style_bg_color(refresh_btn, lv_color_hex(0x1A2638), 0);
  lv_obj_set_style_radius(refresh_btn, 14, 0);
  lv_obj_add_event_cb(refresh_btn, refreshSensorsEventCb, LV_EVENT_CLICKED, nullptr);

  g_lbl_config_refresh = lv_label_create(refresh_btn);
  lv_label_set_text(g_lbl_config_refresh, "Atualizar");
  lv_obj_center(g_lbl_config_refresh);

  if (landscape) {
    createBackButton(
        g_screen_config,
        LV_ALIGN_BOTTOM_RIGHT,
        -8,
        -10,
        right_w);
  } else {
    lv_obj_t *back_btn = lv_button_create(g_screen_config);
    lv_obj_set_size(back_btn, (uiWidth() - 40) / 2, bottom_btn_h);
    lv_obj_align(back_btn, LV_ALIGN_TOP_RIGHT, -12, portrait_buttons_y);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x1A2638), 0);
    lv_obj_set_style_radius(back_btn, 14, 0);
    lv_obj_add_event_cb(back_btn, backToMenuEventCb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "Voltar");
    lv_obj_center(back_label);
  }
}

static void createMeteoScreen()
{
  const bool landscape = isLandscapeUI();
  const int32_t top_y = landscape ? 64 : 70;

  g_screen_meteo = lv_obj_create(nullptr);
  styleScreen(g_screen_meteo);
  lv_obj_set_style_bg_color(g_screen_meteo, lv_color_hex(0x07111A), 0);
  lv_obj_set_style_bg_grad_color(g_screen_meteo, lv_color_hex(0x15395A), 0);
  lv_obj_set_style_bg_grad_dir(g_screen_meteo, LV_GRAD_DIR_VER, 0);
  createHeader(g_screen_meteo, "METEO", "tempo e clima");

  lv_obj_t *hero = lv_obj_create(g_screen_meteo);
  lv_obj_clear_flag(hero, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(hero, uiWidth() - 16, landscape ? 110 : 132);
  lv_obj_align(hero, LV_ALIGN_TOP_MID, 0, top_y);
  lv_obj_set_style_bg_color(hero, lv_color_hex(0x174C77), 0);
  lv_obj_set_style_bg_grad_color(hero, lv_color_hex(0x03131E), 0);
  lv_obj_set_style_bg_grad_dir(hero, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_border_color(hero, lv_color_hex(0x6FD9FF), 0);
  lv_obj_set_style_border_width(hero, 2, 0);
  lv_obj_set_style_radius(hero, 18, 0);
  lv_obj_set_style_pad_all(hero, 14, 0);

  g_lbl_meteo_theme = lv_label_create(hero);
  lv_label_set_text(g_lbl_meteo_theme, "Tema atual: ceu limpo / dia\nDepois: dia, noite, chuva, nublado, tempestade");
  lv_label_set_long_mode(g_lbl_meteo_theme, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_meteo_theme, uiWidth() - 48);
  lv_obj_set_style_text_color(g_lbl_meteo_theme, lv_color_hex(0xF2FAFF), 0);
  lv_obj_align(g_lbl_meteo_theme, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *cards = lv_obj_create(g_screen_meteo);
  lv_obj_set_size(cards, uiWidth() - 16, uiHeight() - top_y - (landscape ? 176 : 214));
  lv_obj_align(cards, LV_ALIGN_TOP_MID, 0, top_y + (landscape ? 118 : 140));
  lv_obj_set_style_bg_color(cards, lv_color_hex(0x09131C), 0);
  lv_obj_set_style_border_color(cards, lv_color_hex(0x3CCB9A), 0);
  lv_obj_set_style_border_width(cards, 2, 0);
  lv_obj_set_style_radius(cards, 18, 0);
  lv_obj_set_style_pad_all(cards, 14, 0);

  g_lbl_meteo_cards = lv_label_create(cards);
  lv_label_set_text(
      g_lbl_meteo_cards,
      "TEMP\n--.- C\n\nUMIDADE\n--.- %\n\nPRESSAO\n---- hPa\n\nUV/LUX\n-- / ---\n\nAR\nBME688 pendente");
  lv_label_set_long_mode(g_lbl_meteo_cards, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_meteo_cards, uiWidth() - 48);
  lv_obj_set_style_text_color(g_lbl_meteo_cards, lv_color_hex(0xE9F7F4), 0);
  lv_obj_align(g_lbl_meteo_cards, LV_ALIGN_TOP_LEFT, 0, 0);

  createBackButton(g_screen_meteo, LV_ALIGN_BOTTOM_MID, 0, -10, uiWidth() - 32);
}

static void createCommsScreen()
{
  const bool landscape = isLandscapeUI();
  const int32_t top_y = landscape ? 64 : 70;
  const lv_coord_t card_w = landscape ? (uiWidth() - 36) / 2 : (uiWidth() - 24) / 2;
  const lv_coord_t card_h = landscape ? 86 : 66;

  g_screen_gps = lv_obj_create(nullptr);
  styleScreen(g_screen_gps);
  lv_obj_set_style_bg_color(g_screen_gps, lv_color_hex(0x090C12), 0);
  lv_obj_set_style_bg_grad_color(g_screen_gps, lv_color_hex(0x1D1111), 0);
  lv_obj_set_style_bg_grad_dir(g_screen_gps, LV_GRAD_DIR_VER, 0);
  createHeader(g_screen_gps, "COMMS", "wifi, lora e links");

  lv_obj_t *hero = lv_obj_create(g_screen_gps);
  lv_obj_clear_flag(hero, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(hero, uiWidth() - 16, landscape ? 86 : 70);
  lv_obj_align(hero, LV_ALIGN_TOP_MID, 0, top_y);
  lv_obj_set_style_bg_color(hero, lv_color_hex(0x101722), 0);
  lv_obj_set_style_border_color(hero, lv_color_hex(0xFFB347), 0);
  lv_obj_set_style_border_width(hero, 2, 0);
  lv_obj_set_style_radius(hero, 18, 0);
  lv_obj_set_style_pad_all(hero, 12, 0);

  g_lbl_comms_status = lv_label_create(hero);
  lv_label_set_text(g_lbl_comms_status, "Central de rede e telemetria.");
  lv_label_set_long_mode(g_lbl_comms_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_status, uiWidth() - 44);
  lv_obj_set_style_text_color(g_lbl_comms_status, lv_color_hex(0xFFF2DA), 0);
  lv_obj_align(g_lbl_comms_status, LV_ALIGN_TOP_LEFT, 0, 0);

  g_lbl_comms_mode = lv_label_create(hero);
  lv_label_set_text(g_lbl_comms_mode, "Modo: cockpit normal");
  lv_label_set_long_mode(g_lbl_comms_mode, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_lbl_comms_mode, uiWidth() - 44);
  lv_obj_set_style_text_color(g_lbl_comms_mode, lv_color_hex(0xFFD699), 0);
  lv_obj_align(g_lbl_comms_mode, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  const int32_t cards_y = top_y + (landscape ? 96 : 80);
  if (landscape) {
    createStatusCard(g_screen_gps, card_w, card_h, LV_ALIGN_TOP_LEFT, 8, cards_y, 0xFFB347, "Wi-Fi / Web", &g_lbl_comms_wifi);
    createStatusCard(g_screen_gps, card_w, card_h, LV_ALIGN_TOP_RIGHT, -8, cards_y, 0x63C7FF, "Bluetooth", &g_lbl_comms_ble);
    createStatusCard(g_screen_gps, card_w, card_h, LV_ALIGN_TOP_LEFT, 8, cards_y + card_h + 8, 0xFF7A59, "LoRa", &g_lbl_comms_lora);
    createStatusCard(g_screen_gps, card_w, card_h, LV_ALIGN_TOP_RIGHT, -8, cards_y + card_h + 8, 0x53C2A3, "GPS", &g_lbl_comms_gps);
  } else {
    createStatusCard(g_screen_gps, uiWidth() - 16, 100, LV_ALIGN_TOP_MID, 0, cards_y, 0xFFB347, "Wi-Fi / Web", &g_lbl_comms_wifi);
    createStatusCard(g_screen_gps, card_w, card_h, LV_ALIGN_TOP_LEFT, 8, cards_y + 108, 0xFF7A59, "LoRa", &g_lbl_comms_lora);
    createStatusCard(g_screen_gps, card_w, card_h, LV_ALIGN_TOP_RIGHT, -8, cards_y + 108, 0x53C2A3, "GPS", &g_lbl_comms_gps);
    g_lbl_comms_ble = nullptr;
  }

  const lv_coord_t button_w = landscape ? (uiWidth() - 40) / 3 : (uiWidth() - 32) / 3;
  const lv_coord_t button_h = 42;
  const lv_coord_t button_y = landscape ? (cards_y + (card_h * 2) + 16) : (cards_y + 182);

  createActionButton(g_screen_gps, button_w, button_h, LV_ALIGN_TOP_LEFT, 8, button_y, 0x285C92, "Wi-Fi AP", toggleWifiEventCb, &g_lbl_comms_wifi_btn);
  createActionButton(g_screen_gps, button_w, button_h, LV_ALIGN_TOP_MID, 0, button_y, 0x8B3E2A, "LoRa", toggleLoraEventCb, &g_lbl_comms_lora_btn);
  createActionButton(g_screen_gps, button_w, button_h, LV_ALIGN_TOP_RIGHT, -8, button_y, 0x2A5E4C, "Rescan", rescanCommsEventCb, &g_lbl_comms_rescan_btn);

  createBackButton(g_screen_gps, LV_ALIGN_BOTTOM_MID, 0, -10, uiWidth() - 32);
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
  lv_label_set_text(g_lbl_home_hint, "IMU -- | SD -- | WEB off");
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
    lv_label_set_text(g_lbl_plane_altitude, "---- ft");
  }

  if (g_lbl_plane_vario) {
    lv_label_set_text(g_lbl_plane_vario, "---- fpm");
  }

  if (g_lbl_plane_heading) {
    lv_label_set_text(g_lbl_plane_heading, "--- dg");
  }

  if (g_lbl_plane_speed) {
    lv_label_set_text(g_lbl_plane_speed, "-- km/h");
  }

  if (g_lbl_plane_skydive) {
    lv_label_set_text(g_lbl_plane_skydive, "SKYDIVE em breve");
  }

  if (g_lbl_efis_status) {
    if (!imu.connected) {
      lv_label_set_text(g_lbl_efis_status, "IMU OFF");
    } else if (!imu.healthy) {
      lv_label_set_text(g_lbl_efis_status, "IMU WARN");
    } else if (g_runtime_services_light) {
      lv_label_set_text_fmt(
          g_lbl_efis_status,
          "IMU OK | Wi-Fi ativo | trim %.1f/%.1f",
          pitch_trim_deg,
          roll_trim_deg);
    } else if (!imu.has_solution) {
      lv_label_set_text(g_lbl_efis_status, "IMU alinhando");
    } else if (!g_lbl_efis_pitch && !g_lbl_efis_roll) {
      lv_label_set_text_fmt(
          g_lbl_efis_status,
          "P %+.1f  R %+.1f  | IMU OK",
          display_imu.pitch_deg,
          display_imu.roll_deg);
    } else {
      lv_label_set_text_fmt(
          g_lbl_efis_status,
          "IMU OK | %.1f C | trim %.1f/%.1f",
          imu.temperature_c,
          pitch_trim_deg,
          roll_trim_deg);
    }
  }

  if (g_lbl_efis_calibration) {
    lv_label_set_text(g_lbl_efis_calibration, "Setar nivel");
  }
}

static void refreshConfigUI()
{
  const BlackboxState blackbox = copyBlackboxState();
  char ip_text[20];
  formatWifiVisibleIp(ip_text, sizeof(ip_text));

  if (g_sd_purge_armed && (millis() - g_sd_purge_arm_ms) > SD_PURGE_CONFIRM_MS) {
    g_sd_purge_armed = false;
  }

  if (g_lbl_config_orientation) {
    lv_label_set_text(g_lbl_config_orientation, "Modo: Vertical");
  }

  if (g_lbl_config_rotation) {
    lv_label_set_text_fmt(g_lbl_config_rotation, "Ativa: %s", rotationLabel(g_display_rotation));
  }

  if (g_lbl_config_net) {
    char net_text[256];
    if (isLandscapeUI()) {
      snprintf(
          net_text,
          sizeof(net_text),
          "WiFi AP: %s\nSSID: %s\nSenha: %s\nIP/URL: http://%s\nClientes: %u\nLoRa: %s\n%s",
          wifiStateLabel(),
          WIFI_AP_SSID,
          WIFI_AP_PASSWORD,
          ip_text,
          static_cast<unsigned>(wifiClientCount()),
          g_lora_enabled ? "ON" : "OFF",
          g_wifi_diag_note);
    } else {
      snprintf(
          net_text,
          sizeof(net_text),
          "WiFi AP: %s\nSSID: %s\nSenha: %s\nIP: %s\nClientes: %u",
          wifiStateLabel(),
          WIFI_AP_SSID,
          WIFI_AP_PASSWORD,
          ip_text,
          static_cast<unsigned>(wifiClientCount()));
    }
    lv_label_set_text(g_lbl_config_net, net_text);
  }

  if (g_lbl_config_sensors) {
    lv_label_set_text(g_lbl_config_sensors, g_sensor_compact_cache.c_str());
  }

  if (g_lbl_config_note) {
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
          "SD para logs e caixa-preta\nWiFi/LoRa em COMMS ou CONFIG\nBOOT = Home");
    }
    lv_label_set_text(g_lbl_config_note, note_text);
  }

  if (g_lbl_config_blackbox) {
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
        g_runtime_services_light ? "\nPausado pelo modo Wi-Fi" : "");
    lv_label_set_text(g_lbl_config_blackbox, status_text);
  }

  if (g_lbl_config_scan_raw) {
    lv_label_set_text_fmt(g_lbl_config_scan_raw, "I2C %s", g_sensor_scan_compact_cache.c_str());
  }

  if (g_lbl_config_blackbox_btn) {
    lv_label_set_text(
        g_lbl_config_blackbox_btn,
        blackbox.logging_enabled ? "Pausar" : "Ligar");
  }

  if (g_lbl_config_format_btn) {
    lv_label_set_text(g_lbl_config_format_btn, "Limpar SD");
  }

  if (g_lbl_config_wifi_btn) {
    lv_label_set_text(g_lbl_config_wifi_btn, g_wifi_mode_requested ? "Desligar Wi-Fi" : "Ligar Wi-Fi");
  }

  if (g_lbl_config_lora_btn) {
    lv_label_set_text(g_lbl_config_lora_btn, g_lora_enabled ? "Desligar LoRa" : "Ligar LoRa");
  }

  if (g_lbl_home_hint) {
    char home_text[160];
    snprintf(
        home_text,
        sizeof(home_text),
        "IMU %s | SD %s\nWiFi %s @ %s | LoRa %s",
        g_imu_state.connected ? (g_imu_state.healthy ? "OK" : "WARN") : "--",
        blackbox.mounted ? "OK" : "--",
        wifiStateLabel(),
        ip_text,
        g_lora_enabled ? "on" : "off");
    lv_label_set_text(g_lbl_home_hint, home_text);
  }

  if (g_lbl_meteo_theme) {
    const uint32_t hours = (millis() / 1000UL / 15UL) % 4UL;
    switch (hours) {
      case 0:
        lv_label_set_text(g_lbl_meteo_theme, "Tema atual: dia claro\nDepois: vincular a sensores, GPS ou web");
        break;
      case 1:
        lv_label_set_text(g_lbl_meteo_theme, "Tema atual: nublado\nDepois: fundo reage ao tempo real");
        break;
      case 2:
        lv_label_set_text(g_lbl_meteo_theme, "Tema atual: chuva\nDepois: animacao leve de clima");
        break;
      default:
        lv_label_set_text(g_lbl_meteo_theme, "Tema atual: noite\nDepois: dia/noite por hora real");
        break;
    }
  }

  if (g_lbl_meteo_cards) {
    lv_label_set_text(
        g_lbl_meteo_cards,
        "TEMP  --.- C\nUMIDADE  --.- %\nPRESSAO / ALT  ---- hPa / ---- m\nUV / LUX  -- / ---\nAR / BATERIA  IAQ pendente / -- %");
  }

  if (g_lbl_comms_wifi) {
    char wifi_text[192];
    if (isLandscapeUI()) {
      snprintf(
          wifi_text,
          sizeof(wifi_text),
          "AP: %s\nSSID: %s\nSenha: %s\nURL: http://%s\nCli: %u | Hits: %lu",
          wifiStateLabel(),
          WIFI_AP_SSID,
          WIFI_AP_PASSWORD,
          ip_text,
          static_cast<unsigned>(wifiClientCount()),
          static_cast<unsigned long>(g_wifi_web_hits));
    } else {
      snprintf(
          wifi_text,
          sizeof(wifi_text),
          "AP: %s\nSSID: %s\nSenha: %s\nIP: %s",
          wifiStateLabel(),
          WIFI_AP_SSID,
          WIFI_AP_PASSWORD,
          ip_text,
          static_cast<unsigned>(wifiClientCount()));
    }
    lv_label_set_text(g_lbl_comms_wifi, wifi_text);
  }

  if (g_lbl_comms_mode) {
    lv_label_set_text(
        g_lbl_comms_mode,
        g_runtime_services_light ? "Modo: Wi-Fi ativo, cockpit em modo leve" : "Modo: cockpit normal");
  }

  if (g_lbl_comms_wifi_btn) {
    lv_label_set_text(
        g_lbl_comms_wifi_btn,
        isLandscapeUI()
            ? (g_wifi_mode_requested ? "Desligar Wi-Fi AP" : "Ligar Wi-Fi AP")
            : "Wi-Fi");
  }

  if (g_lbl_comms_lora_btn) {
    lv_label_set_text(
        g_lbl_comms_lora_btn,
        isLandscapeUI()
            ? (g_lora_enabled ? "Desligar LoRa" : "Ligar LoRa")
            : "LoRa");
  }

  if (g_lbl_comms_rescan_btn) {
    lv_label_set_text(g_lbl_comms_rescan_btn, isLandscapeUI() ? "Rescan SD/I2C" : "Scan");
  }

  if (g_lbl_comms_ble) {
    lv_label_set_text(g_lbl_comms_ble, "BLE: pendente | foco atual = Wi-Fi AP");
  }

  if (g_lbl_comms_lora) {
    lv_label_set_text_fmt(
        g_lbl_comms_lora,
        "Estado: %s\nTX: -- | RX: --\nRSSI: -- | SNR: --",
        g_lora_enabled ? "ON" : "OFF");
  }

  if (g_lbl_comms_gps) {
    lv_label_set_text(g_lbl_comms_gps, "Fix: --\nSats: --\nSpeed: --\nAlt: --");
  }

  if (g_lbl_comms_status) {
    lv_label_set_text(
        g_lbl_comms_status,
        isLandscapeUI()
            ? (g_wifi_ap_started
                   ? "AP ativo. Veja SSID, senha e IP abaixo, abra 192.168.4.1 e acompanhe o serial."
                   : "Ligue o AP aqui. O sistema entra em modo leve enquanto o Wi-Fi estiver ativo.")
            : (g_wifi_ap_started ? "AP ativo. Abra 192.168.4.1" : "Ligue o AP para entrar no modo leve."));
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
      4096,
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
      6144,
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

  initDisplay();
  initTouch();
  initLVGL();
  createUI(SCREEN_HOME);
  startImuTask();
  startBlackboxTask();
  g_wifi_stack_ready = false;
  g_wifi_mode_requested = false;
  g_wifi_ap_started = false;
  g_wifi_web_started = false;
  g_wifi_web_hits = 0;
  g_wifi_last_client_ms = 0;
  g_wifi_last_status_ms = 0;
  updateRuntimeServiceMode();
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
  Serial.println("[WIFI] AP integrado pronto. Ligue em COMMS > Wi-Fi AP");
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
  const uint32_t now = millis();

  handleBootButton();
  updateWifiLed();

  if (now - last_uptime >= 1000U) {
    last_uptime = now;
    const ImuState imu = copyImuState();

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
  }

  if (now - last_touch_ui >= 40U) {
    last_touch_ui = now;
    refreshTouchUI();
  }

  if (now - last_efis_ui >= 100U) {
    last_efis_ui = now;
    refreshEfisUI();
  }

  if (now - last_config_ui >= 250U) {
    last_config_ui = now;
    refreshConfigUI();
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
