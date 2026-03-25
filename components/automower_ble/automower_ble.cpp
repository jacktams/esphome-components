#include "automower_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include "esp_random.h"
#endif

namespace esphome {
namespace automower_ble {

// BLE UUIDs for the Husqvarna Automower protocol
static const char *SERVICE_UUID_STR = "98bd0001-0b0e-421a-84e5-ddbf75dc6de4";
static const char *WRITE_CHAR_UUID_STR = "98bd0002-0b0e-421a-84e5-ddbf75dc6de4";
static const char *NOTIFY_CHAR_UUID_STR = "98bd0003-0b0e-421a-84e5-ddbf75dc6de4";

// Protocol command major/minor codes
static constexpr uint16_t MAJOR_BATTERY = 4106;
static constexpr uint8_t MINOR_GET_BATTERY_LEVEL = 20;
static constexpr uint8_t MINOR_IS_CHARGING = 21;

static constexpr uint16_t MAJOR_MOWER = 4586;
static constexpr uint8_t MINOR_SET_MODE = 0;
static constexpr uint8_t MINOR_GET_STATE = 2;
static constexpr uint8_t MINOR_GET_ACTIVITY = 3;
static constexpr uint8_t MINOR_START_TRIGGER = 4;
static constexpr uint8_t MINOR_PAUSE = 5;
static constexpr uint8_t MINOR_GET_ERROR = 6;

static constexpr uint16_t MAJOR_OVERRIDE = 4658;
static constexpr uint8_t MINOR_SET_OVERRIDE_MOW = 3;
static constexpr uint8_t MINOR_PARK_UNTIL_NEXT = 5;

static constexpr uint16_t MAJOR_AUTH = 4664;
static constexpr uint8_t MINOR_ENTER_PIN = 4;

static constexpr uint16_t MAJOR_KEEPALIVE = 4674;
static constexpr uint8_t MINOR_KEEPALIVE = 2;

// ---- CRC ----

uint8_t AutomowerBLE::crc8(const uint8_t *data, uint8_t offset, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) {
    checksum = CRC8_TABLE[checksum ^ data[offset + i]];
  }
  return checksum;
}

uint8_t AutomowerBLE::crc8(const std::vector<uint8_t> &data, uint8_t offset, uint8_t length) {
  return crc8(data.data(), offset, length);
}

// ---- Packet builders ----

std::vector<uint8_t> AutomowerBLE::build_setup_channel_id_() {
  // Template from protocol: "02fd160000000000002e1400000000000000004d61696e00"
  std::vector<uint8_t> data = {
      0x02, 0xFD, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x2E, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x4D, 0x61, 0x69, 0x6E, 0x00};

  // Insert channel ID at bytes 11-14
  data[11] = (this->channel_id_ >> 0) & 0xFF;
  data[12] = (this->channel_id_ >> 8) & 0xFF;
  data[13] = (this->channel_id_ >> 16) & 0xFF;
  data[14] = (this->channel_id_ >> 24) & 0xFF;

  // Header CRC: bytes 1..8
  data[9] = crc8(data, 1, 8);

  // Payload CRC: bytes 1..(len-1)
  uint8_t payload_crc = crc8(data, 1, data.size() - 1);
  data.push_back(payload_crc);
  data.push_back(0x03);

  return data;
}

std::vector<uint8_t> AutomowerBLE::build_handshake_() {
  // Template from protocol: "02fd0a000000000000d00801"
  std::vector<uint8_t> data = {
      0x02, 0xFD, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0xD0, 0x08, 0x01};

  // Insert channel ID at bytes 4-7
  data[4] = (this->channel_id_ >> 0) & 0xFF;
  data[5] = (this->channel_id_ >> 8) & 0xFF;
  data[6] = (this->channel_id_ >> 16) & 0xFF;
  data[7] = (this->channel_id_ >> 24) & 0xFF;

  // Header CRC: bytes 1..8
  data[9] = crc8(data, 1, 8);

  // Payload CRC: bytes 1..(len-1)
  uint8_t payload_crc = crc8(data, 1, data.size() - 1);
  data.push_back(payload_crc);
  data.push_back(0x03);

  return data;
}

std::vector<uint8_t> AutomowerBLE::build_command_(uint16_t major, uint8_t minor,
                                                    const std::vector<uint8_t> &payload) {
  // Build an 18-byte base packet + optional payload
  std::vector<uint8_t> data(18, 0);

  data[0] = 0x02;          // Start marker
  data[1] = 0xFD;          // Linked type
  // data[2..3] = length (set below)
  data[4] = (this->channel_id_ >> 0) & 0xFF;
  data[5] = (this->channel_id_ >> 8) & 0xFF;
  data[6] = (this->channel_id_ >> 16) & 0xFF;
  data[7] = (this->channel_id_ >> 24) & 0xFF;
  data[8] = 0x01;          // is_linked
  // data[9] = CRC (set below)
  data[10] = 0x00;         // Packet type: request
  data[11] = 0xAF;         // Constant
  data[12] = major & 0xFF;
  data[13] = (major >> 8) & 0xFF;
  data[14] = minor;
  data[15] = 0x00;         // High byte
  data[16] = payload.size();  // Request data length (low)
  data[17] = 0x00;         // Request data length (high)

  // Append payload if present
  if (!payload.empty()) {
    data.insert(data.end(), payload.begin(), payload.end());
  }

  // Set length: total bytes after first 2 (0x02, 0xFD), not counting trailing CRC + 0x03
  data[2] = (data.size() - 2) & 0xFF;
  data[3] = ((data.size() - 2) >> 8) & 0xFF;

  // Header CRC: bytes 1..8
  data[9] = crc8(data, 1, 8);

  // Payload CRC: bytes 1..(len-1)
  uint8_t payload_crc = crc8(data, 1, data.size() - 1);
  data.push_back(payload_crc);
  data.push_back(0x03);

  return data;
}

// ---- BLE operations ----

void AutomowerBLE::write_data_(const std::vector<uint8_t> &data) {
  if (this->write_handle_ == 0) {
    ESP_LOGE(TAG, "Write handle not set");
    return;
  }

  // Log the full packet for debugging
  std::string hex;
  for (size_t i = 0; i < data.size(); i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", data[i]);
    hex += buf;
  }
  ESP_LOGI(TAG, "Writing %d bytes: %s", data.size(), hex.c_str());

  // Send full packet in one write — ESP32 MTU is negotiated (typically 65+)
  // so no need for application-level fragmentation
  auto status = esp_ble_gattc_write_char(
      this->parent()->get_gattc_if(),
      this->parent()->get_conn_id(),
      this->write_handle_,
      data.size(),
      const_cast<uint8_t *>(data.data()),
      ESP_GATT_WRITE_TYPE_NO_RSP,
      ESP_GATT_AUTH_REQ_NONE);

  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Write failed: %d", status);
  }
}

void AutomowerBLE::subscribe_notifications_() {
  if (this->notify_handle_ == 0) {
    ESP_LOGE(TAG, "Notify handle not set");
    this->state_ = ConnectionState::ERROR;
    return;
  }

  // Register local notification callback
  auto status = esp_ble_gattc_register_for_notify(
      this->parent()->get_gattc_if(),
      this->parent()->get_remote_bda(),
      this->notify_handle_);

  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Register for notify failed: %d", status);
    this->state_ = ConnectionState::ERROR;
    return;
  }

  // Explicitly write the CCCD descriptor to enable notifications on the REMOTE device.
  // esp_ble_gattc_register_for_notify() only registers the local callback —
  // without this write, the mower doesn't know to send notifications.
  // CCCD handle is typically notify_handle + 1 (standard BLE layout).
  uint16_t cccd_handle = this->notify_handle_ + 1;
  uint8_t cccd_value[2] = {0x01, 0x00};  // 0x0001 = notifications enabled
  auto cccd_status = esp_ble_gattc_write_char_descr(
      this->parent()->get_gattc_if(),
      this->parent()->get_conn_id(),
      cccd_handle,
      sizeof(cccd_value),
      cccd_value,
      ESP_GATT_WRITE_TYPE_RSP,
      ESP_GATT_AUTH_REQ_NONE);

  if (cccd_status != ESP_OK) {
    ESP_LOGE(TAG, "CCCD write failed: %d", cccd_status);
  } else {
    ESP_LOGI(TAG, "CCCD write sent to handle 0x%04X", cccd_handle);
  }
}

// ---- Component lifecycle ----

void AutomowerBLE::setup() {
  this->channel_id_ = esp_random();
  ESP_LOGI(TAG, "Channel ID: 0x%08X", this->channel_id_);

  // Clear all existing bonds to avoid stale bond conflicts
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num > 0) {
    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (dev_list != nullptr) {
      esp_ble_get_bond_device_list(&dev_num, dev_list);
      for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
        ESP_LOGI(TAG, "Removed stored bond %d", i);
      }
      free(dev_list);
    }
  }

  // Configure BLE security for PIN-based MITM pairing
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t io_cap = ESP_IO_CAP_OUT;  // "Display" capability — provides static passkey
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  // Set the PIN as a static passkey for BLE pairing
  uint32_t passkey = this->pin_;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(passkey));

  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &io_cap, sizeof(io_cap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));

  ESP_LOGI(TAG, "BLE security configured (MITM + Bond, PIN: %lu)", passkey);
}

void AutomowerBLE::dump_config() {
  ESP_LOGCONFIG(TAG, "Automower BLE:");
  ESP_LOGCONFIG(TAG, "  PIN: %s", this->pin_ > 0 ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Channel ID: 0x%08X", this->channel_id_);
}

void AutomowerBLE::loop() {
  // Timeout check for pending responses
  if (this->pending_command_ != CommandType::NONE &&
      millis() - this->command_sent_at_ > RESPONSE_TIMEOUT_MS) {
    ESP_LOGW(TAG, "Response timeout for command %d", (int) this->pending_command_);
    this->pending_command_ = CommandType::NONE;
    this->response_buffer_.clear();

    // If we were in the handshake sequence, retry from the beginning
    if (this->state_ == ConnectionState::WAITING_CHANNEL_RESPONSE ||
        this->state_ == ConnectionState::WAITING_HANDSHAKE_RESPONSE ||
        this->state_ == ConnectionState::WAITING_PIN_RESPONSE) {
      ESP_LOGW(TAG, "Handshake timed out, retrying");
      this->state_ = ConnectionState::SETUP_CHANNEL;
    } else {
      this->state_ = ConnectionState::READY;
    }
  }

  // State machine transitions that need to send data
  switch (this->state_) {
    case ConnectionState::PAIRING: {
      ESP_LOGI(TAG, "Requesting BLE encryption (MITM with PIN)");
      this->state_ = ConnectionState::WAITING_PAIRING;
      this->command_sent_at_ = millis();
      auto status = esp_ble_set_encryption(this->parent()->get_remote_bda(), ESP_BLE_SEC_ENCRYPT_MITM);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_set_encryption returned %d, trying to subscribe anyway", status);
        this->state_ = ConnectionState::SUBSCRIBING;
      }
      break;
    }

    case ConnectionState::WAITING_PAIRING:
      // Timeout after 15s — proceed to subscribe even if pairing didn't complete
      if (millis() - this->command_sent_at_ > 15000) {
        ESP_LOGW(TAG, "Pairing timeout, subscribing anyway");
        this->state_ = ConnectionState::SUBSCRIBING;
      }
      break;

    case ConnectionState::SUBSCRIBING:
      // Move to CONNECTED first to prevent re-entry; subscribe_notifications_
      // may overwrite to ERROR if it fails
      this->state_ = ConnectionState::CONNECTED;
      this->subscribe_notifications_();
      break;

    case ConnectionState::SETUP_CHANNEL: {
      ESP_LOGI(TAG, "Sending channel setup");
      auto pkt = this->build_setup_channel_id_();
      this->response_buffer_.clear();
      this->pending_command_ = CommandType::SETUP_CHANNEL;
      this->command_sent_at_ = millis();
      this->write_data_(pkt);
      this->state_ = ConnectionState::WAITING_CHANNEL_RESPONSE;
      break;
    }

    case ConnectionState::HANDSHAKE: {
      ESP_LOGI(TAG, "Sending handshake");
      auto pkt = this->build_handshake_();
      this->response_buffer_.clear();
      this->pending_command_ = CommandType::HANDSHAKE;
      this->command_sent_at_ = millis();
      this->write_data_(pkt);
      this->state_ = ConnectionState::WAITING_HANDSHAKE_RESPONSE;
      break;
    }

    case ConnectionState::AUTH_PIN: {
      ESP_LOGI(TAG, "Sending PIN authentication");
      std::vector<uint8_t> payload = {
          static_cast<uint8_t>(this->pin_ & 0xFF),
          static_cast<uint8_t>((this->pin_ >> 8) & 0xFF)};
      auto pkt = this->build_command_(MAJOR_AUTH, MINOR_ENTER_PIN, payload);
      this->response_buffer_.clear();
      this->pending_command_ = CommandType::ENTER_PIN;
      this->command_sent_at_ = millis();
      this->write_data_(pkt);
      this->state_ = ConnectionState::WAITING_PIN_RESPONSE;
      break;
    }

    case ConnectionState::READY:
      // Send keepalive if needed
      if (millis() - this->last_keepalive_ > KEEPALIVE_INTERVAL_MS) {
        this->queue_command(CommandType::KEEPALIVE);
        this->last_keepalive_ = millis();
      }
      // Process command queue
      this->send_next_command_();
      break;

    default:
      break;
  }
}

void AutomowerBLE::update() {
  if (this->state_ != ConnectionState::READY)
    return;

  // Queue sensor polling commands
#ifdef USE_SENSOR
  if (this->battery_sensor_ != nullptr)
    this->queue_command(CommandType::GET_BATTERY);
#endif
#ifdef USE_BINARY_SENSOR
  if (this->charging_sensor_ != nullptr)
    this->queue_command(CommandType::IS_CHARGING);
#endif
#ifdef USE_TEXT_SENSOR
  if (this->state_sensor_ != nullptr)
    this->queue_command(CommandType::GET_STATE);
  if (this->activity_sensor_ != nullptr)
    this->queue_command(CommandType::GET_ACTIVITY);
  if (this->error_sensor_ != nullptr)
    this->queue_command(CommandType::GET_ERROR);
#endif
}

// ---- GATT event handler ----

void AutomowerBLE::gattc_event_handler(esp_gattc_cb_event_t event,
                                         esp_gatt_if_t gattc_if,
                                         esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Connection failed: %d", param->open.status);
        this->state_ = ConnectionState::IDLE;
        return;
      }
      ESP_LOGI(TAG, "Connected to mower");
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // Find our characteristics
      auto service_uuid = esp32_ble_tracker::ESPBTUUID::from_raw(std::string(SERVICE_UUID_STR));
      auto write_uuid = esp32_ble_tracker::ESPBTUUID::from_raw(std::string(WRITE_CHAR_UUID_STR));
      auto notify_uuid = esp32_ble_tracker::ESPBTUUID::from_raw(std::string(NOTIFY_CHAR_UUID_STR));

      auto *write_chr = this->parent()->get_characteristic(service_uuid, write_uuid);
      if (write_chr == nullptr) {
        ESP_LOGE(TAG, "Write characteristic not found (98bd0002)");
        this->state_ = ConnectionState::ERROR;
        return;
      }
      this->write_handle_ = write_chr->handle;
      ESP_LOGI(TAG, "Write handle: 0x%04X", this->write_handle_);

      auto *notify_chr = this->parent()->get_characteristic(service_uuid, notify_uuid);
      if (notify_chr == nullptr) {
        ESP_LOGE(TAG, "Notify characteristic not found (98bd0003)");
        this->state_ = ConnectionState::ERROR;
        return;
      }
      this->notify_handle_ = notify_chr->handle;
      ESP_LOGI(TAG, "Notify handle: 0x%04X", this->notify_handle_);

      // Probe readable characteristics (the Python code reads all chars before subscribing)
      auto device_uuid = esp32_ble_tracker::ESPBTUUID::from_raw(
          std::string("98bd0004-0b0e-421a-84e5-ddbf75dc6de4"));
      auto *device_chr = this->parent()->get_characteristic(service_uuid, device_uuid);
      if (device_chr != nullptr) {
        ESP_LOGI(TAG, "Reading device type characteristic (98bd0004)");
        esp_ble_gattc_read_char(this->parent()->get_gattc_if(),
                                 this->parent()->get_conn_id(),
                                 device_chr->handle, ESP_GATT_AUTH_REQ_NONE);
      }

      // Encrypt FIRST, then subscribe — the CCCD write for notifications
      // requires an encrypted link on this mower
      ESP_LOGI(TAG, "Initiating BLE encryption before subscribing");
      this->state_ = ConnectionState::PAIRING;
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "Notification registration failed: %d", param->reg_for_notify.status);
        this->state_ = ConnectionState::ERROR;
        return;
      }
      ESP_LOGI(TAG, "Notifications enabled on encrypted link, waiting 5s");

      // Wait 5s for the mower to settle before protocol handshake
      this->set_timeout("setup_delay", 5000, [this]() {
        if (this->state_ == ConnectionState::CONNECTED) {
          ESP_LOGI(TAG, "Delay complete, starting channel setup");
          this->state_ = ConnectionState::SETUP_CHANNEL;
        }
      });
      this->state_ = ConnectionState::CONNECTED;  // park here until timeout fires
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.status == ESP_GATT_OK) {
        std::string hex;
        for (uint16_t i = 0; i < param->read.value_len && i < 32; i++) {
          char buf[4];
          snprintf(buf, sizeof(buf), "%02X ", param->read.value[i]);
          hex += buf;
        }
        ESP_LOGI(TAG, "Characteristic read (handle 0x%04X): %d bytes: %s",
                 param->read.handle, param->read.value_len, hex.c_str());
      } else {
        ESP_LOGW(TAG, "Characteristic read failed (handle 0x%04X, status: %d)",
                 param->read.handle, param->read.status);
      }
      break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
      if (param->write.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "CCCD descriptor write confirmed (handle 0x%04X)", param->write.handle);
      } else {
        ESP_LOGE(TAG, "CCCD descriptor write FAILED (handle 0x%04X, status: %d)",
                 param->write.handle, param->write.status);
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != this->notify_handle_)
        break;

      this->handle_notification_(param->notify.value, param->notify.value_len);
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "Disconnected from mower (reason: 0x%02X)", param->disconnect.reason);
      this->state_ = ConnectionState::IDLE;
      this->write_handle_ = 0;
      this->notify_handle_ = 0;
      this->pending_command_ = CommandType::NONE;
      this->response_buffer_.clear();
      // Clear command queue
      while (!this->command_queue_.empty())
        this->command_queue_.pop();
#ifdef USE_BINARY_SENSOR
      if (this->connected_sensor_ != nullptr)
        this->connected_sensor_->publish_state(false);
#endif
      break;
    }

    default:
      break;
  }
}

// ---- GAP event handler (BLE pairing) ----

void AutomowerBLE::gap_event_handler(esp_gap_ble_cb_event_t event,
                                       esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_SEC_REQ_EVT:
      ESP_LOGI(TAG, "Security request from mower, accepting");
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;

    case ESP_GAP_BLE_NC_REQ_EVT:
      ESP_LOGI(TAG, "Numeric comparison request: %lu", param->ble_security.key_notif.passkey);
      esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
      break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
      ESP_LOGI(TAG, "Passkey request from mower, replying with PIN: %d", this->pin_);
      esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, this->pin_);
      break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      auto &auth = param->ble_security.auth_cmpl;
      if (auth.success) {
        ESP_LOGI(TAG, "BLE encryption established (auth_mode: %d)", auth.auth_mode);
        // Now that the link is encrypted, subscribe to notifications
        if (this->state_ == ConnectionState::WAITING_PAIRING) {
          this->state_ = ConnectionState::SUBSCRIBING;
        }
      } else {
        ESP_LOGW(TAG, "BLE pairing failed (reason: 0x%X), clearing bond and retrying", auth.fail_reason);
        esp_ble_remove_bond_device(auth.bd_addr);
        // Retry after a short delay
        if (this->state_ == ConnectionState::WAITING_PAIRING) {
          this->set_timeout("pairing_retry", 2000, [this]() {
            if (this->state_ == ConnectionState::WAITING_PAIRING) {
              ESP_LOGI(TAG, "Retrying encryption");
              esp_ble_set_encryption(this->parent()->get_remote_bda(), ESP_BLE_SEC_ENCRYPT_MITM);
            }
          });
        }
      }
      break;
    }

    default:
      break;
  }
}

// ---- Notification / response handling ----

void AutomowerBLE::handle_notification_(const uint8_t *data, uint16_t length) {
  // Log at INFO level during debugging
  std::string hex;
  for (uint16_t i = 0; i < length && i < 30; i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", data[i]);
    hex += buf;
  }
  ESP_LOGI(TAG, "Notification received: %d bytes: %s%s", length,
           hex.c_str(), length > 30 ? "..." : "");

  // Append to response buffer
  this->response_buffer_.insert(this->response_buffer_.end(), data, data + length);

  // Need at least 3 bytes to read the length field
  if (this->response_buffer_.size() < 3)
    return;

  // Total expected = length_field + 4 (start + type + trailing CRC + terminator)
  uint16_t expected = this->response_buffer_[2] + 4;

  if (this->response_buffer_.size() >= expected) {
    ESP_LOGD(TAG, "Complete response: %d bytes", this->response_buffer_.size());
    this->process_response_();
    this->response_buffer_.clear();
  }
}

void AutomowerBLE::process_response_() {
  auto &data = this->response_buffer_;

  // Log the response for debugging
  std::string hex;
  for (size_t i = 0; i < data.size() && i < 30; i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", data[i]);
    hex += buf;
  }
  ESP_LOGD(TAG, "Response [cmd=%d]: %s%s", (int) this->pending_command_,
           hex.c_str(), data.size() > 30 ? "..." : "");

  switch (this->pending_command_) {
    case CommandType::SETUP_CHANNEL:
      ESP_LOGI(TAG, "Channel setup response received");
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::HANDSHAKE;
      break;

    case CommandType::HANDSHAKE:
      ESP_LOGI(TAG, "Handshake response received");
      this->pending_command_ = CommandType::NONE;
      if (this->pin_ > 0) {
        this->state_ = ConnectionState::AUTH_PIN;
      } else {
        ESP_LOGI(TAG, "Mower ready (no PIN)");
        this->state_ = ConnectionState::READY;
        this->last_keepalive_ = millis();
#ifdef USE_BINARY_SENSOR
        if (this->connected_sensor_ != nullptr)
          this->connected_sensor_->publish_state(true);
#endif
      }
      break;

    case CommandType::ENTER_PIN:
      if (data.size() > 16 && data[16] == 0) {
        ESP_LOGI(TAG, "PIN accepted, mower ready");
        this->state_ = ConnectionState::READY;
        this->last_keepalive_ = millis();
#ifdef USE_BINARY_SENSOR
        if (this->connected_sensor_ != nullptr)
          this->connected_sensor_->publish_state(true);
#endif
      } else {
        uint8_t result = data.size() > 16 ? data[16] : 0xFF;
        ESP_LOGE(TAG, "PIN rejected (result: %d)", result);
        this->state_ = ConnectionState::ERROR;
      }
      this->pending_command_ = CommandType::NONE;
      break;

    case CommandType::GET_BATTERY:
#ifdef USE_SENSOR
      if (data.size() > 19 && data[16] == 0 && this->battery_sensor_ != nullptr) {
        uint8_t level = data[19];
        ESP_LOGD(TAG, "Battery: %d%%", level);
        this->battery_sensor_->publish_state(level);
      }
#endif
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;

    case CommandType::IS_CHARGING:
#ifdef USE_BINARY_SENSOR
      if (data.size() > 19 && data[16] == 0 && this->charging_sensor_ != nullptr) {
        bool charging = data[19] != 0;
        ESP_LOGD(TAG, "Charging: %s", YESNO(charging));
        this->charging_sensor_->publish_state(charging);
      }
#endif
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;

    case CommandType::GET_STATE:
#ifdef USE_TEXT_SENSOR
      if (data.size() > 19 && data[16] == 0 && this->state_sensor_ != nullptr) {
        uint8_t state = data[19];
        ESP_LOGD(TAG, "State: %d (%s)", state, this->state_to_string_(state));
        this->state_sensor_->publish_state(this->state_to_string_(state));
      }
#endif
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;

    case CommandType::GET_ACTIVITY:
#ifdef USE_TEXT_SENSOR
      if (data.size() > 19 && data[16] == 0 && this->activity_sensor_ != nullptr) {
        uint8_t activity = data[19];
        ESP_LOGD(TAG, "Activity: %d (%s)", activity, this->activity_to_string_(activity));
        this->activity_sensor_->publish_state(this->activity_to_string_(activity));
      }
#endif
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;

    case CommandType::GET_ERROR:
#ifdef USE_TEXT_SENSOR
      if (data.size() > 22 && data[16] == 0 && this->error_sensor_ != nullptr) {
        uint32_t error = data[19] | (data[20] << 8) | (data[21] << 16) | (data[22] << 24);
        if (error == 0) {
          this->error_sensor_->publish_state("none");
        } else {
          char buf[16];
          snprintf(buf, sizeof(buf), "%u", error);
          this->error_sensor_->publish_state(buf);
        }
      }
#endif
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;

    case CommandType::KEEPALIVE:
      ESP_LOGV(TAG, "Keepalive acknowledged");
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;

    case CommandType::START_TRIGGER:
    case CommandType::PAUSE:
    case CommandType::PARK_UNTIL_NEXT:
    case CommandType::SET_MODE_AUTO:
    case CommandType::SET_OVERRIDE_MOW: {
      uint8_t result = data.size() > 16 ? data[16] : 0xFF;
      if (result == 0) {
        ESP_LOGI(TAG, "Command %d executed successfully", (int) this->pending_command_);
      } else {
        ESP_LOGW(TAG, "Command %d failed (result: %d)", (int) this->pending_command_, result);
      }
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;
    }

    default:
      ESP_LOGW(TAG, "Unexpected response (pending: %d)", (int) this->pending_command_);
      this->pending_command_ = CommandType::NONE;
      this->state_ = ConnectionState::READY;
      break;
  }
}

// ---- Command queue ----

void AutomowerBLE::queue_command(CommandType type) {
  this->command_queue_.push(type);
}

void AutomowerBLE::send_next_command_() {
  if (this->pending_command_ != CommandType::NONE)
    return;
  if (this->command_queue_.empty())
    return;

  CommandType type = this->command_queue_.front();
  this->command_queue_.pop();

  std::vector<uint8_t> pkt;
  switch (type) {
    case CommandType::GET_BATTERY:
      pkt = this->build_command_(MAJOR_BATTERY, MINOR_GET_BATTERY_LEVEL);
      break;
    case CommandType::IS_CHARGING:
      pkt = this->build_command_(MAJOR_BATTERY, MINOR_IS_CHARGING);
      break;
    case CommandType::GET_STATE:
      pkt = this->build_command_(MAJOR_MOWER, MINOR_GET_STATE);
      break;
    case CommandType::GET_ACTIVITY:
      pkt = this->build_command_(MAJOR_MOWER, MINOR_GET_ACTIVITY);
      break;
    case CommandType::GET_ERROR:
      pkt = this->build_command_(MAJOR_MOWER, MINOR_GET_ERROR);
      break;
    case CommandType::KEEPALIVE:
      pkt = this->build_command_(MAJOR_KEEPALIVE, MINOR_KEEPALIVE);
      break;
    case CommandType::START_TRIGGER:
      pkt = this->build_command_(MAJOR_MOWER, MINOR_START_TRIGGER);
      break;
    case CommandType::PAUSE:
      pkt = this->build_command_(MAJOR_MOWER, MINOR_PAUSE);
      break;
    case CommandType::PARK_UNTIL_NEXT:
      pkt = this->build_command_(MAJOR_OVERRIDE, MINOR_PARK_UNTIL_NEXT);
      break;
    case CommandType::SET_MODE_AUTO: {
      std::vector<uint8_t> payload = {0x00};  // AUTO = 0
      pkt = this->build_command_(MAJOR_MOWER, MINOR_SET_MODE, payload);
      break;
    }
    case CommandType::SET_OVERRIDE_MOW: {
      // 3 hours = 10800 seconds
      uint32_t duration = 10800;
      std::vector<uint8_t> payload = {
          static_cast<uint8_t>(duration & 0xFF),
          static_cast<uint8_t>((duration >> 8) & 0xFF),
          static_cast<uint8_t>((duration >> 16) & 0xFF),
          static_cast<uint8_t>((duration >> 24) & 0xFF)};
      pkt = this->build_command_(MAJOR_OVERRIDE, MINOR_SET_OVERRIDE_MOW, payload);
      break;
    }
    default:
      return;
  }

  this->response_buffer_.clear();
  this->pending_command_ = type;
  this->command_sent_at_ = millis();
  this->state_ = ConnectionState::WAITING_RESPONSE;
  this->write_data_(pkt);
}

// ---- String helpers ----

const char *AutomowerBLE::state_to_string_(uint8_t state) {
  switch (static_cast<MowerState>(state)) {
    case MowerState::OFF: return "off";
    case MowerState::WAIT_FOR_SAFETY_PIN: return "wait_for_safety_pin";
    case MowerState::STOPPED: return "stopped";
    case MowerState::FATAL_ERROR: return "fatal_error";
    case MowerState::PENDING_START: return "pending_start";
    case MowerState::PAUSED: return "paused";
    case MowerState::IN_OPERATION: return "in_operation";
    case MowerState::RESTRICTED: return "restricted";
    case MowerState::ERROR: return "error";
    default: return "unknown";
  }
}

const char *AutomowerBLE::activity_to_string_(uint8_t activity) {
  switch (static_cast<MowerActivity>(activity)) {
    case MowerActivity::NONE: return "none";
    case MowerActivity::CHARGING: return "charging";
    case MowerActivity::GOING_TO_CHARGE: return "going_to_charge";
    case MowerActivity::MOWING: return "mowing";
    case MowerActivity::LEAVING_DOCK: return "leaving_dock";
    case MowerActivity::PARKED_IN_DOCK: return "parked_in_dock";
    case MowerActivity::STOPPED_IN_GARDEN: return "stopped_in_garden";
    default: return "unknown";
  }
}

// ---- Button implementations ----

#ifdef USE_BUTTON
void AutomowerStartButton::press_action() {
  ESP_LOGI(TAG, "Start/Resume pressed");
  this->parent_->queue_command(CommandType::START_TRIGGER);
}

void AutomowerPauseButton::press_action() {
  ESP_LOGI(TAG, "Pause pressed");
  this->parent_->queue_command(CommandType::PAUSE);
}

void AutomowerParkButton::press_action() {
  ESP_LOGI(TAG, "Park pressed");
  // Park sequence: SetOverrideParkUntilNextStart + StartTrigger
  this->parent_->queue_command(CommandType::PARK_UNTIL_NEXT);
  this->parent_->queue_command(CommandType::START_TRIGGER);
}
#endif

}  // namespace automower_ble
}  // namespace esphome
