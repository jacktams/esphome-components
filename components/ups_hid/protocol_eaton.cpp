#include "protocol_eaton.h"
#include "constants_hid.h"
#include "constants_ups.h"
#include "protocol_factory.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <initializer_list>

namespace esphome {
namespace ups_hid {

static const char *const EATON_TAG = "ups_hid.eaton";

// Report IDs known to work on Eaton/MGE devices (from Generic protocol logs)
// Only probe IDs we've seen succeed — unknown IDs crash the USB stack
static const uint8_t PROBE_REPORT_IDS[] = {
    0x01, 0x02, 0x03, 0x06, 0x0C, 0x16, 0x30, 0x31,
};

EatonHidProtocol::EatonHidProtocol(UpsHidComponent *parent)
    : UpsProtocolBase(parent) {}

bool EatonHidProtocol::detect() {
    if (!parent_->is_connected()) {
        return false;
    }

    uint16_t vid = parent_->get_vendor_id();
    if (vid != usb::VENDOR_ID_EATON && vid != usb::VENDOR_ID_MGE) {
        return false;
    }

    ESP_LOGI(EATON_TAG, "Eaton/MGE device detected (VID=0x%04X)", vid);
    return true;
}

bool EatonHidProtocol::initialize() {
    ESP_LOGI(EATON_TAG, "Initializing Eaton HID protocol...");

    // Try to read the HID report descriptor first (standard USB request, safe)
    read_descriptor();

    // Discover available reports by probing Input reports only
    // (Feature report requests crash the USB stack on this device)
    for (uint8_t id : PROBE_REPORT_IDS) {
        if (read_report(HID_REPORT_TYPE_INPUT, id, 8)) {
            available_report_ids_.push_back(id);
            ESP_LOGD(EATON_TAG, "Found Input report 0x%02X (%zu data bytes)",
                     id, report_cache_[id].size());
        }
        // Delay between probes to avoid overwhelming the USB stack
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(EATON_TAG, "Found %zu available reports", available_report_ids_.size());

    if (available_report_ids_.empty()) {
        ESP_LOGE(EATON_TAG, "No readable reports found");
        return false;
    }

    // Read device strings
    std::string str;
    if (parent_->get_string_descriptor(1, str) == ESP_OK && !str.empty()) {
        ESP_LOGI(EATON_TAG, "Manufacturer: %s", str.c_str());
    }
    if (parent_->get_string_descriptor(2, str) == ESP_OK && !str.empty()) {
        ESP_LOGI(EATON_TAG, "Product: %s", str.c_str());
    }

    // Log all report data for debugging
    log_all_reports();

    // Log descriptor field mappings if available
    if (descriptor_available_) {
        log_descriptor_fields();
    }

    ESP_LOGI(EATON_TAG, "Eaton HID protocol initialized (descriptor: %s)",
             descriptor_available_ ? "available" : "not available, using fallback offsets");
    return true;
}

bool EatonHidProtocol::read_report(uint8_t report_type, uint8_t report_id, size_t expected_len) {
    uint8_t buf[64] = {0};
    size_t len = sizeof(buf);

    esp_err_t ret = parent_->hid_get_report(report_type, report_id, buf, &len);
    if (ret == ESP_OK && len > 1) {
        // Byte 0 is the report ID (included by ESP32 USB transport) — skip it
        // so that cached data starts at the first actual data byte
        report_cache_[report_id] = std::vector<uint8_t>(buf + 1, buf + len);
        return true;
    }
    return false;
}

bool EatonHidProtocol::read_descriptor() {
    uint8_t desc_buf[512] = {0};
    size_t desc_len = sizeof(desc_buf);

    ESP_LOGD(EATON_TAG, "Attempting to read HID report descriptor...");
    esp_err_t ret = parent_->get_hid_report_descriptor(desc_buf, &desc_len,
                                                        parent_->get_protocol_timeout());
    if (ret != ESP_OK || desc_len == 0) {
        ESP_LOGW(EATON_TAG, "Failed to read HID report descriptor: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(EATON_TAG, "Read HID report descriptor: %zu bytes", desc_len);

    if (!descriptor_parser_.parse(desc_buf, desc_len)) {
        ESP_LOGW(EATON_TAG, "Failed to parse HID report descriptor");
        return false;
    }

    descriptor_available_ = true;
    ESP_LOGI(EATON_TAG, "HID descriptor parsed: %zu fields", descriptor_parser_.get_fields().size());
    return true;
}

int32_t EatonHidProtocol::extract_field_value(const std::vector<uint8_t> &data,
                                                uint16_t bit_offset, uint16_t bit_size) {
    if (data.empty()) return 0;

    // For single-bit boolean fields
    if (bit_size == 1) {
        size_t byte_idx = bit_offset / 8;
        uint8_t bit_idx = bit_offset % 8;
        if (byte_idx < data.size()) {
            return (data[byte_idx] >> bit_idx) & 1;
        }
        return 0;
    }

    // For byte-aligned multi-byte fields
    size_t byte_offset = bit_offset / 8;
    if (bit_offset % 8 == 0 && bit_size % 8 == 0) {
        size_t num_bytes = bit_size / 8;
        if (byte_offset + num_bytes > data.size()) return 0;

        int32_t value = 0;
        for (size_t i = 0; i < num_bytes; i++) {
            value |= static_cast<int32_t>(data[byte_offset + i]) << (i * 8);
        }
        return value;
    }

    // For non-aligned fields, extract bit by bit
    int32_t value = 0;
    for (uint16_t i = 0; i < bit_size && i < 32; i++) {
        uint16_t abs_bit = bit_offset + i;
        size_t byte_idx = abs_bit / 8;
        uint8_t bit_idx = abs_bit % 8;
        if (byte_idx < data.size()) {
            if (data[byte_idx] & (1 << bit_idx)) {
                value |= (1 << i);
            }
        }
    }
    return value;
}

bool EatonHidProtocol::read_field_from_descriptor(uint16_t usage_page, uint16_t usage_id,
                                                    int32_t &value, uint8_t report_type,
                                                    uint16_t parent_collection) {
    if (!descriptor_available_) return false;

    const HidField *field = descriptor_parser_.find_field(usage_page, usage_id,
                                                           report_type, parent_collection);
    if (!field) return false;

    auto it = report_cache_.find(field->report_id);
    if (it == report_cache_.end() || it->second.empty()) return false;

    value = extract_field_value(it->second, field->bit_offset, field->bit_size);

    // Clamp to logical range if defined
    if (field->logical_max > field->logical_min) {
        if (value < field->logical_min) value = field->logical_min;
        if (value > field->logical_max) value = field->logical_max;
    }

    return true;
}

bool EatonHidProtocol::read_data(UpsData &data) {
    if (!parent_->is_connected()) {
        return false;
    }

    // Re-read all discovered reports (Input only)
    int success = 0;
    for (uint8_t id : available_report_ids_) {
        if (read_report(HID_REPORT_TYPE_INPUT, id, 8)) {
            success++;
        }
    }

    if (success == 0) {
        ESP_LOGW(EATON_TAG, "Failed to read any reports");
        return false;
    }

    if (first_read_) {
        log_all_reports();
        first_read_ = false;
    }

    parse_power_summary(data);
    parse_status(data);
    parse_voltages(data);
    parse_load(data);
    read_device_strings(data);

    return true;
}

void EatonHidProtocol::parse_power_summary(UpsData &data) {
    int32_t value;

    // Try descriptor-driven extraction first
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_REMAINING_CAPACITY, value)) {
        if (value >= 0 && value <= 100) {
            data.battery.level = static_cast<float>(value);
        }
        ESP_LOGD(EATON_TAG, "Battery level (descriptor): %d%%", value);
    } else {
        // Fallback: Report 0x0C byte 0 = battery percentage
        auto it = report_cache_.find(0x0C);
        if (it != report_cache_.end() && !it->second.empty()) {
            uint8_t battery_pct = it->second[0];
            if (battery_pct <= 100) {
                data.battery.level = static_cast<float>(battery_pct);
            }
            ESP_LOGD(EATON_TAG, "Battery level (fallback): %u%%", battery_pct);
        }
    }

    // Runtime to empty
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_RUN_TIME_TO_EMPTY, value)) {
        if (value > 0 && value < 1000000) {
            data.battery.runtime_minutes = static_cast<float>(value) / 60.0f;
        }
        ESP_LOGD(EATON_TAG, "Runtime (descriptor): %d sec = %.1f min", value,
                 data.battery.runtime_minutes);
    } else {
        // Fallback: Report 0x0C bytes 1-2 = runtime in seconds (LE uint16)
        auto it = report_cache_.find(0x0C);
        if (it != report_cache_.end() && it->second.size() >= 3) {
            uint16_t runtime_sec = it->second[1] | (it->second[2] << 8);
            if (runtime_sec > 0 && runtime_sec < 65535) {
                data.battery.runtime_minutes = static_cast<float>(runtime_sec) / 60.0f;
            }
            ESP_LOGD(EATON_TAG, "Runtime (fallback): %u sec = %.1f min",
                     runtime_sec, data.battery.runtime_minutes);
        }
    }

    // Battery voltage — try descriptor
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_VOLTAGE, value,
                                    1, HID_USAGE_POW_BATTERY_SYSTEM)) {
        float voltage = static_cast<float>(value);
        if (voltage > 1000) voltage /= 10.0f;  // decivolts
        if (voltage > 0 && voltage < 60.0f) {
            data.battery.voltage = voltage;
            ESP_LOGD(EATON_TAG, "Battery voltage (descriptor): %.1f V", voltage);
        }
    } else if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                           HID_USAGE_POW_VOLTAGE, value,
                                           1, HID_USAGE_POW_POWER_SUMMARY)) {
        // Some descriptors put battery voltage under PowerSummary
        float voltage = static_cast<float>(value);
        if (voltage > 1000) voltage /= 10.0f;
        if (voltage > 0 && voltage < 60.0f) {
            data.battery.voltage = voltage;
            ESP_LOGD(EATON_TAG, "Battery voltage (descriptor/PowerSummary): %.1f V", voltage);
        }
    }

    ESP_LOGD(EATON_TAG, "Power summary: Battery=%.0f%%, Runtime=%.1f min, BattVoltage=%.1f V",
             data.battery.level, data.battery.runtime_minutes, data.battery.voltage);
}

void EatonHidProtocol::parse_status(UpsData &data) {
    int32_t value;
    bool ac_present = false;
    bool charging = false;
    bool discharging = false;
    bool fully_charged = false;
    bool overload = false;
    bool have_status_bits = false;

    // Try descriptor-driven status extraction
    // ACPresent / Present (0x84, 0x60)
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_PRESENT, value)) {
        ac_present = (value != 0);
        have_status_bits = true;
        ESP_LOGD(EATON_TAG, "ACPresent (descriptor): %d", value);
    }

    // Charging (0x85, 0x44)
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_CHARGING, value)) {
        charging = (value != 0);
        have_status_bits = true;
        ESP_LOGD(EATON_TAG, "Charging (descriptor): %d", value);
    }

    // Discharging (0x85, 0x45)
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_DISCHARGING, value)) {
        discharging = (value != 0);
        have_status_bits = true;
        ESP_LOGD(EATON_TAG, "Discharging (descriptor): %d", value);
    }

    // Fully Charged (0x85, 0x46)
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_FULLY_CHARGED, value)) {
        fully_charged = (value != 0);
        have_status_bits = true;
        ESP_LOGD(EATON_TAG, "FullyCharged (descriptor): %d", value);
    }

    // Overload (0x84, 0x65)
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_OVERLOAD, value)) {
        overload = (value != 0);
        have_status_bits = true;
        ESP_LOGD(EATON_TAG, "Overload (descriptor): %d", value);
    }

    // Below Remaining Capacity Limit (usage 0x85, 0x42 — not in our constants, try raw)
    // NeedReplacement (0x85, 0x4B)
    bool need_replacement = false;
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_NEED_REPLACEMENT, value)) {
        need_replacement = (value != 0);
        ESP_LOGD(EATON_TAG, "NeedReplacement (descriptor): %d", value);
    }

    // Log raw status report bytes for debugging
    auto it16 = report_cache_.find(0x16);
    if (it16 != report_cache_.end() && !it16->second.empty()) {
        uint8_t status_byte = it16->second[0];
        ESP_LOGD(EATON_TAG, "Present status raw: 0x%02X (0b%d%d%d%d%d%d%d%d)",
                 status_byte,
                 (status_byte >> 7) & 1, (status_byte >> 6) & 1,
                 (status_byte >> 5) & 1, (status_byte >> 4) & 1,
                 (status_byte >> 3) & 1, (status_byte >> 2) & 1,
                 (status_byte >> 1) & 1, status_byte & 1);
    }

    auto it06 = report_cache_.find(0x06);
    if (it06 != report_cache_.end() && !it06->second.empty()) {
        std::string hex;
        for (size_t i = 0; i < it06->second.size(); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", it06->second[i]);
            hex += buf;
        }
        ESP_LOGD(EATON_TAG, "Battery status raw: %s", hex.c_str());
    }

    if (have_status_bits) {
        // Determine power status from descriptor bits
        if (discharging && !ac_present) {
            data.power.status = status::ON_BATTERY;
        } else {
            data.power.status = status::ONLINE;
        }

        // Determine battery status
        if (fully_charged) {
            data.battery.status = battery_status::FULLY_CHARGED;
        } else if (charging) {
            data.battery.status = battery_status::CHARGING;
        } else if (discharging) {
            data.battery.status = battery_status::DISCHARGING;
        } else {
            data.battery.status = battery_status::NORMAL;
        }

        if (need_replacement) {
            data.battery.status += battery_status::REPLACE_BATTERY_SUFFIX;
        }
        if (overload) {
            data.power.status = std::string(status::ONLINE) + " - Overload";
        }
    } else {
        // Fallback: infer from battery level and voltage
        data.power.status = status::ONLINE;  // default

        float battery_level = data.battery.level;
        bool has_battery_level = !std::isnan(battery_level) && battery_level >= 0;

        if (has_battery_level) {
            if (battery_level >= 100.0f) {
                data.battery.status = battery_status::FULLY_CHARGED;
            } else if (battery_level > 10.0f) {
                data.battery.status = battery_status::CHARGING;
            } else {
                data.battery.status = std::string(battery_status::CHARGING) + " - " + battery_status::LOW;
            }
        } else {
            data.battery.status = battery_status::NORMAL;
        }
    }

    ESP_LOGD(EATON_TAG, "Status: power='%s', battery='%s'",
             data.power.status.c_str(), data.battery.status.c_str());
}

void EatonHidProtocol::parse_voltages(UpsData &data) {
    int32_t value;

    // Input voltage — try descriptor with Input collection parent
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_VOLTAGE, value,
                                    1, HID_USAGE_POW_INPUT)) {
        float voltage = static_cast<float>(value);
        if (voltage > 1000) voltage /= 10.0f;  // decivolts
        if (voltage >= VOLTAGE_MIN_VALID && voltage <= VOLTAGE_MAX_VALID) {
            data.power.input_voltage = voltage;
            ESP_LOGD(EATON_TAG, "Input voltage (descriptor): %.1f V", voltage);
        }
    } else {
        // Fallback: Report 0x30 bytes 0-1 = input voltage (LE uint16)
        auto it = report_cache_.find(0x30);
        if (it != report_cache_.end() && it->second.size() >= 2) {
            uint16_t raw16 = it->second[0] | (it->second[1] << 8);
            float voltage = static_cast<float>(raw16);
            if (voltage > 1000) voltage /= 10.0f;
            if (voltage >= VOLTAGE_MIN_VALID && voltage <= VOLTAGE_MAX_VALID) {
                data.power.input_voltage = voltage;
            }
            ESP_LOGD(EATON_TAG, "Input voltage (fallback 0x30): raw=%u, %.1f V", raw16, voltage);
        }
    }

    // Output voltage — try descriptor with Output collection parent
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_VOLTAGE, value,
                                    1, HID_USAGE_POW_OUTPUT)) {
        float voltage = static_cast<float>(value);
        if (voltage > 1000) voltage /= 10.0f;
        if (voltage >= VOLTAGE_MIN_VALID && voltage <= VOLTAGE_MAX_VALID) {
            data.power.output_voltage = voltage;
            ESP_LOGD(EATON_TAG, "Output voltage (descriptor): %.1f V", voltage);
        }
    } else {
        // Fallback: Report 0x31 bytes 4-5 = output voltage in decivolts
        auto it = report_cache_.find(0x31);
        if (it != report_cache_.end() && it->second.size() >= 6) {
            uint16_t raw16 = it->second[4] | (it->second[5] << 8);
            float voltage = static_cast<float>(raw16);
            if (voltage > 1000) voltage /= 10.0f;
            if (voltage >= VOLTAGE_MIN_VALID && voltage <= VOLTAGE_MAX_VALID) {
                data.power.output_voltage = voltage;
            }
            ESP_LOGD(EATON_TAG, "Output voltage (fallback 0x31[4:5]): raw=%u, %.1f V", raw16, voltage);
        }
    }

    // Frequency — try descriptor
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_FREQUENCY, value)) {
        float freq = static_cast<float>(value);
        if (freq > 100.0f) freq /= 10.0f;  // deci-Hz
        if (freq >= FREQUENCY_MIN_VALID && freq <= FREQUENCY_MAX_VALID) {
            data.power.frequency = freq;
            ESP_LOGD(EATON_TAG, "Frequency (descriptor): %.1f Hz", freq);
        }
    } else {
        // Fallback: Report 0x31 bytes 2-3 = frequency in deci-Hz
        auto it = report_cache_.find(0x31);
        if (it != report_cache_.end() && it->second.size() >= 4) {
            uint16_t freq_raw = it->second[2] | (it->second[3] << 8);
            float freq = static_cast<float>(freq_raw);
            if (freq > 100.0f) freq /= 10.0f;
            if (freq >= FREQUENCY_MIN_VALID && freq <= FREQUENCY_MAX_VALID) {
                data.power.frequency = freq;
            }
            ESP_LOGD(EATON_TAG, "Frequency (fallback 0x31[2:3]): raw=%u, %.1f Hz", freq_raw, freq);
        }
    }

    // Config voltage (nominal) — try descriptor
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_CONFIG_VOLTAGE, value,
                                    1, HID_USAGE_POW_OUTPUT)) {
        float v = static_cast<float>(value);
        if (v > 1000) v /= 10.0f;
        if (v >= VOLTAGE_MIN_VALID && v <= VOLTAGE_MAX_VALID) {
            data.power.output_voltage_nominal = v;
            ESP_LOGD(EATON_TAG, "Output voltage nominal (descriptor): %.1f V", v);
        }
    }

    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_CONFIG_VOLTAGE, value,
                                    1, HID_USAGE_POW_INPUT)) {
        float v = static_cast<float>(value);
        if (v > 1000) v /= 10.0f;
        if (v >= VOLTAGE_MIN_VALID && v <= VOLTAGE_MAX_VALID) {
            data.power.input_voltage_nominal = v;
            ESP_LOGD(EATON_TAG, "Input voltage nominal (descriptor): %.1f V", v);
        }
    }

    // Transfer voltage thresholds
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_LOW_VOLTAGE_TRANSFER, value)) {
        float v = static_cast<float>(value);
        if (v > 1000) v /= 10.0f;
        if (v > 0 && v <= VOLTAGE_MAX_VALID) {
            data.power.input_transfer_low = v;
            ESP_LOGD(EATON_TAG, "Transfer low (descriptor): %.1f V", v);
        }
    }

    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_HIGH_VOLTAGE_TRANSFER, value)) {
        float v = static_cast<float>(value);
        if (v > 1000) v /= 10.0f;
        if (v > 0 && v <= VOLTAGE_MAX_VALID) {
            data.power.input_transfer_high = v;
            ESP_LOGD(EATON_TAG, "Transfer high (descriptor): %.1f V", v);
        }
    }

    // Log raw report bytes for voltage-related reports
    for (uint8_t rid : std::initializer_list<uint8_t>{0x30, 0x31}) {
        auto it = report_cache_.find(rid);
        if (it != report_cache_.end()) {
            const auto& d = it->second;
            std::string hex;
            for (size_t i = 0; i < d.size(); i++) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", d[i]);
                hex += buf;
            }
            ESP_LOGD(EATON_TAG, "Report 0x%02X [%zu]: %s", rid, d.size(), hex.c_str());
        }
    }

    // If we have valid input voltage, confirm online status
    if (!std::isnan(data.power.input_voltage) && data.power.input_voltage > VOLTAGE_MIN_VALID) {
        if (data.power.status.empty()) {
            data.power.status = status::ONLINE;
        }
    }
}

void EatonHidProtocol::parse_load(UpsData &data) {
    int32_t value;

    // Try descriptor — PercentLoad (0x84, 0x35)
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_PERCENT_LOAD, value)) {
        if (value >= 0 && value <= 200) {
            data.power.load_percent = static_cast<float>(value);
            ESP_LOGD(EATON_TAG, "Load (descriptor): %d%%", value);
        }
    }

    // Try descriptor — ApparentPower / ActivePower for nominal ratings
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_CONFIG_ACTIVE_POWER, value)) {
        if (value > 0) {
            data.power.realpower_nominal = static_cast<float>(value);
            ESP_LOGD(EATON_TAG, "Real power nominal (descriptor): %d W", value);
        }
    }

    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_CONFIG_APPARENT_POWER, value)) {
        if (value > 0) {
            data.power.apparent_power_nominal = static_cast<float>(value);
            ESP_LOGD(EATON_TAG, "Apparent power nominal (descriptor): %d VA", value);
        }
    }
}

void EatonHidProtocol::read_device_strings(UpsData &data) {
    if (data.device.manufacturer.empty()) {
        std::string str;
        if (parent_->get_string_descriptor(1, str) == ESP_OK && !str.empty()) {
            data.device.manufacturer = str;
        }
    }
    if (data.device.model.empty()) {
        std::string str;
        if (parent_->get_string_descriptor(2, str) == ESP_OK && !str.empty()) {
            data.device.model = str;
        }
    }
    if (data.device.serial_number.empty()) {
        std::string str;
        if (parent_->get_string_descriptor(3, str) == ESP_OK && !str.empty()) {
            data.device.serial_number = str;
        }
    }
    data.device.usb_vendor_id = parent_->get_vendor_id();
    data.device.usb_product_id = parent_->get_product_id();
}

void EatonHidProtocol::log_all_reports() {
    for (uint8_t id : available_report_ids_) {
        auto it = report_cache_.find(id);
        if (it == report_cache_.end()) continue;
        const auto& d = it->second;
        std::string hex;
        for (size_t i = 0; i < d.size(); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", d[i]);
            hex += buf;
        }
        ESP_LOGI(EATON_TAG, "Report 0x%02X [%zu data bytes]: %s", id, d.size(), hex.c_str());
    }
}

void EatonHidProtocol::log_descriptor_fields() {
    const auto& fields = descriptor_parser_.get_fields();
    ESP_LOGI(EATON_TAG, "=== HID Descriptor Field Map (%zu fields) ===", fields.size());
    for (const auto& f : fields) {
        const char* type_str = (f.report_type == 1) ? "Input" :
                               (f.report_type == 2) ? "Output" : "Feature";
        ESP_LOGI(EATON_TAG, "  %s Report 0x%02X: Page=0x%04X Usage=0x%04X "
                 "Offset=%u bits Size=%u bits Range=[%d,%d] Parent=0x%04X",
                 type_str, f.report_id, f.usage_page, f.usage_id,
                 f.bit_offset, f.bit_size, f.logical_min, f.logical_max,
                 f.parent_collection);
    }
    ESP_LOGI(EATON_TAG, "=== End Field Map ===");
}

// Factory registration
REGISTER_UPS_PROTOCOL_FOR_VENDOR(0x0463, eaton_hid_protocol, esphome::ups_hid::create_eaton_protocol,
    "Eaton HID Protocol", "Eaton/MGE HID Power Device protocol", 100);

REGISTER_UPS_PROTOCOL_FOR_VENDOR(0x06DA, mge_hid_protocol, esphome::ups_hid::create_eaton_protocol,
    "Eaton HID Protocol", "MGE UPS Systems HID protocol", 100);

} // namespace ups_hid
} // namespace esphome
