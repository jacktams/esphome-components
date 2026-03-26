#include "protocol_eaton.h"
#include "constants_hid.h"
#include "constants_ups.h"
#include "protocol_factory.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstring>
#include <memory>

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

    // If descriptor is available, also probe any Input report IDs it defines
    // that we haven't tried yet — the descriptor confirms these exist on the device
    if (descriptor_available_) {
        auto desc_input_ids = descriptor_parser_.get_report_ids(1);  // Input type
        for (uint8_t id : desc_input_ids) {
            // Skip if already probed
            bool already_probed = false;
            for (uint8_t existing : available_report_ids_) {
                if (existing == id) { already_probed = true; break; }
            }
            if (already_probed) continue;

            if (read_report(HID_REPORT_TYPE_INPUT, id, 8)) {
                available_report_ids_.push_back(id);
                ESP_LOGI(EATON_TAG, "Found descriptor Input report 0x%02X (%zu data bytes)",
                         id, report_cache_[id].size());
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    ESP_LOGI(EATON_TAG, "Found %zu available reports", available_report_ids_.size());

    if (available_report_ids_.empty()) {
        ESP_LOGE(EATON_TAG, "No readable reports found");
        return false;
    }

    // Read device strings (only 1-3 standard — 4+ often timeout on Eaton)
    std::string str;
    if (parent_->get_string_descriptor(1, str) == ESP_OK && !str.empty()) {
        ESP_LOGI(EATON_TAG, "Manufacturer: %s", str.c_str());
    }
    if (parent_->get_string_descriptor(2, str) == ESP_OK && !str.empty()) {
        ESP_LOGI(EATON_TAG, "Product: %s", str.c_str());
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
    // Eaton 5P descriptors are ~800-1200 bytes; use heap to avoid stack overflow
    static constexpr size_t DESC_BUF_SIZE = 4096;
    auto desc_buf = std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[DESC_BUF_SIZE]);
    if (!desc_buf) {
        ESP_LOGW(EATON_TAG, "Failed to allocate descriptor buffer");
        return false;
    }
    memset(desc_buf.get(), 0, DESC_BUF_SIZE);
    size_t desc_len = DESC_BUF_SIZE;

    ESP_LOGD(EATON_TAG, "Attempting to read HID report descriptor (max %zu bytes)...", desc_len);
    esp_err_t ret = parent_->get_hid_report_descriptor(desc_buf.get(), &desc_len,
                                                        parent_->get_protocol_timeout());
    if (ret != ESP_OK || desc_len == 0) {
        ESP_LOGW(EATON_TAG, "Failed to read HID report descriptor: %s", esp_err_to_name(ret));
        return false;
    }

    descriptor_size_ = desc_len;
    ESP_LOGI(EATON_TAG, "Read HID report descriptor: %zu bytes", desc_len);

    if (!descriptor_parser_.parse(desc_buf.get(), desc_len)) {
        ESP_LOGW(EATON_TAG, "Failed to parse HID report descriptor");
        return false;
    }

    descriptor_available_ = true;
    ESP_LOGI(EATON_TAG, "HID descriptor parsed: %zu fields", descriptor_parser_.get_fields().size());
    return true;
}

int32_t EatonHidProtocol::extract_field_value(const std::vector<uint8_t> &data,
                                                uint16_t bit_offset, uint16_t bit_size) {
    if (data.empty() || bit_size == 0) return 0;

    // Bounds check: ensure the field fits within the data
    size_t total_bits_needed = static_cast<size_t>(bit_offset) + bit_size;
    size_t total_bits_available = data.size() * 8;
    if (total_bits_needed > total_bits_available) return 0;

    // For single-bit boolean fields
    if (bit_size == 1) {
        size_t byte_idx = bit_offset / 8;
        uint8_t bit_idx = bit_offset % 8;
        return (data[byte_idx] >> bit_idx) & 1;
    }

    // For byte-aligned multi-byte fields
    size_t byte_offset = bit_offset / 8;
    if (bit_offset % 8 == 0 && bit_size % 8 == 0) {
        size_t num_bytes = bit_size / 8;
        int32_t value = 0;
        for (size_t i = 0; i < num_bytes && i < 4; i++) {
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
        if (data[byte_idx] & (1 << bit_idx)) {
            value |= (1 << i);
        }
    }
    return value;
}

bool EatonHidProtocol::read_field_from_descriptor(uint16_t usage_page, uint16_t usage_id,
                                                    int32_t &value, uint8_t report_type,
                                                    uint16_t parent_collection) {
    if (!descriptor_available_) return false;

    // Try the requested report type first, then fall back to other types.
    // Eaton devices define many fields under Feature reports (type=3) in the
    // descriptor, but Input reports (type=1) with the same report ID often
    // share the same byte layout. So if we can't find a field in Input reports,
    // we try Feature report field positions against our cached Input data.
    static const uint8_t try_types[] = {0, 1, 3, 2};  // 0 = requested type
    for (uint8_t t : try_types) {
        uint8_t search_type = (t == 0) ? report_type : t;
        if (t != 0 && search_type == report_type) continue;  // skip duplicate

        const HidField *field = descriptor_parser_.find_field(usage_page, usage_id,
                                                               search_type, parent_collection);
        if (!field) continue;

        auto it = report_cache_.find(field->report_id);
        if (it == report_cache_.end() || it->second.empty()) continue;

        value = extract_field_value(it->second, field->bit_offset, field->bit_size);

        // Clamp to logical range if defined
        if (field->logical_max > field->logical_min) {
            if (value < field->logical_min) value = field->logical_min;
            if (value > field->logical_max) value = field->logical_max;
        }

        return true;
    }

    return false;
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

    // Log descriptor status on first visible cycle
    if (first_read_ == 0) {
        ESP_LOGI(EATON_TAG, "Descriptor: %s (%zu fields from %zu bytes)",
                 descriptor_available_ ? "YES" : "NO",
                 descriptor_available_ ? descriptor_parser_.get_fields().size() : 0,
                 descriptor_size_);
        first_read_ = 1;
    }

    parse_power_summary(data);
    parse_status(data);
    parse_voltages(data);
    parse_load(data);
    parse_config(data);
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
        // Fallback: Report 0x0C (PowerSummary) on Eaton 5P
        // Empirical layout (4 data bytes after report ID strip):
        //   d[0..1] = Voltage in decivolts (battery voltage, e.g. 513 = 51.3V)
        //   d[2]    = RemainingCapacity (battery %)
        //   d[3]    = FullChargeCapacity (usually 100%)
        auto it = report_cache_.find(0x0C);
        if (it != report_cache_.end()) {
            const auto& d = it->second;
            // Battery percentage at byte 2
            if (d.size() >= 3) {
                uint8_t battery_pct = d[2];
                if (battery_pct <= 100) {
                    data.battery.level = static_cast<float>(battery_pct);
                }
                ESP_LOGD(EATON_TAG, "Battery level (fallback 0x0C[2]): %u%%", battery_pct);
            }
        }
    }

    // Runtime to empty — try descriptor first
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_RUN_TIME_TO_EMPTY, value)) {
        if (value > 0 && value < 1000000) {
            data.battery.runtime_minutes = static_cast<float>(value) / 60.0f;
        }
        ESP_LOGD(EATON_TAG, "Runtime (descriptor): %d sec = %.1f min", value,
                 data.battery.runtime_minutes);
    } else {
        // Fallback: Report 0x06 on Eaton 5P
        // Empirical layout (5 data bytes after report ID strip):
        //   d[0]    = RemainingCapacity (battery %, duplicate)
        //   d[1..2] = RunTimeToEmpty in seconds (LE uint16)
        //   d[3..4] = unknown (zeros)
        // Verified: d[1..2] = 0x1838 = 6200 sec = 103.3 min matches device display of 103 min
        auto it06 = report_cache_.find(0x06);
        if (it06 != report_cache_.end() && it06->second.size() >= 3) {
            const auto& d = it06->second;
            uint16_t runtime_sec = d[1] | (d[2] << 8);
            if (runtime_sec > 0 && runtime_sec < 65535) {
                data.battery.runtime_minutes = static_cast<float>(runtime_sec) / 60.0f;
            }
            ESP_LOGD(EATON_TAG, "Runtime (fallback 0x06[1:2]): %u sec = %.1f min",
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
        float voltage = static_cast<float>(value);
        if (voltage > 1000) voltage /= 10.0f;
        if (voltage > 0 && voltage < 60.0f) {
            data.battery.voltage = voltage;
            ESP_LOGD(EATON_TAG, "Battery voltage (descriptor/PowerSummary): %.1f V", voltage);
        }
    } else {
        // Fallback: Report 0x0C bytes 0-1 on Eaton 5P
        // Per NUT: UPS.PowerSummary.Voltage = battery voltage
        // Empirical: d[0..1] = 0x0201 = 513 decivolts = 51.3V
        // (Eaton 5P 1150VA: 4x12V = 48V nominal, ~51V at full charge)
        auto it = report_cache_.find(0x0C);
        if (it != report_cache_.end() && it->second.size() >= 2) {
            uint16_t raw16 = it->second[0] | (it->second[1] << 8);
            float voltage = static_cast<float>(raw16) / 10.0f;  // decivolts
            if (voltage > 0 && voltage < 60.0f) {
                data.battery.voltage = voltage;
            }
            ESP_LOGD(EATON_TAG, "Battery voltage (fallback 0x0C[0:1]): raw=%u, %.1f V",
                     raw16, voltage);
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
        // Determine power status — Discharging is the definitive on-battery indicator.
        // Eaton keeps ACPresent=1 even during battery operation (it means "AC input wired"
        // not "AC power live"), so we don't rely on ACPresent for this decision.
        if (discharging) {
            data.power.status = status::ON_BATTERY;
        } else {
            data.power.status = status::ONLINE;
        }

        // Determine battery status
        if (fully_charged) {
            data.battery.status = battery_status::FULLY_CHARGED;
        } else if (discharging) {
            data.battery.status = battery_status::DISCHARGING;
        } else if (charging) {
            data.battery.status = battery_status::CHARGING;
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
        // Fallback: try to interpret status from report 0x16 on Eaton 5P
        // Report 0x16 contains PresentStatus boolean bits.
        // Observed: 0x69 = 0b01101001 when online + fully charged.
        // The exact bit order depends on the HID descriptor. Based on the
        // observation that 0x69 means "online + fully charged + OK", the
        // most consistent mapping for Eaton 5P is:
        //   bit 0: ACPresent (1 = online)
        //   bit 3: FullyCharged (1 = battery full)
        //   bit 5: Good (1 = no faults)
        //   bit 6: Used/Present (1 = UPS active)
        // Bits 1,2 would be Charging/Discharging (both 0 when fully charged on mains)
        // NOTE: Without HID descriptor these are educated guesses
        auto it16 = report_cache_.find(0x16);
        if (it16 != report_cache_.end() && !it16->second.empty()) {
            uint8_t s = it16->second[0];
            bool fb_ac_present     = (s >> 0) & 1;
            bool fb_charging       = (s >> 1) & 1;
            bool fb_discharging    = (s >> 2) & 1;
            bool fb_fully_charged  = (s >> 3) & 1;
            bool fb_below_capacity = (s >> 4) & 1;

            ESP_LOGD(EATON_TAG, "Status bits (fallback 0x16=0x%02X): AC=%d Chrg=%d Dischrg=%d Full=%d Low=%d",
                     s, fb_ac_present, fb_charging, fb_discharging, fb_fully_charged,
                     fb_below_capacity);

            // Power status — Discharging is definitive
            if (fb_discharging) {
                data.power.status = status::ON_BATTERY;
            } else {
                data.power.status = status::ONLINE;
            }

            // Battery status
            if (fb_fully_charged) {
                data.battery.status = battery_status::FULLY_CHARGED;
            } else if (fb_discharging) {
                data.battery.status = battery_status::DISCHARGING;
            } else if (fb_charging) {
                data.battery.status = battery_status::CHARGING;
            } else {
                data.battery.status = battery_status::NORMAL;
            }

            if (fb_below_capacity) {
                data.battery.status += std::string(" - ") + battery_status::LOW;
            }
        } else {
            // Last resort: infer from battery level
            data.power.status = status::ONLINE;
            float battery_level = data.battery.level;
            if (!std::isnan(battery_level) && battery_level >= 100.0f) {
                data.battery.status = battery_status::FULLY_CHARGED;
            } else if (!std::isnan(battery_level) && battery_level > 10.0f) {
                data.battery.status = battery_status::CHARGING;
            } else {
                data.battery.status = battery_status::NORMAL;
            }
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
    } else {
        // Fallback: Report 0x31 byte 0 on Eaton 5P
        // Per NUT: UPS.PowerSummary.PercentLoad or output flow PercentLoad
        // Empirical: 0x31 d[0] = 0x02 = 2% (plausible for idle UPS)
        auto it = report_cache_.find(0x31);
        if (it != report_cache_.end() && !it->second.empty()) {
            uint8_t load = it->second[0];
            if (load <= 100) {
                data.power.load_percent = static_cast<float>(load);
                ESP_LOGD(EATON_TAG, "Load (fallback 0x31[0]): %u%%", load);
            }
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

void EatonHidProtocol::parse_config(UpsData &data) {
    // Config data is static — only read once
    if (config_read_) return;
    config_read_ = true;

    int32_t value;

    // Delay before shutdown — per NUT: UPS.PowerSummary.DelayBeforeShutdown
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_DELAY_BEFORE_SHUTDOWN, value)) {
        data.config.delay_shutdown = static_cast<int16_t>(value);
        ESP_LOGD(EATON_TAG, "Delay shutdown (descriptor): %d s", value);
    }

    // Delay before startup — per NUT: UPS.PowerSummary.DelayBeforeStartup
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_DELAY_BEFORE_STARTUP, value)) {
        data.config.delay_start = static_cast<int16_t>(value);
        ESP_LOGD(EATON_TAG, "Delay start (descriptor): %d s", value);
    }

    // Delay before reboot — per NUT: UPS.PowerSummary.DelayBeforeReboot
    if (read_field_from_descriptor(HID_USAGE_PAGE_POWER_DEVICE,
                                    HID_USAGE_POW_DELAY_BEFORE_REBOOT, value)) {
        data.config.delay_reboot = static_cast<int16_t>(value);
        ESP_LOGD(EATON_TAG, "Delay reboot (descriptor): %d s", value);
    }

    // Low battery threshold — per NUT: UPS.PowerSummary.RemainingCapacityLimitSetting
    // Usage 0x85, 0x008C (BAT_WARNING_CAPACITY_LIMIT)
    if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                    HID_USAGE_BAT_WARNING_CAPACITY_LIMIT, value)) {
        if (value >= 0 && value <= 100) {
            data.battery.charge_low = static_cast<float>(value);
            ESP_LOGD(EATON_TAG, "Battery charge low threshold (descriptor): %d%%", value);
        }
    }

    // Battery chemistry — per NUT: UPS.PowerSummary.iDeviceChemistry
    // Usage 0x85, 0x0089 (BAT_I_DEVICE_CHEMISTRY)
    if (data.battery.type.empty()) {
        if (read_field_from_descriptor(HID_USAGE_PAGE_BATTERY_SYSTEM,
                                        HID_USAGE_BAT_I_DEVICE_CHEMISTRY, value)) {
            // Value is a string descriptor index — read it
            if (value > 0 && value < 256) {
                std::string chem_str;
                if (parent_->get_string_descriptor(static_cast<uint8_t>(value), chem_str) == ESP_OK
                    && !chem_str.empty()) {
                    data.battery.type = chem_str;
                    ESP_LOGD(EATON_TAG, "Battery chemistry (descriptor): %s", chem_str.c_str());
                }
            } else if (value >= 1 && value <= 6) {
                // Direct chemistry ID per HID spec
                data.battery.type = battery_chemistry::id_to_string(static_cast<uint8_t>(value));
                ESP_LOGD(EATON_TAG, "Battery chemistry (descriptor ID): %s", data.battery.type.c_str());
            }
        }
    }
}

void EatonHidProtocol::read_device_strings(UpsData &data) {
    // Only read strings once — they never change and each USB transfer blocks ~50ms
    if (strings_read_) {
        return;
    }

    std::string str;
    if (data.device.manufacturer.empty()) {
        if (parent_->get_string_descriptor(1, str) == ESP_OK && !str.empty()) {
            data.device.manufacturer = str;
        }
    }
    if (data.device.model.empty()) {
        if (parent_->get_string_descriptor(2, str) == ESP_OK && !str.empty()) {
            data.device.model = str;
        }
    }
    if (data.device.serial_number.empty()) {
        if (parent_->get_string_descriptor(3, str) == ESP_OK && !str.empty()) {
            data.device.serial_number = str;
        }
    }
    data.device.usb_vendor_id = parent_->get_vendor_id();
    data.device.usb_product_id = parent_->get_product_id();
    strings_read_ = true;
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
