#include "protocol_eaton.h"
#include "constants_hid.h"
#include "constants_ups.h"
#include "protocol_factory.h"
#include "esphome/core/log.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace ups_hid {

static const char *const EATON_TAG = "ups_hid.eaton";

EatonHidProtocol::EatonHidProtocol(UpsHidComponent *parent)
    : UpsProtocolBase(parent) {}

bool EatonHidProtocol::detect() {
    if (!parent_->is_connected()) {
        return false;
    }

    uint16_t vid = parent_->get_vendor_id();
    if (vid != usb::VENDOR_ID_EATON && vid != usb::VENDOR_ID_MGE) {
        ESP_LOGD(EATON_TAG, "Not an Eaton/MGE device (VID=0x%04X)", vid);
        return false;
    }

    ESP_LOGI(EATON_TAG, "Eaton/MGE device detected (VID=0x%04X)", vid);
    return true;
}

bool EatonHidProtocol::initialize() {
    ESP_LOGI(EATON_TAG, "Initializing Eaton HID protocol...");

    // Read and parse the HID report descriptor
    uint8_t desc_buf[2048];
    size_t desc_len = sizeof(desc_buf);
    esp_err_t ret = parent_->get_hid_report_descriptor(desc_buf, &desc_len);
    if (ret != ESP_OK || desc_len == 0) {
        ESP_LOGE(EATON_TAG, "Failed to read HID report descriptor");
        return false;
    }

    if (!parser_.parse(desc_buf, desc_len)) {
        ESP_LOGE(EATON_TAG, "Failed to parse HID report descriptor");
        return false;
    }

    // Collect all feature report IDs we'll need to read
    auto feature_ids = parser_.get_report_ids(HID_REPORT_TYPE_FEATURE);
    for (uint8_t id : feature_ids) {
        feature_report_ids_.insert(id);
    }
    // Also check input reports (some Eaton models use them for status)
    auto input_ids = parser_.get_report_ids(HID_REPORT_TYPE_INPUT);
    for (uint8_t id : input_ids) {
        feature_report_ids_.insert(id);
    }

    ESP_LOGI(EATON_TAG, "Found %zu report IDs to poll", feature_report_ids_.size());

    // Log discovered fields at debug level
    const auto& fields = parser_.get_fields();
    for (const auto& f : fields) {
        ESP_LOGD(EATON_TAG, "  Field: page=0x%04X usage=0x%04X report=%d/%d offset=%d size=%d parent=0x%04X",
                 f.usage_page, f.usage_id, f.report_type, f.report_id,
                 f.bit_offset, f.bit_size, f.parent_collection);
    }

    // Verify minimum required fields exist
    const HidField* remaining_cap = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_REMAINING_CAPACITY);
    if (!remaining_cap) {
        // Also try as input report
        remaining_cap = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_REMAINING_CAPACITY, HID_REPORT_TYPE_INPUT);
    }
    if (remaining_cap) {
        ESP_LOGI(EATON_TAG, "RemainingCapacity found in report %d (type %d)", remaining_cap->report_id, remaining_cap->report_type);
    } else {
        ESP_LOGW(EATON_TAG, "RemainingCapacity not found in descriptor - battery level may be unavailable");
    }

    // Read device info strings via USB string descriptors
    std::string str;
    if (parent_->get_string_descriptor(1, str) == ESP_OK && !str.empty()) {
        ESP_LOGI(EATON_TAG, "Manufacturer: %s", str.c_str());
    }
    if (parent_->get_string_descriptor(2, str) == ESP_OK && !str.empty()) {
        ESP_LOGI(EATON_TAG, "Product: %s", str.c_str());
    }

    ESP_LOGI(EATON_TAG, "Eaton HID protocol initialized successfully");
    return true;
}

bool EatonHidProtocol::read_feature_report(uint8_t report_id) {
    // Determine report size from descriptor, with a reasonable minimum
    size_t byte_size = parser_.get_report_byte_size(HID_REPORT_TYPE_FEATURE, report_id);
    if (byte_size == 0) {
        // Try input report type
        byte_size = parser_.get_report_byte_size(HID_REPORT_TYPE_INPUT, report_id);
    }
    if (byte_size == 0) {
        byte_size = 8; // fallback
    }
    byte_size = std::max(byte_size, static_cast<size_t>(8));
    byte_size = std::min(byte_size, static_cast<size_t>(64));

    std::vector<uint8_t> buf(byte_size, 0);
    size_t len = byte_size;

    // Try feature report first, then input
    esp_err_t ret = parent_->hid_get_report(HID_REPORT_TYPE_FEATURE, report_id, buf.data(), &len);
    if (ret != ESP_OK) {
        ret = parent_->hid_get_report(HID_REPORT_TYPE_INPUT, report_id, buf.data(), &len);
    }

    if (ret == ESP_OK && len > 0) {
        buf.resize(len);
        report_cache_[report_id] = std::move(buf);
        return true;
    }
    return false;
}

int32_t EatonHidProtocol::extract_field_value(const HidField& field) const {
    auto it = report_cache_.find(field.report_id);
    if (it == report_cache_.end()) {
        return 0;
    }

    const auto& data = it->second;
    uint16_t bit_off = field.bit_offset;
    uint16_t bit_size = field.bit_size;

    // Extract bits from the byte array
    int32_t value = 0;
    for (uint16_t i = 0; i < bit_size; i++) {
        uint16_t abs_bit = bit_off + i;
        uint16_t byte_idx = abs_bit / 8;
        uint8_t bit_idx = abs_bit % 8;

        if (byte_idx < data.size()) {
            if (data[byte_idx] & (1 << bit_idx)) {
                value |= (1 << i);
            }
        }
    }

    // Sign extend if logical_min is negative
    if (field.logical_min < 0 && bit_size < 32) {
        if (value & (1 << (bit_size - 1))) {
            value |= ~((1 << bit_size) - 1);
        }
    }

    return value;
}

bool EatonHidProtocol::extract_bool_field(const HidField& field) const {
    return extract_field_value(field) != 0;
}

bool EatonHidProtocol::read_data(UpsData &data) {
    if (!parent_->is_connected()) {
        return false;
    }

    // Read all discovered reports
    int success_count = 0;
    for (uint8_t id : feature_report_ids_) {
        if (read_feature_report(id)) {
            success_count++;
        }
    }

    if (success_count == 0) {
        ESP_LOGW(EATON_TAG, "Failed to read any reports");
        return false;
    }

    ESP_LOGD(EATON_TAG, "Read %d/%zu reports successfully", success_count, feature_report_ids_.size());

    // Extract data from cached reports
    extract_battery_data(data);
    extract_power_data(data);
    extract_status_data(data);
    extract_device_info(data);
    extract_config_data(data);

    return true;
}

void EatonHidProtocol::extract_battery_data(UpsData &data) {
    // Battery level (RemainingCapacity)
    const HidField* f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_REMAINING_CAPACITY);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_REMAINING_CAPACITY, HID_REPORT_TYPE_INPUT);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val >= 0 && val <= 100) {
            data.battery.level = static_cast<float>(val);
        }
    }

    // Runtime to empty (seconds -> minutes)
    f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_RUN_TIME_TO_EMPTY);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_RUN_TIME_TO_EMPTY, HID_REPORT_TYPE_INPUT);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val >= 0) {
            data.battery.runtime_minutes = static_cast<float>(val) / 60.0f;
        }
    }

    // Battery voltage (in Power Device page, under Battery collection 0x12)
    auto voltage_fields = parser_.find_fields(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_VOLTAGE);
    for (const auto* vf : voltage_fields) {
        if (vf->parent_collection == HID_USAGE_POW_BATTERY ||
            vf->parent_collection == HID_USAGE_POW_BATTERY_SYSTEM) {
            int32_t val = extract_field_value(*vf);
            if (val > 0) {
                // Many Eaton devices report voltage in decivolts
                float voltage = (vf->logical_max > 0 && vf->logical_max < 1000) ?
                    static_cast<float>(val) : static_cast<float>(val) / 10.0f;
                data.battery.voltage = voltage;
            }
            break;
        }
    }

    // Battery voltage nominal (ConfigVoltage under Battery)
    auto config_voltage_fields = parser_.find_fields(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_CONFIG_VOLTAGE);
    for (const auto* cvf : config_voltage_fields) {
        if (cvf->parent_collection == HID_USAGE_POW_BATTERY ||
            cvf->parent_collection == HID_USAGE_POW_BATTERY_SYSTEM) {
            int32_t val = extract_field_value(*cvf);
            if (val > 0) {
                data.battery.voltage_nominal = static_cast<float>(val);
            }
            break;
        }
    }
}

void EatonHidProtocol::extract_power_data(UpsData &data) {
    auto voltage_fields = parser_.find_fields(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_VOLTAGE);
    for (const auto* vf : voltage_fields) {
        int32_t val = extract_field_value(*vf);
        if (val <= 0) continue;

        float voltage = static_cast<float>(val);
        // Eaton may report in decivolts for AC voltages
        if (vf->logical_max > 1000) {
            voltage /= 10.0f;
        }

        if (vf->parent_collection == HID_USAGE_POW_INPUT) {
            data.power.input_voltage = voltage;
        } else if (vf->parent_collection == HID_USAGE_POW_OUTPUT) {
            data.power.output_voltage = voltage;
        }
    }

    // Input voltage nominal
    auto config_voltage_fields = parser_.find_fields(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_CONFIG_VOLTAGE);
    for (const auto* cvf : config_voltage_fields) {
        if (cvf->parent_collection == HID_USAGE_POW_INPUT) {
            int32_t val = extract_field_value(*cvf);
            if (val > 0) {
                data.power.input_voltage_nominal = static_cast<float>(val);
            }
            break;
        }
    }

    // Frequency
    const HidField* f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_FREQUENCY);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_FREQUENCY, HID_REPORT_TYPE_INPUT);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val > 0) {
            // Some devices report in deci-Hz
            float freq = static_cast<float>(val);
            if (freq > 100.0f) freq /= 10.0f;
            if (freq >= FREQUENCY_MIN_VALID && freq <= FREQUENCY_MAX_VALID) {
                data.power.frequency = freq;
            }
        }
    }

    // Load percentage
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_PERCENT_LOAD);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_PERCENT_LOAD, HID_REPORT_TYPE_INPUT);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val >= 0 && val <= 200) {
            data.power.load_percent = static_cast<float>(val);
        }
    }

    // Transfer voltage limits
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_LOW_VOLTAGE_TRANSFER);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val > 0) data.power.input_transfer_low = static_cast<float>(val);
    }
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_HIGH_VOLTAGE_TRANSFER);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val > 0) data.power.input_transfer_high = static_cast<float>(val);
    }

    // Active power nominal
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_CONFIG_ACTIVE_POWER);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val > 0) data.power.realpower_nominal = static_cast<float>(val);
    }

    // Apparent power nominal
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_CONFIG_APPARENT_POWER);
    if (f) {
        int32_t val = extract_field_value(*f);
        if (val > 0) data.power.apparent_power_nominal = static_cast<float>(val);
    }
}

void EatonHidProtocol::extract_status_data(UpsData &data) {
    // ACPresent (Present usage under Input collection)
    bool ac_present = false;
    auto present_fields = parser_.find_fields(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_PRESENT);
    for (const auto* pf : present_fields) {
        if (pf->parent_collection == HID_USAGE_POW_INPUT ||
            pf->parent_collection == HID_USAGE_POW_POWER_SUMMARY) {
            ac_present = extract_bool_field(*pf);
            break;
        }
    }

    // Charging
    bool charging = false;
    const HidField* f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_CHARGING);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_CHARGING, HID_REPORT_TYPE_INPUT);
    if (f) charging = extract_bool_field(*f);

    // Discharging
    bool discharging = false;
    f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_DISCHARGING);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_DISCHARGING, HID_REPORT_TYPE_INPUT);
    if (f) discharging = extract_bool_field(*f);

    // Fully charged
    bool fully_charged = false;
    f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_FULLY_CHARGED);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_FULLY_CHARGED, HID_REPORT_TYPE_INPUT);
    if (f) fully_charged = extract_bool_field(*f);

    // Below remaining capacity limit (low battery)
    bool low_battery = false;
    // Usage 0x42 = BelowRemainingCapacityLimit in Battery System page
    f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, 0x0042);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, 0x0042, HID_REPORT_TYPE_INPUT);
    if (f) low_battery = extract_bool_field(*f);

    // Need replacement
    bool need_replacement = false;
    f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_NEED_REPLACEMENT);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_BATTERY_SYSTEM, HID_USAGE_BAT_NEED_REPLACEMENT, HID_REPORT_TYPE_INPUT);
    if (f) need_replacement = extract_bool_field(*f);

    // Overload
    bool overload = false;
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_OVERLOAD);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_OVERLOAD, HID_REPORT_TYPE_INPUT);
    if (f) overload = extract_bool_field(*f);

    // Internal failure
    bool fault = false;
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_INTERNAL_FAILURE);
    if (!f) f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_INTERNAL_FAILURE, HID_REPORT_TYPE_INPUT);
    if (f) fault = extract_bool_field(*f);

    // Build power status
    if (ac_present) {
        data.power.status = status::ONLINE;
    } else if (discharging) {
        data.power.status = status::ON_BATTERY;
    } else {
        data.power.status = status::UNKNOWN;
    }

    // Build battery status
    if (fully_charged) {
        data.battery.status = battery_status::FULLY_CHARGED;
    } else if (charging) {
        data.battery.status = battery_status::CHARGING;
    } else if (discharging) {
        data.battery.status = battery_status::DISCHARGING;
    } else {
        data.battery.status = battery_status::NORMAL;
    }

    if (low_battery) {
        data.battery.status += std::string(" - ") + battery_status::LOW;
    }
    if (need_replacement) {
        data.battery.status += battery_status::REPLACE_BATTERY_SUFFIX;
    }
    if (fault) {
        data.battery.status += battery_status::FAULT_SUFFIX;
    }

    ESP_LOGD(EATON_TAG, "Status: AC=%d charging=%d discharging=%d full=%d low=%d overload=%d fault=%d",
             ac_present, charging, discharging, fully_charged, low_battery, overload, fault);
}

void EatonHidProtocol::extract_device_info(UpsData &data) {
    // Read string descriptors for device identification
    // iManufacturer (index 1), iProduct (index 2), iSerialNumber (index 3)
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

    // Try reading iProduct from HID descriptor string indices
    // Look for string index fields in the descriptor
    const HidField* f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_I_PRODUCT);
    if (f && data.device.model.empty()) {
        int32_t idx = extract_field_value(*f);
        if (idx > 0 && parent_->get_string_descriptor(idx, str) == ESP_OK && !str.empty()) {
            data.device.model = str;
        }
    }

    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_I_MANUFACTURER);
    if (f && data.device.manufacturer.empty()) {
        int32_t idx = extract_field_value(*f);
        if (idx > 0 && parent_->get_string_descriptor(idx, str) == ESP_OK && !str.empty()) {
            data.device.manufacturer = str;
        }
    }

    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_I_SERIAL_NUMBER);
    if (f && data.device.serial_number.empty()) {
        int32_t idx = extract_field_value(*f);
        if (idx > 0 && parent_->get_string_descriptor(idx, str) == ESP_OK && !str.empty()) {
            data.device.serial_number = str;
        }
    }

    data.device.usb_vendor_id = parent_->get_vendor_id();
    data.device.usb_product_id = parent_->get_product_id();
}

void EatonHidProtocol::extract_config_data(UpsData &data) {
    // Delay before shutdown
    const HidField* f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_DELAY_BEFORE_SHUTDOWN);
    if (f) {
        int32_t val = extract_field_value(*f);
        data.config.delay_shutdown = val;
    }

    // Delay before startup
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_DELAY_BEFORE_STARTUP);
    if (f) {
        int32_t val = extract_field_value(*f);
        data.config.delay_start = val;
    }

    // Delay before reboot
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_DELAY_BEFORE_REBOOT);
    if (f) {
        int32_t val = extract_field_value(*f);
        data.config.delay_reboot = val;
    }

    // Audible alarm control
    f = parser_.find_field(HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_AUDIBLE_ALARM_CONTROL);
    if (f) {
        int32_t val = extract_field_value(*f);
        switch (val) {
            case 1: data.config.beeper_status = "Disabled"; break;
            case 2: data.config.beeper_status = "Enabled"; break;
            case 3: data.config.beeper_status = "Muted"; break;
            default: data.config.beeper_status = "Unknown"; break;
        }
    }
}

// Factory registration for Eaton (VID 0x0463) and MGE (VID 0x06DA)
REGISTER_UPS_PROTOCOL_FOR_VENDOR(0x0463, eaton_hid_protocol, esphome::ups_hid::create_eaton_protocol,
    "Eaton HID Protocol", "Eaton/MGE HID Power Device protocol with descriptor-driven field mapping", 100);

REGISTER_UPS_PROTOCOL_FOR_VENDOR(0x06DA, mge_hid_protocol, esphome::ups_hid::create_eaton_protocol,
    "Eaton HID Protocol", "MGE UPS Systems HID protocol (same as Eaton)", 100);

} // namespace ups_hid
} // namespace esphome
