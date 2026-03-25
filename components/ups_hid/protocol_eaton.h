#pragma once

#include "ups_hid.h"
#include "hid_descriptor_parser.h"
#include <map>
#include <vector>

namespace esphome {
namespace ups_hid {

class EatonHidProtocol : public UpsProtocolBase {
public:
    explicit EatonHidProtocol(UpsHidComponent *parent);

    bool detect() override;
    bool initialize() override;
    bool read_data(UpsData &data) override;
    DeviceInfo::DetectedProtocol get_protocol_type() const override { return DeviceInfo::PROTOCOL_EATON_HID; }
    std::string get_protocol_name() const override { return "Eaton HID Protocol"; }

private:
    // Read a report and cache it (strips report ID byte from buffer)
    bool read_report(uint8_t report_type, uint8_t report_id, size_t expected_len);

    // Try to read and parse the HID report descriptor for field mapping
    bool read_descriptor();

    // Extract a field value from cached report data using the HID descriptor
    // Returns true if the field was found and extracted
    bool read_field_from_descriptor(uint16_t usage_page, uint16_t usage_id,
                                     int32_t &value, uint8_t report_type = 1,
                                     uint16_t parent_collection = 0);

    // Extract a value from report data at a given bit offset and size
    int32_t extract_field_value(const std::vector<uint8_t> &data,
                                uint16_t bit_offset, uint16_t bit_size);

    // Cached report data (report ID stripped — byte 0 is first data byte)
    std::map<uint8_t, std::vector<uint8_t>> report_cache_;

    // Discovered report IDs during init
    std::vector<uint8_t> available_report_ids_;

    // HID descriptor parser (populated if descriptor read succeeds)
    HidDescriptorParser descriptor_parser_;
    bool descriptor_available_{false};

    // Read cycle counter for logging (log field map first few cycles)
    uint8_t first_read_{0};

    // Raw descriptor size for diagnostics
    size_t descriptor_size_{0};

    // Parse specific data from reports
    void parse_power_summary(UpsData &data);
    void parse_status(UpsData &data);
    void parse_voltages(UpsData &data);
    void parse_load(UpsData &data);
    void parse_config(UpsData &data);
    void read_device_strings(UpsData &data);

    // Debug logging
    void log_all_reports();
    void log_descriptor_fields();
};

// Factory creator function
inline std::unique_ptr<UpsProtocolBase> create_eaton_protocol(UpsHidComponent* parent) {
    return std::make_unique<EatonHidProtocol>(parent);
}

} // namespace ups_hid
} // namespace esphome
