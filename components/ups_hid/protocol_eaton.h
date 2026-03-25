#pragma once

#include "ups_hid.h"
#include "hid_descriptor_parser.h"
#include <map>
#include <set>

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
    HidDescriptorParser parser_;

    // Cached report data by report ID
    std::map<uint8_t, std::vector<uint8_t>> report_cache_;

    // Set of feature report IDs we need to read
    std::set<uint8_t> feature_report_ids_;

    // Read a feature report into cache
    bool read_feature_report(uint8_t report_id);

    // Extract a value from cached report data using a field descriptor
    int32_t extract_field_value(const HidField& field) const;

    // Extract a boolean (1-bit) field
    bool extract_bool_field(const HidField& field) const;

    // Map parsed fields to UpsData
    void extract_battery_data(UpsData &data);
    void extract_power_data(UpsData &data);
    void extract_status_data(UpsData &data);
    void extract_device_info(UpsData &data);
    void extract_config_data(UpsData &data);
};

// Factory creator function
inline std::unique_ptr<UpsProtocolBase> create_eaton_protocol(UpsHidComponent* parent) {
    return std::make_unique<EatonHidProtocol>(parent);
}

} // namespace ups_hid
} // namespace esphome
