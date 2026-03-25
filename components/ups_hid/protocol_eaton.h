#pragma once

#include "ups_hid.h"
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
    // Read a report and cache it
    bool read_report(uint8_t report_type, uint8_t report_id, size_t expected_len);

    // Cached report data
    std::map<uint8_t, std::vector<uint8_t>> report_cache_;

    // Discovered report IDs during init
    std::vector<uint8_t> available_report_ids_;

    // Parse specific report types
    void parse_power_summary(UpsData &data);
    void parse_status(UpsData &data);
    void parse_voltages(UpsData &data);
    void read_device_strings(UpsData &data);
};

// Factory creator function
inline std::unique_ptr<UpsProtocolBase> create_eaton_protocol(UpsHidComponent* parent) {
    return std::make_unique<EatonHidProtocol>(parent);
}

} // namespace ups_hid
} // namespace esphome
