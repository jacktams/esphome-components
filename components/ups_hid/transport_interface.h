#pragma once

#include "esp_err.h"
#include <vector>
#include <cstdint>
#include <memory>

namespace esphome {
namespace ups_hid {

/**
 * Abstract USB Transport Interface
 * 
 * Provides a clean abstraction over USB communication, allowing for
 * different implementations (ESP32 hardware, simulation, etc.)
 * 
 * Design Pattern: Strategy Pattern for transport selection
 */
class IUsbTransport {
public:
    virtual ~IUsbTransport() = default;
    
    // Transport lifecycle
    virtual esp_err_t initialize() = 0;
    virtual esp_err_t deinitialize() = 0;
    
    // Device management
    virtual bool is_connected() const = 0;
    virtual uint16_t get_vendor_id() const = 0;
    virtual uint16_t get_product_id() const = 0;
    
    // HID communication
    virtual esp_err_t hid_get_report(uint8_t report_type, uint8_t report_id, 
                                   uint8_t* data, size_t* data_len, 
                                   uint32_t timeout_ms = 1000) = 0;
    
    virtual esp_err_t hid_set_report(uint8_t report_type, uint8_t report_id,
                                   const uint8_t* data, size_t data_len,
                                   uint32_t timeout_ms = 1000) = 0;
    
    // String descriptors
    virtual esp_err_t get_string_descriptor(uint8_t string_index, 
                                          std::string& result) = 0;
    
    // Error information
    virtual std::string get_last_error() const = 0;

    // Device filtering - set expected VID/PID to avoid claiming wrong devices
    // When both are 0, auto-detect mode uses known UPS vendor list
    virtual void set_device_filter(uint16_t vendor_id, uint16_t product_id) {
        filter_vendor_id_ = vendor_id;
        filter_product_id_ = product_id;
    }

    // Diagnostics
    virtual bool is_initialized() const { return false; }
    virtual bool are_tasks_running() const { return false; }
    virtual bool has_client_handle() const { return false; }
    virtual void poll_for_devices() {}

protected:
    uint16_t filter_vendor_id_{0};
    uint16_t filter_product_id_{0};
};

// Forward declaration - implementation in usb_transport_factory.h
class UsbTransportFactory;

} // namespace ups_hid
} // namespace esphome