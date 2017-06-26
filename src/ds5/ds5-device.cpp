// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include <mutex>
#include <chrono>
#include <vector>
#include <iterator>
#include <cstddef>

#include "device.h"
#include "context.h"
#include "image.h"
#include "metadata-parser.h"

#include "ds5-device.h"
#include "ds5-private.h"
#include "ds5-options.h"
#include "ds5-timestamp.h"

namespace rsimpl2
{
    class ds5_auto_exposure_roi_method : public region_of_interest_method
    {
    public:
        ds5_auto_exposure_roi_method(const hw_monitor& hwm) : _hw_monitor(hwm) {}

        void set(const region_of_interest& roi) override
        {
            command cmd(ds::SETAEROI);
            cmd.param1 = roi.min_y;
            cmd.param2 = roi.max_y;
            cmd.param3 = roi.min_x;
            cmd.param4 = roi.max_x;
            _hw_monitor.send(cmd);
        }

        region_of_interest get() const override
        {
            region_of_interest roi;
            command cmd(ds::GETAEROI);
            auto res = _hw_monitor.send(cmd);

            if (res.size() < 4 * sizeof(uint16_t))
            {
                throw std::runtime_error("Invalid result size!");
            }

            auto words = reinterpret_cast<uint16_t*>(res.data());

            roi.min_y = words[0];
            roi.max_y = words[1];
            roi.min_x = words[2];
            roi.max_x = words[3];

            return roi;
        }

    private:
        const hw_monitor& _hw_monitor;
    };

    std::vector<uint8_t> ds5_device::send_receive_raw_data(const std::vector<uint8_t>& input)
    {
        return _hw_monitor->send(input);
    }

    void ds5_device::hardware_reset()
    {
        command cmd(ds::HWRST);
        _hw_monitor->send(cmd);
    }

    rs2_intrinsics ds5_device::get_intrinsics(unsigned int subdevice, const stream_profile& profile) const
    {
        if (subdevice >= get_sensors_count())
            throw invalid_value_exception(to_string() << "Requested subdevice " <<
                                          subdevice << " is unsupported.");

        if (subdevice == _depth_device_idx)
        {
            return get_intrinsic_by_resolution(
                *_coefficients_table_raw,
                ds::calibration_table_id::coefficients_table_id,
                profile.width, profile.height);
        }

        throw not_implemented_exception("Not Implemented");
    }

    pose ds5_device::get_device_position(unsigned int subdevice) const
    {
        if (subdevice >= get_sensors_count())
            throw invalid_value_exception(to_string() << "Requested subdevice " <<
                                          subdevice << " is unsupported.");

        throw not_implemented_exception("Not Implemented");
    }

    bool ds5_device::is_camera_in_advanced_mode() const
    {
        command cmd(ds::UAMG);
        assert(_hw_monitor);
        auto ret = _hw_monitor->send(cmd);
        if (ret.empty())
            throw invalid_value_exception("command result is empty!");

        return (0 != ret.front());
    }

    std::vector<uint8_t> ds5_device::get_raw_calibration_table(ds::calibration_table_id table_id) const
    {
        command cmd(ds::GETINTCAL, table_id);
        return _hw_monitor->send(cmd);
    }

    std::shared_ptr<uvc_sensor> ds5_device::create_depth_device(const uvc::backend& backend,
                                                                  const std::vector<uvc::uvc_device_info>& all_device_infos)
    {
        using namespace ds;

        std::vector<std::shared_ptr<uvc::uvc_device>> depth_devices;
        for (auto&& info : filter_by_mi(all_device_infos, 0)) // Filter just mi=0, DEPTH
            depth_devices.push_back(backend.create_uvc_device(info));


        std::unique_ptr<frame_timestamp_reader> ds5_timestamp_reader_backup(new ds5_timestamp_reader(backend.create_time_service()));
        auto depth_ep = std::make_shared<uvc_sensor>(std::make_shared<uvc::multi_pins_uvc_device>(depth_devices),
                                                       std::unique_ptr<frame_timestamp_reader>(new ds5_timestamp_reader_from_metadata(std::move(ds5_timestamp_reader_backup))),
                                                       backend.create_time_service());
        depth_ep->register_xu(depth_xu); // make sure the XU is initialized everytime we power the camera


        depth_ep->register_pixel_format(pf_z16); // Depth
        depth_ep->register_pixel_format(pf_y8); // Left Only - Luminance
        depth_ep->register_pixel_format(pf_yuyv); // Left Only
        depth_ep->register_pixel_format(pf_uyvyl); // Color from Depth
        depth_ep->register_pixel_format(pf_rgb888);

        depth_ep->set_pose(lazy<pose>([](){pose p = {{ { 1,0,0 },{ 0,1,0 },{ 0,0,1 } },{ 0,0,0 }}; return p; }));

        return depth_ep;
    }

    ds5_device::ds5_device(const uvc::backend& backend,
                           const std::vector<uvc::uvc_device_info>& dev_info,
                           const std::vector<uvc::usb_device_info>& hwm_device,
                           const std::vector<uvc::hid_device_info>& hid_info)
        : _depth_device_idx(add_sensor(create_depth_device(backend, dev_info)))
    {
        using namespace ds;

        if(hwm_device.size()>0)
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                                     std::make_shared<locked_transfer>(
                                        backend.create_usb_device(hwm_device.front()), get_depth_sensor()));
        }
        else
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                            std::make_shared<locked_transfer>(
                                std::make_shared<command_transfer_over_xu>(
                                    get_depth_sensor(), rsimpl2::ds::depth_xu, rsimpl2::ds::DS5_HWMONITOR),
                                get_depth_sensor()));
        }

        _coefficients_table_raw = [this]() { return get_raw_calibration_table(coefficients_table_id); };

        std::string device_name = (rs4xx_sku_names.end() != rs4xx_sku_names.find(dev_info.front().pid)) ? rs4xx_sku_names.at(dev_info.front().pid) : "RS4xx";
        _fw_version = firmware_version(_hw_monitor->get_firmware_version_string(GVD, camera_fw_version_offset));
        auto serial = _hw_monitor->get_module_serial_string(GVD, module_serial_offset);

        auto& depth_ep = get_depth_sensor();
        auto advanced_mode = is_camera_in_advanced_mode();
        if (advanced_mode)
        {
            depth_ep.register_pixel_format(pf_y8i); // L+R
            depth_ep.register_pixel_format(pf_y12i); // L+R - Calibration not rectified
        }

        auto pid = dev_info.front().pid;
        auto pid_hex_str = hexify(pid>>8) + hexify(static_cast<uint8_t>(pid));

        std::string is_camera_locked{""};
        if (_fw_version >= firmware_version("5.6.3.0"))
        {
            auto is_locked = _hw_monitor->is_camera_locked(GVD, is_camera_locked_offset);
            is_camera_locked = (is_locked)?"YES":"NO";

#ifdef HWM_OVER_XU
            //if hw_monitor was created by usb replace it xu
            if(hwm_device.size() > 0)
            {
                _hw_monitor = std::make_shared<hw_monitor>(
                                std::make_shared<locked_transfer>(
                                    std::make_shared<command_transfer_over_xu>(
                                        get_depth_sensor(), rsimpl2::ds::depth_xu, rsimpl2::ds::DS5_HWMONITOR),
                                    get_depth_sensor()));
            }
#endif

            depth_ep.register_pu(RS2_OPTION_GAIN);
            auto exposure_option = std::make_shared<uvc_xu_option<uint32_t>>(depth_ep,
                                                                             depth_xu,
                                                                             DS5_EXPOSURE,
                                                                             "Depth Exposure");
            depth_ep.register_option(RS2_OPTION_EXPOSURE, exposure_option);

            auto enable_auto_exposure = std::make_shared<uvc_xu_option<uint8_t>>(depth_ep,
                                                                                 depth_xu,
                                                                                 DS5_ENABLE_AUTO_EXPOSURE,
                                                                                 "Enable Auto Exposure");
            depth_ep.register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable_auto_exposure);

            depth_ep.register_option(RS2_OPTION_GAIN,
                                     std::make_shared<auto_disabling_control>(
                                     std::make_shared<uvc_pu_option>(depth_ep, RS2_OPTION_GAIN),
                                     enable_auto_exposure));
            depth_ep.register_option(RS2_OPTION_EXPOSURE,
                                     std::make_shared<auto_disabling_control>(
                                     exposure_option,
                                     enable_auto_exposure));
        }

        if (_fw_version >= firmware_version("5.5.8.0"))
        {
             depth_ep.register_option(RS2_OPTION_OUTPUT_TRIGGER_ENABLED,
                                      std::make_shared<uvc_xu_option<uint8_t>>(depth_ep, depth_xu, DS5_EXT_TRIGGER,
                                      "Generate trigger from the camera to external device once per frame"));

             auto error_control = std::unique_ptr<uvc_xu_option<uint8_t>>(new uvc_xu_option<uint8_t>(depth_ep, depth_xu, DS5_ERROR_REPORTING, "Error reporting"));

             _polling_error_handler = std::unique_ptr<polling_error_handler>(
                 new polling_error_handler(1000,
                     std::move(error_control),
                     depth_ep.get_notifications_proccessor(),

                     std::unique_ptr<notification_decoder>(new ds5_notification_decoder())));

             _polling_error_handler->start();

             depth_ep.register_option(RS2_OPTION_ERROR_POLLING_ENABLED, std::make_shared<polling_errors_disable>(_polling_error_handler.get()));

             depth_ep.register_option(RS2_OPTION_ASIC_TEMPERATURE,
                                      std::make_shared<asic_and_projector_temperature_options>(depth_ep,
                                                                                               RS2_OPTION_ASIC_TEMPERATURE));
        }

        depth_ep.set_roi_method(std::make_shared<ds5_auto_exposure_roi_method>(*_hw_monitor));

        if (advanced_mode)
            depth_ep.register_option(RS2_OPTION_DEPTH_UNITS, std::make_shared<depth_scale_option>(*_hw_monitor));
        else
            depth_ep.register_option(RS2_OPTION_DEPTH_UNITS, std::make_shared<const_value_option>("Number of meters represented by a single depth unit",
                                                                                                  0.001f));
        // Metadata registration
        depth_ep.register_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP,    make_uvc_header_parser(&uvc::uvc_header::timestamp));

        // attributes of md_capture_timing
        auto md_prop_offset = offsetof(metadata_raw, mode) +
                offsetof(md_depth_mode, depth_y_mode) +
                offsetof(md_depth_y_normal_mode, intel_capture_timing);

        depth_ep.register_metadata(RS2_FRAME_METADATA_FRAME_COUNTER,    make_attribute_parser(&md_capture_timing::frame_counter, md_capture_timing_attributes::frame_counter_attribute,md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP, make_rs4xx_sensor_ts_parser(make_uvc_header_parser(&uvc::uvc_header::timestamp),
                make_attribute_parser(&md_capture_timing::sensor_timestamp, md_capture_timing_attributes::sensor_timestamp_attribute, md_prop_offset)));

        // attributes of md_capture_stats
        md_prop_offset = offsetof(metadata_raw, mode) +
                offsetof(md_depth_mode, depth_y_mode) +
                offsetof(md_depth_y_normal_mode, intel_capture_stats);

        depth_ep.register_metadata(RS2_FRAME_METADATA_WHITE_BALANCE,    make_attribute_parser(&md_capture_stats::white_balance, md_capture_stat_attributes::white_balance_attribute, md_prop_offset));

        // attributes of md_depth_control
        md_prop_offset = offsetof(metadata_raw, mode) +
                offsetof(md_depth_mode, depth_y_mode) +
                offsetof(md_depth_y_normal_mode, intel_depth_control);

        depth_ep.register_metadata(RS2_FRAME_METADATA_GAIN_LEVEL,        make_attribute_parser(&md_depth_control::manual_gain, md_depth_control_attributes::gain_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE,   make_attribute_parser(&md_depth_control::manual_exposure, md_depth_control_attributes::exposure_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_AUTO_EXPOSURE,     make_attribute_parser(&md_depth_control::auto_exposure_mode, md_depth_control_attributes::ae_mode_attribute, md_prop_offset));

        // md_configuration - will be used for internal validation only
        md_prop_offset = offsetof(metadata_raw, mode) + offsetof(md_depth_mode, depth_y_mode) + offsetof(md_depth_y_normal_mode, intel_configuration);

        depth_ep.register_metadata((rs2_frame_metadata)RS2_FRAME_METADATA_HW_TYPE,          make_attribute_parser(&md_configuration::hw_type, md_configuration_attributes::hw_type_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata)RS2_FRAME_METADATA_SKU_ID,           make_attribute_parser(&md_configuration::sku_id, md_configuration_attributes::sku_id_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata)RS2_FRAME_METADATA_FORMAT,           make_attribute_parser(&md_configuration::format, md_configuration_attributes::format_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata)RS2_FRAME_METADATA_WIDTH,            make_attribute_parser(&md_configuration::width, md_configuration_attributes::width_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata)RS2_FRAME_METADATA_HEIGHT,           make_attribute_parser(&md_configuration::height, md_configuration_attributes::height_attribute, md_prop_offset));

        // Register sensor_base info
        for(auto& element : dev_info)
        {
            if (element.mi == 0) // mi 0 is defines RS4xx Stereo (Depth) interface
            {
                std::map<rs2_camera_info, std::string> camera_info = {{RS2_CAMERA_INFO_DEVICE_NAME, device_name},
                                                                      {RS2_CAMERA_INFO_MODULE_NAME, "Stereo Module"},
                                                                      {RS2_CAMERA_INFO_DEVICE_SERIAL_NUMBER, serial},
                                                                      {RS2_CAMERA_INFO_CAMERA_FIRMWARE_VERSION, static_cast<const char*>(_fw_version)},
                                                                      {RS2_CAMERA_INFO_DEVICE_LOCATION, element.device_path},
                                                                      {RS2_CAMERA_INFO_DEVICE_DEBUG_OP_CODE, std::to_string(static_cast<int>(fw_cmd::GLD))},
                                                                      {RS2_CAMERA_INFO_ADVANCED_MODE, ((advanced_mode)?"YES":"NO")},
                                                                      {RS2_CAMERA_INFO_PRODUCT_ID, pid_hex_str}};
                if (!is_camera_locked.empty())
                    camera_info[RS2_CAMERA_INFO_IS_CAMERA_LOCKED] = is_camera_locked;

                register_sensor_info(_depth_device_idx, camera_info);
            }
        }
    }

    notification ds5_notification_decoder::decode(int value)
    {
        if (value == 0)
            return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_ERROR, "Success" };
        if (value == ds::ds5_notifications_types::hot_laser_power_reduce)
            return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_ERROR, "Hot laser power reduce" };
        if (value == ds::ds5_notifications_types::hot_laser_disable)
            return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_ERROR, "Hot laser disable" };
        if (value == ds::ds5_notifications_types::flag_B_laser_disable)
            return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_ERROR, "Flag B laser disable" };

        return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_NONE, "Unknown error!" };
    }

    rs2_extrinsics ds5_device::get_extrinsics(int from_subdevice, rs2_stream from_stream, int to_subdevice, rs2_stream to_stream)
    {
        auto is_left = [](rs2_stream s) { return s == RS2_STREAM_INFRARED || s == RS2_STREAM_DEPTH; };

        if (from_subdevice == to_subdevice && from_subdevice == 0)
        {
            rs2_extrinsics ext { {1,0,0,0,1,0,0,0,1}, {0,0,0} };

            if (is_left(to_stream) && from_stream == RS2_STREAM_INFRARED2)
            {
                auto table = ds::check_calib<ds::coefficients_table>(*_coefficients_table_raw);
                ext.translation[0] = -0.001f * table->baseline;
                return ext;
            }
            else if (to_stream == RS2_STREAM_INFRARED2 && is_left(from_stream))
            {
                auto table = ds::check_calib<ds::coefficients_table>(*_coefficients_table_raw);
                ext.translation[0] = 0.001f * table->baseline;
                return ext;
            }
        }
        return device::get_extrinsics(from_subdevice, from_stream, to_subdevice, to_stream);
    }
}