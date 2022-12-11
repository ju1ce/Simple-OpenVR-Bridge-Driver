#pragma once

#include <chrono>
#include "cmath_fix.h"

#include <linalg.h>

#include <Driver/IVRDevice.hpp>
#include <Native/DriverFactory.hpp>

#include <array>
#include <thread>
#include <sstream>
#include <iostream>
#include <string>

namespace ExampleDriver {
    class TrackerDevice : public IVRDevice {
        public:
            // [position[x, y, z], rotation[w, x, y, z]]
            using PoseInfo = std::array<double, 7>;

            using Seconds = std::chrono::duration<double>;

            TrackerDevice(std::string serial, std::string role);
            ~TrackerDevice() = default;

            // Inherited via IVRDevice
            virtual std::string GetSerial() override;
            virtual void Update() override;
            virtual vr::TrackedDeviceIndex_t GetDeviceIndex() override;
            virtual DeviceType GetDeviceType() override;
            virtual void Log(std::string message);

            virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
            virtual void Deactivate() override;
            virtual void EnterStandby() override;
            virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
            virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
            virtual vr::DriverPose_t GetPose() override;

            void reinit(int msaved, double mtime, double msmooth);
            void save_current_pose(double a, double b, double c, double qw, double qx, double qy, double qz, Seconds time_offset);
            int get_next_pose(Seconds time_offset, PoseInfo& pred) const;

    private:
        vr::TrackedDeviceIndex_t device_index_ = vr::k_unTrackedDeviceIndexInvalid;
        std::string serial_;
        std::string role_;

        std::chrono::system_clock::time_point _pose_timestamp;

        vr::DriverPose_t last_pose_ = IVRDevice::MakeDefaultPose();

        struct PrevPose {
            double time = -1;
            PoseInfo pose;
        };

        std::vector<PrevPose> prev_positions; // koliko cajta nazaj se je naredl, torej min-->max
        std::chrono::system_clock::time_point last_update;
        double max_time = 1;
        double smoothing = 0;

    };
};
