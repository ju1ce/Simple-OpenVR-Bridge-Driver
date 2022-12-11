#include "TrackerDevice.hpp"

#include <algorithm>

static void normalizeQuat(ExampleDriver::TrackerDevice::PoseInfo& pose)
{
    //normalize
    double mag = sqrt(pose[3] * pose[3] +
        pose[4] * pose[4] +
        pose[5] * pose[5] +
        pose[6] * pose[6]);

    pose[3] /= mag;
    pose[4] /= mag;
    pose[5] /= mag;
    pose[6] /= mag;
}

ExampleDriver::TrackerDevice::TrackerDevice(std::string serial, std::string role) :
    serial_(serial),
    role_(role)
{
    const auto now = std::chrono::system_clock::now();
    _pose_timestamp = now;
    last_update = now;
}

std::string ExampleDriver::TrackerDevice::GetSerial()
{
    return this->serial_;
}

void ExampleDriver::TrackerDevice::reinit(int msaved, double mtime, double msmooth)
{
    if (msaved < 5)     //prevent having too few values to calculate linear interpolation, and prevent crash on 0
        msaved = 5;

    if (msmooth < 0)
        msmooth = 0;
    else if (msmooth > 0.99)
        msmooth = 0.99;

    max_saved = msaved;
    prev_positions.clear();
    prev_positions.resize(max_saved);
    max_time = mtime;
    smoothing = msmooth;

    //Log("Settings changed! " + std::to_string(msaved) + " " + std::to_string(mtime));
}

void ExampleDriver::TrackerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Setup pose for this frame
    auto pose = this->last_pose_;

    // Update time delta (for working out velocity)
    const auto time_now = std::chrono::system_clock::now();
    const double pose_time_delta_seconds = std::chrono::duration_cast<Seconds>(time_now - _pose_timestamp).count();

    // Update pose timestamp
    _pose_timestamp = time_now;

    // Copy the previous position data
    double previous_position[3] = { 0 };
    std::copy(std::begin(pose.vecPosition), std::end(pose.vecPosition), std::begin(previous_position));

    PoseInfo next_pose;
    if (get_next_pose(Seconds(0), next_pose) != 0)
        return;

    normalizeQuat(next_pose);

    const bool pose_nan = std::any_of(next_pose.begin(), next_pose.end(), [](double d) { return std::isnan(d); });

    if (smoothing == 0 || pose_nan)
    {
        pose.vecPosition[0] = next_pose[0];
        pose.vecPosition[1] = next_pose[1];
        pose.vecPosition[2] = next_pose[2];

        pose.qRotation.w = next_pose[3];
        pose.qRotation.x = next_pose[4];
        pose.qRotation.y = next_pose[5];
        pose.qRotation.z = next_pose[6];
    }
    else
    {
        pose.vecPosition[0] = next_pose[0] * (1 - smoothing) + pose.vecPosition[0] * smoothing;
        pose.vecPosition[1] = next_pose[1] * (1 - smoothing) + pose.vecPosition[1] * smoothing;
        pose.vecPosition[2] = next_pose[2] * (1 - smoothing) + pose.vecPosition[2] * smoothing;

        pose.qRotation.w = next_pose[3] * (1 - smoothing) + pose.qRotation.w * smoothing;
        pose.qRotation.x = next_pose[4] * (1 - smoothing) + pose.qRotation.x * smoothing;
        pose.qRotation.y = next_pose[5] * (1 - smoothing) + pose.qRotation.y * smoothing;
        pose.qRotation.z = next_pose[6] * (1 - smoothing) + pose.qRotation.z * smoothing;
    }
    //normalize
    double mag = sqrt(pose.qRotation.w * pose.qRotation.w +
        pose.qRotation.x * pose.qRotation.x +
        pose.qRotation.y * pose.qRotation.y +
        pose.qRotation.z * pose.qRotation.z);

    pose.qRotation.w /= mag;
    pose.qRotation.x /= mag;
    pose.qRotation.y /= mag;
    pose.qRotation.z /= mag;

    /*
    if (pose_time_delta_seconds > 0)            //unless we get two pose updates at the same time, update velocity so steamvr can do some interpolation
    {
        pose.vecVelocity[0] = 0.8 * pose.vecVelocity[0] + 0.2 * (pose.vecPosition[0] - previous_position[0]) / pose_time_delta_seconds;
        pose.vecVelocity[1] = 0.8 * pose.vecVelocity[1] + 0.2 * (pose.vecPosition[1] - previous_position[1]) / pose_time_delta_seconds;
        pose.vecVelocity[2] = 0.8 * pose.vecVelocity[2] + 0.2 * (pose.vecPosition[2] - previous_position[2]) / pose_time_delta_seconds;
    }
    pose.poseTimeOffset = this->wantedTimeOffset;
    */

    pose.poseTimeOffset = 0;

    //pose.vecVelocity[0] = (pose.vecPosition[0] - previous_position[0]) / pose_time_delta_seconds;
    //pose.vecVelocity[1] = (pose.vecPosition[1] - previous_position[1]) / pose_time_delta_seconds;
    //pose.vecVelocity[2] = (pose.vecPosition[2] - previous_position[2]) / pose_time_delta_seconds;

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

void ExampleDriver::TrackerDevice::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

int ExampleDriver::TrackerDevice::get_next_pose(Seconds time_offset, PoseInfo& next_pose) const
{
    int statuscode = 0;

    const auto time_now = std::chrono::system_clock::now();
    const auto req_time = time_now - time_offset;
    double new_time = std::chrono::duration_cast<Seconds>(last_update - req_time).count();

    if (new_time < -0.2)      //limit prediction to max 0.2 second into the future to prevent your feet from being yeeted into oblivion
    {
        new_time = -0.2;
        statuscode = 1;
    }

    int curr_saved = 0;

    double avg_time = 0;
    double avg_time2 = 0;
    for (int i = 0; i < max_saved; i++)
    {
        if (prev_positions[i].time < 0)
            break;
        curr_saved++;
        avg_time += prev_positions[i].time;
        avg_time2 += (prev_positions[i].time * prev_positions[i].time);
    }

    if (curr_saved == 0)
    {
        return -1;
    }
    else if (curr_saved < 4)
    {
        next_pose = prev_positions.front().pose;
        return statuscode;
    }

    avg_time /= curr_saved;
    avg_time2 /= curr_saved;
    const double st = std::sqrt(avg_time2 - avg_time * avg_time);

    for (int i = 0; i < next_pose.size(); i++)
    {
        double avg_val = 0;
        double avg_tval = 0;
        for (int ii = 0; ii < curr_saved; ii++)
        {
            const PrevPose& prev_pose = prev_positions[ii];
            avg_val += prev_pose.pose[i];
            avg_tval += (prev_pose.time * prev_pose.pose[i]);
        }
        avg_val /= curr_saved;
        avg_tval /= curr_saved;

        const double m = (avg_tval - (avg_val * avg_time)) / (st * st);
        const double y = avg_val + m * (new_time - avg_time);

        next_pose[i] = y;
    }

    return statuscode;
}

void ExampleDriver::TrackerDevice::save_current_pose(double a, double b, double c, double w, double x, double y, double z, Seconds time_offset)
{
    PoseInfo next_pose;
    int pose_valid = get_next_pose(time_offset, next_pose);

    double dot = x * next_pose[4] + y * next_pose[5] + z * next_pose[6] + w * next_pose[3];

    if (dot < 0)
    {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }

    if (max_time == 0)
    {
        this->last_update = std::chrono::system_clock::now();
        PrevPose& prev_pose = prev_positions.front();
        prev_pose.time = time_offset.count();
        prev_pose.pose[0] = a;
        prev_pose.pose[1] = b;
        prev_pose.pose[2] = c;
        prev_pose.pose[3] = w;
        prev_pose.pose[4] = x;
        prev_pose.pose[5] = y;
        prev_pose.pose[6] = z;

        return;
    }

    //update times
    const auto time_now = std::chrono::system_clock::now();
    const double time_since_update = std::chrono::duration_cast<Seconds>(time_now - this->last_update).count();
    this->last_update = time_now;

    for (PrevPose& prev_pose : prev_positions)
    {
        double& prev_time = prev_pose.time;
        if (prev_time >= 0)
            prev_time += time_since_update;
        if (prev_time > max_time)
            prev_time = -1;
    }

    double time = time_offset.count();
    // double offset = (rand() % 100) / 10000.;
    // time += offset;
    // printf("%f %f\n", time, offset);

    //Log("Time: " + std::to_string(time));

    double dist = sqrt(pow(next_pose[0] - a, 2) + pow(next_pose[1] - b, 2) + pow(next_pose[2] - c, 2));
    if (pose_valid == 0 && dist > 0.5)
    {
        Log("Dropped a pose! its error was " + std::to_string(dist));
        Log("Height vs predicted height:" + std::to_string(b) + " " + std::to_string(next_pose[1]));
        return;
    }

    dist = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
    if (dist > 10)
    {
        Log("Dropped a pose! Was outside of playspace: " + std::to_string(dist));
        return;
    }

    if (time > max_time)
        return;

    if (prev_positions[max_saved - 1].time < time && prev_positions[max_saved - 1].time >= 0)
        return;

    int i = 0;
    while (prev_positions[i].time < time && prev_positions[i].time >= 0)
        i++;

    for (int j = max_saved - 1; j > i; j--)
    {
        if (prev_positions[j - 1].time >= 0)
        {
            prev_positions[j] = prev_positions[j - 1];
        }
        else
        {
            prev_positions[j].time = -1;
        }
    }
    prev_positions[i].time = time;
    prev_positions[i].pose[0] = a;
    prev_positions[i].pose[1] = b;
    prev_positions[i].pose[2] = c;
    prev_positions[i].pose[3] = w;
    prev_positions[i].pose[4] = x;
    prev_positions[i].pose[5] = y;
    prev_positions[i].pose[6] = z;
    /*                                                 //for debugging
    Log("------------------------------------------------");
    for (int i = 0; i < max_saved; i++)
    {
        Log("Time: " + std::to_string(prev_positions[i][0]));
        Log("Position x: " + std::to_string(prev_positions[i][1]));
    }
    */
    return;
}

/*
void ExampleDriver::TrackerDevice::UpdatePos(double a, double b, double c, double time, double smoothing)
{
    this->wantedPose[0] = (1 - smoothing) * this->wantedPose[0] + smoothing * a;
    this->wantedPose[1] = (1 - smoothing) * this->wantedPose[1] + smoothing * b;
    this->wantedPose[2] = (1 - smoothing) * this->wantedPose[2] + smoothing * c;

    this->wantedTimeOffset = time;

}

void ExampleDriver::TrackerDevice::UpdateRot(double qw, double qx, double qy, double qz, double time, double smoothing)
{
    //lerp
    double dot = qx * this->wantedPose[4] + qy * this->wantedPose[5] + qz * this->wantedPose[6] + qw * this->wantedPose[3];

    if (dot < 0)
    {
        this->wantedPose[3] = smoothing * qw - (1 - smoothing) * this->wantedPose[3];
        this->wantedPose[4] = smoothing * qx - (1 - smoothing) * this->wantedPose[4];
        this->wantedPose[5] = smoothing * qy - (1 - smoothing) * this->wantedPose[5];
        this->wantedPose[6] = smoothing * qz - (1 - smoothing) * this->wantedPose[6];
    }
    else
    {
        this->wantedPose[3] = smoothing * qw + (1 - smoothing) * this->wantedPose[3];
        this->wantedPose[4] = smoothing * qx + (1 - smoothing) * this->wantedPose[4];
        this->wantedPose[5] = smoothing * qy + (1 - smoothing) * this->wantedPose[5];
        this->wantedPose[6] = smoothing * qz + (1 - smoothing) * this->wantedPose[6];
    }
    //normalize
    double mag = sqrt(this->wantedPose[3] * this->wantedPose[3] +
        this->wantedPose[4] * this->wantedPose[4] +
        this->wantedPose[5] * this->wantedPose[5] +
        this->wantedPose[6] * this->wantedPose[6]);

    this->wantedPose[3] /= mag;
    this->wantedPose[4] /= mag;
    this->wantedPose[5] /= mag;
    this->wantedPose[6] /= mag;

    this->wantedTimeOffset = time;

}
*/

DeviceType ExampleDriver::TrackerDevice::GetDeviceType()
{
    return DeviceType::TRACKER;
}

vr::TrackedDeviceIndex_t ExampleDriver::TrackerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

vr::EVRInitError ExampleDriver::TrackerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating tracker " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 3);
    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "apriltag_tracker");

    // Opt out of hand selection
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);

    // Set up a render model path
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "{htc}/rendermodels/vr_tracker_vive_1_0");

    // Set controller profile
    //GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{apriltagtrackers}/input/example_tracker_bindings.json");

    // Set the icon

    if (this->serial_.find("Apriltag") == std::string::npos)
    {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, "{apriltagtrackers}/icons/tracker_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{apriltagtrackers}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{apriltagtrackers}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{apriltagtrackers}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{apriltagtrackers}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{apriltagtrackers}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{apriltagtrackers}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{apriltagtrackers}/icons/tracker_not_ready.png");
    }
    else
    {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, "{apriltagtrackers}/icons/apriltag_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{apriltagtrackers}/icons/apriltag_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{apriltagtrackers}/icons/apriltag_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{apriltagtrackers}/icons/apriltag_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{apriltagtrackers}/icons/apriltag_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{apriltagtrackers}/icons/apriltag_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{apriltagtrackers}/icons/apriltag_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{apriltagtrackers}/icons/apriltag_not_ready.png");
    }
    /*
    char id = this->serial_.at(12);
    std::string role = "";
    switch (id)
    {
    case '0':
        role = "vive_tracker_waist"; break;
    case '0':
        role = "vive_tracker_left_foot"; break;
    case '1':
        role = "vive_tracker_right_foot"; break;
    }
    */

    //set role, role hint and everything else to ensure trackers are detected as trackers and not controllers

    std::string rolehint = "vive_tracker";
    if (role_ == "TrackerRole_LeftFoot")
        rolehint = "vive_tracker_left_foot";
    else if (role_ == "TrackerRole_RightFoot")
        rolehint = "vive_tracker_right_foot";
    else if (role_ == "TrackerRole_Waist")
        rolehint = "vive_tracker_waist";

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, rolehint.c_str());

    vr::VRProperties()->SetInt32Property(props, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
    vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerHandSelectionPriority_Int32, -1);

    std::string l_registeredDevice("/devices/apriltagtrackers/");
    l_registeredDevice.append(serial_);

    Log("Setting role " + role_ + " to " + l_registeredDevice);

    if(rolehint != "vive_tracker")
        vr::VRSettings()->SetString(vr::k_pch_Trackers_Section, l_registeredDevice.c_str(), role_.c_str());

    return vr::EVRInitError::VRInitError_None;
}

void ExampleDriver::TrackerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void ExampleDriver::TrackerDevice::EnterStandby()
{
}

void* ExampleDriver::TrackerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void ExampleDriver::TrackerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t ExampleDriver::TrackerDevice::GetPose()
{
    return last_pose_;
}
