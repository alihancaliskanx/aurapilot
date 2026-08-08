/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  Camera driver for cameras that implement the older MAVLink camera protocol
 */
#pragma once

#include "AP_Camera_Backend.h"

#if AP_CAMERA_MAVLINK_ENABLED

class AP_Camera_MAVLink : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_MAVLink);

    // entry point to actually take a picture
    bool trigger_pic() override;

    // configure camera
    void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time) override;

    // handle camera control message
    void control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id) override;

    // AURA: stay invisible to the GCS as a camera.
    //
    // This backend has no camera of its own: it only rebroadcasts
    // DO_DIGICAM_CONTROL to the other components (see trigger_pic), and the
    // companion computer is what actually holds the lens. Answering
    // CAMERA_INFORMATION would therefore advertise a camera nobody can drive.
    //
    // It also costs us stream recording. AP_Camera_Backend hardcodes
    // flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE, so the moment we answer, QGC
    // decides a real MAVLink camera is present, drops SimulatedCameraControl
    // for VehicleCameraControl, and capturesVideo() goes false. That kills the
    // only path in QGC that writes the incoming stream to disk
    // (VideoManager::startRecording is reached from SimulatedCameraControl
    // alone). Stock QGC behaves the same way - the fault is not in the GCS.
    //
    // Staying quiet keeps SimulatedCameraControl in place, so the ordinary
    // record button works in ANY unmodified QGC, while CAM1_TYPE stays MAVLink
    // and the mission photo trigger (203) is untouched. What we give up is the
    // camera pane's shutter button, which we do not use - photos come from the
    // mission (see CLAUDE.md "Gorev Fotografi Sistemi").
    //
    // Vehicle-side recording is deliberately unaffected: video_record_service
    // is a black box that records whenever the vehicle is armed and announces
    // itself as MAV_COMP_ID_ONBOARD_COMPUTER, never as a camera.
    void send_camera_information(mavlink_channel_t chan) const override {}
};

#endif // AP_CAMERA_MAVLINK_ENABLED
