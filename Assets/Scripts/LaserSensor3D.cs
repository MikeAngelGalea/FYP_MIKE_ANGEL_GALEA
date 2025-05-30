using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

public class LaserSensor3D
{
    // Sensor parameters
    float RangeMetersMin, RangeMetersMax;
    float fov_horizontal, fov_vertical;
    float angularResolution_horizontal, angularResolution_vertical;

    // Precomputed scan angles
    int NumMeasurementsPerScan_h, NumMeasurementsPerScan_v;
    float[] scanAngleArray_h, scanAngleArray_v;

    // ROS header frame
    string FrameId;

    // Buffer to hold packed XYZIntensity floats
    uint numPoints;
    uint raw_data_len;
    byte[] raw_data;

    GameObject laser_sensor_link;

    public LaserSensor3D(
        GameObject _laser_sensor_link,
        float _RangeMetersMin,
        float _RangeMetersMax,
        float _fov_horizontal,
        float _fov_vertical,
        float _angularResolution_vertical,
        float _angularResolution_horizontal)
    {
        // Clamp FOVs to [0,360]
        fov_horizontal = Mathf.Min(_fov_horizontal, 360f);
        fov_vertical   = Mathf.Min(_fov_vertical,   360f);
        angularResolution_horizontal = _angularResolution_horizontal;
        angularResolution_vertical   = _angularResolution_vertical;

        RangeMetersMin = _RangeMetersMin;
        RangeMetersMax = _RangeMetersMax;

        laser_sensor_link = _laser_sensor_link;
        FrameId = laser_sensor_link.name;

        // Compute horizontal angles
        float start_h = -fov_horizontal * 0.5f, end_h = fov_horizontal * 0.5f;
        NumMeasurementsPerScan_h = Mathf.FloorToInt((end_h - start_h) / angularResolution_horizontal) + 1;
        if (fov_horizontal == 360f) NumMeasurementsPerScan_h--;
        scanAngleArray_h = new float[NumMeasurementsPerScan_h];
        for (int i = 0; i < NumMeasurementsPerScan_h; i++)
            scanAngleArray_h[i] = start_h + i * angularResolution_horizontal;

        // Compute vertical angles
        float start_v = -fov_vertical * 0.5f, end_v = fov_vertical * 0.5f;
        NumMeasurementsPerScan_v = Mathf.FloorToInt((end_v - start_v) / angularResolution_vertical) + 1;
        if (fov_vertical == 360f) NumMeasurementsPerScan_v--;
        scanAngleArray_v = new float[NumMeasurementsPerScan_v];
        for (int j = 0; j < NumMeasurementsPerScan_v; j++)
            scanAngleArray_v[j] = start_v + j * angularResolution_vertical;

        // How many points and bytes?
        numPoints    = (uint)(NumMeasurementsPerScan_h * NumMeasurementsPerScan_v);
        raw_data_len = 16u * numPoints; // 4 floats × 4 bytes each
        raw_data     = new byte[(int)raw_data_len]; // cast uint→int
    }

    public PointCloud2Msg getScanMsg()
    {
        var sensorTf = laser_sensor_link.transform;
        int idx = 0;

        // Raycast each beam
        for (int i = 0; i < NumMeasurementsPerScan_h; i++)
        {
            for (int j = 0; j < NumMeasurementsPerScan_v; j++)
            {
                float θ = Mathf.Deg2Rad * scanAngleArray_h[i];
                float φ = Mathf.Deg2Rad * scanAngleArray_v[j];
                var localDir = new Vector3(
                    Mathf.Cos(φ) * Mathf.Sin(θ),
                    -Mathf.Sin(φ),
                    Mathf.Cos(φ) * Mathf.Cos(θ));
                var dirWorld = sensorTf.rotation * localDir;

                Ray ray = new Ray(sensorTf.position + dirWorld * RangeMetersMin, dirWorld);
                bool hitSomething = Physics.Raycast(ray, out var hit, RangeMetersMax);

                // Pack (z, –x, y, intensity) into raw_data as 4 floats
                float x = hitSomething ? hit.point.z - sensorTf.position.z : float.MaxValue;
                float y = hitSomething ? -(hit.point.x - sensorTf.position.x) : float.MaxValue;
                float z = hitSomething ? hit.point.y - sensorTf.position.y : float.MaxValue;
                float intensity = 0f;

                BitConverter.GetBytes(x).CopyTo(raw_data, idx * 16);
                BitConverter.GetBytes(y).CopyTo(raw_data, idx * 16 + 4);
                BitConverter.GetBytes(z).CopyTo(raw_data, idx * 16 + 8);
                BitConverter.GetBytes(intensity).CopyTo(raw_data, idx * 16 + 12);

                idx++;
            }
        }

        // Build the ROS2 message
        var timestamp = new TimeStamp(Clock.time); // implicit → TimeMsg

        // Build Header via initializer (ROS2 Header only has stamp & frame_id)
        var header = new HeaderMsg
        {
            stamp    = timestamp,
            frame_id = FrameId
        };

        var msg = new PointCloud2Msg
        {
            header       = header,
            height       = 1u,
            width        = numPoints,
            fields       = new PointFieldMsg[]
            {
                new PointFieldMsg("x",  0u, PointFieldMsg.FLOAT32, 1u),
                new PointFieldMsg("y",  4u, PointFieldMsg.FLOAT32, 1u),
                new PointFieldMsg("z",  8u, PointFieldMsg.FLOAT32, 1u),
                new PointFieldMsg("i", 12u, PointFieldMsg.FLOAT32, 1u),
            },
            is_bigendian = false,
            point_step   = 16u,
            row_step     = raw_data_len,
            data         = raw_data,
            is_dense     = false,
        };

        return msg;
    }
}
