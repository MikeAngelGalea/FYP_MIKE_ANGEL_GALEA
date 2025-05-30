using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core;

[RequireComponent(typeof(ROSConnection))]
public class LaserSensor3DPublisher : MonoBehaviour
{
    [Header("LiDAR Mount")]
    [Tooltip("The child GameObject that represents the sensor frame (its position/orientation is the raycast origin).")]
    public GameObject lidarLink;

    [Header("Scan Parameters")]
    [Tooltip("Minimum range (meters)")]
    public float rangeMin = 0.1f;
    [Tooltip("Maximum range (meters)")]
    public float rangeMax = 10f;
    [Tooltip("Horizontal field of view (degrees)")]
    public float horizontalFOV = 360f;
    [Tooltip("Vertical field of view (degrees)")]
    public float verticalFOV = 30f;
    [Tooltip("Horizontal angular resolution (degrees)")]
    public float horizontalResolution = 0.5f;
    [Tooltip("Vertical angular resolution (degrees)")]
    public float verticalResolution = 1f;

    [Header("ROS Settings")]
    [Tooltip("ROS topic for the PointCloud2 messages")]
    public string pointCloudTopic = "/point_cloud";

    ROSConnection  m_Ros;
    LaserSensor3D  m_Scanner;

    void Start()
    {
        if (lidarLink == null)
        {
            Debug.LogError("Please assign the lidarLink GameObject in the inspector.");
            enabled = false;
            return;
        }

        // 1) Grab or create the ROSConnection component
        m_Ros = GetComponent<ROSConnection>();

        // 2) Instantiate your scanner implementation
        m_Scanner = new LaserSensor3D(
            lidarLink,
            rangeMin,
            rangeMax,
            horizontalFOV,
            verticalFOV,
            verticalResolution,
            horizontalResolution);

        // 3) Register the PointCloud2 publisher
        m_Ros.RegisterPublisher<PointCloud2Msg>(pointCloudTopic);
    }

    void FixedUpdate()
    {
        // 4) Each physics tick, generate and publish a new scan
        var msg = m_Scanner.getScanMsg();
        m_Ros.Publish(pointCloudTopic, msg);
    }
}
