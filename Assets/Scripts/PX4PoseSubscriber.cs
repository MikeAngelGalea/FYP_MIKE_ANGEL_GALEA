using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;  // or .Odometry

public class PX4PoseSubscriber : MonoBehaviour
{
    public string odomTopic = "/fmu/out/vehicle_odometry";
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(odomTopic, OnOdometry);
    }

    void OnOdometry(OdometryMsg msg)
    {
        // ROS ENU → Unity FLU (X-forward, Y-left, Z-up)
        Vector3 pos = new Vector3(
            (float)msg.pose.pose.position.x,
            (float)msg.pose.pose.position.z,
            (float)-msg.pose.pose.position.y
        );
        transform.localPosition = pos;

        Quaternion rotRos = new Quaternion(
            (float)msg.pose.pose.orientation.x,
            (float)msg.pose.pose.orientation.y,
            (float)msg.pose.pose.orientation.z,
            (float)msg.pose.pose.orientation.w
        );
        // convert to Unity coordinate frame
        transform.localRotation = new Quaternion(-rotRos.y, rotRos.z, rotRos.x, -rotRos.w);
    }
}
