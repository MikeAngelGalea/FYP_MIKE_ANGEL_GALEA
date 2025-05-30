using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;   // ← for TimeMsg
using RosMessageTypes.Geometry;            // ← for QuaternionMsg & Vector3Msg
using Unity.Robotics.Core;                 // ← for TimeStamp

[RequireComponent(typeof(Rigidbody))]
public class SimpleImuPublisher : MonoBehaviour
{
    [Tooltip("ROS topic to publish IMU data on")]
    public string imuTopic = "/imu/data";

    ROSConnection ros;
    Rigidbody     rb;
    Vector3       lastVel;
    float         lastTime;

    void Start()
    {
        ros      = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(imuTopic);

        rb       = GetComponent<Rigidbody>();
        lastVel  = rb.velocity;
        lastTime = Time.fixedTime;
    }

    void FixedUpdate()
    {
        float now = Time.fixedTime;
        float dt  = now - lastTime;
        lastTime  = now;

        // read angular velocity (rad/s)
        Vector3 angVel = rb.angularVelocity;

        // approximate linear acceleration (m/s²)
        Vector3 linAcc = (rb.velocity - lastVel) / dt;
        lastVel = rb.velocity;

        // build header (uses implicit TimeStamp→TimeMsg)
        var header = new HeaderMsg
        {
            stamp    = new TimeStamp(Clock.time),
            frame_id = gameObject.name
        };

        // build and publish the IMU message
        var imu = new ImuMsg
        {
            header                      = header,
            orientation                 = new QuaternionMsg(0, 0, 0, 1),
            orientation_covariance      = new double[9]{ -1,0,0,  0,0,0,  0,0,0 },
            angular_velocity            = new Vector3Msg(angVel.x, angVel.y, angVel.z),
            angular_velocity_covariance = new double[9]{ 0.01,0,0,  0,0.01,0,  0,0,0.01 },
            linear_acceleration         = new Vector3Msg(linAcc.x, linAcc.y, linAcc.z),
            linear_acceleration_covariance = new double[9]{ 0.1,0,0,  0,0.1,0,  0,0,0 },
        };

        ros.Publish(imuTopic, imu);
    }
}
