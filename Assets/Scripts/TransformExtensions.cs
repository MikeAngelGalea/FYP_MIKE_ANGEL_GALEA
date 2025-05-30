using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public static class TransformExtensions
{
    /// <summary>
    /// Converts a Unity Transform to a ROS TransformMsg (position+rotation) in FLU frame.
    /// </summary>
    public static TransformMsg ToROSTransform(this Transform tfUnity)
    {
        return new TransformMsg(
            tfUnity.localPosition.To<FLU>(),
            tfUnity.localRotation.To<FLU>()
        );
    }

    /// <summary>
    /// Wraps a Unity Transform in a TransformStampedMsg with a Header (timestamp, frame_id).
    /// </summary>
    /// <param name="timeStamp">Time since startup in seconds.</param>
    public static TransformStampedMsg ToROSTransformStamped(this Transform tfUnity, double timeStamp)
    {
        // Convert our TimeStamp struct to a ROS TimeMsg implicitly
        TimeMsg stamp = new TimeStamp(timeStamp);

        // Build the HeaderMsg via object initializer (no seq field in ROS2 Header)
        var header = new HeaderMsg
        {
            stamp    = stamp,
            frame_id = tfUnity.parent != null
                         ? tfUnity.parent.gameObject.name
                         : string.Empty
        };

        // Child frame ID = this GameObject's name
        string childFrameId = tfUnity.gameObject.name;

        // Construct and return the TransformStampedMsg
        return new TransformStampedMsg(
            header,
            childFrameId,
            tfUnity.ToROSTransform()
        );
    }
}
