//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class PosRotMsg : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/PosRot";
        public override string RosMessageName => k_RosMessageName;

        //  PosRot.msg (updated for LiDAR)
        //  Position Arrays
        public float[] positions_x;
        //  X coordinates of all points
        public float[] positions_y;
        //  Y coordinates of all points
        public float[] positions_z;
        //  Z coordinates of all points
        //  Sensor Orientation
        public float rotation_x;
        //  Sensor's rotation (quaternion)
        public float rotation_y;
        public float rotation_z;
        public float rotation_w;

        public PosRotMsg()
        {
            this.positions_x = new float[0];
            this.positions_y = new float[0];
            this.positions_z = new float[0];
            this.rotation_x = 0.0f;
            this.rotation_y = 0.0f;
            this.rotation_z = 0.0f;
            this.rotation_w = 0.0f;
        }

        public PosRotMsg(float[] positions_x, float[] positions_y, float[] positions_z, float rotation_x, float rotation_y, float rotation_z, float rotation_w)
        {
            this.positions_x = positions_x;
            this.positions_y = positions_y;
            this.positions_z = positions_z;
            this.rotation_x = rotation_x;
            this.rotation_y = rotation_y;
            this.rotation_z = rotation_z;
            this.rotation_w = rotation_w;
        }

        public static PosRotMsg Deserialize(MessageDeserializer deserializer) => new PosRotMsg(deserializer);

        private PosRotMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.positions_x, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.positions_y, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.positions_z, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.rotation_x);
            deserializer.Read(out this.rotation_y);
            deserializer.Read(out this.rotation_z);
            deserializer.Read(out this.rotation_w);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.positions_x);
            serializer.Write(this.positions_x);
            serializer.WriteLength(this.positions_y);
            serializer.Write(this.positions_y);
            serializer.WriteLength(this.positions_z);
            serializer.Write(this.positions_z);
            serializer.Write(this.rotation_x);
            serializer.Write(this.rotation_y);
            serializer.Write(this.rotation_z);
            serializer.Write(this.rotation_w);
        }

        public override string ToString()
        {
            return "PosRotMsg: " +
            "\npositions_x: " + System.String.Join(", ", positions_x.ToList()) +
            "\npositions_y: " + System.String.Join(", ", positions_y.ToList()) +
            "\npositions_z: " + System.String.Join(", ", positions_z.ToList()) +
            "\nrotation_x: " + rotation_x.ToString() +
            "\nrotation_y: " + rotation_y.ToString() +
            "\nrotation_z: " + rotation_z.ToString() +
            "\nrotation_w: " + rotation_w.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
