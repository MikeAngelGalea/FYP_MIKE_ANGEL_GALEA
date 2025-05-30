using System;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;

namespace Unity.Robotics.Core
{
    public readonly struct TimeStamp
    {
        // Number of nanoseconds per second
        public const double k_NanosecondsInSecond = 1e9;

        // Match the ROS TimeMsg fields exactly:
        public readonly uint Seconds;
        public readonly uint NanoSeconds;

        /// <summary>
        /// Construct from a Unity time in seconds (e.g. Time.time or Clock.time).
        /// </summary>
        public TimeStamp(double timeInSeconds)
        {
            double secDouble  = Math.Floor(timeInSeconds);
            double nsecDouble = (timeInSeconds - secDouble) * k_NanosecondsInSecond;

            Seconds     = (uint)secDouble;
            NanoSeconds = (uint)nsecDouble;
        }

        /// <summary>
        /// Construct directly from ROS Time message fields.
        /// </summary>
        public TimeStamp(uint sec, uint nsec)
        {
            Seconds     = sec;
            NanoSeconds = nsec;
        }

        /// <summary>
        /// Implicitly convert our TimeStamp → ROS2 TimeMsg.
        /// </summary>
        public static implicit operator TimeMsg(TimeStamp stamp)
        {
            // TimeMsg ctor is: TimeMsg(int sec, uint nanosec)
            return new TimeMsg((int)stamp.Seconds, stamp.NanoSeconds);
        }

        /// <summary>
        /// Implicitly convert a ROS2 TimeMsg → our TimeStamp.
        /// </summary>
        public static implicit operator TimeStamp(TimeMsg stamp)
        {
            // stamp.sec is int, our ctor wants uint:
            return new TimeStamp((uint)stamp.sec, stamp.nanosec);
        }
    }
}
