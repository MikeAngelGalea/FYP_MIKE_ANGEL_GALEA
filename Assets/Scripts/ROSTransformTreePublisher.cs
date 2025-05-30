using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.SlamExample;
using UnityEngine;

public class ROSTransformTreePublisher : MonoBehaviour
{
    const string k_TfTopic = "/tf";
    
    [SerializeField]
    double m_PublishRateHz = 20f;
    [SerializeField]
    List<string> m_GlobalFrameIds = new List<string> { "map", "odom" };
    [SerializeField]
    GameObject m_RootGameObject;
    
    double m_LastPublishTimeSeconds;

    TransformTreeNode m_TransformRoot;
    ROSConnection m_ROS;

    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    void Start()
    {
        if (m_RootGameObject == null)
        {
            Debug.LogWarning($"No GameObject explicitly defined as {nameof(m_RootGameObject)}, so using {name} as root.");
            m_RootGameObject = gameObject;
        }

        m_ROS = ROSConnection.GetOrCreateInstance();
        m_TransformRoot = new TransformTreeNode(m_RootGameObject);
        m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
        m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;
    }

    static void PopulateTFList(List<TransformStampedMsg> tfList, TransformTreeNode tfNode)
    {
        foreach (var childTf in tfNode.Children)
        {
            tfList.Add(TransformTreeNode.ToTransformStamped(childTf));

            if (!childTf.IsALeafNode)
            {
                PopulateTFList(tfList, childTf);
            }
        }
    }

    void PublishMessage()
    {
        var tfMessageList = new List<TransformStampedMsg>();

        // 1) Map → Odom (or whatever global frames you defined)
        if (m_GlobalFrameIds.Count > 0)
        {
            var headerRoot = new HeaderMsg
            {
                // implicit conversion: TimeStamp → TimeMsg
                stamp    = new TimeStamp(Clock.time),
                frame_id = m_GlobalFrameIds.Last()
            };
            var tfRootToGlobal = new TransformStampedMsg(
                headerRoot,
                m_TransformRoot.name, 
                m_TransformRoot.Transform.To<FLU>()
            );
            tfMessageList.Add(tfRootToGlobal);
        }
        else
        {
            Debug.LogWarning($"No {nameof(m_GlobalFrameIds)} specified, transform tree will be entirely local coordinates.");
        }

        // 2) If you have multiple globals chained (e.g. map→odom→base_link), publish identity links
        for (var i = 1; i < m_GlobalFrameIds.Count; ++i)
        {
            var headerChain = new HeaderMsg
            {
                stamp    = new TimeStamp(Clock.time),
                frame_id = m_GlobalFrameIds[i - 1]
            };
            var tfGlobalToGlobal = new TransformStampedMsg(
                headerChain,
                m_GlobalFrameIds[i],
                new TransformMsg()  // identity transform
            );
            tfMessageList.Add(tfGlobalToGlobal);
        }

        // 3) Publish the rest of the scene graph
        PopulateTFList(tfMessageList, m_TransformRoot);

        // 4) Publish the assembled TFMessageMsg
        var tfMessage = new TFMessageMsg(tfMessageList.ToArray());
        m_ROS.Publish(k_TfTopic, tfMessage);

        m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
    }

    void Update()
    {
        if (ShouldPublishMessage)
        {
            PublishMessage();
        }
    }
}
