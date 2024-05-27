using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;


public class JointStatesSubscriber : MonoBehaviour
{
    ROSConnection m_Ros;
    ArticulationBody[] m_UrdfJoint;
    public string m_TopicName = "/joint_states";


    // Start is called before the first frame update
    void Start()
    {
        m_UrdfJoint = GetComponentsInChildren<ArticulationBody>();

        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.Subscribe<JointStateMsg>(m_TopicName, SetJoints);

    }

    // Update is called once per frame
    void Update()
    {

    }

    void SetJoints(JointStateMsg msg)
    {
        // Set the joint angles if the type is revolute
        foreach (var body in m_UrdfJoint)
        {
           if (body.jointType == ArticulationJointType.RevoluteJoint)
            {
                var jointXDrive = body.xDrive;
                jointXDrive.target = (float)msg.position[body.index - 1] * Mathf.Rad2Deg;
                body.xDrive = jointXDrive;
            }
        }

    }

    public void DisableArticulationBody(GameObject robot)
    {
        ArticulationBody[] articulationBody = robot.GetComponentsInChildren<ArticulationBody>();
        foreach (var body in articulationBody)
        {
            body.enabled = false;
        }
    }

    private void OnDisable()
    {
        UnsubscribeFromTopic();
    }

    void OnApplicationQuit()
    {
        UnsubscribeFromTopic();
    }

    private void UnsubscribeFromTopic()
    {
        if (m_Ros != null)
        {
            m_Ros.Unsubscribe(m_TopicName);
            Debug.Log("Unsubscribed from " + m_TopicName + " topic.");
            Destroy(m_Ros);
        }
    }
}
