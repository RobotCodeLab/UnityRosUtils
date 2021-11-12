using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;

public class ROSClock : MonoBehaviour
{
    private ROSConnection ros;
    private TimeMsg timeStamp;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ClockMsg>("clock");
        timeStamp = new TimeMsg(0, 0);
    }

    // Update is called once per frame
    void Update()
    {
        timeStamp = new TimeMsg((uint)Mathf.FloorToInt(Time.time),
                                (uint)Mathf.FloorToInt((Time.time - Mathf.FloorToInt(Time.time)) * 1e9f));
        ros.Publish("clock", new ClockMsg(timeStamp));
    }

    public TimeMsg getTimeStamp()
    {
        return timeStamp;
    }

    
}
