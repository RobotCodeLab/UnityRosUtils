using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class ROSIMU : MonoBehaviour
{

    public string frame_id = "base_link";
    public string topicName = "imu/data";

    private ROSClock clock;
    private ROSConnection ros;


    private Vector3 acceleration;
    private Vector3 velocity;
    private Vector3 lastPosition;
    private Vector3 lastVelocity;
    private Quaternion angularVelocity;
    private Quaternion lastRotation;

    // Start is called before the first frame update
    void Start()
    {
        clock = GameObject.FindObjectOfType<ROSClock>();
        ros = ROSConnection.GetOrCreateInstance();

        if(clock == null)
            Debug.LogError("IMU Error: Cannot find clock");

        ros.RegisterPublisher<ImuMsg>(topicName);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if(lastRotation != null)
            angularVelocity = Quaternion.Inverse(lastRotation) * this.transform.rotation;
        lastRotation = this.transform.rotation;
        if(lastPosition != null)
            velocity = (this.transform.position - lastPosition) / Time.fixedDeltaTime;
        lastPosition = this.transform.position;
        if(lastVelocity != null)
            acceleration = (velocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = velocity;

        ImuMsg message = new ImuMsg();
        
        message.header.frame_id = frame_id;
        message.header.stamp = clock.getTimeStamp();

        message.orientation.w = -this.gameObject.transform.rotation.w;
        message.orientation.x = this.gameObject.transform.rotation.z;
        message.orientation.y = -this.gameObject.transform.rotation.x;
        message.orientation.z = this.gameObject.transform.rotation.y;

        
        message.angular_velocity.x = angularVelocity.eulerAngles.z / Time.fixedDeltaTime;
        message.angular_velocity.y = -angularVelocity.eulerAngles.x / Time.fixedDeltaTime;
        message.angular_velocity.z = angularVelocity.eulerAngles.y / Time.fixedDeltaTime;

        message.linear_acceleration.x = acceleration.z;
        message.linear_acceleration.y = -acceleration.x;
        message.linear_acceleration.z = acceleration.y;

        ros.Publish(topicName, message);
    }
}
