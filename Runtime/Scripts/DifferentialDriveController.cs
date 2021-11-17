using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DifferentialDriveController : MonoBehaviour
{
    public float wheelSeparation = 0.37559f;
    public float wheelRadius = 0.098f;
    public bool fourWheelDrive = false;

    public GameObject frontRightWheelObject;
    public GameObject frontLeftWheelObject;
    public GameObject rearRightWheelObject;
    public GameObject rearLeftWheelObject;

    private ROSConnection ros;
    private TFSystem odomStream;

    private ArticulationBody frontRightWheel;
    private ArticulationBody rearRightWheel;
    private ArticulationBody frontLeftWheel;
    private ArticulationBody rearLeftWheel;

    private float forwardRate = 0.0f;
    private float rotationRate = 0.0f;

    private float poseEncoderX = 0.0f;
    private float poseEncoderY = 0.0f;
    private float poseEncoderTheta = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        //listen for cmd_vel
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/jackal_velocity_controller/cmd_vel", cmdVelCallback);
        //odomStream = new TFStream(null, "odom", "/odom");
        TFSystem system = TFSystem.GetOrCreateInstance();
        //system.GetTransformStream("odom").Add((long) Time.time, new Vector3(1.0f, 1.0f, 1.0f), Quaternion.identity);

        //get and store joints for each wheel
        if(frontRightWheelObject == null || frontLeftWheelObject == null)
            Debug.LogError("Differential Drive Error: Please specify wheel gameobjects");

        frontRightWheel = frontRightWheelObject.GetComponent<ArticulationBody>();
        frontLeftWheel = frontLeftWheelObject.GetComponent<ArticulationBody>();

        if(fourWheelDrive)
        {
            if(rearRightWheelObject == null || rearLeftWheelObject == null)
                Debug.LogError("Differential Drive Error: Four Wheel Drive activated but four wheels not specified");    

            rearRightWheel = rearRightWheelObject.GetComponent<ArticulationBody>();
            rearLeftWheel = rearLeftWheelObject.GetComponent<ArticulationBody>();
        }
        
    }

    void cmdVelCallback(TwistMsg cmdVel)
    {
        //I dont have to follow the cmd_vel rules because I can't read!!!!
        //my simulation my rules!!
        forwardRate = (float) cmdVel.linear.x;
        rotationRate = (float) cmdVel.angular.z;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float vl = (forwardRate + rotationRate * wheelSeparation / 2.0f);
        float vr = (forwardRate - rotationRate * wheelSeparation / 2.0f); 
        ArticulationDrive rightXDrive = frontRightWheel.xDrive;
        rightXDrive.stiffness = 10000;
        rightXDrive.damping = 1000;        
        rightXDrive.target += vr / wheelRadius;

        ArticulationDrive leftXDrive = frontLeftWheel.xDrive;
        leftXDrive.stiffness = 10000;
        leftXDrive.damping = 1000;
        leftXDrive.target += vl / wheelRadius;

        frontRightWheel.xDrive = rightXDrive;  
        frontLeftWheel.xDrive = leftXDrive;
        if(fourWheelDrive)
        {
            rearRightWheel.xDrive = rightXDrive;
            rearLeftWheel.xDrive = leftXDrive;
        }
        
        float ssum = vl + vr;
        float sdiff = vl - vr;

        poseEncoderX +=  ssum / 2.0f * Mathf.Cos( poseEncoderTheta + sdiff / (2.0f*wheelSeparation));
        poseEncoderY +=  ssum / 2.0f * Mathf.Sin( poseEncoderTheta + sdiff / (2.0f*wheelSeparation));
        poseEncoderTheta = sdiff / (wheelSeparation);

    }
}
