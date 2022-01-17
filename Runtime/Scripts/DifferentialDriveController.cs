using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class DifferentialDriveController : MonoBehaviour
{
    public float wheelSeparation = 0.37559f;
    public float wheelRadius = 0.098f;
    public bool fourWheelDrive = false;
    public bool useTruePositionForOdom = false;
    public GameObject chasis;

    public GameObject frontRightWheelObject;
    public GameObject frontLeftWheelObject;
    public GameObject rearRightWheelObject;
    public GameObject rearLeftWheelObject;

    private ROSConnection ros;
    private ROSClock clock;

    private ArticulationBody frontRightWheel;
    private ArticulationBody rearRightWheel;
    private ArticulationBody frontLeftWheel;
    private ArticulationBody rearLeftWheel;

    private float forwardRate = 0.0f;
    private float rotationRate = 0.0f;

    private Vector3 lastPosition;
    private Vector3 startingPosition;
    private float lastRot = 0.0f;

    private float poseEncoderX = 0.0f;
    private float poseEncoderY = 0.0f;
    private float poseEncoderTheta = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        //listen for cmd_vel
        ros = ROSConnection.GetOrCreateInstance();
        ros.listenForTFMessages = false;
        ros.Subscribe<TwistMsg>("/jackal_velocity_controller/cmd_vel", cmdVelCallback);

        //get and store joints for each wheel
        if (frontRightWheelObject == null || frontLeftWheelObject == null)
            Debug.LogError("Differential Drive Error: Please specify wheel gameobjects");

        clock = GameObject.FindObjectOfType<ROSClock>();

        frontRightWheel = frontRightWheelObject.GetComponent<ArticulationBody>();
        frontLeftWheel = frontLeftWheelObject.GetComponent<ArticulationBody>();

        if (fourWheelDrive)
        {
            if (rearRightWheelObject == null || rearLeftWheelObject == null)
                Debug.LogError("Differential Drive Error: Four Wheel Drive activated but four wheels not specified");

            rearRightWheel = rearRightWheelObject.GetComponent<ArticulationBody>();
            rearLeftWheel = rearLeftWheelObject.GetComponent<ArticulationBody>();
        }

        lastPosition = chasis.transform.position;
        startingPosition = lastPosition;
        lastRot = chasis.transform.rotation.eulerAngles.y;

        ros.RegisterPublisher<OdometryMsg>("/jackal_velocity_controller/odom");
    }

    void cmdVelCallback(TwistMsg cmdVel)
    {
        forwardRate = (float)cmdVel.linear.x;
        rotationRate = (float)cmdVel.angular.z;
    }

    // Update is called once per frame
    void Update()
    {
        float phiL = ((wheelSeparation * rotationRate) + forwardRate) / (2 * wheelRadius);
        float phiR = ((-wheelSeparation * rotationRate) + forwardRate) / (2 * wheelRadius);
        ArticulationDrive rightXDrive = frontRightWheel.xDrive;
        rightXDrive.damping = 10;
        rightXDrive.targetVelocity = phiR * Mathf.Rad2Deg;

        ArticulationDrive leftXDrive = frontLeftWheel.xDrive;
        leftXDrive.damping = 10;
        leftXDrive.targetVelocity = phiL * Mathf.Rad2Deg;

        frontRightWheel.xDrive = rightXDrive;
        frontLeftWheel.xDrive = leftXDrive;
        if (fourWheelDrive)
        {
            rearRightWheel.xDrive = rightXDrive;
            rearLeftWheel.xDrive = leftXDrive;
        }

        float dx = 0.0f;
        float dy = 0.0f;
        float dTheta = 0.0f;

        if (useTruePositionForOdom)
        {
            Vector3 offset = chasis.transform.position - startingPosition;
            Vector3 delta = offset - lastPosition;
            lastPosition = offset;

            poseEncoderX = offset.z;
            poseEncoderY = offset.x;
            poseEncoderTheta = chasis.transform.rotation.eulerAngles.y * Mathf.Deg2Rad;

            dx = delta.z;
            dy = delta.x;
            dTheta = chasis.transform.rotation.eulerAngles.y - lastRot;
            dTheta *= Mathf.Deg2Rad;
            lastRot = chasis.transform.rotation.eulerAngles.y;
        }
        else
        {
            float sl = frontLeftWheel.jointVelocity[0];
            float sr = frontRightWheel.jointVelocity[0];

            //round the values to get rid of some error from over precise calculations
            sl = (float)System.Math.Round((decimal)sl, 3);
            sr = (float)System.Math.Round((decimal)sr, 3);

            float dXR = (wheelRadius * (sl + sr)) / 2.0f;
            dTheta = (wheelRadius * (sl - sr)) / wheelSeparation;
            float weirdTheta = (wheelRadius * (sl - sr)) / (2.0f * wheelSeparation);

            poseEncoderTheta += weirdTheta * Time.deltaTime;
            dx = Mathf.Cos(poseEncoderTheta) * dXR;
            dy = Mathf.Sin(poseEncoderTheta) * dXR;

            poseEncoderX += dx * Time.deltaTime;
            poseEncoderY += dy * Time.deltaTime;
        }

        Quaternion qt = Quaternion.Euler(0, 0, Mathf.Rad2Deg * poseEncoderTheta);

        OdometryMsg odom = new OdometryMsg();
        odom.header.frame_id = "odom";
        odom.header.stamp = clock.getTimeStamp();
        PoseWithCovarianceMsg pose = new PoseWithCovarianceMsg();
        TwistWithCovarianceMsg twist = new TwistWithCovarianceMsg();

        pose.pose.position.x = poseEncoderX;
        pose.pose.position.y = poseEncoderY;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = qt.z;
        pose.pose.orientation.y = -qt.x;
        pose.pose.orientation.z = qt.y;
        pose.pose.orientation.w = -qt.w;

        twist.twist.angular.z = dTheta / Time.deltaTime;
        twist.twist.linear.x = dx / Time.deltaTime;
        twist.twist.linear.y = dy / Time.deltaTime;

        // set covariance
        pose.covariance[0] = 0.00001;
        pose.covariance[7] = 0.00001;
        pose.covariance[14] = 1000000000000.0;
        pose.covariance[21] = 1000000000000.0;
        pose.covariance[28] = 1000000000000.0;
        pose.covariance[35] = 0.001;

        odom.pose = pose;
        odom.twist = twist;
        ros.Publish("/jackal_velocity_controller/odom", odom);
    }
}
