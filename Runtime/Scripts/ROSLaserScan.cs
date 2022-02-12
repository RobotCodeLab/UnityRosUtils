using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class ROSLaserScan : MonoBehaviour
{
    public string frame_id = "base_link";
    public string topicName = "imu/data";

    public float angleMin = -45.0f;
    public float angleMax = 45.0f;
    public float rangeMin = 0.0f;
    public float rangeMax = 1000.0f;
    public float measurementsPerScan = 20;
    public float timeBetweenScans = 0.1f;
    public float angleIncrement = 0.0f;
    public float scanTime;

    private bool isScanning = false;
    private List<float> ranges;
    private float measurementsTaken = 0;
    private float nextScanTime = -1;
    private float currentAngleStart;
    private float currentAngleEnd;

    private ROSClock clock;
    private ROSConnection ros;

    // Start is called before the first frame update
    void Start()
    {
        clock = GameObject.FindObjectOfType<ROSClock>();
        ros = ROSConnection.GetOrCreateInstance();

        if (clock == null)
            Debug.LogError("IMU Error: Cannot find clock");

        currentAngleStart = angleMin;
        currentAngleEnd = angleMax;
        ranges = new List<float>();

        ros.RegisterPublisher<LaserScanMsg>(topicName);
    }

    // Update is called once per frame
    void Update()
    {
        float yawBaseDegrees = this.transform.rotation.eulerAngles.y;

        if (!isScanning)
        {
            if (clock.getTimeStamp().sec < nextScanTime)
                return;

            //otherwise start a scan
            nextScanTime = nextScanTime + timeBetweenScans;
            measurementsTaken = 0;
        }

        //get the ranges for this scan
        while (measurementsTaken < measurementsPerScan)
        {
            float t = measurementsTaken / (float)measurementsPerScan;
            float yawSensorDegrees = Mathf.Lerp(angleMin, angleMax, t);
            float yawDegrees = yawBaseDegrees + yawSensorDegrees;
            Vector3 directionVector = Quaternion.Euler(0f, yawDegrees, 0f) * Vector3.forward;
            Vector3 measurementStart = rangeMin * directionVector + transform.position;
            Ray measurementRay = new Ray(measurementStart, directionVector);
            bool isValidRay = Physics.Raycast(measurementStart, directionVector, out var hit, rangeMax);

            if (isValidRay)
            {
                ranges.Add(hit.distance);
            }
            else
            {
                ranges.Add(rangeMax);
            }

            measurementsTaken++;
        }

        //end scan and publish results to ROS
        float angleStartRos = -currentAngleStart * Mathf.Deg2Rad;
        float angleEndRos = -currentAngleEnd * Mathf.Deg2Rad;

        if (angleStartRos > angleEndRos)
        {
            Debug.LogWarning("LaserScan was performed in a clockwise direction but ROS expects a counter-clockwise scan, flipping the ranges...");
            var temp = angleEndRos;
            angleEndRos = angleStartRos;
            angleStartRos = temp;
            ranges.Reverse();
        }


        HeaderMsg header = new HeaderMsg(0, clock.getTimeStamp(), frame_id);

        LaserScanMsg msg = new LaserScanMsg(header, angleStartRos, angleEndRos, (angleEndRos - angleStartRos) / measurementsPerScan, 0.0f, timeBetweenScans, rangeMin, rangeMax, ranges.ToArray(), new float[ranges.Count]);
        ros.Publish(topicName, msg);

        measurementsTaken = 0;
        ranges.Clear();
        isScanning = false;
        float now = clock.getTimeStamp().sec; 
        if (now > nextScanTime)
        {
            nextScanTime = now;
        }

        currentAngleStart += angleIncrement;
        currentAngleEnd += angleIncrement;
        if (currentAngleStart > 360f || currentAngleEnd> 360f)
        {
            currentAngleStart -= 360f;
            currentAngleEnd -= 360f;
        }
    }
}
