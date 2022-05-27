using UnityEngine;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    [SerializeField] private Transform SensorFR;
    [SerializeField] private Transform SensorL1;
    [SerializeField] private Transform SensorL2;
    [SerializeField] private Transform SensorL3;
    [SerializeField] private Transform SensorR1;
    [SerializeField] private Transform SensorR2;
    [SerializeField] private Transform SensorR3;
    [SerializeField] private Transform SensorOR;

    [SerializeField] private float maxSteeringAngle;
    [SerializeField] private float motorForce = 75f;
    [SerializeField] private float brakeForce;

    private Rigidbody rb;

    [SerializeField] private float angle_x;
    [SerializeField] private float angle_z;
    [SerializeField] private float velocity;

    [SerializeField] private float steerDirection;
    [SerializeField] private int GearPosition;

    private float steerAngle;
    private bool isBreaking;

    private float s1dist = 12;
    private float s2dist = 12;
    private float s3dist = 5;

    float s1x = 5; float s1y = 10; float s1z = 0;
    float s2x = 6; float s2y = 15; float s2z = 0;
    float s3x = 20; float s3y = 50; float s3z = 0;

    int var = 0;
    private void Start()
    {
        rb = GetComponent<Rigidbody>();


        AdjustSensors(SensorFR, 20, 0, 0);
        AdjustSensors(SensorL1, s1x, -s1y, s1z);
        AdjustSensors(SensorL2, s2x, -s2y, s2z);
        AdjustSensors(SensorL3, s3x, -s3y, s3z);
        AdjustSensors(SensorR1, s1x, s1y, s1z);
        AdjustSensors(SensorR2, s2x, s2y, s2z);
        AdjustSensors(SensorR3, s3x, s3y, s3z);
        AdjustSensors(SensorOR, 50, 180, 0);
    }

    private void FixedUpdate()
    {
        ECU();
        AdjustSpeed();
        HandleMotor();
        UpdateWheels();

        angle_x = SensorOR.eulerAngles.x;
        angle_z = SensorOR.eulerAngles.z;

        velocity = rb.velocity.magnitude;

    }

    private void AdjustSensors(Transform sensor, float x_angle, float y_angle, float z_angle)
    {
        sensor.transform.Rotate(x_angle, y_angle, z_angle);
    }

    private void HandleMotor()
    {
        int gear = gearShift();
        GearPosition = gear;
        switch (gear)
        {
            case 0:
                motorForce = 0;
                brakeForce = 100;
                break;
            case 1:
                motorForce = 760;
                break;
            case 2:
                motorForce = 100;
                break;
            case 3:
                motorForce = 75;
                break;
            case 4:
                motorForce = 50;
                break;
            case 5:
                motorForce = 0;
                brakeForce = 500;
                break;

        }

        frontLeftWheelCollider.motorTorque = motorForce;
        frontRightWheelCollider.motorTorque = motorForce;
        rearLeftWheelCollider.motorTorque = motorForce;
        rearRightWheelCollider.motorTorque = motorForce;

        frontLeftWheelCollider.brakeTorque = brakeForce;
        frontRightWheelCollider.brakeTorque = brakeForce;
        rearLeftWheelCollider.brakeTorque = brakeForce;
        rearRightWheelCollider.brakeTorque = brakeForce;
    }

    private void UpdateWheels()
    {
        UpdateWheelsPos(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelsPos(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelsPos(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelsPos(rearRightWheelCollider, rearRightWheelTransform);
    }

    private void UpdateWheelsPos(WheelCollider wheelCollider, Transform trans)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        trans.rotation = rot;
        trans.position = pos;
    }

    private void HandleSteering(float direction)
    {
        steerAngle = maxSteeringAngle * direction;
        frontLeftWheelCollider.steerAngle = steerAngle;
        frontRightWheelCollider.steerAngle = steerAngle;

    }

    private bool sense(Transform sensor, float distance)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensor.position, sensor.TransformDirection(Vector3.forward), out hit, distance))
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * distance, Color.clear);
            return true;
        }
        else
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * distance, Color.clear);
            return false;
        }

    }



    private void ECU()
    {
        float dir = StayOnRoad();
        HandleSteering(dir);


        steerDirection = dir;
        if (getHitDistance(SensorL1, s1dist) < 3.5)
        {
            HandleSteering(1f);
        }
        else if (getHitDistance(SensorR1, s1dist) < 3.5)
        {
            HandleSteering(-1f);
        }
        else if (getHitDistance(SensorL1, s1dist) < 8)
        {
            HandleSteering(0.1f);
        }


    }
    private float StayOnRoad()
    {
        bool R1 = sense(SensorR1, s1dist);
        bool R2 = sense(SensorR2, s2dist);
        bool R3 = sense(SensorR3, s3dist);
        bool L1 = sense(SensorL1, s1dist);
        bool L2 = sense(SensorL2, s2dist);
        bool L3 = sense(SensorL3, s3dist);
        bool FR = sense(SensorFR, 3);
        int right_ActiveSensors = 0; int left_ActiveSensors = 0;
        brakeForce = 0;

        if (!R3)
        {

            brakeForce = 700;
            return -1f;

        }
        if (!L3)
        {
            brakeForce = 700;
            return 1f;
        }

        
        if (!R1 & !R2 & !L1 & !L2)
        {
            return 0;
        }
        if (!R2 & right_ActiveSensors >= left_ActiveSensors)
        {
            brakeForce = 70;
            return -0.75f;
        }
        if (!L2 & left_ActiveSensors >= right_ActiveSensors)
        {
            brakeForce = 70;
            return 0.75f;
        }


        if (!R1 & right_ActiveSensors >= left_ActiveSensors)
        {
            brakeForce = 20;
            return -0.5f;
        }
        if (!L1 & left_ActiveSensors >= right_ActiveSensors)
        {
            brakeForce = 20;
            return 0.5f;
        }

        return 0;


    }

    private float getHitDistance(Transform sensor, float distance)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensor.position, sensor.TransformDirection(Vector3.forward), out hit, distance))
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * distance, Color.clear);
            return hit.distance;
        }
        else
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * distance, Color.clear);
            return distance;
        }
    }

    private void AdjustSpeed()
    {
        if (velocity < 2)
        {
            brakeForce = 0;
        }
        if (velocity > 3.5)
        {
            brakeForce = 50;
        }
    }

    private void throttle()
    {

        if (motorForce != 0 & velocity < 1)
        {
            motorForce *= 100;


        }
    }

    private int gearShift()
    {
        if (velocity < 1.7)
        {
            return 1;
        }
        if (velocity > 1.7 & velocity < 2.5)
        {
            return 2;
        }
        if (velocity > 2.5 & velocity < 3.5)
        {
            return 3;
        }
        if (velocity > 3.5 & velocity < 4.5)
        {
            return 4;
        }
        if (velocity > 4.5)
        {
            return 5;
        }
        return 0;
    }


}