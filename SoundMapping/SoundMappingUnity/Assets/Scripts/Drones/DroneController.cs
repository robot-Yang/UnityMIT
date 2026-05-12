using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class DroneController : MonoBehaviour
{

    #region Parameters

    public GameObject droneModel;

    public Vector3 separationForce = Vector3.zero;
    public Vector3 alignmentForce = Vector3.zero;
    public Vector3 cohesionForce = Vector3.zero;
    public Vector3 migrationForce = Vector3.zero;
    public Vector3 obstacleAvoidanceForce = Vector3.zero;

    public Material connectedColor;
    public Material farColor;
    public Material notConnectedColor;
    public Material selectedColor;
    public Material embodiedColor;

    private List<GameObject> bodyParts = new List<GameObject>();
    public bool showGuizmos = false;
    public bool prediction = false;
    const float distanceToHeigth = 3f;


    private GameObject gm
    {
        get
        {
            return GameObject.FindGameObjectWithTag("GameManager");
        }
    }
    private float timeSeparated = 0;
    public GameObject fireworkParticle;

    public DroneFake droneFake;

    public bool dummy = false;
    private Vector3 filteredCommandAccelerationWorld = Vector3.zero;
    private Vector3 filteredTargetVelocityWorld = Vector3.zero;
    private float filteredYawRate = 0f;
    private Vector3 holdPositionWorld = Vector3.zero;
    private bool holdPositionInitialized = false;

    float realScore
    {
        get
        {
            return 0.5f;
        }
    }

    #endregion


    void Start()
    {
        StartNormal();
        Application.targetFrameRate = 30; // Set the target frame rate to 30 FPS
    }

    public void crash()
    {
        if (CameraMovement.embodiedDrone == this.gameObject)
        {
            MigrationPointController.selectedDrone = null;
            CameraMovement.nextEmbodiedDrone = null;


            if (LevelConfiguration._startEmbodied)
            {
                CameraMovement.crashAnimationSetup();
            }
            else
            {
                CameraMovement.DesembodiedDrone(this.gameObject);
                this.droneFake.embodied = false;
                this.droneFake.selected = false;
            }

        }


        gm.GetComponent<swarmModel>().RemoveDrone(this.gameObject);
        gm.GetComponent<HapticsTest>().crash(true);

        GameObject firework = Instantiate(fireworkParticle, transform.position, Quaternion.identity);
        firework.transform.position = transform.position;


        Destroy(firework, 0.5f);
    }

    // float printTimer = 0f;
    void FixedUpdate()
    {
        if (!prediction && !dummy)
        {
            UpdateNormal();
            // printTimer += Time.fixedDeltaTime;
            // if (printTimer >= 0.5f)        // 每 0.5 s 打一次
            // {
            //     printTimer = 0f;
            //     Debug.Log(
            //         $"[t={Time.time:F1}s]  Drone {droneFake.id}  obsForce = {obstacleAvoidanceForce:F2}");
            // }
        }
    }


    #region NormalMode
    void StartNormal()
    {
        //iterate threw all the children and all the children of the children ect and check if tag BodyPart
        checkChildren(this.gameObject);

    }

    void checkChildren(GameObject start)
    {
        foreach (Transform child in start.transform)
        {
            if (child.tag == "BodyMaterial")
            {
                bodyParts.Add(child.gameObject);
            }
            checkChildren(child.gameObject);
        }
    }

    void UpdateNormal()
    {
        try
        {
            if (DroneFake.useRigidbodyCascadeControl)
            {
                SyncDroneFakeFromRigidbody();
                updateColor();
                return;
            }

            Vector3 positionDrome = droneFake.position;

            //check if valid vector3 like nop Nan
            if (float.IsNaN(positionDrome.x) || float.IsNaN(positionDrome.y) || float.IsNaN(positionDrome.z))
            {
                print("Nan++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
                print("accelleration" + droneFake.acceleration);
                print("velocity" + droneFake.velocity);
                print("Allignment force" + droneFake.lastAllignement);
                print("Cohesion force" + droneFake.lastOlfati);
                print("Obstalce force " + droneFake.lastObstacle);

                print("Nan+++++++++++++++++++++++++++++" + this.droneFake.id + "+++++++++++++++++++++++++++++");
                return;
            }


            transform.position = positionDrome;
            updateColor();
            // updateSound();
            // droneAnimate();
        }
        catch (Exception e)
        {
            print("Error in drone update");
            print(e);
        }
    }

    public void SyncDroneFakeFromRigidbody()
    {
        if (droneFake == null) return;

        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            droneFake.position = transform.position;
            return;
        }

        droneFake.position = rb.position;
        droneFake.velocity = rb.velocity;
    }

    public void ApplyRigidbodyCascadeControl(swarmModel cfg)
    {
        if (cfg == null || droneFake == null || !droneFake.isMovable) return;

        Rigidbody rb = EnsureCascadeRigidbody(cfg);
        float g = Mathf.Max(Physics.gravity.magnitude, 1e-3f);

        Vector3 commandAccelerationWorld = droneFake.acceleration * Mathf.Max(cfg.cascadeCommandGain, 0f);
        if (cfg.cascadeIgnoreVerticalCommand)
        {
            commandAccelerationWorld.y = 0f;
        }

        float maxCmdAccel = Mathf.Max(cfg.cascadeMaxCommandAcceleration, 0f);
        if (maxCmdAccel > 0f)
        {
            commandAccelerationWorld = Vector3.ClampMagnitude(commandAccelerationWorld, maxCmdAccel);
        }

        filteredCommandAccelerationWorld = Vector3.Lerp(
            filteredCommandAccelerationWorld,
            commandAccelerationWorld,
            Mathf.Clamp01(cfg.cascadeCommandFilterCoefficient));

        bool hasPilotCommand = MigrationPointController.alignementVector.magnitude > Mathf.Max(cfg.cascadePilotCommandDeadband, 0f);
        if (!hasPilotCommand)
        {
            float decayAlpha = 1f - Mathf.Exp(-Mathf.Max(cfg.cascadeNoInputCommandDecay, 0f) * Time.fixedDeltaTime);
            filteredCommandAccelerationWorld = Vector3.Lerp(filteredCommandAccelerationWorld, Vector3.zero, decayAlpha);
        }

        Vector3 targetVelocityWorld;
        bool usePositionHold = !hasPilotCommand && cfg.cascadeEnablePositionHold;
        if (usePositionHold)
        {
            if (!holdPositionInitialized)
            {
                holdPositionWorld = rb.position;
                holdPositionInitialized = true;
            }

            Vector3 positionError = holdPositionWorld - rb.position;
            Vector3 holdVelocity = positionError * Mathf.Max(cfg.cascadePositionHoldKp, 0f)
                                  - rb.velocity * Mathf.Max(cfg.cascadePositionHoldKd, 0f);
            holdVelocity = Vector3.ClampMagnitude(holdVelocity, Mathf.Max(cfg.cascadePositionHoldMaxSpeed, 0f));
            targetVelocityWorld = holdVelocity;
        }
        else
        {
            holdPositionWorld = rb.position;
            holdPositionInitialized = true;
            targetVelocityWorld = rb.velocity + filteredCommandAccelerationWorld * Mathf.Max(cfg.cascadeVelocityPreview, 0f);
        }

        targetVelocityWorld = Vector3.ClampMagnitude(targetVelocityWorld, DroneFake.maxSpeed);
        float velocityFilter = usePositionHold
            ? Mathf.Clamp01(cfg.cascadePositionHoldFilterCoefficient)
            : Mathf.Clamp01(cfg.cascadeVelocityFilterCoefficient);
        filteredTargetVelocityWorld = Vector3.Lerp(
            filteredTargetVelocityWorld,
            targetVelocityWorld,
            velocityFilter);

        Vector3 velocityBody = transform.InverseTransformDirection(rb.velocity);
        Vector3 targetVelocityBody = transform.InverseTransformDirection(filteredTargetVelocityWorld);
        Vector3 velocityError = velocityBody - targetVelocityBody;

        float tauVel = Mathf.Max(cfg.cascadeVelocityTimeConstant, 1e-3f);
        Vector3 desiredAccelerationBody = -velocityError / tauVel;

        float maxPitch = cfg.cascadeMaxPitchDeg * Mathf.Deg2Rad;
        float maxRoll = cfg.cascadeMaxRollDeg * Mathf.Deg2Rad;
        Vector3 desiredTheta = new Vector3(
            Mathf.Clamp(desiredAccelerationBody.z / g, -maxPitch, maxPitch),
            0f,
            Mathf.Clamp(-desiredAccelerationBody.x / g, -maxRoll, maxRoll));

        Vector3 worldDown = transform.InverseTransformDirection(Vector3.down);
        float pitch = worldDown.z;
        float roll = -worldDown.x;

        float tauAtt = Mathf.Max(cfg.cascadeAttitudeTimeConstant, 1e-3f);
        Vector3 desiredOmegaBody = new Vector3(
            -(pitch - desiredTheta.x) / tauAtt,
            0f,
            -(roll - desiredTheta.z) / tauAtt);

        float targetYawRate = 0f;
        Vector3 planarTargetVelocity = new Vector3(filteredTargetVelocityWorld.x, 0f, filteredTargetVelocityWorld.z);
        if (droneFake.embodied && LevelConfiguration._control_rotation)
        {
            targetYawRate = Input.GetAxis("JoystickRightHorizontal") * Mathf.Max(cfg.cascadeEmbodiedYawInputGain, 0f);
        }
        else if (cfg.cascadeAlignYawToEmbodied && CameraMovement.embodiedDrone != null)
        {
            Vector3 embodiedForward = Vector3.ProjectOnPlane(CameraMovement.embodiedDrone.transform.forward, Vector3.up);
            if (embodiedForward.sqrMagnitude > 1e-6f)
            {
                float targetYaw = Mathf.Atan2(embodiedForward.x, embodiedForward.z) * Mathf.Rad2Deg;
                float currentYaw = transform.eulerAngles.y;
                float yawError = Mathf.Deg2Rad * Mathf.DeltaAngle(currentYaw, targetYaw);
                targetYawRate = yawError / tauAtt;
            }
        }
        else if (planarTargetVelocity.sqrMagnitude > 1e-5f)
        {
            float targetYaw = Mathf.Atan2(planarTargetVelocity.x, planarTargetVelocity.z) * Mathf.Rad2Deg;
            float currentYaw = transform.eulerAngles.y;
            float yawError = Mathf.Deg2Rad * Mathf.DeltaAngle(currentYaw, targetYaw);
            targetYawRate = yawError / tauAtt;
        }
        targetYawRate = Mathf.Clamp(
            targetYawRate,
            -Mathf.Max(cfg.cascadeMaxYawRate, 0f),
            Mathf.Max(cfg.cascadeMaxYawRate, 0f));
        filteredYawRate = Mathf.Lerp(filteredYawRate, targetYawRate, Mathf.Clamp01(cfg.cascadeYawFilterCoefficient));
        desiredOmegaBody.y = filteredYawRate;

        Vector3 angularVelocityBody = transform.InverseTransformDirection(rb.angularVelocity);
        Vector3 omegaError = angularVelocityBody - desiredOmegaBody;

        Vector3 desiredAlpha = new Vector3(
            -omegaError.x / Mathf.Max(cfg.cascadeAngularTimeConstantXY, 1e-3f),
            -omegaError.y / Mathf.Max(cfg.cascadeAngularTimeConstantZ, 1e-3f),
            -omegaError.z / Mathf.Max(cfg.cascadeAngularTimeConstantXY, 1e-3f));
        desiredAlpha -= angularVelocityBody * Mathf.Max(cfg.cascadeAngularRateDamping, 0f);
        desiredAlpha = Vector3.ClampMagnitude(desiredAlpha, Mathf.Max(cfg.cascadeMaxAngularAccel, 0f));

        Vector3 desiredTorque = Vector3.Scale(desiredAlpha, rb.inertiaTensor);
        float thrust = desiredAccelerationBody.y + (cfg.cascadeUseGravityCompensation ? g : 0f);
        float denom = Mathf.Cos(roll) * Mathf.Cos(pitch);
        if (Mathf.Abs(denom) < 1e-3f) denom = 1e-3f * Mathf.Sign(denom == 0f ? 1f : denom);
        thrust /= denom;
        thrust = Mathf.Clamp(thrust, 0f, Mathf.Max(cfg.cascadeMaxThrustMultiplier, 0f) * g);
        Vector3 desiredForce = Vector3.up * thrust * rb.mass;

        // desiredTorque/desiredForce are physical torque/force terms; apply them in Force mode.
        rb.AddRelativeTorque(desiredTorque, ForceMode.Force);
        rb.AddRelativeForce(desiredForce, ForceMode.Force);

        // Safety clamp to stay consistent with legacy speed limits.
        rb.velocity = Vector3.ClampMagnitude(rb.velocity, DroneFake.maxSpeed);

        droneFake.acceleration = transform.TransformDirection(desiredAccelerationBody);
    }

    Rigidbody EnsureCascadeRigidbody(swarmModel cfg)
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();

        rb.mass = Mathf.Max(cfg.cascadeRigidbodyMass, 0.01f);
        rb.drag = Mathf.Max(cfg.cascadeRigidbodyDrag, 0f);
        rb.angularDrag = Mathf.Max(cfg.cascadeRigidbodyAngularDrag, 0f);
        rb.useGravity = cfg.cascadeRigidbodyUseGravity;
        rb.isKinematic = false;

        return rb;
    }

    #endregion


    #region HapticAudio

    void setMaterial(Material mat)
    {
        foreach (GameObject bodyPart in bodyParts)
        {
            bodyPart.GetComponent<Renderer>().material = mat;
        }
    }
    void updateColor()
    {
        if (CameraMovement.embodiedDrone == this.gameObject)
        {
            setMaterial(embodiedColor);
        }
        else
        {
            if (MigrationPointController.selectedDrone == this.gameObject)
            {
                setMaterial(connectedColor);
                this.droneFake.selected = true;
                return;
            }
            else
            {
                this.droneFake.selected = false;

                if (droneFake.score >= 0.9f)
                {
                    setMaterial(connectedColor);
                }
                else
                {
                    // setMaterial(notConnectedColor);
                }
            }
        }
    }

    void updateSound()
    {
        if (CameraMovement.embodiedDrone == this)
        {
            this.GetComponent<AudioSource>().enabled = false;
            return;
        }



        if (swarmModel.dronesInMainNetwork.Contains(this.droneFake))
        {
            timeSeparated += Time.deltaTime;
            this.GetComponent<AudioSource>().enabled = false;
        }
        else
        {
            timeSeparated = 0;
            this.GetComponent<AudioSource>().enabled = true;
        }

    }


    void droneAnimate()
    {
        //look at the same direction as velocity
        if (CameraMovement.embodiedDrone == this.gameObject)
        {
            return;
        }
        if (droneFake.velocity.magnitude > 0.5)
        {
            Vector3 forwardDrone = new Vector3(droneFake.velocity.x, 0, droneFake.velocity.z);
            //lerp the rotation
            transform.forward = Vector3.Lerp(transform.forward, forwardDrone, Time.deltaTime * 5);
        }
        else
        {
            //only keep rotation on y axis
            transform.forward = new Vector3(transform.forward.x, 0, transform.forward.z);
        }

    }
    #endregion
    
    #if UNITY_EDITOR
    void OnDrawGizmos()            // 选中该无人机时才显示
    {
        // ① 取出列表（一个无人机可能有 0-N 条障碍力）
        // List<Vector3> obsForces = droneFake.lastObstacleForces;
        List<Vector3> obsForces = droneFake.lastObstacleForcesFeedback;

        if (obsForces == null || obsForces.Count == 0)
            return;                        // 本帧没有障碍力

        Vector3 origin = transform.position;
        const float SCALE = 0.1f;          // 线段长度放大系数，可调

        // ② 遍历列表，逐条画线
        // Gizmos.color = Color.red;
        // for (int i = 0; i < obsForces.Count; i++)
        // {
        //     Vector3 f   = obsForces[i];

        //     // --- 跳过指向地面的力 ---------------------------------
        //     if (f.y != -0.0f) continue;    // 阈值可调

        //     Vector3 tip = origin + f * SCALE;
        //     Gizmos.DrawLine(origin, tip);

        //     // 简易箭头
        //     Vector3 dir = f.normalized;
        //     float len   = f.magnitude * SCALE * 0.2f;
        //     Vector3 l = Quaternion.AngleAxis(150, Vector3.up) * dir * len;
        //     Vector3 r = Quaternion.AngleAxis(-150,Vector3.up) * dir * len;
        //     Gizmos.DrawLine(tip, tip + l);
        //     Gizmos.DrawLine(tip, tip + r);

        // }
    }
    #endif

}


public class ObstacleInRange
{
    public Vector3 position;
    public float distance;

    public ObstacleInRange(Vector3 position, float distance)
    {
        this.position = position;
        this.distance = distance;
    }
}
