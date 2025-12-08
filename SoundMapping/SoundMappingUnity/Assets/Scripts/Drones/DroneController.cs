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


