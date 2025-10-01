using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
//using FischlWorks_FogWar;
using Unity.VisualScripting;
using UnityEngine;

public class swarmModel : MonoBehaviour
{

    #region Parameters
    public static int idLeader
    {
        get
        {
            if(LevelConfiguration._startEmbodied)
            {
                return CameraMovement.idLeader;
            }
            return MigrationPointController.idLeader;
        }
    }
    public bool saveData
    {
        get
        {
            return LevelConfiguration._SaveData;
        }
    }
    public bool needToSpawn
    {
        get
        {
            return LevelConfiguration._NeedToSpawn;
        }
    }
    public static GameObject swarmHolder;
    public GameObject dronePrefab;
    public int numDrones
    {
        get
        {
            return LevelConfiguration._NumDrones;
        }
    }
    public float spawnRadius
    {
        get
        {
            return LevelConfiguration._SpawnRadius;
        }
    }
    public static float spawnHeight = 5f;

    public float lastObstacleAvoidance = -1f;

    public float maxSpeed = 5f;
    public float maxForce = 10f;

    public static float extraDistanceNeighboor = 2f;
    public static float neighborRadius
    {
        get
        {
            //return 3*desiredSeparation;
            return Mathf.Max(2f * desiredSeparation, desiredSeparation + extraDistanceNeighboor);
        }
    }

    public static float desiredSeparation = 3f;
    public float alpha = 1.5f; // Separation weight
    public float beta = 1.0f;  // Alignment weight
    public float delta = 1.0f; // Migration weight
    public float cVm = 1f;    // Maximum velocity change

    public float avoidanceRadius = 3f;     // Radius for obstacle detection
    public float avoidanceRadiusFeedback = 3f;     // Radius for obstacle detection
    public float desiredSeparationObs = 3f;
    public float avoidanceForce = 10f;     // Strength of the avoidance force

    public static float droneRadius = 0.17f;      // Radius of the drone
    public LayerMask obstacleLayer;        // Layer mask for obstacles

    //public csFogWar fogWar;

    public int PRIORITYWHENEMBODIED = 15;
    public float dampingFactor = 0.98f;

    public static NetworkCreator network;

    public static float minDistance = float.MaxValue;

    public static List<DroneFake> dronesInMainNetwork
    {
        get
        {
            List<DroneFake> drones = new List<DroneFake>();
            if (network == null)
            {
                return drones;
            }

            foreach (DroneFake drone in network.largestComponent)
            {
                drones.Add(drone);
            }

            return drones;
        }
    }


    public static Vector3 swarmVelocityAvg = Vector3.zero;

    public static float swarmConnectionScore = 0f;
    public static int numberOfDroneDiscionnected
    {
        get
        {
            return SwarmDisconnection.dronesID.Count;
        }
    }
    public static int numberOfDroneCrashed
    {
        get
        {
            return Timer.numberDroneDied;
        }
    }
    public static float swarmAskingSpreadness = 0f;

    public static List<DroneFake> drones = new List<DroneFake>();

    [Header("Gizmos")]
    public bool showObstacleGizmos = false;
    public bool showNetworkGizmos = true; //false;
    public bool showNetworkConnexionVulnerability = false;
    public bool showNetworkConnectivity = true; //false;
    public bool showDroneObstacleForces = true; //false;
    public bool showDroneOlfatiForces = true; //false;
    public bool showDroneAllignementForces = false;



    public bool showSwarmObstacleForces =  true; //false;
    public bool showSwarmOlfati = false;


    #endregion


    public static List<Vector3> swarmObstacleForces = new List<Vector3>();
    public static List<Vector3> swarmOlfatiForces = new List<Vector3>();

    private Thread scorePlottingThread;
    private bool isThreadRunning = false;
    private float printInterval = 0.2f; // Print every second
    private object scoreLock = new object();

    private class NetworkScores
    {
        public float connectionScore;
        public float spreadnessScore;
        public int disconnectedCount;
        public int crashedCount;
        public float minDistance;

        public float relativeConnectivity;
        public float cohesionRadius;

        public float normalizeConnectionScore;

        public float velocityMissmatch;

        public List<DroneFake> dronesSnapshot = new List<DroneFake>();
        public Vector3 alignmentVector;
    }

    private NetworkScores currentScores = new NetworkScores();
    private object networkLock = new object();

    void Start()
    {

        TriggerHandlerWithCallback.setGM(this.gameObject);

        Application.targetFrameRate = 30; // Set the target frame rate to 30 FPS

        PRIORITYWHENEMBODIED = (int)(numDrones / 3.5f);
        swarmHolder = needToSpawn ? GameObject.FindGameObjectWithTag("Swarm") : LevelConfiguration.swarmHolder;

        if (swarmHolder == null)
        {
            print("No swarm holder found");
            swarmHolder = new GameObject("Swarm");
        }
        spawn();

        desiredSeparation = LevelConfiguration._StartSperation;

        // Initialize and start the score plotting thread
        isThreadRunning = true;
       // scorePlottingThread = new Thread(PlotNetworkScores);
       // scorePlottingThread.Start();
    }

    public static bool dummyForcesApplied = true;


    void refreshParameters()
    {
        DroneFake.maxForce = maxForce;
        DroneFake.maxSpeed = maxSpeed;
        DroneFake.cVm = cVm;
        DroneFake.desiredSeparation = desiredSeparation;
        DroneFake.alpha = alpha;
        DroneFake.beta = beta;
        DroneFake.delta = delta;
        DroneFake.avoidanceRadius = avoidanceRadius;
        DroneFake.avoidanceRadiusFeedback = avoidanceRadiusFeedback;
        DroneFake.avoidanceForce = avoidanceForce;
        DroneFake.droneRadius = droneRadius;
        DroneFake.neighborRadius = neighborRadius;
        DroneFake.PRIORITYWHENEMBODIED = PRIORITYWHENEMBODIED;
        DroneFake.dampingFactor = dampingFactor;
        DroneFake.spawnHeight = spawnHeight;
        DroneFake.desiredSeparationObs = desiredSeparationObs;
    }

    void FixedUpdate()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            restartFunction();
        }

        refreshSwarm();

        UpdateSwarmForces();

        getSwarmConnexion(); // swarm connexion score
        //swarmSeparation(3); // swarm connexion score



        if (!showNetworkConnexionVulnerability)
        {
            SwarmDisconnection.CheckDisconnection();
        }

        swarmAskingShrink();

        UpdateCurrentScores();

        // Instead of calculating scores here, just update the drones snapshot
        lock (networkLock)
        {
            currentScores.dronesSnapshot = new List<DroneFake>(drones);
            currentScores.alignmentVector = MigrationPointController.alignementVectorNonZero;
        }
    }

    void getSwarmConnexion()
    {
        //if there is one drones Movable
        List<DroneFake> connectedDrone = network.drones.ToList();
        bool hasNonMovable = drones.Exists(d => !d.isMovable);
        if (hasNonMovable)
        {
            connectedDrone = network.largestComponent.ToList();
        }

        NetworkCreator networkToCompute = new NetworkCreator(connectedDrone);
        networkToCompute.refreshNetwork();

        // float velMissmatch = networkToCompute.ComputeNormalizedVelocityMismatch();
        float energyDev = networkToCompute.ComputeNormalizedDeviationEnergy();
        ////   float relativeConnectivity = networkToCompute.ComputeRelativeConnectivity();
        //  float cohesionRadius = networkToCompute.ComputeCohesionRadius();

        swarmConnectionScore = energyDev;

        return;
       /* if(CameraMovement.embodiedDrone == null)
        {
            //if there is one drones Movable
            List<DroneFake> connectedDrone = network.drones.ToList();
            bool hasNonMovable = drones.Exists(d => !d.isMovable);
            if (hasNonMovable)
            {
                connectedDrone = network.largestComponent.ToList();
            }

            NetworkCreator networkToCompute = new NetworkCreator(connectedDrone);
            networkToCompute.refreshNetwork();

           // float velMissmatch = networkToCompute.ComputeNormalizedVelocityMismatch();
            float energyDev = networkToCompute.ComputeNormalizedDeviationEnergy();
         ////   float relativeConnectivity = networkToCompute.ComputeRelativeConnectivity();
          //  float cohesionRadius = networkToCompute.ComputeCohesionRadius();

          swarmConnectionScore = energyDev;
        }else{

            swarmConnectionScore = this.GetComponent<NetworkRepresentation>().UpdateNetworkRepresentation(network.getLayersConfiguration());

            //swarmConnectionScore = 1;
        }

        //  swarmConnectionScore = this.GetComponent<NetworkRepresentation>().UpdateNetworkRepresentation(network.getLayersConfiguration());
    
    */
    }

    public static void restart()
    {
        GameObject.FindGameObjectWithTag("GameManager").GetComponent<swarmModel>().restartFunction();
    }

    void restartFunction()
    {
        print("----------------------------- Restarting -----------------------------");
        SceneSelectorScript.reset();
        // Start();
        // fogWar.ResetMapAndFogRevealers();
        // this.GetComponent<Timer>().Restart();
    }

    void refreshSwarm()
    {
        refreshParameters();

        network = new NetworkCreator(drones);
        network.refreshNetwork();
        Dictionary<int, int> layers = network.getLayersConfiguration();

        this.GetComponent<NetworkRepresentation>().UpdateNetworkRepresentation(layers);

        ClosestPointCalculator.selectObstacle(drones); // update list of obstacle 

        foreach (DroneFake drone in drones.FindAll(d => d.isMovable))
        {
            drone.ComputeForces(MigrationPointController.alignementVector, network);
            drone.score = network.IsInMainNetwork(drone) ? 1.0f : 0.0f;
        }

        foreach (Transform drone in swarmHolder.transform)
        {
            drone.GetComponent<DroneController>().droneFake.UpdatePosition();
            if (drone.GetComponent<DroneController>().droneFake.hasCrashed)
            {
                drone.GetComponent<DroneController>().crash();
            }
        }

    }

    void spawnless()
    {
        drones.Clear();
        int i = 0;

        int droneID = LevelConfiguration._droneID;
        bool startEmbodied = LevelConfiguration._startEmbodied;

        foreach (Transform drone in swarmHolder.transform)
        {
        //    fogWar.AddFogRevealer(drone, 1, true);
            drone.GetComponent<DroneController>().droneFake = new DroneFake(drone.transform.position, Vector3.zero, false, i);
            drones.Add(drone.GetComponent<DroneController>().droneFake);

            if (startEmbodied && i == droneID)
            {
                CameraMovement.SetEmbodiedDrone(drone.gameObject);
                CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.embodied = true;
            }

            i++;
        }

        //select the first drone as the selected drone
        if (LevelConfiguration._droneID < drones.Count && LevelConfiguration._droneID != -1)
        {
            if(!LevelConfiguration._startEmbodied)
            {
                MigrationPointController.selectedDrone = swarmHolder.transform.GetChild(LevelConfiguration._droneID).gameObject;
                MigrationPointController.idLeader = MigrationPointController.selectedDrone.GetComponent<DroneController>().droneFake.id;
            }
            
        }
        else
        {
            print("Drone ID not found");
        }

        getDummies();




        //   this.GetComponent<HapticAudioManager>().Reset();

        return;
    }

    void getDummies()
    {
        //get all the tag dummies
        GameObject[] dummies = GameObject.FindGameObjectsWithTag("Dummy");
        //add it to the drones list
        foreach (GameObject dummy in dummies)
        {
            dummy.GetComponent<DroneController>().droneFake = new DroneFake(dummy.transform.position, Vector3.zero, false, drones.Count, isMovable: false);
            drones.Add(dummy.GetComponent<DroneController>().droneFake);
        }
    }

    void spawn()
    {
        desiredSeparation = 3f;
        if (!needToSpawn)
        {
            spawnless();
            return;
        }

        GameObject[] dronesToDelete = GameObject.FindGameObjectsWithTag("Drone");
        //kill all drones
        foreach (GameObject drone in dronesToDelete)
        {
            Destroy(drone.gameObject);
        }

        drones.Clear();

        bool startEmbodied = LevelConfiguration._startEmbodied;
        int droneID = LevelConfiguration._droneID;


        for (int i = 0; i < numDrones; i++)
        {
            //spawn on a circle
            Vector3 spawnPosition = new Vector3(spawnRadius * Mathf.Cos(i * 2 * Mathf.PI / numDrones), spawnHeight + UnityEngine.Random.Range(-0.5f, 0.5f), spawnRadius * Mathf.Sin(i * 2 * Mathf.PI / numDrones));

            GameObject drone = Instantiate(dronePrefab, spawnPosition, Quaternion.identity);

            drone.GetComponent<DroneController>().droneFake = new DroneFake(spawnPosition, Vector3.zero, false, i);

         //   fogWar.AddFogRevealer(drone.transform, 1, true);

            drones.Add(drone.GetComponent<DroneController>().droneFake);

            drone.transform.parent = swarmHolder.transform;
            drone.name = "Drone" + i.ToString();

            if (startEmbodied && i == droneID)
            {
                CameraMovement.SetEmbodiedDrone(drone);
                CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.embodied = true;
            }else if(!startEmbodied && i == droneID){
                MigrationPointController.selectedDrone = drone;
                MigrationPointController.idLeader = i;
            }
        }

        getDummies();

    }

    public void RemoveDrone(GameObject drone)
    {
        if (drone.transform.parent == swarmHolder.transform)
        {
            drone.gameObject.SetActive(false);
            drone.transform.parent = null;
            //this.GetComponent<CameraMovement>().resetFogExplorers();
        }

        this.GetComponent<Timer>().DroneDiedCallback();

        if (swarmHolder.transform.childCount == 0)
        {
            restartFunction();
        }

        drones.Remove(drone.GetComponent<DroneController>().droneFake);
    }

    public void UpdateSwarmForces()
    {
        List<Vector3> allForcesObstacle = new List<Vector3>();
        List<Vector3> allForcesOlfati = new List<Vector3>();

        swarmObstacleForces.Clear();
        swarmOlfatiForces.Clear();

        // if (CameraMovement.embodiedDrone == null)
        // {
            foreach (DroneFake drone in network.drones)
            {
                allForcesObstacle.AddRange(drone.lastObstacleForcesFeedback);
                allForcesOlfati.Add(drone.lastOlfati);
            }

            swarmObstacleForces = ForceClusterer.getForcesObstacle(allForcesObstacle, 20f, (int)network.largestComponent.Count / 6, 2);
            // foreach (Vector3 f in swarmObstacleForces)
            //     Debug.Log($"[Swarm] obstacleForce = {f:F2}");
            if (swarmObstacleForces.Count > 0)
            {
                // ① 把每个 Vector3 转成 "[(x,y,z)]" 字符串
                string[] forceStrings = new string[swarmObstacleForces.Count];
                for (int i = 0; i < swarmObstacleForces.Count; i++)
                {
                    Vector3 f = swarmObstacleForces[i];
                    forceStrings[i] = $"[{f.x:F2}, {f.y:F2}, {f.z:F2}]";
                }

                // ② 用逗号拼接成一行
                string joined = string.Join(", ", forceStrings);

                // ③ 一次性打印
                //Debug.Log($"SwarmObstacleForces ({swarmObstacleForces.Count}): {joined}");
            }
            else
            {
                //Debug.Log("SwarmObstacleForces: (none)");
            }
            
            swarmOlfatiForces = ForceClusterer.getOlfatiForces(allForcesOlfati, 35f, (int)network.largestComponent.Count / 5, 1);
        // }
        // else
        // {

        //     swarmOlfatiForces.Add(CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.lastOlfati);
        //     (List<Vector3> forces, bool hasCrashed) = CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.getObstacleForces(HapticsTest._distanceDetection, HapticsTest._distanceDetection, 1f);
        //     swarmObstacleForces = forces;
        // }
    }
    //void on Guizmos
    void OnDrawGizmos()
    {
        // Gizmos.color = Color.red;
        List<Obstacle> obstacles = ClosestPointCalculator.obstaclesInRange;
        if (obstacles == null)
        {
            return;
        }


        // obstacle loaded for compurtation
        if (showObstacleGizmos)
        {
            foreach (Obstacle obstacle in obstacles)
            {
                Gizmos.DrawWireSphere(obstacle.centerObs, 0.1f);
            }
        }

        if (showNetworkConnexionVulnerability)
        {
            SwarmDisconnection.CheckDisconnection();
        }


        //draw the network
        // if (network == null)
        // {
        //     return;
        // }
        if (network == null && !showDroneObstacleForces)
        {
            return;    // 没网络也不想画无人机蓝线时才退出
        }


        if (showNetworkGizmos)
        {
            foreach (DroneFake drone in network.drones)
            {
                foreach (DroneFake neighbour in network.GetNeighbors(drone))
                {
                    int minLayer = Mathf.Min(drone.layer, neighbour.layer);
                    if (minLayer == 0)
                    {
                        Gizmos.color = Color.black;
                    }
                    else if (minLayer == 1)
                    {
                        Gizmos.color = Color.green;
                    }
                    else if (minLayer == 2)
                    {
                        Gizmos.color = Color.cyan;
                    }
                    else if (minLayer == 3)
                    {
                        Gizmos.color = Color.blue;
                    }
                    else if (minLayer == 4)
                    {
                        Gizmos.color = Color.cyan;
                    }
                    else if (minLayer == 5)
                    {
                        Gizmos.color = Color.blue;
                    }
                    else if (minLayer == 6)
                    {
                        Gizmos.color = Color.grey;
                    }
                    Gizmos.DrawLine(drone.position, neighbour.position);
                }

            }
        }


        // if (CameraMovement.embodiedDrone == null)
        // {
            if (showSwarmObstacleForces)
            {
                /*----------------------------------------------------*
                * 0) 收集所有无人机 Transform
                *----------------------------------------------------*/
                var drones = FindObjectsOfType<DroneController>()
                                .Select(d => d.transform).ToList();
                if (drones.Count == 0) return;

                /*----------------------------------------------------*
                * 1) 计算几何中心（与你可视化 Cyan 球用的同一函数）
                *----------------------------------------------------*/
                Vector3 centroid = HapticsTest.GetSwarmCentroid(drones);

                /*----------------------------------------------------*
                * 2) 可选：仍然画 cyan 球，方便确认位置
                *----------------------------------------------------*/
                // Gizmos.color = Color.cyan;
                // Gizmos.DrawSphere(centroid, 0.5f);
                // Gizmos.DrawLine(centroid, centroid + Vector3.up);

                const float SCALE = 0.1f;         // 视觉放大系数，可自行调
                const float THICKNESS = 0.25f; // “线粗” (世界单位)
                Gizmos.color = Color.blue;
                foreach (Vector3 f in swarmObstacleForces)
                {
                    // Gizmos.color = Color.blue;
                    // Gizmos.DrawLine(Camera.main.transform.position, Camera.main.transform.position + force * 10);
                    // Debug.Log($"Gizmos called, emb={CameraMovement.embodiedDrone}, forces={swarmObstacleForces.Count}");
                    if (f == Vector3.zero) continue;          // 忽略零向量

                    if (f.y != -0.0f) continue;    // 阈值可调

                    Vector3 tip  = centroid + f * SCALE;
                    // Gizmos.DrawLine(centroid, tip);           // 主干

                    Vector3 dir     = f.normalized;
                    float   length  = f.magnitude * SCALE;
                    Quaternion rot  = Quaternion.LookRotation(dir, Vector3.up);
                    Vector3    pos  = centroid + dir * (length * 0.5f);  // 盒子中心
                    Vector3    size = new Vector3(THICKNESS, THICKNESS, length);

                    Matrix4x4 m = Matrix4x4.TRS(pos, rot, Vector3.one);

                    /*―― ③ 画 WireCube ――*/
                    Gizmos.color = Color.blue;

                    Matrix4x4 old = Gizmos.matrix;    // 备份
                    Gizmos.matrix = m;                // 切到局部空间
                    Gizmos.DrawCube(Vector3.zero, size);
                    Gizmos.matrix = old;              // 还原

                    /* —— 简易箭头 —— */
                    // Vector3 dir  = f.normalized;
                    float    len = f.magnitude * SCALE * 0.2f;
                    Vector3 l = tip + Quaternion.AngleAxis(160, Vector3.up) * dir * len;
                    Vector3 r = tip + Quaternion.AngleAxis(-160,Vector3.up) * dir * len;
                    Gizmos.DrawLine(tip, l);
                    Gizmos.DrawLine(tip, r);

                }
            }

            if (showSwarmOlfati)
            {
                foreach (Vector3 force in swarmOlfatiForces)
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawLine(Camera.main.transform.position, Camera.main.transform.position + force * 10);


                }
            }
        // }

        // if (showDroneObstacleForces)
        // {
        //     (List<Vector3> forces, bool hasCrashed) = CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.getObstacleForces(HapticsTest._distanceDetection, HapticsTest._distanceDetection, 1f);
        //     Vector3 pos = CameraMovement.embodiedDrone.transform.position;

        //     foreach (Vector3 force in forces)
        //     {
        //         Gizmos.color = Color.blue;
        //         Gizmos.DrawLine(pos, pos + force * 10);
        //     }
        // }
        
        // if (showDroneObstacleForces && CameraMovement.embodiedDrone != null)
        // {
        //     var dc = CameraMovement.embodiedDrone.GetComponent<DroneController>();
        //     if (dc != null && dc.droneFake != null)
        //     {
        //         var tuple = dc.droneFake.getObstacleForces(
        //                         HapticsTest._distanceDetection,
        //                         HapticsTest._distanceDetection,
        //                         1f);
        //         List<Vector3> forces = tuple.forces;
        //         Vector3 pos = CameraMovement.embodiedDrone.transform.position;

        //         Gizmos.color = Color.blue;
        //         foreach (Vector3 f in forces)
        //             Gizmos.DrawLine(pos, pos + f * 10f);
        //     }
        // }

        

    }

    void swarmAskingShrink()
    {
        List<float> scores = new List<float>();
        foreach (DroneFake drone in network.drones)
        {
            if (!network.IsInMainNetwork(drone))
            {
                continue;
            }


            List<DroneFake> neighbors = network.GetNeighbors(drone);

            if (neighbors.Count == 0)
            {
                continue;
            }
            neighbors = neighbors.OrderBy(d => Vector3.Distance(d.position, drone.position)).ToList();
            int number = (int)(network.drones.Count * 0.3f);
            if (number > neighbors.Count)
            {
                number = neighbors.Count;
            }

            if (number == 0)
            {
                continue;
            }

            List<DroneFake> mostVulnerable = neighbors.GetRange(0, number);

            float ratioAverageDistance = mostVulnerable.Average(d => Vector3.Distance(d.position, drone.position)) / desiredSeparation;

            float score = ratioAverageDistance;

            scores.Add(score);
        }

        scores = scores.OrderBy(s => s).ToList();

        if (scores.Count == 0)
        {
            BrownToBlueNoise.AnalyseShrinking(0);
            swarmAskingSpreadness = 0;
            return;
        }

        float finalScore = scores.GetRange(0, Mathf.Max((int)(scores.Count * 0.3f), 1)).Average();

        finalScore = (finalScore - 0.55f) / (0.75f - 0.55f);
        finalScore = Mathf.Min(finalScore, 1f);
        finalScore = 1 - Mathf.Max(finalScore, 0f);

        BrownToBlueNoise.AnalyseShrinking(finalScore);
        swarmAskingSpreadness = finalScore;
    }

    void swarmSeparation(int numberOfCut)
    {

        // Get average position and velocity of each group
        if (CameraMovement.embodiedDrone == null)
        {
            List<DroneFake> dronesInDirection = network.largestComponent.OrderBy(d => Vector3.Dot(d.position, MigrationPointController.alignementVectorNonZero.normalized)).ToList();

            // Split the drones into the specified number of groups
            List<List<DroneFake>> droneGroups = new List<List<DroneFake>>();
            int groupSize = dronesInDirection.Count / numberOfCut;
            for (int i = 0; i < numberOfCut; i++)
            {
                int start = i * groupSize;
                int count = (i == numberOfCut - 1) ? dronesInDirection.Count - start : groupSize;
                droneGroups.Add(dronesInDirection.GetRange(start, count));
            }
            List<(Vector3 position, Vector3 velocity)> averages = new List<(Vector3 position, Vector3 velocity)>();
            foreach (var group in droneGroups)
            {
                var avg = group.Aggregate((Vector3.zero, Vector3.zero), (acc, d) => (acc.Item1 + d.position / group.Count, acc.Item2 + d.velocity / group.Count));
                averages.Add(avg);
            }

            // Draw Gizmos for each group
            List<float> scores = new List<float>();
            for (int i = 0; i < averages.Count; i++)
            {
                float diffPos = (i < averages.Count - 1) ? Vector3.Dot(averages[i + 1].position, MigrationPointController.alignementVectorNonZero.normalized) - Vector3.Dot(averages[i].position, MigrationPointController.alignementVectorNonZero.normalized) : 0;


                float fakeDesiredSeparation = desiredSeparation;
                if (desiredSeparation < 2.5)
                {
                    fakeDesiredSeparation = 1 / 3 * (desiredSeparation - 1) + 2;
                }
                float score = 2f * diffPos / fakeDesiredSeparation - 1;
                scores.Add(score);

            }

            float scoreFinal = Mathf.Max(Mathf.Min(scores.Max(), 1.5f), 0f);
            scoreFinal = Mathf.Max((scoreFinal - 0.3f) / (1.5f - 0.3f), 0f);

            swarmConnectionScore = scoreFinal;

        }
        else
        { // embodied drone only looking at the 1st order neighnbor
            swarmConnectionScore = this.GetComponent<NetworkRepresentation>().UpdateNetworkRepresentation(network.getLayersConfiguration());
        }

    }

    private void UpdateCurrentScores()
    {
        lock (scoreLock)
        {
            currentScores.connectionScore = swarmConnectionScore;
            currentScores.spreadnessScore = swarmAskingSpreadness;
            currentScores.disconnectedCount = numberOfDroneDiscionnected;
            currentScores.crashedCount = numberOfDroneCrashed;
            currentScores.minDistance = minDistance;
        }
    }

    private void PlotNetworkScores()
    {
        while (isThreadRunning)
        {
            NetworkScores scores;
            List<DroneFake> currentDrones;
            Vector3 currentAlignmentVector;

            lock (networkLock)
            {
                currentDrones = new List<DroneFake>(currentScores.dronesSnapshot);
                currentAlignmentVector = currentScores.alignmentVector;
            }

            NetworkCreator networkToComputeFirst = new NetworkCreator(currentDrones);
            networkToComputeFirst.refreshNetwork();




            List<DroneFake> movableDrone = networkToComputeFirst.drones.FindAll(d => d.isMovable);
            List<DroneFake> connectedDrone = networkToComputeFirst.drones.FindAll(d => networkToComputeFirst.IsInMainNetwork(d)); // including the dummy drones
                                                                                                                                  //merge the 2 without doublons

            List<DroneFake> connectedMovableDrone = connectedDrone.Union(movableDrone).ToList();

            NetworkCreator networkToCompute = new NetworkCreator(connectedDrone);
            networkToCompute.refreshNetwork();

            //          Debug.Log("Computing scores for " + connectedMovableDrone.Count + " drones");


            // Calculate all scores in the thread
            float velMissmatch = networkToCompute.ComputeNormalizedVelocityMismatch();
            float energyDev = networkToCompute.ComputeNormalizedDeviationEnergy();
            float relativeConnectivity = networkToCompute.ComputeRelativeConnectivity();
            float cohesionRadius = networkToCompute.ComputeCohesionRadius();



            //Debug.Log($"Velocity Missmatch: {velMissmatch}" +
            //          $"  Energy Deviation: {energyDev}" +
            //          $"  Relative Connectivity: {relativeConnectivity}" +
            //          $"  Cohesion Radius: {cohesionRadius}" +
            //          $"   Computing scores for " + networkToCompute.drones.Count);
            //drones number



            Thread.Sleep((int)(printInterval * 1000));
        }
    }

    void OnApplicationQuit()
    {
        print("Application ending after " + Time.time + " seconds");
        isThreadRunning = false;
        if (scorePlottingThread != null && scorePlottingThread.IsAlive)
        {
            scorePlottingThread.Join(100); // Wait up to 100ms for thread to finish
        }

        if (saveData)
        {
            saveInfoToJSON.exportData(true);
        }

    }
}
