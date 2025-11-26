using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;
using Unity.VisualScripting;

/// <summary>
/// Swarm-as-ellipsoid obstacle response model (swarm treated as a single ellipsoid).
/// - Uses the closest point on each obstacle to the swarm centroid.
/// - Direction kept as INWARD (centroid - closestPoint), then projected to XZ plane for viz/mapping.
/// - Intensity = distance-to-ellipsoid-surface mapping, multiplied by an activation gate so it ramps from 0 after entering a radius.
/// - Directions mapped to nearest actuator; intensities exported via actuatorIntensities.
/// </summary>
public class swarmModel : MonoBehaviour
{
    #region Parameters (legacy compatibility)

    public static int idLeader
    {
        get
        {
            if (LevelConfiguration._startEmbodied)
                return CameraMovement.idLeader;
            return MigrationPointController.idLeader;
        }
    }

    // prints once every 2s so the Console doesn't flood
    private float _idPrintCooldown = 0f;

    public bool saveData => LevelConfiguration._SaveData;
    public bool needToSpawn => LevelConfiguration._NeedToSpawn;

    public static GameObject swarmHolder;
    public GameObject dronePrefab;

    public int numDrones => LevelConfiguration._NumDrones;
    public float spawnRadius => LevelConfiguration._SpawnRadius;
    public static float spawnHeight = 5f;

    public float maxSpeed = 5f;
    public float maxForce = 10f;

    public static float desiredSeparation = 5f;
    public static float extraDistanceNeighboor = 2f;
    public static float neighborRadius
    {
        get { return Mathf.Max(2f * desiredSeparation, desiredSeparation + extraDistanceNeighboor); }
    }

    public float alpha = 1.5f;  // separation weight
    public float beta = 1.0f;   // alignment weight
    public float delta = 1.0f;  // migration weight
    public float cVm = 1f;      // max velocity change

    [Header("Obstacle/Avoidance (legacy compatibility)")]
    public float avoidanceRadius = 1.0f;
    public float avoidanceRadiusFeedback = 3f;
    public float desiredSeparationObs = 3f;
    public float avoidanceForce = 10f;
    public static float droneRadius = 0.17f;
    public LayerMask obstacleLayer;

    public int PRIORITYWHENEMBODIED = 15;
    public float dampingFactor = 0.98f;

    public static NetworkCreator network;

    public static float minDistance = float.MaxValue;
    public static List<DroneFake> drones = new List<DroneFake>();

    public static Vector3 swarmVelocityAvg = Vector3.zero;
    public static float swarmConnectionScore = 0f;
    public static float avgDist;
    public static float swarmAskingSpreadness = 0f;

    public static int numberOfDroneDiscionnected => SwarmDisconnection.dronesID.Count;
    public static int numberOfDroneCrashed => Timer.numberDroneDied;

    [Header("Gizmos")]
    public bool showObstacleGizmos = false;
    public bool showNetworkGizmos = false;
    public bool showNetworkConnexionVulnerability = false;
    public bool showNetworkConnectivity = false;
    public bool showDroneObstacleForces = false;
    public bool showDroneOlfatiForces = false;
    public bool showDroneAllignementForces = false;
    public bool showSwarmObstacleForces = true;
    public bool showSwarmOlfati = false;

    [Header("Ellipsoid Force Gizmos")]
    [Tooltip("Draw actuator rays colored by intensity (from centroid).")]
    public bool showActuatorRays = true;

    [Tooltip("Color at low intensity (0).")]
    public Color gizmoLowColor = new Color(0.2f, 0.6f, 1f, 0.8f);

    [Tooltip("Color at high intensity (1).")]
    public Color gizmoHighColor = new Color(0.1f, 1f, 0.2f, 0.95f);

    [Header("Ellipsoid Force Gizmos – Debug/Visual")]
    public bool gizmoForcePreviewFallback = true;
    public bool gizmoShowHUD = true;

    [Tooltip("Arrow length scale in Scene view.")]
    public float gizmoArrowScale = 1.2f;

    [Tooltip("Arrow head size.")]
    public float gizmoArrowHead = 0.20f;

    [Tooltip("Line thickness for Handles.")]
    public float gizmoLineThickness = 8.0f;

    [Tooltip("Don't draw arrows if intensity < this (visual only).")]
    public float gizmoVisualMin = 0.02f;

    public static bool dummyForcesApplied = true;

    #endregion

    #region New ellipsoid-based obstacle response fields

    [Header("Ellipsoid model")]
    [Tooltip("Scale factor applied to computed axis (X,Y,Z). Acts like a safety margin.")]
    public Vector3 axisScale = new Vector3(1.0f, 1.0f, 1.0f);

    [Tooltip("Minimum ellipsoid semi-axes (avoid degeneracy).")]
    public Vector3 axisFloor = new Vector3(0.75f, 0.5f, 0.75f);

    [Tooltip("Clamp of intensity after distance mapping.")]
    public Vector2 intensityClamp01 = new Vector2(0, 1);

    [Tooltip("Intensity falloff exponent. >1 makes it sharper.")]
    public float intensityGamma = 1.0f;

    [Tooltip("Max detection distance to consider an obstacle (world units). 0 = ignore (use ClosestPointCalculator list only).")]
    public float maxObstacleDistance = 0f;

    [Header("Actuator mapping")]
    [Tooltip("Actuator directions in WORLD space. Will be normalized.")]
    public List<Vector3> actuatorDirections = new List<Vector3>();

    [Tooltip("Resulting actuator intensities (auto-sized to directions).")]
    public float[] actuatorIntensities;

    [Tooltip("If true, take max per actuator; else sum and clamp.")]
    public bool actuatorUseMax = true;

    // For gizmos: store per-obstacle world vectors = dir * intensity
    public static List<Vector3> swarmObstacleForces = new List<Vector3>();
    public static List<Vector3> swarmOlfatiForces = new List<Vector3>(); // reserved

    [Header("Activation gate (distance → 0..1)")]
    [Tooltip("Within this centroid→closestPoint distance, force ramps from 0 to full. 0 = disabled.")]
    public float swarmActivationRadius = 5.0f;

    [Tooltip("Use horizontal (XZ) distance for activation distance.")]
    public bool activationHorizontalOnly = false;

    [Tooltip("Use SmoothStep for a softer ramp instead of linear.")]
    public bool activationUseSmoothStep = true;

    [Tooltip("Optional extra shaping: gain^gamma. 1 = no change.")]
    public float activationGamma = 1.0f;

    [Header("Swarm Spawn Center (World)")]
    public Vector3 spawnCenterWorld = Vector3.zero;

    // 方式B：可选：用场景中的一个锚点（优先级更高）
    public Transform spawnCenterAnchor;

    #endregion


    [Header("Flocking Runtime Tuning")]
    [Range(0f, 20f)]
    public float extraDistanceNeighboorRuntime = 5f;

    /// <summary>
    /// 运行时设置接口：设置后立即刷新邻域半径与网络
    /// 用法：FindObjectOfType<swarmModel>().ExtraDistanceNeighboor = 8f;
    /// </summary>
    public float ExtraDistanceNeighboor
    {
        get => extraDistanceNeighboor; // 仍使用你现有的 static 值作为权威源
        set
        {
            float v = Mathf.Max(0f, value);
            if (Mathf.Approximately(v, extraDistanceNeighboor)) return;

            // 1) 写回 static（影响 neighborRadius 计算）
            extraDistanceNeighboor = v;

            // 2) 同步到 Inspector 滑块（保持面板一致）
            extraDistanceNeighboorRuntime = v;

            // 3) 立即下发到 DroneFake.neighborRadius，并刷新邻接网络
            //    refreshParameters() 内部会把 DroneFake.neighborRadius = neighborRadius（你已有的逻辑）
            refreshParameters();
            if (network != null)
            {
                network.refreshNetwork();
            }
        }
    }

    #region Internal score-thread scaffolding

    private class NetworkScores
    {
        public float connectionScore;
        public float spreadnessScore;
        public int disconnectedCount;
        public int crashedCount;
        public float minDistance;
        public List<DroneFake> dronesSnapshot = new List<DroneFake>();
        public Vector3 alignmentVector;
    }

    private NetworkScores currentScores = new NetworkScores();
    private readonly object scoreLock = new object();
    private readonly object networkLock = new object();
    private Thread scorePlottingThread;
    private bool isThreadRunning = false;
    private float printInterval = 0.5f;

    #endregion

    #region Unity Lifecycle

    void Start()
    {
        TriggerHandlerWithCallback.setGM(this.gameObject);

        Application.targetFrameRate = 30;
        PRIORITYWHENEMBODIED = (int)(numDrones / 3.5f);

        swarmHolder = needToSpawn ? GameObject.FindGameObjectWithTag("Swarm") : LevelConfiguration.swarmHolder;
        if (swarmHolder == null)
        {
            Debug.LogWarning("No swarm holder found, creating one.");
            swarmHolder = new GameObject("Swarm");
        }

        spawn();

        desiredSeparation = LevelConfiguration._StartSperation;

        // prepare actuators array
        ResizeActuatorArray();

        isThreadRunning = false;
        // scorePlottingThread = new Thread(PlotNetworkScores);
        // scorePlottingThread.Start();

        extraDistanceNeighboorRuntime = extraDistanceNeighboor;

    }

    void FixedUpdate()
    {
        if (Input.GetKeyDown(KeyCode.R))
            restartFunction();

        refreshSwarm();

        // === New: Ellipsoid-based obstacle evaluation ===
        UpdateSwarmForcesEllipsoid();

        getSwarmConnexion();
        swarmAskingShrink();

        UpdateCurrentScores();

        lock (networkLock)
        {
            currentScores.dronesSnapshot = new List<DroneFake>(drones);
            currentScores.alignmentVector = MigrationPointController.alignementVectorNonZero;
        }

        // 当你在 Inspector 拖动 extraDistanceNeighboorRuntime 时，自动应用到 static，并立即刷新网络
        if (!Mathf.Approximately(extraDistanceNeighboorRuntime, extraDistanceNeighboor))
        {
            ExtraDistanceNeighboor = extraDistanceNeighboorRuntime;
        }

        // ids check
        // _idPrintCooldown -= Time.fixedDeltaTime;
        // if (_idPrintCooldown <= 0f)
        // {
        //     _idPrintCooldown = 2.0f;

        //     var seenId  = new HashSet<int>();
        //     var seenIdS = new HashSet<string>();

        //     foreach (var d in drones)
        //     {
        //         if (d == null) continue;
        //         string s = string.IsNullOrEmpty(d.idS) ? "(null/empty)" : d.idS;
        //         // Debug.Log($"[ID CHECK] name:{d.gameObject?.name}  id:{d.id}  idS:{s}");
        //         Debug.Log($"[ID CHECK] id:{d.id}  idS:{s}");


        //         if (!seenId.Add(d.id))
        //             Debug.LogWarning($"[ID COLLISION] duplicate id: {d.id}");
        //         if (!seenIdS.Add(s))
        //             Debug.LogWarning($"[ID COLLISION] duplicate idS: {s}");
        //     }

        //     Debug.Log($"[ID SUMMARY] N={drones.Count}  unique id:{seenId.Count}  unique idS:{seenIdS.Count}");
        // }

        
    }

    void OnApplicationQuit()
    {
        Debug.Log($"Application ending after {Time.time:F1} seconds");
        isThreadRunning = false;
        if (scorePlottingThread != null && scorePlottingThread.IsAlive)
            scorePlottingThread.Join(100);

        // if (saveData)
        //     saveInfoToJSON.exportData(true);
    }

    #endregion

    #region Spawning & refresh

    public static void restart()
    {
        GameObject.FindGameObjectWithTag("GameManager").GetComponent<swarmModel>().restartFunction();
    }

    void restartFunction()
    {
        Debug.Log("----------------------------- Restarting -----------------------------");
        SceneSelectorScript.reset();
    }

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

    void refreshSwarm()
    {
        refreshParameters();

        network = new NetworkCreator(drones);
        network.refreshNetwork();
        Dictionary<int, int> layers = network.getLayersConfiguration();
        this.GetComponent<NetworkRepresentation>().UpdateNetworkRepresentation(layers);

        // Update obstacle list
        ClosestPointCalculator.selectObstacle(drones);

        // Physics/logic update per drone
        foreach (DroneFake df in drones.FindAll(d => d.isMovable))
        {
            df.ComputeForces(MigrationPointController.alignementVector, network);
            df.score = network.IsInMainNetwork(df) ? 1.0f : 0.0f;
        }

        foreach (Transform child in swarmHolder.transform)
        {
            var dc = child.GetComponent<DroneController>();
            if (dc == null || dc.droneFake == null) continue;

            dc.droneFake.UpdatePosition();
            if (dc.droneFake.hasCrashed)
                dc.crash();
        }
    }

    void spawn()
    {
        desiredSeparation = 5f;

        if (!needToSpawn)
        {
            spawnless();
            return;
        }

        GameObject[] dronesToDelete = GameObject.FindGameObjectsWithTag("Drone");
        foreach (GameObject go in dronesToDelete)
            Destroy(go);

        drones.Clear();

        bool startEmbodied = LevelConfiguration._startEmbodied;
        int droneID = LevelConfiguration._droneID;

        Vector3 center = (spawnCenterAnchor != null) ? spawnCenterAnchor.position : spawnCenterWorld;


        for (int i = 0; i < numDrones; i++)
        {
            // Vector3 spawnPosition = new Vector3(
            //     spawnRadius * Mathf.Cos(i * 2 * Mathf.PI / numDrones),
            //     spawnHeight + UnityEngine.Random.Range(-0.5f, 0.5f),
            //     spawnRadius * Mathf.Sin(i * 2 * Mathf.PI / numDrones));
            Vector3 spawnPosition = center + new Vector3(
                spawnRadius * Mathf.Cos(i * 2 * Mathf.PI / numDrones),
                spawnHeight + UnityEngine.Random.Range(-0.5f, 0.5f),
                spawnRadius * Mathf.Sin(i * 2 * Mathf.PI / numDrones));


            GameObject drone = Instantiate(dronePrefab, spawnPosition, Quaternion.identity);
            drone.GetComponent<DroneController>().droneFake = new DroneFake(spawnPosition, Vector3.zero, false, i);
            drones.Add(drone.GetComponent<DroneController>().droneFake);

            drone.transform.parent = swarmHolder.transform;
            drone.name = "Drone" + i;

            if (startEmbodied && i == droneID)
            {
                CameraMovement.SetEmbodiedDrone(drone);
                CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.embodied = true;
            }
            else if (!startEmbodied && i == droneID)
            {
                MigrationPointController.selectedDrone = drone;
                MigrationPointController.idLeader = i;
            }
        }

        getDummies();
    }

    void spawnless()
    {
        drones.Clear();
        int i = 0;

        int droneID = LevelConfiguration._droneID;
        bool startEmbodied = LevelConfiguration._startEmbodied;

        foreach (Transform t in swarmHolder.transform)
        {
            var dc = t.GetComponent<DroneController>();
            if (dc == null) continue;

            dc.droneFake = new DroneFake(t.position, Vector3.zero, false, i);
            drones.Add(dc.droneFake);

            if (startEmbodied && i == droneID)
            {
                CameraMovement.SetEmbodiedDrone(t.gameObject);
                CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.embodied = true;
            }
            i++;
        }

        if (LevelConfiguration._droneID < drones.Count && LevelConfiguration._droneID != -1)
        {
            if (!LevelConfiguration._startEmbodied)
            {
                MigrationPointController.selectedDrone = swarmHolder.transform.GetChild(LevelConfiguration._droneID).gameObject;
                MigrationPointController.idLeader = MigrationPointController.selectedDrone.GetComponent<DroneController>().droneFake.id;
            }
        }
        else
        {
            Debug.LogWarning("Drone ID not found");
        }

        getDummies();
    }

    void getDummies()
    {
        GameObject[] dummies = GameObject.FindGameObjectsWithTag("Dummy");
        foreach (GameObject dummy in dummies)
        {
            var dc = dummy.GetComponent<DroneController>();
            if (dc == null) continue;

            dc.droneFake = new DroneFake(dummy.transform.position, Vector3.zero, false, drones.Count, isMovable: false);
            drones.Add(dc.droneFake);
        }
    }

    public void RemoveDrone(GameObject drone)
    {
        if (drone.transform.parent == swarmHolder.transform)
        {
            drone.SetActive(false);
            drone.transform.parent = null;
        }

        this.GetComponent<Timer>().DroneDiedCallback();

        if (swarmHolder.transform.childCount == 0)
            restartFunction();

        var dc = drone.GetComponent<DroneController>();
        if (dc != null) drones.Remove(dc.droneFake);
    }

    #endregion

    #region Ellipsoid-based obstacle evaluation

    // private static (Vector3 centroid, Vector3 axes) ComputeSwarmEllipsoidAxes(List<Transform> droneTs, Vector3 axisFloor, Vector3 axisScale)
    // {
    //     if (droneTs == null || droneTs.Count == 0)
    //         return (Vector3.zero, Vector3.one);

    //     Vector3 c = Vector3.zero;
    //     foreach (var t in droneTs) c += t.position;
    //     c /= droneTs.Count;

    //     float maxX = 0f, maxY = 0f, maxZ = 0f;
    //     foreach (var t in droneTs)
    //     {
    //         Vector3 d = t.position - c;
    //         maxX = Mathf.Max(maxX, Mathf.Abs(d.x));
    //         maxY = Mathf.Max(maxY, Mathf.Abs(d.y));
    //         maxZ = Mathf.Max(maxZ, Mathf.Abs(d.z));
    //     }

    //     Vector3 axes = new Vector3(
    //         Mathf.Max(maxX, axisFloor.x) * Mathf.Max(axisScale.x, 0.0001f),
    //         Mathf.Max(maxY, axisFloor.y) * Mathf.Max(axisScale.y, 0.0001f),
    //         Mathf.Max(maxZ, axisFloor.z) * Mathf.Max(axisScale.z, 0.0001f)
    //     );

    //     return (c, axes);
    // }

    // ===============================
    // 1) ComputeSwarmEllipsoidAxes
    // ===============================

    // Primary overload: work with positions
    private static (Vector3 centroid, Vector3 axes) ComputeSwarmEllipsoidAxes(
        List<Vector3> points, Vector3 axisFloor, Vector3 axisScale)
    {
        if (points == null || points.Count == 0)
            return (Vector3.zero, Vector3.one);

        // Centroid
        Vector3 c = Vector3.zero;
        for (int i = 0; i < points.Count; i++)
            c += points[i];
        c /= points.Count;

        // Half-extent along world axes
        float maxX = 0f, maxY = 0f, maxZ = 0f;
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 d = points[i] - c;
            float ax = Mathf.Abs(d.x);
            float ay = Mathf.Abs(d.y);
            float az = Mathf.Abs(d.z);
            if (ax > maxX) maxX = ax;
            if (ay > maxY) maxY = ay;
            if (az > maxZ) maxZ = az;
        }

        // Apply floors and axis scaling
        Vector3 axes = new Vector3(
            Mathf.Max(maxX, axisFloor.x) * Mathf.Max(axisScale.x, 0.0001f),
            Mathf.Max(maxY, axisFloor.y) * Mathf.Max(axisScale.y, 0.0001f),
            Mathf.Max(maxZ, axisFloor.z) * Mathf.Max(axisScale.z, 0.0001f)
        );

        return (c, axes);
    }

    // Compatibility shim: accepts List<Transform> and forwards to the primary overload
    private static (Vector3 centroid, Vector3 axes) ComputeSwarmEllipsoidAxes(
        List<Transform> droneTs, Vector3 axisFloor, Vector3 axisScale)
    {
        List<Vector3> pts = (droneTs == null) ? null : droneTs.Select(t => t.position).ToList();
        return ComputeSwarmEllipsoidAxes(pts, axisFloor, axisScale);
    }



    /// <summary>
    /// Intensity from distance to ellipsoid surface along direction centroid→obstaclePos.
    /// Returns 0..1 (after clamp/gamma).
    /// </summary>
    private float ComputeIntensityFromEllipsoid(Vector3 centroid, Vector3 axes, Vector3 obstaclePos)
    {
        Vector3 v = obstaclePos - centroid;
        float vLen = v.magnitude;
        if (vLen < 1e-4f) return intensityClamp01.y; // degenerate: max

        float denom =
            (v.x * v.x) / (axes.x * axes.x) +
            (v.y * v.y) / (axes.y * axes.y) +
            (v.z * v.z) / (axes.z * axes.z);

        if (denom <= 0f) return 0f;

        float tSurface = 1.0f / Mathf.Sqrt(denom);
        float dToSurface = Mathf.Max(vLen - tSurface, 0f);

        const float k = 1.0f; // falloff
        float intensity = 1.0f / (1.0f + k * dToSurface);
        intensity = Mathf.Clamp(intensity, intensityClamp01.x, intensityClamp01.y);

        if (intensityGamma > 0.001f && Math.Abs(intensityGamma - 1f) > 0.001f)
            intensity = Mathf.Pow(intensity, intensityGamma);

        return intensity;
    }

    private Color ColorByIntensity(float t)
    {
        t = Mathf.Clamp01(t);
        return Color.Lerp(gizmoLowColor, gizmoHighColor, t);
    }

#if UNITY_EDITOR
    private void DrawArrow(Vector3 from, Vector3 dirNorm, float intensity, float scale, float headSize)
    {
        Vector3 to = from + dirNorm * (intensity * scale);
        UnityEditor.Handles.DrawAAPolyLine(gizmoLineThickness, new Vector3[] { from, to });

        float h = Mathf.Lerp(0.05f, headSize, Mathf.Clamp01(intensity));
        Vector3 left = Quaternion.AngleAxis(165f, Vector3.up) * dirNorm * h;
        Vector3 right = Quaternion.AngleAxis(-165f, Vector3.up) * dirNorm * h;

        UnityEditor.Handles.DrawAAPolyLine(gizmoLineThickness, new Vector3[] { to, to + left });
        UnityEditor.Handles.DrawAAPolyLine(gizmoLineThickness, new Vector3[] { to, to + right });
    }
#endif

    private void ResizeActuatorArray()
    {
        if (actuatorDirections == null) actuatorDirections = new List<Vector3>();
        if (actuatorIntensities == null || actuatorIntensities.Length != actuatorDirections.Count)
            actuatorIntensities = new float[actuatorDirections.Count];
    }

    private int FindClosestActuator(Vector3 dir)
    {
        if (actuatorDirections == null || actuatorDirections.Count == 0) return -1;

        dir = dir.normalized;
        float bestDot = -1f;
        int bestIdx = 0;

        for (int i = 0; i < actuatorDirections.Count; i++)
        {
            Vector3 a = actuatorDirections[i].sqrMagnitude < 1e-6f ? Vector3.zero : actuatorDirections[i].normalized;
            float dot = Vector3.Dot(dir, a);
            if (dot > bestDot)
            {
                bestDot = dot;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    public static List<DroneFake> dronesInMainNetwork
    {
        get
        {
            var list = new List<DroneFake>();
            if (network == null) return list;
            foreach (DroneFake d in network.largestComponent) list.Add(d);
            return list;
        }
    }

    /// <summary>
    /// Map distance d to a gate in [0,1]: d>=R→0; d==0→1 (linear or SmoothStep), then ^activationGamma.
    /// </summary>
    private float ActivationGain(float d, float R)
    {
        if (R <= 0f) return 1f;   // disabled
        if (d >= R) return 0f;    // outside
        float t = 1f - (d / R);   // linear ramp (R→0 maps to 0→1)
        if (activationUseSmoothStep)
            t = t * t * (3f - 2f * t);
        if (activationGamma > 0.001f && Mathf.Abs(activationGamma - 1f) > 0.001f)
            t = Mathf.Pow(t, activationGamma);
        return Mathf.Clamp01(t);
    }

    /// <summary>
    /// Main new method: fills swarmObstacleForces and actuatorIntensities.
    /// </summary>
    // public void UpdateSwarmForcesEllipsoid()
    // {
    //     ResizeActuatorArray();

    //     for (int i = 0; i < actuatorIntensities.Length; i++)
    //         actuatorIntensities[i] = 0f;

    //     swarmObstacleForces.Clear();

    //     var droneTs = FindObjectsOfType<DroneController>().Select(d => d.transform).ToList();
    //     if (droneTs.Count == 0) return;

    //     var (centroid, axes) = ComputeSwarmEllipsoidAxes(droneTs, axisFloor, axisScale);

    //     List<Obstacle> obsSnapshot = null;
    //     var src = ClosestPointCalculator.obstaclesInRange;
    //     if (src == null || src.Count == 0) return;
    //     lock (src) { obsSnapshot = new List<Obstacle>(src); }

    //     foreach (var obstacle in obsSnapshot)
    //     {
    //         Vector3 oPos = obstacle.ClosestPoint(centroid);

    //         // Activation distance d
    //         float d = activationHorizontalOnly
    //             ? Vector2.Distance(new Vector2(oPos.x, oPos.z), new Vector2(centroid.x, centroid.z))
    //             : Vector3.Distance(oPos, centroid);

    //         float gain = ActivationGain(d, swarmActivationRadius);
    //         if (gain <= 0f) continue;

    //         if (maxObstacleDistance > 0f && d > maxObstacleDistance) continue;

    //         // INWARD direction (keep as requested)
    //         Vector3 dir3 = -oPos + centroid; // == centroid - oPos
    //         Vector3 dir = Vector3.ProjectOnPlane(dir3, Vector3.up);
    //         if (dir.sqrMagnitude < 1e-6f) continue;
    //         Vector3 ndir = dir.normalized;

    //         // Base intensity (3D). If you want strictly horizontal sensitivity, use pFlat (oPos.x, centroid.y, oPos.z)
    //         float baseIntensity = ComputeIntensityFromEllipsoid(centroid, axes, oPos);

    //         float intensity = baseIntensity * gain;

    //         // Visual min threshold (only affects gizmo & actuator write if desired)
    //         if (intensity < gizmoVisualMin) continue;

    //         swarmObstacleForces.Add(ndir * intensity);

    //         int idx = FindClosestActuator(ndir);
    //         if (idx >= 0 && idx < actuatorIntensities.Length)
    //         {
    //             if (actuatorUseMax)
    //                 actuatorIntensities[idx] = Mathf.Max(actuatorIntensities[idx], intensity);
    //             else
    //                 actuatorIntensities[idx] = Mathf.Clamp(actuatorIntensities[idx] + intensity, intensityClamp01.x, intensityClamp01.y);
    //         }
    //     }
    // }

    public void UpdateSwarmForcesEllipsoid()
    {
        ResizeActuatorArray();
        for (int i = 0; i < actuatorIntensities.Length; i++) actuatorIntensities[i] = 0f;
        swarmObstacleForces.Clear();

        // === Use MAIN GROUP only (largest component); fallback to all drones if unavailable ===
        List<Vector3> pts;
        if (Application.isPlaying && network != null && network.largestComponent != null && network.largestComponent.Count > 0)
            pts = network.largestComponent.Select(df => df.position).ToList();
        else
            pts = FindObjectsOfType<DroneController>().Select(d => d.transform.position).ToList();

        if (pts == null || pts.Count == 0) return;

        var (centroid, axes) = ComputeSwarmEllipsoidAxes(pts, axisFloor, axisScale);

        // Snapshot obstacles
        List<Obstacle> obsSnapshot = null;
        var src = ClosestPointCalculator.obstaclesInRange;
        if (src == null || src.Count == 0) return;
        lock (src) { obsSnapshot = new List<Obstacle>(src); }

        foreach (var obstacle in obsSnapshot)
        {
            Vector3 oPos = obstacle.ClosestPoint(centroid);

            // Activation distance
            float d = activationHorizontalOnly
                ? Vector2.Distance(new Vector2(oPos.x, oPos.z), new Vector2(centroid.x, centroid.z))
                : Vector3.Distance(oPos, centroid);

            float gain = ActivationGain(d, swarmActivationRadius);
            if (gain <= 0f) continue;
            if (maxObstacleDistance > 0f && d > maxObstacleDistance) continue;

            // INWARD direction (centroid - obstacle)
            Vector3 dir3 = centroid - oPos;
            Vector3 dir  = Vector3.ProjectOnPlane(dir3, Vector3.up);
            if (dir.sqrMagnitude < 1e-6f) continue;
            Vector3 ndir = dir.normalized;

            float baseIntensity = ComputeIntensityFromEllipsoid(centroid, axes, oPos);
            // float intensity     = baseIntensity * gain;
            // if (intensity < gizmoVisualMin) continue;

            // swarmObstacleForces.Add(ndir * intensity);
            float intensity = baseIntensity * gain;
            swarmObstacleForces.Add(ndir * intensity);  // always keep it; we’ll filter in gizmo draw

            int idx = FindClosestActuator(ndir);
            if (idx >= 0 && idx < actuatorIntensities.Length)
            {
                if (actuatorUseMax)
                    actuatorIntensities[idx] = Mathf.Max(actuatorIntensities[idx], intensity);
                else
                    actuatorIntensities[idx] = Mathf.Clamp(
                        actuatorIntensities[idx] + intensity, intensityClamp01.x, intensityClamp01.y);
            }
        }
    }



    #endregion

    #region Connectivity / shrink scores

    void getSwarmConnexion()
    {
        List<DroneFake> connectedDrone = network.drones.ToList();
        bool hasNonMovable = drones.Exists(d => !d.isMovable);
        if (hasNonMovable)
            connectedDrone = network.largestComponent.ToList();

        NetworkCreator networkToCompute = new NetworkCreator(connectedDrone);
        networkToCompute.refreshNetwork();

        // float avgDist;

        float energyDev = networkToCompute.ComputeNormalizedDeviationEnergy(out avgDist);
        swarmConnectionScore = energyDev;
    }

    void swarmAskingShrink()
    {
        List<float> scores = new List<float>();
        foreach (DroneFake drone in network.drones)
        {
            if (!network.IsInMainNetwork(drone)) continue;

            List<DroneFake> neighbors = network.GetNeighbors(drone);
            if (neighbors.Count == 0) continue;

            neighbors = neighbors.OrderBy(d => Vector3.Distance(d.position, drone.position)).ToList();
            int number = (int)(network.drones.Count * 0.3f);
            if (number > neighbors.Count) number = neighbors.Count;
            if (number == 0) continue;

            List<DroneFake> mostVulnerable = neighbors.GetRange(0, number);
            float ratioAverageDistance = mostVulnerable.Average(d => Vector3.Distance(d.position, drone.position)) / desiredSeparation;
            scores.Add(ratioAverageDistance);
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
            List<DroneFake> currentDrones;
            Vector3 currentAlignmentVector;

            lock (networkLock)
            {
                currentDrones = new List<DroneFake>(currentScores.dronesSnapshot);
                currentAlignmentVector = currentScores.alignmentVector;
            }

            NetworkCreator networkToCompute = new NetworkCreator(currentDrones);
            networkToCompute.refreshNetwork();

            float velMissmatch = networkToCompute.ComputeNormalizedVelocityMismatch();
            float avgDist;
            float energyDev = networkToCompute.ComputeNormalizedDeviationEnergy(out avgDist);
            float relativeConnectivity = networkToCompute.ComputeRelativeConnectivity();
            float cohesionRadius = networkToCompute.ComputeCohesionRadius();

            Debug.Log($"[Scores] VelMismatch:{velMissmatch:F3}  EnergyDev:{energyDev:F3}  RelConn:{relativeConnectivity:F3}  CohesionR:{cohesionRadius:F3}  N={networkToCompute.drones.Count}");

            Thread.Sleep((int)(printInterval * 1000));
        }
    }

    #endregion

    #region Gizmos

    #if UNITY_EDITOR
    void OnDrawGizmos()
    {
        // -------- Collect points: prefer main group at runtime; otherwise all drones in scene --------
        List<Vector3> pts = null;

        if (Application.isPlaying && network != null && network.largestComponent != null && network.largestComponent.Count > 0)
        {
            // Largest connected component (main group)
            pts = network.largestComponent.Select(df => df.position).ToList();
        }
        else
        {
            // Editor / fallback: use all DroneController transforms
            var allDrones = FindObjectsOfType<DroneController>();
            if (allDrones != null && allDrones.Length > 0)
                pts = allDrones.Select(d => d.transform.position).ToList();
        }

        if (pts == null || pts.Count == 0) return;

        // -------- Fit ellipsoid on chosen set of points --------
        var (centroid, axes) = ComputeSwarmEllipsoidAxes(pts, axisFloor, axisScale);

        // -------- Draw centroid marker --------
        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(centroid, 0.2f); //0.2f
        Gizmos.DrawLine(centroid, centroid + Vector3.up * 0.5f);

        // -------- Draw ellipsoid wireframe --------
        Gizmos.color = new Color(0f, 0.5f, 1f, 0.8f);
        DrawWireEllipsoid(centroid, axes);

        // ============================================================================================
        // Runtime force visualization (uses swarmObstacleForces populated by UpdateSwarmForcesEllipsoid)
        // ============================================================================================
        bool drewAnyRuntimeForces = false;

        if (Application.isPlaying && showSwarmObstacleForces && swarmObstacleForces != null && swarmObstacleForces.Count > 0)
        {
            foreach (var f in swarmObstacleForces)
            {
                float intensity = f.magnitude;
                if (intensity < gizmoVisualMin) continue;

                Vector3 dir = (intensity > 1e-6f) ? (f / intensity) : Vector3.forward;

                UnityEditor.Handles.color = Color.red;
                DrawArrow(centroid, dir, intensity, gizmoArrowScale, gizmoArrowHead);
                drewAnyRuntimeForces = true;
            }
        }

        // ============================================================================================
        // Fallback previews from obstacles (when no runtime forces exist yet)
        // ============================================================================================
        if (gizmoForcePreviewFallback && !drewAnyRuntimeForces)
        {
            // Snapshot obstacles safely
            List<Obstacle> obsSnapshot = null;
            var src = ClosestPointCalculator.obstaclesInRange;
            if (src != null && src.Count > 0)
            {
                lock (src) { obsSnapshot = new List<Obstacle>(src); }
            }

            if (obsSnapshot != null && obsSnapshot.Count > 0)
            {
                foreach (var o in obsSnapshot)
                {
                    Vector3 p = o.ClosestPoint(centroid);

                    // Distance for activation gating
                    float d = activationHorizontalOnly
                        ? Vector2.Distance(new Vector2(p.x, p.z), new Vector2(centroid.x, centroid.z))
                        : Vector3.Distance(p, centroid);

                    float gain = ActivationGain(d, swarmActivationRadius);
                    if (gain <= 0f) continue;

                    // Intensity preview from ellipsoid geometry * distance gate
                    float baseIntensity = ComputeIntensityFromEllipsoid(centroid, axes, p);
                    float intensity = baseIntensity * gain;
                    if (intensity < gizmoVisualMin) continue;

                    // Horizontal inward direction (project to XZ for clarity, like runtime)
                    Vector3 dir3 = centroid - p;
                    Vector3 dir = Vector3.ProjectOnPlane(dir3, Vector3.up).normalized;
                    if (dir.sqrMagnitude < 1e-6f) continue;

                    UnityEditor.Handles.color = Color.red; // preview color
                    DrawArrow(centroid, dir, intensity, gizmoArrowScale, gizmoArrowHead);
                }
            }
        }

        // -------- Optional: visualize the points used for the fit (comment in if helpful) --------
        // Gizmos.color = new Color(0.2f, 1f, 0.6f, 0.9f);
        // for (int i = 0; i < pts.Count; i++) Gizmos.DrawSphere(pts[i], 0.06f);
    }
    #endif



    private void DrawWireEllipsoid(Vector3 c, Vector3 axes)
    {
        int seg = 48;
        DrawWireCircle(c, Vector3.forward, axes.x, axes.y, seg); // XY
        DrawWireCircle(c, Vector3.up,      axes.x, axes.z, seg); // XZ
        DrawWireCircle(c, Vector3.right,   axes.y, axes.z, seg); // YZ
    }

    private void DrawWireCircle(Vector3 c, Vector3 normal, float rx, float ry, int segments)
    {
        if (rx <= 0f || ry <= 0f) return;
        Quaternion rot = Quaternion.FromToRotation(Vector3.forward, normal.normalized);
        Vector3 prev = Vector3.zero;
        for (int i = 0; i <= segments; i++)
        {
            float t = (i / (float)segments) * Mathf.PI * 2f;
            Vector3 p = new Vector3(Mathf.Cos(t) * rx, Mathf.Sin(t) * ry, 0f);
            p = rot * p + c;
            if (i > 0) Gizmos.DrawLine(prev, p);
            prev = p;
        }
    }

    #endregion
}
