using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using System.Threading;
using Unity.VisualScripting;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Interactions;
using System.Linq;          // ← ADD THIS

public class HapticsTest : MonoBehaviour
{
    #region ObstalceInRange
    public int dutyIntensity = 4;
    public int frequencyInit = 1;
    public float distanceDetection = 3;

    public static float _distanceDetection
    {
        get
        {
            return GameObject.FindGameObjectWithTag("GameManager").GetComponent<HapticsTest>().distanceDetection;
        }
    }
    List<Actuators> actuatorsRange = new List<Actuators>();

    #endregion
    public bool Haptics_Obstacle
    {
        get
        {
            return LevelConfiguration._Haptics_Obstacle;
        }
    }
    public bool Haptics_Network
    {
        get
        {
            return LevelConfiguration._Haptics_Network;
        }
    }
    public bool Haptics_Forces
    {
        get
        {
            return LevelConfiguration._Haptics_Forces;
        }
    }
    public bool Haptics_Crash
    {
        get
        {
            return LevelConfiguration._Haptics_Crash;
        }
    }
    public bool Haptics_Controller
    {
        get
        {
            return LevelConfiguration._Haptics_Controller;
        }
    }

    // 3) accessor
    // public int[] GetDutySnapshot() => duty;
    public int[] GetDutySnapshot()
    {
        return dutyByTile;
    }

    // public static readonly int[] ObstacleAddrs =
    // { 60, 61, 62, 63, 64, 65, 66, 67 };
    // public static readonly int[] ObstacleAddrs =
    // { 16, 17, 18, 19, 20, 21, 22, 23 };
    public static readonly int[] ObstacleAddrs =
    { 0, 1, 2, 3, 4, 5, 6, 7 };

    // public static int[] GetObstacleDutySnapshot()   // 8 长度
    // {
    //     return duty;
    // }

    public static int[] GetObstacleDutySnapshot() { return dutyObstacle; }


    /// <summary>
    /// Returns the geometric centre of the swarm, i.e. the midpoint of the
    /// axis-aligned bounding box that encloses every drone.
    /// “Most-left” and “most-right” drones carry the same weight.
    /// </summary>
    public static Vector3 GetSwarmCentroid(IReadOnlyList<Transform> drones)
    {
        // Only use drones from the main connected group
        var connectedDrones = drones.Where(d =>
            d.GetComponent<DroneController>()?.droneFake != null &&
            swarmModel.network.IsInMainNetwork(d.GetComponent<DroneController>().droneFake)
        ).ToList();

        if (connectedDrones == null || connectedDrones.Count == 0)
            return Vector3.zero;

        // Initialize mins & maxes with the first connected drone's position
        Vector3 p0 = connectedDrones[0].position;
        float minX = p0.x, maxX = p0.x;
        float minY = p0.y, maxY = p0.y;
        float minZ = p0.z, maxZ = p0.z;

        // Expand bounds only using connected drones
        for (int i = 1; i < connectedDrones.Count; i++)
        {
            Vector3 p = connectedDrones[i].position;
            if (p.x < minX) minX = p.x; else if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y; else if (p.y > maxY) maxY = p.y;
            if (p.z < minZ) minZ = p.z; else if (p.z > maxZ) maxZ = p.z;
        }

        return new Vector3(
            (minX + maxX) * 0.5f,
            (minY + maxY) * 0.5f,
            (minZ + maxZ) * 0.5f);
    }

    public static Vector2 GetSwarmCentroid2D(IReadOnlyList<Transform> drones)
    {
        // Only use drones from the main connected group
        var connectedDrones = drones.Where(d =>
            d.GetComponent<DroneController>()?.droneFake != null &&
            swarmModel.network.IsInMainNetwork(d.GetComponent<DroneController>().droneFake)
        ).ToList();

        if (connectedDrones == null || connectedDrones.Count == 0)
            return Vector2.zero;

        float minX = connectedDrones[0].position.x, maxX = minX;
        float minZ = connectedDrones[0].position.z, maxZ = minZ;

        for (int i = 1; i < connectedDrones.Count; i++)
        {
            Vector3 p = connectedDrones[i].position;
            if (p.x < minX) minX = p.x; else if (p.x > maxX) maxX = p.x;
            if (p.z < minZ) minZ = p.z; else if (p.z > maxZ) maxZ = p.z;
        }

        return new Vector2((minX + maxX) * 0.5f, (minZ + maxZ) * 0.5f);
    }

    // Call this instead of GetSwarmCentroid(...)
    static bool BuildSwarmFrameAndLocalBounds(
        IReadOnlyList<Transform> drones,
        Transform embodiedDrone,
        out Vector3 minLocalCentered,
        out Vector3 maxLocalCentered,
        out Vector3 centroidWorld)
    {
        minLocalCentered = maxLocalCentered = centroidWorld = Vector3.zero;

        if (embodiedDrone == null || drones == null || drones.Count == 0)
            return false;

        // 1) pick the frame rotation from embodied
        Quaternion swarmRot = Quaternion.LookRotation(embodiedDrone.forward, embodiedDrone.up);
        Quaternion invRot   = Quaternion.Inverse(swarmRot);

        // 2) filter to main connected group (same as your original)
        var connected = new List<Transform>();
        foreach (var t in drones)
        {
            var dc = t.GetComponent<DroneController>();
            var df = dc ? dc.droneFake : null;
            if (df != null && swarmModel.network.IsInMainNetwork(df))
                connected.Add(t);
        }
        if (connected.Count == 0) return false;

        // 3) choose a temporary origin (any consistent point is fine)
        Vector3 refOrigin = connected[0].position;

        // 4) compute oriented AABB in swarm frame (relative to refOrigin)
        Vector3 p0Local = invRot * (connected[0].position - refOrigin);
        Vector3 minL = p0Local, maxL = p0Local;

        for (int i = 1; i < connected.Count; i++)
        {
            Vector3 lp = invRot * (connected[i].position - refOrigin);
            if (lp.x < minL.x) minL.x = lp.x; else if (lp.x > maxL.x) maxL.x = lp.x;
            if (lp.y < minL.y) minL.y = lp.y; else if (lp.y > maxL.y) maxL.y = lp.y;
            if (lp.z < minL.z) minL.z = lp.z; else if (lp.z > maxL.z) maxL.z = lp.z;
        }

        // 5) center of the oriented AABB in local (relative to refOrigin)
        Vector3 centerLocal = 0.5f * (minL + maxL);

        // 6) final world centroid to place the swarm frame
        centroidWorld = refOrigin + (swarmRot * centerLocal);

        // 7) report min/max relative to the FINAL frame origin (centered)
        minLocalCentered = minL - centerLocal;
        maxLocalCentered = maxL - centerLocal;

        return true;
    }

    // Mean (center-of-mass) centroid, optionally restricting to the main connected group.
    public static Vector3 GetSwarmCentroidMean(IReadOnlyList<Transform> drones, bool mainGroupOnly = true)
    {
        if (drones == null || drones.Count == 0) return Vector3.zero;

        int n = 0;
        Vector3 sum = Vector3.zero;

        for (int i = 0; i < drones.Count; i++)
        {
            var t = drones[i];
            if (t == null) continue;

            var dc = t.GetComponent<DroneController>();
            var df = dc != null ? dc.droneFake : null;
            if (df == null) continue;

            if (mainGroupOnly)
            {
                // If network not ready yet, skip filtering; otherwise require membership in main group
                if (swarmModel.network != null && !swarmModel.network.IsInMainNetwork(df))
                    continue;
            }

            sum += t.position;
            n++;
        }

        return (n > 0) ? (sum / n) : Vector3.zero;
    }



    // -- Highlight-helper state ---------------------------------------------------
    private Transform _highlightedDrone = null;   // the drone we tinted last frame
    private static readonly Color _highlightColor = Color.blue;

    private void HighlightClosestDrone()
    {
        // IReadOnlyList<Transform> drones = swarmModel.dronesTransforms;   // adjust if your list has a different name
        var drones = FindObjectsOfType<DroneController>()
             .Select(d => d.transform).ToList();
        if (drones == null || drones.Count == 0) { return; }

        // 1) where is the swarm centre?
        // Vector3 centre = GetSwarmCentroid(drones);
        Vector2 centre2D = GetSwarmCentroid2D(drones);

        // 2) pick the nearest drone
        Transform closest = null;
        float bestSq = float.PositiveInfinity;
        foreach (Transform t in drones)
        {
            // float sq = (t.position - centre).sqrMagnitude;   // cheaper than magnitude
            float sq = (new Vector2(t.position.x, t.position.z) - centre2D).sqrMagnitude; // 2D distance
            if (sq < bestSq) { bestSq = sq; closest = t; }
        }
        if (closest == null) { return; }

        // 3) if it changed, restore the old one and tint the new one
        if (_highlightedDrone != null && _highlightedDrone != closest)
        {
            SetDroneTint(_highlightedDrone, Color.white);    // or whatever the default is
        }
        _highlightedDrone = closest;
        SetDroneTint(_highlightedDrone, _highlightColor);
    }

    /*---------------------------------------------------------------*/
    /* visualization of swarm centroid                          */
    /*---------------------------------------------------------------*/
    void OnDrawGizmos()
    {
        var drones = FindObjectsOfType<DroneController>()
             .Select(d => d.transform).ToList();
        if (drones == null || drones.Count == 0) return;

        // Vector3 c = drones.Aggregate(Vector3.zero,
        //             (sum, t) => sum + t.position) / drones.Count;
        Vector3 c = GetSwarmCentroid(drones);  // or use the centroid function above

        // Gizmos.color = Color.blue;
        // Gizmos.DrawSphere(c, 0.2f);        // 5 cm sphere
        // Gizmos.DrawLine(c, c + Vector3.up); // little “stem” so it’s easy to spot
    }

    void OnDrawGizmosSelected()
    {
        // var drones = FindObjectsOfType<DroneController>()
        //      .Select(d => d.transform).ToList();
        // if (drones == null || drones.Count == 0) { return; }

        // _swarmFrame.position = GetSwarmCentroid(drones);
        // _swarmFrame.rotation = Quaternion.LookRotation(
        // embodiedDrone.forward,
        // embodiedDrone.up);

        if (_swarmFrame == null) return;

        /*---------------------------------------------------------*
        * 1) work in swarm-frame space
        *---------------------------------------------------------*/
        Gizmos.matrix = _swarmFrame.localToWorldMatrix;

        /*  rectangle (you had this)  */
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(Vector3.zero,
                            // new Vector3(halfW * 2f, halfH * 2f, 0.05f));
                            new Vector3(halfW * 2f, 0.05f, halfH * 2f));

        /*---------------------------------------------------------*
        * 2) forward arrow (red)
        *---------------------------------------------------------*/
        Gizmos.color = Color.red;

        float arrowLen = Mathf.Max(halfW, halfH) * 1.1f;   // a bit beyond box
        Vector3 tail   = Vector3.zero;                     // start at centroid
        Vector3 tip    = Vector3.forward * arrowLen;       // +Z in swarm frame

        // shaft
        Gizmos.DrawLine(tail, tip);

        // simple 2-line arrowhead (~20° cone)
        Vector3 headL = tip + (Quaternion.Euler(0, 160, 0) * (tip - tail).normalized) * (arrowLen * 0.15f);
        Vector3 headR = tip + (Quaternion.Euler(0, -160, 0) * (tip - tail).normalized) * (arrowLen * 0.15f);
        Gizmos.DrawLine(tip, headL);
        Gizmos.DrawLine(tip, headR);
    }
    
    // ------------------------------------------------------------
    //  Local "Swarm Frame" (created once, reused every frame)
    // ------------------------------------------------------------
    private Transform _swarmFrame;               // invisible helper transform
    public Transform embodiedDrone;             // assign in the Inspector

    private float halfW = 1f;
    private float halfH = 1f;

    private float actuator_W = 3f; //4f;
    private float actuator_H = 3f; //4f; //2f; // 5f;
    private const float initial_actuator_W= 3f; //4f;
    private const float initial_actuator_H = 3f; //4f; //2f; // 5f;
    private const float center_W = initial_actuator_W / 2f;   // 1.5 m wide, 2 m high
    private const float center_H = initial_actuator_H / 2f;   // 2 m high, 1.5 m wide

    // private static readonly int[,] matrix = {
    // {119,1,0,119},
    // { 2, 3, 30, 31},
    // {35, 34, 33, 32},
    // {36, 37, 38, 39},
    // {119,41,40,119}
    // };

    // private static readonly int[,] matrix = {
    // {3,2,1,0},
    // { 4, 5, 6, 7},
    // {11, 10, 9, 8},
    // {12, 13, 14, 15}
    // };

    private static readonly int[,] matrix = {
    {3,2,1,0},
    { 4, 5, 6, 7},
    {33, 32, 31, 30},
    {34, 35, 36, 37}
    };

    private static readonly int[] duty = new int[120];   // one per vibrator (0-14)
    private static readonly int[] dutyByTile = new int[matrix.Length];   // 20-cell visual panel (0-14)
    static int[] dutyObstacle = new int[128]; // size per your addr range
    int[] freq   = new int[120];   // keep simple: all 1

    // —— 行为参数 —— 可按需要微调
    const float EPS          = 0.9f;   // 变化阈值：≈“1架无人机变动”
    const float STABLE_FOR   = 2f;   // 连续稳定多久后开始衰减（秒）
    const float DECAY_PER_S  = 8f;     // 衰减速度（每秒减少的 duty “格数”）
    const int   DUTY_GAIN    = 3;      // 密度→强度, 3 for num=9, 
    const int   DUTY_MAX     = 14;     // 上限

    // —— 状态缓存 —— 按你的地址空间大小分配
    float[] lastRaw      = new float[256]; // 上一帧的密度
    float[] stableTimer  = new float[256]; // “持续稳定”的计时
    int[]   smoothedDuty = new int[256];   // 平滑后的最终强度（写回给硬件）
    float[] rawByAddr    = new float[256]; // 本帧密度（临时）

    // —— 可调参数 ——
    const float GAIN_PER_DRONE = 12f / 20 * 9; //6f; //12f;   // 一架无人机贡献的总强度 12f for num=9
    const float TAU_SMOOTH     = 0.20f;// 时间平滑常数(秒)，越大越稳

    // —— 状态缓存 ——
    float[] targetDuty = new float[256];   // 本帧按权重累加的目标强度（float）
    float[] smoothDuty = new float[256];   // 时间平滑后的强度（float，最终会转 int）

        // 列/行“数量”（必须是4，而不是3f）
    private const int   COLS = 4, ROWS = 4;
    private const float COLS_MINUS1 = COLS - 1f;  // OK
    private const float ROWS_MINUS1 = ROWS - 1f;  // OK

    // --- Size-change gate config ---
    [SerializeField] float refWorldHalfW = 4.5f;  // reference half-width (meters)
    [SerializeField] float SIZE_EPS01 = 0.001f;    // “small change” threshold in normalized units
    [SerializeField] float SIZE_STABLE_FOR = 1.00f; // must stay small for this long (s)

    // --- Size-change gate state ---
    float _lastHalfW01 = -1f;
    float _sizeStableTimer = 0f;

    private const float max_half_width= 4.1f; //4f;

    // —— 断连提示（中间两列动态条）——
    [SerializeField] float disconnectTau = 0.25f;   // 分数平滑时间常数(s)
    [SerializeField] float flowSpeedHz   = 6f;      // 高亮条向下“流”的速度(Hz)
    [SerializeField] int   baseDuty      = 0;       // 已填充区域的基础强度
    [SerializeField] int   peakDuty      = 8;      // 流动高亮的峰值
    [SerializeField] bool  overlayMode   = true;    // true=与其它图层叠加(取max)，false=覆盖

    float _discScoreSmooth = 0f;                    // 平滑后的score

    // —— 放在类作用域（和其他字段一起）——
    [SerializeField] float DISC_ON  = 0.20f;  // 触发阈值
    [SerializeField] float DISC_OFF = 0.15f;  // 释抑阈值（迟滞）
    private bool disconnActive = false;      // 断连模式的状态机

    // --- per-frame cache for assignments ---
    static int _assignmentsFrame = -1;
    static Dictionary<int, float> _assignedMagnitude = new Dictionary<int, float>(); // adresse -> magnitude
    static Dictionary<int, int>   _assignedDuty      = new Dictionary<int, int>();   // adresse -> duty (viz)

    // === Size rendering log ===
    private List<float> logTime = new();
    private List<float> logHalfW01 = new();
    private List<int[]> logSizeDuties = new();
    private float logStartTime;
    private string logFilePath;
    private bool enableLogging = true;  // can toggle if needed

    [Header("Debug / Disconnected")]
    [SerializeField] bool drawDisconnectedInSwarmFrame = true;
    [SerializeField] Color disconnectedColor = Color.red;

    private static void GetDynamicExtents(IReadOnlyList<Transform> drones,
                                    Transform swarmFrame,
                                    out float halfWidth,
                                    out float halfHeight)
    {
        // Get only drones from the main connected group
        var connectedDrones = drones.Where(d =>
            d.GetComponent<DroneController>()?.droneFake != null &&
            swarmModel.network.IsInMainNetwork(d.GetComponent<DroneController>().droneFake)
        ).ToList();

        float maxAbsX = 0f;
        float maxAbsY = 0f;

        // Only process connected drones
        foreach (var t in connectedDrones)
        {
            Vector3 p = swarmFrame.InverseTransformPoint(t.position); // local
            maxAbsX = Mathf.Max(maxAbsX, Mathf.Abs(p.x));
            maxAbsY = Mathf.Max(maxAbsY, Mathf.Abs(p.z));
        }

        // Avoid divide-by-zero when the swarm collapses to a point
        halfWidth = Mathf.Max(maxAbsX, 0.01f);   // at least 1 cm
        halfHeight = Mathf.Max(maxAbsY, 0.01f);
    }

    // returns a continuous column coordinate in [0, COLS-1]
    float ColUFromX(float xLocal)
    {
        // your swarm width is normalized by 4.5f above, and you center it with +center_W
        // let's reuse that logic but keep it continuous
        float u = Mathf.Clamp(-xLocal / max_half_width * 1.5f + center_W, 0f, COLS_MINUS1);
        return u;
    }

    int ColFromX(float x, float halfW, float actuator_W)    // halfW ≥ 0.01
    {
        // float t = (x + halfW) / (2f * halfW);      // → [0..1]
        // return Mathf.Clamp(Mathf.RoundToInt(t * actuator_W + center_W - actuator_W / 2f), 0, Mathf.RoundToInt(initial_actuator_W));
        float t = x / max_half_width * 1.5f;      // → [0..1]
        return Mathf.Clamp(Mathf.RoundToInt(-t + center_W), 0, Mathf.RoundToInt(initial_actuator_W));
        return Mathf.Clamp(Mathf.RoundToInt(t + center_W), 0, Mathf.RoundToInt(initial_actuator_W));
        // return Mathf.Clamp(Mathf.RoundToInt(t *  3f), 0, 3);
    }

    int RowFromY(float y, float halfH, float actuator_H)    // halfH ≥ 0.01
    {
        // float t = (-y + halfH) / (2f * halfH);      // → [0..1]
        // return Mathf.Clamp(Mathf.RoundToInt(t * actuator_H + center_H - actuator_H / 2f), 0, Mathf.RoundToInt(initial_actuator_H));
        float t = -y / max_half_width * 1.5f;      // → [0..1]
        return Mathf.Clamp(Mathf.RoundToInt(- t + center_H), 0, Mathf.RoundToInt(initial_actuator_H));
        // return Mathf.Clamp(Mathf.RoundToInt(t * 4f), 0, 4);
    }

    private int _prevAddr = -1;          // -1 = nothing buzzing yet
    
    // put next to your other member fields
    private readonly int[] _prevDuty = new int[30 + matrix.Length];   // 40 tactors, init 0
    private readonly int[] _prevFreq = new int[30 + matrix.Length];   // same size, init 0

    /// <summary>
    /// Re-positions the `_swarmFrame` at the swarm centroid, aligns it with
    /// the embodied drone’s forward-up axes, and prints every drone’s
    /// position in that local frame.
    /// Call this once per frame from LateUpdate().
    /// </summary>
    private void UpdateSwarmFrameAndLog()
    {
        // --- 0) collect all drones ------------------------------------------
        var drones = FindObjectsOfType<DroneController>()
                    .Select(d => d.transform).ToList();
        if (drones.Count == 0) return;          // nothing to do

        // 0 bis) make sure we track the *current* embodied drone every frame
        Transform current = CameraMovement.embodiedDrone ?
                            CameraMovement.embodiedDrone.transform : null;

        if (current == null)
        {
            Debug.LogWarning("No embodied drone in scene – skipping swarm-frame update.");
            return;
        }

        embodiedDrone = current;          // <-- always keep the latest reference

        // --- 1) create helper transform once --------------------------------
        if (_swarmFrame == null)
        {
            _swarmFrame = new GameObject("SwarmFrame").transform;
            _swarmFrame.hideFlags = HideFlags.HideInHierarchy;
        }

        // --- 2) place & orient frame ----------------------------------------
        // Vector3 centroid = GetSwarmCentroid(drones); // or use the centroid function above
        // _swarmFrame.position = centroid;           // place at the centroid
        // _swarmFrame.rotation = Quaternion.LookRotation(
        //                             embodiedDrone.forward,
        // embodiedDrone.up);
        // Debug.Log($"swarmFrame.rotation = {_swarmFrame.rotation.eulerAngles:F2} " +
        //           $"(centroid at {centroid:F2})");

        // Vector3 minL, maxL, centroidW;
        // if (BuildSwarmFrameAndLocalBounds(drones, embodiedDrone, out minL, out maxL, out centroidW))
        // {
        //     // Place & orient the swarm frame correctly
        //     _swarmFrame.SetPositionAndRotation(
        //         centroidW,
        //         Quaternion.LookRotation(embodiedDrone.forward, embodiedDrone.up)
        //     );

        //     // minL / maxL are now in the swarm coordinate (centered at the frame)
        //     // e.g. minL.x is your local minX, etc.
        // }
        
        // 1) Compute true centroid
        Vector3 centroid = GetSwarmCentroidMean(drones, mainGroupOnly: true);

        // 2) Place & orient the swarm frame
        _swarmFrame.SetPositionAndRotation(
            centroid,
            Quaternion.LookRotation(embodiedDrone.forward, embodiedDrone.up)
        );

        // --- (Optional) visualize disconnected drones in the swarm frame ---
        if (drawDisconnectedInSwarmFrame)
        {
            var list = GetDisconnectedDronesLocal();
            foreach (var (go, local) in list)
            {
                Vector3 world = _swarmFrame.TransformPoint(local);
                Debug.DrawLine(_swarmFrame.position, world, disconnectedColor, 0f, false);
                // If you prefer a dot:
                // Debug.DrawRay(world, Vector3.up * 0.15f, disconnectedColor, 0f, false);
            }
        }

        // ② measure current half-sizes
        GetDynamicExtents(drones, _swarmFrame, out halfW, out halfH);

        // --- measure normalized half-width and gate stability ---
        float dt = Time.deltaTime; // we’ll reuse dt later as well
        float halfW01 = Mathf.Clamp01(halfW / refWorldHalfW);     // normalize width to [0,1]
        Debug.Log($"halfW01: {halfW01:F3}");
        Debug.Log($"halfW: {halfW:F3}");
        float sizeDiff01 = (_lastHalfW01 < 0f) ? 1f : Mathf.Abs(halfW01 - _lastHalfW01);

        // hysteresis on size: must stay “small change” for a while
        if (sizeDiff01 < SIZE_EPS01) _sizeStableTimer += dt;
        else _sizeStableTimer = 0f;

        bool muteTargetRow = (_sizeStableTimer >= SIZE_STABLE_FOR);

        // Debug.Log($"_lastHalfW01: {_lastHalfW01:F3}, Current halfW01: {halfW01:F3}, Difference: {sizeDiff01:F3}");

        _lastHalfW01 = halfW01;

        // Debug.Log($"halfW = {halfW:F2} m, halfH = {halfH:F2} m " +
        //           $"(norm {halfW01:F3}, Δ {sizeDiff01:F3}, " +
        //           $"{_sizeStableTimer:F2}s stable, " +
        //           $"{(muteTargetRow ? "MUTING" : "active")})");

        if (initial_actuator_W / initial_actuator_H > halfW / halfH)
        {
            actuator_W = initial_actuator_H * halfW / halfH; // make it 4:3 aspect ratio
            // Debug.Log($"actuator_W = {actuator_W:F2} (halfW {halfW:F2}, halfH {halfH:F2})");
        }
        else
        {
            actuator_H = initial_actuator_W * halfH / halfW; // make it 4:3 aspect ratio
            // Debug.Log($"actuator_H = {actuator_H:F2} (halfW {halfW:F2}, halfH {halfH:F2})");
        }

        // ③ zero-out per-vibrator accumulators
        // Array.Clear(duty, 0, duty.Length);   // duty[20]; declared elsewhere
        // Array.Clear(dutyByTile, 0, dutyByTile.Length);   // dutyByTile[20]; declared elsewhere

        /*-------------------------------------------------------------*
    * 1) ensure the embodied drone is in the list we iterate
    *    (FindObjectsOfType may or may not include it, so we add it
    *    explicitly if needed)
    *-------------------------------------------------------------*/
        if (!drones.Contains(embodiedDrone.transform))
            drones.Add(embodiedDrone.transform);

        var connectedDrones = drones.Where(d => d.GetComponent<DroneController>()?.droneFake != null && swarmModel.network.IsInMainNetwork(d.GetComponent<DroneController>().droneFake)).ToList();

        // 1) 清空本帧目标强度
        System.Array.Clear(targetDuty, 0, targetDuty.Length);
        System.Array.Clear(dutyByTile, 0, dutyByTile.Length);

        // 2) 每架无人机 -> 对周围4格做双线性分配
        foreach (Transform d in connectedDrones)
        {
            Vector3 local = _swarmFrame.InverseTransformPoint(d.position);

            // ---- 连续坐标（0..W-1 / 0..H-1），保证在边界内 ----
            // float u = Mathf.Clamp01((local.x*2f) / (max_half_width)) * COLS_MINUS1;
            // float v = Mathf.Clamp01((local.z + halfH) / (max_half_width)) * ROWS_MINUS1;

            // float t = local.x / max_half_width * 2f;      // → [0..1]
            float u = Mathf.Clamp(local.x / max_half_width * 1.5f + center_W, 0, Mathf.RoundToInt(initial_actuator_W));
            float v = Mathf.Clamp(-local.z / max_half_width * 1.5f + center_H, 0, Mathf.RoundToInt(initial_actuator_H));

            // find the nearest grid cell locations
            int c0 = Mathf.FloorToInt(u);
            int r0 = Mathf.FloorToInt(v);
            int c1 = Mathf.Min(c0 + 1, COLS - 1);
            int r1 = Mathf.Min(r0 + 1, ROWS - 1);

            // weights
            float wc1 = u - c0, wc0 = 1f - wc1;
            float wr1 = v - r0, wr0 = 1f - wr1;

            // weights for neighboring cells
            float w00 = wc0 * wr0;
            float w10 = wc1 * wr0;
            float w01 = wc0 * wr1;
            float w11 = wc1 * wr1;

            // 每架无人机的总贡献为 GAIN_PER_DRONE，按权重分摊
            void Add(int rr, int cc, float w)
            {
                int addr = matrix[rr, cc];
                // targetDuty[addr] += GAIN_PER_DRONE * (1 - halfW01) * w;
                targetDuty[addr] += GAIN_PER_DRONE * Mathf.Clamp(1 - halfW01, 0.35f, 0.75f) * w; 
            }

            Add(r0, c0, w00);
            Add(r0, c1, w10);
            Add(r1, c0, w01);
            Add(r1, c1, w11);
        }


        // float dt = Time.deltaTime;
        float alpha = 1f - Mathf.Exp(-dt / TAU_SMOOTH);

        // === 先计算列合并需要的中间量（保留你现在的 colSum 计算） ===
        // int[] colSum = new int[COLS];
        float[] colSum = new float[COLS];
        for (int row = 0; row < ROWS; row++)
        {
            for (int col = 0; col < COLS; col++)
            {
                int addr = matrix[row, col];

                // cell 平滑，累加到列
                smoothDuty[addr] = Mathf.Lerp(smoothDuty[addr], targetDuty[addr], alpha);
                // int cellDuty = Mathf.Min(DUTY_MAX, Mathf.RoundToInt(smoothDuty[addr]));
                float cellDuty = smoothDuty[addr];  //Mathf.Min(DUTY_MAX, smoothDuty[addr]);
                colSum[col] += cellDuty;

                // 清空所有单元；稍后只由“唯一被选中的模式”写入
                duty[addr] = 0;
                dutyByTile[row * COLS + col] = 0;
            }
        }

        // === 计算“模式选择”的判据（只在这里做一次） ===
        // 1) Disconnection 是否触发（给它迟滞，避免闪烁）
        // const float DISC_ON  = 0.20f;  // 0..1，超过即触发
        // const float DISC_OFF = 0.15f;  // 低于此阈值才关闭
        float score01 = Mathf.Clamp01(swarmModel.swarmConnectionScore);
        Debug.Log($"Connection score: {score01:F3}");
        Debug.Log($"                   Average distance: {swarmModel.avgDist* DroneFake.desiredSeparation:F3} m");
        Debug.Log($"                              Average distance / spreadness: {swarmModel.avgDist:F3} m");

        // 用你已有的平滑（可选）
        float discA = 1f - Mathf.Exp(-dt / disconnectTau);
        _discScoreSmooth = Mathf.Lerp(_discScoreSmooth, score01, discA);

        // 带迟滞的开关
        // static bool disconnActive; // 放到类字段更好（避免每帧重新置 false）
        if (!disconnActive && _discScoreSmooth >= DISC_ON && swarmModel.avgDist < 4.0f) disconnActive = true;
        if (disconnActive && (swarmModel.avgDist >= 4.0f || _discScoreSmooth <= DISC_OFF)) disconnActive = false;

        // 2) if size changed obviously
        bool sizeActive = !muteTargetRow;
        // 注意：这里不再“静音某行”，而是：只有在 sizeActive==true 时才渲染 size bar

        // render according to priority
        const int TARGET_ROW = 2;
        float Compress = 1f / ROWS; // 列合并用平均，避免饱和

        if (disconnActive)
        {
            // ① Disconnection（最高优先级）：只渲染中间两列的 motion
            // RenderDisconnectMotion(_discScoreSmooth, dt);  // 内部用 Max 叠加已被清零的缓冲即可
            RenderDisconnectedDirectionsToTiles(10);  // choose the intensity you like (e.g., 8–12)
            Debug.Log("[MODE] Disconnection");
        }
        else if (sizeActive)
        {
            // ② Size rendering: divide by 4 rows → write into TARGET_ROW
            for (int col = 0; col < COLS; col++)
            {
                int collapsed = Mathf.Min(DUTY_MAX, Mathf.RoundToInt(colSum[col] * Compress));
                Debug.Log($"[SizeBar] col={col} value={colSum[col] * Compress:F3}");
                int addr = matrix[TARGET_ROW, col];
                duty[addr] = collapsed;
                dutyByTile[TARGET_ROW * COLS + col] = collapsed;
            }
            // Debug.Log("[MODE] Size bar");
        }
        // else
        // {
        //     // ③ Embodied blink
        //     const float blinkRate = 3f; // Hz, blink frequency
        //     bool blinkOn = (Mathf.FloorToInt(Time.time * blinkRate) & 1) == 0;
        //     int dutyVal = blinkOn ? 7 : 0;

        //     Vector3 localE = _swarmFrame.InverseTransformPoint(embodiedDrone.position);
        //     int colE = ColFromX(localE.x, halfW, actuator_W);
        //     int rowE = 1; //RowFromY(localE.z, halfH, actuator_H); // 如果你想固定在哪一行，可直接 rowE = 1;

        //     // 写入（注意 tile 步长 = COLS）
        //     int addrE = matrix[rowE, colE];
        //     duty[addrE] = dutyVal;
        //     dutyByTile[rowE * COLS + colE] = dutyVal;

        //     // Debug.Log("[MODE] Embodied blink");
        // }
        else
        {
            // ③ Embodied blink (but blended to 2 nearest actuators in row=1)
            const float blinkRate = 3f; // Hz
            bool blinkOn = (Mathf.FloorToInt(Time.time * blinkRate) & 1) == 0;
            int baseDuty = blinkOn ? 10 : 0;

            Vector3 localE = _swarmFrame.InverseTransformPoint(embodiedDrone.position);

            // 1) continuous column
            float u = ColUFromX(localE.x);   // e.g. 1.3 means 30% between col 1 and 2

            // 2) nearest two columns
            int c0 = Mathf.FloorToInt(u);
            int c1 = Mathf.Min(c0 + 1, COLS - 1);
            float t = u - c0;          // 0..1, how far to the right

            // 3) fixed row = 1
            int row = 1;

            // 4) weights (left gets 1-t, right gets t)
            float w0 = 1f - t;
            float w1 = t;

            // 5) write to the two tiles
            int addr0 = matrix[row, c0];
            int addr1 = matrix[row, c1];

            int duty0 = Mathf.RoundToInt(baseDuty * w0);
            int duty1 = Mathf.RoundToInt(baseDuty * w1);

            // since the buffer might already have other modes, take max
            duty[addr0] = Mathf.Max(duty[addr0], duty0);
            duty[addr1] = Mathf.Max(duty[addr1], duty1);

            dutyByTile[row * COLS + c0] = Mathf.Max(dutyByTile[row * COLS + c0], duty0);
            dutyByTile[row * COLS + c1] = Mathf.Max(dutyByTile[row * COLS + c1], duty1);
        }


        // ④ find which addresses changed since last frame
        const int BASE_FREQ = 1;                 // you keep freq fixed for now
        List<int> dirty = new();                 // addresses that changed

        for (int addr = 0; addr < (30 + matrix.Length); addr++)    // full belt range
        {
            int newDuty = duty[addr];            // duty[] you filled above
            int newFreq = BASE_FREQ;             // or freq[addr] if you vary it

            if (newDuty != _prevDuty[addr] || newFreq != _prevFreq[addr])
            {
                dirty.Add(addr);                 // remember to send it
                _prevDuty[addr] = newDuty;       // cache for next frame
                _prevFreq[addr] = newFreq;
            }
        }

        // actually send
        foreach (int addr in dirty)
        {
            VibraForge.SendCommand(addr,
                _prevDuty[addr] == 0 ? 0 : 1,    // enable flag
                _prevDuty[addr],                 // duty 0-14
                _prevFreq[addr]);                // freq (fixed = 1 here)
        }
        
        // --- Logging size rendering values ---
        if (enableLogging)
        {
            float currentTime = Time.time - logStartTime;
            int[] rowDuties = new int[COLS];
            for (int col = 0; col < COLS; col++)
            {
                int addr = matrix[TARGET_ROW, col];
                rowDuties[col] = duty[addr];
            }
            logTime.Add(currentTime);
            logHalfW01.Add(_lastHalfW01);  // normalized width
            logSizeDuties.Add(rowDuties);
        }

    }

    void RenderDisconnectMotion(float score01, float dt)
    {
        // 1) score 平滑，避免跳动
        float a = 1f - Mathf.Exp(-dt / disconnectTau);
        _discScoreSmooth = Mathf.Lerp(_discScoreSmooth, Mathf.Clamp01(score01), a);

        // 2) score→高度(行数)，从顶行(row=0)开始向下填充
        int rows = ROWS, cols = COLS;
        int filledRows = Mathf.Clamp(Mathf.CeilToInt(_discScoreSmooth * rows), 0, rows);

        // 3) 流动高亮：在 [0, filledRows-1] 区间内“一条亮线”向下滚动
        int highlightRow = -1;
        if (filledRows > 0)
        {
            // 以 flowSpeedHz 频率轮转 0..filledRows-1
            float phase = Time.time * flowSpeedHz;
            highlightRow = Mathf.FloorToInt(Mathf.Repeat(phase, filledRows));
        }

        // 4) 只画中间两列：col = 1, 2
        int cLeft  = 1;
        int cRight = 2;

        for (int r = 0; r < rows; r++)
        {
            // 该行是否在已填充高度内
            bool inFill = (r < filledRows);

            // 行的强度：基础 or 高亮 or 0
            int val = 0;
            if (inFill)
            {
                val = baseDuty;
                if (r == highlightRow) val = peakDuty;  // 高亮行更强
            }

            // 写左列
            int addrL = matrix[r, cLeft];
            int tileL = r * cols + cLeft;
            if (overlayMode) {
                duty[addrL]      = Mathf.Max(duty[addrL], val);
                dutyByTile[tileL]= Mathf.Max(dutyByTile[tileL], val);
            } else {
                duty[addrL]      = val;
                dutyByTile[tileL]= val;
            }

            // 写右列
            int addrR = matrix[r, cRight];
            int tileR = r * cols + cRight;
            if (overlayMode) {
                duty[addrR]      = Mathf.Max(duty[addrR], val);
                dutyByTile[tileR]= Mathf.Max(dutyByTile[tileR], val);
            } else {
                duty[addrR]      = val;
                dutyByTile[tileR]= val;
            }
        }
    }

    // World → Swarm-Local positions for all disconnected drones
    List<(GameObject go, Vector3 local)> GetDisconnectedDronesLocal()
    {
        var res = new List<(GameObject, Vector3)>();
        if (_swarmFrame == null || swarmModel.swarmHolder == null || swarmModel.network == null)
            return res;

        var W2L = _swarmFrame.worldToLocalMatrix;

        foreach (Transform t in swarmModel.swarmHolder.transform)
        {
            var dc = t.GetComponent<DroneController>();
            var df = (dc != null) ? dc.droneFake : null;
            if (df == null) continue;

            // “Disconnected” = not in the main (largest/leader) component
            if (!swarmModel.network.IsInMainNetwork(df))
            {
                // Use df.position (simulation state) or t.position (Transform) – both fine
                Vector3 local = W2L.MultiplyPoint(df.position);
                res.Add((t.gameObject, local));
            }
        }
        return res;
    }

    // Light one tile in the 4x4 matrix in the direction of each disconnected drone
    void RenderDisconnectedDirectionsToTiles(int dutyVal)
    {
        var disc = GetDisconnectedDronesLocal();
        if (disc == null || disc.Count == 0) return;

        foreach (var (_, local) in disc)
        {
            // Only direction from center (ignore distance)
            Vector2 dir = new Vector2(local.x, local.z);
            if (dir.sqrMagnitude < 1e-6f) continue;
            dir.Normalize();

            // Convert direction to continuous grid coords centered at panel center
            // Keep the sign conventions consistent with your Col/Row mapping
            float u = Mathf.Clamp( dir.x * center_W + center_W, 0f, COLS - 1); // // columns (X)
            float v = Mathf.Clamp(-dir.y * center_H + center_H, 0f, ROWS - 1); // rows (Z)


            int col = Mathf.RoundToInt(u);
            int row = Mathf.RoundToInt(v);

            int addr = matrix[row, col];

            // Overlay: take max with existing buffer this frame
            duty[addr] = Mathf.Max(duty[addr], dutyVal);
            dutyByTile[row * COLS + col] = Mathf.Max(dutyByTile[row * COLS + col], dutyVal);
        }
    }


    /*---------------------------------------------------------*/
    private static void SetDroneTint(Transform drone, Color c)
    {
        if (drone == null) return;

        // handle one or many renderers
        foreach (Renderer r in drone.GetComponentsInChildren<Renderer>())
        {
            // IMPORTANT: r.material instantiates a copy so we don’t overwrite the shared material
            r.material.color = c;
        }
    }

    List<Actuators> actuatorsBelly = new List<Actuators>();

    List<Actuators> lastDefined = new List<Actuators>();

    public List<Actuators> crashActuators = new List<Actuators>();

    public List<Actuators> actuatorsVariables = new List<Actuators>();

    public List<Actuators> actuatorNetwork = new List<Actuators>();

    public List<Actuators> actuatorsMovingPlane = new List<Actuators>();


    List<Actuators> finalList = new List<Actuators>();

    private Coroutine hapticsCoroutine = null;

    // Dictionary<AnimatedActuator, IEnumerator> animatedActuators = new Dictionary<AnimatedActuator, IEnumerator>();

    /// <summary>Latest centre of the swarm on the ground plane (player-centric X-Z).</summary>
    public static Vector2 swarmCentroid2D = Vector2.zero;

    public static bool gamePadConnected
    {
        get
        {
            return currentGamepad != null;
        }
    }

    Coroutine gamnePadCoroutine;

    #region HapticsGamePad

    private static Gamepad currentGamepad;

    public static bool send = false;

    #endregion

    public int sendEvery = 50; //1000;
    // Update is called once per frame

    public static void lateStart()
    {
        // launch start function
        GameObject.FindGameObjectWithTag("GameManager").GetComponent<HapticsTest>().Start();
    }

    void Start()
    {
        // --- ADD THESE CONSTANTS for clarity ---
        const int DRONE_SLAVE_ID = 0;    // Haptics for the main drone swarm
        const int OBSTACLE_SLAVE_ID = 1; // Haptics for obstacles

        VibraForge.Reset();
        print("HapticsTest Start");
        finalList = new List<Actuators>();
        actuatorsRange = new List<Actuators>();
        actuatorsVariables = new List<Actuators>();
        actuatorNetwork = new List<Actuators>();
        actuatorsMovingPlane = new List<Actuators>();
        crashActuators = new List<Actuators>();
        lastDefined = new List<Actuators>();
        // animatedActuators = new Dictionary<AnimatedActuator, IEnumerator>();

        _swarmFrame = new GameObject("SwarmFrame").transform;
        _swarmFrame.gameObject.hideFlags = HideFlags.HideInHierarchy;   // keep Hierarchy clean

        // ... (mapping definitions remain the same) ...

        Dictionary<int, int> angleMappingDict = new Dictionary<int, int> {
            {64, 160},{65, 115},{66, 65},{67, 20}, {120, 200}, {121, 245},{122, 295},{123, 340},
            {90, 160},{91, 115},{92, 65},{93, 20}, {210, 200}, {211, 245},{212, 295},{213, 340},
             {60, 340},{61, 295},{62, 245},{63, 200}, {150, 200}, {151, 245},{152, 295},{153, 340},
        };

    //     Dictionary<int, int> angleMappingDict = new Dictionary<int, int> {
    //     {20, 160},{21, 115},{22, 65},{23, 20}, {120, 200}, {121, 245},{122, 295},{123, 340},
    //     {90, 160},{91, 115},{92, 65},{93, 20}, {210, 200}, {211, 245},{212, 295},{213, 340},
    //      {16, 340},{17, 295},{18, 245},{19, 200}, {150, 200}, {151, 245},{152, 295},{153, 340},
    // };

        // Dictionary<int, int> angleMappingDict = new Dictionary<int, int> {
        //         {4, 160},{5, 115},{6, 65},{7, 20},
        //         {0, 340},{1, 295},{2, 245},{3, 200}
        //     };

        // int[] angleMapping = Haptics_Obstacle ? new int[] { 0, 1, 2, 3, 4, 5, 6, 7 } : new int[] { };
        int[] angleMapping = Haptics_Obstacle ? new int[] { 60, 61, 62, 63, 64, 65, 66, 67 } : new int[] { };
        // int[] angleMapping = Haptics_Obstacle ? new int[] { 16, 17, 18, 19, 20, 21, 22, 23 } : new int[] { };
        int[] crashMapping = Haptics_Crash ? new int[] { 4, 5, 124, 125 } : new int[] { };

        // --- OBSTACLE ACTUATOR CREATION ---
        for (int i = 0; i < angleMapping.Length; i++)
        {
            int adresse = angleMapping[i];
            int angle = angleMappingDict.ContainsKey(adresse) ? angleMappingDict[adresse] : 0;
            var pidActuator = new PIDActuator(adresse: adresse, angle: angle,
                                                    kp: 0f, kd: 160, referencevalue: 0,
                                                    refresh: CloseToWallrefresherFunction);

            // --- ASSIGN THE SLAVE ID ---
            pidActuator.SlaveId = OBSTACLE_SLAVE_ID; // This command will now go to Slave #1

            actuatorsRange.Add(pidActuator);
        }

        // --- CRASH ACTUATOR CREATION ---
        for (int i = 0; i < crashMapping.Length; i++)
        {
            int adresse = crashMapping[i];
            var crashActuator = new Actuators(adresse, 0);

            // --- ASSIGN THE SLAVE ID ---
            crashActuator.SlaveId = DRONE_SLAVE_ID; // Crash commands will go to Slave #0

            crashActuators.Add(crashActuator);
        }

        // ... (rest of the Start method is the same) ...

        finalList.AddRange(actuatorsRange);
        finalList.AddRange(crashActuators);
        finalList.AddRange(actuatorNetwork);
        finalList.AddRange(actuatorsVariables);
        finalList.AddRange(actuatorsMovingPlane);

        if (hapticsCoroutine != null)
        {
            StopCoroutine(hapticsCoroutine);
        }

        hapticsCoroutine = StartCoroutine(HapticsCoroutine());

        currentGamepad = Gamepad.current;
        if (currentGamepad == null)
        {
            Debug.LogWarning("No gamepad connected.");
        }
        else
        {
            currentGamepad.SetMotorSpeeds(0.0f, 0.0f);
        }
    
        // --- Logging setup ---
        if (enableLogging)
        {
            logStartTime = Time.time;
            string logDir = Application.dataPath + "/Logs";
            if (!System.IO.Directory.Exists(logDir))
                System.IO.Directory.CreateDirectory(logDir);

            logFilePath = $"{logDir}/SizeRenderLog_{DateTime.Now:yyyyMMdd_HHmmss}.csv";
            System.IO.File.WriteAllText(logFilePath, "time,halfW01,d0,d1,d2,d3\n");
        }

}

    // Disable is called when the object is disabled
    void Disable()
    {
       // hapticsThread.Abort();
        currentGamepad.SetMotorSpeeds(0, 0);
    }
    
    #region Gamepad Crash Prediction
    
    private void OnEnable()
    {
        InputSystem.onDeviceChange += OnDeviceChange;
        currentGamepad = Gamepad.current; // Store the currently connected gamepad (if any)
    }

    private void OnDisable()
    {
        InputSystem.onDeviceChange -= OnDeviceChange;
    }

    private void OnDeviceChange(InputDevice device, InputDeviceChange change)
    {
        if (device is Gamepad gamepad)
        {
            switch (change)
            {
                case InputDeviceChange.Added:
                    Debug.Log("Controller Connected: " + gamepad.name);
                    currentGamepad = gamepad;
                    break;

                case InputDeviceChange.Removed:
                    Debug.Log("Controller Disconnected!");
                    
                    // Check if the removed device was the active gamepad
                    if (currentGamepad == gamepad)
                    {
                        currentGamepad = null;
                    }
                    break;
            }
        }
    }

    #endregion

    IEnumerator HapticsCoroutine()
    {
        while (true)
        {
            // (A) clear visual buffers
            Array.Clear(duty, 0, duty.Length);
            Array.Clear(dutyByTile, 0, dutyByTile.Length);

            // HighlightClosestDrone(); // highlight the closest drone to the swarm centroid

            foreach (Actuators actuator in finalList)
            {
                actuator.update();
            }

            //  sendCommands();
            UpdateSwarmFrameAndLog();

            yield return new WaitForSeconds(sendEvery / 1000);
        }
    }


    // void animationHandler(int start, AnimatedActuator actuator)
    // {
    //     if(animatedActuators.ContainsKey(actuator)) {
    //         StopCoroutine(animatedActuators[actuator]);
    //         actuator.stopAnimation();
    //     }

    //     actuator.defineAnimation(start, actuator.dutyIntensity);
    //     animatedActuators[actuator] = hapticAnimation(start, actuator);
    //     StartCoroutine(animatedActuators[actuator]);
    // }


    #region ForceActuators

    Actuators getDirectionActuator(Vector3 direction, List<Actuators> actuatorList)
    {
        float angle = Vector3.SignedAngle(direction, CameraMovement.embodiedDrone.transform.forward, Vector3.up);
        if(angle < 0) {
            angle += 360;
        }

        //FIUND THE closest actuator
        float minAngle = 360;
        Actuators closestActuator = null;
        foreach(Actuators actuator in actuatorList) {
            float diff = Math.Abs(actuator.Angle - angle);
            if(diff < minAngle) {
                minAngle = diff;
                closestActuator = actuator;
            }
        }

        return closestActuator;
    }
    
    void ForceActuator(RefresherActuator actuator)
    {   
        
        List<Vector3> forces = swarmModel.swarmOlfatiForces;
        actuator.dutyIntensity = 0;
        actuator.frequency = 1;
        foreach(Vector3 forcesDir in forces) {
            float angle = Vector3.SignedAngle(forcesDir, CameraMovement.forward, -CameraMovement.up)-180;
            if(angle < 0) {
                angle += 360;
            }
            
            float diff = Math.Abs(actuator.Angle - angle);
            if (diff < 45)
            {
                actuator.dutyIntensity = Mathf.Max(actuator.dutyIntensity, (int)(forcesDir.magnitude * 2));
                actuator.frequency = 1;
            }
        }
    }
    #endregion

    #region ObstacleInRange

    void CloseToWallrefresherFunction(PIDActuator actuator)
    {
        PrepareObstacleAssignments();  // compute once per frame

        actuator.dutyIntensity = 0;
        actuator.frequency = 1;

        if (_assignedMagnitude.TryGetValue(actuator.Adresse, out float mag))
        {
            actuator.UpdateValue(mag);
            dutyObstacle[actuator.Adresse] = _assignedDuty[actuator.Adresse]; // for visualization
            // Debug.Log($"[CloseToWallrefresherFunction] addr={actuator.Adresse} assigned mag={mag:F3} duty={_assignedDuty[actuator.Adresse]}");

            return;
        }
        else
        {
            // ensure instant clear when no assignment for this address
            dutyObstacle[actuator.Adresse] = 0;
        }
    }

    void PrepareObstacleAssignments()
    {
        if (_assignmentsFrame == Time.frameCount) return;   // already computed this frame

        _assignmentsFrame = Time.frameCount;
        _assignedMagnitude.Clear();
        _assignedDuty.Clear();

        var forces = swarmModel.swarmObstacleForces;
        if (forces == null || forces.Count == 0) return;

        // Candidate actuators on the horizontal ring
        // var ringActuators = actuatorsRange.Where(a => a.Angle >= 0).ToList();
        var ringActuators = actuatorsRange.Where(a => a.Angle >= 0)
                                 .OfType<PIDActuator>()
                                 .ToList();

        var assignedForce = new bool[forces.Count]; // track which forces were consumed by a ring actuator

        // ---- Map each force to its single best ring actuator (by smallest wrapped angle diff) ----
        for (int i = 0; i < forces.Count; i++)
        {
            Vector3 f = forces[i];
            if (f.sqrMagnitude <= 1e-6f || ringActuators.Count == 0) continue;

            // keep your velocity->force gating
            // float threshold = f.magnitude > 3.5f ? 0.3f : 0.7f;
            // if (Vector3.Dot(MigrationPointController.alignementVector.normalized, -f.normalized) <= threshold)
            //     continue;

            // same azimuth computation you used before
            float forceAngle = Vector3.SignedAngle(f, CameraMovement.forward, -CameraMovement.up) - 180f;
            if (forceAngle < 0f) forceAngle += 360f;

            PIDActuator best = null;
            float bestAbsDelta = float.MaxValue;

            foreach (var act in ringActuators)
            {
                float delta = Mathf.DeltaAngle(act.Angle, forceAngle); // [-180, 180]
                float absDelta = Mathf.Abs(delta);
                if (absDelta < bestAbsDelta)
                {
                    bestAbsDelta = absDelta;
                    best = act;
                }
            }

            // keep your ±30° sectoring (equivalent to diff < 30 or > 330)
            if (best == null || bestAbsDelta > 30f) continue;

            float mag = f.magnitude;
            // int vizDuty = (int)(mag / 8.0f);
            int vizDuty = (int)(mag * 10.0f);
            // Debug.Log($"[PrepareObstacleAssignments] addr={best.Adresse} mag={mag:F3} forceAngle={forceAngle:F1} bestAbsDelta={bestAbsDelta:F1} vizDuty={vizDuty}");

            // one actuator per force; if multiple forces target same actuator, keep the strongest
            if (_assignedMagnitude.TryGetValue(best.Adresse, out float existing))
            {
                if (mag > existing)
                {
                    _assignedMagnitude[best.Adresse] = mag;
                    _assignedDuty[best.Adresse]      = vizDuty;
                }
            }
            else
            {
                _assignedMagnitude[best.Adresse] = mag;
                _assignedDuty[best.Adresse]      = vizDuty;

            }

            assignedForce[i] = true; // this force has been consumed by a ring actuator
        }
    }

    #endregion

    #region crashActuators 
    void DroneCrashrefresher(RefresherActuator actuator)
    {
        return;
    }

    public void crash(bool reset )
    {
        print("Crash and reset " + reset);
        if(reset) {
            foreach(Actuators actuator in crashActuators) {
                actuator.dutyIntensity = 0;
                actuator.frequency = 1;

                actuator.sendValue();
            }
        }
        
        StartCoroutine(crashCoroutine());
    }

    public IEnumerator crashCoroutine()
    {

        foreach(Actuators actuator in crashActuators) {
            actuator.dutyIntensity = 10;
            actuator.frequency = 1;
            actuator.sendValue();
         //   print("Actuator: " + actuator.Adresse + " Duty: " + actuator.duty + " Frequency: " + actuator.frequency);
        }

        yield return new WaitForSeconds(1);

        foreach(Actuators actuator in crashActuators) {
            actuator.dutyIntensity = 0;
            actuator.frequency = 1;
            actuator.sendValue();
        }
    }
    
    #endregion

    #region swarmVelocityActuators
    void SwarmVelocityRefresher(RefresherActuator actuator)
    {
        Vector3 velDir  = swarmModel.swarmVelocityAvg;
        if(velDir.magnitude < 1) {
            actuator.dutyIntensity = 0;
            actuator.frequency = 1;
            return;
        }
        if(CameraMovement.embodiedDrone != null) { //carefull change only return the Vector



            float angle = Vector3.SignedAngle(velDir, CameraMovement.embodiedDrone.transform.forward, Vector3.up);
            if(angle < 0) {
                angle += 360;
            }
            
            float diff = Math.Abs(actuator.Angle - angle);
            if(diff < 30) {
                actuator.dutyIntensity = 4;
                actuator.frequency = 2;
                return;
            }
        }else{
       
            float angle = Vector3.SignedAngle(velDir, CameraMovement.cam.transform.up, -Vector3.up);
            if(angle < 0) {
                angle += 360;
            }
            
            float diff = Math.Abs(actuator.Angle - angle);
            if(diff < 30) {
                actuator.dutyIntensity = 4;
                actuator.frequency = 2;
                return;
            }
        }


        actuator.dutyIntensity = 0;
        actuator.frequency = 1;
    }

    #endregion

    #region NetworkActuators


    int step = 4;
    IEnumerator hapticAnimation(int oldActIntensity, Actuators newAct)
    {
        int startIntensity = oldActIntensity;
        int endIntensity = newAct.dutyIntensity;

        int currentIntensity = startIntensity;

        while(currentIntensity != endIntensity) {
            if(currentIntensity < endIntensity) {
                currentIntensity = currentIntensity + step > endIntensity ? endIntensity : currentIntensity + step;
            }else {
                currentIntensity = currentIntensity - step < endIntensity ? endIntensity : currentIntensity - step;
            }

            VibraForge.SendCommand(newAct.Adresse, (int)currentIntensity == 0 ? 0:1, (int)currentIntensity, (int)newAct.frequency);
            yield return new WaitForSeconds(0.1f);
        }

    }

    IEnumerator hapticAnimation(Actuators newAct)
    {
        int startIntensity = 0;
        int endIntensity = newAct.dutyIntensity;

        int currentIntensity = startIntensity;

        while(currentIntensity != endIntensity) {
            if(currentIntensity < endIntensity) {
                currentIntensity = currentIntensity + step > endIntensity ? endIntensity : currentIntensity + step;
            }else {
                currentIntensity = currentIntensity - step < endIntensity ? endIntensity : currentIntensity - step;
            }
            VibraForge.SendCommand(newAct.Adresse, (int)currentIntensity == 0 ? 0:1, (int)currentIntensity, (int)newAct.frequency);
            yield return new WaitForSeconds(0.1f);
        }

    }
    void movingPlaneRefresher(RefresherActuator actuator)
    {

        float score = swarmModel.swarmConnectionScore;
        int resol = 10;

        score*=resol;
        int angleToMove = (int)score;


        if(score >= 9f)
        {
            if(actuator.Angle >= 8 )
            {
                actuator.dutyIntensity = 13;
                actuator.frequency = 3;
                return;
            }
         }

        if(score <= 0)
        {
            actuator.dutyIntensity = 0;
            actuator.frequency = 1;
            return;
        }

        if(actuator.Angle == angleToMove) {
            actuator.dutyIntensity = (int)Mathf.Min(14, Mathf.Max(8, score));
            actuator.frequency = 1;
            return;
        }


        actuator.dutyIntensity = 0;
        actuator.frequency = 1;

    }

    private void OnApplicationQuit()
    {
        if (enableLogging) SaveHapticsLog();
    }

    private void SaveHapticsLog()
    {
        using (System.IO.StreamWriter sw = new System.IO.StreamWriter(logFilePath, true))
        {
            for (int i = 0; i < logTime.Count; i++)
            {
                int[] d = logSizeDuties[i];
                string line = $"{logTime[i]:F3},{logHalfW01[i]:F4},{d[0]},{d[1]},{d[2]},{d[3]}";
                sw.WriteLine(line);
            }
        }

        Debug.Log($"[HapticsTest] Saved size rendering log ({logTime.Count} samples) → {logFilePath}");
    }
    #endregion
}

// public class AnimatedActuator: RefresherActuator
// {
//     int animationEnd = 0;
//     int animationStart = 0;

//     public void defineAnimation(int start, int end)
//     {
//         animationStart = start;
//         animationEnd = end;
//     }

//     public void stopAnimation()
//     {
//         VibraForge.SendCommand(Adresse, 0, 0, 1);
//     }
//     public AnimatedActuator(int adresse, float angle, updateFunction refresh) : base(adresse, angle, refresh)
//     {
//     }
// }

public class RefresherActuator: Actuators
{
    public delegate void updateFunction(RefresherActuator actuator);
    public updateFunction refresherFunction { get; set; }

    public RefresherActuator(int adresse, float angle, updateFunction refresh) : base(adresse, angle)
    {
        this.refresherFunction = refresh;
    }

    public override void update()
    {
        refresherFunction(this);
        sendValue();
    }
}

public class PIDActuator : Actuators // creae Ki
{
    public float Kp { get; set; }
    public float Kd { get; set; }

    public float referenceValue { get; set; }

    public float lastValue = 0;

    public delegate void updateFunction(PIDActuator actuator);
    public updateFunction refresherFunction { get; set; }

    public PIDActuator(int adresse, float angle, float kp, float kd, float referencevalue, updateFunction refresh) : base(adresse, angle)
    {
        this.Kp = kp;
        this.Kd = kd;
        this.referenceValue = referencevalue;
        this.refresherFunction = refresh;
    }

    // public void UpdateValue(float newValue)
    // {
    //     float error = newValue - referenceValue;
    //     float derivative = newValue - lastValue;

    //     lastValue = newValue;
    //     dutyIntensity = Mathf.Max((int)(Kp * error + Kd * derivative), dutyIntensity);

    //     frequency = 2;
    // }

    // public void UpdateValue(float newValue)
    // {
    //     float error = newValue - referenceValue;
    //     float derivative = newValue - lastValue;
    //     lastValue = newValue;
    //     // Calculate the raw PID output
    //     float pidOutput = Kp * error + Kd * derivative;
    //     // Set the duty intensity, clamping it between 0 and 14
    //     dutyIntensity = Mathf.Clamp(Mathf.RoundToInt(pidOutput), 0, 14);
    //     frequency = 2;
    // }

    public void UpdateValue(float newValue)
    {
        // 把 newValue 映射到 0..14，可按需要缩放
        // 例：障碍力幅值≈0..1 → 乘以 10 得到 0..10 的占空比
        float p = 10f * newValue;                  // ← 调整这个“10f”匹配你的幅值范围
        float d = Kd * (newValue - lastValue);     // 保留微分做“变化增强”
        lastValue = newValue;

        float outF = p + d;                        // 先按幅值给基线，再加微分
        dutyIntensity = Mathf.Clamp(Mathf.RoundToInt(outF), 0, 14);
        frequency = 2;
    }

    override public void update()
    {
        refresherFunction(this);
        sendValue();
    }

}

public class Actuators
{
    public int SlaveId { get; set; }    // to target a specific slave (0=all)
    public int Adresse { get; set; }
    public float Angle { get; set; }

    public int dutyIntensity = 0;
    public int frequency = 1;

    public int lastSendDuty = 0;
    public int lastSendFrequency = 0;


    public int duty
    {
        get{
            if(dutyIntensity > 14) {
                return 14;
            }else if (dutyIntensity < 0) {
                return 0;
            }else {
                return dutyIntensity;
            }
        }
    }

    public Actuators(int adresse, float angle)
    {
        Adresse = adresse;
        Angle = angle;
    }

    //create operator overload
    public bool Equal(Actuators a)
    {
        return a.duty == this.duty && a.frequency == this.frequency;
    }

    public void forceIntensity(float force)
    {
        dutyIntensity = (int)force;
        frequency = 1;
    }

    public virtual void update()
    {
        sendValue();
        return;
    }

    public virtual void sendValue()
    {
        if( lastSendFrequency != frequency || lastSendDuty != duty) {
            // --- MODIFIED LINE ---
            // Add "this.SlaveId" as the first parameter to the SendCommand call.
            VibraForge.SendCommand(Adresse, (int)duty == 0 ? 0:1, (int)duty, (int)frequency);
            
            lastSendDuty = duty;
            lastSendFrequency = frequency;
    //      Debug.Log("Send Command: " + Adresse + " Duty: " + duty + " Frequency: " + frequency);
        }

        // VibraForge.SendCommand(Adresse, (int)duty == 0 ? 0:1, (int)duty, (int)frequency);

    }

    public IEnumerator sendDelayedVal(float delay)
    {
        yield return new WaitForSeconds(delay);
        sendValue();

        yield return new WaitForSeconds(0.1f);
        HapticsTest.send = false;
    }


}


// public class GamepadMonitor : MonoBehaviour
// {
//     private void OnEnable()
//     {
//         InputSystem.onDeviceChange += OnDeviceChange;
//     }

//     private void OnDisable()
//     {
//         InputSystem.onDeviceChange -= OnDeviceChange;
//     }

//     private void OnDeviceChange(InputDevice device, InputDeviceChange change)
//     {
//         if (device is Gamepad)
//         {
//             switch (change)
//             {
//                 case InputDeviceChange.Added:
//                     Debug.Log("Gamepad Connected: " + device.name);
//                     break;
//                 case InputDeviceChange.Removed:
//                     Debug.Log("Gamepad Disconnected!");
//                     break;
//             }
//         }
//     }
// }