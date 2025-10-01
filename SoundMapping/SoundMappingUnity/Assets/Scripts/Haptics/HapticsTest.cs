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

    /// <summary>Returns the geometric centre of all swarm members.</summary>
    // public static Vector3 GetSwarmCentroid(IReadOnlyList<Transform> drones)
    // {
    //     if (drones == null || drones.Count == 0)
    //         return Vector3.zero;                       // fallback: no drones

    //     Vector3 sum = Vector3.zero;
    //     foreach (Transform t in drones) sum += t.position;
    //     return sum / drones.Count;                     // (x, y, z)
    // }

    // 1) class field
    // private readonly int[] duty = new int[40];   // 20-cell visual panel (index 0-19)

    // 3) accessor
    // public int[] GetDutySnapshot() => duty;
    public int[] GetDutySnapshot()
    {
        // Debug.Log($"Duty[0] from HapticsTest = {dutyByTile[0]} (frame {Time.frameCount})");
        return dutyByTile;
    }

    public static readonly int[] ObstacleAddrs =
    { 60, 61, 62, 63, 64, 65, 66, 67 };

    public static int[] GetObstacleDutySnapshot()   // 8 长度
    {
        // int[] snap = new int[ObstacleAddrs.Length];
        // for (int i = 0; i < ObstacleAddrs.Length; i++)
        //     snap[i] = duty[ObstacleAddrs[i]];
        // return snap;
        return duty;
    }

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
            if (sq < bestSq) {bestSq = sq; closest = t; }
        }
        if (closest == null) { return; }

        // 3) if it changed, restore the old one and tint the new one
        if (_highlightedDrone != null && _highlightedDrone != closest)
        {
            print("Closest drone: "+ closest);
            SetDroneTint(_highlightedDrone, Color.white);    // or whatever the default is
        }
        _highlightedDrone = closest;
        SetDroneTint(_highlightedDrone, _highlightColor);
    }

    /*---------------------------------------------------------------*/
    /* Editor-only visual of swarm centroid                          */
    /*---------------------------------------------------------------*/
#if UNITY_EDITOR            // keeps the code out of runtime builds
    void OnDrawGizmos()
    {
#if UNITY_EDITOR        // avoid shipping gizmo code in builds
        var drones = FindObjectsOfType<DroneController>()
             .Select(d => d.transform).ToList();
        if (drones == null || drones.Count == 0) return;

        // Vector3 c = drones.Aggregate(Vector3.zero,
        //             (sum, t) => sum + t.position) / drones.Count;
        Vector3 c = GetSwarmCentroid(drones);  // or use the centroid function above

        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(c, 0.2f);        // 5 cm sphere
        Gizmos.DrawLine(c, c + Vector3.up); // little “stem” so it’s easy to spot
#endif
    }
#endif

    
    #if UNITY_EDITOR
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
    #endif
    
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

    //     col 0   col 1   col 2   col 3      (front view)
    // int[,] matrix = {
    //     {  0,   1,   2,   3 },   // row 0  (bottom)
    //     {  4,   5,   6,   7 },   // row 1
    //     {  8,   9,  10,  11 },   // row 2
    //     { 12,  13,  14,  15 },   // row 3
    //     { 16,  17,  18,  19 }    // row 4  (top)
    // };

    // int[,] matrix = {
    //     {  9,   8,   7,   6 },   // row 0  (bottom)
    //     {  2,   3,   4,   5 },   // row 1
    //     {  1,   0,  30,  31 },   // row 2
    //     { 35,  34,  33,  32 },   // row 3
    //     { 36,  37,  38,  39 }    // row 4  (top)
    // };
    
    // int[,] matrix = {
    //     { 36,  37,  38,  39 },    // row 0  (top)
    //     { 35,  34,  33,  32 },   // row 1
    //     {  1,   0,  30,  31 },   // row 2
    //     {  2,   3,   4,   5 },   // row 3
    //     {  9,   8,   7,   6 }   // row 4  (bottom)

    // };

    // int[,] matrix = {
    //     {6, 7, 8, 9},
    //     {5, 4, 3, 2},
    //     {31, 30, 0, 1},
    //     {32, 33, 34, 35},
    //     {39, 38, 37, 36}

    // };
    // private static readonly int[,] matrix = {
    // {10, 11, 12, 13, 14},
    // {9, 8, 7, 6, 5},
    // {0, 1, 2, 3, 4},
    // {30, 31, 32, 33, 34},
    // {39, 38, 37, 36, 35},
    // {40, 41, 42, 43, 44}
    // };
    // // vibratorAddress = matrix[row, col]

    // private static readonly int[,] matrix = {
    // {-1,1,0,-1},
    // { 2, 3, 30, 31},
    // {61, 60, 33, 32},
    // {62, 63, 90, 91},
    // {-1,93,92,-1}
    // };
    // vibratorAddress = matrix[row, col]

    // private static readonly int[,] matrix = {
    // {119,1,0,119},
    // { 2, 3, 30, 31},
    // {35, 34, 33, 32},
    // {36, 37, 38, 39},
    // {119,41,40,119}
    // };

    private static readonly int[,] matrix = {
    {3,2,1,0},
    { 4, 5, 6, 7},
    {33, 32, 31, 30},
    {34, 35, 36, 37}
    };

    // private static readonly int[,] matrix = {
    // {31, 30, 3, 2},
    // {32, 33, 60, 61},
    // {91, 90, 63, 62}
    // };
    // // vibratorAddress = matrix[row, col]

    private static readonly int[] duty = new int[120];   // one per vibrator (0-14)
    private static readonly int[] dutyByTile = new int[matrix.Length];   // 20-cell visual panel (0-14)
    int[] freq   = new int[120];   // keep simple: all 1

    // —— 行为参数 —— 可按需要微调
    const float EPS          = 0.9f;   // 变化阈值：≈“1架无人机变动”
    const float STABLE_FOR   = 2f;   // 连续稳定多久后开始衰减（秒）
    const float DECAY_PER_S  = 8f;     // 衰减速度（每秒减少的 duty “格数”）
    const int   DUTY_GAIN    = 3;      // 密度→强度：每架无人机 +2
    const int   DUTY_MAX     = 14;     // 上限

    // —— 状态缓存 —— 按你的地址空间大小分配
    float[] lastRaw      = new float[256]; // 上一帧的密度
    float[] stableTimer  = new float[256]; // “持续稳定”的计时
    int[]   smoothedDuty = new int[256];   // 平滑后的最终强度（写回给硬件）
    float[] rawByAddr    = new float[256]; // 本帧密度（临时）

    // —— 可调参数 ——
    const float GAIN_PER_DRONE = 3f;   // 一架无人机贡献的总强度（等价于你原来的 +2）
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
    [SerializeField] float SIZE_EPS01 = 0.004f;    // “small change” threshold in normalized units
    [SerializeField] float SIZE_STABLE_FOR = 1.30f; // must stay small for this long (s)

    // --- Size-change gate state ---
    float _lastHalfW01 = -1f;
    float _sizeStableTimer = 0f;

    /// <summary>
    /// Returns the horizontal and vertical half-sizes (metres) of the swarm,
    /// measured in the SwarmFrame’s local X-Y plane.
    /// </summary>
    // private static void GetDynamicExtents(IReadOnlyList<Transform> drones,
    //                                     Transform swarmFrame,
    //                                     out float halfWidth,
    //                                     out float halfHeight)
    // {
    //     float maxAbsX = 0f;
    //     float maxAbsY = 0f;

    //     foreach (var t in drones)
    //     {
    //         Vector3 p = swarmFrame.InverseTransformPoint(t.position); // local
    //         maxAbsX = Mathf.Max(maxAbsX, Mathf.Abs(p.x));
    //         maxAbsY = Mathf.Max(maxAbsY, Mathf.Abs(p.z));
    //     }

    //     // Avoid divide-by-zero when the swarm collapses to a point
    //     halfWidth = Mathf.Max(maxAbsX, 0.01f);   // at least 1 cm
    //     halfHeight = Mathf.Max(maxAbsY, 0.01f);
    // }

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

    int ColFromX(float x, float halfW, float actuator_W)    // halfW ≥ 0.01
    {
        // float t = (x + halfW) / (2f * halfW);      // → [0..1]
        // return Mathf.Clamp(Mathf.RoundToInt(t * actuator_W + center_W - actuator_W / 2f), 0, Mathf.RoundToInt(initial_actuator_W));
        float t = x / 4.5f * 2f;      // → [0..1]
        return Mathf.Clamp(Mathf.RoundToInt(t + center_W), 0, Mathf.RoundToInt(initial_actuator_W));
        // return Mathf.Clamp(Mathf.RoundToInt(t *  3f), 0, 3);
    }

    int RowFromY(float y, float halfH, float actuator_H)    // halfH ≥ 0.01
    {
        // float t = (-y + halfH) / (2f * halfH);      // → [0..1]
        // return Mathf.Clamp(Mathf.RoundToInt(t * actuator_H + center_H - actuator_H / 2f), 0, Mathf.RoundToInt(initial_actuator_H));
        float t = -y / 4.5f * 2f;      // → [0..1]
        return Mathf.Clamp(Mathf.RoundToInt(t + center_H), 0, Mathf.RoundToInt(initial_actuator_H));
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
        Vector3 centroid = GetSwarmCentroid(drones); // or use the centroid function above
        _swarmFrame.position = centroid;           // place at the centroid
        _swarmFrame.rotation = Quaternion.LookRotation(
                                    embodiedDrone.forward,
                                    embodiedDrone.up);
        // Debug.Log($"swarmFrame.rotation = {_swarmFrame.rotation.eulerAngles:F2} " +
        //           $"(centroid at {centroid:F2})");

        // ② measure current half-sizes
        GetDynamicExtents(drones, _swarmFrame, out halfW, out halfH);

        // --- measure normalized half-width and gate stability ---
        float dt = Time.deltaTime; // we’ll reuse dt later as well
        float halfW01 = Mathf.Clamp01(halfW / refWorldHalfW);     // normalize width to [0,1]
        float sizeDiff01 = (_lastHalfW01 < 0f) ? 1f : Mathf.Abs(halfW01 - _lastHalfW01);

        // hysteresis on size: must stay “small change” for a while
        if (sizeDiff01 < SIZE_EPS01) _sizeStableTimer += dt;
        else                         _sizeStableTimer  = 0f;

        bool muteTargetRow = (_sizeStableTimer >= SIZE_STABLE_FOR);

        // Debug.Log($"_lastHalfW01: {_lastHalfW01:F3}, Current halfW01: {halfW01:F3}, Difference: {sizeDiff01:F3}");

        _lastHalfW01 = halfW01;

        //Debug.Log($"halfW = {halfW:F2} m, halfH = {halfH:F2} m " +
        //          $"(norm {halfW01:F3}, Δ {sizeDiff01:F3}, " +
        //          $"{_sizeStableTimer:F2}s stable, " +
        //          $"{(muteTargetRow ? "MUTING" : "active")})");

        if (initial_actuator_W / initial_actuator_H > halfW / halfH)
        {
            actuator_W = initial_actuator_H * halfW / halfH; // make it 4:3 aspect ratio
            //Debug.Log($"actuator_W = {actuator_W:F2} (halfW {halfW:F2}, halfH {halfH:F2})");
        }
        else
        {
            actuator_H = initial_actuator_W * halfH / halfW; // make it 4:3 aspect ratio
            //Debug.Log($"actuator_H = {actuator_H:F2} (halfW {halfW:F2}, halfH {halfH:F2})");
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

        // /*-------------------------------------------------------------*
        // * 2) mark light vibration everywhere a drone appears
        // *-------------------------------------------------------------*/
        // const int LIGHT_DUTY = 3;                   // tweak to taste (1–4)
        // foreach (Transform d in connectedDrones)
        // {
        //     Vector3 local = _swarmFrame.InverseTransformPoint(d.position);

        //     int col = ColFromX(local.x, halfW, actuator_W);    // 0 … 3
        //     int row = RowFromY(local.z, halfH, actuator_H);    // 0 … 4
        //     int addr = matrix[row, col];            // 4 × 5 lookup

        //     // Debug.Log($"Drone {d.name} at {local:F2} " +
        //     //           $"(col {col}, row {row}, addr {addr})");

        //     duty[addr] = duty[addr] + 2; //LIGHT_DUTY;                // overwrite is fine
        //     if (duty[addr] > 14) duty[addr] = 14; // clamp to max

        //     int tile = (row * (Mathf.RoundToInt(initial_actuator_W) + 1)) + col;       // 0 … 19 for the visual panel
        //     dutyByTile[tile] = dutyByTile[tile] + 1; //LIGHT_DUTY;          // same duty for visual panel
        // }
        
        // 1) 清空本帧目标强度
        System.Array.Clear(targetDuty, 0, targetDuty.Length);
        System.Array.Clear(dutyByTile, 0, dutyByTile.Length); 

        // 2) 每架无人机 -> 对周围4格做双线性分配
        foreach (Transform d in connectedDrones)
        {
            Vector3 local = _swarmFrame.InverseTransformPoint(d.position);

            // ---- 连续坐标（0..W-1 / 0..H-1），保证在边界内 ----
            // float u = Mathf.Clamp01((local.x*2f) / (4.5f)) * COLS_MINUS1;
            // float v = Mathf.Clamp01((local.z + halfH) / (4.5f)) * ROWS_MINUS1;

            // float t = local.x / 4.5f * 2f;      // → [0..1]
            float u = Mathf.Clamp(local.x / 4.5f * 2f + center_W, 0, Mathf.RoundToInt(initial_actuator_W));
            float v = Mathf.Clamp(-local.z / 4.5f * 2f + center_H, 0, Mathf.RoundToInt(initial_actuator_H));


            int c0 = Mathf.FloorToInt(u);
            int r0 = Mathf.FloorToInt(v);
            int c1 = Mathf.Min(c0 + 1, COLS - 1);
            int r1 = Mathf.Min(r0 + 1, ROWS - 1);

            // 权重
            float wc1 = u - c0, wc0 = 1f - wc1;
            float wr1 = v - r0, wr0 = 1f - wr1;

            // 四邻域权重
            float w00 = wc0 * wr0;
            float w10 = wc1 * wr0;
            float w01 = wc0 * wr1;
            float w11 = wc1 * wr1;

            // 每架无人机的总贡献为 GAIN_PER_DRONE，按权重分摊
            void Add(int rr, int cc, float w)
            {
                int addr = matrix[rr, cc];
                targetDuty[addr] += GAIN_PER_DRONE * w;
            }

            Add(r0, c0, w00);
            Add(r0, c1, w10);
            Add(r1, c0, w01);
            Add(r1, c1, w11);
        }

        // float dt    = Time.deltaTime;
        float alpha = 1f - Mathf.Exp(-dt / TAU_SMOOTH);

        // dutyByTile 必须是 ROWS*COLS 大小

        // for (int row = 0; row < ROWS; row++)        // < 不是 <=
        // {
        //     for (int col = 0; col < COLS; col++)    // < 不是 <=
        //     {
        //         int addr = matrix[row, col];

        //         smoothDuty[addr] = Mathf.Lerp(smoothDuty[addr], targetDuty[addr], alpha);
        //         int outDuty = Mathf.Min(DUTY_MAX, Mathf.RoundToInt(smoothDuty[addr]));
        //         duty[addr] = outDuty;

        //         // 面板的步长=COLS（不要 +1）
        //         int tile = row * COLS + col;
        //         dutyByTile[tile] = outDuty;
        //     }
        // }

        // 1) Smooth per cell, accumulate per-COLUMN sums, and clear outputs for now
        int[] colSum = new int[COLS];

        for (int row = 0; row < ROWS; row++)
        {
            for (int col = 0; col < COLS; col++)
            {
                int addr = matrix[row, col];

                // smooth toward this frame’s target
                smoothDuty[addr] = Mathf.Lerp(smoothDuty[addr], targetDuty[addr], alpha);
                int cellDuty = Mathf.Min(DUTY_MAX, Mathf.RoundToInt(smoothDuty[addr]));

                colSum[col] += cellDuty;                // accumulate by column

                // we'll only light the chosen row later
                duty[addr] = 0;
                dutyByTile[row * COLS + col] = 0;
                // duty[addr] = Mathf.RoundToInt(smoothDuty[addr]);
                // dutyByTile[row * COLS + col] = Mathf.RoundToInt(smoothDuty[addr]);
            }
        }

        // 2) Collapse each column to a single actuator on TARGET_ROW
        const int TARGET_ROW = 0;              // <- pick which row receives the column sum
        // choose scaling: strict sum (1f) or average (1f/ROWS). Average avoids fast saturation.
        float Compress = 1f / ROWS;

        int maxDuty = -1;
        int maxCol  = -1;
        int maxAddr = -1;

        for (int col = 0; col < COLS; col++)
        {
            int collapsed = Mathf.Min(DUTY_MAX, Mathf.RoundToInt(colSum[col] * Compress));

            // --- NEW: if swarm width is stable, mute the entire TARGET_ROW ---
            if (muteTargetRow) collapsed = 0;

            int addr = matrix[TARGET_ROW, col];
            duty[addr] = collapsed;

            int tile = TARGET_ROW * COLS + col; // correct tile stride
            dutyByTile[tile] = collapsed;

            // track maximum
            if (collapsed > maxDuty) { maxDuty = collapsed; maxCol = col; maxAddr = addr; }
        }

        //Debug.Log($"[WidthBar] max duty = {maxDuty} at col {maxCol} (addr {maxAddr})");



        // 统计本帧每个地址的“无人机密度”
        // System.Array.Clear(rawByAddr, 0, rawByAddr.Length);   // rawByAddr[256]; declared elsewhere
        // System.Array.Clear(dutyByTile, 0, dutyByTile.Length);   // 可视面板本帧重填

        // foreach (Transform d in connectedDrones)
        // {
        //     Vector3 local = _swarmFrame.InverseTransformPoint(d.position);

        //     int col  = ColFromX(local.x, halfW, actuator_W);
        //     int row  = RowFromY(local.z, halfH, actuator_H);
        //     int addr = matrix[row, col];

        //     rawByAddr[addr] += 2f;  // 一架无人机计 1
        // }

        // // 将“密度 + 变化”转成 duty（有增益，有衰减）
        // // float dt = Time.deltaTime;  // 在协程里用这个即可（或用 sendEvery/1000f）
        // for (int row = 0; row <= Mathf.RoundToInt(actuator_H); row++)
        // {
        //     for (int col = 0; col <= Mathf.RoundToInt(actuator_W); col++)
        //     {
        //         int addr = matrix[row, col];
        //         float raw  = duty[addr];//rawByAddr[addr];                // 当前密度
        //         float diff = Mathf.Abs(raw - lastRaw[addr]); // 与上一帧的变化

        //         if (diff > EPS)
        //         {
        //             // 有“显著变化” → 按密度增长强度，且重置稳定计时
        //             stableTimer[addr] = 0f;
        //             // int inc = DUTY_GAIN * Mathf.RoundToInt(raw);  // ∝ 密度
        //             // smoothedDuty[addr] = Mathf.Min(DUTY_MAX, smoothedDuty[addr] + inc);
        //             smoothedDuty[addr] = Mathf.RoundToInt(raw);
        //         }
        //         else
        //         {
        //             // 变化很小 → 累计稳定时长；超过阈值后开始慢慢降为 0
        //             stableTimer[addr] += dt;
        //             if (stableTimer[addr] >= STABLE_FOR)
        //             {
        //                 float newDuty = smoothedDuty[addr] - DECAY_PER_S * dt;
        //                 smoothedDuty[addr] = (int)Mathf.Max(0f, newDuty);
        //             }
        //         }

        //         lastRaw[addr] = raw;

        //         // 写回硬件与可视面板（面板显示的是“平滑后的最终强度”）
        //         duty[addr] = smoothedDuty[addr];

        //         int tile = (row * (Mathf.RoundToInt(initial_actuator_W) + 1)) + col;
        //         dutyByTile[tile] = smoothedDuty[addr];
        //     }
        // }

        // /*-------------------------------------------------------------*
        // * 3) overwrite with STRONG duty for the embodied-drone cell
        // *-------------------------------------------------------------*/
        if (dutyByTile[1] == 0 && dutyByTile[2] == 0)   // only if the embodied drone is not already marked
        {
            // Vector3 localE = _swarmFrame.InverseTransformPoint(embodiedDrone.position);
            // int colE = ColFromX(localE.x, halfW, actuator_W);
            // int rowE = RowFromY(localE.z, halfH, actuator_H);
            // Debug.Log($"embodiedDrone at {localE:F2} " +
            //           $"(col {colE}, row {rowE})");
            // int addrE = matrix[rowE, colE];

            // duty[addrE] = 8;                       // full-strength buzz
            // dutyByTile[(rowE * (Mathf.RoundToInt(initial_actuator_W)+1)) + colE] = 8;    // same for visual panel

            // 🔔 自定义 —— 每秒闪几次？2 = 1 Hz（0.5 s 亮，0.5 s 灭）
            const float blinkRate = 3f;

            /* ……上面保持不变…… */
            Vector3 localE = _swarmFrame.InverseTransformPoint(embodiedDrone.position);
            int colE = ColFromX(localE.x, halfW, actuator_W);
            int rowE = 1;//RowFromY(localE.z, halfH, actuator_H);
            int addrE = matrix[rowE, colE];

            // === 让化身无人机所在格“闪烁” ===
            bool blinkOn  = (Mathf.FloorToInt(Time.time * blinkRate) & 1) == 0; // 奇偶翻转
            int  dutyVal  = blinkOn ? 7 : 0;

            duty[addrE] = dutyVal;
            dutyByTile[(rowE * (Mathf.RoundToInt(initial_actuator_W) + 1)) + colE] = dutyVal;


            //Debug.Log($"embodiedDrone addr {addrE} " +
            //$"(duty {duty[addrE]})");
        }

        // generate motion pattern when disconnection happens

        // ⑤ transmit (same as before)
        // for (int addr = 0; addr < 10; addr++)
        //     VibraForge.SendCommand(addr,
        //                         duty[addr] == 0 ? 0 : 1,
        //                         duty[addr],
        //                         1);
        // for (int addr = 30; addr < 40 && addr > 29; addr++)
        //     VibraForge.SendCommand(addr,
        //                         duty[addr] == 0 ? 0 : 1,
        //                         duty[addr],
        //                         1);

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

        /*-------------------------------------------------------------*
        * ④ choose the embodied-drone cell only
        *-------------------------------------------------------------*/
        // Vector3 localE = _swarmFrame.InverseTransformPoint(embodiedDrone.position);

        // // NB: use Y for vertical if your grid is front-view;               ⇣
        // // if you really want Z for “height”, keep RowFromY(localE.z, …)
        // int colE = ColFromX(localE.x, halfW);
        // int rowE = RowFromY(localE.z, halfH);

        // int addrE = matrix[rowE, colE];         // 4 × 5 → hardware addr
        // Debug.Log($"embodiedDrone addr {addrE} " +
        //             $"(duty {duty[addrE]})");
        // /*-------------------------------------------------------------*
        // * ⑤ send: 1) silence the old tactor (if any)
        // *          2) buzz the new one at full power
        // *-------------------------------------------------------------*/
        // if (_prevAddr != -1 && _prevAddr != addrE)
        // {
        //     // turn the old one off
        //     VibraForge.SendCommand(_prevAddr, 0, 0, 1);
        // }

        // if (addrE != _prevAddr)                 // changed cell → send a new on-command
        // {
        //     VibraForge.SendCommand(addrE, 1, 14, 1);   // enable, duty 14, freq 1
        //     _prevAddr = addrE;                         // remember for next frame
        // }

        /* if the drone stayed in the same cell as last frame,
        nothing is sent at all → less traffic */

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

    Dictionary<AnimatedActuator, IEnumerator> animatedActuators = new Dictionary<AnimatedActuator, IEnumerator>();

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
        VibraForge.Reset();
        print("HapticsTest Start");
        finalList = new List<Actuators>();
        actuatorsRange = new List<Actuators>();
        actuatorsVariables = new List<Actuators>();
        actuatorNetwork = new List<Actuators>();
        actuatorsMovingPlane = new List<Actuators>();
        crashActuators = new List<Actuators>();
        lastDefined = new List<Actuators>();
        animatedActuators = new Dictionary<AnimatedActuator, IEnumerator>();

        _swarmFrame = new GameObject("SwarmFrame").transform;
        _swarmFrame.gameObject.hideFlags = HideFlags.HideInHierarchy;   // keep Hierarchy clean


        //
        int[] mappingOlfati = Haptics_Forces ? new int[] {/*0,1,2,3,120,121,122,123*/} : new int[] {}; 
    //    int[] mappingOlfati = Haptics_Forces ? new int[] {90,91,92,93,180,181,182,183} : new int[] {}; 
        
        int [] velocityMapping = {}; //relative mvt of the swarm

        // Dictionary<int, int> angleMappingDict = new Dictionary<int, int> {
        //     {0, 160},{1, 115},{2, 65},{3, 20}, {120, 200}, {121, 245},{122, 295},{123, 340},
        //     {90, 160},{91, 115},{92, 65},{93, 20}f, {210, 200}, {211, 245},{212, 295},{213, 340},
        //      {30, 160},{31, 115},{32, 65},{33, 20}, {150, 200}, {151, 245},{152, 295},{153, 340},
        // };
        // Dictionary<int, int> angleMappingDict = new Dictionary<int, int> {
        //     {64, 160},{65, 115},{66, 65},{67, 20}, {120, 200}, {121, 245},{122, 295},{123, 340},
        //     {90, 160},{91, 115},{92, 65},{93, 20}, {210, 200}, {211, 245},{212, 295},{213, 340},
        //      {60, 200},{61, 245},{62, 295},{63, 340}, {150, 200}, {151, 245},{152, 295},{153, 340},
        // };


        //obstacle in Range mapping
        // int[] angleMapping =  Haptics_Obstacle ? new int[] {30,31,32,33,150,151,152,153}  : new int[] {};
        // int[] angleMapping =  Haptics_Obstacle ? new int[] {0,1,2,3,60,61,62,63}  : new int[] {};
        // int[] angleMapping =  Haptics_Obstacle ? new int[] {60,61,62,63,64,65,66,67}  : new int[] {};
        int[] angleMapping =  Haptics_Obstacle ? ObstacleAddrs : Array.Empty<int>();

        //drone crash mapping
        int[] crashMapping =  Haptics_Crash ? new int[] {4,5,124,125}  : new int[] {};
//        print("Crash Mapping: " + crashMapping.Length);
        
        
        //layers movement on arm mapping
        // int[] movingPlaneMapping =  Haptics_Network ? new int[] {60,61,62,63, 64, 65, 66, 67, 68, 69,
        //                                                             180,181, 182, 183, 184, 185, 186, 187, 188, 189}
        //                                                                 : new int[] {};
        // int[] movingPlaneMapping =  Haptics_Network ? new int[] {0,1,2,3, 4, 5, 6, 7, 8, 9,
        //                                                             30,31, 32, 33, 34, 35, 36, 37, 38, 39}
        //                                                                 : new int[] {};
        // 96,97,98,99,100,101,102,103,104,105 //48,49, 50,51,52,53,54,55,56,57 // 16,17, 18, 19, 20, 21, 22, 23, 24, 25

        // for (int i = 0; i < angleMapping.Length; i++)
        // {
        //     int adresse = angleMapping[i];
        //     int angle = angleMappingDict.ContainsKey(adresse) ? angleMappingDict[adresse] : 0; 
        //     actuatorsRange.Add(new PIDActuator(adresse:adresse, angle:angleMappingDict[adresse],
        //                                             kp:0f, kd:160, referencevalue:0, 
        //                                             refresh:CloseToWallrefresherFunction));
        // }

        // for (int i = 0; i < mappingOlfati.Length; i++)
        // {
        //     int adresse = mappingOlfati[i];
        //     actuatorsVariables.Add(new RefresherActuator(adresse:adresse, angle:angleMappingDict[adresse], refresh:ForceActuator));
        // }

        for (int i = 0; i < crashMapping.Length; i++)
        {
            int adresse = crashMapping[i];
            crashActuators.Add(new Actuators(adresse, 0));
        }

        // for (int i = 0; i < velocityMapping.Length; i++)
        // {
        //     int adresse = velocityMapping[i];
        //     actuatorsVariables.Add(new RefresherActuator(adresse:adresse, angle:angleMappingDict[adresse], refresh:SwarmVelocityRefresher));
        // }

        // for (int i = 0; i < movingPlaneMapping.Length; i++)
        // {
        //     int adresse = movingPlaneMapping[i];
        //     actuatorsMovingPlane.Add(new RefresherActuator(adresse:adresse, angle:adresse%10, refresh:movingPlaneRefresher));
        // }

        finalList.AddRange(actuatorsRange);
        finalList.AddRange(crashActuators);
        finalList.AddRange(actuatorNetwork);

        finalList.AddRange(actuatorsVariables);
        finalList.AddRange(actuatorsMovingPlane);

        if(hapticsCoroutine != null) {
            StopCoroutine(hapticsCoroutine);
        }


        hapticsCoroutine = StartCoroutine(HapticsCoroutine());

        currentGamepad = Gamepad.current;
        if (currentGamepad == null)
        {
            Debug.LogWarning("No gamepad connected.");
        }else {
            currentGamepad.SetMotorSpeeds(0.0f, 0.0f);
        }
    }

 
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
    public void VibrateController(float leftMotor, float rightMotor, float duration)
    {
        if (gamePadConnected == false)
        {
            currentGamepad = Gamepad.current;
            return;
        }

        if (gamnePadCoroutine != null)
        {
            StopCoroutine(gamnePadCoroutine);
        }
        gamnePadCoroutine = StartCoroutine(vibrateControllerForTime(leftMotor, rightMotor, duration));
    }

    public IEnumerator vibrateControllerForTime(float leftMotor, float rightMotor, float duration)
    {
        if(!Haptics_Controller)
        {
            yield break;
        }
        currentGamepad.SetMotorSpeeds(leftMotor, rightMotor);
        yield return new WaitForSeconds(duration);
        currentGamepad.SetMotorSpeeds(0, 0);
        gamnePadCoroutine = null;
    }


    public void HapticsPrediction(Prediction pred)
    {
        if (currentGamepad == null)
        {
            return;
        }

        if (pred.allData == null || pred.allData.Count == 0)
        {
            return; // Exit if no data to draw
        }

        //check is there is a crash
        float bestFractionOfPath = 2;
        foreach(DroneDataPrediction data in pred.allData) {
            if(data.idFirstCrash <= 0) {
                continue;
            }

            float fractionOfPath = 1-(float)data.idFirstCrash / data.positions.Count;
            if(fractionOfPath < bestFractionOfPath) {
                bestFractionOfPath = fractionOfPath;
            }
        }

        if(bestFractionOfPath < 1) {
            if(Haptics_Controller) {
                currentGamepad.SetMotorSpeeds(bestFractionOfPath, bestFractionOfPath);
            }
        }else {
            if(gamnePadCoroutine == null) {
                currentGamepad.SetMotorSpeeds(0, 0);
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

            HighlightClosestDrone(); // highlight the closest drone to the swarm centroid

            foreach (Actuators actuator in finalList)
            {
                actuator.update();
            }

            //  sendCommands();
            UpdateSwarmFrameAndLog();

            yield return new WaitForSeconds(sendEvery / 1000);
        }
    }

    void sendCommands()
    {

        //check if the actuators have the same adresse is so add the duty and keep highest frequency
        List<Actuators> finalListNoDouble = new List<Actuators>();
        foreach(Actuators actuator in finalList) {
            bool found = false;
            foreach(Actuators actuatorNoDouble in finalListNoDouble) {
                if(actuator.Adresse == actuatorNoDouble.Adresse) {
                    actuatorNoDouble.dutyIntensity += actuator.dutyIntensity;
                    actuatorNoDouble.frequency = Math.Max(actuator.frequency, actuatorNoDouble.frequency);
                    found = true;
                }
            }
            if(!found) {
                finalListNoDouble.Add(actuator);
            }
        }

        List<Actuators> toSendList = new List<Actuators>();
        foreach (Actuators actuator in finalListNoDouble)
        {
            bool found = false;
            foreach (Actuators last in lastDefined)
            {
                if(actuator.Adresse == last.Adresse) {
                    found = true;
                    if (!actuator.Equal(last))
                    {
                        toSendList.Add(actuator); //send the new data

                                                //check if it is a AnimatedActuator
                        if(actuator is AnimatedActuator) {
                            animationHandler(last.dutyIntensity, (AnimatedActuator)actuator);
                        }

                        last.dutyIntensity = actuator.dutyIntensity; // update the old data 
                        last.frequency = actuator.frequency;


                    }
                }
            }

            if(!found) {
                Actuators newActuator = new Actuators(actuator.Adresse, actuator.Angle);
                newActuator.dutyIntensity = actuator.dutyIntensity;
                newActuator.frequency = actuator.frequency;

                toSendList.Add(newActuator);
                lastDefined.Add(newActuator);
                if(actuator is AnimatedActuator) {
                    animationHandler(0,(AnimatedActuator)actuator);

                }
            }
        }
      //  print("FinalList: " + finalListNoDouble.Count + " toSendList: " + toSendList.Count + " lastDefined: " + lastDefined.Count);


        foreach(Actuators actuator in toSendList) {
            if(actuator is AnimatedActuator) {
                continue;
            }
            VibraForge.SendCommand(actuator.Adresse, (int)actuator.duty == 0 ? 0:1, (int)actuator.duty, (int)actuator.frequency);
        }
    }

    void animationHandler(int start, AnimatedActuator actuator)
    {
        if(animatedActuators.ContainsKey(actuator)) {
            StopCoroutine(animatedActuators[actuator]);
            actuator.stopAnimation();
        }

        actuator.defineAnimation(start, actuator.dutyIntensity);
        animatedActuators[actuator] = hapticAnimation(start, actuator);
        StartCoroutine(animatedActuators[actuator]);
    }


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
        List<Vector3> forces = swarmModel.swarmObstacleForces;

        actuator.dutyIntensity = 0;
        actuator.frequency = 1;


        foreach (Vector3 forcesDir in forces)
        {
            if (actuator.Angle >= 0)
            {
                float angle = Vector3.SignedAngle(forcesDir, CameraMovement.forward, -CameraMovement.up) - 180;
                if (angle < 0)
                {
                    angle += 360;
                }




                float diff = Math.Abs(actuator.Angle - angle);
                //   print("Diff: " + diff); 


                if (diff < 40 || diff > 320)
                {
                    //Debug.Log("forcesDir: " + forcesDir + " angle: " + angle + " diff: " + diff);
                    float threshold = forcesDir.magnitude > 3.5f ? 0.3f : 0.7f;
                    if (Vector3.Dot(MigrationPointController.alignementVector.normalized, -forcesDir.normalized) > threshold)
                    { // if col with velocity
                        actuator.UpdateValue(forcesDir.magnitude);
                        duty[actuator.Adresse] = actuator.dutyIntensity; // update the duty for visualization
                        //Debug.Log("duty: " + actuator.dutyIntensity + " actuator.Adresse" + actuator.Adresse);
                        return;
                    }

                    // if (CameraMovement.embodiedDrone != null)
                    // {
                    //     if (Vector3.Dot(CameraMovement.embodiedDrone.GetComponent<DroneController>().droneFake.velocity.normalized, -forcesDir.normalized) > threshold)
                    //     {
                    //         actuator.UpdateValue(forcesDir.magnitude);
                    //         duty[actuator.Adresse] = actuator.dutyIntensity; // update the duty for visualization
                    //         return;
                    //     }
                    // }

                    // actuator.UpdateValue(0);
                    // duty[actuator.Adresse] = 0; // update the duty for visualization
                    // return;
                }


            }
            else
            {
                //gte the y component
                float y = forcesDir.y;
                if (Mathf.Abs(y) > 0)
                {
                    actuator.UpdateValue(y);
                    duty[actuator.Adresse] = (int)(y / 14f); // update the duty for visualization
                    return;
                }
            }


        }

        // actuator.UpdateValue(0);
        // duty[actuator.Adresse] = 0; // update the duty for visualization

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


    #endregion
}

public class AnimatedActuator: RefresherActuator
{
    int animationEnd = 0;
    int animationStart = 0;

    public void defineAnimation(int start, int end)
    {
        animationStart = start;
        animationEnd = end;
    }

    public void stopAnimation()
    {
        VibraForge.SendCommand(Adresse, 0, 0, 1);
    }
    public AnimatedActuator(int adresse, float angle, updateFunction refresh) : base(adresse, angle, refresh)
    {
    }
}

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

    public void UpdateValue(float newValue)
    {
        float error = newValue - referenceValue;
        float derivative = newValue - lastValue;

        lastValue = newValue;
        dutyIntensity = Mathf.Max((int)(Kp * error + Kd * derivative), dutyIntensity);

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
            VibraForge.SendCommand(Adresse, (int)duty == 0 ? 0:1, (int)duty, (int)frequency);
            lastSendDuty = duty;
            lastSendFrequency = frequency;
      //      Debug.Log("Send Command: " + Adresse + " Duty: " + duty + " Frequency: " + frequency);
        }
    }


    public IEnumerator sendDelayedVal(float delay)
    {
        yield return new WaitForSeconds(delay);
        sendValue();

        yield return new WaitForSeconds(0.1f);
        HapticsTest.send = false;
    }


}


public class GamepadMonitor : MonoBehaviour
{
    private void OnEnable()
    {
        InputSystem.onDeviceChange += OnDeviceChange;
    }

    private void OnDisable()
    {
        InputSystem.onDeviceChange -= OnDeviceChange;
    }

    private void OnDeviceChange(InputDevice device, InputDeviceChange change)
    {
        if (device is Gamepad)
        {
            switch (change)
            {
                case InputDeviceChange.Added:
                    Debug.Log("Gamepad Connected: " + device.name);
                    break;
                case InputDeviceChange.Removed:
                    Debug.Log("Gamepad Disconnected!");
                    break;
            }
        }
    }
}