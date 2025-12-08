using System;
using System.Collections.Generic;
using UnityEngine;

public class MigrationPointController : MonoBehaviour
{
    public static int idLeader = -1;
    public Camera mainCamera; // Assign your main camera in the Inspector
    public LayerMask groundLayer; // Layer mask for the ground
    public LayerMask droneLayer; // Layer mask for the drones
    public float spawnHeight = 10f; // Height at which drones operate
    public float radius = 1f; // Radius of the migration point

    public Vector2 migrationPoint = new Vector2(0, 0);

    public static GameObject selectedDrone = null;

    public Material normalMaterial;
    public Material selectedMaterial;

    public Vector3 deltaMigration = new Vector3(0, 0, 0); 
    public static Vector3 alignementVector = new Vector3(0, 0, 0);
    public static Vector3 alignementVectorNonZero = new Vector3(0, 0, 0);

    public static float maxSpreadness = 5f;
    public static float minSpreadness = 1f;

    public static bool InControl = true;

    bool firstTime = true;

    [Header("Swarm Heading")]
    public bool useGlobalHeading = true;
    public float headingSmooth = 5f; // higher = faster follow
    private Vector3 _swarmHeading = Vector3.forward; // world-space, horizontal
    private bool _headingInitialized = false;

    // === Auto-frontmost switching (config) ===
    [Header("Auto Frontmost Switching")]
    public bool autoSwitchFrontmost = true;   // turn on/off
    public float checkInterval = 0.15f;       // seconds between checks
    public float maxViewAngleDeg = 360f;       // view cone half-angle
    public float minForwardDist = 0.01f;       // must be at least this far ahead
    public float minHoldTime = 0.5f;          // must stay best for this long before switching
    public float minSwitchCooldown = 1.0f;    // cooldown after a switch to avoid flip-flop

    // === Auto-frontmost switching (runtime state) ===
    float _nextCheckTime = 0f;
    GameObject _candidateFrontmost = null;
    float _candidateSinceTime = 0f;
    float _lastSwitchTime = -999f;

    [Header("Group Filtering")]
    public bool restrictToMainGroup = true;   // only switch within the largest connected subnetwork
    public float groupRefreshInterval = 0.5f; // how often to recompute the main group (s)
    private HashSet<DroneFake> _mainGroup = null;
    private float _nextGroupRefreshTime = 0f;

    [Header("Embodiment Input Gates")]
    public bool allowManualSelection = false;    // disables selection by buttons 4/5 when false
    public bool allowManualEmbodiment = false;   // disables button 0 embodiment when false

    [Header("Frontmost Sensitivity")]
    [Tooltip("Candidate must be at least this much farther forward than the current embodied drone (in meters).")]
    public float frontLeadThreshold = 0.5f;

    [Tooltip("Reject candidates farther than this lateral distance from the viewing axis (in meters).")]
    public float maxCandidateLateral = 0.0f;

    // GameObject FindFrontmostDroneInView(Transform reference,
    //                                     float maxAngleDeg,
    //                                     float minFwd,
    //                                     HashSet<DroneFake> allowedGroup = null)   // <— NEW
    // {
    //     if (reference == null || swarmModel.swarmHolder == null) return null;

    //     Vector3 refPos = reference.position;
    //     Vector3 fwd    = reference.forward;

    //     GameObject best = null;
    //     float bestForward = -Mathf.Infinity;
    //     float bestLateral = Mathf.Infinity;

    //     foreach (Transform t in swarmModel.swarmHolder.transform)
    //     {
    //         GameObject go = t.gameObject;

    //         // skip self
    //         if (CameraMovement.embodiedDrone != null && go == CameraMovement.embodiedDrone)
    //             continue;

    //         // NEW: filter out drones not in the main group
    //         if (allowedGroup != null)
    //         {
    //             var dc = go.GetComponent<DroneController>();
    //             if (dc == null || dc.droneFake == null || !allowedGroup.Contains(dc.droneFake))
    //                 continue;
    //         }

    //         Vector3 diff = t.position - refPos;
    //         float forwardDist = Vector3.Dot(diff, fwd);
    //         if (forwardDist < minFwd) continue;

    //         float angle = Vector3.Angle(fwd, diff);
    //         if (angle > maxAngleDeg) continue;

    //         Vector3 lateral = diff - fwd * forwardDist;
    //         float lateralMag = lateral.magnitude;

    //         bool better =
    //             forwardDist > bestForward ||
    //             (Mathf.Approximately(forwardDist, bestForward) && lateralMag < bestLateral);

    //         if (better)
    //         {
    //             best = go;
    //             bestForward = forwardDist;
    //             bestLateral = lateralMag;
    //         }
    //     }
    //     return best;
    // }

    GameObject FindFrontmostDroneInView(Transform reference,
                                        float maxAngleDeg,
                                        float minFwd,
                                        HashSet<DroneFake> allowedGroup = null)
    {
        if (reference == null || swarmModel.swarmHolder == null) return null;

        Vector3 refPos = reference.position;
        Vector3 fwd    = reference.forward;

        // Baseline = current embodied drone's forward distance (if any)
        float baselineForward = float.NegativeInfinity;
        if (CameraMovement.embodiedDrone != null)
        {
            Vector3 curDiff = CameraMovement.embodiedDrone.transform.position - refPos;
            baselineForward = Vector3.Dot(curDiff, fwd);
        }

        GameObject best = null;
        float bestScore = float.NegativeInfinity;

        foreach (Transform t in swarmModel.swarmHolder.transform)
        {
            GameObject go = t.gameObject;

            // Skip the currently embodied drone
            if (CameraMovement.embodiedDrone != null && go == CameraMovement.embodiedDrone)
                continue;

            // Filter by main group if provided
            if (allowedGroup != null)
            {
                var dc = go.GetComponent<DroneController>();
                if (dc == null || dc.droneFake == null || !allowedGroup.Contains(dc.droneFake))
                    continue;
            }

            // Geometry relative to reference (embodied or camera)
            Vector3 diff        = t.position - refPos;
            float   forwardDist = Vector3.Dot(diff, fwd);
            if (forwardDist < minFwd) continue;

            float angle = Vector3.Angle(fwd, diff);
            if (angle > maxAngleDeg) continue;

            Vector3 lateral    = diff - fwd * forwardDist;
            float   lateralMag = lateral.magnitude;

            // --- Stability gates ---
            // 1) Off-axis rejection (keep candidates close to the view axis)
            if (maxCandidateLateral > 0f && lateralMag > maxCandidateLateral)
                continue;

            // 2) Must lead the current embodied by a clear margin
            if (!float.IsNegativeInfinity(baselineForward) &&
                (forwardDist - baselineForward) < frontLeadThreshold)
                continue;

            // Score: prefer more forward; tie-break by smaller lateral
            float score = forwardDist - 0.1f * lateralMag; // small penalty for off-axis
            if (score > bestScore)
            {
                bestScore = score;
                best      = go;
            }
        }

        return best;
    }

    // void RefreshMainGroupIfNeeded()
    // {
    //     if (!restrictToMainGroup) return;
    //     if (Time.time < _nextGroupRefreshTime) return;
    //     _nextGroupRefreshTime = Time.time + groupRefreshInterval;

    //     _mainGroup = null;

    //     var subnetworks = swarmModel.network.GetSubnetworks(); // you already use this elsewhere
    //     if (subnetworks == null || subnetworks.Count == 0) return;

    //     // pick the largest component as the "main swarm group"
    //     int maxCount = -1;
    //     foreach (var sub in subnetworks)
    //     {
    //         if (sub != null && sub.Count > maxCount)
    //         {
    //             maxCount = sub.Count;
    //             _mainGroup = sub;
    //         }
    //     }
    // }

    void RefreshMainGroupIfNeeded()
    {
        if (!restrictToMainGroup) return;
        if (Time.time < _nextGroupRefreshTime) return;
        _nextGroupRefreshTime = Time.time + groupRefreshInterval;

        _mainGroup = null;

        // Get all subnetworks from the existing swarmModel.network
        var subnetworks = swarmModel.network.GetSubnetworks();
        if (subnetworks == null || subnetworks.Count == 0) return;

        // Identify the seed: the currently embodied drone's DroneFake
        var embodiedGO = CameraMovement.embodiedDrone;
        if (embodiedGO == null) return;

        var embodiedDC = embodiedGO.GetComponent<DroneController>();
        if (embodiedDC == null || embodiedDC.droneFake == null) return;

        var embodiedFake = embodiedDC.droneFake;

        // Pick the subnetwork that contains the embodied drone
        foreach (var sub in subnetworks)
        {
            if (sub != null && sub.Contains(embodiedFake))
            {
                _mainGroup = sub;
                break;
            }
        }

        // Optional fallback: if somehow not found (e.g., transient), keep the largest set
        if (_mainGroup == null)
        {
            int maxCount = -1;
            foreach (var sub in subnetworks)
            {
                if (sub != null && sub.Count > maxCount)
                {
                    maxCount = sub.Count;
                    _mainGroup = sub;
                }
            }
        }
    }

    void UpdateSwarmHeading(float dt)
    {
        if (!useGlobalHeading) return;

        // Choose a source direction (embodied if present, else camera)
        Transform refTf = CameraMovement.embodiedDrone != null
            ? CameraMovement.embodiedDrone.transform
            : CameraMovement.cam.transform;

        Vector3 desired = Vector3.ProjectOnPlane(refTf.forward, Vector3.up).normalized;
        if (desired.sqrMagnitude < 1e-6f) return;

        if (!_headingInitialized)
        {
            _swarmHeading = desired;
            _headingInitialized = true;
        }
        else
        {
            // Smoothly follow (exp smoothing, frame-rate independent)
            float a = 1f - Mathf.Exp(-headingSmooth * dt);
            _swarmHeading = Vector3.Slerp(_swarmHeading, desired, a).normalized;
        }
    }

    public bool control_movement
    {
        get{
            return LevelConfiguration._control_movement;
        }
    }
    public bool control_spreadness
    {
        get{
            return LevelConfiguration._control_spreadness;
        }
    }
    public bool control_embodiement
    {
        get{
            return LevelConfiguration._control_embodiement;
        }
    }

    public bool _control_desembodiement
    {
        get{
            return LevelConfiguration._control_desembodiement;
        }
    }
    public bool control_selection
    {
        get
        {
            return LevelConfiguration._control_selection;
        }
    }
    public bool control_rotation
    {
        get
        {
            return LevelConfiguration._control_rotation;
        }
    }

    void AutoSwitchToFrontmost()
    {
        if (!autoSwitchFrontmost) return;
        if (CameraMovement.embodiedDrone == null) return;

        // NEW: keep the main-group cache fresh
        RefreshMainGroupIfNeeded();

        if (Time.time < _nextCheckTime) return;
        _nextCheckTime = Time.time + checkInterval;

        Transform refTf = CameraMovement.embodiedDrone.transform;
        GameObject currentBest = FindFrontmostDroneInView(
            refTf, maxViewAngleDeg, minForwardDist,
            restrictToMainGroup ? _mainGroup : null); // <— pass group filter

        if (currentBest == null)
        {
            _candidateFrontmost = null;
            return;
        }

        if (_candidateFrontmost != currentBest)
        {
            _candidateFrontmost = currentBest;
            _candidateSinceTime = Time.time;
            return;
        }

        bool heldLongEnough = (Time.time - _candidateSinceTime) >= minHoldTime;
        bool cooldownDone   = (Time.time - _lastSwitchTime) >= minSwitchCooldown;

        if (heldLongEnough && cooldownDone)
        {
            if (_candidateFrontmost != CameraMovement.embodiedDrone)
            {
                CameraMovement.nextEmbodiedDrone = _candidateFrontmost;
                _lastSwitchTime = Time.time;
                Debug.Log($"[AutoSwitch] Next embodied (frontmost/main-group): " +
                        _candidateFrontmost.GetComponent<DroneController>().droneFake.id);
            }
            _candidateSinceTime = Time.time;
        }
    }



    [Header("Align headings to embodied drone")]
    public bool alignHeadingToEmbodied = true; // toggle on/off
    public bool yawOnly = true;                // copy only yaw (recommended)
    public float alignSlerpSpeed = 8f;         // higher = snappier
    void AlignOthersToEmbodiedHeading(float dt)
    {
        if (!alignHeadingToEmbodied) return;
        if (swarmModel.swarmHolder == null) return;
        if (!_headingInitialized) return;

        Quaternion targetRot = Quaternion.LookRotation(_swarmHeading, Vector3.up);
        float a = 1f - Mathf.Exp(-alignSlerpSpeed * dt);

        Transform embodied = CameraMovement.embodiedDrone != null
            ? CameraMovement.embodiedDrone.transform
            : null;

        foreach (Transform t in swarmModel.swarmHolder.transform)
        {
            if (embodied != null && t == embodied) continue; // never rotate the embodied
            if (yawOnly)
            {
                Vector3 tfwdYaw = Vector3.ProjectOnPlane(t.forward, Vector3.up).normalized;
                if (tfwdYaw.sqrMagnitude < 1e-6f) tfwdYaw = t.forward;
                Quaternion currentYawRot = Quaternion.LookRotation(tfwdYaw, Vector3.up);
                t.rotation = Quaternion.Slerp(currentYawRot, targetRot, a);
            }
            else
            {
                t.rotation = Quaternion.Slerp(t.rotation, targetRot, a);
            }
        }
    }


    void Update()
    {
        if(!InControl)
        {
            return;
        }
        
        UpdateMigrationPoint();
        SelectionUpdate();  
        SpreadnessUpdate();

        // AlignOthersToEmbodiedHeading(Time.deltaTime); // <— add this
        // NEW: automatic handoff to frontmost drone
        AutoSwitchToFrontmost();

        UpdateSwarmHeading(Time.deltaTime);
        AlignOthersToEmbodiedHeading(Time.deltaTime);

    }

    void SelectionUpdate()
    { 
        if(Input.GetKeyDown("joystick button " + 3))
        {
            
            swarmModel.dummyForcesApplied = !swarmModel.dummyForcesApplied;
        }       

        // if((Input.GetKeyDown("joystick button " + 5) || Input.GetKeyDown("joystick button " + 4)) && control_selection) //selection
        if (allowManualSelection && (Input.GetKeyDown("joystick button " + 5) || Input.GetKeyDown("joystick button " + 4)) && control_selection) // selection
        {
            if(selectedDrone == null && CameraMovement.embodiedDrone == null) // if nothing selected
            {
                if(swarmModel.swarmHolder.transform.childCount > 0)
                {
                    if(LevelConfiguration._startEmbodied)
                    {
                        selectedDrone = swarmModel.swarmHolder.transform.GetChild(0).gameObject;
                        idLeader = selectedDrone.GetComponent<DroneController>().droneFake.id;
                    }
                }
            }
            else
            {
                if(CameraMovement.embodiedDrone != null)
                {
                    Dictionary<GameObject, float> scores = new Dictionary<GameObject, float>();

                    foreach(Transform drone in swarmModel.swarmHolder.transform)
                    {
                        if(drone.gameObject == CameraMovement.embodiedDrone)
                        {
                            continue;
                        }


                        Vector3 diff = drone.position - CameraMovement.embodiedDrone.transform.position;
                        float score = Vector3.Dot(diff, CameraMovement.embodiedDrone.transform.forward);

                        if(score > 0.5)
                        {
                            score /= diff.magnitude;
                            scores.Add(drone.gameObject, score);
                        }

                    }

                    //select the highest score
                    if(scores.Count > 0)
                    {
                        //sort the dictionary
                        List<KeyValuePair<GameObject, float>> sortedScores = new List<KeyValuePair<GameObject, float>>(scores);
                        sortedScores.Sort((x, y) => y.Value.CompareTo(x.Value));

                        //select the highest score
                        CameraMovement.nextEmbodiedDrone = sortedScores[0].Key;
                        print("Next Selected drone: " + CameraMovement.nextEmbodiedDrone.GetComponent<DroneController>().droneFake.id);
                    }
                }
                else
                {
                    int increment = Input.GetKeyDown("joystick button " + 5) ? 1 : -1;
                    if(increment == 0)
                    {
                        return;
                    }

                    List<HashSet<DroneFake>> subnetwork = swarmModel.network.GetSubnetworks();
                   // print("Number of subnetworks: " + subnetwork.Count);
                    //compute average position of each subnetwork
                    Dictionary<HashSet<DroneFake>, Vector3> averagePositions = new Dictionary<HashSet<DroneFake>, Vector3>();
                    foreach(HashSet<DroneFake> sub in subnetwork)
                    {
                        Vector3 averagePosition = new Vector3(0, 0, 0);
                        foreach(DroneFake drone in sub)
                        {
                            averagePosition += drone.position;
                        }
                        averagePosition /= sub.Count;
                        averagePositions.Add(sub, averagePosition);
                    }

                    //find the subnetwork of the selected drone
                    HashSet<DroneFake> selectedSubnetwork = null;
                    foreach(HashSet<DroneFake> sub in subnetwork)
                    {
                        if(sub.Contains(selectedDrone.GetComponent<DroneController>().droneFake)){
                            selectedSubnetwork = sub;
                            break;
                        }
                    }

                    //order the subnetworks by the position.z of their average position
                    List<KeyValuePair<HashSet<DroneFake>, Vector3>> sortedSubnetworks = new List<KeyValuePair<HashSet<DroneFake>, Vector3>>(averagePositions);
                    sortedSubnetworks.Sort((x, y) => y.Value.z.CompareTo(x.Value.z));

                    //find the index of the selected subnetwork
                    int selectedSubnetworkIndex = -1;
                    for(int i = 0; i < sortedSubnetworks.Count; i++)
                    {
                        if(sortedSubnetworks[i].Key == selectedSubnetwork)
                        {
                            selectedSubnetworkIndex = i;
                            break;
                        }
                    }
//                    print("Selected subnetwork index: " + selectedSubnetworkIndex);

                    //select the next subnetwork
                    int nextSubnetworkIndex = (selectedSubnetworkIndex + increment) % sortedSubnetworks.Count;
             //       print("Next subnetwork index: " + nextSubnetworkIndex);
                    if(nextSubnetworkIndex < 0)
                    {
                        nextSubnetworkIndex = sortedSubnetworks.Count - 1;
                    }

                    HashSet<DroneFake> nextSubnetwork = sortedSubnetworks[nextSubnetworkIndex].Key;

                    //get the first droneFake in the subnetwork
                    DroneFake nextDrone = null;
                    foreach (DroneFake drone in nextSubnetwork)
                    {
                        nextDrone = drone;
                        break;
                    }

                    if(nextDrone == null)
                    {
               //         print("No drone in the subnetwork");
                        return;
                    }

           //         print("Next drone: " + nextDrone.id);

                    //select the drone
                    foreach(Transform drone in swarmModel.swarmHolder.transform)
                    {
                        if(drone.gameObject.GetComponent<DroneController>().droneFake == nextDrone)
                        {
                            selectedDrone = drone.gameObject;
                            idLeader = nextDrone.id;
                            break;
                        }
                    }
                }
            }

            // this.GetComponent<HapticsTest>().VibrateController(0.3f, 0.3f, 0.2f); // selection vibration
        }

        // button 0
        // if(Input.GetKeyDown("joystick button " + 0) && control_embodiement) //embodiement
        if (allowManualEmbodiment && Input.GetKeyDown("joystick button " + 0) && control_embodiement) // embodiment
        {

            if(CameraMovement.embodiedDrone != null)
            {
                if(selectedDrone != CameraMovement.embodiedDrone)//drone 2 drone
                {
                    CameraMovement.nextEmbodiedDrone = selectedDrone; // set next selected drone diff to null to trigger animation to the other drone
                }
            }
            else if(selectedDrone != null)
            {
                CameraMovement.SetEmbodiedDrone(selectedDrone);
            }
        }

        if(Input.GetKeyDown("joystick button " + 1) && _control_desembodiement) //desembodie
        {
            if(CameraMovement.embodiedDrone != null)
            {
                CameraMovement.embodiedDrone.GetComponent<Camera>().enabled = false;                
                CameraMovement.DesembodiedDrone(CameraMovement.embodiedDrone); 
            }
        }
    }

    void SpreadnessUpdate()
    {
        float spreadness = Input.GetAxis("LR");
        if(spreadness != 0 && control_spreadness)
        {
            swarmModel.desiredSeparation+= spreadness * Time.deltaTime * 1.3f;
            swarmModel.desiredSeparation = Mathf.Clamp(swarmModel.desiredSeparation, minSpreadness, maxSpreadness);
        }
    }

    void UpdateMigrationPoint()
    {
        if(!control_movement)
        {
            return;
        }


        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        float heightControl = Input.GetAxis("JoystickRightVertical");
        Transform body = null;
        Vector3 right = new Vector3(0, 0, 0);
        Vector3 forward = new Vector3(0, 0, 0);
        Vector3 up = new Vector3(0, 0, 0);

        Vector3 final = new Vector3(0, 0, 0);

        if(CameraMovement.embodiedDrone == null)
        {
            body = CameraMovement.cam.transform;
            right = body.right;
            forward = body.up;
            up = -body.forward;

        }else{
            body = CameraMovement.embodiedDrone.transform;
            right = body.right;
            forward = body.forward;
            up = body.up;
        }

        CameraMovement.forward = forward;
        CameraMovement.right = right;
        CameraMovement.up = up;

        if(horizontal == 0 && vertical == 0 && heightControl == 0)
        {
            if(firstTime)
            {
                migrationPoint = new Vector2(body.position.x, body.position.z);
                firstTime = false;
            }
            //migrationPoint = new Vector2(body.position.x, body.position.z);
            deltaMigration = new Vector3(0, 0, 0);
        }else{
            firstTime = true;
            Vector3 centerOfSwarm = body.position;
            final = vertical * forward + horizontal * right + heightControl * up;
            final.Normalize();

            float newR = Mathf.Sqrt(horizontal * horizontal + vertical * vertical + heightControl * heightControl);
            Vector3 finalAlignement = final * newR * radius;

            final = final * radius;


            migrationPoint = new Vector2(centerOfSwarm.x + final.x, centerOfSwarm.z + final.z);
            deltaMigration = new Vector3(finalAlignement.x, finalAlignement.y, finalAlignement.z);
        }
        if( deltaMigration.magnitude > 0.1f)
        {
            alignementVectorNonZero = deltaMigration;
        }
        
        alignementVector = deltaMigration;

        Debug.DrawRay(body.position, alignementVector, Color.red, 0.01f);
    }

}