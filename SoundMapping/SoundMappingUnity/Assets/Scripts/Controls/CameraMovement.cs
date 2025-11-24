using System.Collections.Generic;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{
    public static int idLeader = -1;


    public static Vector3 forward;
    public static Vector3 right;
    public static Vector3 up;


    public static Camera cam;
    public GameObject camMinimap;
    public GameObject minimap;
    public Transform swarmHolder;
    public float heightCamera = 20f;
    public float rotationSpeed = 80f;
    public const float animationTime = 0.1f;
    public static GameObject embodiedDrone = null;
    public static GameObject nextEmbodiedDrone = null;
    public GameObject lastEmbodiedDrone = null;
    public Quaternion initialCamRotation;
    private const float DEFAULT_HEIGHT_CAMERA = 20f;

    // A simple state machine for the camera
    private enum CameraState { TDView, Animation, DroneView, Crash, AnimationDroneToDrone };
    private static CameraState currentState;

    // Animation variables (used when transitioning between states)
    private static float animTimer = 0f;
    private Vector3 animStartPos;
    private Vector3 animTargetPos;

    

    void Start()
    {
        _this = gameObject;


        cam = Camera.main;
        cam.transform.position = new Vector3(0, DEFAULT_HEIGHT_CAMERA, 0);
        heightCamera = DEFAULT_HEIGHT_CAMERA;
        initialCamRotation = cam.transform.rotation;

        // Begin in top-down view mode
        currentState = CameraState.TDView;
    }

    void Update()
    {

      //  print("CameraMovement: " + idLeader + " Selected: " + MigrationPointController.idLeader);

        // Run main loop based on current state.
        switch (currentState)
        {
            case CameraState.TDView:
                UpdateTDView();
                // If a drone becomes embodied, start the animation to switch view.
                if (embodiedDrone != null)
                {
                    BeginAnimation(embodiedDrone.transform.position);
                    currentState = CameraState.Animation;
                }
                break;

            case CameraState.Animation:
                UpdateAnimation();
                break;

            case CameraState.AnimationDroneToDrone:
                AnimationDroneToDroneFunc();
                break;

            case CameraState.DroneView:
                UpdateDroneView();
                // If a new drone has been chosen, initiate the transition.
                if (nextEmbodiedDrone != null)
                {
                    startFOV = embodiedDrone.GetComponent<Camera>().fieldOfView;
                    BeginAnimation(embodiedDrone.transform.position);
                    currentState = CameraState.AnimationDroneToDrone;
                }
                break;

            case CameraState.Crash:
                HandleCrash();
                break;
        }
    }

    //setup of the crash animation

    // TDView mode: Camera follows a “center of mass” of the drones.
    void UpdateTDView()
    {
        // Update the camera position to center on all drones.
        List<DroneFake> drones = swarmModel.dronesInMainNetwork; // Assumes swarmModel exists.
        if (drones.Count > 0)
        {
            Vector3 center = Vector3.zero;
            foreach (DroneFake drone in drones)
            {
                center += drone.position;
            }
            center /= drones.Count;
            center.y = heightCamera;
            cam.transform.position = Vector3.Lerp(cam.transform.position, center, Time.deltaTime * 2f);
        }

        // Rotate camera using joystick input (if rotation is enabled)
        float rightStickHorizontal = (GetComponent<MigrationPointController>().control_rotation) ? Input.GetAxis("JoystickRightHorizontal") : 0;
        cam.transform.Rotate(-Vector3.forward, rightStickHorizontal * Time.deltaTime * rotationSpeed);

        // Adjust orthographic size based on swarm separation.
        float targetSize = Mathf.Max(swarmModel.desiredSeparation * 3, 6);
        cam.GetComponent<Camera>().orthographicSize = Mathf.Lerp(cam.GetComponent<Camera>().orthographicSize, targetSize, Time.deltaTime * 2f);
    }

    // DroneView mode: Camera follows the embodied drone.
    void UpdateDroneView()
    {
        // Rotate the drone with joystick input.
        float rightStickHorizontal = (GetComponent<MigrationPointController>().control_rotation) ? Input.GetAxis("JoystickRightHorizontal") : 0;
        if (embodiedDrone != null)
        {
            embodiedDrone.transform.Rotate(Vector3.up, rightStickHorizontal * Time.deltaTime * rotationSpeed);
            // The camera follows the drone at the set height.
            cam.transform.position = new Vector3(embodiedDrone.transform.position.x, heightCamera, embodiedDrone.transform.position.z);
        }

        // Update minimap camera if enabled.
        camMinimap.GetComponent<Camera>().orthographicSize = swarmModel.desiredSeparation * 3;
    }

    // Begins an animation from the current camera position to a target position.
    void BeginAnimation(Vector3 targetPos)
    {
        animTimer = 0f;
        animStartPos = cam.transform.position;
        animTargetPos = targetPos;
    }

    // Animation update: smoothly moves the camera from animStartPos to animTargetPos.
    void UpdateAnimation()
    {
        animTimer += Time.deltaTime;
        float t = Mathf.Clamp01(animTimer / animationTime);
        cam.transform.position = Vector3.Lerp(animStartPos, animTargetPos, t);
        cam.GetComponent<Camera>().orthographicSize = Mathf.Lerp(cam.GetComponent<Camera>().orthographicSize, 15, t);

        // When the animation completes, update the embodied drone’s forward direction and switch to DroneView.
        if (t >= 1f)
        {
            if (embodiedDrone != null)
            {
                // Vector3 forwardDrone = cam.transform.up;
                // forwardDrone.y = 0;
                // embodiedDrone.transform.forward = forwardDrone;

                // activate the camera 
                embodiedDrone.GetComponent<Camera>().enabled = true;
               // MigrationPointController.selectedDrone = embodiedDrone;
                cam.enabled = true; //false;
            }else
            {
                cam.enabled = true;
            }
            currentState = CameraState.DroneView;
        }
    }

    private float startFOV;

    // void AnimationDroneToDroneFunc()
    // {
    //     animTimer += Time.deltaTime;
    //     float t = Mathf.Clamp01(animTimer / animationTime);
    //    // print("t: " + t);
    //     embodiedDrone.transform.LookAt(nextEmbodiedDrone.transform);
    //     embodiedDrone.GetComponent<Camera>().fieldOfView = Mathf.Lerp(embodiedDrone.GetComponent<Camera>().fieldOfView, 20, t);

    //     if (t >= 1f)
    //     {
    //         embodiedDrone.GetComponent<Camera>().fieldOfView = startFOV;
    //         embodiedDrone.GetComponent<Camera>().enabled = false;
    //         Vector3 forwardDrone = embodiedDrone.transform.forward;
    //         forwardDrone.y = 0;
    //         SetEmbodiedDrone(nextEmbodiedDrone);
    //         embodiedDrone.GetComponent<Camera>().enabled = true;
    //         embodiedDrone.transform.forward = forwardDrone;
    //         currentState = CameraState.DroneView;
    //     }
    // }

    void AnimationDroneToDroneFunc()
    {
        animTimer += Time.deltaTime;
        float t = Mathf.Clamp01(animTimer / animationTime);

        // Keep the FOV zoom if you like:
        embodiedDrone.GetComponent<Camera>().fieldOfView =
            Mathf.Lerp(embodiedDrone.GetComponent<Camera>().fieldOfView, 20, t);

        if (t >= 1f)
        {
            embodiedDrone.GetComponent<Camera>().fieldOfView = startFOV;
            embodiedDrone.GetComponent<Camera>().enabled = false;

            SetEmbodiedDrone(nextEmbodiedDrone);
            embodiedDrone.GetComponent<Camera>().enabled = true;

            currentState = CameraState.DroneView;
        }
    }

    // Crash handling: reposition the camera and log the crash.
    private static float startAvoidanceForce;
    public static void crashAnimationSetup()
    {
        animTimer = 0f;
        if(embodiedDrone != null)
        {
            cam.enabled = false;
            embodiedDrone.GetComponent<Camera>().enabled = false;

            DesembodiedDrone(embodiedDrone);
        }

        nextEmbodiedDrone = GetEmbodiedDrone();
        nextEmbodiedDrone.GetComponent<Camera>().enabled = true;

        startAvoidanceForce = DroneFake.avoidanceForce;
        DroneFake.avoidanceForce = 200f;


        currentState = CameraState.Crash;
    }


    void HandleCrash() // crash animation
    {
        MigrationPointController.InControl = false;
        MigrationPointController.alignementVector = Vector3.zero;
        animTimer += Time.deltaTime;
        float t = Mathf.Clamp01(animTimer / 2f);
        textInfo.setDeathImageStatic(t);

        Debug.Log(t);

        
        if (t >= 1f)
        {
            DroneFake.avoidanceForce = startAvoidanceForce;
            MigrationPointController.InControl = true;
            if(nextEmbodiedDrone.activeSelf)
            {
                nextEmbodiedDrone.GetComponent<Camera>().enabled = true;
                SetEmbodiedDrone(nextEmbodiedDrone);
                currentState = CameraState.DroneView;
            }else
            {
                crashAnimationSetup();
                currentState = CameraState.DroneView;
            }
        }
    }

    // Returns an available drone from the swarm that hasn’t crashed.
    public static GameObject GetEmbodiedDrone()
    {
        Transform swarmHolder = swarmModel.swarmHolder.transform;
        nextEmbodiedDrone = null;

        if (swarmHolder.childCount > 0)
        {
            List<int> indices = new List<int>();
            for (int i = 0; i < swarmHolder.childCount; i++)
            {
                indices.Add(i);
            }
            while (indices.Count > 0)
            {
                int randomListIndex = UnityEngine.Random.Range(0, indices.Count);
                int droneIndex = indices[randomListIndex];
                indices.RemoveAt(randomListIndex);

                GameObject drone = swarmHolder.GetChild(droneIndex).gameObject;
                DroneController droneController = drone.GetComponent<DroneController>();
                if (droneController != null && !droneController.droneFake.hasCrashed)
                {
                    return drone;
                }
            }
        }
        return null;
    }

    // Sets the next drone to be embodied; the Update loop will transition to it.
    public static void SetNextEmbodiedDrone()
    {
        GameObject drone = GetEmbodiedDrone();
        if (drone != null)
        {
            SetEmbodiedDrone(drone);
        }else
        {
           swarmModel.restart();
        }
    }

    // Immediately set the given drone as the embodied drone.
    public static void SetEmbodiedDrone(GameObject drone)
    {
        // if(embodiedDrone != drone)
        // {
        //     swarmModel.drones.Find(x => x.id == embodiedDrone.GetComponent<DroneController>().droneFake.id).embodied = false;
        //     swarmModel.drones.Find(x => x.id == embodiedDrone.GetComponent<DroneController>().droneFake.id).embodied = false;
        // }
        
        embodiedDrone = drone;
        idLeader = drone.GetComponent<DroneController>().droneFake.id;
        DroneController controller = drone.GetComponent<DroneController>();
        if (controller != null)
        {
            controller.droneFake.embodied = true;
            controller.droneFake.selected = true;
        }
        swarmModel.drones.Find(x => x.id == drone.GetComponent<DroneController>().droneFake.id).embodied = true;
        swarmModel.drones.Find(x => x.id == drone.GetComponent<DroneController>().droneFake.id).selected = false;
        nextEmbodiedDrone = null;
    }

    public static Vector3 getCameraPosition()
    {
        return cam.transform.position;
    }

    public static void DesembodiedDrone(GameObject drone)
    {
        if (drone == embodiedDrone)
        {
            embodiedDrone = null;
        }
    }


    public static GameObject _this; 
    public static void setNextEmbodiedDrone()
    {
    }

}
