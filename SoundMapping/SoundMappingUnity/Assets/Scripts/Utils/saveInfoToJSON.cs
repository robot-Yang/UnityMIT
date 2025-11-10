using System.Collections;
using System.Collections.Generic;
using System.IO;
using JetBrains.Annotations;
using Unity.VisualScripting;
using UnityEditor.SearchService;
using UnityEngine;

public class saveInfoToJSON : MonoBehaviour
{
    public static SwarmState swarmData = new SwarmState();
    public int saveEvery = 100; // in ms
    float time = 0;

    public void Update()
    {
        time += Time.deltaTime;
        if (LevelConfiguration._SaveData)
        {
            if (time > saveEvery / 1000)
            {
                time = 0;
                saveDataPoint();
            }
        }
    }

    public void Start()
    {
        isSaving = false;
        swarmData = new SwarmState();
    }


    // Call this method whenever you want to record a new data point
    public static void saveDataPoint()
    {
        swarmData.saveDataPoint();
    }


    public static bool isSaving = false;
    
    public static void exportData(bool force)
    {
        // if(LevelConfiguration._SaveData)
        // {
            if (!isSaving)
            {
                isSaving = true;
                saveDataThread(force);
            }else{
                if(force)
                {
                    saveDataThread(force);
                }
            }
       // }
    }

    static void saveDataThread(bool force)
    {
        string PID = SceneSelectorScript.pid;
        bool haptics = SceneSelectorScript._haptics;
        bool order = SceneSelectorScript._order;
        int experimentNumber = SceneSelectorScript.experimentNumber;

        string nameScene = SceneSelectorScript.getNameScene();


        string forceString = force ? "dataForce_" : "";
        string hapticSring = haptics ? "H" : "NH";
        string orderString = order ? "O" : "NO";

        if(LevelConfiguration._SaveData)
        {

            //convert dataSave into JSON
            string json = JsonUtility.ToJson(swarmData, true);

        // string date = System.DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");
            string date = System.DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");
            string fileName =  forceString+nameScene+"_"+hapticSring+"_"+orderString+"_"+date+".json";

            //Create a folder with the name of the PID
            if (!System.IO.Directory.Exists("./Assets/Data/"+PID))
            {
                System.IO.Directory.CreateDirectory("./Assets/Data/"+PID);
            }
            

            System.IO.File.WriteAllText("./Assets/Data/"+PID+"/"+fileName, json);
        }

        // wait an extra 1s to make sure the file is written
        System.Threading.Thread.Sleep(100);

        if(!force)
        {
            SceneSelectorScript.nextScene();
        }

        //restart 
    }

    public static void addStarData(string starName, float timeCollected, int droneId, Vector3 position)
    {
        swarmData.addStar(starName, timeCollected, droneId, position);
    }

    public static void addHapticRecord(int id, int duty, int freq)
    {
        swarmData.addHapticRecord(id, duty, freq);
    }
}


[System.Serializable]
public class StarRecord
{
    public string starName;
    public float timeCollected;
    public int droneId;
    public Vector3 position;

    public StarRecord(string starName, float timeCollected, int droneId, Vector3 position)
    {
        this.starName = starName;
        this.timeCollected = timeCollected;
        this.droneId = droneId;
        this.position = position;
    }
}

public class HapticRecord
{
    public int i;
    public float t;

    public int d;
    public int f;

    public HapticRecord(int id, float time, int duty, int freq)
    {
        this.i = id;
        this.t = time;
        this.d = duty;
        this.f = freq;
    }
}

[System.Serializable]
public class SwarmState
{
    public List<DroneStateEntry> swarmState = new List<DroneStateEntry>();



    public List<Vector3> alignment = new List<Vector3>();
    public List<float> desiredSeparation = new List<float>();
    public List<float> time = new List<float>();
    public List<float> swarmConnectivness = new List<float>();
    public List<int> isolation = new List<int>();
    public List<int> idLeader = new List<int>();
    public List<int> subNetworkNumber = new List<int>();
    public List<Vector3> cameraForward = new List<Vector3>();




    //stars
    public List<StarRecord> stars = new List<StarRecord>();

    //haptics
    public List<HapticRecord> hapticRecords = new List<HapticRecord>();

    public int a = 1;


    // Constants
    public float maxSpeed = DroneFake.maxSpeed; 
    public float maxForce = DroneFake.maxForce;
    public float alpha = DroneFake.alpha;
    public float beta = DroneFake.beta;
    public float delta = DroneFake.delta;
    public float cvm = DroneFake.cVm;
    public float avoidanceRadius = DroneFake.avoidanceRadius;  
    public float desiredSeparationToObs = DroneFake.desiredSeparationObs;
    public float avoidanceForce = DroneFake.avoidanceForce;   
    public float droneRadius = DroneFake.droneRadius;
    public float dampingFactor = DroneFake.dampingFactor;
    public bool FPV = LevelConfiguration._startEmbodied;
    public string PID = SceneSelectorScript.pid;
    public bool haptics = SceneSelectorScript._haptics;
    public bool order = SceneSelectorScript._order;



    public void saveDataPoint()
    {
        // Example: gather your drones
        List<DroneFake> drones = swarmModel.drones;
        NetworkCreator network = swarmModel.network;

        subNetworkNumber.Add(network.adjacencyList.Count);

        foreach(DroneFake drone in drones)
        {
            // Find or create the DroneStateEntry for this drone
            DroneStateEntry entry = swarmState.Find(x => x.droneId == drone.idS);
            if (entry == null)
            {
                entry = new DroneStateEntry();
                entry.droneId = drone.idS;
                entry.droneState = new DroneState();
                swarmState.Add(entry);
            }

            // Append data to that DroneState
            List<DroneFake> net = new List<DroneFake>();
            if( network != null)
            {
                if( network.adjacencyList.ContainsKey(drone))
                {
                    net = network.adjacencyList[drone];
                }
            }
            entry.droneState.add(drone, net, MakePrediction.shortPred);
        }

        desiredSeparation.Add(DroneFake.desiredSeparation);
        alignment.Add(MigrationPointController.alignementVector);
        cameraForward.Add(CameraMovement.forward);
        time.Add(Timer.elapsedTime);
        swarmConnectivness.Add(swarmModel.swarmConnectionScore);
        isolation.Add(swarmModel.numberOfDroneDiscionnected);
        idLeader.Add(swarmModel.idLeader);

    }

    public void addStar(string starName, float timeCollected, int droneId, Vector3 position)
    {
        stars.Add(new StarRecord(starName, timeCollected, droneId, position));
    }

    public void addHapticRecord(int id, int duty, int freq)
    {
        hapticRecords.Add(new HapticRecord(id, Timer.elapsedTime, duty, freq));
    }

}

// This is our "key-value pair" entry
[System.Serializable]
public class DroneStateEntry
{
    public string droneId;
    public DroneState droneState;
}

[System.Serializable]
public class DroneState
{
    public List<Vector3> position = new List<Vector3>();
    public List<Vector3> velocity = new List<Vector3>();
    public List<Vector3> FobstacleAvoidance = new List<Vector3>();
    public List<Vector3> FolfatiSaber = new List<Vector3>();

    public List<Vector3> Falignment = new List<Vector3>();
    public List<string> network = new List<string>();
    public List<int> layer = new List<int>();

    public List<int> crashedPred = new List<int>();

    public void add(DroneFake drone, List<DroneFake> connected, Prediction pred)
    {

        position.Add(drone.position);
        velocity.Add(drone.velocity);
        FobstacleAvoidance.Add(drone.lastObstacle);
        FolfatiSaber.Add(drone.lastOlfati);
        Falignment.Add(drone.lastAllignement);
        layer.Add(drone.layer);

        addNetwork(connected);
        // addPredictrion(pred, drone);
    }

    private void addPredictrion(Prediction pred, DroneFake drone)
    {
        int indexPred = pred.dronesPrediction.IndexOf(drone);
        if (indexPred != -1)
        {
            crashedPred.Add(pred.allData[indexPred].idFirstCrash);
        }
        else
        {
            crashedPred.Add(-1);
        }
    }

    private void addNetwork(List<DroneFake> connected)
    {
        // Example
        string networkS = "";
        foreach (DroneFake d in connected)
        {
            networkS += d.id + "-";
        }
        network.Add(networkS);
    }
}


