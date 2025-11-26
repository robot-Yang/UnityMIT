using System;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using Unity.VisualScripting.Dependencies.Sqlite;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.InputSystem.Interactions;
using UnityEngine.Scripting;

public class MakePrediction : MonoBehaviour
{
    Material defaultMaterial;
    public Transform allPredictionsHolder;
    public static Prediction shortPred;


    public Transform longPredictionLineHolder, shortPredictionLineHolder;

    Thread predictionThread;    


    void Start()
    {
        defaultMaterial = new Material(Shader.Find("Unlit/Color"));

        shortPred = new Prediction(true, 20, 2, 0, shortPredictionLineHolder);

        launchPreditionThread(shortPred);
    }

    void StartPrediction(Prediction pred)
    {
        Vector3 alignementVector = pred.alignementVector;   

        for(int i = 0; i < pred.deep; i++)
        {
            NetworkCreator network = new NetworkCreator(pred.dronesPrediction);
            network.refreshNetwork(idLeader: pred.idLeader);
            foreach (DroneFake drone in pred.dronesPrediction)
            {
                drone.startPrediction(alignementVector, network);
            }


            for(int j = 0; j < pred.dronesPrediction.Count; j++)
            {
                pred.dronesPrediction[j].UpdatePositionPrediction(pred.step);
                pred.allData[j].positions.Add(pred.dronesPrediction[j].position);
                pred.allData[j].crashed.Add(pred.dronesPrediction[j].hasCrashed);

                if (pred.dronesPrediction[j].hasCrashed && !pred.allData[j].crashedPrediction)
                {
                    pred.allData[j].crashedPrediction = true;
                    pred.allData[j].idFirstCrash = i;
                }
            }
        }

        pred.donePrediction = true;
    }


    void launchPreditionThread(Prediction pred)
    {
        pred.alignementVector = MigrationPointController.alignementVector;

        shortPred.directionOfMigration = this.GetComponent<MigrationPointController>().deltaMigration;
        spawnPrediction(shortPred);
        lock (shortPred)
        {
            predictionThread = new Thread(() => StartPrediction(pred));
            predictionThread.Start();
        }
    }
    void spawnPrediction(Prediction pred)
    {
        pred.allData = new List<DroneDataPrediction>();
        pred.dronesPrediction = new List<DroneFake>();
        pred.refreshIdLeader();

        foreach (DroneFake child in swarmModel.drones)
        {
            DroneDataPrediction data = new DroneDataPrediction();
            DroneFake copy = new DroneFake(child.position, child.velocity, false, child.id);
            copy.embodied = child.embodied;
            copy.selected = child.selected;
            
            pred.dronesPrediction.Add(copy);
            pred.allData.Add(data);
        }
    }

    void Update()
    {   
        if(shortPred.donePrediction)
        {
            // this.GetComponent<HapticsTest>().HapticsPrediction(shortPred);
            UpdateTubes(shortPred);
            shortPred.donePrediction = false;
            launchPreditionThread(shortPred);
        }
        //check if thread crashed
        if (predictionThread != null && !predictionThread.IsAlive)
        {
            predictionThread = null;
            //restart the thread
            launchPreditionThread(shortPred);
        }
    }

    //on exit
    void OnDisable()
    {
        StopAllCoroutines();
        // stop the prediction thread
        if (predictionThread != null)
        {
            predictionThread.Abort();
        }

    }


    void UpdateTubes(Prediction pred)
    {
        if (pred.allData == null || pred.allData.Count == 0)
            return;

        // Destroy existing tube objects.
        foreach (GameObject tube in pred.TubeObjects)
        {
            Destroy(tube);
        }
        pred.TubeObjects.Clear();

        // Settings for tube generation:
        float tubeRadius = CameraMovement.embodiedDrone != null ? 0.01f : 0.04f;
        int radialSegments = 8; // adjust for smoothness vs. performance

        foreach (DroneDataPrediction data in pred.allData)
        {
            if (data.positions == null || data.positions.Count < 2)
                continue;

            GameObject tubeObj = new GameObject("DronePredictionTube");
            tubeObj.transform.SetParent(pred.lineHolder);

            MeshFilter mf = tubeObj.AddComponent<MeshFilter>();
            MeshRenderer mr = tubeObj.AddComponent<MeshRenderer>();

            Mesh tubeMesh = GenerateTubeMesh(data.positions, tubeRadius, radialSegments);
            mf.mesh = tubeMesh;
            tubeObj.layer = 10;

            // Determine tube color (red if any segment has crashed, grey otherwise)
            bool hasCrashed = false;
            for (int i = 0; i < data.crashed.Count; i++)
            {
                hasCrashed = hasCrashed || data.crashed[i];
            }
            Color tubeColor = hasCrashed ? Color.red : Color.grey;

            //change opacity of the tube
            tubeColor.a = 0.75f;

            // Instantiate a new material instance to avoid modifying the shared material.
            Material mat = new Material(defaultMaterial);
            mat.color = tubeColor;
            mr.material = mat;

            pred.TubeObjects.Add(tubeObj);
        }
    }

    Mesh GenerateTubeMesh(List<Vector3> points, float radius, int radialSegments)
    {
        int numPoints = points.Count;
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();
        List<Vector3> normals = new List<Vector3>();
        List<Vector2> uvs = new List<Vector2>();

        // We'll compute an orientation (normal/binormal) for each cross-section.
        Vector3 prevNormal = Vector3.zero;
        for (int i = 0; i < numPoints; i++)
        {
            // Determine the tangent direction:
            Vector3 tangent;
            if (i < numPoints - 1)
                tangent = (points[i + 1] - points[i]).normalized;
            else
                tangent = (points[i] - points[i - 1]).normalized;

            // For the first point, pick an arbitrary normal:
            Vector3 normal;
            if (i == 0)
            {
                normal = Vector3.Cross(tangent, Vector3.up);
                if (normal.sqrMagnitude < 0.001f)
                    normal = Vector3.Cross(tangent, Vector3.right);
                normal.Normalize();
                prevNormal = normal;
            }
            else
            {
                // Use a simple parallel transport method:
                normal = prevNormal - Vector3.Dot(prevNormal, tangent) * tangent;
                if (normal.sqrMagnitude < 0.001f)
                {
                    normal = Vector3.Cross(tangent, Vector3.up);
                    if (normal.sqrMagnitude < 0.001f)
                        normal = Vector3.Cross(tangent, Vector3.right);
                }
                normal.Normalize();
                prevNormal = normal;
            }
            // Binormal completes the frame.
            Vector3 binormal = Vector3.Cross(tangent, normal).normalized;

            // Create vertices around a circle in the plane defined by normal and binormal.
            for (int j = 0; j < radialSegments; j++)
            {
                float theta = 2 * Mathf.PI * j / radialSegments;
                Vector3 radialDir = Mathf.Cos(theta) * normal + Mathf.Sin(theta) * binormal;
                vertices.Add(points[i] + radialDir * radius);
                normals.Add(radialDir);
                uvs.Add(new Vector2((float)j / radialSegments, (float)i / (numPoints - 1)));
            }
        }

        // Build triangles between consecutive rings.
        for (int i = 0; i < numPoints - 1; i++)
        {
            for (int j = 0; j < radialSegments; j++)
            {
                int current = i * radialSegments + j;
                int next = current + radialSegments;
                int currentNext = i * radialSegments + ((j + 1) % radialSegments);
                int nextNext = currentNext + radialSegments;

                // First triangle of quad
                triangles.Add(current);
                triangles.Add(next);
                triangles.Add(currentNext);

                // Second triangle of quad
                triangles.Add(currentNext);
                triangles.Add(next);
                triangles.Add(nextNext);
            }
        }

        Mesh mesh = new Mesh();
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);
        mesh.SetNormals(normals);
        mesh.SetUVs(0, uvs);
        return mesh;
    }

}





public class Prediction
{
    public bool donePrediction = false;
    public bool shortPrediction;
    public int deep;
    public int current;

    public Vector3 directionOfMigration;

    public int step = 1;

    public Transform lineHolder;    

    public List<DroneFake> dronesPrediction;

    public List<DroneDataPrediction> allData;
   // public List<LineRenderer> LineRenderers;

    public List<GameObject> TubeObjects = new List<GameObject>();

    public Vector3 alignementVector;

    public GameObject selectedDrone = MigrationPointController.selectedDrone;
    public GameObject embodiedDrone = CameraMovement.embodiedDrone;

    public int idLeader = -1;

    public Prediction(bool prediction, int deep, int step, int current,  Transform lineHolder)
    {
        this.shortPrediction = prediction;
        this.deep = deep;
        this.step = step;
        this.current = current;
        this.lineHolder = lineHolder;
        this.allData = new List<DroneDataPrediction>();
        //this.LineRenderers = new List<LineRenderer>();
        this.TubeObjects = new List<GameObject>();
        directionOfMigration = Vector3.zero;

        refreshIdLeader();

    }


    public void refreshIdLeader(){
        idLeader = -1;
        if(CameraMovement.embodiedDrone != null)
        {
//            Debug.Log("Embodied drone prediction" + idLeader);
            idLeader = CameraMovement.idLeader;
        }else if(MigrationPointController.selectedDrone != null)
        {
            idLeader = MigrationPointController.idLeader;
 //           Debug.Log("Selected drone prediction" + idLeader);
        }
    }

}

public class DroneDataPrediction
{
    public List<Vector3> positions;
    public List<bool> crashed;
    public bool crashedPrediction = false;
    public int idFirstCrash = 0;

    public DroneDataPrediction()
    {
        positions = new List<Vector3>();
        crashed = new List<bool>();
    }
}
