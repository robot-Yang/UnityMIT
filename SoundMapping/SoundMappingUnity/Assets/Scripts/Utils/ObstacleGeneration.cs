using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class ObstacleGeneration : MonoBehaviour
{
    public GameObject[] floorObjects;    // Array of floor GameObjects
    public GameObject obstaclePrefab;    // The obstacle prefab to instantiate
    public Transform floorParent;        // The parent object for the instantiated obstacles
    public float densityWall = 0.1f;         // Obstacles per unit area
    public float densityCylinder = 0.03f;         // Obstacles per unit area

    public GameObject wallPrefab;    // The wall prefab to instantiate
    private float minWallLength = 10f; // Minimum wall length
    private float wallHeight = 50f;    // Fixed wall height
    private float wallDepth = 2f;   // Fixed wall depth



    void Awake()
    {
       // PlaceObstacles();
       // PlaceWalls();
    }

    void Start()
    {
        AddObstacles();
    }

    void PlaceWalls()
    {
        floorObjects = GameObject.FindGameObjectsWithTag("FloorWall");

        foreach (GameObject floor in floorObjects)
        {
            // Get the width of the floor
            float floorWidth = GetFloorWidth(floor);

            if (floorWidth <= 0f)
            {
                // Debug.LogWarning($"Cannot determine width of floor {floor.name}");
                continue;
            }

            float maxWallLength = floorWidth * 0.75f;

            int wallCount = DecideNumberOfWalls(floor);

            for (int i = 0; i < wallCount; i++)
            {
                // Random length between minLength and maxWallLength
                float wallLength = Random.Range(minWallLength, maxWallLength);

                // Random position on floor
                Mesh mesh = floor.GetComponent<MeshFilter>().sharedMesh;
                if (mesh == null)
                {
                    // Debug.LogWarning($"No mesh found on {floor.name}");
                    continue;
                }
                Vector3 position = GetRandomPointOnMesh(mesh, floor.transform);

                // Adjust position.y to be at the floor level plus half wall height
                position.y = 5;

                // Random rotation
                Quaternion rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

                // Instantiate wall
                GameObject wall = Instantiate(wallPrefab, position, rotation, floorParent);

                wall.transform.position = position;
                wall.transform.localScale = new Vector3(wallLength, wallHeight, wallDepth);
            }
        }
    }

    int DecideNumberOfWalls(GameObject floor)
    {
        // Decide number of walls based on floor area and wall density
        Mesh mesh = floor.GetComponent<MeshFilter>().sharedMesh;
        if (mesh == null)
        {
            // Debug.LogWarning($"No mesh found on {floor.name}");
            return 0;
        }
        float area = CalculateMeshArea(mesh, floor.transform);
        int wallCount = Mathf.RoundToInt(area * densityWall);
        return wallCount;
    }


    float GetFloorWidth(GameObject floor)
    {
        Renderer renderer = floor.GetComponent<Renderer>();
        if (renderer != null)
        {
            return renderer.bounds.size.x;
        }
        else
        {
            // Try getting from mesh bounds
            MeshFilter meshFilter = floor.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.sharedMesh != null)
            {
                Vector3 size = meshFilter.sharedMesh.bounds.size;
                return size.x * floor.transform.lossyScale.x;
            }
        }
        return 0f;
    }


    float CalculateMeshArea(Mesh mesh, Transform transform)
    {
        float totalArea = 0f;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // Iterate over each triangle
        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 p0 = transform.TransformPoint(vertices[triangles[i]]);
            Vector3 p1 = transform.TransformPoint(vertices[triangles[i + 1]]);
            Vector3 p2 = transform.TransformPoint(vertices[triangles[i + 2]]);

            // Calculate the area of the triangle
            float triangleArea = Vector3.Cross(p1 - p0, p2 - p0).magnitude * 0.5f;
            totalArea += triangleArea;
        }

        return totalArea;
    }

    Vector3 GetRandomPointOnMesh(Mesh mesh, Transform transform)
    {
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        int triangleCount = triangles.Length / 3;

        // Step 1: Calculate the area of each triangle and the total area
        float[] cumulativeAreas = new float[triangleCount];
        float totalArea = 0f;

        for (int i = 0; i < triangleCount; i++)
        {
            int index = i * 3;
            Vector3 p0 = transform.TransformPoint(vertices[triangles[index]]);
            Vector3 p1 = transform.TransformPoint(vertices[triangles[index + 1]]);
            Vector3 p2 = transform.TransformPoint(vertices[triangles[index + 2]]);

            // Calculate the area of the triangle
            float area = Vector3.Cross(p1 - p0, p2 - p0).magnitude * 0.5f;
            totalArea += area;
            cumulativeAreas[i] = totalArea; // Build cumulative area
        }

        // Step 2: Pick a random value between 0 and total area
        float randomSample = Random.value * totalArea;

        // Step 3: Find the triangle that corresponds to the random sample
        int selectedTriangleIndex = -1;
        for (int i = 0; i < triangleCount; i++)
        {
            if (randomSample <= cumulativeAreas[i])
            {
                selectedTriangleIndex = i;
                break;
            }
        }

        // Fallback in case of floating-point inaccuracies
        if (selectedTriangleIndex == -1)
        {
            selectedTriangleIndex = triangleCount - 1;
        }

        // Step 4: Get the vertices of the selected triangle
        int triangleVertexIndex = selectedTriangleIndex * 3;
        Vector3 v0 = vertices[triangles[triangleVertexIndex]];
        Vector3 v1 = vertices[triangles[triangleVertexIndex + 1]];
        Vector3 v2 = vertices[triangles[triangleVertexIndex + 2]];

        // Step 5: Generate a random point within the triangle
        Vector3 randomPoint = GetRandomPointInTriangle(v0, v1, v2);

        // Transform the point to world coordinates
        return transform.TransformPoint(randomPoint);
    }

    Vector3 GetRandomPointInTriangle(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        // Generate random barycentric coordinates
        float r1 = Random.value;
        float r2 = Random.value;

        // Ensure the point lies within the triangle
        if (r1 + r2 > 1f)
        {
            r1 = 1f - r1;
            r2 = 1f - r2;
        }

        float r3 = 1f - r1 - r2;

        // Calculate the random point
        return v0 * r1 + v1 * r2 + v2 * r3;
    }
    
    void AddObstacles()
    {
        GameObject[] obstacleObjects = GameObject.FindGameObjectsWithTag("Obstacle");
        List<Obstacle> obstacles = new List<Obstacle>();

        foreach (GameObject obstacleObject in obstacleObjects)
        {
            Obstacle obstacle = null;
            if (obstacleObject.GetComponent<SphereCollider>() != null)
            {
                SphereCollider sphereCollider = obstacleObject.GetComponent<SphereCollider>();
                obstacle = new SphereObstacle(obstacleObject.transform.position, sphereCollider.radius);
            }
            else if (obstacleObject.GetComponent<BoxCollider>() != null)
            {
                BoxCollider boxCollider = obstacleObject.GetComponent<BoxCollider>();
                obstacle = new BoxObstacle(obstacleObject.transform.position, obstacleObject.transform.lossyScale, obstacleObject.transform.rotation);
            }
            else if (obstacleObject.GetComponent<CapsuleCollider>() != null)
            {
                CapsuleCollider capsuleCollider = obstacleObject.GetComponent<CapsuleCollider>();
                obstacle = new CylinderObstacle(obstacleObject.transform.position, obstacleObject.transform.lossyScale.x, obstacleObject.transform.lossyScale.y, obstacleObject.transform.rotation);
            }
            else
            {
                // Debug.LogWarning(obstacleObject.name + " has no supported collider type.");
                continue;
            }

            if(obstacleObject.layer == 0)
            {
                obstacle.transparent = true;
            }    

            obstacles.Add(obstacle);
        }


        ClosestPointCalculator.obstacles = obstacles;
        ClosestPointCalculator.obstaclesInRange = obstacles;

    }

}
