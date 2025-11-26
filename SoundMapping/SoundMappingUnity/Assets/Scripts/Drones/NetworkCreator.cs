using System.Collections.Generic;
using UnityEngine;
using System.Collections;
using System.Linq;
using System.Threading;
using UnityEngine.InputSystem.Interactions;

public class NetworkCreator
{
    public List<DroneFake> drones = new List<DroneFake>();
    public Dictionary<DroneFake, List<DroneFake>> adjacencyList = new Dictionary<DroneFake, List<DroneFake>>();
    public HashSet<DroneFake> largestComponent = new HashSet<DroneFake>();

    bool hasEmbodied = false;

    public NetworkCreator(List<DroneFake> dr)
    {
        drones = dr;
        foreach (DroneFake drone in drones)
        {
            adjacencyList[drone] = new List<DroneFake>();
            if (drone.embodied)
            {
                hasEmbodied = true;
            }
        }
    }

    public void refreshNetwork()
    {
        BuildNetwork(drones);
        FindLargestComponent(drones);
        AssignLayers();
    }

    public void refreshNetwork(int idLeader)
    {
        BuildNetwork(drones);
        FindLargestComponent(drones, idLeader);
        AssignLayers(idLeader);
    }

    void BuildNetwork(List<DroneFake> drones)
    {
        try
        {
            // Clear previous connections.
            foreach (var drone in adjacencyList.Keys)
            {
                adjacencyList[drone].Clear();
            }
            // Build new connections.
            foreach (DroneFake drone in drones)
            {
                foreach (DroneFake otherDrone in drones)
                {
                    if (drone == otherDrone) continue;
                    if (IsDistanceNeighbor(drone, otherDrone))
                    {
                        adjacencyList[drone].Add(otherDrone);
                    }
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error in BuildNetwork: " + e.Message);
            return;
        }
    }

    void FindLargestComponent(List<DroneFake> drones)
    {
        largestComponent.Clear();

        HashSet<DroneFake> visited = new HashSet<DroneFake>();
        List<HashSet<DroneFake>> components = new List<HashSet<DroneFake>>();

        foreach (DroneFake drone in drones)
        {
            if (!visited.Contains(drone))
            {
                HashSet<DroneFake> component = new HashSet<DroneFake>();
                Queue<DroneFake> queue = new Queue<DroneFake>();
                queue.Enqueue(drone);
                visited.Add(drone);

                while (queue.Count > 0)
                {
                    DroneFake current = queue.Dequeue();
                    component.Add(current);
                    foreach (DroneFake neighbor in adjacencyList[current])
                    {
                        if (!visited.Contains(neighbor))
                        {
                            visited.Add(neighbor);
                            queue.Enqueue(neighbor);
                        }
                    }
                }
                components.Add(component);
            }
        }

        // Choose the component that either contains an embodied or selected drone;
        // if none, choose the largest.
        largestComponent.Clear();
        int maxCount = 0;

        int idLeader = -1;
        if(CameraMovement.embodiedDrone != null)
        {
            idLeader = CameraMovement.idLeader;
        }else if(MigrationPointController.selectedDrone != null)
        {
            idLeader = MigrationPointController.idLeader;
        }

        foreach (HashSet<DroneFake> component in components)
        {
            bool isLeader = component.Any(d => d.id == idLeader);
            if (isLeader)
            {
                largestComponent = component;
                break;
            }
            if (component.Count > maxCount)
            {
                maxCount = component.Count;
                largestComponent = component;
            }
        }

      //  Debug.Log("Largest component: " + largestComponent.Count);
    }

    void FindLargestComponent(List<DroneFake> drones, int idLeader)
    {
        largestComponent.Clear();

        HashSet<DroneFake> visited = new HashSet<DroneFake>();
        List<HashSet<DroneFake>> components = new List<HashSet<DroneFake>>();

        foreach (DroneFake drone in drones)
        {
            if (!visited.Contains(drone))
            {
                HashSet<DroneFake> component = new HashSet<DroneFake>();
                Queue<DroneFake> queue = new Queue<DroneFake>();
                queue.Enqueue(drone);
                visited.Add(drone);

                while (queue.Count > 0)
                {
                    DroneFake current = queue.Dequeue();
                    component.Add(current);
                    foreach (DroneFake neighbor in adjacencyList[current])
                    {
                        if (!visited.Contains(neighbor))
                        {
                            visited.Add(neighbor);
                            queue.Enqueue(neighbor);
                        }
                    }
                }
                components.Add(component);
            }
        }

        // Choose the component that either contains an embodied or selected drone;
        // if none, choose the largest.
        largestComponent.Clear();
        int maxCount = 0;

        foreach (HashSet<DroneFake> component in components)
        {
            bool isLeader = component.Any(d => d.id == idLeader);
            if (isLeader)
            {
                largestComponent = component;
                break;
            }
            if (component.Count > maxCount)
            {
                maxCount = component.Count;
                largestComponent = component;
            }
        }

      //  Debug.Log("Largest component: " + largestComponent.Count);
    }

    public bool IsInMainNetwork(DroneFake drone)
    {
        return largestComponent.Contains(drone);
    }

    bool IsDistanceNeighbor(DroneFake a, DroneFake b)
    {
        float distance = Vector3.Distance(a.position, b.position);
        if (distance > DroneFake.neighborRadius) return false;
        // bool visible = ClosestPointCalculator.IsLineIntersecting(a.position, b.position);
        // return !visible;
        return true; // ← add this line so the function always returns
    }

    public List<DroneFake> GetNeighbors(DroneFake drone)
    {
        if (!adjacencyList.ContainsKey(drone))
        {
            return new List<DroneFake>();
        }
        return adjacencyList[drone];
    }

    public DroneFake GetLeader()
    {
        int idLeader = -1;
        if(CameraMovement.embodiedDrone != null)
        {
            idLeader = CameraMovement.idLeader;
        }else if(MigrationPointController.selectedDrone != null)
        {
            idLeader = MigrationPointController.idLeader;
        }

        return drones.Find(d => d.id == idLeader);

    }

    public DroneFake GetLeader(int idLeader)
    {
        return drones.Find(d => d.id == idLeader);

    }

    public void AssignLayers()
    {
        // Use the embodied (or selected) drone as the core if possible.
        DroneFake coreDrone = GetLeader();
        foreach (var drone in adjacencyList.Keys)
        {
            drone.layer = 0;
        }
        if (coreDrone == null)
        {
            return;
        }
        if (!adjacencyList.ContainsKey(coreDrone))
        {
            return;
        }
        Queue<DroneFake> queue = new Queue<DroneFake>();
        coreDrone.layer = 1;
        queue.Enqueue(coreDrone);
        while (queue.Count > 0)
        {
            DroneFake currentDrone = queue.Dequeue();
            int currentLayer = currentDrone.layer;
            if (adjacencyList.ContainsKey(currentDrone))
            {
                foreach (DroneFake neighbor in adjacencyList[currentDrone])
                {
                    if (neighbor.layer == 0) // unassigned drone 
                    {
                        neighbor.layer = currentLayer + 1;
                        queue.Enqueue(neighbor);
                    }
                }
            }
        }
    }

    public void AssignLayers(int idLeader)
    {
        // Use the embodied (or selected) drone as the core if possible.
        DroneFake coreDrone = GetLeader(idLeader);
        foreach (var drone in adjacencyList.Keys)
        {
            drone.layer = 0;
        }
        if (coreDrone == null)
        {
            return;
        }
        if (!adjacencyList.ContainsKey(coreDrone))
        {
            return;
        }
        Queue<DroneFake> queue = new Queue<DroneFake>();
        coreDrone.layer = 1;
        queue.Enqueue(coreDrone);
        while (queue.Count > 0)
        {
            DroneFake currentDrone = queue.Dequeue();
            int currentLayer = currentDrone.layer;
            if (adjacencyList.ContainsKey(currentDrone))
            {
                foreach (DroneFake neighbor in adjacencyList[currentDrone])
                {
                    if (neighbor.layer == 0) // unassigned drone 
                    {
                        neighbor.layer = currentLayer + 1;
                        queue.Enqueue(neighbor);
                    }
                }
            }
        }
    }


    public Dictionary<int, int> getLayersConfiguration()
    {
        Dictionary<int, int> layers = new Dictionary<int, int>();
        foreach (var drone in drones)
        {
            if (!layers.ContainsKey(drone.layer))
            {
                layers[drone.layer] = 0;
            }
            layers[drone.layer]++;
        }
        return layers;
    }

    public Dictionary<int, List<DroneFake>> getLayers()
    {
        Dictionary<int, List<DroneFake>> layers = new Dictionary<int, List<DroneFake>>();
        foreach (var drone in drones)
        {
            if (!layers.ContainsKey(drone.layer))
            {
                layers[drone.layer] = new List<DroneFake>();
            }
            layers[drone.layer].Add(drone);
        }
        return layers;
    }

    // --- NEW: Formation / Cohesion Metrics Methods ---

    // 1. Count the number of connected components in the network.
    public int CountConnectedComponents()
    {
        HashSet<DroneFake> visited = new HashSet<DroneFake>();
        int components = 0;
        foreach (DroneFake drone in drones)
        {
            if (!visited.Contains(drone))
            {
                components++;
                Queue<DroneFake> queue = new Queue<DroneFake>();
                queue.Enqueue(drone);
                visited.Add(drone);
                while (queue.Count > 0)
                {
                    DroneFake current = queue.Dequeue();
                    foreach (DroneFake neighbor in adjacencyList[current])
                    {
                        if (!visited.Contains(neighbor))
                        {
                            visited.Add(neighbor);
                            queue.Enqueue(neighbor);
                        }
                    }
                }
            }
        }
        return components;
    }

    // 2. Relative Connectivity (C(t)):
    //    For n drones and k connected components, we have:
    //    rank(L) = n - k, so that C(t) = (n - k) / (n - 1).
    public float ComputeRelativeConnectivity()
    {
        int n = drones.Count;
        if (n <= 1) return 1f;  // A single agent is “fully connected.”
        int components = CountConnectedComponents();
        int rank = n - components;
        return (float)rank / (n - 1);
    }

    // 3. Cohesion Radius (R(t)):
    //    Compute the centroid of all drones and return the maximum distance from any drone to that centroid.
    public float ComputeCohesionRadius()
    {
        if (drones.Count == 0) return 0;
        Vector3 centroid = Vector3.zero;
        foreach (DroneFake drone in drones)
        {
            centroid += drone.position;
        }
        centroid /= drones.Count;
        float maxDist = 0;
        foreach (DroneFake drone in drones)
        {
            float dist = (drone.position - centroid).magnitude;
            if (dist > maxDist) maxDist = dist;
        }
        return maxDist;
    }

    // 4. Normalized Deviation Energy (~E(q)):
    //    Here we use a simple proxy: for every unique pair of drones, compare the actual distance with the desired separation.
    //    We average the squared error and then normalize by desiredSeparation^2.
    // public float ComputeNormalizedDeviationEnergy()
    // {
    //     int n = drones.Count;
    //     if (n < 2) return 0;
    //     float sumSquaredError = 0;
    //     int pairCount = 0;
    //     for (int i = 0; i < n; i++)
    //     {
    //         for (int j = i + 1; j < n; j++)
    //         {
    //             float distance = Vector3.Distance(drones[i].position, drones[j].position);
    //             float error = distance - DroneFake.desiredSeparation;
    //             sumSquaredError += error * error;
    //             pairCount++;
    //         }
    //     }
    //     if (pairCount == 0) return 0;

    //     //check if there is a drone that is not in the main network
    //     foreach (var drone in drones)
    //     {
    //         if (!IsInMainNetwork(drone))
    //         {
    //             return 1;
    //         }
    //     }

    //     float avgSquaredError = sumSquaredError / pairCount;
    //     float score = Mathf.Clamp01((avgSquaredError / (DroneFake.desiredSeparation * DroneFake.desiredSeparation)-0.3f)/(0.7f-0.22f));
    //     return Mathf.Clamp01(score-0.1f);
    // }

    public float ComputeNormalizedDeviationEnergy(out float averageDistance)
    {
        averageDistance = 0f;

        int n = drones.Count;
        if (n < 2) return 0f;

        float sumSquaredError = 0f;
        float sumDistance = 0f;     // NEW: accumulate raw pairwise distances
        int pairCount = 0;

        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                float distance = Vector3.Distance(drones[i].position, drones[j].position);
                float error = distance - DroneFake.desiredSeparation;

                sumSquaredError += error * error;
                sumDistance += distance;   // NEW
                pairCount++;
            }
        }

        if (pairCount == 0) return 0f;

        // If any drone isn't in the main network, keep previous behavior (score=1)
        // foreach (var drone in drones)
        // {
        //     if (!IsInMainNetwork(drone))
        //     {
        //         averageDistance = sumDistance / pairCount; // still report the average distance we computed
        //         return 1f;
        //     }
        // }

        // averageDistance = sumDistance / pairCount; // NEW: report average pairwise distance
        averageDistance = sumDistance / pairCount / DroneFake.desiredSeparation; // NEW: report average pairwise distance

        float avgSquaredError = sumSquaredError / pairCount;
        float score = Mathf.Clamp01(
            (avgSquaredError / (DroneFake.desiredSeparation * DroneFake.desiredSeparation) - 0.3f)
            / (0.7f - 0.22f)
        );

        // averageDistance = avgSquaredError / (DroneFake.desiredSeparation * DroneFake.desiredSeparation); // NEW: replace averageDistance temporarily with normalized metric

        return Mathf.Clamp01(score - 0.1f);
    }

    // 5. Normalized Velocity Mismatch (~K(v)):
    //    Compute the average deviation (squared) of each drone’s velocity from the group’s average velocity,
    //    and then normalize by maxSpeed^2.
    public float ComputeNormalizedVelocityMismatch()
    {
        if (drones.Count == 0) return 0;
        Vector3 avgVelocity = Vector3.zero;
        foreach (DroneFake drone in drones)
        {
            avgVelocity += drone.velocity;
        }
        avgVelocity /= drones.Count;
        float sumSquaredVelocityError = 0;
        foreach (DroneFake drone in drones)
        {
            Vector3 diff = drone.velocity - avgVelocity;
            sumSquaredVelocityError += diff.sqrMagnitude;
        }
        float avgVelocityError = sumSquaredVelocityError / drones.Count;
        return Mathf.Clamp01(avgVelocityError / (DroneFake.maxSpeed * DroneFake.maxSpeed));
    }

    // 6. Overall Cohesion Score:
    //    We now combine the four metrics. We define “error” measures so that 0 means perfect cohesion.
    //    For example, connectivityError = 1 - C(t).
    //    We also normalize the cohesion radius (e.g. dividing by neighborRadius).
    //    Finally, we take an average so that the overall score is scaled in [0,1], where 0 is fully cohesive.
    public float ComputeOverallCohesionScore()
    {
        int n = drones.Count;
        if (n <= 1) return 0;
        
        // Connectivity error: 0 if fully connected.
        float relativeConnectivity = ComputeRelativeConnectivity();
        float connectivityError = 1f - relativeConnectivity;

        // Cohesion error: we normalize the maximum distance from the centroid by the neighbor radius.
        float cohesionRadius = ComputeCohesionRadius();
        float cohesionError = Mathf.Clamp01(cohesionRadius / DroneFake.neighborRadius);

        // Deviation energy error: already normalized.
        float avgDist;
        float deviationEnergyError = ComputeNormalizedDeviationEnergy(out avgDist);

        // Velocity mismatch error: already normalized.
        float velocityMismatchError = ComputeNormalizedVelocityMismatch();

        // Average the errors (or use different weights if desired).
        float overallScore = (connectivityError + cohesionError + deviationEnergyError + velocityMismatchError) / 4f;
        return overallScore;
    }


    public List<HashSet<DroneFake>> GetSubnetworks()
    {
        List<HashSet<DroneFake>> subnetworks = new List<HashSet<DroneFake>>();
        HashSet<DroneFake> visited = new HashSet<DroneFake>();

        foreach (DroneFake drone in drones)
        {
            if (!visited.Contains(drone))
            {
                HashSet<DroneFake> component = new HashSet<DroneFake>();
                Queue<DroneFake> queue = new Queue<DroneFake>();

                queue.Enqueue(drone);
                visited.Add(drone);

                while (queue.Count > 0)
                {
                    DroneFake current = queue.Dequeue();
                    component.Add(current);

                    foreach (DroneFake neighbor in adjacencyList[current])
                    {
                        if (!visited.Contains(neighbor))
                        {
                            visited.Add(neighbor);
                            queue.Enqueue(neighbor);
                        }
                    }
                }
                subnetworks.Add(component);
            }
        }

        return subnetworks;
    }

    public bool IsFullyConnected()
    {
        Debug.Log("Subnetworks: " + GetSubnetworks().Count);
        return GetSubnetworks().Count == 1;
    }
}

