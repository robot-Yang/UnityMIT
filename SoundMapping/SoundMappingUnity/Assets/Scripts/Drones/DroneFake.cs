
using System.Collections.Generic;
using UnityEngine;


public class DroneFake
{
    #region Paramters Classes
    public bool isMovable = true;
    public Vector3 position;
    public Vector3 acceleration;
    public Vector3 velocity;

    public int layer = 0;

    public int id = 0;
    public string idS = "0";
    
    public static float maxSpeed;
    public static float maxForce;
    public static float desiredSeparation = 3f;
    public static float desiredSeparationObs = 3f;
    public static float neighborRadius = 5f;
    public static float alpha = 1.5f; // c
    public static float beta = 1.0f;  // c
    public static float delta = 1.0f; // c
    public static float cVm = 1.0f; // Velocity matching coefficient
    public static float avoidanceRadius = 2f;     // Radius for obstacle detection
    public static float avoidanceRadiusFeedback = 2f;     // Radius for obstacle detection
    public static float avoidanceForce = 10f;     // Strength of the avoidance force
    public static float droneRadius = 0.17f;

    public static float dampingFactor = 0.96f;

    public static float lastDT = 0.02f;

    public static float spawnHeight = 0.5f;

    public bool embodied = false;
    public bool selected = false;

    public float score = 1.0f;

    public float scoreFollowing = 1.0f;
    public float networkScore = 1.0f;
    public int numberNeighboor = 0;

    public static int PRIORITYWHENEMBODIED = 1;

    public bool hasCrashed = false;
    public bool hasCrashedFeedback = false;

    public Vector3 lastOlfati = Vector3.zero;
    public Vector3 lastObstacle = Vector3.zero;
    public Vector3 lastAllignementSwarm = Vector3.zero; // forces applied

    public List<Vector3> lastObstacleForces = new List<Vector3>();  
    public List<Vector3> lastObstacleForcesFeedback = new List<Vector3>();  

    public Vector3 lastAllignement = Vector3.zero; //direction of migration


    public List<float> olfatiForce = new List<float>();
    public List<float> obstacleForce = new List<float>();

    public List<Vector3> olfatiForceVec = new List<Vector3>();
    public List<Vector3> obstacleForceVec = new List<Vector3>();

    #endregion

    public DroneFake(Vector3 position, Vector3 velocity, bool embodied, int id, bool isMovable = true)
    {

        this.position = position;
        this.velocity = velocity;
        this.embodied = embodied;
        this.id = id;
        this.idS = id.ToString();
        this.isMovable = isMovable;
    }
    public List<DroneFake> GetNeighbors(List<DroneFake> allDrones)
    {
        List<DroneFake> neighbors = new List<DroneFake>();
        foreach (DroneFake drone in allDrones)
        {
            if (drone == this) continue;

            if (Vector3.Distance(this.position, drone.position) < neighborRadius)
            {
                if(drone.hasCrashed)
                {
                    continue;
                }
                neighbors.Add(drone);
            }
        }
        return neighbors;
    }

    #region forcesFunction

    public (List<Vector3> forces, bool hasCrashed) getObstacleForces(float avoidanceRadius, float distanceObs, float cPmObs = 10f)
    {
        List<Vector3> forces = new List<Vector3>();
        float c = (beta - alpha) / (2 * Mathf.Sqrt(alpha * beta));
        bool hasCrashed = false;

        List<Vector3> obstacles = ClosestPointCalculator.ClosestPointsWithinRadius(position, avoidanceRadius);
        foreach (Vector3 obsPos in obstacles)
        {
            Vector3 posRel = position - obsPos;
            float dist = posRel.magnitude - droneRadius;
            if (dist <= Mathf.Epsilon)
            {
                hasCrashed = true;
                continue;
            }

            // Apply forces similar to your original logic
            Vector3 force = cPmObs * (GetNeighbourWeight(dist, distanceObs, delta) *
                        GetCohesionForce(dist, distanceObs, alpha, beta, c, avoidanceRadius, delta, obsPos - position)
                        );

            forces.Add(force);
        }

        return (forces, hasCrashed);
    }

    public (List<Vector3> forces, bool hasCrashedFeedback) getObstacleForcesFeedback(float avoidanceRadiusFeedback, float distanceObs, float cPmObs = 10f)
    {
        List<Vector3> forces = new List<Vector3>();
        float c = (beta - alpha) / (2 * Mathf.Sqrt(alpha * beta));
        bool hasCrashedFeedback = false;

        List<Vector3> obstacles = ClosestPointCalculator.ClosestPointsWithinRadius(position, avoidanceRadiusFeedback);
        foreach (Vector3 obsPos in obstacles)
        {
            Vector3 posRel = position - obsPos;
            float dist = posRel.magnitude - droneRadius;
            if (dist <= Mathf.Epsilon)
            {
                hasCrashedFeedback = true;
                continue;
            }

            // Apply forces similar to your original logic
            Vector3 force = cPmObs * (GetNeighbourWeight(dist, distanceObs, delta) *
                        GetCohesionForce(dist, distanceObs, alpha, beta, c, avoidanceRadiusFeedback, delta, obsPos - position)
                        );

            forces.Add(force);
        }

        return (forces, hasCrashedFeedback);
    }

    public void startPrediction(Vector3 alignementVector, NetworkCreator network)
    {
        ComputeForces(alignementVector, network);
    }
    private float GetCohesionIntensity(float r, float dRef, float a, float b, float c)
    {
        float diff = r - dRef;
        return ((a + b) / 2) * (Mathf.Sqrt(1 + Mathf.Pow(diff + c, 2)) - Mathf.Sqrt(1 + c * c)) + ((a - b) * diff / 2);
    }

    // Calculate cohesion intensity derivative
    private float GetCohesionIntensityDer(float r, float dRef, float a, float b, float c)
    {
        float diff = r - dRef;
        return ((a + b) / 2) * (diff + c) / Mathf.Sqrt(1 + Mathf.Pow(diff + c, 2)) + ((a - b) / 2);
    }

    // Calculate neighbor weight
    private float GetNeighbourWeight(float r, float r0, float delta)
    {
        float rRatio = r / r0;

        if (rRatio < delta)
            return 1;
        else if (rRatio < 1)
            return 0.25f * Mathf.Pow(1 + Mathf.Cos(Mathf.PI * (rRatio - delta) / (1 - delta)), 2);
        else
            return 0;
    }

    // Calculate neighbor weight derivative
    private float GetNeighbourWeightDer(float r, float r0, float delta)
    {
        float rRatio = r / r0;

        if (rRatio < delta)
            return 0;
        else if (rRatio < 1)
        {
            float arg = Mathf.PI * (rRatio - delta) / (1 - delta);
            return -0.5f * (Mathf.PI / (1 - delta)) * (1 + Mathf.Cos(arg)) * Mathf.Sin(arg);
        }
        else
            return 0;
    }

    // Calculate cohesion force
    private Vector3 GetCohesionForce(float r, float dRef, float a, float b, float c, float r0, float delta, Vector3 posRel)
    {
        float weightDer = GetNeighbourWeightDer(r, r0, delta);
        float intensity = GetCohesionIntensity(r, dRef, a, b, c);
        float intensityDer = GetCohesionIntensityDer(r, dRef, a, b, c);
        float weight = GetNeighbourWeight(r, r0, delta);

        return (weightDer * intensity / r0 + weight * intensityDer) * (posRel / r);
    }

    #endregion

    public void ComputeForces(Vector3 alignmentVector, NetworkCreator network)
    {
        List<DroneFake> allDrones = network.drones;
        List<DroneFake> neighbors = network.GetNeighbors(this);

        // Constants
        float dRef = desiredSeparation;
        // if(this.layer >= 3)
        // {
        //     dRef = Mathf.Max(desiredSeparation*(0.8f - 0.2f*((float)layer - 3f)), 1);
        // }
        float dRefObs = avoidanceRadius;
        float dRefObsFb = avoidanceRadiusFeedback;

        float a = alpha;
        float b = beta;
        float c = (b - a) / (2 * Mathf.Sqrt(a * b));

        float r0Coh = neighborRadius;
        float r0Obs = avoidanceRadius;
        float cPmObs = avoidanceForce;

                // Reference velocity
        Vector3 vRef = alignmentVector;

        Vector3 accCoh = Vector3.zero;
        Vector3 accVel = Vector3.zero;

        float basePriority = 1;
        DroneFake embodiedDrone = allDrones.Find(d => d.embodied);


        if (embodiedDrone != null)
        {
            basePriority = 1; 
        }

        basePriority = 1;

        float totalPriority = 0;

        foreach (DroneFake neighbour in neighbors)
        {
            float neighborPriority = getPriority(basePriority, neighbour);
            totalPriority += neighborPriority;


            Vector3 posRelD = neighbour.position - position;
            float distD = posRelD.magnitude - 2*droneRadius;
            if (distD <= Mathf.Epsilon)
            {
                this.hasCrashed = true;
            }
            if(neighbour.isMovable == false)
            {
                if(distD < 0.5f)
                {
                    this.hasCrashed = true;                
                }
            }
            accCoh += GetCohesionForce(distD, dRef, a, b, c, r0Coh, delta, posRelD) * neighborPriority;

           accVel += (neighbour.velocity - velocity) * neighborPriority;
        }

        if(totalPriority == 0){
            accVel = cVm * (vRef - velocity); // 50% of the velocity matching force
        }else{
            accVel = (accVel + cVm * (vRef - velocity)) / 2; // 50% of the velocity matching force
        }

        // Obstacle avoidance
        Vector3 accObs = Vector3.zero;
        List<Vector3> obstacles = ClosestPointCalculator.ClosestPointsWithinRadius(position, avoidanceRadius);


        // get obstacle forces
        (List<Vector3> obstacleForces, bool hasCrashed) = getObstacleForces(avoidanceRadius, dRefObs, cPmObs);
        (List<Vector3> obstacleForcesFeedback, bool hasCrashedFeedback) = getObstacleForcesFeedback(avoidanceRadiusFeedback, dRefObsFb, cPmObs);

        this.hasCrashed = hasCrashed || this.hasCrashed;
        this.lastObstacleForces = obstacleForces;
        this.lastObstacleForcesFeedback = obstacleForcesFeedback;

        foreach (Vector3 force in obstacleForces)
        {
            accObs += force;
        }
        
        lastOlfati = accCoh;
        lastObstacle = accObs;
        lastAllignementSwarm = accVel;

//         if (layer == 1 && embodied)
//         {
// //            Debug.Log(id);
//             accVel = cVm * (vRef - velocity);
//             lastAllignement = accVel;

//             Vector3 force = accVel;
//             force = Vector3.ClampMagnitude(force, maxForce/3f);
//             acceleration = force;
//             return;
//         }

        if(!network.IsInMainNetwork(this))
        {
            accVel = Vector3.zero;
        }

        lastAllignementSwarm = accVel;
        lastAllignement = accVel;

        Vector3 fo = accCoh + accObs + accVel;
        fo = Vector3.ClampMagnitude(fo, maxForce);
        
        acceleration = fo;
    }

    float getPriority(float basePriority, DroneFake neighbour)
    {
        float neighborPriority = basePriority;
        neighborPriority = 1;
        if (neighbour.layer == 1) // embodied drone
        {
            neighborPriority = Mathf.Max((int)(PRIORITYWHENEMBODIED / 2), PRIORITYWHENEMBODIED);
            // return PRIORITYWHENEMBODIED;
            return 1f;
        }
        else if (neighbour.layer == 2) // neighbor to embodied
        {
            // neighborPriority = Mathf.Max((int)(PRIORITYWHENEMBODIED/4),1);
        }
        else if (neighbour.layer == 3)
        {
            // neighborPriority = Mathf.Max((int)(PRIORITYWHENEMBODIED/8), 1);
        }
        else
        {
            //  neighborPriority = 0.5f;
        }

        return neighborPriority;
    }

    public float getThisPrioity()
    {
        return getPriority(1, this);
    }
    public void resetEmbodied()
    {
        olfatiForce.Clear();
        obstacleForce.Clear();

        olfatiForceVec.Clear();
        obstacleForceVec.Clear();
    }

    public void addDataEmbodied(Vector3 olfati, Vector3 obstacle)
    {
        olfatiForce.Add(olfati.magnitude);
        obstacleForce.Add(obstacle.magnitude);

        olfatiForceVec.Add(olfati);
        obstacleForceVec.Add(obstacle);

        if (olfatiForce.Count > 20)
        {
            olfatiForce.RemoveAt(0);
            obstacleForce.RemoveAt(0);

            olfatiForceVec.RemoveAt(0);
            obstacleForceVec.RemoveAt(0);
        }
    }

    public float getHaptic()
    {
        //takje the last 10 and average them
        float olfati = 0;
        float obstacle = 0;
        int count = Mathf.Min(olfatiForce.Count, 12);

        if (count < 2)
        {
            return 0;
        }

        for (int i = 0; i < count-1; i++)
        {
            float diffOlfati = olfatiForce[olfatiForce.Count - 1 - i] - olfatiForce[olfatiForce.Count - 2 - i];
            olfati += diffOlfati;

            float diffObstacle = obstacleForce[obstacleForce.Count - 1 - i] - obstacleForce[obstacleForce.Count - 2 - i];
            obstacle += diffObstacle;
        }

        olfati /= count;
        obstacle /= count;

        olfati = Mathf.Max(olfati, 0) / 0.3f * 10;
        obstacle = Mathf.Max(obstacle, 0) * 10;

        return olfati + obstacle;

    }

    public Vector3 getHapticVector()
    {
        Vector3 olfatiVec = Vector3.zero;
        Vector3 obstacleVec = Vector3.zero;

        int count = Mathf.Min(olfatiForce.Count, 12);

        if (count < 2)
        {
            return Vector3.zero;
        }

       // return olfatiForceVec[olfatiForceVec.Count - 1];



        for (int i = 0; i < count-1; i++)
        {
            Vector3 diffOlfati = olfatiForceVec[olfatiForceVec.Count - 1 - i] - olfatiForceVec[olfatiForceVec.Count - 2 - i];
            olfatiVec += diffOlfati;

            Vector3 diffObstacle = obstacleForceVec[obstacleForceVec.Count - 1 - i] - obstacleForceVec[obstacleForceVec.Count - 2 - i];
            obstacleVec += diffObstacle;
        }

        olfatiVec /= count;
        obstacleVec /= count;

        olfatiVec *= 10;


        return olfatiVec;
    }

    public bool isNeighboor(DroneFake drone)
    {
        return Vector3.Distance(position, drone.position) < neighborRadius;
    }

    public void UpdatePositionPrediction(int numberOfTimeApplied)
    {
        for (int i = 0; i < numberOfTimeApplied; i++)
        {
            velocity += acceleration * 0.02f; //v(t+1) = v(t) + a(t) * dt  @ a(t) = f(t) w a(t) E Vec3
            // if (layer == 1 && embodied)
            // {
            //     velocity = Vector3.ClampMagnitude(velocity, maxSpeed/2f);
            // }else{
            //     velocity = Vector3.ClampMagnitude(velocity, maxSpeed);
            // }
            velocity = Vector3.ClampMagnitude(velocity, maxSpeed);

            // Apply damping to reduce the velocity over time
            velocity *= dampingFactor;

            position += velocity * 0.02f;
            //position.y = spawnHeight;
        }

        acceleration = Vector3.zero;
    }

    public void UpdatePosition()
    {
        UpdatePositionPrediction(1);
    }

}
