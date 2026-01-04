using System;
using System.Collections.Generic;
using System.Data;
using UnityEngine;

public abstract class Obstacle
{
    public bool transparent = false;
    public Vector3 centerObs;
    public float radiusObs;
    public abstract Vector3 ClosestPoint(Vector3 point);
    public abstract bool IsLineIntersecting(Vector3 start, Vector3 end);
}

public class SphereObstacle : Obstacle
{
    public Vector3 Center;
    public float Radius;

    public SphereObstacle(Vector3 center, float radius)
    {
        Center = center;
        Radius = radius;

        centerObs = center;
        radiusObs = radius;        
    }

    public override Vector3 ClosestPoint(Vector3 point)
    {
        Vector3 direction = point - Center;
        if (direction.sqrMagnitude <= Radius * Radius)
        {
            // Point is inside the sphere
            return point;
        }
        else
        {
            // Point is outside the sphere
            return Center + direction.normalized * Radius;
        }
    }
    
    public override bool IsLineIntersecting(Vector3 start, Vector3 end)
    {
        Vector3 d = end - start;          // direction of the line segment
        Vector3 m = start - Center;       // vector from sphere center to segment start
        float a = Vector3.Dot(d, d);      // d·d
        float b = Vector3.Dot(m, d);      // m·d
        float c = Vector3.Dot(m, m) - Radius * Radius; // m·m - R^2

        // Solve quadratic a*t^2 + 2*b*t + c = 0
        float discriminant = b * b - a * c;
        if (discriminant < 0.0f) 
        {
            // No real intersection
            return false;
        }

        discriminant = Mathf.Sqrt(discriminant);
        // Two possible solutions: (-b ± sqrt(discriminant)) / a
        float t1 = (-b - discriminant) / a;
        float t2 = (-b + discriminant) / a;

        // We only need to check if there's any t in the [0, 1] interval
        if ((t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f))
            return true;

        return false;
    }
}
public class BoxObstacle : Obstacle
{
    public Vector3 Center;
    public Vector3 Size;
    public Quaternion Rotation;

    public BoxObstacle(Vector3 center, Vector3 size, Quaternion rotation)
    {
        Center = center;
        Size = size;
        Rotation = rotation;

        centerObs = center;
        radiusObs = size.magnitude;
    }

    public override Vector3 ClosestPoint(Vector3 point)
    {
        // Transform point into the box's local space
        Vector3 localPoint = Quaternion.Inverse(Rotation) * (point - Center);

        // Compute closest point in the axis-aligned bounding box
        Vector3 halfSize = Size * 0.5f;
        Vector3 clampedPoint = new Vector3(
            Mathf.Clamp(localPoint.x, -halfSize.x, halfSize.x),
            Mathf.Clamp(localPoint.y, -halfSize.y, halfSize.y),
            Mathf.Clamp(localPoint.z, -halfSize.z, halfSize.z)
        );

        // Transform back to world space
        return Center + Rotation * clampedPoint;
    }

    public override bool IsLineIntersecting(Vector3 start, Vector3 end)
    {
        // 1. Transform line segment into box-local space
        Quaternion invRot = Quaternion.Inverse(Rotation);
        Vector3 localStart = invRot * (start - Center);
        Vector3 localEnd   = invRot * (end - Center);

        Vector3 direction = localEnd - localStart;
        Vector3 halfSize = Size * 0.5f;

        // 2. AABB line-segment intersection in local space
        // One known approach is the "slab" method:
        float tMin = 0f;
        float tMax = 1f;

        // We'll define a function to handle each axis (x, y, z)
        bool CheckAxis(float startCoord, float dirCoord, float minVal, float maxVal, ref float t0, ref float t1)
        {
            // If direction is nearly 0, the line is parallel to that axis
            if (Mathf.Approximately(dirCoord, 0f))
            {
                // If startCoord is outside [minVal, maxVal], no intersection
                if (startCoord < minVal || startCoord > maxVal)
                    return false;
            }
            else
            {
                // Find intersection t values
                float tA = (minVal - startCoord) / dirCoord;
                float tB = (maxVal - startCoord) / dirCoord;

                // tA should be the lower value, tB the higher
                if (tA > tB)
                {
                    float tmp = tA;
                    tA = tB;
                    tB = tmp;
                }
                // Narrow the interval [t0, t1] based on [tA, tB]
                if (tA > t0) t0 = tA;
                if (tB < t1) t1 = tB;

                // If this is invalid, no intersection
                if (t0 > t1) return false;
            }
            return true;
        }

        // Check each axis
        if (!CheckAxis(localStart.x, direction.x, -halfSize.x, halfSize.x, ref tMin, ref tMax)) return false;
        if (!CheckAxis(localStart.y, direction.y, -halfSize.y, halfSize.y, ref tMin, ref tMax)) return false;
        if (!CheckAxis(localStart.z, direction.z, -halfSize.z, halfSize.z, ref tMin, ref tMax)) return false;

        // If we have a valid intersection range, we must ensure it overlaps with [0..1]
        return tMax >= 0f && tMin <= 1f;
    }
}
public class CylinderObstacle : Obstacle
{
    public Vector3 Center;
    public float Radius;
    public float Height;
    public Quaternion Rotation;

    public CylinderObstacle(Vector3 center, float radius, float height, Quaternion rotation)
    {
        Center = center;
        Radius = radius/2;
        Height = 2*height;
        Rotation = rotation;

        centerObs = center;
        radiusObs = radius;
    }

    public override Vector3 ClosestPoint(Vector3 point)
    {
        // Transform the point into the cylinder's local space
        Quaternion inverseRotation = Quaternion.Inverse(Rotation);
        Vector3 localPoint = inverseRotation * (point - Center);

        // Cylinder's axis is along the local Y-axis
        float halfHeight = Height * 0.5f;

        // Clamp the local point's Y coordinate to the cylinder's height
        float clampedY = Mathf.Clamp(localPoint.y, -halfHeight, halfHeight);

        // Compute the distance from the point to the cylinder's axis in the XZ plane
        Vector2 localPointXZ = new Vector2(localPoint.x, localPoint.z);
        float distanceXZ = localPointXZ.magnitude;

        Vector3 closestLocalPoint;

        if (distanceXZ <= Radius)
        {
            if (localPoint.y >= -halfHeight && localPoint.y <= halfHeight)
            {
                // The point is inside the cylinder
                closestLocalPoint = localPoint;
            }
            else
            {
                // The point is inside the side projection but outside along Y
                closestLocalPoint = new Vector3(localPoint.x, clampedY, localPoint.z);
            }
        }
        else
        {
            // Point is outside the cylinder's side projection
            Vector2 projectedPointXZ = localPointXZ.normalized * Radius;
            closestLocalPoint = new Vector3(projectedPointXZ.x, clampedY, projectedPointXZ.y);
        }

        // Transform back to world space
        return Center + Rotation * closestLocalPoint;
    }

    public override bool IsLineIntersecting(Vector3 start, Vector3 end)
    {
        // 1. Transform line to cylinder-local space
        Quaternion invRot = Quaternion.Inverse(Rotation);
        Vector3 localStart = invRot * (start - Center);
        Vector3 localEnd   = invRot * (end - Center);
        Vector3 d = localEnd - localStart;

        float halfHeight = Height * 0.5f;

        // We'll track any intersection t in [0..1]
        bool hit = false;

        // a) Check infinite side cylinder in XZ (ignore Y for a moment)
        // Cylinder in local: x^2 + z^2 = Radius^2
        // Parametric: x(t) = localStart.x + d.x * t
        //             z(t) = localStart.z + d.z * t
        // Solve x(t)^2 + z(t)^2 = R^2
        float A = d.x * d.x + d.z * d.z;
        float B = 2f * (localStart.x * d.x + localStart.z * d.z);
        float C = localStart.x * localStart.x + localStart.z * localStart.z - Radius * Radius;

        if (!Mathf.Approximately(A, 0f))
        {
            float disc = B * B - 4f * A * C;
            if (disc >= 0f)
            {
                float sqrtDisc = Mathf.Sqrt(disc);
                float t1 = (-B - sqrtDisc) / (2f * A);
                float t2 = (-B + sqrtDisc) / (2f * A);

                // Check if either t is in [0..1], and if so, whether the Y is within [-halfHeight, +halfHeight].
                if (CheckT(t1, localStart, d, halfHeight)) return true;
                if (CheckT(t2, localStart, d, halfHeight)) return true;
            }
        }
        else
        {
            // If A == 0, line is parallel to the XZ plane in terms of direction
            // So we won't get a side intersection (unless it's a degenerate case).
        }

        // b) Check top and bottom caps
        // Planes: y = +halfHeight and y = -halfHeight
        // param eqn: y(t) = localStart.y + d.y * t
        // Solve localStart.y + d.y * t = ±halfHeight => t = (±halfHeight - localStart.y) / d.y
        if (!Mathf.Approximately(d.y, 0f))
        {
            float tCapTop = ( halfHeight - localStart.y) / d.y;
            if (CheckCap(tCapTop, localStart, d, Radius)) return true;

            float tCapBottom = (-halfHeight - localStart.y) / d.y;
            if (CheckCap(tCapBottom, localStart, d, Radius)) return true;
        }

        return hit;

        // --- Local helper functions ---
        bool CheckT(float t, Vector3 ls, Vector3 dir, float h)
        {
            if (t >= 0f && t <= 1f)
            {
                float yVal = ls.y + dir.y * t;
                if (yVal >= -h && yVal <= h)
                {
                    return true;
                }
            }
            return false;
        }

        bool CheckCap(float t, Vector3 ls, Vector3 dir, float r)
        {
            if (t >= 0f && t <= 1f)
            {
                // x(t), z(t) in local
                float xVal = ls.x + dir.x * t;
                float zVal = ls.z + dir.z * t;
                if (xVal * xVal + zVal * zVal <= r * r)
                {
                    return true;
                }
            }
            return false;
        }
    }
}


public static class ClosestPointCalculator
{
    public static List<Obstacle> obstacles;
    public static List<Obstacle> obstaclesInRange;


    public static void selectObstacle(List<DroneFake> drones)
    {

        if (drones.Count == 0)
        {
            return;
        }

        Vector3 center = new Vector3(0, 0, 0);
        foreach (var drone in drones)
        {
            center += drone.position;
        }
        center /= drones.Count;

        float longestDistanceToCenter = 0;
        foreach (var drone in drones)
        {
            float distance = (drone.position - center).magnitude;
            if (distance > longestDistanceToCenter)
            {
                longestDistanceToCenter = distance;
            }
        }

        longestDistanceToCenter += DroneFake.avoidanceRadius;

    
        List<Obstacle> obstacleUpdated = new List<Obstacle>();
        foreach (Obstacle obstacle in obstacles)
        {
            if((center - obstacle.centerObs).magnitude <= longestDistanceToCenter + obstacle.radiusObs)
            {
                obstacleUpdated.Add(obstacle);
            }
        }

        obstaclesInRange = new List<Obstacle>(obstacleUpdated);
    }
    public static Vector3 ClosestPoint(Vector3 point)
    {
        Vector3 closestPoint = point;
        float minSqrDistance = float.MaxValue;
        string name = "";


        lock (obstaclesInRange)
        {
            foreach (var obstacle in obstaclesInRange)
            {
                Vector3 cp = obstacle.ClosestPoint(point);
                float sqrDist = (cp - point).sqrMagnitude;
                if (sqrDist < minSqrDistance)
                {
                    minSqrDistance = sqrDist;
                    closestPoint = cp;
                    name = obstacle.GetType().Name;
                }
            }
        }

        // Debug.Log($"Closest point to {point} is {closestPoint} on {name}");
        return closestPoint;
    }

    public static List<Vector3> ClosestPointsWithinRadius(Vector3 refPoint, float radius)
    {
        List<Vector3> closestPoints = new List<Vector3>();
        lock(obstaclesInRange)
        {
            foreach (var obstacle in obstaclesInRange)
            {
                Vector3 cp = obstacle.ClosestPoint(refPoint);
                if ((cp - refPoint).sqrMagnitude <= radius * radius)
                {
                    closestPoints.Add(cp);
                }
            }
        }

        return closestPoints;
    }

    public static bool IsLineIntersecting(Vector3 start, Vector3 end)
    {
        lock(obstaclesInRange)
        {
            foreach (var obstacle in obstaclesInRange)
            {
                if (obstacle.IsLineIntersecting(start, end))
                {
                    if(obstacle.transparent)
                    {
                        return false;
                    }
                    
                    return true;
                }
            }
        }

        return false;
    }
}
