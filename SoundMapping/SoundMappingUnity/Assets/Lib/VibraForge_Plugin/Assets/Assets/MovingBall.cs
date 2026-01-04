using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingBall : MonoBehaviour
{
    public float speed = 2f; // Speed of the sphere
    private float startZ = 5f;
    private float endZ = -5f;
    private bool movingForward = true;

    // Start is called before the first frame update
    void Start()
    {
        // Initialize the sphere's position
        transform.position = new Vector3(0f, 0f, startZ);
    }

    // Update is called once per frame
    void Update()
    {
        // Calculate movement step
        float step = speed * Time.deltaTime;

        // Move the sphere along the z-axis
        if (movingForward)
        {
            transform.Translate(0f, 0f, -step);
            if (transform.position.z <= endZ)
            {
                movingForward = false;
            }
        }
        else
        {
            transform.Translate(0f, 0f, step);
            if (transform.position.z >= startZ)
            {
                movingForward = true;
            }
        }
    }

    // Alternatively, detect trigger events if using triggers
    void OnTriggerEnter(Collider other)
    {
        // Debug.Log("Trigger detected with " + other.gameObject.name);
        VibraForge.SendCommand(0, 1, 7, 2);
    }

    void OnTriggerExit(Collider other)
    {
        // Debug.Log("Trigger exited with " + other.gameObject.name);
        VibraForge.SendCommand(0, 0, 7, 2);
    }
}
