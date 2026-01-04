using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;

public class SwarmDisconnection : MonoBehaviour
{
    // Start is called before the first frame update
    public int numberOfDronesLost = 4;
    public List<string> options = new List<string> { "Bip", "White Noise", "Bap", "Bop", "Beep" };
    private List<GameObject> drones
    {
        get
        {
            List<GameObject> drone = new List<GameObject>();
            foreach (Transform child in swarmModel.swarmHolder.transform)
            {
                if (dronesID.Contains(child.GetComponent<DroneController>().droneFake.id))
                {
                    drone.Add(child.gameObject);
                }
            }
            return drone;
        }
    }
    public List<AudioClip> droneSounds = new List<AudioClip>();
    public int selectedOption;

    public int radiusMin = 30;
    public int radiusMax = 60;

    private Coroutine currentSoundCoroutine = null;
    static public List<int> dronesID = new List<int>();
    public static bool playingSound = false;

    private static GameObject gm;

    public static bool hasChanged = false;

    bool playSpreadness
    {
        get
        {
            return LevelConfiguration._Audio_spreadness;
        }
    }


    void Start()
    {
        gm = this.gameObject;
    }

    public static void CheckDisconnection()
    {
        List<int> dronesIDAnalysis = new List<int>();
        
        NetworkCreator network = swarmModel.network;

        foreach (DroneFake drone in network.drones)
        {
            if (!network.IsInMainNetwork(drone))
            {
                dronesIDAnalysis.Add(drone.id);
            }
        }

        // if(!LevelConfiguration._Audio_isolation)
        // {
        //     return; 
        // }

        //chedck if every element of dronesIDAnalysis is in dronesID
        bool changeOfList = false;
        if(dronesIDAnalysis.Count != dronesID.Count)
        {
            changeOfList = true;
        }
        else
        {
            foreach (int id in dronesIDAnalysis)
            {
                if (!dronesID.Contains(id))
                {
                    changeOfList = true;
                    break;
                }
            }
        }
        dronesID = dronesIDAnalysis;

        
        if(!LevelConfiguration._Audio_isolation)
        {
            return; 
        }

        if(changeOfList)
        {
            hasChanged = true;
            gm.GetComponent<SwarmDisconnection>().CancelInvoke("ResetHasChanged");
            gm.GetComponent<SwarmDisconnection>().Invoke("ResetHasChanged", 3f);
            
            
            if(dronesIDAnalysis.Count == 0)
            {
//                print("Stop sound");
                gm.GetComponent<SwarmDisconnection>().StopSound();
            }
            else
            {
       //         print("Disconnection detected");
                gm.GetComponent<SwarmDisconnection>().StopAndPlaySound(0);
            }
        }
        
    }

    public void ResetHasChanged()
    {
        hasChanged = false;
    }

    public void StopAndPlaySound(int clipIndex)
    {
        if (!playingSound) // if he wasnrt playing any sound and wa sjust waiting before relaunching the sound then stop the coroutine
        {
            StopSound();
        }
    }

    

    public IEnumerator disconnectionSound()
    {
        while (true)
        {
            
            playingSound = true;
            foreach (GameObject drone in drones)
            {
                drone.GetComponent<AudioSource>().clip = droneSounds[0];
                drone.GetComponent<AudioSource>().Play();
                yield return new WaitForSeconds(0.4f);
                drone.GetComponent<AudioSource>().Stop();
                yield return new WaitForSeconds(0.1f);
            }
            yield return new WaitForSeconds(0.01f);
            playingSound = false;

            yield return new WaitForSeconds(2f);
        }
    }

    public void StopSound()
    {
        if (currentSoundCoroutine != null)
        {
            StopCoroutine(currentSoundCoroutine);
        }

        foreach (GameObject drone in drones)
        {
            if (drone.GetComponent<AudioSource>().isPlaying)
            {
                drone.GetComponent<AudioSource>().Stop();
            }
        }
        

        playingSound = false;
        currentSoundCoroutine =  StartCoroutine(disconnectionSound());
    }

    public void PlaySound()
    {
        if(selectedOption == 0)
        {
            // Debug.Log("Playing Bip sound");
        }
        else if(selectedOption == 1)
        {
            // Debug.Log("Playing White Noise sound");
        }
        else if(selectedOption == 2)
        {
            // Debug.Log("Playing Bap sound");
        }
        else if(selectedOption == 3)
        {
            // Debug.Log("Playing Bop sound");
        }
        else if(selectedOption == 4)
        {
            // Debug.Log("Playing Beep sound");
        }
    }

    public void RegenerateSwarm()
    {

        return;


        foreach (GameObject drone in drones)
        {
            Destroy(drone);
        }

        for (int i = 0; i < numberOfDronesLost; i++)
        {
            GameObject drone = GameObject.CreatePrimitive(PrimitiveType.Cube);
            float randomRadius = Random.Range(radiusMin, radiusMax);
            float randomAngle = Random.Range(0, 360);
            drone.transform.position = new Vector3(randomRadius * Mathf.Cos(randomAngle), 0, randomRadius * Mathf.Sin(randomAngle));
            
            //add audio source
            drone.AddComponent<AudioSource>();
            drone.GetComponent<AudioSource>().spatialBlend = 1;
            //make linear falloff
            drone.GetComponent<AudioSource>().rolloffMode = AudioRolloffMode.Linear;
            drones.Add(drone);
        }
    }

    public void RandomizeSwarm()
    {
        // Debug.Log("Randomizing swarm");
    }
}
