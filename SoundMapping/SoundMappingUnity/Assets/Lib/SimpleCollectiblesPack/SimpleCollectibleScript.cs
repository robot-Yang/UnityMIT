 using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

[RequireComponent(typeof(AudioSource))]
public class SimpleCollectibleScript : MonoBehaviour {

	public enum CollectibleTypes {NoType, Type1, Type2, Type3, Type4, Type5}; // you can replace this with your own labels for the types of collectibles in your game!

	public CollectibleTypes CollectibleType; // this gameObject's type

	public bool rotate; // do you want it to rotate?

	public float rotationSpeed;

	public AudioClip collectSound;

	public GameObject collectEffect;

	// Use this for initialization
	void Start () {
		if(LevelConfiguration._startEmbodied)
		{
			transform.rotation = Quaternion.Euler(0, 0, 0);
			//scale box collider to 1 6 1
			BoxCollider boxCollider = GetComponent<BoxCollider>();
			boxCollider.size = new Vector3(1, 6, 1);
		}
		else
		{
			transform.rotation = Quaternion.Euler(0, 0, 90);
			BoxCollider boxCollider = GetComponent<BoxCollider>();
			boxCollider.size = new Vector3(6, 1, 1);
		}
	}
	
	// Update is called once per frame
	void Update () {

		if (rotate)
			transform.Rotate (Vector3.up * rotationSpeed * Time.deltaTime, Space.World);


	}

	// SimpleCollectibleScript.cs : Code à insérer dans OnTriggerEnter(Collider other)

void OnTriggerEnter(Collider other)
{
    if (other.tag == "Drone") {
        DroneController droneController = other.gameObject.GetComponent<DroneController>();
        
        if (droneController != null && droneController.droneFake != null)
        {
            int droneId = droneController.droneFake.id;
            
            // 💡 NOUVEL APPEL AU LOG DEDICACE (StarLogger)
            StarLogger.RecordStar(
                starName: this.name, 
                timeCollected: Timer.elapsedTime, // Assurez-vous que Timer est accessible
                droneId: droneId, 
                position: transform.position
            );
            
            Collect ();
        }
    }
}

	public void Collect()
	{
		if(collectSound)
		{
			AudioSource audioSource = GetComponent<AudioSource>();
			audioSource.clip = collectSound;
			audioSource.volume = 0.2f;
			audioSource.Play();
		//	Destroy(audioSource, collectSound.length);
		}
		if(collectEffect)
		{
			GameObject sound = Instantiate(collectEffect, transform.position, Quaternion.identity);
			AudioSource audioSource = sound.AddComponent<AudioSource>();
			audioSource.clip = collectSound;
			audioSource.spatialBlend = 1;
			audioSource.rolloffMode = AudioRolloffMode.Linear;
			audioSource.maxDistance = 4;
			audioSource.clip = collectSound;
			audioSource.volume = 0.35f;
			audioSource.Play();
			sound.GetComponent<AudioSource>().Play();
			Destroy(sound, collectSound.length);
		}

		//Below is space to add in your code for what happens based on the collectible type

		if (CollectibleType == CollectibleTypes.NoType) {

			//Add in code here;

			Debug.Log ("Do NoType Command");

//			Debug.Log ("Do NoType Command");
		}
		if (CollectibleType == CollectibleTypes.Type1) {

			//Add in code here;

		}
		if (CollectibleType == CollectibleTypes.Type2) {

			//Add in code here;

		}
		if (CollectibleType == CollectibleTypes.Type3) {

			//Add in code here;

		}
		if (CollectibleType == CollectibleTypes.Type4) {

			//Add in code here;

		}
		if (CollectibleType == CollectibleTypes.Type5) {

			//Add in code here;

		}

		Destroy (gameObject);
	}
}
