using UnityEngine;

public class ControllerTest : MonoBehaviour
{
    public bool printDebug = false;
    void Update()
    {
        if (!printDebug)
            return;
            
        // Check for button presses
        for (int i = 0; i < 20; i++) // Assuming up to 20 buttons (adjust if needed)
        {
            // if (Input.GetKey("joystick button " + i))
            // {
            //     Debug.Log("Button " + i + " pressed");
            // }
        }

        // Check for joystick axis movements
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");
        
        float rightStickHorizontal = Input.GetAxis("JoystickRightHorizontal");
        float rightStickVertical = Input.GetAxis("JoystickRightVertical");

        float leftrightTrigger = Input.GetAxis("LR");

        // if (horizontal != 0)
        //     Debug.Log("Left Stick Horizontal: " + horizontal);
        // if (vertical != 0)
        //     Debug.Log("Left Stick Vertical: " + vertical);
        // if (rightStickHorizontal != 0)
        //     Debug.Log("Right Stick Horizontal: " + rightStickHorizontal);
        // if (rightStickVertical != 0)
        //     Debug.Log("Right Stick Vertical: " + rightStickVertical);
        // if (leftrightTrigger != 0)
        //     Debug.Log("Left/Right Trigger: " + leftrightTrigger);

    }
}
