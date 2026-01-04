using UnityEngine;
using UnityEngine.UI;
using WebSocketSharp;

public class WebSocketClient : MonoBehaviour
{
    private WebSocket ws;

    void Start()
    {
        ws = new WebSocket("ws://localhost:9052");

        ws.OnOpen += (sender, e) =>
        {
            UpdateStatus("Connected to server.");
        };

        ws.OnMessage += (sender, e) =>
        {
            // Debug.Log("Message received: " + e.Data);
        };

        ws.OnError += (sender, e) =>
        {
            UpdateStatus("Error: " + e.Message);
        };

        ws.OnClose += (sender, e) =>
        {
            UpdateStatus("Connection closed oh noooo.");
        };

        ws.Connect();

    }

    void UpdateStatus(string message)
    {

        // Debug.Log(message);
    }

    public void SendMessageToServer(string message)
    {
        if (ws != null && ws.IsAlive)
        {
            ws.Send(message);
            // Debug.Log("Sent: " + message);
        }
        else
        {
            UpdateStatus("Cannot send message. Not connected to server.");
        }
    }

    void OnApplicationQuit()
    {
        if (ws != null)
        {
            ws.Close();
        }
    }

    void OnDestroy()
    {
        if (ws != null)
        {
            ws.Close();
        }
    }
}
