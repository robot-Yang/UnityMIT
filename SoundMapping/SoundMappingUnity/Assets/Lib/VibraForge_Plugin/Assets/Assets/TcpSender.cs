using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WebSocketSharp;

public class TcpSender : MonoBehaviour
{
    public static bool bluetoothon = false;
    public static bool tcpserveron = false;
    private WebSocket ws;
    public bool isConnected = false;

    void Start()
    {
        ws = new WebSocket("ws://localhost:9052");

        // Enable detailed logging
        ws.Log.Level = LogLevel.Trace;
        ws.Log.Output = (logData, s) => // Debug.Log($"{logData.Level}: {logData.Message}");

        ws.OnOpen += (sender, e) =>
        {
            // Debug.Log("WebSocket connected.");
            isConnected = true;
            tcpserveron = true;
        };

        ws.OnMessage += (sender, e) =>
        {
            if(e.Data == "C")
            {
                bluetoothon = true;
            }
            else
            {
                bluetoothon = false;
            }
        };

        ws.OnError += (sender, e) =>
        {
            // Debug.LogError("WebSocket error: " + e.Message);
            if (e.Exception != null)
            {
                // Debug.LogError("Exception: " + e.Exception);
            }
        };

        ws.OnClose += (sender, e) =>
        {
            // Debug.Log("WebSocket closed.");
            isConnected = false;
            tcpserveron = false;
        };

        try
        {
            ws.Connect();
        }
        catch (System.Exception ex)
        {
            // Debug.LogError("Exception during WebSocket connection: " + ex.Message);
        }
    }

    void Update()
    {
        // Example input for testing
        if (Input.GetKeyDown(KeyCode.Space))
        {
            SendData("{\"addr\":1,\"mode\":1,\"duty\":10,\"freq\":5}");
        }
    }

    public void SendData(string data)
    {
        if (isConnected)
        {
            ws.Send(data);
        }
    }

    private void OnApplicationQuit()
    {
        if (ws != null)
        {
            ws.Close();
            // Debug.Log("WebSocket closed.");
        }
    }
}
