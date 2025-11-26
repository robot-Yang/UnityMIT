using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VFX;
using System;
using UnityEditor.PackageManager;
using System.Net.WebSockets;

[Serializable]
public class VibraForge : MonoBehaviour
{
    private static TcpSender sender;
    private static Dictionary<string, int> command;

    void Start()
    {
        sender = this.GetComponent<TcpSender>();
        command = new Dictionary<string, int>()
        {
            { "addr", -1 },
            { "mode", 0 },
            { "duty", 0 },
            { "freq", 2 }
        };
    }

    public static string DictionaryToString(Dictionary<string, int> dictionary)
    {
        string dictionaryString = "{";
        foreach (KeyValuePair<string, int> keyValues in dictionary)
        {
            dictionaryString += "\"" + keyValues.Key + "\": " + keyValues.Value + ", ";
        }
        return dictionaryString.TrimEnd(',', ' ') + "}";
    }

    public static void SendCommand(int addr, int mode, int duty, int freq)
    {
        command["addr"] = addr;
        command["mode"] = mode;
        command["duty"] = duty;
        command["freq"] = freq;
        sender.SendData(DictionaryToString(command));

        saveInfoToJSON.addHapticRecord(addr, duty, freq);
    }


    //on quit
    void OnApplicationQuit()
    {
        Reset();
        
        //wait for 1 second
   //     System.Threading.Thread.Sleep(1000);

    }

    public static void Reset()
    {
        for(int i = 0; i < 210; i++)
        {
            SendCommand(i, 0, 0, 0);
            if(i % 20 == 0)
            {
                System.Threading.Thread.Sleep(100);
            }
        }
    }
}