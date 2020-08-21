using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEditor;

public class OpenSocket : EditorWindow
{
    string runButton = "Run";
    //string disconButton = "Disconnect";
    bool running = false;
    bool connected = false;
    string status = "Stop";
    string connection = "Server Stopped";
    string Port = "8001";
    
    // Socket
    //Socket sock;
    //IPEndPoint ep;
    //Socket clientSock;
    
    // Thread for socket
    // https://stackoverflow.com/questions/36526332/simple-socket-server-in-unity
    Thread SocketThread;
    volatile bool keepReading = false;
    
    void Start()
    {
        Application.runInBackground = true;
        startServer();
    }
    
    void startServer()
    {
        SocketThread = new Thread(networkCode);
        SocketThread.IsBackground = true;
        SocketThread.Start();
    }
    
    
    

    void stopServer()
    {
        keepReading = false;

        //stop thread
        if (SocketThread != null)
        {
            SocketThread.Abort();
        }

        if (handler != null && handler.Connected)
        {
            handler.Disconnect(false);
            Debug.Log("Disconnected!");
        }
    }

    void OnDisable()
    {
        stopServer();
    }

    
    
    [MenuItem ("Custom/OpenSocket")]
    static void Init()
    {
        OpenSocket window = (OpenSocket)EditorWindow.GetWindow (typeof (OpenSocket));
    }
    
    void OnGUI()
    {
        if (running)
        {
            EditorGUILayout.LabelField("Port: ", Port);
        }
        else
        {
            Port = EditorGUILayout.TextField("Port: ", Port);
        }
        
        
        EditorGUILayout.LabelField("Server: ", status);
        EditorGUILayout.LabelField("Connection: ", connection);
        
        if (GUILayout.Button(runButton))
        {
            if (running)
            {
            runButton = "Run";
            running = false;
            connection = "Server Stopped";
            status = "Stop";
            }
            else
            {
            runButton = "Stop";
            running = true;
            connection = "Waiting client"; 
            status = "Running";
            }
        }  
    }
    
    void Update()
    {
        //Debug.Log();
        Repaint();
    }
}
