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
    string pose;
    byte[] posStr;
    
    // Socket
    //Socket sock;
    //IPEndPoint ep;
    //Socket clientSock;
    
    // Thread for socket
    // https://stackoverflow.com/questions/36526332/simple-socket-server-in-unity
    Thread SocketThread;
    volatile bool keepReading = false;

    /*
    void Start()
    {
        Application.runInBackground = true;
        startServer();
    }
    */
    
    void startServer()
    {
        SocketThread = new Thread(networkCode);
        SocketThread.IsBackground = true;
        SocketThread.Start();
    }

    void flagRun()
    {

    }

    void flagStop()
    {

    }


    Socket handler;
    Socket listner;
    void networkCode()
    {
        string data;
        byte[] bytes = new byte[1024];
        listner = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        IPEndPoint localEndPoint = new IPEndPoint(IPAddress.Any, Convert.ToInt16(Port));

        try
        {
            listner.Bind(localEndPoint);
            listner.Listen(10);
            connected = true;

            while (true)
            {
                //keepReading = true;

                Debug.Log("Waiting for Connection");
                handler = listner.Accept();
                connection = "Connected";
                data = null;

                while (keepReading)
                {
                    handler.Send(posStr);
                    /*
                    bytes = new byte[1024];
                    int bytesRec = handler.Receive(bytes);
                    Debug.Log("Received");

                    if (bytesRec <= 0)
                    {
                        keepReading = false;
                        handler.Disconnect(true);
                        break;
                    }

                    data += Encoding.ASCII.GetString(bytes, 0, bytesRec);
                    if (data.IndexOf("<EOF>") < -1)
                    {
                        break;
                    }
                    */

                    Thread.Sleep(50);
                }
                Thread.Sleep(50);
            }
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());

            keepReading = false;
            connected = false;
            connection = "Server Stopped";
            runButton = "Run";
            status = "Stop";
            keepReading = false;
            running = false;
            handler.Disconnect(false);
            listner.Disconnect(false);
            stopServer();
        }
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

        listner.Disconnect(false);
    }

    void OnDestroy()
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
                stopServer();
                runButton = "Run";
                running = false;
                connection = "Server Stopped";
                status = "Stop";
                keepReading = false;
            }

            else
            {
                runButton = "Stop";
                running = true;
                connection = "Waiting client..."; 
                status = "Running";
                //Start();
                keepReading = true;
                Application.runInBackground = true;
                startServer();
            }
        }  
    }
    
    void Update()
    {
        //Debug.Log();
        pose = "" + SceneView.lastActiveSceneView.camera.transform.position + "/" + SceneView.lastActiveSceneView.camera.transform.rotation; 
        posStr = Encoding.ASCII.GetBytes(pose);
        Repaint();
    }
}
