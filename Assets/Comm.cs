using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public class Comm
{
    //public static string test="is it?";
    static string test="go!!!";
    static Socket sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
    static IPEndPoint ep = new IPEndPoint(IPAddress.Any, 8001);
    static Socket clientSock;
    static string test2;
    
    static Comm ()
    {
        //IPEndPoint ep = new IPEndPoint(IPAddress.Any, 8001);
        //sock.Bind(ep);
        //sock.Listen(100);
        //clientSock = sock.Accept();
        test2="update?!";
        Debug.Log("Startup!!");
        EditorApplication.update += Update;
    }

    static void Update ()
    {
        //string data = ""+SceneView.lastActiveSceneView.camera.transform.position;
        //Byte[] _data = Encoding.Default.GetBytes(data);
        //clientSock.Send(_data);
        //string data;
        //data = Comm.test2;
        //Debug.Log(data);
    }
}
