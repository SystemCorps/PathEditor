using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEditor;

public class SceneViewCameraTest : EditorWindow {
    [MenuItem ("Window/SceneViewCameraTest")]
    static void Init () {
        // Get existing open window or if none, make a new one:
        SceneViewCameraTest window = (SceneViewCameraTest)EditorWindow.GetWindow (typeof (SceneViewCameraTest));
        //Socket sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        //IPEndPoint ep = new IPEndPoint(IPAddress.Any, 8001);
        //sock.Bind(ep);
        //sock.Listen(100);
        //Socket clientSock = sock.Accept();
    }

    void OnGUI () {
    EditorGUILayout.TextField("Scene Camera Position", ""+SceneView.lastActiveSceneView.camera.transform.position);
    EditorGUILayout.TextField("Scene Camera Rotation", ""+SceneView.lastActiveSceneView.camera.transform.rotation);
    }

    void Update() {//
    Repaint();
    //string data = ""+SceneView.lastActiveSceneView.camera.transform.position;
    //Debug.Log("Data: " + data);
    //Byte[] _data = Encoding.Default.GetBytes(data);
    //clientSock.Send(_data);
    }

}


