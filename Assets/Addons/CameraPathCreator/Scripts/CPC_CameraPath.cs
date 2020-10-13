using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

[System.Serializable]
public class CPC_Visual
{
    public Color pathColor = Color.green;
    public Color inactivePathColor = Color.gray;
    public Color frustrumColor = Color.white;
    public Color handleColor = Color.yellow;
}

public enum CPC_ECurveType
{
    EaseInAndOut,
    Linear,
    Custom
}

public enum CPC_EAfterLoop
{
    Continue,
    Stop
}

[System.Serializable]


public class TrajectoryInfo
{
    public float timeInit;
    public float timeConst;
    public float timeFinal;
    public float travelInit;
    public float travelConst;
    public float travelFinal;
    public float speedConst;
    public float speedMean;
    public float accInitSign;

    public TrajectoryInfo(float ti, float tc, float tf, float trvi, float trvc, float trvf, float speedc, float speedm)
    {
        timeInit = ti;
        timeConst = tc;
        timeFinal = tf;
        travelInit = trvi;
        travelConst = trvc;
        travelFinal = trvf;
        speedConst = speedc;
        speedMean = speedm;
        accInitSign = 1.0F;
    }
}

public class CPC_Point
{

    public Vector3 position;
    public Quaternion rotation;
    public Vector3 handleprev;
    public Vector3 handlenext;
    public CPC_ECurveType curveTypeRotation;
    public AnimationCurve rotationCurve;
    public CPC_ECurveType curveTypePosition;
    public AnimationCurve positionCurve;
    public bool chained;
    // Customized
    public bool hold;
    public float holdTime;
    public float startTime;
    public float execTime;
    public float minExecTime;
    public float finalSpeed;
    public TrajectoryInfo trajectoryInfo;

    public CPC_Point(Vector3 pos, Quaternion rot)
    {
        position = pos;
        rotation = rot;
        handleprev = Vector3.back;
        handlenext = Vector3.forward;
        curveTypeRotation = CPC_ECurveType.EaseInAndOut;
        rotationCurve = AnimationCurve.EaseInOut(0,0,1,1);
        curveTypePosition = CPC_ECurveType.Linear;
        positionCurve = AnimationCurve.Linear(0,0,1,1);
        chained = true;
        hold = false;
        holdTime = 0.0F;
        startTime = 0.0F;
        execTime = 0.0F;
        minExecTime = 0.0F;
        finalSpeed = 0.0F;
        trajectoryInfo = new TrajectoryInfo(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    }
}

public class RobotPath
{
    public Vector3 position;
    public Quaternion rotation;
    public float time;
    public float mobileSpeed;
    public float travelRange;
    public float relTimeInWay;

    public RobotPath(float t, Vector3 pos, Quaternion rot, float spd, float travel)
    {
        time = t;
        position = pos;
        rotation = rot;
        mobileSpeed = spd;
        travelRange = travel;
        relTimeInWay = 0.0F;
    }
}

public class CPC_CameraPath : MonoBehaviour
{

    public bool useMainCamera = true;
    public Camera selectedCamera;
    public bool lookAtTarget = false;
    public Transform target;
    public bool playOnAwake = false;
    public float playOnAwakeTime = 10;
    public List<CPC_Point> points = new List<CPC_Point>();
    public CPC_Visual visual;
    public bool looped = false;
    public bool alwaysShow = true;
    public CPC_EAfterLoop afterLoop = CPC_EAfterLoop.Continue;

    private int currentWaypointIndex;
    private float currentTimeInWaypoint;
    private float timePerSegment;

    private bool paused = false;
    private bool playing = false;


    // Robot Path
    public List<RobotPath> rpaths = new List<RobotPath>();
    //public List<float> execTime = new List<float>();
    public string rpathStr;
    //public RobotPath temp;


    void Start ()
    {
        
        if (Camera.main == null) { Debug.LogError("There is no main camera in the scene!"); }
	    if (useMainCamera)
	        selectedCamera = Camera.main;
	    else if (selectedCamera == null)
	    {
            selectedCamera = Camera.main;
            Debug.LogError("No camera selected for following path, defaulting to main camera");
        }

	    if (lookAtTarget && target == null)
	    {
	        lookAtTarget = false;
            Debug.LogError("No target selected to look at, defaulting to normal rotation");
        }

	    foreach (var index in points)
	    {
            if (index.curveTypeRotation == CPC_ECurveType.EaseInAndOut) index.rotationCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);
            if (index.curveTypeRotation == CPC_ECurveType.Linear) index.rotationCurve = AnimationCurve.Linear(0, 0, 1, 1);
            if (index.curveTypePosition == CPC_ECurveType.EaseInAndOut) index.positionCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);
            if (index.curveTypePosition == CPC_ECurveType.Linear) index.positionCurve = AnimationCurve.Linear(0, 0, 1, 1);
        }

        if (playOnAwake)
            PlayPath(playOnAwakeTime);
    }

    /// <summary>
    /// Plays the path
    /// </summary>
    /// <param name="time">The time in seconds how long the camera takes for the entire path</param>
    public void PlayPath(float time)
    {
        if (time <= 0) time = 0.001f;
        paused = false;
        playing = true;
        StopAllCoroutines();
        StartCoroutine(FollowPath(time));
    }

    /// <summary>
    /// Stops the path
    /// </summary>
    public void StopPath()
    {
        playing = false;
        paused = false;
        StopAllCoroutines();
    }

    /// <summary>
    /// Allows to change the time variable specified in PlayPath(float time) on the fly
    /// </summary>
    /// <param name="seconds">New time in seconds for entire path</param>
    public void UpdateTimeInSeconds(float seconds)
    {
        int holdCount = 0;
        for (int i=0; i < points.Count; i++)
        {
            if (points[i].hold) holdCount++;
        }

        if (holdCount == 0)
        {
            timePerSegment = seconds / ((looped) ? points.Count : points.Count - 1);
        }
        else
        {
            timePerSegment = seconds / ((looped) ? (points.Count - holdCount) : (points.Count - 1 - holdCount));
        }
    }

    /// <summary>
    /// Pauses the camera's movement - resumable with ResumePath()
    /// </summary>
    public void PausePath()
    {
        paused = true;
        playing = false;
    }

    /// <summary>
    /// Can be called after PausePath() to resume
    /// </summary>
    public void ResumePath()
    {
        if (paused)
            playing = true;
        paused = false;
    }

    /// <summary>
    /// Gets if the path is paused
    /// </summary>
    /// <returns>Returns paused state</returns>
    public bool IsPaused()
    {
        return paused;
    }

    /// <summary>
    /// Gets if the path is playing
    /// </summary>
    /// <returns>Returns playing state</returns>
    public bool IsPlaying()
    {
        return playing;
    }

    /// <summary>
    /// Gets the index of the current waypoint
    /// </summary>
    /// <returns>Returns waypoint index</returns>
    public int GetCurrentWayPoint()
    {
        return currentWaypointIndex;
    }

    /// <summary>
    /// Gets the time within the current waypoint (Range is 0-1)
    /// </summary>
    /// <returns>Returns time of current waypoint (Range is 0-1)</returns>
    public float GetCurrentTimeInWaypoint()
    {
        return currentTimeInWaypoint;
    }

    /// <summary>
    /// Sets the current waypoint index of the path
    /// </summary>
    /// <param name="value">Waypoint index</param>
    public void SetCurrentWayPoint(int value)
    {
        currentWaypointIndex = value;
    }

    /// <summary>
    /// Sets the time in the current waypoint 
    /// </summary>
    /// <param name="value">Waypoint time (Range is 0-1)</param>
    public void SetCurrentTimeInWaypoint(float value)
    {
        currentTimeInWaypoint = value;
    }

    /// <summary>
    /// When index/time are set while the path is not playing, this method will teleport the camera to the position/rotation specified
    /// </summary>
    public void RefreshTransform()
    {
        selectedCamera.transform.position = GetBezierPosition(currentWaypointIndex, currentTimeInWaypoint);
        if (!lookAtTarget)
            selectedCamera.transform.rotation = GetLerpRotation(currentWaypointIndex, currentTimeInWaypoint);
        else
            selectedCamera.transform.rotation = Quaternion.LookRotation((target.transform.position - selectedCamera.transform.position).normalized);
    }

    
    public void PathToStr()
    {
        rpathStr = "";
        for (int i = 0; i < rpaths.Count; i++)
        {
            string temp = "" + rpaths[i].time + "," + rpaths[i].position.x + "," + rpaths[i].position.y + "," + rpaths[i].position.z
                + "," + rpaths[i].rotation.w + "," + rpaths[i].rotation.x + "," + rpaths[i].rotation.y + "," + rpaths[i].rotation.z + "\n";
            temp = temp.Replace("(", "").Replace(")", "");
            rpathStr += temp;
        }
    }

    public void PathSave(string directory)
    {
        File.WriteAllText(directory, rpathStr);
    }



    void UpdateTrajectoryInfo(int index, float maxAcc)
    {
        float Length = CalLengthBtwWaypoint(index);
        float execTime = points[index].execTime;
        float curMeanSpeed = Length / execTime;
        float preMeanSpeed = 0.0F;
        float Acc;
        float sign;
        float initSpeed = 0.0F;
        float timeInit;
        float timeConst;
        float timeFinal;
        float speedConst;
        float travelInit;
        float travelConst;
        float travelFinal;

        if (index > 0)
        {
            if (!points[index-1].hold)
            {
                preMeanSpeed = points[index - 1].trajectoryInfo.speedMean;
                initSpeed = points[index - 1].trajectoryInfo.speedConst;
            }
        }

        
        if (curMeanSpeed > preMeanSpeed)
        {
            Acc = Mathf.Abs(maxAcc);
            sign = -1.0F;
        }
        else
        {
            Acc = -Mathf.Abs(maxAcc);
            sign = +1.0F;
        }

        if ((index == points.Count-1) || points[index+1].hold)
        {
            if (curMeanSpeed > preMeanSpeed)
            {
                float term1 = (maxAcc * execTime - initSpeed) / (2 * maxAcc);
                float term2 = Mathf.Sqrt(Mathf.Pow(maxAcc * execTime, 2) + 2 * maxAcc * initSpeed * execTime - 4 * Length * maxAcc - Mathf.Pow(initSpeed, 2)) / (2 * maxAcc);
                timeInit = term1 - term2;
                
            }
            else
            {
                timeInit = (2 * maxAcc * initSpeed * execTime - 2 * Length * maxAcc - Mathf.Pow(initSpeed, 2)) / (2 * maxAcc * (maxAcc * execTime - initSpeed));
            }

            speedConst = initSpeed + (Acc * timeInit);
            timeFinal = speedConst / maxAcc;
            timeConst = execTime - timeInit - timeFinal;

            travelInit = (initSpeed * timeInit) + Acc * (Mathf.Pow(timeInit, 2) / 2);
            travelConst = speedConst * timeConst;
            travelFinal = maxAcc * (Mathf.Pow(timeFinal, 2) / 2);

            points[index].trajectoryInfo.timeInit = timeInit;
            points[index].trajectoryInfo.timeConst = timeConst;
            points[index].trajectoryInfo.timeFinal = timeFinal;
            points[index].trajectoryInfo.speedConst = speedConst;
            points[index].trajectoryInfo.travelInit = travelInit;
            points[index].trajectoryInfo.travelConst = travelConst;
            points[index].trajectoryInfo.travelFinal = travelFinal;

            points[index].trajectoryInfo.speedMean = curMeanSpeed;
            points[index].trajectoryInfo.accInitSign = -sign;
        }
        else
        {
            timeInit = (Acc * execTime + sign * Mathf.Sqrt(Acc * (Acc * Mathf.Pow(execTime, 2) + 2 * execTime * initSpeed - 2 * Length))) / Acc;
            timeConst = execTime - timeInit;
            speedConst = initSpeed + (Acc * timeInit);

            travelInit = (initSpeed * timeInit) + Acc * (Mathf.Pow(timeInit, 2) / 2);
            travelConst = speedConst * timeConst;

            points[index].trajectoryInfo.timeInit = timeInit;
            points[index].trajectoryInfo.timeConst = timeConst;
            points[index].trajectoryInfo.speedConst = speedConst;
            points[index].trajectoryInfo.travelInit = travelInit;
            points[index].trajectoryInfo.travelConst = travelConst;

            points[index].trajectoryInfo.speedMean = curMeanSpeed;
            points[index].trajectoryInfo.accInitSign = -sign;
        }
    }





    float CalLengthBtwWaypoint(int currentIndex)
    {
        // UnityEditor.Handles.DrawBezier(index.position, indexNext.position, index.position + index.handlenext, indexNext.position + indexNext.handleprev,((UnityEditor.Selection.activeGameObject == gameObject) ? visual.pathColor : visual.inactivePathColor), null, 5);
        Vector3 startPosition = points[currentIndex].position;
        Vector3 startTangent = points[currentIndex].position + points[currentIndex].handlenext;

        int nextIndex = GetNextIndex(currentIndex);
        Vector3 endPosition = points[nextIndex].position;
        Vector3 endTangent = points[nextIndex].position + points[nextIndex].handleprev;

        Vector3[] bezierPoints;
        int division = 100;

        bezierPoints = UnityEditor.Handles.MakeBezierPoints(startPosition, endPosition, startTangent, endTangent, division);

        float length = 0;

        for (int i = 1; i < bezierPoints.Length; i++)
        {
            Vector3 relPosition = bezierPoints[i] - bezierPoints[i - 1];
            length = length + relPosition.sqrMagnitude;
        }

        return length;
    }


    Tuple<float, Vector3, Quaternion, float, float, float> GetRobotPath(int index, float timeInWay, float step, float threshold, float maxAcc)
    {
        // timeInWay should start from 1 step (ex. 0.05 sec) except very first (absolute time 0.0 sec)
        // Should use x, z (plane)
        Vector3 curWayPos = points[index].position;
        Vector3 nextWayPos = points[index + 1].position;
        Vector3 curWayPosPlane = curWayPos;
        curWayPosPlane.y = 0;
        Vector3 nextWayPosPlane = nextWayPos;
        nextWayPosPlane.y = 0;

        TrajectoryInfo curInfo = points[index].trajectoryInfo;
        float timeInit = curInfo.timeInit;
        float attenTimeConst = timeInit + curInfo.timeConst;
        float execTime = points[index].execTime;
        float totalTime = 0.0F;

        float targetDistDiff = 0.0F;
        float initSpeed = 0.0F;
        float speedConst = curInfo.speedConst;
        float preTravel = 0.0F;
        float curTravel = 0.0F;
        float relTime = 0.0F;
        float accInitSign = points[index - 1].trajectoryInfo.accInitSign;

        if (timeInWay > 0.05)
        {
            relTime = rpaths[rpaths.Count - 1].relTimeInWay;
        }
        if (index > 0)
        {
            if (!points[index].hold)
            {
                initSpeed = points[index - 1].trajectoryInfo.speedConst;
            }
        }
        if (rpaths.Count > 0)
        {
            preTravel = rpaths[rpaths.Count - 1].travelRange;
            totalTime = rpaths[rpaths.Count - 1].time;
        }


        
        if (timeInWay >= 0 && timeInWay < timeInit)
        {
            targetDistDiff = initSpeed * step + accInitSign * maxAcc * (Mathf.Pow(timeInWay+step, 2) - Mathf.Pow(timeInWay, 2)) / 2;

        }
        else if (timeInWay > timeInit && timeInWay < attenTimeConst)
        {
            targetDistDiff = speedConst * step;
        }
        else if (timeInWay >= attenTimeConst)
        {
            float tempTime1 = timeInWay - attenTimeConst;
            float tempTime2 = tempTime1 + step;
            targetDistDiff = speedConst * step - maxAcc * (Mathf.Pow(tempTime2, 2) - Mathf.Pow(tempTime1, 2)) / 2;
        }


        bool stop = false;
        float curDistDiff = 0.0F;
        float error = 0.0F;
        float nextRelTime = relTime + 0.01F;
        Vector3 posRelTime;
        Vector3 posNextRelTime;
        Vector3 nextPos = GetBezierPosition(index, nextRelTime);
        Quaternion nextRot = GetLerpRotation(index, nextRelTime);
        while (!stop)
        {
            posRelTime = GetBezierPosition(index, relTime);
            posRelTime.y = 0;
            posNextRelTime = GetBezierPosition(index, nextRelTime);
            nextPos = posNextRelTime;
            nextRot = GetLerpRotation(index, nextRelTime);
            posNextRelTime.y = 0;
            curDistDiff = posNextRelTime.magnitude - posRelTime.magnitude;
            error = targetDistDiff - curDistDiff;

            if (error < threshold)
            {
                stop = true;
            }
            else
            {
                nextRelTime = nextRelTime + error;
            }
        }

        float curSpeed = curDistDiff / step;
        curTravel = preTravel + curDistDiff;
        totalTime = totalTime + step;
        //RobotPath currentPath = new RobotPath(totalTime, nextPos, nextRot, curSpeed, curTravel);
        //currentPath.relTimeInWay = nextRelTime;

        //return currentPath;
        return new Tuple<float, Vector3, Quaternion, float, float, float>(totalTime, nextPos, nextRot, curSpeed, curTravel, nextRelTime);
    }

    
    public void GenPath(float rate, float threshold, float maxAcc)
    {
        rpaths = new List<RobotPath>();
        //int length = (int)(time / rate) + 1;
        if (rate == 0) return;
        float step = 1.0F / rate;

        currentWaypointIndex = 0;
        bool holdDone = false;
        float currentTime = 0.0F;
        //RobotPath temp = new RobotPath(currentTime, tpos, trot, 0.0F, 0.0F);

        while (currentWaypointIndex < points.Count)
        {
            while (currentTimeInWaypoint < 1)
            {
                float timeInWay = 0.0F;
                if (!points[currentWaypointIndex].hold || holdDone)
                {
                    var result = GetRobotPath(currentWaypointIndex, timeInWay, step, threshold, maxAcc);
                    RobotPath temp = new RobotPath(result.Item1, result.Item2, result.Item3, result.Item4, result.Item5);
                    temp.relTimeInWay = result.Item6;
                    rpaths.Add(temp);
                    currentTime += step;
                    //currentTimeInWaypoint += step / points[currentWaypointIndex].execTime;
                    //position = GetBezierPosition(currentWaypointIndex, currentTimeInWaypoint);
                    //rotation = GetLerpRotation(currentWaypointIndex, currentTimeInWaypoint);
                }
                else
                {
                    var result = GetRobotPath(currentWaypointIndex, timeInWay, step, threshold, maxAcc);
                    RobotPath temp = new RobotPath(result.Item1, result.Item2, result.Item3, result.Item4, result.Item5);
                    temp.relTimeInWay = result.Item6;
                    rpaths.Add(temp);
                    currentTime += step;
                    //currentTimeInWaypoint += step / points[currentWaypointIndex].holdTime;
                    //position = GetBezierPosition(currentWaypointIndex, 0.0F);
                    //rotation = GetLerpRotation(currentWaypointIndex, 0.0F);
                }

                //RobotPath temp = new RobotPath(, position, rotation, speed, travelRange);
                //rpaths.Add(temp);
                //currentTime += step;
            }

            if (!points[currentWaypointIndex].hold || holdDone)
            {
                ++currentWaypointIndex;
                holdDone = false;
            }
            else holdDone = true;

            if (currentWaypointIndex == points.Count - 1 && !looped) break;
            if (currentWaypointIndex == points.Count && afterLoop == CPC_EAfterLoop.Continue) currentWaypointIndex = 0;
        }

        PathToStr();
    }
    


    IEnumerator FollowPath(float time)
    {
        UpdateTimeInSeconds(time);
        currentWaypointIndex = 0;
        bool holdDone = false;
        while (currentWaypointIndex < points.Count)
        {
            currentTimeInWaypoint = 0;
            while (currentTimeInWaypoint < 1)
            {
                if (!paused)
                {
                    if (!points[currentWaypointIndex].hold || holdDone)
                    {
                        currentTimeInWaypoint += Time.deltaTime / timePerSegment;
                        selectedCamera.transform.position = GetBezierPosition(currentWaypointIndex, currentTimeInWaypoint);
                        
                        if (!lookAtTarget)
                            selectedCamera.transform.rotation = GetLerpRotation(currentWaypointIndex, currentTimeInWaypoint);
                        else
                            selectedCamera.transform.rotation = Quaternion.LookRotation((target.transform.position - selectedCamera.transform.position).normalized);
                    }
                    else
                    {
                        currentTimeInWaypoint += Time.deltaTime / points[currentWaypointIndex].holdTime;
                        selectedCamera.transform.position = GetBezierPosition(currentWaypointIndex, 0.0F);

                        if (!lookAtTarget)
                            selectedCamera.transform.rotation = GetLerpRotation(currentWaypointIndex, 0.0F);
                        else
                            selectedCamera.transform.rotation = Quaternion.LookRotation((target.transform.position - selectedCamera.transform.position).normalized);
                    }
                }
                yield return 0;
            }
            if (!points[currentWaypointIndex].hold || holdDone)
            {
                ++currentWaypointIndex;
                holdDone = false;
            }
            else holdDone = true;

            if (currentWaypointIndex == points.Count - 1 && !looped) break;
            if (currentWaypointIndex == points.Count && afterLoop == CPC_EAfterLoop.Continue) currentWaypointIndex = 0;
        }
        StopPath();
    }

    int GetNextIndex(int index)
    {
        if (index == points.Count-1)
            return 0;
        return index + 1;
    }

    Vector3 GetBezierPosition(int pointIndex, float time)
    {
        float t = points[pointIndex].positionCurve.Evaluate(time);
        int nextIndex = GetNextIndex(pointIndex);
        return
            Vector3.Lerp(
                Vector3.Lerp(
                    Vector3.Lerp(points[pointIndex].position,
                        points[pointIndex].position + points[pointIndex].handlenext, t),
                    Vector3.Lerp(points[pointIndex].position + points[pointIndex].handlenext,
                        points[nextIndex].position + points[nextIndex].handleprev, t), t),
                Vector3.Lerp(
                    Vector3.Lerp(points[pointIndex].position + points[pointIndex].handlenext,
                        points[nextIndex].position + points[nextIndex].handleprev, t),
                    Vector3.Lerp(points[nextIndex].position + points[nextIndex].handleprev,
                        points[nextIndex].position, t), t), t);
    }

    public Quaternion GetLerpRotation(int pointIndex, float time)
    {
        return Quaternion.LerpUnclamped(points[pointIndex].rotation, points[GetNextIndex(pointIndex)].rotation, points[pointIndex].rotationCurve.Evaluate(time));
    }

#if UNITY_EDITOR
    public void OnDrawGizmos()
    {
        //Gizmos.color = Color.blue;
        //Gizmos.DrawSphere(points[3].position, 0.1f);
        if (UnityEditor.Selection.activeGameObject == gameObject || alwaysShow)
        {
            if (points.Count >= 2)
            {
                for (int i = 0; i < points.Count; i++)
                {
                    if (i < points.Count - 1)
                    {
                        var index = points[i];
                        var indexNext = points[i + 1];
                        UnityEditor.Handles.DrawBezier(index.position, indexNext.position, index.position + index.handlenext,
                            indexNext.position + indexNext.handleprev,((UnityEditor.Selection.activeGameObject == gameObject) ? visual.pathColor : visual.inactivePathColor), null, 5);
                    }
                    else if (looped)
                    {
                        var index = points[i];
                        var indexNext = points[0];
                        UnityEditor.Handles.DrawBezier(index.position, indexNext.position, index.position + index.handlenext,
                            indexNext.position + indexNext.handleprev, ((UnityEditor.Selection.activeGameObject == gameObject) ? visual.pathColor : visual.inactivePathColor), null, 5);
                    }
                }
            }

            for (int i = 0; i < points.Count; i++)
            {
                var index = points[i];
                Gizmos.matrix = Matrix4x4.TRS(index.position, index.rotation, Vector3.one);
                Gizmos.color = visual.frustrumColor;
                Gizmos.DrawFrustum(Vector3.zero, 90f, 0.25f, 0.01f, 1.78f);
                Gizmos.matrix = Matrix4x4.identity;
            }
        }
    }
#endif

}
