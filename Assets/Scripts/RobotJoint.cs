using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotJoint : MonoBehaviour
{
    private const float MIN_ANGLE_LIMIT = 0f;
    private const float MAX_ANGLE_LIMIT = 30f;
        
    public Vector3 Axis;
    public Vector3 StartOffset;
    
    [SerializeField, Range(MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT)]
    public float MinAngle = MIN_ANGLE_LIMIT;
    
    [SerializeField, Range(MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT)]
    public float MaxAngle = MAX_ANGLE_LIMIT;
    
    void Awake ()
    {
        // Position of the transform relative to the parent transform.
        StartOffset = transform.localPosition;
    }
}