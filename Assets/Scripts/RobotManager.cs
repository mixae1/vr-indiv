using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RobotManager : MonoBehaviour
{
    public GameObject endEffector;
    RobotJoint[] joints;
    public bool useForwardKinematics = true;
    
    public float samplingDistance = 0.1f;
    public float learningRate = 40f;
    public float distanceThreshold = 0.2f;
    
    private void Awake()
    {
        joints = GetComponentsInChildren<RobotJoint>();
    }
    
    private void FixedUpdate()
    {
        // angles[i] === projection of the joint's local rotation vector onto its rotation axis
        // each joint.Axis is already normalized
        float[] angles = joints.Select(joint => Vector3.Dot(joint.Axis, joint.transform.localRotation.eulerAngles)).ToArray();

        if (useForwardKinematics)
        {
             endEffector.transform.position = ForwardKinematics(angles);
        }
        else
        {
            InverseKinematics(endEffector.transform.position, angles);
            foreach (var (joint, i) in joints.Select((joint, i) => (joint, i)))
            {
                joint.transform.localRotation = Quaternion.AngleAxis(angles[i], joint.Axis);
            }
            joints[0].transform.localRotation = Quaternion.Inverse(transform.localRotation) * joints[0].transform.localRotation;
        }
    }
    
    // returns the position of the end effector, in global coordinates.
    // For each joint, method calculates the next position by rotating the offset (distance from one joint to the next) using the cumulative rotation up to that joint
    private Vector3 ForwardKinematics(float[] angles)
    {
        Vector3 prevPoint = joints[0].transform.position;
        Quaternion rotation = Quaternion.identity; // indicates "no rotation"
        for (int i = 1; i < joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis(angles[i - 1], joints[i - 1].Axis);
            Vector3 nextPoint = prevPoint + rotation * joints[i].StartOffset;
            Debug.DrawLine(prevPoint, nextPoint, Color.white, 0.1f);
            prevPoint = nextPoint;
        }
        return prevPoint;
    }
    
    public float DistanceFromTarget(Vector3 target, float [] angles)
    {
        Vector3 point = ForwardKinematics(angles);
        return Vector3.Distance(point, target);
    }
    
    public float PartialGradient (Vector3 target, float[] angles, int i)
    {
        // Saves the angle,
        // it will be restored later
        float angle = angles[i];
        // Gradient : [F(x+SamplingDistance) - F(x)] / h
        float f_x = DistanceFromTarget(target, angles);
        angles[i] += samplingDistance;
        float f_x_plus_d = DistanceFromTarget(target, angles);
        float gradient = (f_x_plus_d - f_x) / samplingDistance;
        // Restores
        angles[i] = angle;
        return gradient;
    }
    
    public void InverseKinematics (Vector3 target, float [] angles)
    {
        if (DistanceFromTarget(target, angles) < distanceThreshold) return;
        for (int i = joints.Length -1; i >= 0; i --)
        {
            // Gradient descent
            // Update : Solution -= LearningRate * Gradient
            float gradient = PartialGradient(target, angles, i);
            angles[i] -= learningRate * gradient;
            angles[i] = Mathf.Clamp(angles[i], joints[i].MinAngle, joints[i].MaxAngle);
            // Early termination
            if (DistanceFromTarget(target, angles) < distanceThreshold) return;
        }
    }
}
