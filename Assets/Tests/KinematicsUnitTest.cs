using System;
using Unity.Mathematics;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class KinematicsUnitTest : MonoBehaviour
{
    [SerializeField] bool run = false;
    [SerializeField] float3x3 exp;
    private string jointAngles = "";


    void OnDrawGizmos()
    {
        if (run) {
            run = false;
            float3x3 rotation = new float3x3(
                                        new float3(0.16f, -0.93f, 0.33f),
                                        new float3(0.84f, 0.30f, 0.45f),
                                        new float3(-0.52f, 0.20f, 0.83f)
                                    );
            float3 translation = new float3(3f,3f,3f);
            float3 omega = new float3(0f, 1f, 0f);
            float theta = math.radians(90f); 


            float4x4 transform = Kinematics.CreateTransform(rotation, translation);
            //float trace = Kinematics.Trace(rotation);
            //float magnitudeCol1 = Kinematics.GetMagnitude(new float3(rotation.c0[0], rotation.c1[0], rotation.c2[0]));
            //Quaternion rotationToQuaternion = Kinematics.ToQuaternion(rotation);
            float3x3 skew = Kinematics.Skew(new float3(1f,2f,3f));
            exp = Kinematics.Exp(omega, theta);
            Matrix<float> adj = Kinematics.Adjoint(transform);
            float3x3 ginv = Kinematics.Ginv(theta, omega);
            float3 v = math.mul(Kinematics.Ginv(theta, omega), new float3(1f, 1f, 1f));
            Matrix<float> V = Kinematics.TransMatrixLog(transform);
            

            Console.WriteLine("");
            Debug.Log("");

        }
    }
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void IKStepVisualization(int i, float[] thetaList, float3[] omegaList, float3[] vList) {
        if (i == 0) {
            Debug.Log("i | Joint Angles |        V_i       |   ||w||   |   ||v||   ");
            foreach (float theta in thetaList) {
                jointAngles += $"{theta,6:F2}, ";
            }
        }
        Debug.Log($"{i} | {jointAngles}");
    }
}
