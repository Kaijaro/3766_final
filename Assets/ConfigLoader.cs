using System;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Collections.Generic;

/**
    The config loader finds the twist postion for the NumericalIK.cs script.

    Allows for the potential to apply those rotations, visualize the configured forward kinematics, and reset back to the original rotations
*/
public class ConfigLoader : MonoBehaviour
{
    [SerializeField] Transform[] joints;

    [SerializeField] bool update = false;

    [SerializeField] float3[] omegaList;
    public float3[] OmegaList { get { return omegaList; } }
    [SerializeField] float3[] vList;
    public float3[] VList { get { return vList; } }
    [SerializeField] float[] thetaList;
    public float[] ThetaList { get { return thetaList; } }
    [SerializeField] float4x4[] mList;
    [SerializeField] float4x4 m;
    public float4x4 M { get { return m; } }

    public void ProcessJoints()
    {
        m = Kinematics.CreateTransform(float3x3.identity, float3.zero);
        thetaList = new float[joints.Length * 3];
        vList = new float3[joints.Length * 3];
        omegaList = new float3[joints.Length * 3];
        mList = new float4x4[joints.Length];

        int i = 0;
        foreach (Transform joint in joints)
        {

            float3 translation = Kinematics.GetTranslation(m);
            float3x3 rotation = Kinematics.GetRotation(m);

            Vector3 offset = i == 0 ? joint.position : joint.localPosition;

            translation.x += offset.x;
            translation.y += offset.y;
            translation.z += offset.z;

            m = Kinematics.CreateTransform(rotation, translation);
            mList[i / 3] = new float4x4(m);

            thetaList[i] = math.radians(joint.localEulerAngles.y);
            thetaList[i + 1] = math.radians(joint.localEulerAngles.x);
            thetaList[i + 2] = math.radians(joint.localEulerAngles.z);


            omegaList[i] = new float3(0, 1, 0);
            omegaList[i + 1] = new float3(1, 0, 0);
            omegaList[i + 2] = new float3(0, 0, 1);

            vList[i] = math.cross(-omegaList[i], translation);
            vList[i + 1] = math.cross(-omegaList[i + 1], translation);
            vList[i + 2] = math.cross(-omegaList[i + 2], translation);

            i += 3;
        }

    }




    void OnDrawGizmos()
    {
        if (update)
        {
            update = false;
            ProcessJoints();
        }

    }

    public void ApplyRotations(float[] rotations)
    {
        for (int i = 0; i < rotations.Length / 3; i++)
        {

            Transform joint = joints[i];
            float rotY = math.degrees(rotations[i * 3]);
            float rotX = math.degrees(rotations[i * 3 + 1]);
            float rotZ = math.degrees(rotations[i * 3 + 2]);

            if (i == 0)
            {
                joint.eulerAngles = new Vector3(rotX, rotY, rotZ);
            }
            else
            {
                joint.localEulerAngles = new Vector3(rotX, rotY, rotZ);
            }

        }
    }

    public void ResetRotations()
    {
        for (int i = 0; i < thetaList.Length / 3; i++)
        {

            Transform joint = joints[i];
            float rotY = math.degrees(thetaList[i * 3]);
            float rotX = math.degrees(thetaList[i * 3 + 1]);
            float rotZ = math.degrees(thetaList[i * 3 + 2]);

            joint.localEulerAngles = new Vector3(rotX, rotY, rotZ);
        }
    }

    private float4x4 CalcFrame(float[] thetaList, int jointIndex)
    {
        float4x4 frame = float4x4.identity;

        for (int j = 0; j <= jointIndex; j++)
        {
            try
            {
                float4x4 expY = Kinematics.JointV(omegaList[j * 3], vList[j * 3], thetaList[j * 3]);
                float4x4 expX = Kinematics.JointV(omegaList[j * 3 + 1], vList[j * 3 + 1], thetaList[j * 3 + 1]);
                float4x4 expZ = Kinematics.JointV(omegaList[j * 3 + 2], vList[j * 3 + 2], thetaList[j * 3 + 2]);
                frame = math.mul(frame, expY);
                frame = math.mul(frame, expX);
                frame = math.mul(frame, expZ);
            }
            catch (System.Exception)
            {
                Debug.Log(j);
                throw;
            }
        }

        return math.mul(frame, mList[jointIndex]); ;
    }

    public void DisplayConfig(float[] thetaList)
    {
        for (int i = 0; i < mList.Length; i++)
        {
            float4x4 config = CalcFrame(thetaList, i);
            Vector3 pos = Kinematics.ToVector(Kinematics.GetTranslation(config));

            if (i > 0)
            {
                float4x4 lastPosConfig = CalcFrame(thetaList, i - 1);

                Gizmos.color = Color.HSVToRGB((i - 1f) / (mList.Length - 1f), .35f, 1f);
                Vector3 lastPos = Kinematics.ToVector(Kinematics.GetTranslation(lastPosConfig));

                Gizmos.DrawLine(lastPos, pos);
            }

            if (i >= mList.Length - 1)
            {
                float3x3 rot = Kinematics.GetRotation(config);
                Vector3 rotX = Kinematics.ToVector(rot.c0);
                Vector3 rotY = Kinematics.ToVector(rot.c1);
                Vector3 rotZ = Kinematics.ToVector(rot.c2);
                Gizmos.color = Color.white;
                Gizmos.DrawSphere(pos, .05f);
                Gizmos.color = Color.red;
                Gizmos.DrawLine(pos, pos + rotX);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(pos, pos + rotY);
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(pos, pos + rotZ);
            }

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
}
