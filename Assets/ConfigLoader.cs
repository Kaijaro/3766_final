using System;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

public class ConfigLoader : MonoBehaviour
{
    [SerializeField] Transform[] joints;

    [SerializeField] bool update = false;

    [SerializeField] float3[] omegaList;
    [SerializeField] float3[] vList;
    [SerializeField] float[] thetaList;
    [SerializeField] float4x4[] mList;
    [SerializeField] float4x4 m;

    [SerializeField] string jres = "a\nb";

    Matrix jacobian;

    void ProcessJoints()
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

            translation.x += joint.localPosition.x;
            translation.y += joint.localPosition.y;
            translation.z += joint.localPosition.z;

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

    void UpdateJacobian()
    {
        jacobian = new Matrix(6, joints.Length);
        float4x4 currentExponential = float4x4.identity;
        if (joints.Length > 0)
        {
            float[] s1 = { omegaList[0][0], omegaList[0][1], omegaList[0][2], vList[0][0], vList[0][1], vList[0][2] };
            jacobian.SetColumn(s1, 0);
        }
        for (int i = 1; i < joints.Length; i++)
        {
            float4x4 lastExponential = Kinematics.JointV(omegaList[i - 1], vList[i - 1], thetaList[i - 1]);
            currentExponential = math.mul(currentExponential, lastExponential);
            float[,] si = { { omegaList[i][0] }, { omegaList[i][1] }, { omegaList[i][2] }, { vList[i][0] }, { vList[i][1] }, { vList[i][2] } };
            Matrix siMatrix = new Matrix(si);
            Matrix currentJ = Kinematics.Adjoint(currentExponential).MatMul(siMatrix);
            jacobian.SetColumn(currentJ, i);
        }
    }

    void OnDrawGizmos()
    {
        // if (update)
        // {
        //     update = false;
        //     ProcessJoints();
        // }
        ProcessJoints();
        UpdateJacobian();
        jres = jacobian.Data.ToString();
        for (int i = 0; i < mList.Length; i++)
        {
            float4x4 config = float4x4.identity;

            for (int j = 0; j <= i; j++)
            {
                float4x4 expY = Kinematics.JointV(omegaList[j * 3], vList[j * 3], thetaList[j * 3]);
                float4x4 expX = Kinematics.JointV(omegaList[j * 3 + 1], vList[j * 3 + 1], thetaList[j * 3 + 1]);
                float4x4 expZ = Kinematics.JointV(omegaList[j * 3 + 2], vList[j * 3 + 2], thetaList[j * 3 + 2]);
                config = math.mul(config, expY);
                config = math.mul(config, expX);
                config = math.mul(config, expZ);

            }
            config = math.mul(config, mList[i]);

            Vector3 pos = Kinematics.ToVector(Kinematics.GetTranslation(config));
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



    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }
}
