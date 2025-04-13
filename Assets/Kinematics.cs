

using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System;
using MathNet.Numerics.Random;

class Kinematics
{


    public static float4x4 CreateTransform(float3x3 rotation, float3 translation)
    {
        float4x4 matrix = new float4x4(rotation, translation);



        return matrix;
    }

    public static float3x3 GetRotation(float4x4 mat4)
    {
        return new float3x3(mat4);
    }

    public static Vector3 ToVector(float3 vec3)
    {
        return new Vector3(vec3.x, vec3.y, vec3.z);
    }
    public static float3 FromVector(Vector3 vec3)
    {
        return new float3(vec3.x, vec3.y, vec3.z);
    }

    public static float3 GetTranslation(float4x4 mat4)
    {
        return new float3(mat4.c3.x, mat4.c3.y, mat4.c3.z);
    }

    public static float GetMagnitude(float3 vec)
    {

        float total = 0;
        for (int i = 0; i < 3; i++)
        {
            total += math.square(vec[i]);
        }
        return math.sqrt(total);
    }

    public static float[] ToArray(float3 f)
    {
        return new float[3] { f[0], f[1], f[2] };
    }
    public static float[][] ToJaggedArray(float3x3 mat3)
    {
        float[][] jaggedArray = new float[3][];
        jaggedArray[0] = ToArray(mat3.c0);
        jaggedArray[1] = ToArray(mat3.c1); ;
        jaggedArray[2] = ToArray(mat3.c2); ;
        return jaggedArray;
    }

    public static Matrix4x4 ToMatrix(float4x4 mat4)
    {
        return mat4.ConvertTo<Matrix4x4>();
    }

    public static Matrix4x4 ToMatrix(float3x3 mat3)
    {
        return mat3.ConvertTo<Matrix4x4>();
    }

    public static Matrix4x4 ToMatrix(float3 vec3)
    {
        return vec3.ConvertTo<Matrix4x4>();
    }

    public static Quaternion ToQuaternion(float3x3 rotation)
    {
        return math.normalize(math.quaternion(rotation));
    }

    public static Quaternion ToQuaternion(float4x4 transformation)
    {
        return math.normalize(math.quaternion(transformation));
    }

    public static float Trace(float3x3 so3)
    {
        return so3.c0.x + so3.c1.y + so3.c2.z;
    }
    public static float Trace(float4x4 se3)
    {
        return se3.c0.x + se3.c1.y + se3.c2.z + se3.c3.w;
    }

    public static float3x3 Skew(float3 vec3)
    {
        return new float3x3(
            0, -vec3.z, vec3.y,
            vec3.z, 0, -vec3.x,
            -vec3.y, vec3.x, 0
            );
    }

    public static float3 SkewInverse(float3x3 mat3)
    {
        return new float3(
            mat3.c1.z,
            mat3.c2.x,
            mat3.c0.y
        );
    }

    public static float3x3 Exp(float3 omega, float theta)
    {
        float3x3 omegaSkew = Skew(omega);
        float3x3 omegaSkew2 = math.mul(omegaSkew, omegaSkew);

        return float3x3.identity + math.sin(theta) * omegaSkew + (1 - math.cos(theta)) * omegaSkew2;
    }

    public static float3x3 ExpG(float3 omega, float theta)
    {
        float3x3 omegaSkew = Skew(omega);
        float3x3 omegaSkew2 = math.mul(omegaSkew, omegaSkew);

        return float3x3.identity * theta + (1 - math.cos(theta)) * omegaSkew + (theta - math.sin(theta)) * omegaSkew2;
    }

    public static float LogTheta(float3x3 mat3)
    {
        if (mat3.Equals(float3x3.identity))
        {
            return 0;
        }

        float tr = Trace(mat3);
        float num = tr - 1f;
        float theta = math.acos(num / 2f);

        return theta;
    }

    public static float3 LogOmega(float3x3 mat3)
    {
        float theta = LogTheta(mat3);

        if (mat3.Equals(float3x3.identity) || theta == 0)
        {
            return float3.zero;
        }


        if (math.max(Trace(mat3), -1) == -1)
        {
            float product = 1 / math.sqrt(2 * (1 + mat3.c1.y));

            return product * new float3(
                mat3.c2.x,
                mat3.c1.y + 1,
                mat3.c1.z
            );
        }
        else
        {
            float product = 1 / (2 * math.sin(theta));

            return product * new float3(
                mat3.c1.z - mat3.c2.y,
                mat3.c2.x - mat3.c0.z,
                mat3.c0.y - mat3.c1.x);
        }
    }

    public static float4x4 JointV(float3 omega, float3 v, float theta)
    {
        return CreateTransform(Exp(omega, theta), math.mul(ExpG(omega, theta), v));
    }

    public static float4x4 fkInSpace(float4x4 m, float3[] omegaList, float3[] vList, float[] thetaList)
    {
        float4x4 config = float4x4.identity;

        for (int i = 0; i < thetaList.Length; i++)
        {
            float4x4 expValue = Kinematics.JointV(omegaList[i], vList[i], thetaList[i]);
            config = math.mul(config, expValue);
        }

        return math.mul(config, m);
    }
    public static float4x4 TransInverse(float4x4 matrix)
    {
        float3x3 rot = math.transpose(GetRotation(matrix));

        return CreateTransform(rot, math.mul(-rot, GetTranslation(matrix)));
    }

    public static Matrix<float> SpaceJacobian(float[] thetaList, float3[] omegaList, float3[] vList)
    {
        Matrix<float> jacobian = Matrix<float>.Build.Dense(6, thetaList.Length);
        float4x4 currentExponential = float4x4.identity;

        if (thetaList.Length > 0)
        {
            float[] s1 = { omegaList[0][0], omegaList[0][1], omegaList[0][2], vList[0][0], vList[0][1], vList[0][2] };
            jacobian.SetColumn(0, s1);
        }

        for (int i = 1; i < thetaList.Length; i++)
        {
            float4x4 lastExponential = JointV(omegaList[i - 1], vList[i - 1], thetaList[i - 1]);
            currentExponential = math.mul(currentExponential, lastExponential);

            float[] si = { omegaList[i][0], omegaList[i][1], omegaList[i][2], vList[i][0], vList[i][1], vList[i][2] };
            Matrix<float> siMatrix = Matrix<float>.Build.DenseOfColumnArrays(si);
            Matrix<float> adj_e = Adjoint(currentExponential);
            Matrix<float> currentJ = adj_e.Multiply(siMatrix);
            jacobian.SetColumn(i, currentJ.Column(0));
        }

        return jacobian;
    }

    public static float4x4 JointSpaceQ(float3 omega, float3 q, float theta)
    {
        return JointV(omega, math.cross(-omega, q), theta);
    }

    public static float4x4 JointBodyQ(float3 omega, float3 q, float theta)
    {
        return JointV(omega, math.cross(omega, q), theta);
    }

    public static Matrix<float> Adjoint(float4x4 T)
    {
        float3x3 rotation = GetRotation(T);
        float3 translation = GetTranslation(T);
        Matrix<float> result = Matrix<float>.Build.Dense(6, 6);
        Matrix<float> R = Matrix<float>.Build.DenseOfColumnArrays(ToJaggedArray(rotation));//FromFloat3x3(GetRotation(T));
        Matrix<float> skewP = Matrix<float>.Build.DenseOfColumnArrays(ToJaggedArray(Skew(translation)));//.FromFloat3x3(Skew(GetTranslation(T)));
        //Matrix RP = R.MatMul(skewP);
        Matrix<float> RP = R.Multiply(skewP);

        result.SetSubMatrix(0, 0, R);
        result.SetSubMatrix(3, 0, RP);
        result.SetSubMatrix(3, 3, R);

        return result;
    }


    public static float3x3 Ginv(float theta, float3 omega)
    {
        float3x3 skew = Skew(omega);
        float3x3 result = 1f / theta * float3x3.identity - 1f / 2f * skew + (1f / theta - 1f / 2f * (1f / math.tan(theta / 2f))) * math.mul(skew, skew);
        return result;
    }

    public static Matrix<float> TransMatrixLog(float4x4 mat)
    {
        float3x3 R = GetRotation(mat);
        float3 p = GetTranslation(mat);
        float3 v;
        float3 omega;
        float theta = LogTheta(R);

        if (R.Equals(float3x3.identity) || theta == 0)
        {
            omega = float3.zero;
            v = p / GetMagnitude(p);
        }
        else
        {
            omega = LogOmega(R);
            v = math.mul(Ginv(theta, omega), p);
        }
        float[,] wv = { { omega.x }, { omega.y }, { omega.z }, { v.x }, { v.y }, { v.z } };
        Matrix<float> twist = Matrix<float>.Build.DenseOfArray(wv).Multiply(theta);
        return twist;
    }
}