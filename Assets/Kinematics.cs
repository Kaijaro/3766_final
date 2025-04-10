

using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.AI;
using UE = UnityEngine;


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

        return float3x3.identity * theta + (1 - math.cos(theta)) * omegaSkew + omegaSkew2 * (theta - math.sin(theta));
    }

    public static float LogTheta(float3x3 mat3)
    {
        return math.acos((Trace(mat3) - 1f) / 2f);
    }

    public static float3 LogOmega(float3x3 mat3)
    {
        float theta = LogTheta(mat3);

        if (Trace(mat3) == -1)
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

    public static float4x4 JointSpaceQ(float3 omega, float3 q, float theta)
    {
        return JointV(omega, math.cross(-omega, q), theta);
    }

    public static float4x4 JointBodyQ(float3 omega, float3 q, float theta)
    {
        return JointV(omega, math.cross(omega, q), theta);
    }

    public static Matrix Adjoint(float4x4 T)
    {
        Matrix result = new Matrix(6, 6);
        Matrix R = Matrix.FromFloat3x3(GetRotation(T));
        Matrix skewP = Matrix.FromFloat3x3(Skew(GetTranslation(T)));
        Matrix RP = R.MatMul(skewP);
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                if (row < 3 && col < 3)
                {
                    result[row, col] = R[row, col]; ;
                }
                else if (row < 3 && col >= 3)
                {
                    result[row, col] = 0;
                }
                else if (row >= 3 && col < 3)
                {
                    result[row, col] = RP[row - 3, col];
                }
                else if (row >= 3 && col >= 3)
                {
                    result[row, col] = R[row - 3, col - 3];
                }
            }
        }
        return result;
    }

    // public static Matrix FKinSpace(Transform rootJoint)
    // {
    //     Transform current = rootJoint;
    //     float3 translation = float3.zero;

    //     while (current.childCount > 0)
    //     {

    //     }

    // }
}