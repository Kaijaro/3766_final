using MathNet.Numerics.LinearAlgebra;
using Unity.Mathematics;
using UnityEngine;

public class NumericalIK : MonoBehaviour
{
    [SerializeField] ConfigLoader config;
    [SerializeField] Transform target;

    [SerializeField] double errorOmega = 0.001;
    [SerializeField] double errorV = 0.0001;
    [SerializeField] double currentOmegaError;
    [SerializeField] double currentVError;
    [SerializeField] bool step = false;


    [SerializeField] float[] thetaList;
    [SerializeField] float3[] omegaList;
    [SerializeField] float3[] vList;
    [SerializeField] float4x4 m;


    void IKStep()
    {
        // if (thetaList == null || thetaList.Length == 0)
        // {
        //     thetaList = config.ThetaList;
        // }


        float4x4 T_sb = Kinematics.fkInSpace(m, omegaList, vList, thetaList);
        float4x4 T_bs = Kinematics.TransInverse(T_sb);
        float4x4 T_sd = float4x4.TRS(target.position, target.rotation, 1);

        float4x4 T_bd = math.mul(T_bs, T_sd);

        Matrix<float> V = Kinematics.TransMatrixLog(T_bd);

        Vector<float> omega = V.Column(0).SubVector(0, 3);
        Vector<float> v = V.Column(0).SubVector(3, 3);

        // Error of ||V.w|| and ||V.v|| 
        currentOmegaError = omega.L2Norm();
        currentVError = v.L2Norm();

        var jacobian = Kinematics.SpaceJacobian(thetaList, omegaList, vList);
        var jacobianInverse = jacobian.PseudoInverse();

        // Next Guess
        // thetaList + jacobian * V
        Vector<float> thetaVector = Vector<float>.Build.DenseOfArray(thetaList);
        var result = jacobianInverse.Multiply(V);

        thetaList = (thetaVector + result.Column(0)).AsArray();

        DisplayTransformation(T_sb);
        DisplayTransformation(T_sd);
    }

    void OnDrawGizmos()
    {
        if (!step)
        {
            return;
        }
        step = false;
        IKStep();
    }

    void DisplayTransformation(float4x4 config)
    {
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


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        config.ProcessJoints();
        IKStep();
    }

    // Update is called once per frame
    void Update()
    {

    }
}
