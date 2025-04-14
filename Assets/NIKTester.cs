using MathNet.Numerics.LinearAlgebra;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

public class NIKTester : MonoBehaviour
{
    [Header("Input Values")]
    [SerializeField] float4x4 m;
    [SerializeField] float4x4 target;
    [SerializeField] float[] thetaList;
    [SerializeField] float3[] omegaList;
    [SerializeField] float3[] vList;

    [SerializeField] bool nextStep = false;
    [SerializeField] bool firstStep = false;
    [SerializeField] bool simulate = false;


    [Header("Resulting Values")]
    [SerializeField] float[] thetaGuess;
    [SerializeField] double currentOmegaError;
    [SerializeField] double currentVError;
    [SerializeField] float4x4 OutFrameSB;
    [SerializeField] float4x4 OutFrameBS;
    [SerializeField] float4x4 OutFrameBD;
    [SerializeField] string OutVbd;
    [SerializeField] string OutVs;
    [SerializeField] string OutAdjVsb;
    [SerializeField] string OutJacobian;
    [SerializeField] string OutJacobianInv;
    [SerializeField] string OutJacobianStep;
    Matrix<float> JacobianStep;
    void IKCalcValues()
    {
        if (thetaGuess == null || thetaGuess.Length == 0)
        {
            thetaGuess = (float[])thetaList.Clone();
        }


        OutFrameSB = Kinematics.fkInSpace(m, omegaList, vList, thetaGuess);
        OutFrameBS = Kinematics.TransInverse(OutFrameSB);
        OutFrameBD = math.mul(OutFrameBS, target);

        Matrix<float> Vbd = Kinematics.TransMatrixLog(OutFrameBD);
        OutVbd = Vbd.ToString();

        Matrix<float> AdjVsb = Kinematics.Adjoint(OutFrameSB);
        OutAdjVsb = AdjVsb.ToString();

        Matrix<float> Vs = AdjVsb.Multiply(Vbd);
        OutVs = Vs.ToString();

        Matrix<float> jacobian = Kinematics.SpaceJacobian(thetaGuess, omegaList, vList);
        OutJacobian = jacobian.ToString();

        Matrix<float> jacobianInv = jacobian.PseudoInverse();
        OutJacobianInv = jacobianInv.ToString();

        JacobianStep = jacobianInv.Multiply(Vs);
        OutJacobianStep = JacobianStep.ToString();
    }

    void IKStep()
    {
        // if (thetaGuess == null || thetaGuess.Length == 0)
        // {
        //     thetaGuess = (float[])thetaList.Clone();
        // }

        // // good
        // OutFrameSB = Kinematics.fkInSpace(m, omegaList, vList, thetaGuess);

        // // good
        // float4x4 T_bs = Kinematics.TransInverse(OutFrameSB);

        // //good
        // OutFrameBD = math.mul(T_bs, target);

        // //!
        // Matrix<float> V = Kinematics.TransMatrixLog(OutFrameBD);
        // OutV = V.ToString();

        // Vector<float> omega = V.Column(0).SubVector(0, 3);
        // Vector<float> v = V.Column(0).SubVector(3, 3);

        // // Error of ||V.w|| and ||V.v|| 
        // currentOmegaError = omega.L2Norm();
        // currentVError = v.L2Norm();

        // Matrix<float> jacobian = Kinematics.SpaceJacobian(thetaGuess, omegaList, vList);
        // OutJacobian = jacobian.ToString();

        // Matrix<float> jacobianInverse = jacobian.PseudoInverse();

        // // Next Guess
        // // thetaList + jacobian * V
        // Vector<float> thetaVector = Vector<float>.Build.DenseOfArray(thetaGuess);
        // var result = jacobianInverse.Multiply(V);

        // thetaGuess = (thetaVector + result.Column(0)).AsArray();
    }

    void OnDrawGizmos()
    {
        DisplayTransformation(OutFrameSB);
        DisplayTransformation(target);
        if (nextStep)
        {
            nextStep = false;
            Vector<float> thetaVector = Vector<float>.Build.DenseOfArray(thetaGuess);
            thetaGuess = (thetaVector + JacobianStep.Column(0)).AsArray();
            IKCalcValues();
        }

        if (firstStep)
        {
            firstStep = false;
            thetaGuess = (float[])thetaList.Clone();
            IKCalcValues();
        }

        if (simulate)
        {
            Vector<float> thetaVector = Vector<float>.Build.DenseOfArray(thetaGuess);
            thetaGuess = (thetaVector + JacobianStep.Column(0)).AsArray();
            IKCalcValues();
        }
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
}
