using MathNet.Numerics.LinearAlgebra;
using Unity.Mathematics;
using Unity.VisualScripting;
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
    [SerializeField] bool reset = false;

    Matrix<float> JacobianStep;
    float[] thetaGuess;

    void NextValues()
    {
        if (thetaGuess == null || thetaGuess.Length == 0)
        {
            thetaGuess = (float[])config.ThetaList.Clone();
        }
        float4x4 T = float4x4.TRS(target.position, target.rotation, target.lossyScale);
        float4x4 Tsb = Kinematics.fkInSpace(config.M, config.OmegaList, config.VList, thetaGuess);
        float4x4 Tbs = Kinematics.TransInverse(Tsb);
        float4x4 Tbd = math.mul(Tbs, T);

        Matrix<float> Vbd = Kinematics.TransMatrixLog(Tbd);
        Matrix<float> AdjVsb = Kinematics.Adjoint(Tsb);

        Matrix<float> Vs = AdjVsb.Multiply(Vbd);
        Matrix<float> jacobian = Kinematics.SpaceJacobian(thetaGuess, config.OmegaList, config.VList);
        Matrix<float> jacobianInv = jacobian.PseudoInverse();

        JacobianStep = jacobianInv.Multiply(Vs);

        Vector<float> omega = Vs.Column(0).SubVector(0, 3);
        Vector<float> v = Vs.Column(0).SubVector(3, 3);
        currentOmegaError = omega.L2Norm();
        currentVError = v.L2Norm();
    }

    float[] IK()
    {
        NextValues();

        int i = 0;
        while ((i++ < 20) && (currentOmegaError > errorOmega || currentVError > errorV))
        {
            Vector<float> thetaVector = Vector<float>.Build.DenseOfArray(thetaGuess);
            thetaGuess = (thetaVector + JacobianStep.Column(0)).AsArray();
            NextValues();
        }

        return thetaGuess;
    }

    void OnDrawGizmos()
    {
        if (reset)
        {
            reset = false;
            thetaGuess = config.ThetaList;
        }
        if (step)
        {
            float[] guess = IK();

        }
        config.DisplayConfig(thetaGuess);

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

    }

    // Update is called once per frame
    void Update()
    {

    }
}
