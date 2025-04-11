using MathNet.Numerics.LinearAlgebra;
using Unity.Mathematics;
using UnityEngine;

public class NumericalIK : MonoBehaviour
{
    [SerializeField] ConfigLoader config;
    [SerializeField] Transform target;

    [SerializeField] double errorOmega = 0.001;
    [SerializeField] double errorV = 0.0001;

    float[] thetaList;

    void IKStep()
    {
        if (thetaList == null || thetaList.Length == 0)
        {
            thetaList = config.ThetaList;
        }


        float4x4 T_sb = Kinematics.fkInSpace(config.M, config.OmegaList, config.VList, thetaList);
        float4x4 T_bs = Kinematics.TransInverse(T_sb);
        float4x4 T_sd = float4x4.TRS(target.position, target.rotation, target.lossyScale);

        float4x4 T_bd = math.mul(T_bs, T_sd);

        Matrix<float> V = Kinematics.TransMatrixLog(T_bd);

        Vector<float> omega = V.Column(0).SubVector(0, 3);
        Vector<float> v = V.Column(0).SubVector(3, 3);


        double currentOmegaError = omega.SumMagnitudes();
        double currentVError = v.SumMagnitudes();

        // Error of ||V.w|| and ||V.v|| 
        var jacobian = Kinematics.SpaceJacobian(thetaList, config.OmegaList, config.VList);
        var jacobianInverse = jacobian.PseudoInverse();

        // Next Guess
        // thetaList + jacobian * V
        Vector<float> thetaVector = Vector<float>.Build.DenseOfArray(thetaList);
        var result = jacobianInverse.Multiply(V);

        Debug.Log("");
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
