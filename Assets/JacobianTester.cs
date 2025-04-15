using MathNet.Numerics.LinearAlgebra;
using Unity.Mathematics;
using UnityEngine;
/**
    File not in use for final product. Used to debug calculations
*/
public class JacobianTester : MonoBehaviour
{
    [SerializeField] float3[] omegalist;
    [SerializeField] float3[] vlist;
    [SerializeField] float[] thetaList;
    [SerializeField] string resultString;

    Matrix<float> result;

    [SerializeField] bool run;

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
            float4x4 lastExponential = Kinematics.JointV(omegaList[i - 1], vList[i - 1], thetaList[i - 1]);
            Debug.Log(i);
            Debug.Log(lastExponential);
            currentExponential = math.mul(currentExponential, lastExponential);

            float[] si = { omegaList[i][0], omegaList[i][1], omegaList[i][2], vList[i][0], vList[i][1], vList[i][2] };
            Matrix<float> siMatrix = Matrix<float>.Build.DenseOfColumnArrays(si);
            Matrix<float> adj_e = Kinematics.Adjoint(currentExponential);
            Matrix<float> currentJ = adj_e.Multiply(siMatrix);
            jacobian.SetColumn(i, currentJ.Column(0));
        }

        return jacobian;
    }

    void OnDrawGizmos()
    {
        if (run)
        {
            run = false;

            result = SpaceJacobian(thetaList, omegalist, vlist);
            resultString = result.ToString();
        }
    }
}
