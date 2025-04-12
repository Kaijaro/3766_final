using Unity.Mathematics;
using UnityEngine;

public class KinematicsTesting : MonoBehaviour
{
    [Header("Input")]
    [SerializeField] float3 omega;
    [SerializeField] float3 q;
    [SerializeField] float theta;

    [SerializeField] bool update = false;

    [Header("Output")]
    [SerializeField] int rows;
    [SerializeField] int cols;
    [SerializeField] float element;
    float3x3 I = float3x3.identity;
    float3 p = float3.zero;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    void OnDrawGizmos()
    {
        if (update)
        {
            update = false;

            // rows = Kinematics.Adjoint(Kinematics.CreateTransform(I,p)).Rows;
            // cols = Kinematics.Adjoint(Kinematics.CreateTransform(I,p)).Cols;
            // element = Kinematics.Adjoint(Kinematics.CreateTransform(I,p))[0,1];

        }
    }

    // Update is called once per frame
    void Update()
    {

    }
}
