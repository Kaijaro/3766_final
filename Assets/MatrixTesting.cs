using Unity.Mathematics;
using UnityEngine;

public class MatrixTesting : MonoBehaviour
{
    [Header("Input")]
    [SerializeField] float3 omega;
    [SerializeField] float3 q;
    [SerializeField] float theta;

    [SerializeField] bool update = false;

    [Header("Output")]
    [SerializeField] Matrix result;
    Matrix B;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    void OnDrawGizmos()
    {
        if (update)
        {
            update = false;
            result = new Matrix(4,3);
            result.fill(1f);
            B = new Matrix(3,2);
            B.fill(2f);
            result = result.MatMul(B);

        }
    }

    // Update is called once per frame
    void Update()
    {

    }
}