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
    [SerializeField] float4x4 exp4;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    void OnDrawGizmos()
    {
        if (update)
        {
            update = false;

            exp4 = Kinematics.JointSpaceQ(omega, q, math.radians(theta));

        }
    }

    // Update is called once per frame
    void Update()
    {

    }
}
