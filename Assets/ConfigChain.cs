using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

/**
    Final not in use. First attempt at implimenting config
*/
public class ConfigChain : MonoBehaviour
{
    [SerializeField] Revolute[] joints;
    [Range(-360f, 360f)][SerializeField] float[] angles;

    void OnDrawGizmos()
    {
        float4x4 M = float4x4.identity;
        float3 s = float3.zero;

        for (int i = 0; i < joints.Length; i++)
        {
            Revolute joint = joints[i];
            float angle = angles[i];

            Gizmos.DrawSphere(joint.q, .1f);
        }
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
