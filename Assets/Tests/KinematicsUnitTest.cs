using Unity.Mathematics;
using UnityEngine;

public class KinematicsUnitTest : MonoBehaviour
{

    void OnDrawGizmos()
    {
        float3x3 rotation = float3x3.identity;
        float3 translation = new float3(3f,3f,3f);


        float4x4 transform = Kinematics.CreateTransform(rotation, translation);
        float3x3 extractedRotation = Kinematics.GetRotation(transform);
        float3 extractedtranslation = Kinematics.GetTranslation(transform);
        float trace = Kinematics.Trace(rotation);
        
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
