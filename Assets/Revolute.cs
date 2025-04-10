
using System;
using UnityEngine;

[Serializable]
public class Revolute : MonoBehaviour
{
    public Vector3 omega = Vector3.up;
    public Vector3 q = Vector3.zero;
    private Vector3 startOffset;
    private Transform childJointTransform;
    private Vector3 linkA;
    private Vector3 linkB;
    private Boolean hasChildJoint;
    private Boolean isRootJoint;

    public Vector3 Omega
    {
        get
        {
            return omega.magnitude == 0 ? Vector3.up : omega.normalized;
        }
        set
        {
            omega = value.normalized;
        }
    }



    public Vector3 v
    {
        get
        {
            return Vector3.Cross(Omega, q);
        }
        set
        {
            q = -Vector3.Cross(Omega, value);
        }
    }

    void Awake()
    {
        startOffset = transform.localPosition;
    }

    void OnDrawGizmos()
    {
        // Vector3 computedOmega = Omega;
        // Vector3 computedQ = q;
        // Vector3 computedV = v;

        // Gizmos.color = Color.yellow;
        // Gizmos.DrawSphere(transform.position, 0.1f);
        // Gizmos.DrawLine(transform.position, transform.position + computedOmega);
        // Gizmos.color = Color.magenta;
        // Gizmos.DrawLine(transform.position, transform.position + computedQ);
        // Gizmos.color = Color.cyan;
        // Gizmos.DrawLine(transform.position + computedQ, transform.position + computedQ + computedV);

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + Omega);

        if (transform.childCount > 0) {
            hasChildJoint = true;
            childJointTransform = transform.GetChild(0);
            linkA = transform.position;
            linkB = childJointTransform.position;
        }
        if (hasChildJoint) {
            Gizmos.color = Color.gray;
            Gizmos.DrawLine(linkA, linkB);
        }
    }


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (transform.parent == null) {
            isRootJoint = true;
            transform.Rotate(0,90,0);
        }
        
    }

    // Update is called once per frame
    void Update()
    {

    }
}
