
using UnityEngine;

public class Revolute : MonoBehaviour
{
    [SerializeField] Vector3 omega = Vector3.up;
    [SerializeField] Vector3 q = Vector3.zero;
    [Range(-360f, 360f)][SerializeField] float theta = 0;




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



    void OnDrawGizmos()
    {
        Vector3 computedOmega = Omega;
        Vector3 computedQ = q;
        Vector3 computedV = v;

        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(transform.position, 0.1f);
        Gizmos.DrawLine(transform.position, transform.position + computedOmega);
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(transform.position, transform.position + computedQ);
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position + computedQ, transform.position + computedQ + computedV);
    }


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Vector3 a = new Vector3(1, 0, 0);
        Vector3 b = new Vector3(0, 0, 1);

        Debug.Log(Vector3.Cross(a, b));


    }

    // Update is called once per frame
    void Update()
    {

    }
}
