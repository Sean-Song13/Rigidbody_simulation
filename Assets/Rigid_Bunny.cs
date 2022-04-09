using System;
using UnityEngine;
using System.Collections;
using TreeEditor;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    private float gravity = 9.8f;
    Vector3 v = new Vector3(0, 0, 0); // velocity
    Vector3 w = new Vector3(0, 0, 0); // angular velocity

    float mass; // mass
    Matrix4x4 I_ref; // reference inertia

    float linear_decay = 0.999f; // for velocity decay
    float angular_decay = 0.98f;
    public float restitution = 0.5f; // for collision

    public float friction = 0.9f;

    private Vector3 scale = Vector3.one;

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        scale = transform.localScale;
        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * Vector3.Scale(scale, vertices[i]).sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }

        I_ref[3, 3] = 1;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    Matrix4x4 Subtract_Matrix(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 res = Matrix4x4.identity;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                res[i, j] = a[i, j] - b[i, j];
            }
        }

        return res;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        var x0 = transform.position;
        var q0 = transform.rotation;
        for (int i = 0; i < vertices.Length; i++)
        {
            var Rri = Matrix4x4.Rotate(q0).MultiplyVector(Vector3.Scale(scale, vertices[i]));
            var x = x0 + Rri;
            float phiX = Vector3.Dot(x - P, N);
            if (phiX > 0 || Mathf.Abs(phiX) < 0.002) continue;
            var vi = v + Get_Cross_Matrix(w).MultiplyVector(Rri);
            var vDirection = Vector3.Dot(vi, N);
            if (vDirection > 0) continue;
            var vi_new = Impulse_Method(P, N, vi);
            Matrix4x4 Mmass = Matrix4x4.zero;
            Mmass[0, 0] = 1 / mass;
            Mmass[1, 1] = 1 / mass;
            Mmass[2, 2] = 1 / mass;
            Mmass[3, 3] = 1;
            var k = Subtract_Matrix(Mmass, Get_Cross_Matrix(Rri) * I_ref.inverse * Get_Cross_Matrix(Rri));
            var j = k.inverse * (vi_new - vi);
            v = linear_decay * v + 1 / mass * (Vector3) j;
            w = angular_decay * w + (Vector3) (I_ref.inverse * Get_Cross_Matrix(Rri) * j);
        }
    }

    private Vector3 Impulse_Method(Vector3 P, Vector3 N, Vector3 vi)
    {
        var vn = Vector3.Dot(vi, N) * N;
        var vt = vi - vn;
        vn = -restitution * vn;
        var alpha = Mathf.Max(1 - friction * (1 + restitution) * vn.magnitude / vt.magnitude, 0);
        vt = alpha * vt;
        return vn + vt;
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }

        if (Input.GetKey("l"))
        {
            v = new Vector3(4, 2, -1);
            launched = true;
        }

        if (launched)
        {
            // Part I: Update velocities

            var force = mass * gravity * Vector3.down;
            v = linear_decay * v + dt * force / mass;


            // Part II: Collision Impulse
            Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

            // Part III: Update position & orientation
            //Update linear status
            Vector3 x = transform.position + dt * v;
            //Update angular status
            Quaternion q = transform.rotation;
            var dq = dt / 2 * w;
            var q_temp = new Quaternion(dq.x, dq.y, dq.z, 0) * q;
            q = new Quaternion(q.x + q_temp.x, q.y + q_temp.y, q.z + q_temp.z, q.w + q_temp.w);
            // Part IV: Assign to the object
            transform.position = x;
            transform.rotation = q;
        }
    }
}