using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{
    public Rigidbody rigid;

    void Awake()
    {
        rigid.AddForce(transform.up * 3000);
    }
}
