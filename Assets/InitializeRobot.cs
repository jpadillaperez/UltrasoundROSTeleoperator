using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InitializeRobot : MonoBehaviour
{
    private ArticulationBody[] articulationChain;
    public float stiffness = 10000;
    public float damping = 100;
    public float forceLimit = 1000;
    public float speed = 30f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 10f;// Units: m/s^2 / degree/s^2
    // Start is called before the first frame update
    void Start()
    {
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
    }
}
