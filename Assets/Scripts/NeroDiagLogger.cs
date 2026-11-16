using System.IO;
using System.Text;
using UnityEngine;

/// <summary>Records arm / gripper / object state every physics step to a CSV (diagnostics).</summary>
[DefaultExecutionOrder(1200)]
public class NeroDiagLogger : MonoBehaviour
{
    public string objectName = "Red Cube";
    public string path = "Verification/diag.csv";
    NeroCartesianController arm;
    NeroPickPlace pp;
    ArticulationBody fingerA, fingerB;
    Rigidbody obj;
    readonly StringBuilder sb = new StringBuilder();

    void Start()
    {
        arm = GetComponent<NeroCartesianController>();
        pp = GetComponent<NeroPickPlace>();
        foreach (var b in GetComponentsInChildren<ArticulationBody>())
        {
            if (b.name == "gripper_link1") fingerA = b;
            if (b.name == "gripper_link2") fingerB = b;
        }
        var go = GameObject.Find(objectName);
        if (go) obj = go.GetComponent<Rigidbody>();
        sb.Append("t,mode,busy,tx,ty,tz,gx,gy,gz,pos_err,rot_err,grip_cmd,fA,fB,ox,oy,oz,locked,cvx,cvy,cvz,ctx,cty,ctz");
        for (int i = 0; i < 7; i++) sb.Append($",qc{i},qm{i}");
        sb.AppendLine();
    }

    void FixedUpdate()
    {
        var p = arm.ToolPosition; var g = arm.targetPosition;
        Vector3 o = obj ? obj.position : Vector3.zero;
        sb.Append($"{Time.time:F3},{arm.mode},{(pp && pp.Busy ? 1 : 0)},{p.x:F4},{p.y:F4},{p.z:F4},{g.x:F4},{g.y:F4},{g.z:F4},{arm.PositionError:F4},{arm.RotationErrorDeg:F2},{arm.gripperWidth:F3},{fingerA.jointPosition[0]:F4},{fingerB.jointPosition[0]:F4},{o.x:F4},{o.y:F4},{o.z:F4},{arm.LockedJointMask},{arm.commandedVelocity.x:F4},{arm.commandedVelocity.y:F4},{arm.commandedVelocity.z:F4},{arm.CommandedToolPosition.x:F4},{arm.CommandedToolPosition.y:F4},{arm.CommandedToolPosition.z:F4}");
        for (int i = 0; i < 7; i++) sb.Append($",{arm.JointCommand[i]:F4},{arm.JointMeasured[i]:F4}");
        sb.AppendLine();
    }

    public void Flush()
    {
        Directory.CreateDirectory(Path.GetDirectoryName(path));
        File.WriteAllText(path, sb.ToString());
    }

    void OnDisable() => Flush();
}
