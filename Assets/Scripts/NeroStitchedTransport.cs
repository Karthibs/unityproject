using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Runs the DemoStitching chained DS as the transport phase of NeroPickPlace.
///
/// Every physics step the stitched DS is evaluated at the commanded tool
/// position and its velocity is sent to NeroCartesianController (Velocity
/// mode, gripper kept pointing down). Transport ends when the tool reaches the
/// DS attractor (above the tray). Each run is logged to
/// Verification/stitched_&lt;task&gt;.csv and compared with the Python rollout.
/// </summary>
public class NeroStitchedTransport : MonoBehaviour
{
    public NeroCartesianController arm;
    public NeroPickPlace pickPlace;

    [Tooltip("Relative to the Unity project folder.")]
    public string exportPath = "DemoData/chained_ds_nero_unity.json";
    public float transportMaxSpeed = 0.45f;   // m/s, controller clamp during transport
    public float arriveTolerance = 0.006f;    // m
    public float handoffDistance = 0.025f;    // m: hand over to Pose control when this close...
    public float handoffSpeed = 0.03f;        // m/s: ...and the DS velocity has dropped below this
    public float minHeight = 0.82f;           // m: tool may not be driven below this during transport
    public float timeout = 20f;               // s
    public bool drawExecutedPath = true;
    public bool showPanel = true;

    public string Status { get; private set; } = "";
    public string LastReport { get; private set; } = "";

    NeroChainedDS.Export export;
    readonly List<LineRenderer> lines = new List<LineRenderer>();

    void Awake()
    {
        if (!arm) arm = GetComponent<NeroCartesianController>();
        if (!pickPlace) pickPlace = GetComponent<NeroPickPlace>();
    }

    void Start() => Reload();

    public void Reload()
    {
        string path = Path.IsPathRooted(exportPath) ? exportPath : Path.GetFullPath(Path.Combine(Application.dataPath, "..", exportPath));
        try
        {
            export = NeroChainedDS.Load(path);
            Status = $"loaded {export.tasks.Length} stitched tasks";
        }
        catch (System.Exception e)
        {
            export = null;
            Status = "no chained DS export: " + e.Message;
            Debug.LogWarning("NeroStitchedTransport: " + Status);
        }
    }

    public NeroChainedDS.Task FindTask(string obj, string tray)
    {
        if (export == null) return null;
        foreach (var t in export.tasks)
            if (t.startObject == obj && t.endObject == tray) return t;
        return null;
    }

    /// <summary>Pick obj, carry it with the stitched DS, place it in tray.</summary>
    public void RunStitched(string obj, string tray)
    {
        if (pickPlace.Busy) return;
        if (FindTask(obj, tray) == null) { Status = $"no stitched task for {obj} -> {tray}"; return; }
        StartCoroutine(RunStitchedRoutine(obj, tray));
    }

    public bool BenchmarkRunning { get; private set; }

    /// <summary>Run all exported tasks one after another (cubes reset in between) and write Verification/stitched_summary.txt.</summary>
    public void RunAll()
    {
        if (!pickPlace.Busy && !BenchmarkRunning && export != null) StartCoroutine(RunAllRoutine());
    }

    IEnumerator RunAllRoutine()
    {
        BenchmarkRunning = true;
        var reset = FindFirstObjectByType<TabletopReset>();
        var summary = new StringBuilder($"DemoStitching transport benchmark ({System.DateTime.Now:yyyy-MM-dd HH:mm})\n");
        int placed = 0;
        foreach (var task in export.tasks)
        {
            if (reset) reset.ResetObjects();
            yield return new WaitForSeconds(0.5f);
            yield return pickPlace.MoveTo(arm.readyAimPoint);
            LastReport = "";
            yield return RunStitchedRoutine(task.startObject, task.endObject);
            bool ok = pickPlace.LastResult.StartsWith("placed");
            if (ok) placed++;
            summary.AppendLine($"- {LastReport}\n    pick/place: {pickPlace.LastResult}");
        }
        summary.AppendLine($"placed {placed}/{export.tasks.Length}");
        Directory.CreateDirectory("Verification");
        File.WriteAllText(Path.Combine("Verification", "stitched_summary.txt"), summary.ToString());
        Status = $"benchmark done: placed {placed}/{export.tasks.Length} (Verification/stitched_summary.txt)";
        Debug.Log("NERO_STITCHED_BENCHMARK:\n" + summary);
        BenchmarkRunning = false;
    }

    IEnumerator RunStitchedRoutine(string obj, string tray)
    {
        pickPlace.transport = Transport;
        pickPlace.Run(obj, tray);
        while (pickPlace.Busy) yield return null;
        pickPlace.transport = null;
    }

    /// <summary>Transport coroutine plugged into NeroPickPlace.transport.</summary>
    public IEnumerator Transport(Vector3 from, Vector3 to)
    {
        var task = FindTask(pickPlace.CurrentObject, pickPlace.CurrentTray);
        if (task == null)
        {
            Status = "no stitched task; straight move";
            yield return pickPlace.MoveTo(to);
            yield break;
        }

        var ds = new NeroChainedDS(export, task);
        Vector3 goal = ds.AttractorUnity;
        float oldMax = arm.maxLinearSpeed;
        arm.maxLinearSpeed = transportMaxSpeed;
        arm.controlOrientation = true;
        arm.followNaturalYaw = true;
        arm.azimuthPoint = Vector3.zero;
        arm.mode = NeroCartesianController.Mode.Velocity;

        var log = new StringBuilder("t,seg,transition,cmd_x,cmd_y,cmd_z,meas_x,meas_y,meas_z,v_x,v_y,v_z,clipped,locked_mask,q1,q2,q3,q4,q5,q6,q7\n");
        var executed = new List<Vector3>();
        float t0 = Time.fixedTime;
        int clipped = 0, steps = 0;
        float maxTilt = 0f;
        bool arrived = false;
        while (Time.fixedTime - t0 < timeout)
        {
            float t = Time.fixedTime - t0;
            Vector3 x = arm.CommandedToolPosition;
            float distToGoal = (x - goal).magnitude;
            if (distToGoal < arriveTolerance) { arrived = true; break; }
            Vector3 v = ds.StepUnity(x, t);
            // Final approach: once close and the DS has slowed down, hand over to the
            // precise Pose controller (small DS velocities are absorbed by IK damping).
            if (distToGoal < handoffDistance && v.magnitude < handoffSpeed) { arrived = true; break; }
            // Safety floor: never drive the carried object down into the table.
            if (x.y < minHeight && v.y < 0f) v.y = 0f;
            bool clip = v.magnitude > transportMaxSpeed;
            if (clip) clipped++;
            arm.commandedVelocity = v;
            Vector3 m = arm.ToolPosition;
            executed.Add(m);
            log.AppendLine(string.Join(",", new[] {
                t.ToString("F3", CultureInfo.InvariantCulture), ds.SegmentIndex.ToString(), ds.InTransition ? "1" : "0",
                F(x.x), F(x.y), F(x.z), F(m.x), F(m.y), F(m.z), F(v.x), F(v.y), F(v.z), clip ? "1" : "0",
                arm.LockedJointMask.ToString(), F(arm.JointCommand[0]), F(arm.JointCommand[1]), F(arm.JointCommand[2]), F(arm.JointCommand[3]), F(arm.JointCommand[4]), F(arm.JointCommand[5]), F(arm.JointCommand[6]) }));
            steps++;
            maxTilt = Mathf.Max(maxTilt, arm.RotationErrorDeg);
            Status = $"stitched {task.name}: segment {ds.SegmentIndex + 1}/{ds.SegmentCount}{(ds.InTransition ? " (blending)" : "")}, {(x - goal).magnitude * 100f:F1} cm to go";
            yield return new WaitForFixedUpdate();
        }
        arm.commandedVelocity = Vector3.zero;
        arm.followNaturalYaw = false;
        arm.maxLinearSpeed = oldMax;
        float duration = Time.fixedTime - t0;

        // Compare the executed path with the Python reference rollout (metres).
        float worst = 0f, sum = 0f;
        int M = task.referenceRollout.Length / 3;
        for (int i = 0; i < M; i++)
        {
            var r = ds.FromDs(new[] { task.referenceRollout[3 * i], task.referenceRollout[3 * i + 1], task.referenceRollout[3 * i + 2] });
            float best = float.MaxValue;
            foreach (var p in executed) best = Mathf.Min(best, (p - r).sqrMagnitude);
            best = Mathf.Sqrt(best);
            worst = Mathf.Max(worst, best);
            sum += best;
        }
        LastReport = $"{task.kind} {task.name}: {(arrived ? "arrived" : "TIMEOUT")} in {duration:F2} s, " +
                     $"path vs Python mean {sum / Mathf.Max(1, M) * 1000f:F1} mm / max {worst * 1000f:F1} mm, " +
                     $"max gripper tilt {maxTilt:F1} deg, speed-clipped {100f * clipped / Mathf.Max(1, steps):F0}% of steps";
        Status = LastReport;
        Debug.Log("NERO_STITCHED: " + LastReport);

        Directory.CreateDirectory("Verification");
        string safe = task.name.Replace(" -> ", "_to_").Replace(" ", "");
        File.WriteAllText(Path.Combine("Verification", $"stitched_{safe}.csv"), log.ToString());
        if (drawExecutedPath) DrawPath(executed, task.kind == "stitched" ? new Color(0.1f, 0.9f, 0.4f) : new Color(0.9f, 0.9f, 0.9f));

        // Finish precisely above the tray (also the fallback after a timeout).
        pickPlace.yawDeg = arm.NaturalYawDeg(to);
        yield return pickPlace.MoveTo(to);
    }

    static string F(float v) => v.ToString("F4", CultureInfo.InvariantCulture);

    void DrawPath(List<Vector3> pts, Color c)
    {
        if (pts.Count < 2) return;
        var go = new GameObject("Stitched path");
        var lr = go.AddComponent<LineRenderer>();
        lr.positionCount = pts.Count;
        lr.SetPositions(pts.ToArray());
        lr.widthMultiplier = 0.005f;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startColor = lr.endColor = c;
        lines.Add(lr);
    }

    public void ClearPaths()
    {
        foreach (var l in lines) if (l) Destroy(l.gameObject);
        lines.Clear();
    }

    void OnGUI()
    {
        if (!showPanel || pickPlace == null) return;
        int n = export != null ? export.tasks.Length : 0;
        GUILayout.BeginArea(new Rect(Screen.width - 316, 146, 300, 150 + 24 * n), GUI.skin.box);
        GUILayout.Label("DemoStitching transport (chained DS)");
        GUI.enabled = !pickPlace.Busy && !BenchmarkRunning && export != null;
        if (GUILayout.Button($"Run all {n} (benchmark)")) RunAll();
        if (export != null)
            foreach (var t in export.tasks)
            {
                string label = $"{Short(t.startObject)} -> {Short(t.endObject)}   ({(t.kind == "stitched" ? "STITCHED" : "demonstrated")})";
                if (GUILayout.Button(label)) RunStitched(t.startObject, t.endObject);
            }
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Clear paths")) ClearPaths();
        if (GUILayout.Button("Reload export")) Reload();
        GUILayout.EndHorizontal();
        GUI.enabled = true;
        GUILayout.Label(Status);
        GUILayout.Label("Pick/place: " + (pickPlace.Busy ? "running..." : pickPlace.LastResult));
        GUILayout.EndArea();
    }

    static string Short(string name) =>
        name.Replace(" Cube", "").Replace("Tray A - ", "").Replace("Tray B - ", "").ToLowerInvariant();
}
