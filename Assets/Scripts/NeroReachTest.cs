using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Play-mode reachability sweep for NeroCartesianController. Add at runtime;
/// it visits named task points plus a grid over the tabletop with the gripper
/// pointing down, and writes Verification/reach-test.txt.
/// </summary>
public class NeroReachTest : MonoBehaviour
{
    public float settleTime = 3f;
    public float timeScale = 4f;
    public float posTol = 0.01f;      // m
    public float rotTol = 5f;         // deg
    public float[] heights = { 0.78f, 0.90f };
    [Tooltip("Set to 0 to test only the named task points.")]
    public float gridStep = 0.1f;
    public float approachHeight = 0.86f;
    public bool Done { get; private set; }

    IEnumerator Start()
    {
        var c = GetComponent<NeroCartesianController>();
        var points = new List<(string, Vector3)>();
        void Add(string name, string obj, float y)
        {
            var go = GameObject.Find(obj);
            if (go) points.Add((name, new Vector3(go.transform.position.x, y, go.transform.position.z)));
        }
        var cubes = new List<string>();
        var reset = FindFirstObjectByType<TabletopReset>();
        if (reset) foreach (var rb in reset.objects) if (rb) cubes.Add(rb.name);
        string[] trays = { "Tray A - White", "Tray B - Yellow" };
        foreach (var cube in cubes) Add(cube + " grasp", cube, 0.765f);
        foreach (var t in trays) Add(t + " place", t, 0.80f);
        foreach (var cube in cubes) Add(cube + " above", cube, approachHeight);
        foreach (var t in trays) Add(t + " above", t, approachHeight);
        if (gridStep > 0f)
        foreach (float y in heights)
            for (float x = -0.65f; x <= 0.651f; x += gridStep)
                for (float z = -0.45f; z <= 0.451f; z += gridStep)
                    points.Add(("grid", new Vector3(x, y, z)));

        float oldScale = Time.timeScale;
        Time.timeScale = timeScale;
        var sb = new StringBuilder("name,x,y,z,pos_err_m,rot_err_deg,reached\n");
        int reached = 0;
        foreach (var (name, p) in points)
        {
            c.mode = NeroCartesianController.Mode.Hold;
            c.TeleportJoints(c.PostureFor(p));
            yield return new WaitForFixedUpdate();
            c.targetPosition = p;
            c.targetRotation = NeroCartesianController.DownRotation();
            c.mode = NeroCartesianController.Mode.Pose;
            float t0 = Time.time;
            while (Time.time - t0 < settleTime)
            {
                if (Time.time - t0 > 0.5f && c.PositionError < posTol * 0.3f && c.RotationErrorDeg < rotTol * 0.3f) break;
                yield return new WaitForFixedUpdate();
            }
            bool ok = c.PositionError < posTol && c.RotationErrorDeg < rotTol;
            if (ok) reached++;
            sb.AppendLine($"{name},{p.x:F3},{p.y:F3},{p.z:F3},{c.PositionError:F4},{c.RotationErrorDeg:F2},{ok}");
        }
        Time.timeScale = oldScale;
        c.mode = NeroCartesianController.Mode.Hold;
        Directory.CreateDirectory("Verification");
        File.WriteAllText("Verification/reach-test.txt", sb.ToString());
        Debug.Log($"NERO_REACH_TEST: {reached}/{points.Count} reached");
        Done = true;
    }
}
