using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Records transport demonstrations for DemoStitching.
///
/// Each demonstration is a family of smooth curved paths between two task
/// points (above an object -> above a tray) that pass through a shared "hub"
/// region in front of the robot. The arm executes every path with
/// NeroCartesianController (Velocity mode, feed-forward + feedback) and the
/// *measured* tool position is recorded, like a kinesthetic demonstration.
///
/// Output (DemoStitching format):
///   outputDir/demonstration_k/trajectory_j.json  { "x": [[x,y,z],...], "x_dot": [[...],...], ... }
///   outputDir/demo_plan.json                      frame, units and per-demo metadata
/// Frame: Unity world (left-handed, y up), metres; x_dot in m/s.
/// </summary>
public class NeroDemoRecorder : MonoBehaviour
{
    [System.Serializable]
    public class DemoSpec
    {
        public string name;
        public string startObject;
        public string endObject;
    }

    public NeroCartesianController arm;

    [Header("Demonstrations")]
    public DemoSpec[] demos =
    {
        new DemoSpec { name = "red_to_trayB",  startObject = "Red Cube",  endObject = "Tray B - Yellow" },
        new DemoSpec { name = "blue_to_trayA", startObject = "Blue Cube", endObject = "Tray A - White" },
    };
    public int trajectoriesPerDemo = 6;
    public int pointsPerTrajectory = 180;
    public int seed = 7;

    [Header("Path shape")]
    public float endHeight = 0.86f;
    public float hubHeight = 0.875f;
    public Vector3 hub = new Vector3(-0.13f, 0f, 0.0f);   // x,z used; y from hubHeight. ~0.32 m from the base: well inside the reach so the IK stays far from singular
    public float endJitter = 0.015f;     // m, start/end offsets in the table plane
    public float hubJitter = 0.02f;      // m
    public float heightJitter = 0.01f;   // m
    public Vector2 durationRange = new Vector2(5.0f, 6.0f);  // s

    public enum PathShape { BezierThroughHub, PolarArcThroughHub, PolarDirect }
    [Tooltip("BezierThroughHub: quadratic curve through 'hub'. PolarArcThroughHub: arc around arcCenter " +
             "(the robot base) through a hub at (hubRadius, hubAngleDeg); keeps a safe distance from the base.")]
    public PathShape pathShape = PathShape.BezierThroughHub;
    public Vector3 arcCenter = new Vector3(0f, 0f, -0.38f);   // x,z used
    public float hubRadius = 0.35f;                            // m from arcCenter
    public float hubAngleDeg = 0f;                             // 0 = +z, positive towards +x
    public float hubAngleJitterDeg = 4f;
    [Tooltip("PolarDirect: lift at mid-path (m) above the straight height profile.")]
    public float directLift = 0.015f;
    [Tooltip("If > 0, duration = clamp(1 s + path length / autoDurationSpeed, durationRange).")]
    public float autoDurationSpeed = 0f;

    [Header("Tracking")]
    public float trackingGain = 4f;
    public float recordMaxSpeed = 0.45f;

    [Header("Output")]
    [Tooltip("Relative paths are resolved against the Unity project folder.")]
    public string outputDir = "DemoData/nero_tabletop";
    public bool drawRecordedPaths = true;
    public bool showPanel = true;

    public bool Busy { get; private set; }
    public string Status { get; private set; } = "idle";

    readonly List<LineRenderer> lines = new List<LineRenderer>();
    static readonly Color[] demoColors = { new Color(0.95f, 0.3f, 0.2f), new Color(0.2f, 0.5f, 1f), new Color(0.2f, 0.8f, 0.3f), new Color(0.9f, 0.7f, 0.1f) };

    void Awake()
    {
        if (!arm) arm = GetComponent<NeroCartesianController>();
    }

    public string ResolvedOutputDir =>
        Path.IsPathRooted(outputDir) ? outputDir : Path.GetFullPath(Path.Combine(Application.dataPath, "..", outputDir));

    public void StartRecording()
    {
        if (!Busy) StartCoroutine(RecordAll());
    }

    public IEnumerator RecordAll()
    {
        Busy = true;
        var rng = new System.Random(seed);
        string root = ResolvedOutputDir;
        Directory.CreateDirectory(root);
        ClearLines();
        float oldMax = arm.maxLinearSpeed;
        arm.maxLinearSpeed = recordMaxSpeed;
        arm.gripperWidth = 0.09f;

        var plan = new StringBuilder();
        plan.Append("{\n  \"frame\": \"unity_world_left_handed_y_up\",\n  \"units\": {\"x\": \"m\", \"x_dot\": \"m/s\"},\n");
        plan.Append($"  \"points_per_trajectory\": {pointsPerTrajectory},\n  \"trajectories_per_demo\": {trajectoriesPerDemo},\n  \"seed\": {seed},\n  \"demonstrations\": [\n");

        float worstTrack = 0f;
        for (int d = 0; d < demos.Length; d++)
        {
            var spec = demos[d];
            Vector3 s0 = Above(spec.startObject), e0 = Above(spec.endObject);
            string demoDir = Path.Combine(root, $"demonstration_{d}");
            Directory.CreateDirectory(demoDir);

            for (int j = 0; j < trajectoriesPerDemo; j++)
            {
                Status = $"demo {d + 1}/{demos.Length} ({spec.name}), trajectory {j + 1}/{trajectoriesPerDemo}";
                Vector3 s = s0 + PlaneJitter(rng, endJitter);
                Vector3 e = e0 + PlaneJitter(rng, endJitter);
                Vector3 h;
                if (pathShape == PathShape.PolarDirect)
                {
                    ToPolar(s, out float rs_, out float ts_);
                    ToPolar(e, out float re_, out float te_);
                    h = Polar(0.5f * (rs_ + re_) + Range(rng, -hubJitter, hubJitter), 0.5f * (ts_ + te_),
                              Mathf.Max(s.y, e.y) + directLift + Range(rng, -heightJitter, heightJitter));
                }
                else if (pathShape == PathShape.PolarArcThroughHub)
                    h = Polar(hubRadius + Range(rng, -hubJitter, hubJitter),
                              hubAngleDeg + Range(rng, -hubAngleJitterDeg, hubAngleJitterDeg),
                              hubHeight + Range(rng, -heightJitter, heightJitter));
                else
                    h = new Vector3(hub.x, hubHeight, hub.z) + PlaneJitter(rng, hubJitter)
                        + Vector3.up * Range(rng, -heightJitter, heightJitter);
                float T = Range(rng, durationRange.x, durationRange.y);
                if (autoDurationSpeed > 0f)
                {
                    float L = 0f; Vector3 prev = s;
                    for (int k = 1; k <= 50; k++) { var q = ArcPoint(s, h, e, k / 50f); L += (q - prev).magnitude; prev = q; }
                    T = Mathf.Clamp(1f + L / autoDurationSpeed, durationRange.x, durationRange.y) * Range(rng, 0.95f, 1.05f);
                }

                // Move to the start (not recorded) and settle.
                yield return MoveToAndSettle(s);

                // Execute and record.
                var samples = new List<Vector3>();
                var times = new List<float>();
                arm.mode = NeroCartesianController.Mode.Velocity;
                arm.controlOrientation = true;
                arm.followNaturalYaw = true;
                float t0 = Time.fixedTime;
                float trackErr = 0f;
                while (true)
                {
                    float t = Time.fixedTime - t0;
                    float tau = Mathf.Clamp01(t / T);
                    RefPath(s, h, e, tau, T, out Vector3 pRef, out Vector3 vRef);
                    arm.azimuthPoint = pRef;
                    arm.commandedVelocity = vRef + trackingGain * (pRef - arm.CommandedToolPosition);
                    samples.Add(arm.ToolPosition);
                    times.Add(t);
                    trackErr = Mathf.Max(trackErr, (arm.ToolPosition - pRef).magnitude);
                    if (t >= T + 0.3f) break;   // short tail so the end velocity decays to ~0
                    yield return new WaitForFixedUpdate();
                }
                arm.commandedVelocity = Vector3.zero;
                arm.azimuthPoint = Vector3.zero;
                arm.followNaturalYaw = false;
                worstTrack = Mathf.Max(worstTrack, trackErr);

                Resample(samples, times, pointsPerTrajectory, out var x, out var xdot);
                WriteTrajectory(Path.Combine(demoDir, $"trajectory_{j}.json"), x, xdot, spec, j, T, trackErr);
                if (drawRecordedPaths) DrawLine(x, demoColors[d % demoColors.Length]);
            }

            plan.Append($"    {{\"demo_dir\": \"demonstration_{d}\", \"name\": \"{spec.name}\", \"start_object\": \"{spec.startObject}\", \"end_object\": \"{spec.endObject}\", ");
            plan.Append($"\"start_anchor\": {Vec(s0)}, \"end_anchor\": {Vec(e0)}, \"hub\": {Vec(new Vector3(hub.x, hubHeight, hub.z))}}}");
            plan.Append(d < demos.Length - 1 ? ",\n" : "\n");
        }
        plan.Append("  ]\n}\n");
        File.WriteAllText(Path.Combine(root, "demo_plan.json"), plan.ToString());

        arm.maxLinearSpeed = oldMax;
        arm.mode = NeroCartesianController.Mode.Hold;
        Status = $"done: {demos.Length * trajectoriesPerDemo} trajectories -> {root} (max tracking error {worstTrack * 1000f:F1} mm)";
        Debug.Log("NERO_DEMOS: " + Status);
        Busy = false;
    }

    // ---- path: quadratic Bezier through s -> h -> e with min-jerk timing ------
    void RefPath(Vector3 s, Vector3 h, Vector3 e, float tau, float T, out Vector3 p, out Vector3 v)
    {
        if (pathShape == PathShape.PolarArcThroughHub || pathShape == PathShape.PolarDirect)
        {
            // Velocity by central difference of the (smooth) position profile.
            const float d = 1e-3f;
            p = ArcPoint(s, h, e, tau);
            Vector3 pa = ArcPoint(s, h, e, Mathf.Max(0f, tau - d));
            Vector3 pb = ArcPoint(s, h, e, Mathf.Min(1f, tau + d));
            float span = (Mathf.Min(1f, tau + d) - Mathf.Max(0f, tau - d)) * T;
            v = span > 1e-6f ? (pb - pa) / span : Vector3.zero;
            return;
        }
        // Control point so the curve passes through h at u = 0.5.
        Vector3 c = 2f * h - 0.5f * (s + e);
        float u = 10f * tau * tau * tau - 15f * tau * tau * tau * tau + 6f * tau * tau * tau * tau * tau;
        float du = (30f * tau * tau - 60f * tau * tau * tau + 30f * tau * tau * tau * tau) / T;
        float a = 1f - u;
        p = a * a * s + 2f * a * u * c + u * u * e;
        Vector3 dp = 2f * a * (c - s) + 2f * u * (e - c);
        v = dp * du;
    }

    // ---- polar arc around arcCenter through the hub, min-jerk timing ----------
    Vector3 Polar(float r, float thetaDeg, float y)
    {
        float t = thetaDeg * Mathf.Deg2Rad;
        return new Vector3(arcCenter.x + r * Mathf.Sin(t), y, arcCenter.z + r * Mathf.Cos(t));
    }

    void ToPolar(Vector3 p, out float r, out float thetaDeg)
    {
        float dx = p.x - arcCenter.x, dz = p.z - arcCenter.z;
        r = Mathf.Sqrt(dx * dx + dz * dz);
        thetaDeg = Mathf.Atan2(dx, dz) * Mathf.Rad2Deg;
    }

    static float Smooth(float t) { t = Mathf.Clamp01(t); return t * t * (3f - 2f * t); }

    Vector3 ArcPoint(Vector3 s, Vector3 h, Vector3 e, float tau)
    {
        float u = 10f * tau * tau * tau - 15f * tau * tau * tau * tau + 6f * tau * tau * tau * tau * tau;
        ToPolar(s, out float rs, out float ts);
        ToPolar(h, out float rh, out float th);
        ToPolar(e, out float re, out float te);
        float theta = Mathf.Lerp(ts, te, u);
        // Radius blends start -> hub -> end as a function of angle (flat at hub and ends).
        float r = (te - ts) * (theta - th) <= 0f
            ? Mathf.Lerp(rs, rh, Smooth(Mathf.InverseLerp(ts, th, theta)))
            : Mathf.Lerp(rh, re, Smooth(Mathf.InverseLerp(th, te, theta)));
        // Height: linear between the ends plus a bump that peaks at the hub.
        float uh = Mathf.Clamp(Mathf.InverseLerp(ts, te, th), 0.05f, 0.95f);
        float bump = u <= uh ? Mathf.Sin(0.5f * Mathf.PI * u / uh) : Mathf.Sin(0.5f * Mathf.PI * (1f - u) / (1f - uh));
        float yLine = Mathf.Lerp(s.y, e.y, u);
        float y = yLine + (h.y - Mathf.Lerp(s.y, e.y, uh)) * bump;
        return Polar(r, theta, y);
    }

    IEnumerator MoveToAndSettle(Vector3 p)
    {
        // Joint-space pre-positioning into the natural posture, then settle.
        yield return arm.GoToNatural(p, arm.NaturalDown(p));
        yield return new WaitForSeconds(0.3f);
    }

    // ---- data handling ---------------------------------------------------
    static void Resample(List<Vector3> pts, List<float> ts, int n, out Vector3[] x, out Vector3[] xdot)
    {
        x = new Vector3[n];
        xdot = new Vector3[n];
        float T = ts[ts.Count - 1];
        int k = 0;
        for (int i = 0; i < n; i++)
        {
            float t = T * i / (n - 1);
            while (k < ts.Count - 2 && ts[k + 1] < t) k++;
            float span = Mathf.Max(1e-6f, ts[k + 1] - ts[k]);
            x[i] = Vector3.Lerp(pts[k], pts[k + 1], Mathf.Clamp01((t - ts[k]) / span));
        }
        float dt = T / (n - 1);
        for (int i = 0; i < n; i++)
        {
            int a = Mathf.Max(0, i - 1), b = Mathf.Min(n - 1, i + 1);
            xdot[i] = (x[b] - x[a]) / ((b - a) * dt);
        }
        xdot[n - 1] = Vector3.zero;   // trajectories end at rest at the attractor
    }

    static void WriteTrajectory(string path, Vector3[] x, Vector3[] xdot, DemoSpec spec, int idx, float duration, float trackErr)
    {
        var sb = new StringBuilder();
        sb.Append("{\"x\": [");
        for (int i = 0; i < x.Length; i++) sb.Append(i > 0 ? ", " : "").Append(Vec(x[i]));
        sb.Append("], \"x_dot\": [");
        for (int i = 0; i < xdot.Length; i++) sb.Append(i > 0 ? ", " : "").Append(Vec(xdot[i]));
        sb.Append($"], \"source_task\": \"{spec.name}\", \"source_demo_idx\": {idx}, \"segment_role\": \"transport\"");
        sb.Append(", \"duration_s\": ").Append(duration.ToString("F3", CultureInfo.InvariantCulture));
        sb.Append(", \"max_tracking_error_m\": ").Append(trackErr.ToString("F4", CultureInfo.InvariantCulture));
        sb.Append("}\n");
        File.WriteAllText(path, sb.ToString());
    }

    static string Vec(Vector3 v) =>
        "[" + v.x.ToString("R", CultureInfo.InvariantCulture) + ", " + v.y.ToString("R", CultureInfo.InvariantCulture) + ", " + v.z.ToString("R", CultureInfo.InvariantCulture) + "]";

    Vector3 Above(string obj)
    {
        var go = GameObject.Find(obj);
        var p = go.transform.position;
        return new Vector3(p.x, endHeight, p.z);
    }

    static Vector3 PlaneJitter(System.Random r, float a) => new Vector3(Range(r, -a, a), 0f, Range(r, -a, a));
    static float Range(System.Random r, float lo, float hi) => lo + (float)r.NextDouble() * (hi - lo);

    // ---- visualisation -----------------------------------------------------
    void DrawLine(Vector3[] pts, Color c)
    {
        var go = new GameObject("Demo path");
        var lr = go.AddComponent<LineRenderer>();
        lr.positionCount = pts.Length;
        lr.SetPositions(pts);
        lr.widthMultiplier = 0.004f;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startColor = lr.endColor = c;
        lines.Add(lr);
    }

    public void ClearLines()
    {
        foreach (var l in lines) if (l) Destroy(l.gameObject);
        lines.Clear();
    }

    void OnGUI()
    {
        if (!showPanel) return;
        GUILayout.BeginArea(new Rect(Screen.width - 296, 16, 280, 120), GUI.skin.box);
        GUILayout.Label("Demo recorder (DemoStitching)");
        GUI.enabled = !Busy;
        if (GUILayout.Button("Record demonstrations")) StartRecording();
        if (GUILayout.Button("Clear drawn paths")) ClearLines();
        GUI.enabled = true;
        GUILayout.Label(Status);
        GUILayout.EndArea();
    }
}
