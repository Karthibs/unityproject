using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Workspace grid + DemoStitching navigation for the NERO arm.
///
/// The reachable half-ring in front of the robot is split into rings x sectors
/// (default 3 x 6 = 18 cells, named A1..C6: A = inner ring, columns left to
/// right as seen from above with the robot at the bottom). One demonstration per
/// cell goes from the cell to the rest position; DemoStitching then builds a
/// route from any cell to any other cell (the second half uses a reversed demo).
///
/// The GUI lets you pick a start and an end cell and run the stitched route:
/// the arm first goes to the start cell (joint-space move into the natural
/// posture), then follows the exported chained DS(s) to the end cell.
/// </summary>
public class NeroGridNavigator : MonoBehaviour
{
    public NeroCartesianController arm;

    [Header("Grid (polar around the robot base)")]
    public Vector3 center = new Vector3(0f, 0f, -0.38f);           // x,z used
    public float[] ringRadii = { 0.30f, 0.37f, 0.44f };             // A, B, C
    public float ringHalfWidth = 0.035f;
    public float[] sectorAnglesDeg = { -75f, -45f, -15f, 15f, 45f, 75f };   // 0 = straight ahead (+z), - = left (-x)
    public float sectorHalfWidthDeg = 15f;
    public float travelHeight = 0.86f;
    public float tableHeight = 0.742f;
    [Tooltip("Rest position the demos go to (the 'hub' of the stitching graph).")]
    public Vector3 restPoint = new Vector3(0f, 0.86f, -0.02f);

    [Header("Stitched routes")]
    [Tooltip("Relative to the Unity project folder.")]
    public string exportPath = "DemoData/chained_ds_grid_unity.json";
    public float maxSpeed = 0.40f;
    public float arriveTolerance = 0.006f;
    public float handoffDistance = 0.035f;
    public float handoffSpeed = 0.03f;
    public float legTimeout = 25f;
    [Tooltip("Time scaling of the stitched DS: velocity x speedScale and DS clock x speedScale. " +
             "Same path and convergence, just faster (capped by maxSpeed).")]
    public float speedScale = 2f;
    [Tooltip("Minimum speed along the stitched path (m/s) while not yet near the goal. The DS direction is kept, " +
             "so the path is unchanged; demos end at rest with ~0 velocity, which otherwise makes the arm crawl " +
             "(and stall in the IK) when a route passes through the rest area.")]
    public float minPathSpeed = 0.04f;

    [Header("Safety net")]
    [Tooltip("Abort a stitched leg if it heads outside this radius band (m from the base) or below safeMinHeight, " +
             "or commands more than abortSpeed for longer than abortTime; the route is then finished via the rest " +
             "position with the end cell's reversed demo model.")]
    public Vector2 safeRadius = new Vector2(0.26f, 0.49f);
    public float safeMinHeight = 0.82f;
    public float abortSpeed = 0.6f;
    public float abortTime = 0.2f;

    [Header("Display")]
    public bool drawGrid = true;
    public bool showPanel = true;

    // ---- export format (written by Tools/demostitching/export_grid.py) ----
    [System.Serializable] public class Route
    {
        public string start, end, kind;   // kind: "direct" (one stitched chain) or "via_rest" (two chains)
        public int[] chains;
    }

    [System.Serializable] public class GridExport
    {
        public string format;
        public double scale;
        public int[] axisPerm;
        public double dtReference;
        public double velocityMax;
        public NeroChainedDS.Task[] tasks;
        public Route[] routes;
    }

    public int Rows => ringRadii.Length;
    public int Cols => sectorAnglesDeg.Length;
    public bool Busy { get; private set; }
    public string Status { get; private set; } = "";
    public string LastReport { get; private set; } = "";

    GridExport export;
    NeroChainedDS.Export dsExport;
    int selStart = -1, selEnd = -1;
    readonly List<LineRenderer> cellLines = new List<LineRenderer>();
    readonly List<LineRenderer> pathLines = new List<LineRenderer>();
    Material lineMat;

    // ---- cell geometry -------------------------------------------------------
    public static string CellName(int row, int col) => $"{(char)('A' + row)}{col + 1}";

    public Vector3 CellPoint(int row, int col, float y)
    {
        float t = sectorAnglesDeg[col] * Mathf.Deg2Rad;
        float r = ringRadii[row];
        return new Vector3(center.x + r * Mathf.Sin(t), y, center.z + r * Mathf.Cos(t));
    }

    public Vector3 CellPoint(int index, float y) => CellPoint(index / Cols, index % Cols, y);
    public string CellName(int index) => CellName(index / Cols, index % Cols);

    void Awake()
    {
        if (!arm) arm = GetComponent<NeroCartesianController>();
        CreateMarkers();
    }

    void Start()
    {
        if (drawGrid) DrawGrid();
        Reload();
    }

    /// <summary>Invisible marker objects ("Cell A1", ..., "Grid Rest") used by the demo recorder.</summary>
    void CreateMarkers()
    {
        var root = GameObject.Find("Workspace Grid") ?? new GameObject("Workspace Grid");
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
            {
                string n = "Cell " + CellName(r, c);
                var go = GameObject.Find(n) ?? new GameObject(n);
                go.transform.SetParent(root.transform, false);
                go.transform.position = CellPoint(r, c, tableHeight);
            }
        var rest = GameObject.Find("Grid Rest") ?? new GameObject("Grid Rest");
        rest.transform.SetParent(root.transform, false);
        rest.transform.position = new Vector3(restPoint.x, tableHeight, restPoint.z);
    }

    public void Reload()
    {
        string path = Path.IsPathRooted(exportPath) ? exportPath : Path.GetFullPath(Path.Combine(Application.dataPath, "..", exportPath));
        if (!File.Exists(path)) { export = null; Status = "no grid routes yet (" + exportPath + ")"; return; }
        export = JsonUtility.FromJson<GridExport>(File.ReadAllText(path));
        dsExport = new NeroChainedDS.Export
        {
            format = export.format, scale = export.scale, axisPerm = export.axisPerm,
            dtReference = export.dtReference, velocityMax = export.velocityMax, tasks = export.tasks,
        };
        Status = $"loaded {export.routes.Length} routes ({export.tasks.Length} chains)";
    }

    public Route FindRoute(string start, string end)
    {
        if (export == null) return null;
        foreach (var r in export.routes) if (r.start == start && r.end == end) return r;
        return null;
    }

    int FindTaskIndex(string name)
    {
        if (export == null) return -1;
        for (int i = 0; i < export.tasks.Length; i++) if (export.tasks[i].name == name) return i;
        return -1;
    }

    // ---- running a route -------------------------------------------------------
    public void Go(int start, int end)
    {
        if (Busy || start < 0 || end < 0 || start == end) return;
        StartCoroutine(GoRoutine(start, end, null));
    }

    IEnumerator GoRoutine(int start, int end, List<string> reportSink)
    {
        Busy = true;
        string s = CellName(start), e = CellName(end);
        var route = FindRoute(s, e);
        if (route == null) { Status = $"no route {s} -> {e}"; Busy = false; yield break; }

        Vector3 startP = CellPoint(start, travelHeight), endP = CellPoint(end, travelHeight);
        Status = $"moving to start cell {s}";
        yield return arm.GoToNatural(startP, arm.NaturalDown(startP));

        var executed = new List<Vector3>();
        float t0 = Time.time;
        bool allArrived = true, usedFallback = false;
        for (int k = 0; k < route.chains.Length; k++)
        {
            var ds = new NeroChainedDS(dsExport, export.tasks[route.chains[k]]);
            Status = $"{s} -> {e}: stitched leg {k + 1}/{route.chains.Length}";
            int st = 1;
            yield return FollowChain(ds, executed, v => st = v);
            if (st == 2)
            {
                // Safety net: go to the rest position, then the end cell's reversed demo model.
                usedFallback = true;
                Status = $"{s} -> {e}: leg {k + 1} aborted (safety net) -> via rest";
                Debug.LogWarning("NeroGridNavigator: " + Status);
                yield return arm.GoToNatural(restPoint, arm.NaturalDown(restPoint));
                int fb = FindTaskIndex($"rest->{e}");
                if (fb >= 0)
                {
                    int st2 = 1;
                    yield return FollowChain(new NeroChainedDS(dsExport, export.tasks[fb]), executed, v => st2 = v);
                    allArrived &= st2 == 0;
                }
                else allArrived = false;
                break;
            }
            allArrived &= st == 0;
        }
        // Finish precisely on the end cell.
        arm.targetPosition = endP;
        arm.targetRotation = arm.NaturalDown(endP);
        arm.mode = NeroCartesianController.Mode.Pose;
        float tp = Time.time;
        while (arm.PositionError > 0.004f && Time.time - tp < 4f) yield return new WaitForFixedUpdate();
        float duration = Time.time - t0;

        float pathLen = 0f, minR = 9f, maxR = 0f;
        for (int i = 1; i < executed.Count; i++) pathLen += (executed[i] - executed[i - 1]).magnitude;
        foreach (var p in executed)
        {
            float r = new Vector2(p.x - center.x, p.z - center.z).magnitude;
            minR = Mathf.Min(minR, r); maxR = Mathf.Max(maxR, r);
        }
        float endErr = (arm.ToolPosition - endP).magnitude;
        LastReport = $"{s} -> {e} ({route.kind}{(usedFallback ? " + SAFETY FALLBACK" : "")}, {route.chains.Length} chain{(route.chains.Length > 1 ? "s" : "")}): " +
                     $"{(allArrived ? "arrived" : "TIMEOUT")}, {duration:F1} s, path {pathLen:F2} m, " +
                     $"radius {minR:F2}-{maxR:F2} m, end error {endErr * 1000f:F1} mm";
        Status = LastReport;
        reportSink?.Add(LastReport);
        SaveRun(s, e, route, allArrived, usedFallback, duration, endErr, executed);
        DrawPath(executed, route.kind == "direct" ? new Color(0.1f, 0.9f, 0.4f) : new Color(1f, 0.6f, 0.1f));
        Busy = false;
    }

    /// <summary>Executed tool path of one route (100 Hz, stitched legs only) -> Verification/grid_runs/S_E.csv.</summary>
    void SaveRun(string s, string e, Route route, bool arrived, bool fallback, float duration, float endErr, List<Vector3> executed)
    {
        var ci = CultureInfo.InvariantCulture;
        var sb = new StringBuilder();
        sb.AppendLine($"# start={s} end={e} kind={route.kind} chains={string.Join(";", route.chains)} arrived={arrived} fallback={fallback} " +
                      $"duration_s={duration.ToString("F3", ci)} end_error_m={endErr.ToString("F4", ci)} dt=0.01 frame=unity_world");
        sb.AppendLine("x,y,z");
        foreach (var p in executed)
            sb.AppendLine(p.x.ToString("F5", ci) + "," + p.y.ToString("F5", ci) + "," + p.z.ToString("F5", ci));
        string dir = Path.Combine("Verification", "grid_runs");
        Directory.CreateDirectory(dir);
        File.WriteAllText(Path.Combine(dir, $"{s}_{e}.csv"), sb.ToString());
    }

    /// <summary>Run a list of routes given as "A1-C4,B2-A6,..." and write Verification/grid_benchmark.txt.</summary>
    public void RunPairs(string pairs)
    {
        if (Busy || export == null) return;
        var list = new List<(int, int)>();
        foreach (var tok in pairs.Split(','))
        {
            var ab = tok.Trim().Split('-');
            if (ab.Length != 2) continue;
            int a = IndexOf(ab[0]), b = IndexOf(ab[1]);
            if (a >= 0 && b >= 0 && a != b) list.Add((a, b));
        }
        StartCoroutine(RunListRoutine(list, $"{list.Count} selected routes"));
    }

    int IndexOf(string cell)
    {
        for (int i = 0; i < Rows * Cols; i++) if (CellName(i) == cell) return i;
        return -1;
    }

    // status: 0 = arrived, 1 = timeout, 2 = aborted by the safety net
    IEnumerator FollowChain(NeroChainedDS ds, List<Vector3> executed, System.Action<int> done)
    {
        Vector3 goal = ds.AttractorUnity;
        float oldMax = arm.maxLinearSpeed;
        arm.maxLinearSpeed = maxSpeed;
        arm.controlOrientation = true;
        arm.followNaturalYaw = true;
        arm.azimuthPoint = Vector3.zero;
        arm.mode = NeroCartesianController.Mode.Velocity;
        float t0 = Time.fixedTime;
        int status = 1;
        float fastFor = 0f;
        while (Time.fixedTime - t0 < legTimeout)
        {
            float t = Time.fixedTime - t0;
            Vector3 x = arm.CommandedToolPosition;
            float dist = (x - goal).magnitude;
            if (dist < arriveTolerance) { status = 0; break; }
            Vector3 v = ds.StepUnity(x, t * speedScale) * speedScale;
            if (dist < handoffDistance && v.magnitude < handoffSpeed * speedScale) { status = 0; break; }
            // ---- safety net: speed spike or leaving the safe workspace band -> abort
            fastFor = v.magnitude > abortSpeed ? fastFor + Time.fixedDeltaTime : 0f;
            float rad = new Vector2(x.x - center.x, x.z - center.z).magnitude;
            Vector3 xNext = x + v.normalized * 0.01f;
            float radNext = new Vector2(xNext.x - center.x, xNext.z - center.z).magnitude;
            bool leaving = (rad > safeRadius.y && radNext > rad) || (rad < safeRadius.x && radNext < rad) || (x.y < safeMinHeight && v.y < 0f);
            if (fastFor > abortTime || leaving) { status = 2; break; }
            float sp = v.magnitude;
            if (dist > 2f * handoffDistance && sp > 1e-6f && sp < minPathSpeed) v *= minPathSpeed / sp;
            if (x.y < 0.82f && v.y < 0f) v.y = 0f;   // never dive into the table
            arm.commandedVelocity = v;
            executed.Add(arm.ToolPosition);
            yield return new WaitForFixedUpdate();
        }
        arm.commandedVelocity = Vector3.zero;
        arm.followNaturalYaw = false;
        arm.maxLinearSpeed = oldMax;
        done(status);
    }

    /// <summary>Run n random start/end pairs and write Verification/grid_benchmark.txt.</summary>
    public void RunRandom(int n, int seed = 1)
    {
        if (!Busy && export != null) StartCoroutine(RandomRoutine(n, seed));
    }

    IEnumerator RandomRoutine(int n, int seed)
    {
        var rng = new System.Random(seed);
        int cells = Rows * Cols;
        var list = new List<(int, int)>();
        for (int i = 0; i < n; i++)
        {
            int a = rng.Next(cells), b;
            do b = rng.Next(cells); while (b == a);
            list.Add((a, b));
        }
        yield return RunListRoutine(list, $"{n} random routes");
    }

    IEnumerator RunListRoutine(List<(int, int)> list, string title)
    {
        var reports = new List<string>();
        foreach (var (a, b) in list)
        {
            selStart = a; selEnd = b; HighlightSelection();
            yield return GoRoutine(a, b, reports);
        }
        int ok = reports.FindAll(r => r.Contains("arrived")).Count;
        var sb = new StringBuilder($"Grid navigation benchmark ({System.DateTime.Now:yyyy-MM-dd HH:mm}), {title}\n");
        foreach (var r in reports) sb.AppendLine("- " + r);
        sb.AppendLine($"arrived {ok}/{list.Count}");
        Directory.CreateDirectory("Verification");
        File.WriteAllText(Path.Combine("Verification", "grid_benchmark.txt"), sb.ToString());
        Status = $"benchmark: arrived {ok}/{list.Count} (Verification/grid_benchmark.txt)";
    }

    // ---- drawing -----------------------------------------------------------------
    Material LineMat() => lineMat ? lineMat : (lineMat = new Material(Shader.Find("Sprites/Default")));

    void DrawGrid()
    {
        foreach (var l in cellLines) if (l) Destroy(l.gameObject);
        cellLines.Clear();
        var root = GameObject.Find("Workspace Grid").transform;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
            {
                var go = new GameObject("Grid line " + CellName(r, c));
                go.transform.SetParent(root, false);
                var lr = go.AddComponent<LineRenderer>();
                lr.loop = true;
                lr.widthMultiplier = 0.003f;
                lr.material = LineMat();
                var pts = new List<Vector3>();
                float r0 = ringRadii[r] - ringHalfWidth, r1 = ringRadii[r] + ringHalfWidth;
                float a0 = sectorAnglesDeg[c] - sectorHalfWidthDeg, a1 = sectorAnglesDeg[c] + sectorHalfWidthDeg;
                const int seg = 8;
                for (int k = 0; k <= seg; k++) pts.Add(PolarAt(r0, Mathf.Lerp(a0, a1, k / (float)seg)));
                for (int k = seg; k >= 0; k--) pts.Add(PolarAt(r1, Mathf.Lerp(a0, a1, k / (float)seg)));
                lr.positionCount = pts.Count;
                lr.SetPositions(pts.ToArray());
                cellLines.Add(lr);
            }
        HighlightSelection();
    }

    Vector3 PolarAt(float r, float deg)
    {
        float t = deg * Mathf.Deg2Rad;
        return new Vector3(center.x + r * Mathf.Sin(t), tableHeight + 0.002f, center.z + r * Mathf.Cos(t));
    }

    void HighlightSelection()
    {
        for (int i = 0; i < cellLines.Count; i++)
        {
            if (!cellLines[i]) continue;
            Color col = i == selStart ? new Color(0.1f, 0.9f, 0.2f) : i == selEnd ? new Color(1f, 0.25f, 0.2f) : new Color(1f, 1f, 1f, 0.55f);
            cellLines[i].startColor = cellLines[i].endColor = col;
            cellLines[i].widthMultiplier = (i == selStart || i == selEnd) ? 0.006f : 0.003f;
        }
    }

    void DrawPath(List<Vector3> pts, Color c)
    {
        if (pts.Count < 2) return;
        var go = new GameObject("Grid route");
        var lr = go.AddComponent<LineRenderer>();
        lr.positionCount = pts.Count;
        lr.SetPositions(pts.ToArray());
        lr.widthMultiplier = 0.005f;
        lr.material = LineMat();
        lr.startColor = lr.endColor = c;
        pathLines.Add(lr);
    }

    public void ClearPaths()
    {
        foreach (var l in pathLines) if (l) Destroy(l.gameObject);
        pathLines.Clear();
    }

    // ---- GUI -------------------------------------------------------------------------
    void OnGUI()
    {
        if (!showPanel || arm == null) return;
        // Cell labels in the Game view.
        var cam = Camera.main;
        if (drawGrid && cam)
        {
            var style = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter };
            for (int i = 0; i < Rows * Cols; i++)
            {
                Vector3 sp = cam.WorldToScreenPoint(CellPoint(i, tableHeight));
                if (sp.z <= 0) continue;
                GUI.Label(new Rect(sp.x - 20, Screen.height - sp.y - 10, 40, 20), CellName(i), style);
            }
        }

        const float cell = 44f;
        float w = 24 + cell * Cols, h = 230 + cell * Rows;
        GUILayout.BeginArea(new Rect(16, 16, w, h), GUI.skin.box);
        GUILayout.Label("DemoStitching grid navigation  (robot at the bottom)");
        GUILayout.Label("Click START, then END:   " +
                        $"start = {(selStart >= 0 ? CellName(selStart) : "-")}   end = {(selEnd >= 0 ? CellName(selEnd) : "-")}");
        GUI.enabled = !Busy;
        // Outer ring (C) on top, inner ring (A) at the bottom; columns left -> right.
        for (int r = Rows - 1; r >= 0; r--)
        {
            GUILayout.BeginHorizontal();
            for (int c = 0; c < Cols; c++)
            {
                int idx = r * Cols + c;
                var old = GUI.backgroundColor;
                if (idx == selStart) GUI.backgroundColor = Color.green;
                else if (idx == selEnd) GUI.backgroundColor = new Color(1f, 0.35f, 0.3f);
                if (GUILayout.Button(CellName(r, c), GUILayout.Width(cell - 4), GUILayout.Height(cell - 8)))
                {
                    if (selStart < 0 || (selStart >= 0 && selEnd >= 0)) { selStart = idx; selEnd = -1; }
                    else if (idx != selStart) selEnd = idx;
                    HighlightSelection();
                }
                GUI.backgroundColor = old;
            }
            GUILayout.EndHorizontal();
        }
        GUILayout.BeginHorizontal();
        GUI.enabled = !Busy && selStart >= 0 && selEnd >= 0 && export != null;
        if (GUILayout.Button("GO", GUILayout.Height(28))) Go(selStart, selEnd);
        GUI.enabled = !Busy;
        if (GUILayout.Button("Swap")) { (selStart, selEnd) = (selEnd, selStart); HighlightSelection(); }
        if (GUILayout.Button("Clear")) { selStart = selEnd = -1; HighlightSelection(); }
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        GUI.enabled = !Busy && export != null;
        if (GUILayout.Button("10 random routes")) RunRandom(10, Random.Range(1, 100000));
        GUI.enabled = !Busy;
        if (GUILayout.Button("Rest pose")) StartCoroutine(arm.GoToNatural(restPoint, arm.NaturalDown(restPoint)));
        if (GUILayout.Button("Clear paths")) ClearPaths();
        GUILayout.EndHorizontal();
        GUI.enabled = true;
        GUILayout.Label(Busy ? Status : (string.IsNullOrEmpty(LastReport) ? Status : LastReport));
        GUILayout.Label("green path = one stitched chain, orange = via rest (2 chains)");
        GUILayout.EndArea();
    }
}
