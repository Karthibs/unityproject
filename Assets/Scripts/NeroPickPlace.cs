using System.Collections;
using UnityEngine;

/// <summary>
/// Pick-and-place sequencer for the NERO arm.
///
///   above object -> descend -> close -> lift -> [transport] -> descend -> open -> retreat
///
/// The transport phase is a pluggable coroutine: by default a straight Pose
/// move to the point above the tray; the DemoStitching controller replaces it
/// with a stitched dynamical system.
/// </summary>
public class NeroPickPlace : MonoBehaviour
{
    public NeroCartesianController arm;

    [Header("Heights (world y of the tool point)")]
    public float approachHeight = 0.86f;
    public float graspHeight = 0.765f;   // cube centre
    public float placeHeight = 0.80f;

    [Header("Gripper")]
    public float openWidth = 0.09f;
    public float closedWidth = 0.0f;     // force-limited; fingers stop on the object
    public float gripperSettle = 0.6f;

    [Header("Motion")]
    public float posTol = 0.008f;
    public float rotTol = 3f;
    public float moveTimeout = 8f;
    public float yawDeg = 0f;

    /// <summary>Optional transport phase: (from, to) -> coroutine that moves the tool.</summary>
    public System.Func<Vector3, Vector3, IEnumerator> transport;

    public bool Busy { get; private set; }
    public string LastResult { get; private set; } = "";
    public string CurrentObject { get; private set; } = "";
    public string CurrentTray { get; private set; } = "";

    void Awake()
    {
        if (!arm) arm = GetComponent<NeroCartesianController>();
    }

    [Header("Test panel")]
    public bool showTestPanel = true;
    public string[] testObjects = { "Red Cube", "Blue Cube", "Green Cube", "Purple Cube" };
    public string[] testTrays = { "Tray A - White", "Tray B - Yellow" };

    void OnGUI()
    {
        if (!showTestPanel || !arm) return;
        GUILayout.BeginArea(new Rect(16, 110, 290, 120 + 24 * testObjects.Length + 110), GUI.skin.box);
        GUILayout.Label("NERO test panel (straight-line transport)");
        GUI.enabled = !Busy;
        foreach (var obj in testObjects)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(obj.Replace(" Cube", ""), GUILayout.Width(56));
            foreach (var tray in testTrays)
            {
                string label = "-> " + (tray.Contains("White") ? "white" : tray.Contains("Yellow") ? "yellow" : tray);
                if (GUILayout.Button(label)) Run(obj, tray);
            }
            GUILayout.EndHorizontal();
        }
        if (GUILayout.Button("Move to ready pose")) StartCoroutine(arm.GoToNatural(arm.readyAimPoint, NeroCartesianController.DownRotation(yawDeg)));
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Open gripper")) arm.gripperWidth = openWidth;
        if (GUILayout.Button("Close gripper")) arm.gripperWidth = closedWidth;
        GUILayout.EndHorizontal();
        GUI.enabled = true;
        GUILayout.Label("Status: " + (Busy ? "running..." : LastResult));
        GUILayout.Label($"Tool: {arm.ToolPosition:F3}");
        GUILayout.Label($"Error: {arm.PositionError * 1000f:F1} mm, {arm.RotationErrorDeg:F1} deg");
        GUILayout.Label("Tip: use 'Reset blocks' above between runs.");
        GUILayout.EndArea();
    }

    public void Run(string objectName, string trayName)
    {
        if (Busy) return;
        var obj = GameObject.Find(objectName);
        var tray = GameObject.Find(trayName);
        if (!obj || !tray) { LastResult = "missing object or tray"; return; }
        StartCoroutine(PickAndPlace(obj.GetComponent<Rigidbody>(), tray.transform));
    }

    public IEnumerator PickAndPlace(Rigidbody obj, Transform tray)
    {
        Busy = true;
        LastResult = "running";
        CurrentObject = obj.name;
        CurrentTray = tray.name;
        Vector3 o = obj.position, t = tray.position;
        Vector3 aboveObj = new Vector3(o.x, approachHeight, o.z);
        Vector3 aboveTray = new Vector3(t.x, approachHeight, t.z);

        arm.gripperWidth = openWidth;
        // Grasp yaw: the arm's natural yaw at the object, snapped to 90 deg so the
        // fingers stay square to the (axis-aligned) cube.
        yawDeg = Mathf.Round(arm.NaturalYawDeg(aboveObj) / 90f) * 90f;
        yield return arm.GoToNatural(aboveObj, NeroCartesianController.DownRotation(yawDeg));
        yield return MoveTo(new Vector3(o.x, graspHeight, o.z));
        arm.gripperWidth = closedWidth;
        yield return new WaitForSeconds(gripperSettle);
        yield return MoveTo(aboveObj);

        if (obj.position.y < graspHeight + 0.05f)
        {
            LastResult = $"grasp failed: object y={obj.position.y:F3}";
            arm.gripperWidth = openWidth;
            Busy = false;
            yield break;
        }

        if (transport != null) yield return transport(aboveObj, aboveTray);
        else yield return MoveTo(aboveTray);
        // Place with the natural yaw at the tray (cube yaw does not matter in the tray).
        yawDeg = arm.NaturalYawDeg(aboveTray);

        yield return MoveTo(new Vector3(t.x, placeHeight, t.z));
        arm.gripperWidth = openWidth;
        yield return new WaitForSeconds(gripperSettle);
        yield return MoveTo(aboveTray);

        yield return new WaitForSeconds(0.5f);
        Vector3 local = tray.InverseTransformPoint(obj.position);
        bool inTray = Mathf.Abs(local.x) < 0.1f && Mathf.Abs(local.z) < 0.09f && local.y < 0.06f;
        LastResult = inTray ? $"placed: object at {obj.position:F3}" : $"place missed: object at {obj.position:F3}";
        Busy = false;
    }

    public IEnumerator MoveTo(Vector3 p)
    {
        arm.targetPosition = p;
        arm.targetRotation = NeroCartesianController.DownRotation(yawDeg);
        arm.mode = NeroCartesianController.Mode.Pose;
        float t0 = Time.time;
        while (Time.time - t0 < moveTimeout)
        {
            if (arm.PositionError < posTol && arm.RotationErrorDeg < rotTol) yield break;
            yield return new WaitForFixedUpdate();
        }
        Debug.LogWarning($"NeroPickPlace: move to {p:F3} timed out (err {arm.PositionError:F3} m, {arm.RotationErrorDeg:F1} deg)");
    }
}
