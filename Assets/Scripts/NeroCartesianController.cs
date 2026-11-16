using UnityEngine;

/// <summary>
/// Cartesian controller for the NERO 7-DOF arm built on Unity ArticulationBodies.
///
/// Each physics step it builds the 6x7 geometric Jacobian of the tool frame,
/// solves damped-least-squares IK for a Cartesian twist, adds a null-space
/// posture term, integrates joint targets and writes them to the joint drives.
///
/// Modes:
///   Hold      - keep the current joint targets.
///   Pose      - servo the tool to targetPosition / targetRotation.
///   Velocity  - follow commandedVelocity (world frame, m/s) while servoing
///               orientation to targetRotation. Intended for dynamical-system
///               (DemoStitching) control, where a DS supplies x_dot each step.
///
/// The gripper is driven by a single width command that is mirrored to both
/// fingers, which replaces the URDF mimic joints the importer drops.
/// </summary>
[DefaultExecutionOrder(1100)]
public class NeroCartesianController : MonoBehaviour
{
    public enum Mode { Hold, Pose, Velocity, Joint }

    [Header("Mode")]
    public Mode mode = Mode.Hold;

    [Header("Targets (world frame)")]
    public Vector3 targetPosition;
    public Quaternion targetRotation = Quaternion.identity;
    public Vector3 commandedVelocity;
    [Tooltip("Servo orientation to targetRotation in Velocity mode.")]
    public bool controlOrientation = true;
    [Tooltip("Weight of the orientation task in Velocity mode (1 = equal to position). Lower = position has priority.")]
    [Range(0f, 1f)] public float velocityModeRotationWeight = 0.3f;

    [Header("Tool frame")]
    [Tooltip("Distance from gripper_base along the approach axis to the grasp centre (m).")]
    public float toolOffset = 0.12f;
    public Transform toolFrame;

    [Header("Gains and limits")]
    public float positionGain = 3f;
    public float rotationGain = 3f;
    public float maxLinearSpeed = 0.25f;     // m/s
    public float maxAngularSpeed = 1.5f;     // rad/s
    public float maxJointSpeed = 1.5f;       // rad/s
    public float dampingLambda = 0.04f;      // DLS damping
    public float nullSpaceGain = 1.0f;
    [Tooltip("Margin kept from joint limits (rad).")]
    public float limitMargin = 0.03f;
    [Tooltip("Max allowed lead of commanded over measured joint position (rad).")]
    public float maxTrackingLead = 0.35f;

    [Header("Posture (rad) used by the null-space term")]
    public float[] restPosture = { 0f, 0.5f, 0f, 1.9f, 0f, 0.6f, 0f };
    public float limitAvoidGain = 0.5f;
    [Range(0.02f, 0.3f)] public float limitZoneFraction = 0.08f;

    [Header("Gripper")]
    [Range(0f, 0.1f)] public float gripperWidth = 0.09f;   // finger gap (m)
    public float fingerStiffness = 2000f;
    public float fingerDamping = 60f;
    public float fingerForceLimit = 20f;

    // ---- runtime state -----------------------------------------------------
    public const int N = 7;
    ArticulationBody[] joints;
    ArticulationBody fingerA, fingerB, gripperDummy, gripperBase;
    readonly float[] qCmd = new float[N];
    readonly float[] qMeas = new float[N];
    readonly float[] lower = new float[N];
    readonly float[] upper = new float[N];
    readonly double[,] J = new double[6, N];
    bool ready;

    public bool IsReady => ready;
    public Vector3 ToolPosition => toolFrame ? toolFrame.position : Vector3.zero;
    public Quaternion ToolRotation => toolFrame ? toolFrame.rotation : Quaternion.identity;
    public float PositionError => (targetPosition - ToolPosition).magnitude;
    public float RotationErrorDeg => Quaternion.Angle(targetRotation, ToolRotation);
    public float[] JointCommand => qCmd;
    public float[] JointMeasured => qMeas;

    /// <summary>Gripper pointing straight down; fingers open along world X by default.</summary>
    public static Quaternion DownRotation(float yawDeg = 0f) =>
        Quaternion.LookRotation(Vector3.down, Quaternion.Euler(0, yawDeg, 0) * Vector3.right);

    void Awake() => Init();

    public void Init()
    {
        if (ready) return;
        var bodies = GetComponentsInChildren<ArticulationBody>();
        var list = new System.Collections.Generic.List<ArticulationBody>();
        foreach (var b in bodies)
        {
            if (b.jointType == ArticulationJointType.RevoluteJoint) list.Add(b);
            else if (b.name == "gripper_link1") fingerA = b;
            else if (b.name == "gripper_link2") fingerB = b;
            else if (b.name == "gripper_link") gripperDummy = b;
            else if (b.name == "gripper_base") gripperBase = b;
        }
        if (list.Count != N || !gripperBase)
        {
            Debug.LogError($"NeroCartesianController: expected {N} revolute joints and gripper_base, found {list.Count}.");
            enabled = false;
            return;
        }
        joints = list.ToArray();

        if (!toolFrame)
        {
            var existing = gripperBase.transform.Find("Tool Frame");
            toolFrame = existing ? existing : new GameObject("Tool Frame").transform;
            toolFrame.SetParent(gripperBase.transform, false);
        }
        // forward = approach axis, up = finger opening axis.
        toolFrame.localPosition = Vector3.up * toolOffset;
        toolFrame.localRotation = Quaternion.LookRotation(Vector3.up, Vector3.right);

        for (int i = 0; i < N; i++)
        {
            var d = joints[i].xDrive;
            lower[i] = d.lowerLimit * Mathf.Deg2Rad + limitMargin;
            upper[i] = d.upperLimit * Mathf.Deg2Rad - limitMargin;
        }
        ready = true;
    }

    void Start()
    {
        if (!ready) return;
        ReadJoints();
        for (int i = 0; i < N; i++) qCmd[i] = qMeas[i];
        for (int i = 0; i < N; i++)
        {
            var d = joints[i].xDrive;
            d.driveType = ArticulationDriveType.Target;
            d.target = qCmd[i] * Mathf.Rad2Deg;
            d.targetVelocity = 0;
            joints[i].xDrive = d;
        }
        targetPosition = ToolPosition;
        targetRotation = ToolRotation;
        ApplyGripper();

        // Reference reach direction: horizontal base->tool direction of the rest
        // posture with joint 1 at zero. Used to aim joint 1 at the task target.
        baseAxis = (joints[0].transform.rotation * (joints[0].anchorRotation * Vector3.right)).normalized;
        basePoint = joints[0].transform.TransformPoint(joints[0].anchorPosition);
        var q0 = (float[])restPosture.Clone();
        q0[0] = 0f;
        ForwardFromMeasured(q0, out Vector3 p0, out _);
        azimuthRef = Vector3.ProjectOnPlane(p0 - basePoint, baseAxis).normalized;

        if (startInReadyPose)
        {
            TeleportJoints(PostureFor(readyAimPoint));
            mode = Mode.Hold;
        }
    }

    [Header("Start")]
    [Tooltip("Teleport to the rest posture (aimed at readyAimPoint) when Play starts.")]
    public bool startInReadyPose = true;
    public Vector3 readyAimPoint = new Vector3(-0.1f, 0.86f, 0f);

    [Header("Posture aiming")]
    [Tooltip("Aim joint 1 of the null-space posture at the task target's azimuth.")]
    public bool autoAzimuth = true;
    [Tooltip("Optional point to aim at in Velocity mode (e.g. the DS attractor). Zero = use commanded tool position.")]
    public Vector3 azimuthPoint;
    Vector3 baseAxis, basePoint, azimuthRef;

    /// <summary>Joint-1 angle (rad) that points the arm toward world point p.</summary>
    public float AzimuthFor(Vector3 p)
    {
        Vector3 d = Vector3.ProjectOnPlane(p - basePoint, baseAxis);
        if (d.sqrMagnitude < 1e-6f || azimuthRef.sqrMagnitude < 1e-6f) return restPosture[0];
        return Mathf.Clamp(Vector3.SignedAngle(azimuthRef, d, baseAxis) * Mathf.Deg2Rad, lower[0], upper[0]);
    }

    /// <summary>Rest posture with joint 1 aimed at p; good seed for teleporting or planning.</summary>
    public float[] PostureFor(Vector3 p)
    {
        var q = (float[])restPosture.Clone();
        q[0] = AzimuthFor(p);
        return q;
    }

    float PostureJoint1()
    {
        if (!autoAzimuth) return restPosture[0];
        if (mode == Mode.Pose) return AzimuthFor(targetPosition);
        return AzimuthFor(azimuthPoint != Vector3.zero ? azimuthPoint : cmdToolPos);
    }

    void ReadJoints()
    {
        for (int i = 0; i < N; i++) qMeas[i] = joints[i].jointPosition[0];
    }

    /// <summary>Instantly set the joint configuration (teleport), e.g. to start in a ready pose.</summary>
    [Header("Joint-space moves")]
    public float jointMoveSpeed = 0.8f;   // rad/s for the fastest joint
    readonly float[] jointTarget = new float[N];

    /// <summary>Start a synchronised joint-space move to q (Mode.Joint).</summary>
    public void MoveJointsTo(float[] q)
    {
        for (int i = 0; i < N; i++) jointTarget[i] = Mathf.Clamp(q[i], lower[i], upper[i]);
        mode = Mode.Joint;
    }

    /// <summary>True when the joint-space move has finished and the drives have caught up.</summary>
    public bool JointMoveDone(float tol = 0.01f)
    {
        for (int i = 0; i < N; i++)
            if (Mathf.Abs(jointTarget[i] - qCmd[i]) > 1e-4f || Mathf.Abs(qMeas[i] - qCmd[i]) > tol) return false;
        return true;
    }

    /// <summary>
    /// Kinematic IK (no physics): joint configuration that puts the tool at (p, rot),
    /// starting from 'seed' and staying close to it in the null space. Uses the exact
    /// forward kinematics of the measured chain. Returns false if it did not converge.
    /// </summary>
    public bool SolveIK(Vector3 p, Quaternion rot, float[] seed, float[] qOut, int iterations = 300)
    {
        var q = new float[N];
        for (int i = 0; i < N; i++) q[i] = Mathf.Clamp(seed[i], lower[i], upper[i]);
        var A = new double[6, 6];
        var e = new double[6];
        float posErr = 1f, rotErr = 180f;
        for (int it = 0; it < iterations; it++)
        {
            ForwardFromMeasured(q, out Vector3 tp, out Quaternion tr);
            Vector3 dp = p - tp;
            Quaternion qe = rot * Quaternion.Inverse(tr);
            qe.ToAngleAxis(out float ang, out Vector3 ax);
            if (ang > 180f) ang -= 360f;
            Vector3 dw = (ax.sqrMagnitude > 1e-9f ? ax.normalized : Vector3.zero) * ang * Mathf.Deg2Rad;
            posErr = dp.magnitude; rotErr = Mathf.Abs(ang);
            if (posErr < 5e-4f && rotErr < 0.3f) break;
            // Jacobian at q (axes/anchors were filled by ForwardFromMeasured).
            for (int i = 0; i < N; i++)
            {
                Vector3 lin = Vector3.Cross(cmdAxis[i], tp - cmdAnchor[i]);
                J[0, i] = lin.x; J[1, i] = lin.y; J[2, i] = lin.z;
                J[3, i] = cmdAxis[i].x; J[4, i] = cmdAxis[i].y; J[5, i] = cmdAxis[i].z;
            }
            double[] x = { dp.x, dp.y, dp.z, dw.x, dw.y, dw.z };
            var qn = new double[N];
            for (int i = 0; i < N; i++) qn[i] = 0.1 * (seed[i] - q[i]);
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++)
                {
                    double sum = 0;
                    for (int k = 0; k < N; k++) sum += J[r, k] * J[c, k];
                    A[r, c] = sum + (r == c ? 1e-4 : 0);
                }
                double jq = 0;
                for (int k = 0; k < N; k++) jq += J[r, k] * qn[k];
                e[r] = 0.5 * x[r] - jq;
            }
            var y = Solve(A, e, 6);
            for (int i = 0; i < N; i++)
            {
                double d = qn[i];
                for (int r = 0; r < 6; r++) d += J[r, i] * y[r];
                q[i] = Mathf.Clamp(q[i] + Mathf.Clamp((float)d, -0.2f, 0.2f), lower[i], upper[i]);
            }
        }
        for (int i = 0; i < N; i++) qOut[i] = q[i];
        return posErr < 0.003f && rotErr < 2f;
    }

    /// <summary>
    /// Gripper yaw (deg, as used by DownRotation) the arm has at p in its natural
    /// posture (base turned to p, wrist centred). Holding this yaw keeps the wrist
    /// joints away from their limits.
    /// </summary>
    public float NaturalYawDeg(Vector3 p)
    {
        ForwardFromMeasured(PostureFor(p), out _, out Quaternion r);
        Vector3 finger = Vector3.ProjectOnPlane(r * Vector3.up, Vector3.up);   // tool.up = finger opening axis
        if (finger.sqrMagnitude < 1e-6f) return 0f;
        return Vector3.SignedAngle(Vector3.right, finger, Vector3.up);
    }

    /// <summary>Gripper pointing down with the natural yaw at p; snapDeg > 0 rounds the yaw (e.g. 90 for cubes).</summary>
    public Quaternion NaturalDown(Vector3 p, float snapDeg = 0f)
    {
        float yaw = NaturalYawDeg(p);
        if (snapDeg > 0f) yaw = Mathf.Round(yaw / snapDeg) * snapDeg;
        return DownRotation(yaw);
    }

    [Tooltip("In Velocity mode, continuously set targetRotation to the natural gripper yaw at the current position.")]
    public bool followNaturalYaw = false;

    /// <summary>Joint configuration for tool pose (p, rot) in the natural posture branch (base turned to p).</summary>
    public bool SolveIKNatural(Vector3 p, Quaternion rot, float[] qOut) => SolveIK(p, rot, PostureFor(p), qOut);

    /// <summary>
    /// Go to (p, rot) in the natural posture: kinematic IK, synchronised joint-space
    /// move (the arm unwinds cleanly from any configuration), then a short Cartesian
    /// settle in Pose mode. Falls back to a Pose move if IK does not converge.
    /// </summary>
    public System.Collections.IEnumerator GoToNatural(Vector3 p, Quaternion rot, float posTol = 0.004f, float rotTol = 2f, float timeout = 10f)
    {
        var q = new float[N];
        bool ok = SolveIKNatural(p, rot, q);
        float t0 = Time.time;
        if (ok)
        {
            MoveJointsTo(q);
            while (!JointMoveDone() && Time.time - t0 < timeout) yield return new WaitForFixedUpdate();
        }
        else Debug.LogWarning($"NeroCartesianController: natural IK did not converge for {p:F3}; using Pose move");
        targetPosition = p;
        targetRotation = rot;
        mode = Mode.Pose;
        while ((PositionError > posTol || RotationErrorDeg > rotTol) && Time.time - t0 < timeout)
            yield return new WaitForFixedUpdate();
    }

    public void TeleportJoints(float[] q)
    {
        for (int i = 0; i < N; i++)
        {
            float v = Mathf.Clamp(q[i], lower[i], upper[i]);
            joints[i].jointPosition = new ArticulationReducedSpace(v);
            joints[i].jointVelocity = new ArticulationReducedSpace(0f);
            qCmd[i] = v;
            var d = joints[i].xDrive;
            d.target = v * Mathf.Rad2Deg;
            joints[i].xDrive = d;
        }
    }

    public void SetJointCommand(float[] q)
    {
        for (int i = 0; i < N; i++) qCmd[i] = Mathf.Clamp(q[i], lower[i], upper[i]);
    }

    void FixedUpdate()
    {
        if (!ready) return;
        ReadJoints();
        float dt = Time.fixedDeltaTime;

        if (mode == Mode.Velocity && followNaturalYaw)
            targetRotation = NaturalDown(cmdToolPos);   // uses last step's commanded tool position

        // Kinematic state of the *commanded* configuration (IK runs on this, the
        // joint drives track it). Using measured feedback here would put an
        // integrator around the drive lag and cause a limit cycle.
        ForwardFromMeasured(qCmd, out cmdToolPos, out cmdToolRot);

        if (mode == Mode.Joint)
        {
            // Synchronised joint-space move: all joints arrive at the same time.
            float maxRemain = 0f;
            for (int i = 0; i < N; i++) maxRemain = Mathf.Max(maxRemain, Mathf.Abs(jointTarget[i] - qCmd[i]));
            float step = jointMoveSpeed * dt;
            float frac = maxRemain <= step ? 1f : step / maxRemain;
            for (int i = 0; i < N; i++)
            {
                float next = qCmd[i] + (jointTarget[i] - qCmd[i]) * frac;
                next = Mathf.Clamp(next, qMeas[i] - maxTrackingLead, qMeas[i] + maxTrackingLead);
                qCmd[i] = Mathf.Clamp(next, lower[i], upper[i]);
            }
        }
        else if (mode != Mode.Hold)
        {
            // ---- task-space twist --------------------------------------------
            Vector3 v, w = Vector3.zero;
            if (mode == Mode.Pose)
                v = Vector3.ClampMagnitude(positionGain * (targetPosition - cmdToolPos), maxLinearSpeed);
            else
                v = Vector3.ClampMagnitude(commandedVelocity, maxLinearSpeed);

            bool useRot = mode == Mode.Pose || controlOrientation;
            if (useRot)
            {
                Quaternion qe = targetRotation * Quaternion.Inverse(cmdToolRot);
                qe.ToAngleAxis(out float angDeg, out Vector3 axis);
                if (angDeg > 180f) angDeg -= 360f;
                if (axis.sqrMagnitude > 1e-9f && Mathf.Abs(angDeg) > 1e-3f)
                    w = Vector3.ClampMagnitude(rotationGain * angDeg * Mathf.Deg2Rad * axis.normalized, maxAngularSpeed);
            }

            // ---- Jacobian (at commanded config) + limit-aware DLS --------------
            // A joint at its limit that the solution would push further is locked
            // (its Jacobian column removed) and the task is re-solved with the
            // remaining joints. Plain clipping afterwards would bend the
            // resulting tool motion in an arbitrary direction.
            // Orientation rows are weighted: < 1 lets position take priority when
            // the two conflict (e.g. a wrist joint at its limit).
            float rw = mode == Mode.Velocity ? Mathf.Clamp01(velocityModeRotationWeight) : 1f;
            double[] xd = { v.x, v.y, v.z, rw * w.x, rw * w.y, rw * w.z };
            int rows = useRot ? 6 : 3;
            for (int i = 0; i < N; i++) locked[i] = false;
            double[] qd = null;
            for (int iter = 0; iter < N; iter++)
            {
                ComputeJacobianAtCommand();
                for (int r = 3; r < 6; r++) for (int i = 0; i < N; i++) J[r, i] *= rw;
                for (int i = 0; i < N; i++)
                    if (locked[i]) for (int r = 0; r < 6; r++) J[r, i] = 0;
                qd = SolveDls(xd, rows);
                bool changed = false;
                for (int i = 0; i < N; i++)
                {
                    if (locked[i]) continue;
                    float next = qCmd[i] + (float)qd[i] * dt;
                    if ((next > upper[i] && qd[i] > 0) || (next < lower[i] && qd[i] < 0)) { locked[i] = true; changed = true; }
                }
                if (!changed) break;
            }
            LockedJointMask = 0;
            for (int i = 0; i < N; i++) if (locked[i]) { qd[i] = 0; LockedJointMask |= 1 << i; }

            // ---- integrate, respecting speed / limits / tracking lead ---------
            float scale = 1f;
            for (int i = 0; i < N; i++)
                scale = Mathf.Min(scale, maxJointSpeed / Mathf.Max(1e-6f, Mathf.Abs((float)qd[i])));
            for (int i = 0; i < N; i++)
            {
                float next = qCmd[i] + (float)qd[i] * scale * dt;
                next = Mathf.Clamp(next, qMeas[i] - maxTrackingLead, qMeas[i] + maxTrackingLead);
                qCmd[i] = Mathf.Clamp(next, lower[i], upper[i]);
            }
        }

        for (int i = 0; i < N; i++)
        {
            var d = joints[i].xDrive;
            d.target = qCmd[i] * Mathf.Rad2Deg;
            joints[i].xDrive = d;
        }
        ApplyGripper();
    }

    readonly bool[] locked = new bool[N];
    /// <summary>Bit i set = joint i+1 was locked at a limit in the last IK step.</summary>
    public int LockedJointMask { get; private set; }

    // Joint axes/anchors of the commanded configuration (filled by ForwardFromMeasured).
    readonly Vector3[] cmdAxis = new Vector3[N];
    readonly Vector3[] cmdAnchor = new Vector3[N];
    Vector3 cmdToolPos;
    Quaternion cmdToolRot = Quaternion.identity;

    /// <summary>Tool pose of the commanded configuration (what the arm is converging to).</summary>
    public Vector3 CommandedToolPosition => cmdToolPos;
    public Quaternion CommandedToolRotation => cmdToolRot;

    /// <summary>
    /// Exact forward kinematics for configuration q, computed from the measured
    /// chain: joint i's measured axis is carried by the rotations of joints
    /// 0..i-1, then everything distal is rotated by (q_i - qMeas_i).
    /// </summary>
    public void ForwardFromMeasured(float[] q, out Vector3 toolPos, out Quaternion toolRot)
    {
        Quaternion G = Quaternion.identity;   // accumulated rotation
        Vector3 Gt = Vector3.zero;            // accumulated translation: x' = G x + Gt
        for (int i = 0; i < N; i++)
        {
            var b = joints[i];
            Vector3 axisM = (b.transform.rotation * (b.anchorRotation * Vector3.right)).normalized;
            Vector3 anchorM = b.transform.TransformPoint(b.anchorPosition);
            Vector3 axis = G * axisM;
            Vector3 anchor = G * anchorM + Gt;
            cmdAxis[i] = axis;
            cmdAnchor[i] = anchor;
            Quaternion R = Quaternion.AngleAxis((q[i] - qMeas[i]) * Mathf.Rad2Deg, axis);
            // x'' = R (x' - anchor) + anchor
            G = R * G;
            Gt = R * (Gt - anchor) + anchor;
        }
        toolPos = G * ToolPosition + Gt;
        toolRot = G * ToolRotation;
    }

    /// <summary>Geometric Jacobian at the commanded configuration. Rows: vx vy vz wx wy wz (world).</summary>
    public void ComputeJacobianAtCommand()
    {
        for (int i = 0; i < N; i++)
        {
            Vector3 axis = cmdAxis[i];
            Vector3 lin = Vector3.Cross(axis, cmdToolPos - cmdAnchor[i]);
            J[0, i] = lin.x; J[1, i] = lin.y; J[2, i] = lin.z;
            J[3, i] = axis.x; J[4, i] = axis.y; J[5, i] = axis.z;
        }
    }

    /// <summary>qd = J+ xd + (I - J+ J) qd_null, with J+ = J^T (J J^T + l^2 I)^-1.</summary>
    double[] SolveDls(double[] xd, int rows)
    {
        // A = J J^T + l^2 I  (rows x rows)
        var A = new double[rows, rows];
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < rows; c++)
            {
                double s = 0;
                for (int k = 0; k < N; k++) s += J[r, k] * J[c, k];
                A[r, c] = s + (r == c ? dampingLambda * dampingLambda : 0);
            }

        // Null-space posture velocity.
        var qn = new double[N];
        for (int i = 0; i < N; i++)
        {
            float rest = i == 0 ? PostureJoint1() : (restPosture != null && i < restPosture.Length ? restPosture[i] : 0f);
            qn[i] = nullSpaceGain * (rest - qCmd[i]);

            // Joint-limit avoidance: push away when within limitZoneFraction of the range of a limit.
            float range = upper[i] - lower[i];
            float zone = limitZoneFraction * range;
            float dLow = qCmd[i] - lower[i], dHigh = upper[i] - qCmd[i];
            if (dLow < zone) qn[i] += limitAvoidGain * (zone - dLow) / zone;
            if (dHigh < zone) qn[i] -= limitAvoidGain * (zone - dHigh) / zone;
            if (locked[i]) qn[i] = 0;
        }

        // Task residual after the null-space motion: xd - J qn.
        var e = new double[rows];
        for (int r = 0; r < rows; r++)
        {
            double s = 0;
            for (int k = 0; k < N; k++) s += J[r, k] * qn[k];
            e[r] = xd[r] - s;
        }
        var y = Solve(A, e, rows);

        // qd = qn + J^T y  ==  J+ xd + (I - J+ J) qn
        var qd = new double[N];
        for (int i = 0; i < N; i++)
        {
            double s = qn[i];
            for (int r = 0; r < rows; r++) s += J[r, i] * y[r];
            qd[i] = s;
        }
        return qd;
    }

    static double[] Solve(double[,] A, double[] b, int n)
    {
        var M = (double[,])A.Clone();
        var x = (double[])b.Clone();
        for (int c = 0; c < n; c++)
        {
            int piv = c;
            for (int r = c + 1; r < n; r++) if (System.Math.Abs(M[r, c]) > System.Math.Abs(M[piv, c])) piv = r;
            if (piv != c)
            {
                for (int k = 0; k < n; k++) { var t = M[c, k]; M[c, k] = M[piv, k]; M[piv, k] = t; }
                var tb = x[c]; x[c] = x[piv]; x[piv] = tb;
            }
            double d = M[c, c];
            if (System.Math.Abs(d) < 1e-12) d = 1e-12;
            for (int r = c + 1; r < n; r++)
            {
                double f = M[r, c] / d;
                if (f == 0) continue;
                for (int k = c; k < n; k++) M[r, k] -= f * M[c, k];
                x[r] -= f * x[c];
            }
        }
        for (int r = n - 1; r >= 0; r--)
        {
            double s = x[r];
            for (int k = r + 1; k < n; k++) s -= M[r, k] * x[k];
            double d = System.Math.Abs(M[r, r]) < 1e-12 ? 1e-12 : M[r, r];
            x[r] = s / d;
        }
        return x;
    }

    void ApplyGripper()
    {
        float half = Mathf.Clamp(gripperWidth, 0f, 0.1f) * 0.5f;
        SetFinger(fingerA, half);     // limits 0 .. 0.05
        SetFinger(fingerB, -half);    // limits -0.05 .. 0
        if (gripperDummy) SetFinger(gripperDummy, half * 2f);
    }

    void SetFinger(ArticulationBody f, float target)
    {
        if (!f) return;
        var d = f.xDrive;
        d.driveType = ArticulationDriveType.Force;
        d.stiffness = fingerStiffness;
        d.damping = fingerDamping;
        d.forceLimit = fingerForceLimit;
        d.target = target;
        f.xDrive = d;
    }

    void OnDrawGizmos()
    {
        if (!toolFrame) return;
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(toolFrame.position, 0.01f);
        Gizmos.DrawLine(toolFrame.position, toolFrame.position + toolFrame.forward * 0.05f);
        if (mode == Mode.Pose)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(targetPosition, 0.012f);
        }
    }
}
