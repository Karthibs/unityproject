using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// C# runtime of DemoStitching's chained dynamical system (ChainedSegmentedDS,
/// ds_method = "chain", transition_trigger_method = "distance_ratio").
///
/// Each segment i is an LPV-DS:  v_i(x) = sum_k gamma_k(x) A_k (x - x_att_i),
/// gamma = softmax_k( log prior_k + log N(x; mu_k, Sigma_k) ).
/// The chain follows segment idx until  |x-n1| / |x-n2| >= edgeRatio[idx],
/// then blends linearly in time into segment idx+1 over times[idx] seconds.
///
/// All of this happens in the DS frame used for learning:
///   p_ds = scale * perm(p_unity)      (perm = (x, z, y))
///   v_unity = perm^-1(v_ds) / scale   (m/s)
/// Mirrors src/stitching/chaining.py (ChainedDS._transition_velocity etc.).
/// </summary>
public class NeroChainedDS
{
    // ---- serialized export (DemoData/chained_ds_nero_unity.json) ----------
    [Serializable] public class Segment
    {
        public double[] xAtt; public int K;
        public double[] priors, mus, sigmas, A;
    }

    [Serializable] public class Task
    {
        public string name, kind, startObject, endObject;
        public double[] initial, attractor;
        public int[] pathDemos;
        public Segment[] segments;
        public double[] ratioStartNodes, ratioNodes, edgeRatios, times;
        public double[] referenceRollout;
        public double referenceFinalErrorM;
    }

    [Serializable] public class Export
    {
        public string format;
        public double scale;
        public int[] axisPerm;
        public double dtReference;
        public double velocityMax;
        public Task[] tasks;
    }

    public static Export Load(string path)
    {
        var e = JsonUtility.FromJson<Export>(File.ReadAllText(path));
        if (e == null || e.tasks == null) throw new Exception("Could not parse chained DS export: " + path);
        return e;
    }

    // ---- precomputed per-segment Gaussian data ----------------------------
    class Gauss
    {
        public double[] mu = new double[3];
        public double[,] inv = new double[3, 3];
        public double logNorm;   // log prior - 0.5*log det(2 pi Sigma)
        public double[,] A = new double[3, 3];
    }

    class Seg
    {
        public double[] xAtt = new double[3];
        public Gauss[] g;
    }

    public readonly Task task;
    public readonly double scale;
    readonly int[] perm;
    readonly double vmax;
    readonly Seg[] segs;
    int idx;
    int transitionFrom = -1;
    double transitionT0;

    public int SegmentIndex => idx;
    public int SegmentCount => segs.Length;
    public bool InTransition => transitionFrom >= 0;

    public NeroChainedDS(Export export, Task task)
    {
        this.task = task;
        scale = export.scale;
        perm = export.axisPerm != null && export.axisPerm.Length == 3 ? export.axisPerm : new[] { 0, 2, 1 };
        vmax = export.velocityMax;
        segs = new Seg[task.segments.Length];
        for (int s = 0; s < segs.Length; s++)
        {
            var src = task.segments[s];
            var seg = new Seg { g = new Gauss[src.K] };
            for (int i = 0; i < 3; i++) seg.xAtt[i] = src.xAtt[i];
            for (int k = 0; k < src.K; k++)
            {
                var g = new Gauss();
                var S = new double[3, 3];
                for (int i = 0; i < 3; i++)
                {
                    g.mu[i] = src.mus[3 * k + i];
                    for (int j = 0; j < 3; j++)
                    {
                        S[i, j] = src.sigmas[9 * k + 3 * i + j];
                        g.A[i, j] = src.A[9 * k + 3 * i + j];
                    }
                }
                double det = Inverse3(S, g.inv);
                g.logNorm = Math.Log(Math.Max(src.priors[k], 1e-300)) - 0.5 * (3 * Math.Log(2 * Math.PI) + Math.Log(Math.Max(det, 1e-300)));
                seg.g[k] = g;
            }
            segs[s] = seg;
        }
        Reset();
    }

    public void Reset()
    {
        idx = 0;
        transitionFrom = -1;
        transitionT0 = 0;
    }

    // ---- frame conversion --------------------------------------------------
    public double[] ToDs(Vector3 p)
    {
        double[] u = { p.x, p.y, p.z };
        return new[] { scale * u[perm[0]], scale * u[perm[1]], scale * u[perm[2]] };
    }

    public Vector3 FromDs(double[] q)
    {
        var u = new double[3];
        for (int i = 0; i < 3; i++) u[perm[i]] = q[i] / scale;
        return new Vector3((float)u[0], (float)u[1], (float)u[2]);
    }

    public Vector3 AttractorUnity => FromDs(task.attractor);
    public Vector3 InitialUnity => FromDs(task.initial);

    // ---- one control step (stateful, like ChainedSegmentedDS.step_once) ----
    /// <summary>Velocity in the DS frame for state x (DS frame) at time t (s).</summary>
    public double[] Step(double[] x, double t)
    {
        // Start a transition when the state trigger fires.
        if (transitionFrom < 0 && idx < segs.Length - 1 && Trigger(idx, x))
        {
            transitionFrom = idx;
            transitionT0 = t;
        }
        double[] v;
        if (transitionFrom < 0)
        {
            v = SegmentVelocity(idx, x);
        }
        else
        {
            int i = transitionFrom;
            double T = i < task.times.Length ? task.times[i] : 0.0;
            double alpha = T <= 1e-12 ? 1.0 : (t - transitionT0) / T;
            alpha = Math.Max(0.0, Math.Min(1.0, alpha));
            var a = SegmentVelocity(i, x);
            var b = SegmentVelocity(i + 1, x);
            v = new double[3];
            for (int d = 0; d < 3; d++) v[d] = (1 - alpha) * a[d] + alpha * b[d];
            if (alpha >= 1.0)
            {
                idx = Math.Min(i + 1, segs.Length - 1);
                transitionFrom = -1;
            }
        }
        return Clip(v);
    }

    /// <summary>Convenience: Unity world position in, Unity world velocity (m/s) out.</summary>
    public Vector3 StepUnity(Vector3 p, double t)
    {
        var v = Step(ToDs(p), t);
        var u = new double[3];
        for (int i = 0; i < 3; i++) u[perm[i]] = v[i] / scale;
        return new Vector3((float)u[0], (float)u[1], (float)u[2]);
    }

    bool Trigger(int i, double[] x)
    {
        if (i >= task.edgeRatios.Length) return false;
        double thr = task.edgeRatios[i];
        if (thr < 0 || double.IsNaN(thr) || double.IsInfinity(thr)) return false;
        double d1 = Dist(x, task.ratioStartNodes, 3 * i);
        double d2 = Dist(x, task.ratioNodes, 3 * i);
        return d1 / Math.Max(d2, 1e-12) >= thr;
    }

    double[] SegmentVelocity(int s, double[] x)
    {
        var seg = segs[s];
        int K = seg.g.Length;
        var logp = new double[K];
        double maxl = double.NegativeInfinity;
        for (int k = 0; k < K; k++)
        {
            var g = seg.g[k];
            double d0 = x[0] - g.mu[0], d1 = x[1] - g.mu[1], d2 = x[2] - g.mu[2];
            double m = d0 * (g.inv[0, 0] * d0 + g.inv[0, 1] * d1 + g.inv[0, 2] * d2)
                     + d1 * (g.inv[1, 0] * d0 + g.inv[1, 1] * d1 + g.inv[1, 2] * d2)
                     + d2 * (g.inv[2, 0] * d0 + g.inv[2, 1] * d1 + g.inv[2, 2] * d2);
            logp[k] = g.logNorm - 0.5 * m;
            if (logp[k] > maxl) maxl = logp[k];
        }
        double sum = 0;
        for (int k = 0; k < K; k++) { logp[k] = Math.Exp(logp[k] - maxl); sum += logp[k]; }
        double e0 = x[0] - seg.xAtt[0], e1 = x[1] - seg.xAtt[1], e2 = x[2] - seg.xAtt[2];
        var v = new double[3];
        for (int k = 0; k < K; k++)
        {
            double gam = logp[k] / sum;
            var A = seg.g[k].A;
            v[0] += gam * (A[0, 0] * e0 + A[0, 1] * e1 + A[0, 2] * e2);
            v[1] += gam * (A[1, 0] * e0 + A[1, 1] * e1 + A[1, 2] * e2);
            v[2] += gam * (A[2, 0] * e0 + A[2, 1] * e1 + A[2, 2] * e2);
        }
        return v;
    }

    double[] Clip(double[] v)
    {
        if (vmax <= 0) return v;
        double n = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n > vmax && n > 1e-12) for (int i = 0; i < 3; i++) v[i] *= vmax / n;
        return v;
    }

    static double Dist(double[] x, double[] arr, int o)
    {
        double a = x[0] - arr[o], b = x[1] - arr[o + 1], c = x[2] - arr[o + 2];
        return Math.Sqrt(a * a + b * b + c * c);
    }

    static double Inverse3(double[,] m, double[,] inv)
    {
        double a = m[0, 0], b = m[0, 1], c = m[0, 2], d = m[1, 0], e = m[1, 1], f = m[1, 2], g = m[2, 0], h = m[2, 1], i = m[2, 2];
        double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
        double det = a * A + b * B + c * C;
        double s = Math.Abs(det) < 1e-300 ? 0 : 1.0 / det;
        inv[0, 0] = A * s; inv[0, 1] = -(b * i - c * h) * s; inv[0, 2] = (b * f - c * e) * s;
        inv[1, 0] = B * s; inv[1, 1] = (a * i - c * g) * s; inv[1, 2] = -(a * f - c * d) * s;
        inv[2, 0] = C * s; inv[2, 1] = -(a * h - b * g) * s; inv[2, 2] = (a * e - b * d) * s;
        return det;
    }

    // ---- kinematic rollout (verification against the Python reference) ----
    /// <summary>Euler rollout in the DS frame, exactly like ChainedDS.sim (dt, tol 0.01, max steps).</summary>
    public List<double[]> SimulateKinematic(double[] x0, double dt = 0.01, int maxSteps = 20000, double tol = 0.01)
    {
        Reset();
        var x = (double[])x0.Clone();
        var traj = new List<double[]> { (double[])x.Clone() };
        double t = 0;
        for (int n = 0; n < maxSteps; n++)
        {
            double dx = x[0] - task.attractor[0], dy = x[1] - task.attractor[1], dz = x[2] - task.attractor[2];
            if (Math.Sqrt(dx * dx + dy * dy + dz * dz) < tol) break;
            var v = Step(x, t);
            for (int i = 0; i < 3; i++) x[i] += dt * v[i];
            t += dt;
            traj.Add((double[])x.Clone());
        }
        Reset();
        return traj;
    }
}
