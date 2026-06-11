/**
 * @file path.hpp
 * @brief Path geometry, smoothing and speed-profile module for the path follower.
 *
 * Holds the `Path` class: a piecewise-linear path in the ENU frame with optional
 * cubic B-spline smoothing, analytic curvature, cross-track/heading error, and a
 * curvature-aware speed-profile builder. Also provides the build-time CSV loader.
 *
 * This is the "build + query" half of the old monolithic lateral_mpc_node.cpp:
 *   - build-time (run once per path load): loadCSV / smoothSpline /
 *     buildSpeedProfile
 *   - query-time (every control tick): findClosest / position / heading /
 *     curvature / crossTrackError / headingError / vref
 *
 * The class is ROS-agnostic (no rclcpp); the node orchestrates loading and logs.
 */
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// ============================================================
// Vehicle constants (shared by Path and the controller node)
// ============================================================
static constexpr double WHEELBASE      = 2.70;    // Kia Niro [m]
static constexpr double STEERING_RATIO = 13.3;    // sw-deg per road-wheel deg (measured; 460°/34.6°)

// ============================================================
// Piecewise-linear path in ENU frame
// ============================================================
class Path
{
public:
    Path() = default;

    void clear()
    {
        wpts_.clear();
        s_.clear();
        v_ref_.clear();
        knots_.clear();
        cx_.resize(0);
        cy_.resize(0);
        spline_L_ = 0.0;
    }

    bool isEmpty() const { return wpts_.empty(); }

    void addWaypoint(double x, double y)
    {
        if (wpts_.empty()) {
            s_.push_back(0.0);
        } else {
            double dx = x - wpts_.back().first;
            double dy = y - wpts_.back().second;
            s_.push_back(s_.back() + std::hypot(dx, dy));
        }
        wpts_.emplace_back(x, y);
    }

    // ── Build-time generators ─────────────────────────────────────────────
    // Load waypoints from a CSV of "x,y" (or "x y") rows; '#' and header rows
    // starting with x/X are skipped. Returns the waypoint count, or -1 if the
    // file could not be opened. Clears any existing path first.
    int loadCSV(const std::string& filename)
    {
        clear();
        std::ifstream f(filename);
        if (!f.is_open()) return -1;

        int count = 0;
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            if (line[0] == 'x' || line[0] == 'X') continue;

            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream ss(line);
            double x = 0.0, y = 0.0;
            if (!(ss >> x >> y)) continue;
            addWaypoint(x, y);
            ++count;
        }
        return count;
    }

    double totalLength() const
    {
        return s_.empty() ? 0.0 : s_.back();
    }

    size_t waypointCount() const
    {
        return wpts_.size();
    }

    /** Find arc-length of closest point to (qx, qy). Updates hint in place. */
    double findClosest(double qx, double qy, size_t & hint) const
    {
        if (wpts_.size() < 2) return 0.0;

        double best_sq = std::numeric_limits<double>::max();
        double best_s  = s_[hint];
        size_t end_idx = std::min(wpts_.size() - 1, hint + 300);

        for (size_t i = hint; i < end_idx; ++i) {
            double ax = wpts_[i].first,   ay = wpts_[i].second;
            double bx = wpts_[i+1].first, by = wpts_[i+1].second;
            double dx = bx - ax,          dy = by - ay;
            double seg2 = dx*dx + dy*dy;
            if (seg2 < 1e-12) continue;

            double t = ((qx-ax)*dx + (qy-ay)*dy) / seg2;
            t = std::clamp(t, 0.0, 1.0);

            double px = ax + t*dx, py = ay + t*dy;
            double d2 = (qx-px)*(qx-px) + (qy-py)*(qy-py);

            if (d2 < best_sq) {
                best_sq = d2;
                best_s  = s_[i] + t * std::sqrt(seg2);
                hint    = i;
            }
        }
        return best_s;
    }

    bool hasSpline() const { return !knots_.empty() && cx_.size() > 0; }

    /** Position at arc-length s. */
    std::pair<double,double> position(double s) const
    {
        if (hasSpline()) {
            Eigen::RowVectorXd B, Bp, Bpp;
            bsplineBasisAndDerivs(splineParam(s), B, Bp, Bpp);
            return {B.dot(cx_), B.dot(cy_)};
        }
        return interp(s);
    }

    /** Path tangent heading [rad] (ENU: East=0, CCW positive). */
    double heading(double s) const
    {
        if (hasSpline()) {
            Eigen::RowVectorXd B, Bp, Bpp;
            bsplineBasisAndDerivs(splineParam(s), B, Bp, Bpp);
            return std::atan2(Bp.dot(cy_), Bp.dot(cx_));
        }
        constexpr double ds = 0.2;
        auto [x0, y0] = interp(std::max(0.0, s - ds));
        auto [x1, y1] = interp(std::min(totalLength(), s + ds));
        return std::atan2(y1 - y0, x1 - x0);
    }

    /**
     * Signed cross-track error at (qx, qy) for path point at arc-length s.
     * Positive = vehicle is to the RIGHT of the path.
     */
    double crossTrackError(double qx, double qy, double s) const
    {
        auto [px, py] = interp(s);
        double h  = heading(s);
        double nx =  std::sin(h);
        double ny = -std::cos(h);
        return (qx - px)*nx + (qy - py)*ny;
    }

    /**
     * Signed heading error: path_heading – car_heading, wrapped to [-π, π].
     * Positive = car pointing right of path.
     */
    double headingError(double s, double car_heading) const
    {
        double err = heading(s) - car_heading;
        while (err >  M_PI) err -= 2.0*M_PI;
        while (err < -M_PI) err += 2.0*M_PI;
        return err;
    }

    /** Path curvature at arc-length s [rad/m]. */
    double curvature(double s) const
    {
        if (hasSpline()) {
            // Analytic κ = (x'·y'' - y'·x'') / (x'² + y'²)^{3/2}.
            // Parameterisation-invariant — works even though the spline parameter
            // is the original (pre-smoothing) arc length, not the chord length.
            Eigen::RowVectorXd B, Bp, Bpp;
            bsplineBasisAndDerivs(splineParam(s), B, Bp, Bpp);
            const double dx  = Bp.dot(cx_),  dy  = Bp.dot(cy_);
            const double ddx = Bpp.dot(cx_), ddy = Bpp.dot(cy_);
            const double speed2 = dx*dx + dy*dy;
            if (speed2 < 1e-12) return 0.0;
            return (dx*ddy - dy*ddx) / std::pow(speed2, 1.5);
        }
        // Fallback: piecewise-linear path → noisy finite-difference κ.
        constexpr double ds = 0.5;
        double h0 = heading(std::max(0.0, s - ds));
        double h1 = heading(std::min(totalLength(), s + ds));
        double dh = h1 - h0;
        while (dh >  M_PI) dh -= 2.0 * M_PI;
        while (dh < -M_PI) dh += 2.0 * M_PI;
        return dh / (2.0 * ds);
    }

    bool hasSpeedProfile() const { return v_ref_.size() == s_.size() && !v_ref_.empty(); }

    double vref(double s) const
    {
        if (v_ref_.empty()) return 0.0;
        if (s <= 0.0)       return v_ref_.front();
        if (s >= s_.back()) return v_ref_.back();
        auto it  = std::lower_bound(s_.begin(), s_.end(), s);
        size_t i = std::distance(s_.begin(), it);
        if (i == 0) return v_ref_[0];
        i = std::min(i, v_ref_.size() - 1);
        double t = (s_[i] - s_[i-1] > 1e-12) ? (s - s_[i-1]) / (s_[i] - s_[i-1]) : 0.0;
        return v_ref_[i-1] + std::clamp(t, 0.0, 1.0) * (v_ref_[i] - v_ref_[i-1]);
    }

    // Build a per-waypoint speed reference by inverting the sched_fo2 K_ss table.
    // sched_kss is monotone-decreasing: high gain at low speed, low gain at high speed.
    // For each waypoint, finds the max speed where the steering model can still achieve
    // the required angle, then applies backward/forward kinematic smoothing passes.
    void buildSpeedProfile(
        const std::vector<double>& sched_v_kmh,
        const std::vector<double>& sched_kss,
        double torque_limit,
        double desired_speed_mps,
        double decel_mps2,
        double accel_mps2,
        double margin_factor,
        double v_min_mps,
        double slew_budget_radps,
        double speed_lookahead_s = 0.0)
    {
        const int n = static_cast<int>(wpts_.size());
        v_ref_.resize(n);

        // Pass 1: raw steering-model speed limit per waypoint
        for (int i = 0; i < n; i++) {
            double kappa     = curvature(s_[i]);
            double delta_req = std::abs(kappa) * WHEELBASE;
            // Required K_ss so delta_max(v) >= delta_req / margin_factor
            double kss_req   = delta_req * STEERING_RATIO * (180.0 / M_PI)
                               / (torque_limit * margin_factor);
            double v_limit;
            if (sched_kss.empty() || kss_req <= 0.0) {
                v_limit = desired_speed_mps;
            } else if (kss_req <= sched_kss.back()) {
                // Even at the highest scheduled speed the model can handle this curvature
                v_limit = sched_v_kmh.back() / 3.6;
            } else if (kss_req >= sched_kss.front()) {
                // Even the lowest scheduled speed cannot handle it — use floor
                v_limit = v_min_mps;
            } else {
                // Walk table (monotone-decreasing K_ss) to find the crossing segment
                v_limit = v_min_mps;
                for (size_t j = 0; j + 1 < sched_kss.size(); j++) {
                    if (sched_kss[j] >= kss_req && kss_req > sched_kss[j + 1]) {
                        double t = (kss_req - sched_kss[j])
                                   / (sched_kss[j + 1] - sched_kss[j]);
                        v_limit = (sched_v_kmh[j]
                                   + t * (sched_v_kmh[j + 1] - sched_v_kmh[j])) / 3.6;
                        break;
                    }
                }
            }

            // Curvature-RATE (slew) limit — the S-curve fix. Following the path
            // demands a front-wheel angle rate dδ/dt = v·dδ/ds ≈ v·L·|dκ/ds|
            // (small-angle δ≈κL). At an S-curve inflection |κ|≈0 (so the model
            // limit above sees a "straight" and allows full speed) yet |dκ/ds| is
            // large — exactly where the rate-limited actuator must reverse fastest.
            // Cap the demanded slew at slew_budget_radps to give it time/distance:
            //   v ≤ slew_budget / (L·|dκ/ds|).
            //
            // IMPORTANT: use a ±3 m distance-based window to compute dκ/ds, not
            // index ±1. Index ±1 is only ~0.2 m on a dense path and measures noise
            // (max dkappa/ds 600+ rad/m²), which slew-caps 87 % of waypoints to
            // v_min regardless of actual path curvature. A 3 m window reduces that
            // to ~17 % and gives physically meaningful S-curve slowdowns only.
            if (slew_budget_radps > 1e-9 && n >= 3) {
                constexpr double SLEW_HALF_WINDOW_M = 3.0;
                // Find indices ±3 m from current arc-length
                const double s_lo = s_[i] - SLEW_HALF_WINDOW_M;
                const double s_hi = s_[i] + SLEW_HALF_WINDOW_M;
                // Lower bound: walk backward
                int im = i;
                while (im > 0     && s_[im - 1] >= s_lo) --im;
                // Upper bound: walk forward
                int ip = i;
                while (ip < n - 1 && s_[ip + 1] <= s_hi) ++ip;
                const double dss = s_[ip] - s_[im];
                if (dss > 1e-6) {
                    const double dkappa_ds =
                        std::abs(curvature(s_[ip]) - curvature(s_[im])) / dss;
                    const double denom = WHEELBASE * dkappa_ds;
                    if (denom > 1e-9)
                        v_limit = std::min(v_limit, slew_budget_radps / denom);
                }
            }

            v_ref_[i] = std::clamp(v_limit, v_min_mps, desired_speed_mps);
        }

        // Pass 2: backward pass — braking constraint
        for (int i = n - 2; i >= 0; i--) {
            double ds      = s_[i + 1] - s_[i];
            double v_brake = std::sqrt(v_ref_[i + 1] * v_ref_[i + 1] + 2.0 * decel_mps2 * ds);
            v_ref_[i] = std::min(v_ref_[i], v_brake);
        }

        // Pass 3: forward pass — acceleration constraint
        for (int i = 1; i < n; i++) {
            double ds      = s_[i] - s_[i - 1];
            double v_accel = std::sqrt(v_ref_[i - 1] * v_ref_[i - 1] + 2.0 * accel_mps2 * ds);
            v_ref_[i] = std::min(v_ref_[i], v_accel);
        }

        // Pass 4: time-based forward lookahead — for each waypoint, scan ahead by
        // v_ref[i] * lookahead_s metres (pure time window: same seconds regardless of
        // speed) and pull v_ref[i] down to the minimum over that window.
        // A re-run of the backward pass (Pass 5) then ensures decel feasibility.
        if (speed_lookahead_s > 1e-6) {
            for (int i = 0; i < n; i++) {
                double lookahead_m = v_ref_[i] * speed_lookahead_s;
                double s_end = s_[i] + lookahead_m;
                for (int j = i + 1; j < n && s_[j] <= s_end; j++)
                    v_ref_[i] = std::min(v_ref_[i], v_ref_[j]);
            }

            // Pass 5: backward pass again to restore braking feasibility after lookahead
            for (int i = n - 2; i >= 0; i--) {
                double ds      = s_[i + 1] - s_[i];
                double v_brake = std::sqrt(v_ref_[i + 1] * v_ref_[i + 1] + 2.0 * decel_mps2 * ds);
                v_ref_[i] = std::min(v_ref_[i], v_brake);
            }
        }

        for (auto& vv : v_ref_) vv = std::max(vv, v_min_mps);
    }

    // For diagnostics only — returns (max_abs_kappa, min_vref, max_vref) over all waypoints.
    std::tuple<double,double,double> speedProfileStats() const
    {
        if (v_ref_.empty() || s_.empty()) return {0.0, 0.0, 0.0};
        double kappa_max = 0.0;
        for (const double s : s_)
            kappa_max = std::max(kappa_max, std::abs(curvature(s)));
        double v_min = *std::min_element(v_ref_.begin(), v_ref_.end());
        double v_max = *std::max_element(v_ref_.begin(), v_ref_.end());
        return {kappa_max, v_min, v_max};
    }

    const std::vector<std::pair<double,double>>& waypoints() const { return wpts_; }

    /**
     * Cubic B-spline least-squares path smoother.
     *
     * Fits independent cubic B-splines x(s) and y(s) with interior knots
     * placed every knot_spacing_m metres along the arc-length.  Solves the
     * normal equations via Cholesky decomposition (Eigen::LLT), then
     * resamples the spline at all original waypoint arc-lengths.
     *
     * This matches the Python LSQUnivariateSpline(k=3) implementation used
     * for offline analysis, with the same 5 m default knot spacing.
     */
    void smoothSpline(double knot_spacing_m)
    {
        const int n = static_cast<int>(wpts_.size());
        if (n < 4) return;

        const double L = s_.back();
        if (L < knot_spacing_m) return;

        // --- Build interior knot vector ---
        std::vector<double> interior_knots;
        for (double t = knot_spacing_m; t < L - knot_spacing_m * 0.5; t += knot_spacing_m)
            interior_knots.push_back(t);

        // Full knot vector: degree k=3 requires k+1 repeated knots at each end.
        knots_.clear();
        for (int i = 0; i <= K_; i++) knots_.push_back(0.0);
        for (double t : interior_knots) knots_.push_back(t);
        for (int i = 0; i <= K_; i++) knots_.push_back(L);
        spline_L_ = L;

        const int nc = static_cast<int>(knots_.size()) - K_ - 1;

        // --- Build least-squares system A (n x nc) ---
        Eigen::MatrixXd A(n, nc);
        for (int i = 0; i < n; i++)
            A.row(i) = bsplineBasis(s_[i]);

        Eigen::VectorXd fx(n), fy(n);
        for (int i = 0; i < n; i++) {
            fx(i) = wpts_[i].first;
            fy(i) = wpts_[i].second;
        }

        // Normal equations: (A^T A) c = A^T f
        Eigen::MatrixXd ATA = A.transpose() * A;
        Eigen::LLT<Eigen::MatrixXd> llt(ATA);
        if (llt.info() != Eigen::Success) {
            knots_.clear();              // singular — disable spline path
            spline_L_ = 0.0;
            return;
        }

        cx_ = llt.solve(A.transpose() * fx);
        cy_ = llt.solve(A.transpose() * fy);

        // --- Resample spline at original arc-lengths ---
        for (int i = 0; i < n; i++) {
            Eigen::RowVectorXd B = bsplineBasis(s_[i]);
            wpts_[i].first  = B.dot(cx_);
            wpts_[i].second = B.dot(cy_);
        }

        // Rebuild arc-length table (chord length of smoothed polyline).  The
        // spline coefficients remain parameterised by the *original* L; a
        // small scale factor in splineParam() bridges the two.
        s_[0] = 0.0;
        for (int i = 1; i < n; i++) {
            double dx = wpts_[i].first  - wpts_[i-1].first;
            double dy = wpts_[i].second - wpts_[i-1].second;
            s_[i] = s_[i-1] + std::hypot(dx, dy);
        }
    }

private:
    std::vector<std::pair<double,double>> wpts_;
    std::vector<double>                   s_;
    std::vector<double>                   v_ref_;

    // ── Cubic B-spline geometry (filled by smoothSpline) ──────────────────
    // Persisting these lets the controller evaluate κ analytically instead
    // of recovering it from finite differences on the discrete waypoints —
    // which was the source of the high-frequency jitter on desired_delta.
    static constexpr int K_ = 3;        // cubic
    std::vector<double>  knots_;        // length nc + K + 1
    Eigen::VectorXd      cx_, cy_;      // control points (length nc)
    double               spline_L_ = 0.0;  // original arc-length the spline was fit on

    // Map a chord-arc-length s on the resampled polyline back to the
    // spline parameter (which was fit on the *pre-resample* arc length).
    // The two differ by < 0.1 % in practice; the linear rescale removes it.
    double splineParam(double s) const
    {
        if (s_.empty() || s_.back() <= 0.0) return 0.0;
        const double ratio = spline_L_ / s_.back();
        return std::clamp(s * ratio, 0.0, spline_L_);
    }

    // Cox-de Boor degree-K_ basis evaluation at parameter t.
    Eigen::RowVectorXd bsplineBasis(double t) const
    {
        const int m  = static_cast<int>(knots_.size());
        const int nc = m - K_ - 1;
        t = std::clamp(t, 0.0, spline_L_);
        if (t >= spline_L_) t = spline_L_ - 1e-10;

        std::vector<double> d(m - 1, 0.0);
        for (int i = 0; i < m - 1; i++)
            if (t >= knots_[i] && t < knots_[i + 1]) d[i] = 1.0;

        for (int deg = 1; deg <= K_; deg++) {
            std::vector<double> d2(m - 1 - deg, 0.0);
            for (int i = 0; i < static_cast<int>(d2.size()); i++) {
                double dl = knots_[i + deg]     - knots_[i];
                double dr = knots_[i + deg + 1] - knots_[i + 1];
                double left  = (dl > 1e-12) ? (t - knots_[i])           / dl * d[i]   : 0.0;
                double right = (dr > 1e-12) ? (knots_[i + deg + 1] - t) / dr * d[i+1] : 0.0;
                d2[i] = left + right;
            }
            d.swap(d2);
        }

        Eigen::RowVectorXd B = Eigen::RowVectorXd::Zero(nc);
        for (int i = 0; i < nc; i++) B(i) = d[i];
        return B;
    }

    // Cox-de Boor with first and second derivatives at parameter t.
    // Uses the standard recursion dN_{i,k}/dt =
    //   k * [ N_{i,k-1}(t) / (knots[i+k] - knots[i])
    //       - N_{i+1,k-1}(t) / (knots[i+k+1] - knots[i+1]) ].
    void bsplineBasisAndDerivs(double t,
                               Eigen::RowVectorXd& B,
                               Eigen::RowVectorXd& Bp,
                               Eigen::RowVectorXd& Bpp) const
    {
        const int m  = static_cast<int>(knots_.size());
        const int nc = m - K_ - 1;
        t = std::clamp(t, 0.0, spline_L_);
        if (t >= spline_L_) t = spline_L_ - 1e-10;

        // Build the degree-0 ladder, then climb to degrees 1, 2, K_ (=3).
        std::vector<double> d0(m - 1, 0.0);
        for (int i = 0; i < m - 1; i++)
            if (t >= knots_[i] && t < knots_[i + 1]) d0[i] = 1.0;

        auto climb = [&](const std::vector<double>& d_in, int deg_out) {
            std::vector<double> d_out(m - 1 - deg_out, 0.0);
            for (int i = 0; i < static_cast<int>(d_out.size()); i++) {
                double dl = knots_[i + deg_out]     - knots_[i];
                double dr = knots_[i + deg_out + 1] - knots_[i + 1];
                double left  = (dl > 1e-12) ? (t - knots_[i])               / dl * d_in[i]   : 0.0;
                double right = (dr > 1e-12) ? (knots_[i + deg_out + 1] - t) / dr * d_in[i+1] : 0.0;
                d_out[i] = left + right;
            }
            return d_out;
        };

        std::vector<double> d1 = climb(d0, 1);            // degree-1 basis
        std::vector<double> d2 = climb(d1, 2);            // degree-2 basis
        std::vector<double> d3 = climb(d2, K_);           // degree-K_ basis  (= cubic)

        B   = Eigen::RowVectorXd::Zero(nc);
        Bp  = Eigen::RowVectorXd::Zero(nc);
        Bpp = Eigen::RowVectorXd::Zero(nc);
        for (int i = 0; i < nc; i++) B(i) = d3[i];

        // First derivative (degree-(K_-1) combination)
        for (int i = 0; i < nc; i++) {
            double dl = knots_[i + K_]     - knots_[i];
            double dr = knots_[i + K_ + 1] - knots_[i + 1];
            double left  = (dl > 1e-12) ? d2[i]   / dl : 0.0;
            double right = (dr > 1e-12) ? d2[i+1] / dr : 0.0;
            Bp(i) = static_cast<double>(K_) * (left - right);
        }

        // Second derivative: apply the same recursion to the degree-(K_-1) basis,
        // which lives in d2 with derivative built from d1.
        const int n2 = static_cast<int>(d2.size());
        std::vector<double> dp_deg2(n2, 0.0);
        for (int i = 0; i < n2; i++) {
            double dl = knots_[i + (K_ - 1)]     - knots_[i];
            double dr = knots_[i + K_]           - knots_[i + 1];
            double left  = (dl > 1e-12) ? d1[i]   / dl : 0.0;
            double right = (dr > 1e-12) ? d1[i+1] / dr : 0.0;
            dp_deg2[i] = static_cast<double>(K_ - 1) * (left - right);
        }
        for (int i = 0; i < nc; i++) {
            double dl = knots_[i + K_]     - knots_[i];
            double dr = knots_[i + K_ + 1] - knots_[i + 1];
            double left  = (dl > 1e-12) ? dp_deg2[i]   / dl : 0.0;
            double right = (dr > 1e-12) ? dp_deg2[i+1] / dr : 0.0;
            Bpp(i) = static_cast<double>(K_) * (left - right);
        }
    }

    std::pair<double,double> interp(double s) const
    {
        if (wpts_.empty()) return {0.0, 0.0};
        if (s <= 0.0)           return wpts_.front();
        if (s >= s_.back())     return wpts_.back();

        auto it  = std::lower_bound(s_.begin(), s_.end(), s);
        size_t i = std::distance(s_.begin(), it);
        if (i == 0) return wpts_[0];
        i = std::min(i, wpts_.size() - 1);

        double t = (s_[i] - s_[i-1] > 1e-12) ?
                   (s - s_[i-1]) / (s_[i] - s_[i-1]) : 0.0;
        t = std::clamp(t, 0.0, 1.0);

        return {
            wpts_[i-1].first  + t*(wpts_[i].first  - wpts_[i-1].first),
            wpts_[i-1].second + t*(wpts_[i].second - wpts_[i-1].second)
        };
    }
};
