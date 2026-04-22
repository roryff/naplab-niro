# Steering System Identification - Field Test Plan

**Vehicle:** Kia Niro EV  
**Purpose:** Identify the steering actuator transfer function `u -> steering_angle_deg` to
replace hand-tuned MPC model parameters with fitted values.

---

## What we are identifying

The MPC uses a **first-order lag actuator model**:

```
u(t) [-1,1]  ->  dRate [sw-deg/s]  ->  delta [sw-deg]
               tau_r, G_r(v)            integrator
```

Parameters to extract:

| Parameter | Meaning | Current guess |
|-----------|---------|---------------|
| `tau_r` | Time constant of steering rate response [s] | 0.78 s |
| `G_r(v)` | Steady-state rate per unit torque [sw-deg/s] | 36 (constant, not speed-dependent) |
| `G0, va, vb` | Speed-dependent model `G0/(va*v^2+vb*v+1)` | Not fitted |
| `delay_s` | Transport delay [s] | 0.0 (unknown) |

---

## Primary output signal

**`vehicle/state.steering_angle_deg`** - steering wheel angle from CAN bus, smooth, ~100 Hz.

Do **not** use lateral acceleration: it couples in tyre/vehicle dynamics and requires ADR mode.

Also record:
- `vehicle/state.car_output_torque` - what panda actually applied (reveals rate limiting)
- `vehicle/state.actuators_torque` - commanded echo
- `vehicle/state.v_ego` - for speed-segmented gain fitting
- `gnss/gyro` (z) - yaw-rate cross-check (if ADR active)

---

## Amplitude choices

| Mode | Amplitude | Reason |
|------|-----------|--------|
| `ratelimit` | 1.0 (fixed) | Measuring the limits themselves |
| `step` | 1.0 | Real operating point; actuator saturates in normal use |
| `prbs` | 0.70 | **Primary broadband ID signal.** Always at +/-A (fully settled), so rate-limiting only creates brief ramp transients, not distorted waveforms. Use `car_output_torque` as MATLAB input. |
| `chirp` | 0.30 | Validation / Bode plot only. Keep amplitude low (<=0.30) so sinusoids stay undistorted below ~0.55 Hz. |

**Why PRBS instead of chirp at high amplitude:**
At A=0.70 a sine wave at f>0.24 Hz hits the panda rate limiter (rise: 1.033/s, fall: 1.833/s).
Because the two rates differ, the clipped sine becomes an **asymmetric** triangle/sawtooth wave —
not just a scaled version — which introduces even harmonics and can bias spectral estimates.
PRBS sidesteps this entirely: it only ever commands +A or -A, so the rate limiter only affects
the brief transitions between levels. With min_hold=0.5s >> 1/1.033=0.97s (one full-step ramp
time), most of each bit period is settled flat — `car_output_torque` is a clean binary signal.

---

## Safety

| Constraint | Limit | sysid_node parameter |
|------------|-------|---------------------|
| Steering angle watchdog | +/-300 sw-deg | `steer_abort_deg` |
| Speed deviation from target | +/-3 m/s | `speed_abort_mps` |
| Requires Comma lat_active | true | hard gate, not configurable |

Road must be **straight and clear**. A co-pilot must be ready to take the wheel immediately.

---

## Pre-test checklist

- [ ] `sudo bash scripts/apply_sysid_cmake.sh && colcon build --packages-select car_control`
- [ ] Comma device connected, `lat_active` can be asserted
- [ ] `ros2 topic hz /vehicle/state` -> >= 50 Hz
- [ ] `ros2 topic echo /sysid/status` -> visible
- [ ] rosbag recording started (Terminal 2 command below) **before** starting sysid_node
- [ ] Driver briefed: hold target speed; grab wheel if node prints ABORT

---

## Software setup

### Terminal 1 - sensor bridge

```bash
# Start comma + GNSS nodes only (no path follower -- sysid_node owns cmd_vel)
ros2 run car_control gnss_node &
ros2 run car_control comma_node
```

### Terminal 2 - rosbag (start BEFORE sysid_node)

```bash
ros2 bag record -o ~/bags/sysid_$(date +%Y%m%d_%H%M%S) \
  /vehicle/state \
  /gnss/gyro \
  /gnss/accel \
  /cmd_vel \
  /sysid/torque_cmd \
  /sysid/status
```

### Terminal 3 - sysid_node

```bash
ros2 run car_control sysid_node --ros-args \
  -p test_mode:=<MODE> \
  -p amplitude:=<A> \
  -p desired_speed_mps:=<V> \
  [other params]
```

The node waits until `lat_active=true`, then begins automatically.
Status is printed on `/sysid/status` and the node logger.

---

## Test sequence

### Test 0 - Rate-limit characterisation (~5 min, car stationary or <= 3 km/h)

**Purpose:** Measure the panda asymmetric safety rate limits precisely.

```bash
ros2 run car_control sysid_node --ros-args \
  -p test_mode:=ratelimit \
  -p duration_s:=25.0 \
  -p desired_speed_mps:=0.0
```

Automatic sequence:

| Time | Torque command |
|------|---------------|
| 0-2 s | 0.0 (baseline) |
| 2-7 s | +1.0 (observe rise in `car_output_torque`) |
| 7-12 s | 0.0 (observe fall) |
| 12-17 s | -1.0 |
| 17-22 s | 0.0 |

**What to check:** plot `car_output_torque` vs time, measure rise and fall slopes.
Expected: rise ~1.03 normalized/s, fall ~1.83 normalized/s.

---

### Test 1 - Step responses (~20 min total, straight road)

**Purpose:** Fit `tau_r` and `G_r` at multiple speeds.

**Speeds:** 5, 10, 15, 20 km/h. Run each speed as a separate bag segment.

```bash
# Example at 10 km/h (2.78 m/s)
ros2 run car_control sysid_node --ros-args \
  -p test_mode:=step \
  -p amplitude:=1.0 \
  -p duration_s:=80.0 \
  -p desired_speed_mps:=2.78

```

speeds in mps: 5 km/h = 1.39 m/s, 10 km/h = 2.78 m/s, 15 km/h = 4.17 m/s, 20 km/h = 5.56 m/s
25 km/h = 6.94 m/s, 36 km/h = 10.0 m/s keep going further up
30 km/h = 8.33 m/s
40 km/h = 11.11 m/s 
45 km/h = 12.5 m/s
50 km/h = 13.89 m/s
55 km/h = 15.28 m/s
60 km/h = 16.67 m/s
Automatic step pattern (repeats every 16 s for 5 full cycles at 80 s):

| Phase | Torque | Duration |
|-------|--------|----------|
| +step | +1.0 | 4 s |
| zero  | 0.0  | 4 s |
| -step | -1.0 | 4 s |
| zero  | 0.0  | 4 s |

Repeat with `amplitude:=0.5` to check linearity. If step shapes are identical (just scaled),
the system is linear across the amplitude range.

---

### Test 2 - PRBS broadband excitation (~30 min total)

**Purpose:** Primary broadband identification — fits `tau_r`, `G_r(v)`, delay. Works correctly
with the panda rate limiter because the signal is always fully settled at +/-amplitude.

**Speeds:** 5, 10, 20 km/h. Each run ~90 s (gives many transitions for good spectral coverage).
10 km/h = 2.78 m&s
15 km/h = 4.17 m/s
20 km/h = 5.56 m/s
```bash
# Example at 5 km/h (1.39 m/s)
ros2 run car_control sysid_node --ros-args \
  -p test_mode:=prbs \
  -p amplitude:=0.70 \
  -p prbs_min_hold_s:=0.5 \
  -p prbs_max_hold_s:=3.0 \
  -p duration_s:=90.0 \
  -p desired_speed_mps:=1.39
```

With min_hold=0.5s the fastest switching frequency is ~1 Hz. With max_hold=3.0s you get
low-frequency content down to ~0.15 Hz — covering the full MPC-relevant bandwidth.
The panda ramp-up takes ~0.7/1.033 = 0.68s for a full step, so each bit is settled for
most of its hold time at min_hold=0.5s... consider increasing min_hold to 1.5s if you
want the signal fully settled before the next switch:

```bash
# Slower switching, fully settled each bit (recommended if first run shows ragged car_output_torque)
  -p prbs_min_hold_s:=1.5 \
  -p prbs_max_hold_s:=4.0 \
  -p duration_s:=120.0
```

### Test 2b - Chirp (optional, Bode validation only)

Only needed to visually validate the fitted model against a Bode plot.
Keep amplitude low so the sinusoid stays undistorted:

```bash
ros2 run car_control sysid_node --ros-args \
  -p test_mode:=chirp \
  -p amplitude:=0.30 \
  -p freq_start:=0.20 \
  -p freq_end:=2.0 \
  -p duration_s:=60.0 \
  -p desired_speed_mps:=1.39
```

---

## Post-processing (Python, offline)

### Extract data from rosbag

```python
import glob, os
import sqlite3
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def bag_to_df(bag_dir, topic, fields):
    db3 = glob.glob(os.path.join(bag_dir, "*.db3"))[0]
    conn = sqlite3.connect(db3)
    type_str = conn.execute(
        "SELECT type FROM topics WHERE name=?", (topic,)).fetchone()[0]
    MsgType = get_message(type_str)
    rows = conn.execute(
        "SELECT timestamp, data FROM messages "
        "JOIN topics ON messages.topic_id = topics.id "
        "WHERE topics.name=?", (topic,)).fetchall()
    conn.close()
    records = []
    for ts, blob in rows:
        msg = deserialize_message(blob, MsgType)
        rec = {"t": ts * 1e-9}
        for f in fields:
            rec[f] = getattr(msg, f)
        records.append(rec)
    return pd.DataFrame(records)

BAG = "~/bags/sysid_20250422_120000"
df_state = bag_to_df(BAG, "/vehicle/state",
    ["steering_angle_deg", "v_ego", "car_output_torque", "actuators_torque"])
df_cmd   = bag_to_df(BAG, "/sysid/torque_cmd", ["data"])
df_cmd   = df_cmd.rename(columns={"data": "u_cmd"})

df = pd.merge_asof(df_state.sort_values("t"), df_cmd.sort_values("t"), on="t")
df["v_mps"] = df["v_ego"] / 3.6
```

### Step response fitting

```python
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter

dt = np.mean(np.diff(df["t"]))
df["dRate"] = savgol_filter(
    df["steering_angle_deg"], window_length=11, polyorder=3, deriv=1, delta=dt)

# Isolate one rising edge: u goes 0 -> +1.0, lasting 4 s
mask = (df["t"] > t0) & (df["t"] < t0 + 4.0)
seg  = df[mask].copy()
seg["t_rel"] = seg["t"] - seg["t"].iloc[0]

def first_order_step(t, G_r, tau_r):
    return G_r * 1.0 * (1.0 - np.exp(-t / tau_r))   # u=1.0

popt, _ = curve_fit(first_order_step, seg["t_rel"], seg["dRate"], p0=[36.0, 0.78])
G_r_fit, tau_r_fit = popt
print(f"G_r = {G_r_fit:.1f} sw-deg/s/unit    tau_r = {tau_r_fit:.3f} s")
```

### Bode plot from chirp

```python
from scipy import signal
import matplotlib.pyplot as plt

fs = 20.0
u = df["u_cmd"].values
y = df["steering_angle_deg"].values

f, Pxx = signal.welch(u, fs=fs, nperseg=512)
_, Pxy = signal.csd(u, y, fs=fs, nperseg=512)
H = Pxy / Pxx   # H1 estimate: torque -> steering angle

fig, ax = plt.subplots(2, 1, sharex=True)
ax[0].semilogx(f, 20 * np.log10(np.abs(H)));  ax[0].set_ylabel("Magnitude [dB]")
ax[1].semilogx(f, np.angle(H, deg=True));      ax[1].set_ylabel("Phase [deg]")
ax[1].set_xlabel("Frequency [Hz]")
plt.tight_layout(); plt.show()
```

### Speed-dependent gain model

```python
# After extracting G_r per speed bin from step fitting:
speeds_mps = np.array([5, 10, 15, 20]) / 3.6
gains      = np.array([G_r_5, G_r_10, G_r_15, G_r_20])

def gain_model(v, G0, va, vb):
    return G0 / (va * v**2 + vb * v + 1.0)

popt, _ = curve_fit(gain_model, speeds_mps, gains, p0=[300.0, 0.01, 0.08])
G0_fit, va_fit, vb_fit = popt
print(f"G0={G0_fit:.1f}  va={va_fit:.4f}  vb={vb_fit:.4f}  tau_r={tau_r_fit:.3f}")
```

### Update model config

Edit `config/integrator_model.yaml`:
```yaml
G0: <G0_fit>
va: <va_fit>
vb: <vb_fit>
tau_r: <tau_r_fit>
```

---

## Expected results

| Speed | Expected G_r [sw-deg/s / unit torque] |
|-------|--------------------------------------|
| 5 km/h | 150-200 |
| 10 km/h | 100-130 |
| 20 km/h | 60-80 |
| 36 km/h | ~36 (current baseline) |

Expected `tau_r`: 0.5-1.0 s. Expected transport delay: 0-100 ms.

If fitted values differ significantly from defaults (`tau_r=0.78`, `gain_r=36`), update
`config/integrator_model.yaml` and the hardcoded defaults in
`src/steering_mpc_node.cpp` and `src/lateral_mpc_node.cpp`.

---

## MATLAB System Identification Toolbox workflow

### Step 1 - Convert bag to CSV

Run on any machine that has Python 3 + numpy (no ROS needed):

```bash
python3 scripts/bag_to_csv.py  ~/bags/sysid_20250422_120000  ~/sysid_step_10kmh.csv
```

This resamples everything to a **uniform 20 Hz** time base and writes:

| Column | Description |
|--------|-------------|
| `t` | Time from bag start [s] |
| `u_cmd` | Excitation command sent [-1, 1] |
| `steering_angle_deg` | Steering wheel angle [sw-deg] |
| `car_output_torque` | Torque panda actually applied [-1, 1] |
| `actuators_torque` | Command echo [-1, 1] |
| `v_ego_kmh` | Speed [km/h] |

**Always use `car_output_torque` as the MATLAB input `u`**, not `u_cmd`.
`car_output_torque` is what the panda actually applied to the steering column — it captures
any rate-limiting faithfully. For PRBS data it looks like a clean binary signal with short
ramp transitions at each switch.

### Step 2 - Load into MATLAB

```matlab
data = readtable('sysid_prbs_10kmh.csv');
Ts   = 0.05;   % 20 Hz

u = data.car_output_torque;  % ACTUAL applied torque — always use this, not u_cmd
y = data.steering_angle_deg; % output: steering wheel angle [sw-deg]

% Inspect first — verify u looks like a clean binary signal (not a sawtooth)
figure; plot(data.t, u, data.t, y/max(abs(y))*max(abs(u)))
legend('car\_output\_torque','steer (normalised)')

sys_data = iddata(y, u, Ts, 'Name', 'sysid_10kmh');
sys_data.InputName  = 'TorqueCmd';
sys_data.OutputName = 'SteeringAngleDeg';
sys_data.TimeUnit   = 's';
```

### Step 3 - Model estimation

**Option A: Transfer function (recommended, maps directly to MPC model)**

The expected structure is an integrator (angle = integral of rate) with a first-order
rate lag, giving a 2nd-order system with 1 zero:

```matlab
% tfest(data, num_poles, num_zeros)
% Start with 2 poles (integrator + lag), 0 zeros
model_tf = tfest(sys_data, 2, 0);
present(model_tf)
```

If residuals show systematic error, try 2 poles 1 zero:
```matlab
model_tf = tfest(sys_data, 2, 1);
```

**Option B: State-space (for cross-validation)**

```matlab
model_ss = ssest(sys_data, 2);   % 2nd order state-space
```

**Option C: Parametric fit with known structure (best for MPC)**

Constrain the model to the known first-order rate + integrator structure to get
`tau_r` and `G_r` directly:

```matlab
% G(s) = G_r / (s * (tau_r * s + 1))
%      = G_r / (tau_r * s^2 + s)
% Initial guesses from current MPC parameters:
tau_r_init = 0.78;
G_r_init   = 36.0;

sys_init = tf([G_r_init], [tau_r_init 1 0]);
opt = tfestOptions('InitialCondition', 'estimate');
model_constrained = tfest(sys_data, sys_init, opt);
[num, den] = tfdata(model_constrained, 'v');
% den = [tau_r, 1, 0]  ->  tau_r = den(1)
% num = [G_r]          ->  G_r   = num(1) / den(2)
tau_r_fit = den(1);
G_r_fit   = num(end);
fprintf('tau_r = %.3f s    G_r = %.1f sw-deg/s/unit\n', tau_r_fit, G_r_fit)
```

### Step 4 - Validate

```matlab
% Split data: first half for estimation, second half for validation
N = length(sys_data);
data_est = sys_data(1:floor(N/2));
data_val = sys_data(floor(N/2)+1:end);

model_tf = tfest(data_est, 2, 0);
compare(data_val, model_tf)   % plots measured vs simulated, shows fit %

% Residual analysis (good fit: residuals should be white noise)
resid(data_val, model_tf)

% Bode plot of the identified model
bode(model_tf)
```

### Step 5 - Fit speed-dependent gain G_r(v)

Run the above for each speed bin (5, 10, 15, 20 km/h). Extract `G_r` from each,
then fit the MPC speed model:

```matlab
% Collected from step fits:
speeds_mps = [5 10 15 20] / 3.6;
G_r_values = [G_r_5 G_r_10 G_r_15 G_r_20];

% Fit G_r(v) = G0 / (va*v^2 + vb*v + 1)
% Rearranged: G_r * (va*v^2 + vb*v + 1) = G0
% Linear in [G0, G0*va, G0*vb] -> use least squares
A = [ones(4,1), speeds_mps'.^2 .* G_r_values', speeds_mps' .* G_r_values'];
b = G_r_values';
x = A \ b;   % x = [G0; va; vb]

G0_fit = x(1);
va_fit = x(2) / x(1);
vb_fit = x(3) / x(1);
fprintf('G0=%.1f  va=%.4f  vb=%.4f  tau_r=%.3f\n', G0_fit, va_fit, vb_fit, tau_r_fit)
```

### Step 6 - Update MPC config

Edit `config/integrator_model.yaml` with the fitted values:
```yaml
G0: <G0_fit>
va: <va_fit>
vb: <vb_fit>
tau_r: <tau_r_fit>
```

Also update the hardcoded defaults in `src/steering_mpc_node.cpp` and
`src/lateral_mpc_node.cpp` (around line 29-34, variables `tau_r` and `gain_r`).
