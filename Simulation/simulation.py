import numpy as np
import matplotlib.pyplot as plt

# =========================================================
# 1) PI gains from settling time (2% rule)
# =========================================================
def pi_gains_from_settling_time(K_rpm_per_V, tau, Ts, zeta=0.7):
    wn = 4.0 / (zeta * Ts)
    Kp = (2.0 * zeta * wn * tau - 1.0) / K_rpm_per_V   # units: V/rpm
    Ki = (wn**2 * tau) / K_rpm_per_V                   # units: V/(rpm*s)
    return Kp, Ki, wn

# =========================================================
# 2) Plant Euler step: tau*dω/dt + ω = K*u
#    ω in rpm, u in volts, K in rpm/V
# =========================================================
def plant_euler_step(omega_prev_rpm, u_volts, dt, K_rpm_per_V, tau):
    domega = (-omega_prev_rpm + K_rpm_per_V * u_volts) / tau
    return omega_prev_rpm + dt * domega

# =========================================================
# 3) Open-loop simulation
#    Use u = omega_ref/K (feedforward) and clamp to 0..2.5V
# =========================================================
def simulate_open_loop_rpm(omega_ref_rpm, T, dt, K_rpm_per_V, tau, u_min=0.0, u_max=2.5):
    t = np.arange(0.0, T + dt, dt)
    N = len(t)

    omega = np.zeros(N)
    u = np.zeros(N)

    u_step = omega_ref_rpm / K_rpm_per_V
    u_step = np.clip(u_step, u_min, u_max)

    for n in range(1, N):
        u[n] = u_step
        omega[n] = plant_euler_step(omega[n-1], u[n], dt, K_rpm_per_V, tau)

    return t, omega, u

# =========================================================
# 4) Closed-loop incremental PI with saturation + anti-windup
#    u[n] = u[n-1] + (Kp + Ki*dt)e[n] - Kp e[n-1]
# =========================================================
def simulate_closed_loop_incremental_pi_rpm(omega_ref_rpm, Kp, Ki, T, dt, K_rpm_per_V, tau, u_min=0.0, u_max=2.5):
    t = np.arange(0.0, T + dt, dt)
    N = len(t)

    omega = np.zeros(N)
    u = np.zeros(N)
    e = np.zeros(N)

    # init
    e[0] = omega_ref_rpm - omega[0]
    u[0] = np.clip(Kp * e[0], u_min, u_max)
    omega[0] = plant_euler_step(omega[0], u[0], dt, K_rpm_per_V, tau)

    for n in range(1, N):
        e[n] = omega_ref_rpm - omega[n-1]

        du = (Kp + Ki*dt) * e[n] - Kp * e[n-1]
        u_unsat = u[n-1] + du
        u_sat = np.clip(u_unsat, u_min, u_max)

        # anti-windup: if saturated and du pushes further into saturation, freeze u
        if u_unsat != u_sat:
            if (u_sat == u_max and du > 0) or (u_sat == u_min and du < 0):
                u[n] = u[n-1]
            else:
                u[n] = u_sat
        else:
            u[n] = u_unsat

        omega[n] = plant_euler_step(omega[n-1], u[n], dt, K_rpm_per_V, tau)

    return t, omega, u, e

# =========================================================
# 5) Plot: open-loop vs closed-loop on same axes
# =========================================================
def plot_open_vs_closed(t, omega_open, omega_closed, omega_ref_rpm, u_open=None, u_closed=None):
    plt.figure(figsize=(9, 5))
    plt.plot(t, omega_open, "--", label="Open-loop (u = ωref/K, clamped)")
    plt.plot(t, omega_closed, "-", label="Closed-loop (PI + saturation)")
    plt.axhline(omega_ref_rpm, linestyle=":", linewidth=1, color="k", label="Reference")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed ω [rpm]")
    plt.title("RPM Response: Open-loop vs PI Closed-loop (0–2.5 V)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # optional: plot voltage too
    if (u_open is not None) and (u_closed is not None):
        plt.figure(figsize=(9, 4))
        plt.plot(t, u_open, "--", label="Open-loop u [V]")
        plt.plot(t, u_closed, "-", label="Closed-loop u [V]")
        plt.axhline(2.5, linestyle=":", linewidth=1, color="k", label="u_max = 2.5 V")
        plt.axhline(0.0, linestyle=":", linewidth=1, color="k", label="u_min = 0 V")
        plt.xlabel("Time [s]")
        plt.ylabel("Voltage u [V]")
        plt.title("Control Voltage (with saturation)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

# =========================================================
# Example run
# =========================================================
if __name__ == "__main__":
    K = 1000.0      # rpm/V
    tau = 0.87
    Ts = 1.0
    zeta = 0.7

    dt = 0.001
    T = 5.0

    # Choose a reference <= 2500 rpm to be reachable
    omega_ref = 2000.0  # rpm

    Kp, Ki, wn = pi_gains_from_settling_time(K, tau, Ts, zeta)
    print(f"wn={wn:.6f} rad/s, Kp={Kp:.9f} V/rpm, Ki={Ki:.9f} V/(rpm*s)")

    t, omega_ol, u_ol = simulate_open_loop_rpm(omega_ref, T, dt, K, tau, 0.0, 2.5)
    t, omega_cl, u_cl, e_cl = simulate_closed_loop_incremental_pi_rpm(omega_ref, Kp, Ki, T, dt, K, tau, 0.0, 2.5)

    plot_open_vs_closed(t, omega_ol, omega_cl, omega_ref, u_ol, u_cl)
