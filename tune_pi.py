"""tune_pi.py - PI tuning for the hybrid-storage controller, no MATLAB toolbox needed.

Same method as MATLAB's pidtune with a target speed and margin: for each loop,
solve for the Kp and Ki that put the crossover at the target speed with the
target phase margin. Then simulate a 1.2 A load step to get the bus dip.
Needs only numpy:  python tune_pi.py
"""
import numpy as np

Vbus, L, C, D, Td = 48.0, 1e-3, 3300e-6, 0.5, 75e-6   # model values; Td = C2000 delay

def pi_for(G, f_c, pm_deg):
    """Kp, Ki that give crossover f_c [Hz] and phase margin pm_deg for plant G(jw)."""
    w = 2*np.pi*f_c
    g = G(w)
    phi_c = -180 + pm_deg - np.degrees(np.angle(g))     # phase the PI must add
    if not (-90 < phi_c <= 0):
        raise ValueError(f"{pm_deg} deg at {f_c} Hz is not reachable with a PI (needs {phi_c:.1f} deg)")
    wz = w*np.tan(np.radians(-phi_c))                    # PI zero = Ki/Kp
    kp = 1/(abs(g)*np.sqrt(1 + (wz/w)**2))
    return kp, kp*wz

def margin(Cpi, G, f_lo=1, f_hi=1e5):
    f = np.logspace(np.log10(f_lo), np.log10(f_hi), 200000); w = 2*np.pi*f
    Lw = (Cpi[0] + Cpi[1]/(1j*w))*G(w)
    i = np.argmin(abs(abs(Lw) - 1))
    return f[i], 180 + np.degrees(np.angle(Lw[i]))

Gi = lambda w: Vbus/(1j*w*L)*np.exp(-1j*w*Td)                    # duty -> inductor current
def Gv_with(ci):                                                  # current target -> bus voltage
    def G(w):
        Li = (ci[0] + ci[1]/(1j*w))*Gi(w)
        return Li/(1 + Li)*D/(1j*w*C)
    return G
R = 48/1.8                                                        # average load, 26.7 ohm
Gg = lambda w: 1/(1j*w*C + 1/R)/(1 + 1j*w/(2*np.pi*10))           # grid current -> bus volts, 10 Hz filter

def bus_dip(ci, cv, step=1.2, T=0.05, dt=1e-6):
    """Small-signal cascade with the delay; returns the worst bus dip [V]."""
    n = int(T/dt); nd = int(round(Td/dt)); dq = np.zeros(nd+1)
    IL = V = xi = xv = 0.0; worst = 0.0
    for k in range(n):
        iload = step if k*dt > 1e-3 else 0.0
        ev = -V; xv += cv[1]*ev*dt; Iref = cv[0]*ev + xv          # voltage loop
        ei = Iref - IL; xi += ci[1]*ei*dt; d = ci[0]*ei + xi      # current loop
        dq = np.roll(dq, -1); dq[-1] = d; d_applied = dq[0]       # C2000 delay
        IL += Vbus*d_applied/L*dt
        V += (D*IL - iload)/C*dt
        worst = min(worst, V)
    return -worst

if __name__ == "__main__":
    ci = pi_for(Gi, 1000, 60)
    cv = pi_for(Gv_with(ci), 100, 60)
    cg = pi_for(Gg, 10, 45)
    rows = [("Current loop  (1 kHz, 60 deg)", ci, Gi, (0.20, 192)),
            ("Voltage loop  (100 Hz, 60 deg)", cv, Gv_with(ci), (4.2, 265)),
            ("Grid loop     (10 Hz, 45 deg)", cg, Gg, (0.2, 2.5))]
    for name, c, G, now in rows:
        f, pm = margin(c, G)
        fn, pmn = margin(now, G if name[0] != "V" else Gv_with((0.20, 192)))
        print(f"{name}:  Kp = {c[0]:.3f}  Ki = {c[1]:.1f}   -> {f:.0f} Hz, {pm:.0f} deg"
              f"   | model now Kp {now[0]}, Ki {now[1]} -> {fn:.0f} Hz, {pmn:.0f} deg")
    print(f"\nBus dip, 1.2 A step:  tuned gains {bus_dip(ci, cv):.2f} V   |   model now {bus_dip((0.20,192),(4.2,265)):.2f} V")
