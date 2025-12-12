import numpy as np
import time

from calibration import DIST, SAMPLE, THRESHOLDS, PROFILE

if PROFILE:
    class Stats:
        cols = []
        def __init__(self):
            self.times = []
        def collect(self, msg):
            self.times.append((msg, time.perf_counter(), time.process_time()))
        def print(self):
            t00, t01 = self.times[0][1:]
            cols = ["TIME"]
            realtime = ["REAL"]
            proctime = ["PROC"]
            for msg, t0, t1 in self.times:
                cols.append(msg)
                realtime.append(int((t0 - t00) * 1000000))
                proctime.append(int((t1 - t01) * 1000000))
            if not Stats.cols:
                Stats.cols = cols
                print(";".join(cols))
            print(";".join(map(str, realtime)))
            print(";".join(map(str, proctime)))
            realtime = ["REAL inc", 0] + list(np.diff(realtime[1:]))
            proctime = ["PROC inc", 0] + list(np.diff(proctime[1:]))
            print(";".join(map(str, realtime)))
            print(";".join(map(str, proctime)))
else:
    class Stats:
        def collect(self, msg):
            pass
        def print(self):
            pass

def polar(i, q):
    SAMPLES = 80
    c = np.array(i) + np.array(q) * 1j
    c = np.pad(c, (0, SAMPLES), mode="constant", constant_values=0)[:SAMPLES]
    return (np.abs(c), np.angle(c, deg=False))

def fixed_len_append(arr, elem):
    arr[0:-1] = arr[1:]
    arr[-1] = elem

class KalmanFilter:
    def __init__(self, A=1, H=1, R=4):
        self.A = A
        self.H = H
        self.R = R
        self.x_est = None
    def update(self, x, Q):
        # Prediction
        if self.x_est is None or np.isnan(self.x_est):
            self.x_est = x
            self.P = 1.0
        else:
            x_pred = self.A * self.x_est
            P_pred = self.A * self.P * self.A + Q
            # Kalman Gain
            K = P_pred * self.H / (self.H * P_pred * self.H + self.R)

            # Update
            self.x_est = x_pred + K * (x - self.H * x_pred)
            self.P = (1 - K * self.H) * P_pred
        return self.x_est

class Measure:
    def __init__(self, axes, n_campioni, q_kalman):
        self.axes_sel = {}
        for axis in "xyz":
            self.axes_sel[axis] = axis in axes
        self.n_campioni = n_campioni
        self.clear()
        #
        self.q_kalman = q_kalman
        self.std_threshold = THRESHOLDS["rho"]
        self.std_soglia = THRESHOLDS["scale"]
        self.v_offset = np.full(len("xyz"), 0.0)
        self.alpha_dc = 0.01

    def clear(self):
        self.CALIBRATION = {}
        self.CALIBRATION_INPUT = {}
        self.CALIBRATION_COUNT = 0
        for axis in self.get_axes():
            for sensor in [0,1]:
                self.CALIBRATION_INPUT[f"dist_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"T_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"f_{axis}{sensor}"] = np.nan
        self.v_air_history = {axis: np.full(10, np.nan) for axis in "xyz"}
        self.deltaphi_last = {axis: np.full(2, np.nan) for axis in "xyz"}
#       self.v_air_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.v_sound_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.phi_history = {axis: np.full((2,10), np.nan) for axis in "xyz"}
        self.phi_last = {axis: np.full(2, np.nan) for axis in "xyz"}
        self.tof_last = {axis: np.full(2, np.nan) for axis in "xyz"}
        self.count = {axis: np.full(2, 0) for axis in "xyz"}
        self.Autocal = {axis: 0 for axis in "xyz"}
        self.rho_history = {axis: np.full((2, self.n_campioni), np.nan) for axis in "xyz"}
        # Flag di autocalibrazione per ogni asse
        self.autocal_completa = {axis: False for axis in "xyz"}
        self.autocal_misura = {axis: False for axis in "xyz"}
        self.firstCal = {axis: True for axis in "xyz"}

    def get_axes(self):
        return [axis for axis in "xyz" if self.axes_sel[axis]]

    def compute(self, measure):
        if self.CALIBRATION_COUNT < 30:
            print(".", flush=True, end="")
            self.parse_measure_calibration(measure)
            self.CALIBRATION_COUNT += 1
            return
        if not len(self.CALIBRATION):
            self.compute_calibration()
            print(" priming done.", flush=True)
        return self.parse_measure(measure)

    def parse_measure_calibration(self, measure):
        for i, axis in enumerate(self.get_axes()):
            for item in measure.get(axis, []):
                if "ch101" in item:
                    sensor = item["ch101"]
                    rho, phi = polar(item["i"], item["q"])
                    peak = rho.argmax()
                    self.CALIBRATION_INPUT[f"dist_{axis}{sensor}"].append(item["range_mm"])
                    self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"].append(peak)
                    self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"].append(phi)
                    self.CALIBRATION_INPUT[f"f_{axis}{sensor}"] = item["freq"]
                elif "hdc3020" in item:
                    sensor = item["hdc3020"]
                    self.CALIBRATION_INPUT[f"T_{axis}{sensor}"].append(item["temp"])

    def compute_calibration(self):
        # calibration output - per axis, sensor
        # phi0 - phase avg
        # T0   - temperature avg
        # tof0 - base time of flight
        # f    - frequency
        def mode(arr):
            vals,counts = np.unique(arr, return_counts=True)
            index = np.argmax(counts)
            return vals[index]

        axes = self.get_axes()
        N = len(axes)
        output = {k: np.full((N, 2), np.nan) for k in ["dist0", "sample0", "phi0", "T0", "tof0", "f"]}
        for iaxis, axis in enumerate(axes):
            for sensor in [0,1]:
                dist0 = DIST[f"{axis}{sensor}"]
                peak = self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"]
                if len(peak) == 0:
                    self.axes_sel[axis] = False
                    continue

                mode_val = mode(peak)

                if not np.isfinite(mode_val):
                    self.axes_sel[axis] = False
                    continue

                sample0 = int(mode_val) - 2
                phi_array = np.array([phi[sample0] for phi in self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"]])
                if not len(phi_array):
                    self.axes_sel[axis] = False
                    continue
                T = self.CALIBRATION_INPUT[f"T_{axis}{sensor}"]
                f = self.CALIBRATION_INPUT[f"f_{axis}{sensor}"]
                mean_phi = np.mean(phi_array)
                std_phi = np.std(phi_array)
                # Definisci i limiti (ad esempio, 2 deviazioni standard dalla media)
                lower_bound = mean_phi - std_phi
                upper_bound = mean_phi + std_phi
                # Filtra gli outlier
                filtered_phi = phi_array[(phi_array >= lower_bound) & (phi_array <= upper_bound)]
                phi0 = np.mean(filtered_phi)
                T0 = np.mean(T)
                v_sound = np.sqrt(1.4 * 287 * (T0 + 273.15))
                try:
                    tof0 = dist0 / v_sound
                except ZeroDivisionError:
                    self.axes_sel[axis] = False
                    continue
                output["dist0"][iaxis,sensor] = dist0
                output["sample0"][iaxis,sensor] = sample0
                output["phi0"][iaxis,sensor] = phi0
                output["T0"][iaxis,sensor] = T0
                output["tof0"][iaxis,sensor] = tof0
                output["f"][iaxis,sensor] = f
        self.CALIBRATION = output

    def parse_measure(self, measure):
        if not len(self.CALIBRATION):
            # calibration output not yet available, skip
            return

        stats = Stats()
        stats.collect("start")
        axes = self.get_axes()
        N = len(axes)
        v_air = np.full(N, np.nan)
        v_sound = np.full(N, np.nan)
        temp_sonica = np.full(N, np.nan)
        vmean_ax = np.full(N, np.nan)
        scala_ax = np.full(N, 1.0) # default = nessuna scala
        v_air_out = np.full(N, np.nan)

        stats.collect("init done")

        for iaxis, axis in enumerate(axes):
            s = 0
            tof0 = np.array([0., 0.])
            T_now = []
            delta_fase = np.array([0., 0.])
            for item in measure.get(axis, []):
                if "hdc3020" in item:
                    sensor = item["hdc3020"]
                    dist = DIST[f"{axis}{sensor}"]
                    T_now.append(item["temp"])
                    Tmean = np.mean(T_now)
                    v_sound_cal = np.sqrt(1.4 * 287 * (Tmean + 273.15))
                    g = len(T_now) - 1
                    tof0[g] = dist / v_sound_cal
                    if g == 1:
                        tof0[0] = tof0[1]
                    stats.collect(f"{axis} hdc3020.{g} done")

                elif "ch101" in item:
                    sensor = item["ch101"]
                    rho, phi = polar(item["i"], item["q"])
                    finite_idx = np.where(np.isfinite(rho))[0]
                    if len(finite_idx) == 0:
                        continue

                    stats.collect(f"{axis} ch101.{s} data collection done")

                    sample_measure = int(finite_idx[np.argmax(rho[finite_idx])]) - 2

                    sample0_val = self.CALIBRATION["sample0"][iaxis,sensor]
                    if not np.isfinite(sample0_val):
                        continue
                    sample0 = int(sample0_val)
                    amp_max = rho[sample0]

                    soglia_amp = 3000
                    tmp = self.rho_history[axis][sensor]
                    amp_hist = tmp[np.isfinite(tmp)]
                    mean_amp = np.mean(amp_hist) if len(amp_hist) else amp_max

                    # print(f"Mean amplitude for axis {axis}, sensor {sensor}: {mean_amp}", "amp_max:",amp_max)
                    f_amp = 0
                    if (amp_max < (mean_amp - soglia_amp)) or (amp_max > (mean_amp + soglia_amp)):
                        amp_max =  self.rho_history[axis][sensor][-1]
                        f_amp = 1

                    fixed_len_append(self.rho_history[axis][sensor], amp_max)

                    stats.collect(f"{axis} ch101.{s} amp_max filtering done")

                    fase = phi[sample0]
                    #Unwrap fase
                    if abs(fase - self.phi_history[axis][s][-1]) >= np.pi:
                        fase -= np.sign(fase)*2*np.pi
                    # Registrazione fase solo iniziale
                    if np.isnan(self.phi_last[axis][s]):
                        self.phi_last[axis][s] = fase
                    # Registrazione tof solo iniziale
                    if np.isnan(self.tof_last[axis][s]):
                        self.tof_last[axis][s] = tof0[s]
                    if sample_measure==sample0 and f_amp==0:
                        delta_fase[s] = fase - self.phi_last[axis][s]
                        self.deltaphi_last[axis][sensor] = delta_fase[s]
                        fixed_len_append(self.phi_history[axis][s], fase)

                        tmp = self.rho_history[axis][sensor]
                        std_rho = np.std(tmp[np.isfinite(tmp)])
                        if std_rho < self.std_threshold[axis]  and self.firstCal[axis] == True:
                            self.count[axis][s] +=1

                            if self.count[axis][s] == 30:
                                self.Autocal[axis] = 1
                        else:
                                self.count[axis][s] = 0
                    else:
                        delta_fase[s] = self.deltaphi_last[axis][sensor]

                    stats.collect(f"{axis} ch101.{s} done")
                    s += 1

            if self.Autocal[axis] == 1 and self.firstCal[axis]==True:
                self.Autocal[axis] = 0
                self.firstCal[axis]=False
                self.autocal_misura[axis] = True
                self.autocal_completa[axis] = True
                # min e max e prendere media DA AGGIUNGERE
                self.phi_last[axis] = self.phi_history[axis].mean(axis=1)

                self.tof_last[axis] = tof0
                self.count[axis] = [0, 0]

                # Valutare
                delta_fase = np.array([0., 0.])
            else:
                self.autocal_misura[axis] = False

            stats.collect(f"{axis} autocal check done")

            for s in [0,1]:
                if abs(delta_fase[s]) > np.pi:
                    delta_fase[s] = - np.sign(delta_fase[s]) * (2 * np.pi - abs(delta_fase[s]))

            # --- Controllo outlier su delta_fase0 / delta_fase1 ---
            delta_abs = np.abs(delta_fase)

            # Soglie
            OUTLIER_RATIO = 4.0     # quanto può essere più grande uno rispetto all'altro
            MIN_DELTA     = 1e-3    # sotto questo valore consideriamo "rumore"

            if delta_abs[np.isfinite(delta_abs)].size and delta_abs.max() > MIN_DELTA:
                piccolo = delta_abs.min()
                grande  = delta_abs.max()
                # Rapporto tra il più grande e il più piccolo
                ratio = grande / (piccolo + 1e-12)   # evitiamo divisione per zero
                if ratio > OUTLIER_RATIO:
                    w_out = OUTLIER_RATIO / ratio
                    w_out = np.clip(w_out, 0.0, 1.0)
                    # nuova ampiezza "corretta": interpolo tra piccolo e grande
                    # se w_out = 1  -> tengo grande
                    # se w_out = 0  -> porto grande alla stessa ampiezza del piccolo
                    nuova_amp = piccolo + w_out * (grande - piccolo)
                    # applico solo al delta che è outlier (quello "grande")
                    s = 0 if delta_abs[0] > delta_abs[1] else 1
                    delta_fase[s] = np.sign(delta_fase[s]) * nuova_amp

            # # Filtro Mario
            if delta_fase[0] * delta_fase[1] > 0:  # concordi
                alpha = np.abs(delta_fase).min() * np.sign(delta_fase[0])
                delta_fase -= [alpha, alpha]
            elif delta_fase[0] * delta_fase[1] < 0:  # discordi
                delta_phi_avg = (delta_fase[0] - delta_fase[1]) / 2
                delta_fase = np.array([delta_phi_avg, -delta_phi_avg])

            stats.collect(f"{axis} delta fase filtering done")

            # Calcola la velocità del flusso d'aria
            tof = self.tof_last[axis] - (1000000 * delta_fase) / (2 * np.pi * self.CALIBRATION["f"][iaxis])
            dist = self.CALIBRATION["dist0"][iaxis]
            q = dist / tof
            v_air[iaxis]   = 0.5 * (q[0] - q[1])
            v_sound[iaxis] = 0.5 * (q[0] + q[1])

            # Aggiorna lo storico dei valori precedenti
            fixed_len_append(self.v_air_history[axis], v_air[iaxis])

            stats.collect(f"{axis} v_air/v_sound done")

            # --- MEDIA ROBUSTA ---
            vmean = np.abs(np.nan_to_num(self.v_air_history[axis]).mean())
            vmean_ax[iaxis] = vmean

            # --- PARAMETRI ---
            scala_min = 0.35
            scala_max = 1.15
            std_lim   = self.std_soglia[axis]
            v_ref     = 0.15         # soglia per la scala
            v_ref_offset = 0.1       # soglia per il DC remover

            # --- STD MEDIA ---
            std_mean = np.nan_to_num(np.std(self.rho_history[axis], axis=1).mean(), nan=np.inf)

            # 1) Fattore da velocità (continuo e saturato)
            t = np.clip(vmean / v_ref, 0.0, 1.0)
            scala_vel = scala_min + (scala_max - scala_min) * t

            # 2) Piccola correzione da std
            r_std = np.clip(std_mean / std_lim, 0.0, 1.0)
            std_gain_min = 0.9
            std_gain_max = 1.1
            g_std = std_gain_min + (std_gain_max - std_gain_min) * r_std

            # peso della std solo a bassa velocità
            w_std = 1.0 - t      # t=0 -> w_std=1 ; t>=1 -> w_std=0

            # 3) Scala finale complessiva (CONTINUA)
            scala = scala_vel * (1.0 + w_std * (g_std - 1.0))
            scala_ax[iaxis] = scala
            #   SCALA PURA
            v_air_scaled = scala * v_air[iaxis]

            # =========================
            vmean_abs = abs(vmean)
            if vmean_abs <= v_ref_offset:
                if not np.isfinite(self.v_offset[iaxis]):
                    self.v_offset[iaxis] = 0.0

                # --- DC removal ATTIVO (bassa velocità) ---
                self.v_offset[iaxis] = ((1.0 - self.alpha_dc) * self.v_offset[iaxis] + self.alpha_dc * v_air_scaled)
                v_air_out[iaxis] = v_air_scaled - self.v_offset[iaxis]

            else:
                # --- DC removal DISATTIVO (alta velocità) ---
                v_air_out[iaxis] = v_air_scaled

            stats.collect(f"{axis} rescaling for v_air_out done")

            #   KALMAN
            Q = self.q_kalman

            # Per la temperatura usiamo il valore di velocità uscente dal filtro di Kalman
            v_sound_filtered = self.v_sound_filter[axis].update(v_sound[iaxis], Q)
            temp_sonica[iaxis]=v_sound_filtered**2/(1.4*287)-273.15

            stats.collect(f"{axis} kalman for temp_sonica done")

        # =========================
        # SCALA SELETTIVA PER ASSE
        # =========================
        if len(vmean_ax[vmean_ax <= v_ref_offset]) == N:
            # CASO 1: tutti sotto → tengo la scala continua già calcolata
            pass
        else:
            # CASO 2: almeno uno sopra → scala selettiva
            vmean_ax[vmean_ax > v_ref_offset] = 1.15
            vmean_ax[vmean_ax <= v_ref_offset] = 1.0

        # =========================
        # RIAPPLICA LA SCALA ALLE VELOCITÀ
        # =========================
        v_air_out = scala_ax * v_air - self.v_offset

        stats.collect("per axis rescaling done")

        output = {'timestamp': measure["timestamp_end"]}
        for iaxis, axis in enumerate(axes):
            output[axis] = v_air_out[iaxis]
            # --- DIAGNOSTICA / AUTOCAL / TEMPERATURA ---
            output[f"autocalibrazione_asse_{axis}"]   = bool(self.autocal_completa[axis])
            output[f"autocalibrazione_misura_{axis}"] = self.autocal_misura[axis]
            output[f"temp_sonica_{axis}"]             = temp_sonica[iaxis]

            output[f"{axis}_v_mean"] = vmean_ax[iaxis]
            output[f"{axis}_scala"]  = scala_ax[iaxis]

            # --- VELOCITÀ ---
            # v_air grezza
            output[f"{axis}_vair"] = v_air[iaxis]
            # v_air_out filtrata (scala + DC remover)
            output[f"{axis}_vout"] = v_air_out[iaxis]
            # chiave "semplice" usata da new_qt come RAW
            output[axis] = v_air[iaxis]

        stats.collect("end")
        stats.print()

        return output
