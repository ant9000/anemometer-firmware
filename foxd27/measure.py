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
        self.v_offset = {axis: 0.0 for axis in "xyz"}
        self.alpha_dc = 0.01

    def clear(self):
        self.CALIBRATION = {}
        self.CALIBRATION_INPUT = {}
        self.CALIBRATION_COUNT = 0
        self.TOF = {}
        for axis in self.get_axes():
            for sensor in [0,1]:
                self.CALIBRATION_INPUT[f"dist_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"T_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"f_{axis}{sensor}"] = np.nan
        self.v_air_history = {axis: np.full(10, np.nan) for axis in "xyz"}
        self.deltaphi_last = {axis: [np.nan for s in [0,1]] for axis in "xyz"}
#       self.v_air_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.v_sound_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.phi_history = {axis: [np.full(10, np.nan) for s in [0,1]] for axis in "xyz"}
        self.phi_last = {axis: [np.nan for s in [0,1]] for axis in "xyz"}
        self.tof_last = {axis: [np.nan for s in [0,1]] for axis in "xyz"}
        self.count = {axis: [0, 0] for axis in "xyz"}
        self.Autocal = {axis: 0 for axis in "xyz"}
        self.rho_history = {axis: {s: np.full(self.n_campioni, np.nan) for s in [0,1]} for axis in "xyz"}
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
            print(" priming done.")
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

        output = {}
        for axis in self.get_axes():
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
                output[f"dist0_{axis}{sensor}"] = dist0
                output[f"sample0_{axis}{sensor}"] = sample0
                output[f"phi0_{axis}{sensor}"] = phi0
                output[f"T0_{axis}{sensor}"] = T0
                output[f"tof0_{axis}{sensor}"] = tof0
                output[f"f_{axis}{sensor}"] = f
        self.CALIBRATION = output

    def parse_measure(self, measure):
        if not len(self.CALIBRATION):
            # calibration output not yet available, skip
            return
        stats = Stats()
        stats.collect("start")
        v_air = {}
        v_sound = {}
        temp_sonica = {}
#       v_air_filtered = {}
        v_sound_filtered = {}
        v_air_out = {}
#       v_air_filtered_out = {}
        vmean_ax = {axis: np.nan for axis in self.get_axes()}
        scala_ax = {axis: 1.0 for axis in self.get_axes()}   # default = nessuna scala

        # INIZIALIZZA TUTTE LE CHIAVI PER OGNI ASSE!
        for axis in self.get_axes():
            v_air[axis] = np.nan
            v_sound[axis] = np.nan
#           v_air_filtered[axis] = np.nan
            v_sound_filtered[axis] = np.nan
            temp_sonica[axis] = np.nan

        stats.collect("init done")

        for i, axis in enumerate(self.get_axes()):
            s = 0
            f = np.array([0., 0.])
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

                    sample_measure = int(finite_idx[np.argmax(rho[finite_idx])]) - 2

                    sample0_val = self.CALIBRATION.get(f"sample0_{axis}{sensor}", np.nan)
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

                    rho_hist = self.rho_history[axis][sensor]
                    std_rho = np.std(rho_hist[np.isnan(rho_hist) == False])

                    f[s] = self.CALIBRATION[f"f_{axis}{sensor}"]
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

                        if std_rho < self.std_threshold[axis]  and self.firstCal[axis] == True:
                            self.count[axis][s] +=1

                            if self.count[axis][s] == 30:
                                self.Autocal[axis] = 1
                        else:
                                self.count[axis][s] = 0
                    else:
                        delta_fase[s] = self.deltaphi_last[axis][sensor]

                    stats.collect(f"{axis} ch101.{s} done")

                    if s == 0:
                        s = 1
                    else:
                        if self.Autocal[axis] == 1 and self.firstCal[axis]==True:
                            self.Autocal[axis] = 0
                            self.firstCal[axis]=False
                            self.autocal_misura[axis] = True
                            self.autocal_completa[axis] = True
                            # min e max e prendere media DA AGGIUNGERE
                            self.phi_last[axis][0] = np.mean(self.phi_history[axis][0])
                            self.phi_last[axis][1] = np.mean(self.phi_history[axis][1])

                            self.tof_last[axis][0] = tof0[0]
                            self.tof_last[axis][1] = tof0[1]
                            self.count[axis] = [0, 0]

                            # Valutare
                            delta_fase = [0, 0]
                        else:
                            self.autocal_misura[axis] = False

                        stats.collect(f"{axis} autocal check done")

                        for i in [0,1]:
                            if abs(delta_fase[i]) > np.pi:
                                delta_fase[i] = - np.sign(delta_fase[i]) * (2 * np.pi - abs(delta_fase[i]))

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
                                i = 0 if delta_abs[0] > delta_abs[1] else 1
                                delta_fase[i] = np.sign(delta_fase[i]) * nuova_amp

                        # # Filtro Mario
                        if delta_fase[0] * delta_fase[1] > 0:  # concordi
                            alpha = np.abs(delta_fase).min() * np.sign(delta_fase[0])
                            delta_fase -= [alpha, alpha]
                        elif delta_fase[0] * delta_fase[1] < 0:  # discordi
                            delta_phi_avg = (delta_fase[0] - delta_fase[1]) / 2
                            delta_fase = np.array([delta_phi_avg, -delta_phi_avg])

                        stats.collect(f"{axis} delta fase filtering done")

            for i in [0,1]:
                self.TOF[f"{axis}{i}"] = self.tof_last[axis][i] - (1000000 * delta_fase[i]) / (2 * np.pi * f[i])

            # Calcola la velocità del flusso d'aria
            dist = [self.CALIBRATION[f"dist0_{axis}{sensor}"] for sensor in [0,1]]
            v_air[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] - dist[1] / self.TOF[f"{axis}1"])
            v_sound[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] + dist[1] / self.TOF[f"{axis}1"])

            # Aggiorna lo storico dei valori precedenti
            fixed_len_append(self.v_air_history[axis], v_air[axis])

            stats.collect(f"{axis} v_air/v_sound done")

            tmp = self.rho_history[axis][0]
            rho_hist0 = tmp[np.isfinite(tmp)]
            std_rho0 = np.std(rho_hist0) if len(rho_hist0) else np.inf

            tmp = self.rho_history[axis][1]
            rho_hist1 = tmp[np.isfinite(tmp)]
            std_rho1 = np.std(rho_hist1) if len(rho_hist1) else np.inf

            # --- MEDIA ROBUSTA ---
            tmp = self.v_air_history[axis]
            hist = tmp[np.isfinite(tmp)]
            vmean = abs(np.mean(hist)) if len(hist) > 0 else 0.0
            vmean_ax[axis] = vmean

            # --- PARAMETRI ---
            scala_min = 0.35
            scala_max = 1.15
            std_lim   = self.std_soglia[axis]
            v_ref     = 0.15         # soglia per la scala
            v_ref_offset = 0.1       # soglia per il DC remover

            # --- STD MEDIA ---
            std_mean = 0.5 * (std_rho0 + std_rho1)

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
            scala_ax[axis] = scala
            #   SCALA PURA
            v_air_scaled = scala * v_air[axis]

            # =========================
            vmean_abs = abs(vmean)
            if vmean_abs <= v_ref_offset:
                if not np.isfinite(self.v_offset[axis]):
                    self.v_offset[axis] = 0.0

                # --- DC removal ATTIVO (bassa velocità) ---
                self.v_offset[axis] = ((1.0 - self.alpha_dc) * self.v_offset[axis] + self.alpha_dc * v_air_scaled)
                v_air_out[axis] = v_air_scaled - self.v_offset[axis]

            else:
                # --- DC removal DISATTIVO (alta velocità) ---
                v_air_out[axis] = v_air_scaled

            stats.collect(f"{axis} rescaling for v_air_out done")

            #   KALMAN
            Q = self.q_kalman

            # Kalman sempre sulla velocità grezza
#           v_air_filtered[axis] = self.v_air_filter[axis].update(v_air[axis], Q)
            v_sound_filtered[axis] = self.v_sound_filter[axis].update(v_sound[axis], Q)

            # Kalman scalato e con OFFSET TOLTO
#           v_air_filtered_out[axis] = scala_ax[axis] * v_air_filtered[axis] - self.v_offset[axis]

            # Per la temperatura usiamo il valore di velocità uscente dal filtro di Kalman
            temp_sonica[axis]=v_sound_filtered[axis]*v_sound_filtered[axis]/(1.4*287)-273.15

            stats.collect(f"{axis} kalman for temp_sonica done")

        # =========================
        # SCALA SELETTIVA PER ASSE
        # =========================

        assi_sotto = [
            a for a in self.get_axes()
            if vmean_ax[a] <= v_ref_offset
        ]

        assi_sopra = [
            a for a in self.get_axes()
            if vmean_ax[a] > v_ref_offset
        ]

        # CASO 1: tutti sotto → tengo la scala continua già calcolata
        if len(assi_sotto) == len(self.get_axes()):
            pass

        # CASO 2: almeno uno sopra → scala selettiva
        else:
            for a in self.get_axes():
                if a in assi_sopra:
                    scala_ax[a] = 1.15
                else:
                    scala_ax[a] = 1.0

        # =========================
        # RIAPPLICA LA SCALA ALLE VELOCITÀ
        # =========================
        for a in self.get_axes():
#           v_air_filtered_out[a] = scala_ax[a] * v_air_filtered[a] - self.v_offset[a]
            v_air_out[a] = scala_ax[a] * v_air[a] - self.v_offset[a]

        stats.collect("per axis rescaling done")

        if len(v_air_out) == len(self.get_axes()):
            v_air_out['timestamp'] = measure["timestamp_end"]

            for axis in self.get_axes():
                # --- KALMAN (già scalato e con offset tolto) ---
#               v_air_out[f"{axis}_kalman"] = v_air_filtered_out.get(axis, np.nan)

                # --- DIAGNOSTICA / AUTOCAL / TEMPERATURA ---
                v_air_out[f"autocalibrazione_asse_{axis}"]   = bool(self.autocal_completa[axis])
                v_air_out[f"autocalibrazione_misura_{axis}"] = self.autocal_misura[axis]
                v_air_out[f"temp_sonica_{axis}"]             = temp_sonica[axis]

                v_air_out[f"{axis}_v_mean"] = vmean_ax[axis]
                v_air_out[f"{axis}_scala"]  = scala_ax[axis]

                # --- VELOCITÀ ---
                # v_air grezza
                v_air_out[f"{axis}_vair"] = v_air[axis]
                # v_air_out filtrata (scala + DC remover)
                v_air_out[f"{axis}_vout"] = v_air_out[axis]
                # chiave "semplice" usata da new_qt come RAW
                v_air_out[axis] = v_air[axis]

            stats.collect("end")
            stats.print()

            return v_air_out
