from scipy import stats
import numpy as np

from calibration import DIST, SAMPLE

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
        self.std_threshold = {"x": 60, "y": 60, "z": 60}
        self.std_soglia = {"x": 150, "y": 150, "z": 150}
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
        self.v_air_history = {axis: [np.nan]*10 for axis in "xyz"}
        self.deltaphi_history = {axis: {0: [np.nan] * 3, 1: [np.nan] * 3} for axis in "xyz"}
        self.v_sound_history = {axis: [np.nan]*3 for axis in "xyz"}
        self.v_air_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.v_sound_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.phi0_history = {axis: {0: np.full(15, np.nan)} for axis in "xyz"}
        self.phi1_history = {axis: {1: np.full(15, np.nan)} for axis in "xyz"}
        self.phi00_history = {axis: {0: np.full(2, np.nan)} for axis in "xyz"}
        self.phi11_history = {axis: {1: np.full(2, np.nan)} for axis in "xyz"}
        self.tof00_history = {axis: {0: np.full(2, np.nan)} for axis in "xyz"}
        self.tof11_history = {axis: {1: np.full(2, np.nan)} for axis in "xyz"}
        self.count0 = {axis: 0 for axis in "xyz"}
        self.count1 = {axis: 0 for axis in "xyz"}
        self.Autocal = {axis: 0 for axis in "xyz"}
        # Inizializza la history per ampiezza e fase raw
        self.rho_history = {axis: {s: np.full(self.n_campioni, np.nan) for s in [0,1]} for axis in "xyz"}
        self.fase_history = {axis: {s: np.full(self.n_campioni, np.nan) for s in [0,1]} for axis in "xyz"}
        # Flag di autocalibrazione per ogni asse
        self.autocal_completa = {axis: False for axis in "xyz"}
        self.autocal_misura = {axis: False for axis in "xyz"}
        self.firstCal = {axis: True for axis in "xyz"}

    def get_axes(self):
        return [axis for axis in "xyz" if self.axes_sel[axis]]

    def compute(self, measure):
        if self.CALIBRATION_COUNT < 30:
            self.parse_measure_calibration(measure)
            self.CALIBRATION_COUNT += 1
            return
        if not len(self.CALIBRATION):
            self.compute_calibration()
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
        output = {}
        for axis in self.get_axes():
            for sensor in [0,1]:
                dist0 = DIST[f"{axis}{sensor}"]
                peak = self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"]
                if len(peak) == 0:
                    self.axes_sel[axis] = False
                    continue

                mode_val = stats.mode(peak, keepdims=True).mode[0]

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
        v_air = {}
        v_sound = {}
        temp_sonica = {}
        v_air_filtered = {}
        v_sound_filtered = {}
        v_air_out = {}
        v_air_filtered_out = {}
        vmean_ax = {axis: np.nan for axis in self.get_axes()}
        scala_ax = {axis: 1.0 for axis in self.get_axes()}   # default = nessuna scala


        # INIZIALIZZA TUTTE LE CHIAVI PER OGNI ASSE!
        for axis in self.get_axes():
            v_air[axis] = np.nan
            v_sound[axis] = np.nan
            v_air_filtered[axis] = np.nan
            v_sound_filtered[axis] = np.nan
            temp_sonica[axis] = np.nan
        for i, axis in enumerate(self.get_axes()):
            g = 0
            s = 0
            f0 = 0
            f1 = 0
            tof0_0 = 0
            tof0_1 = 0
            for item in measure.get(axis, []):
                if "hdc3020" in item:
                    sensor = item["hdc3020"]
                    # Sensore 0
                    if g == 0:
                        g = g + 1
                        dist0 = DIST[f"{axis}{sensor}"]
                        T0_now = item["temp"]
                        v_sound_cal = np.sqrt(1.4 * 287 * (T0_now + 273.15))
                        tof0_0 = dist0 / v_sound_cal
                    # Sensore 1
                    else:
                        dist1 = DIST[f"{axis}{sensor}"]
                        T1_now = item["temp"]
                        Tmean = (T1_now + T0_now)/2
                        v_sound_cal = np.sqrt(1.4 * 287 * (Tmean+ 273.15))
                        tof0_1 = dist1 / v_sound_cal
                        tof0_0 = tof0_1

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
                    tof0 = self.CALIBRATION[f"tof0_{axis}{sensor}"]
                    phi0 = self.CALIBRATION[f"phi0_{axis}{sensor}"]
                    f = self.CALIBRATION[f"f_{axis}{sensor}"]
                    amp_max = rho[sample0]
                    fase_max = phi[sample0]

                    soglia_amp = 3000
                    tmp = self.rho_history[axis][sensor]
                    amp_hist = tmp[np.isfinite(tmp)]

                    if len(amp_hist) == 0:
                        mean_amp = amp_max
                    else:
                        mean_amp = np.mean(amp_hist)
                    # print(f"Mean amplitude for axis {axis}, sensor {sensor}: {mean_amp}", "amp_max:",amp_max)
                    f_amp = 0
                    if (amp_max < (mean_amp - soglia_amp)) or (amp_max > (mean_amp + soglia_amp)):
                        amp_max =  self.rho_history[axis][sensor][-1]
                        f_amp = 1


                    fixed_len_append(self.rho_history[axis][sensor], amp_max)
                    fixed_len_append(self.fase_history[axis][sensor], fase_max)

                    rho_hist = self.rho_history[axis][sensor]
                    std_rho = np.std(rho_hist[np.isnan(rho_hist) == False])

                    #Sensore 0: primo ciclo in cui si hanno i dati del sensore 0 inerenti alla misura attuale, vengono registrati
                    if s == 0:
                        #Unwrap fase
                        if abs(phi[sample0] - self.phi0_history[axis][0][-1]) >= np.pi:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi
                        # Registrazione fase solo iniziale
                        if np.isnan(self.phi00_history[axis][sensor][-1]):
                            fixed_len_append(self.phi00_history[axis][sensor], phi[sample0])
                        # Registrazione tof solo iniziale
                        if np.isnan(self.tof00_history[axis][sensor][-1]):
                            fixed_len_append(self.tof00_history[axis][sensor], tof0_0)
                        f0 = f
                        s = s + 1
                        if sample_measure==sample0 and f_amp==0:
                            delta_fase0 = phi[sample0] - self.phi00_history[axis][sensor][-1]
                            fixed_len_append(self.phi0_history[axis][0], phi[sample0])

                            if std_rho < self.std_threshold[axis]  and self.firstCal[axis] == True:
                                self.count0[axis] +=1

                                if self.count0[axis] == 30:
                                    self.Autocal[axis] = 1
                            else:
                                    self.count0[axis] = 0
                        else:
                            delta_fase0 = self.deltaphi_history[axis][sensor][-1]
                        fixed_len_append(self.deltaphi_history[axis][sensor], delta_fase0)

                    #Sensore 1: acquisizione dei dati sul sensore 1 relativi alla misura attuale
                    else:
                        if abs(phi[sample0] - self.phi1_history[axis][1][-1]) >= np.pi:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi

                        if np.isnan(self.phi11_history[axis][sensor][-1]):
                            fixed_len_append(self.phi11_history[axis][sensor], phi[sample0])

                        if np.isnan(self.tof11_history[axis][sensor][-1]):
                            fixed_len_append(self.tof11_history[axis][sensor], tof0_1)

                        f1 = f
                        if sample_measure==sample0 and f_amp==0:
                            delta_fase1 = phi[sample0] - self.phi11_history[axis][sensor][-1]
                            fixed_len_append(self.phi1_history[axis][1], phi[sample0])

                            if std_rho < self.std_threshold[axis]  and self.firstCal[axis] == True:
                                self.count1[axis] +=1
                                if self.count1[axis] == 30:
                                    self.Autocal[axis] = 1
                            else:
                                    self.count1[axis] = 0
                        else:
                            delta_fase1 = self.deltaphi_history[axis][sensor][-1]
                        fixed_len_append(self.deltaphi_history[axis][sensor], delta_fase1)

                        if self.Autocal[axis] == 1 and self.firstCal[axis]==True:
                            self.Autocal[axis] = 0
                            self.firstCal[axis]=False
                            self.autocal_misura[axis] = True
                            self.autocal_completa[axis] = True
                            # min e max e prendere media DA AGGIUNGERE
                            phi00_mean = np.mean(self.phi0_history[axis][0][-10:])
                            phi11_mean = np.mean(self.phi1_history[axis][1][-10:])

                            fixed_len_append(self.phi00_history[axis][0], phi00_mean)
                            fixed_len_append(self.tof00_history[axis][0], tof0_0)

                            fixed_len_append(self.phi11_history[axis][1], phi11_mean)
                            fixed_len_append(self.tof11_history[axis][1], tof0_1)
                            self.count0[axis] = 0
                            self.count1[axis] = 0

                            # Valutare
                            delta_fase0 = 0
                            delta_fase1 = 0
                        else:
                            self.autocal_misura[axis] = False

                        if abs(delta_fase0) > np.pi:
                                delta_fase0 = -1 * np.sign(delta_fase0) * (2 * np.pi - abs(delta_fase0))
                        if abs(delta_fase1) > np.pi:
                                delta_fase1 = -1 * np.sign(delta_fase1) * (2 * np.pi - abs(delta_fase1))

                        # --- Controllo outlier su delta_fase0 / delta_fase1 ---
                        abs0 = abs(delta_fase0)
                        abs1 = abs(delta_fase1)

                        # Soglie
                        OUTLIER_RATIO = 4.0     # quanto può essere più grande uno rispetto all'altro
                        MIN_DELTA     = 1e-3    # sotto questo valore consideriamo "rumore"

                        if np.isfinite(abs0) and np.isfinite(abs1) and max(abs0, abs1) > MIN_DELTA:
                            piccolo = min(abs0, abs1)
                            grande  = max(abs0, abs1)

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
                                if abs0 > abs1:
                                    delta_fase0 = np.sign(delta_fase0) * nuova_amp
                                else:
                                    delta_fase1 = np.sign(delta_fase1) * nuova_amp


                        # # Filtro Mario
                        if delta_fase0 * delta_fase1 > 0:  # concordi
                            alpha = min(abs(delta_fase0), abs(delta_fase1))
                            segno = 1 if delta_fase0 > 0 else -1
                            alpha *= segno
                            delta_fase0 -= alpha
                            delta_fase1 -= alpha

                        if delta_fase0 * delta_fase1 < 0:  # discordi
                            delta_phi_avg = (delta_fase0 - delta_fase1) / 2
                            delta_fase0 = delta_phi_avg
                            delta_fase1 = -delta_phi_avg

            tof00 = self.tof00_history[axis][0][-1] -  (1000000 * delta_fase0) / (2 * np.pi * f0)
            self.TOF[f"{axis}0"] = tof00

            tof11 = self.tof11_history[axis][1][-1] -  (1000000 * delta_fase1) / (2 * np.pi * f1)
            self.TOF[f"{axis}1"] = tof11

            # Calcola la velocità del flusso d'aria se entrambi i TOF sono disponibili
            if f"{axis}0" in self.TOF and f"{axis}1" in self.TOF:
                dist = [self.CALIBRATION[f"dist0_{axis}{sensor}"] for sensor in [0,1]]
                v_air[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] - dist[1] / self.TOF[f"{axis}1"])
                v_sound[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] + dist[1] / self.TOF[f"{axis}1"])

            # Aggiorna lo storico dei valori precedenti
            fixed_len_append(self.v_air_history[axis], v_air[axis])

            tmp = self.rho_history[axis][0]
            rho_hist0 = tmp[np.isfinite(tmp)]
            std_rho0 = np.std(rho_hist0) if len(rho_hist0) else np.inf

            tmp = self.rho_history[axis][1]
            rho_hist1 = tmp[np.isfinite(tmp)]
            std_rho1 = np.std(rho_hist1) if len(rho_hist1) else np.inf

            # --- MEDIA ROBUSTA ---
            hist = np.array(self.v_air_history[axis][-10:])
            hist = hist[np.isfinite(hist)]
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


            #   KALMAN
            Q = self.q_kalman

            # Kalman sempre sulla velocità grezza
            v_air_filtered[axis] = self.v_air_filter[axis].update(v_air[axis], Q)
            v_sound_filtered[axis] = self.v_sound_filter[axis].update(v_sound[axis], Q)

            # Kalman scalato e con OFFSET TOLTO
            v_air_filtered_out[axis] = scala_ax[axis] * v_air_filtered[axis] - self.v_offset[axis]

            # Per la temperatura usiamo il valore di velocità uscente dal filtro di Kalman
            temp_sonica[axis]=v_sound_filtered[axis]*v_sound_filtered[axis]/(1.4*287)-273.15

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
            v_air_filtered_out[a] = scala_ax[a] * v_air_filtered[a] - self.v_offset[a]
            v_air_out[a] = scala_ax[a] * v_air[a] - self.v_offset[a]

        if len(v_air_out) == len(self.get_axes()):
            v_air_out['timestamp'] = measure["timestamp_end"]

            for axis in self.get_axes():
                # --- KALMAN (già scalato e con offset tolto) ---
                v_air_out[f"{axis}_kalman"] = v_air_filtered_out.get(axis, np.nan)

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

            return v_air_out
