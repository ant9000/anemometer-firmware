from scipy import stats
import numpy as np

DIST = { # in micron
    'x0': 102000, 'x1': 102000,
    'y0': 102000, 'y1': 102000,
    'z0': 102000, 'z1': 102000,
}
SAMPLE = { # in samples
    'x0': 28, 'x1': 28,
    'y0': 28, 'y1': 28,
    'z0': 28, 'z1': 28,
}

def polar(i, q):
    SAMPLES = 80
    c = np.array(i) + np.array(q) * 1j
    c = np.pad(c, (0, SAMPLES), mode="constant", constant_values=0)[:SAMPLES]
    return (np.abs(c), np.angle(c, deg=False))

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
        self.clear()
        #
        self.n_campioni = n_campioni
        self.q_kalman = q_kalman

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
        self.deltaphi_history = {axis: {"0": [np.nan] * 3, "1": [np.nan] * 3} for axis in "xyz"}
        self.v_sound_history = {axis: [np.nan]*3 for axis in "xyz"}
        self.v_air_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.v_sound_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.phi0_history = {axis: {"0": [np.nan] * 15} for axis in "xyz"}
        self.phi1_history = {axis: {"1": [np.nan] * 15} for axis in "xyz"}
        self.phi00_history = {axis: {"0": [np.nan] * 2} for axis in "xyz"}
        self.phi11_history = {axis: {"1": [np.nan] * 2} for axis in "xyz"}
        self.tof00_history = {axis: {"0": [np.nan] * 2} for axis in "xyz"}
        self.tof11_history = {axis: {"1": [np.nan] * 2} for axis in "xyz"}
        self.count0 = {axis: 0 for axis in "xyz"}
        self.count1 = {axis: 0 for axis in "xyz"}
        self.Autocal = {axis: 0 for axis in "xyz"}
        # Inizializza la history per ampiezza e fase raw
        self.rho_history = {axis: {"0": [], "1": []} for axis in "xyz"}
        self.fase_history = {axis: {"0": [], "1": []} for axis in "xyz"}
        # Flag di autocalibrazione per ogni asse
        self.autocal_completa = {axis: False for axis in "xyz"}
        self.autocal_misura = {axis: False for axis in "xyz"}


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
                sample0 = stats.mode(peak, keepdims=True).mode[0] - 2
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
                    sample_measure = rho.argmax()-2
                    sample0 = self.CALIBRATION[f"sample0_{axis}{sensor}"]
                    tof0 = self.CALIBRATION[f"tof0_{axis}{sensor}"]
                    phi0 = self.CALIBRATION[f"phi0_{axis}{sensor}"]
                    f = self.CALIBRATION[f"f_{axis}{sensor}"]
                    amp_max = rho[sample0]
                    # print(amp_max, sample0)
                    fase_max = phi[sample0]
                    self.rho_history[axis][str(sensor)].append(amp_max)
                    self.fase_history[axis][str(sensor)].append(fase_max)
                    self.rho_history[axis][str(sensor)] = self.rho_history[axis][str(sensor)][-self.n_campioni:]
                    self.fase_history[axis][str(sensor)] = self.fase_history[axis][str(sensor)][-self.n_campioni:]
                    # print(self.rho_history[axis][str(sensor)], len(self.rho_history[axis][str(sensor)]))
                    # print('la')

                    # fase_hist = self.fase_history[axis][str(sensor)][-self.n_campioni:]
                    # fase_hist = [v for v in fase_hist if not np.isnan(v)]
                    # std_fase = np.std(fase_hist) if fase_hist else float('nan')
                    # std_fase = std_fase * 180 / np.pi

                    rho_hist = self.rho_history[axis][str(sensor)][-self.n_campioni:]
                    rho_hist = [v for v in rho_hist if not np.isnan(v)]
                    std_rho = np.std(rho_hist) if rho_hist else float('nan')

                    #Sensore 0: primo ciclo in cui si hanno i dati del sensore 0 inerenti alla misura attuale, vengono registrati
                    if s == 0:
                        #Unwrap fase
                        if abs(phi[sample0] - self.phi0_history[axis]["0"][-1]) >= np.pi:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi
                        # Registrazione fase solo iniziale
                        if np.isnan(self.phi00_history[axis][f"{sensor}"][-1]):
                            self.phi00_history[axis][f"{sensor}"].append(phi[sample0])
                            self.phi00_history[axis][f"{sensor}"] = self.phi00_history[axis][f"{sensor}"][1:]
                        # Registrazione tof solo iniziale
                        if np.isnan(self.tof00_history[axis][f"{sensor}"][-1]):
                            self.tof00_history[axis][f"{sensor}"].append(tof0_0)
                            self.tof00_history[axis][f"{sensor}"] = self.tof00_history[axis][f"{sensor}"][1:]
                        f0 = f
                        s = s + 1
                        if sample_measure==sample0:
                            delta_fase0 = phi[sample0] - self.phi00_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase0)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]

                            self.phi0_history[axis]["0"].append(phi[sample0])
                            self.phi0_history[axis]["0"] = self.phi0_history[axis]["0"][1:]

                            if std_rho < 40:
                                self.count0[axis] +=1

                                if self.count0[axis] == 15:
                                    self.Autocal[axis] = 1
                            else:
                                    self.count0[axis] = 0
                        else:
                            delta_fase0 = self.deltaphi_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase0)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]

                    #Sensore 1: acquisizione dei dati sul sensore 1 relativi alla misura attuale
                    else:
                        if abs(phi[sample0] - self.phi1_history[axis]["1"][-1]) >= np.pi:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi

                        if np.isnan(self.phi11_history[axis][f"{sensor}"][-1]):
                            self.phi11_history[axis][f"{sensor}"].append(phi[sample0])
                            self.phi11_history[axis][f"{sensor}"] = self.phi11_history[axis][f"{sensor}"][1:]

                        if np.isnan(self.tof11_history[axis][f"{sensor}"][-1]):
                            self.tof11_history[axis][f"{sensor}"].append(tof0_1)
                            self.tof11_history[axis][f"{sensor}"] = self.tof11_history[axis][f"{sensor}"][1:]
                        f1 = f
                        if sample_measure==sample0:
                            delta_fase1 = phi[sample0] - self.phi11_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase1)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]

                            self.phi1_history[axis]["1"].append(phi[sample0])
                            self.phi1_history[axis]["1"] = self.phi1_history[axis]["1"][1:]

                            if std_rho < 40:
                                self.count1[axis] +=1
                                if self.count1[axis] == 15:
                                    self.Autocal[axis] = 1
                            else:
                                    self.count1[axis] = 0
                        else:
                            delta_fase1 = self.deltaphi_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase1)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]

                        if self.Autocal[axis] == 1:
                            self.Autocal[axis] = 0
                            self.autocal_misura[axis] = True
                            self.autocal_completa[axis] = True
                            # min e max e prendere media DA AGGIUNGERE
                            phi00_mean = np.mean(self.phi0_history[axis]["0"][-10:])
                            phi11_mean = np.mean(self.phi1_history[axis]["1"][-10:])

                            self.phi00_history[axis]["0"].append(phi00_mean)
                            self.phi00_history[axis]["0"] = self.phi00_history[axis]["0"][1:]
                            self.tof00_history[axis]["0"].append(tof0_0)
                            self.tof00_history[axis]["0"] = self.tof00_history[axis]["0"][1:]

                            self.phi11_history[axis]["1"].append(phi11_mean)
                            self.phi11_history[axis]["1"] = self.phi11_history[axis]["1"][1:]
                            self.tof11_history[axis]["1"].append(tof0_1)
                            self.tof11_history[axis]["1"] = self.tof11_history[axis]["1"][1:]
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

            tof00 = self.tof00_history[axis]["0"][-1] -  (1000000 * delta_fase0) / (2 * np.pi * f0)
            self.TOF[f"{axis}0"] = tof00

            tof11 = self.tof11_history[axis]["1"][-1] -  (1000000 * delta_fase1) / (2 * np.pi * f1)
            self.TOF[f"{axis}1"] = tof11

            # Calcola la velocità del flusso d'aria se entrambi i TOF sono disponibili
            if f"{axis}0" in self.TOF and f"{axis}1" in self.TOF:
                dist = [self.CALIBRATION[f"dist0_{axis}{sensor}"] for sensor in [0,1]]
                v_air[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] - dist[1] / self.TOF[f"{axis}1"])
                v_sound[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] + dist[1] / self.TOF[f"{axis}1"])

            # Aggiorna lo storico dei valori precedenti
            self.v_air_history[axis].append(v_air[axis])
            self.v_air_history[axis] = self.v_air_history[axis][1:]

            #Aggiorna il filtro di Kalman
            Q = self.q_kalman
            v_air_filtered[axis] = self.v_air_filter[axis].update(v_air[axis], Q)
            v_sound_filtered[axis] = self.v_sound_filter[axis].update(v_sound[axis], Q)

            # Per la temperatura usiamo il valore di velocità uscente dal filtro di Kalman
            temp_sonica[axis]=v_sound_filtered[axis]*v_sound_filtered[axis]/(1.4*287)-273.15

        if len(v_air) == len(self.get_axes()):
            v_air['timestamp'] = measure["timestamp_end"]
            for i, axis in enumerate(self.get_axes()):
                v_air[f'{axis}_kalman'] = v_air_filtered.get(axis, np.nan)
                v_air[f"autocalibrazione_asse_{axis}"] = bool(self.autocal_completa[axis])
                v_air[f"autocalibrazione_misura_{axis}"] = self.autocal_misura[axis]
                v_air[f"temp_sonica_{axis}"] = temp_sonica[axis]
            #print(f'{v_air["timestamp"]:.4f} ', end="")
            #for i, axis in enumerate(self.get_axes()):
            #    print(f" {v_air_filtered.get(axis, np.nan):+6.4f}", end="")
            #for i, axis in enumerate(self.get_axes()):
            #    print(f" {temp_sonica[axis]:+3.2f}", end="")
            #print("")
            return v_air
