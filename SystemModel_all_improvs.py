import math
import numpy as np

np.seterr(divide='ignore', invalid='ignore')

# IMPLEMENTAÇÃO DAS DUAS MELHORIAS


class SystemModel(object):
    def __init__(self):
        self.user_num = 10              # 用户数
        self.beta_0 = 10 ** -5          # 1m参考距离的信道增益
        self.H = 100                    # 无人机的飞行高度
        self.B0 = 10 ** 7               # 分配给用户的带宽
        self.derta = 10 ** -5           # 无人机处的噪声功率
        self.P_user = 0.5               # 用户的发射功率
        self.C_user = 800               # 用户处理1bit数据需要的CPU计算周期数
        self.C_uav = 10 ** 3            # 无人机处理1bit数据需要的CPU计算周期数
        self.f_user = 1 * (10 ** 9)     # 本机计算资源
        self.f_uav = 3 * (10 ** 9)      # 分配给无人机的计算资源
        self.K_user = 10 ** -27         # 用户CPU的电容系数
        self.K_uav = 10 ** -28          # 无人机CPU的电容系数
        self.w = 0.75                   # 能耗与时延占比

        self.P_uav = 0.5
        self.theta = 0.3  # 0.1 0.3 0.6

    def failure_prob(self, d):
        if d < 400:
            p_falha = 0
        elif d < 7000:
            p_falha = 0.1
        elif d < 8200:
            p_falha = 0.3
        else:
            p_falha = 0.5

        return p_falha

    def failure_prob_uav(self, L):
        L = L / (10 ** 6)
        if L < 3:
            p_falha = 0
        elif L < 7:
            p_falha = 0.1
        elif L < 9:
            p_falha = 0.3
        else:
            p_falha = 0.5

        return p_falha

    def System_Model(self, user_location, L, uav_location):
        L_user = []
        L_uav = []
        channel_gain = []
        # r = []
        aerfa = []
        e_up = []
        e_user = []
        e_uav = []
        e_local = []
        e_load = []
        t_up = []
        t_user = []
        t_uav = []
        t_local = []
        t_load = []
        Pro_load = []
        Pro_local = []
        Pro_load_sum = []
        Pro_local_sum = []

        # My arrays
        d = []          # dintancias
        r_up = []       # transmissão uplink
        r_down = []     # transmissão downlink
        t_down = []     # delay do downlink
        e_down = []     # consumo do downlink
        t_merge = []    # tempo para mergear
        e_merge = []    # energia para mergear
        t_up_total = []
        e_up_total = []
        t_uav_total = []
        e_uav_total = []
        t_down_total = []
        e_down_total = []

        self.user_location = user_location
        self.uav_location = uav_location
        self.L = L

        # 通信模型
        # Ganho do Canal - Uplink e Downlink
        for i in range(self.user_num):
            d_ = (self.uav_location[i][0] - self.user_location[i][0]) ** 2 + \
                 (self.uav_location[i][1] - self.user_location[i][1])
            d.append(d_)
            gain = self.beta_0 / (self.H ** 2 + d_)
            channel_gain.append(gain)

        channel_gain = np.array(channel_gain).reshape(self.user_num, 1)

        # Transmissão uplink
        for i in range(self.user_num):
            r_ = self.B0 * math.log(1 + self.P_user *
                                    channel_gain[i] / (self.derta ** 2), 2)
            r_up.append(r_)

        r_up = np.array(r_up).reshape(self.user_num, 1)

        # Transmissão downlink
        for i in range(self.user_num):
            r_ = self.B0 * math.log(1 + self.P_uav *
                                    channel_gain[i] / (self.derta ** 2), 2)
            r_down.append(r_)

        r_down = np.array(r_down).reshape(self.user_num, 1)

        # 计算卸载比例
        # T = max(t_user, t_up + t_uav)
        # 要得到 min(T),则需要把 t_user = t_up +t_uav 时的aerfa值作为最优卸载比例
        for i in range(self.user_num):
            aerfa_ = (self.C_user * self.f_uav * r_up[i]) \
                / (self.C_user * self.f_uav * r_up[i] + self.C_uav * self.f_user * r_up[i] + self.f_user * self.f_uav)
            L_user_ = (1 - aerfa_) * self.L[i]
            L_uav_ = aerfa_ * self.L[i]
            aerfa.append(aerfa_)
            L_user.append(L_user_)
            L_uav.append(L_uav_)

        aerfa = np.array(aerfa).reshape(self.user_num, 1)
        L_user = np.array(L_user).reshape(self.user_num, 1)
        L_uav = np.array(L_uav).reshape(self.user_num, 1)

        '''Modelo de Comunicação'''
        # Tempo e Energia - Uplink
        for i in range(self.user_num):
            p_falha_uav = self.failure_prob(d[i])
            t_up_ = L_uav[i] / r_up[i]
            t_up_total_ = t_up_ * ((1 - p_falha_uav) + 1.5 * (p_falha_uav))
            e_up_energy = self.P_user * t_up_
            e_up_total_ = self.P_user * t_up_total_
            t_up.append(t_up_)
            e_up.append(e_up_energy)
            t_up_total.append(t_up_total_)
            e_up_total.append(e_up_total_)

        t_up = np.array(t_up).reshape(self.user_num, 1)
        e_up = np.array(e_up).reshape(self.user_num, 1)
        t_up_total = np.array(t_up_total).reshape(self.user_num, 1)
        e_up_total = np.array(e_up_total).reshape(self.user_num, 1)

        t_up_sum = sum(t_up)
        e_up_sum = sum(e_up)
        t_up_total_sum = sum(t_up_total)
        e_up_total_sum = sum(e_up_total)

        # Tempo e Energia - Downlink
        for i in range(self.user_num):
            p_falha_uav = self.failure_prob(d[i])
            t_down_ = self.theta * L_uav[i] / r_down[i]
            t_down_total_ = t_down_ * ((1 - p_falha_uav) + 1.5 * (p_falha_uav))
            e_down_energy = self.P_uav * t_down_
            e_down_total_ = self.P_user * t_down_total_
            t_down.append(t_down_)
            e_down.append(e_down_energy)
            t_down_total.append(t_down_total_)
            e_down_total.append(e_down_total_)

        t_down = np.array(t_down).reshape(self.user_num, 1)
        e_down = np.array(e_down).reshape(self.user_num, 1)
        t_down_total = np.array(t_down_total).reshape(self.user_num, 1)
        e_down_total = np.array(e_down_total).reshape(self.user_num, 1)

        t_down_sum = sum(t_down)
        e_down_sum = sum(e_down)
        t_down_total_sum = sum(t_down_total)
        e_down_total_sum = sum(e_down_total)

        '''Modelo computacional'''
        'Latência total de computação local e consumo de energia de computação'
        for i in range(self.user_num):
            t_local_ = L[i] * self.C_user / self.f_user
            e_local_ = self.K_user * (self.f_user ** 3) * t_local_
            t_local.append(t_local_)
            e_local.append(e_local_)

        t_local = np.array(t_local).reshape(self.user_num, 1)
        e_local = np.array(e_local).reshape(self.user_num, 1)

        t_local_sum = sum(t_local)
        e_local_sum = sum(e_local)

        'Descarregamento parcial para o UAV para latência de computação e consumo de energia de computação'
        # Parte do calculo localmente
        for i in range(self.user_num):
            t_user_ = L_user[i] * self.C_user / self.f_user
            e_user_ = self.K_user * (self.f_user ** 3) * t_user_
            t_user.append(t_user_)
            e_user.append(e_user_)

        t_user = np.array(t_user).reshape(self.user_num, 1)
        e_user = np.array(e_user).reshape(self.user_num, 1)

        t_user_sum = sum(t_user)
        e_user_sum = sum(e_user)

        # Calculo do custo de merge
        for i in range(self.user_num):
            t_merge_ = self.theta * L_user[i] * self.C_user / self.f_user
            e_merge_ = self.K_user * (self.f_user ** 3) * t_merge_
            t_merge.append(t_merge_)
            e_merge.append(e_merge_)

        t_merge = np.array(t_merge).reshape(self.user_num, 1)
        e_merge = np.array(e_merge).reshape(self.user_num, 1)

        t_merge_sum = sum(t_merge)
        e_merge_sum = sum(e_merge)

        # Parte da descarga para a computação de drones
        for i in range(self.user_num):
            p_falha_uav = self.failure_prob_uav(L_uav[i])
            t_uav_ = L_uav[i] * self.C_uav / self.f_uav
            t_uav_total_ = t_uav_ * ((1 - p_falha_uav) + 1.5 * p_falha_uav)
            e_uav_ = self.K_uav * (self.f_uav ** 3) * t_uav_
            e_uav_total_ = self.K_uav * (self.f_uav ** 3) * t_uav_total_
            t_uav.append(t_uav_)
            e_uav.append(e_uav_)
            t_uav_total.append(t_uav_total_)
            e_uav_total.append(e_uav_total_)

        t_uav = np.array(t_uav).reshape(self.user_num, 1)
        e_uav = np.array(e_uav).reshape(self.user_num, 1)
        t_uav_total = np.array(t_uav_total).reshape(self.user_num, 1)
        e_uav_total = np.array(e_uav_total).reshape(self.user_num, 1)

        t_uav_sum = sum(t_uav)
        e_uav_sum = sum(e_uav)
        t_uav_total_sum = sum(t_uav_total)
        e_uav_total_sum = sum(e_uav_total)

        # 上传的总能耗和总时延 - O consumo total de energia e tempo
        for i in range(self.user_num):
            t_load_ = max(
                t_user[i], (t_up_total[i] + t_uav_total[i] + t_down_total[i] + t_merge[i]))
            e_load_ = e_up_total[i] + e_user[i] + \
                e_uav_total[i] + e_down_total[i] + e_merge[i]
            t_load.append(t_load_)
            e_load.append(e_load_)

        t_load = np.array(t_load).reshape(self.user_num, 1)
        e_load = np.array(e_load).reshape(self.user_num, 1)

        t_load_sum = sum(t_load)
        e_load_sum = sum(e_load)

        delay_power_sum = {
            "t_load_sum": sum(t_load_sum),
            "e_load_sum": sum(e_load_sum),

            "t_uav_sum": sum(t_uav_sum),
            "e_uav_sum": sum(e_uav_sum),

            "t_uav_total_sum": sum(t_uav_total_sum),
            "e_uav_total_sum": sum(e_uav_total_sum),

            "t_user_sum": sum(t_user_sum),
            "e_user_sum": sum(e_user_sum),

            "t_local_sum": sum(t_local_sum),
            "e_local_sum": sum(e_local_sum),

            "t_up_sum": sum(t_up_sum),
            "e_up_sum": sum(e_up_sum),

            "t_up_total_sum": sum(t_up_total_sum),
            "e_up_total_sum": sum(e_up_total_sum),

            "t_down_sum": sum(t_down_sum),
            "e_down_sum": sum(e_down_sum),

            "t_down_total_sum": sum(t_down_total_sum),
            "e_down_total_sum": sum(e_down_total_sum),

            "t_merge_sum": sum(t_merge_sum),
            "e_merge_sum": sum(e_merge_sum),
        }

        delay_power_by_user = {
            "t_load": t_load,
            "e_load": e_load,

            "t_uav": t_uav,
            "e_uav": e_uav,

            "t_uav_total": t_uav_total,
            "e_uav_total": e_uav_total,

            "t_user": t_user,
            "e_user": e_user,

            "t_local": t_local,
            "e_local": e_local,

            "t_up": t_up,
            "e_up": e_up,

            "t_up_total": t_up_total,
            "e_up_total": e_up_total,

            "t_down": t_down,
            "e_down": e_down,

            "t_down_total": t_down_total,
            "e_down_total": e_down_total,

            "t_merge": t_merge,
            "e_merge": e_merge,
        }

        # 能耗与时延问题总和
        for i in range(self.user_num):
            Pro_load_ = self.w * e_load[i] + (1 - self.w) * t_load[i]
            Pro_local_ = self.w * e_local[i] + (1 - self.w) * t_local[i]
            if self.L[i] < 3 * 10 ** 6:
                Pro_load_ = (1 + 0.3) * Pro_load_
            elif self.L[i] > 8 * 10 ** 6:
                Pro_local_ = (1 + 0.3) * Pro_load_
            Pro_load.append(Pro_load_)
            Pro_local.append(Pro_local_)

        Pro_load = np.array(Pro_load).reshape(self.user_num, 1)
        Pro_local = np.array(Pro_local).reshape(self.user_num, 1)
        Pro_load_sum.append(sum(Pro_load))
        Pro_local_sum.append(sum(Pro_local))

        return Pro_load, Pro_local, delay_power_sum, delay_power_by_user
