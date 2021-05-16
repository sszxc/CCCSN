# Author: Xuechao Zhang
# Date: May 16th, 2021
# Description: 粒子群优化

import numpy as np
import random
import matplotlib.pyplot as plt

class PSO:
    def __init__(self, parameters, bundary_u, bundary_l, fitness_function):
        '''
        parameters: 待优化参数
        bundary: 每个参数的上下界
        fitness_function: 目标函数
        '''
        self.w = 0.6  # 惯性权重
        self.c1 = self.c2 = 2 # 学习因子
        self.population = 20  # 粒子数量
        self.dim = len(parameters)  # 维度
        self.max_steps = 100  # 最大迭代次数
        self.bundary_lower = bundary_l
        self.bundary_upper = bundary_u
        x_init = []
        for i in range(self.population):
            x_init.append([np.random.uniform(self.bundary_lower[i],
                                             self.bundary_upper[i])
                           for i in range(self.dim)])
        self.x = np.array(x_init)  # 随机初始位置
        self.v_max = 1  # 随机初始速度上限
        self.v_min = -1  # 随机初始速度下限
        self.v = np.random.uniform(
            self.v_max, self.v_min, (self.population, self.dim))  # 随机初始速度
        self.cal_fitness = fitness_function  # 目标函数指针
        self.fitness = [self.cal_fitness(self.x[i]) for i in range(self.population)]
        self.p_bestx = self.x.copy()  # 个体最佳位置
        self.g_bestx = self.x[np.argmax(self.fitness)]  # 全局最佳位置
        self.p_bestf = self.fitness.copy()  # 个体最优适应度
        self.g_bestf = np.max(self.fitness)  # 全局最佳适应度

    def update(self):
        for i in range(self.population):
            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
            # 更新速度和位置
            self.v[i] = self.w*self.v[i] \
                + self.c1*r1*(self.p_bestx[i]-self.x[i]) \
                + self.c2*r2*(self.g_bestx-self.x[i])
            for j in range(self.dim):
                if self.v[i][j] < self.v_min:
                    self.v[i][j] = self.v_min
                elif self.v[i][j] > self.v_max:
                    self.v[i][j] = self.v_max
            self.x[i] = self.v[i] + self.x[i]
            for j in range(self.dim):
                if self.x[i][j] > self.bundary_upper[j]:
                    self.x[i][j] = self.bundary_upper[j]
                elif self.x[i][j] < self.bundary_lower[j]:
                    self.x[i][j] = self.bundary_lower[j]

            self.fitness[i] = self.cal_fitness(self.x[i])
            # 更新达到更好成绩的个体
            if self.fitness[i]>self.p_bestf[i]:
                self.p_bestx[i] = self.x[i]
                self.p_bestf[i] = self.fitness[i]
                if self.fitness[i]>self.g_bestf:
                    self.g_bestx = self.x[i]
                    self.g_bestf = self.fitness[i]

    def evolve(self):
        history = []
        plt.figure().canvas.set_window_title('Evolve')  # 窗口名
        plt.ion()

        for step in range(self.max_steps):
            self.update()
            print(str(step)+' best score:'+str(self.g_bestf))
            history.append(self.g_bestf)
            plt.clf()
            plt.plot(range(step+1), history)
            plt.rcParams['font.sans-serif'] = 'SimHei'  # 显示中文不乱码
            plt.xlabel(u"迭代次数")  # X轴标签
            plt.ylabel(u"适应度")  # Y轴标签
            plt.title(u"迭代过程")  # 标题
            plt.pause(0.0001)
            plt.ioff()

        plt.show()

def Euclidean_Distance(x, y=np.array([5, 6, 7, 8])):
    '''
    欧氏距离
    '''
    return np.linalg.norm(x-y)


if __name__ == "__main__":
    parameters = [1, 2, 3, 4]
    bundary_lower = [-10, -10, -50, -5]
    bundary_upper = [10, 20, 30, 40]
    pso = PSO(parameters, bundary_upper, bundary_lower, Euclidean_Distance)

    pso.evolve()
