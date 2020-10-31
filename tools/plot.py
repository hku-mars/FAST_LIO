# import matplotlib
# matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt

fig, axs = plt.subplots(5)
lab_pre = ['', 'pre-x', 'pre-y', 'pre-z']
lab_out = ['', 'out-x', 'out-y', 'out-z']
plot_ind = range(7,10)
a_pre=np.loadtxt('Log/mat_pre.txt')
a_out=np.loadtxt('Log/mat_out.txt')
time=a_pre[:,0]
axs[0].set_title('Attitude')
axs[1].set_title('Translation')
axs[2].set_title('Velocity')
axs[3].set_title('bg')
axs[4].set_title('ba')
for i in range(1,4):
    for j in range(5):
        axs[j].plot(time, a_pre[:,i+j*3],'.-', label=lab_pre[i])
        axs[j].plot(time, a_out[:,i+j*3],'.-', label=lab_out[i])

for i in range(5):
    # axs[i].set_xlim(386,389)
    axs[i].grid()
    axs[i].legend()


# #### Draw IMU data
# fig, axs = plt.subplots(2)
# imu=np.loadtxt('Log/imu_0.txt')
# time=imu[:,0]
# axs[0].set_title('Gyroscope')
# axs[1].set_title('Accelerameter')
# lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
# lab_2 = ['acc-x', 'acc-y', 'acc-z']
# for i in range(3):
#     # if i==1:
#     axs[0].plot(time, imu[:,i+1],'.-', label=lab_1[i])
#     axs[1].plot(time, imu[:,i+4],'.-', label=lab_2[i])
# for i in range(2):
#     axs[i].set_xlim(386,389)
#     axs[i].grid()
#     axs[i].legend()

# #### Draw time calculation
# fig = plt.figure()
# font1 = {'family' : 'Times New Roman',
# 'weight' : 'normal',
# 'size'   : 12,
# }
# c="red"
# a_out1=np.loadtxt('Log/mat_out_time_indoor1.txt')
# a_out2=np.loadtxt('Log/mat_out_time_indoor2.txt')
# a_out3=np.loadtxt('Log/mat_out_time_outdoor.txt')
# # n = a_out[:,1].size
# # time_mean = a_out[:,1].mean()
# # time_se   = a_out[:,1].std() / np.sqrt(n)
# # time_err  = a_out[:,1] - time_mean
# # feat_mean = a_out[:,2].mean()
# # feat_err  = a_out[:,2] - feat_mean
# # feat_se   = a_out[:,2].std() / np.sqrt(n)
# ax1 = fig.add_subplot(111)
# ax1.set_ylabel('Effective Feature Numbers',font1)
# ax1.boxplot(a_out1[:,2], showfliers=False, positions=[0.9])
# ax1.boxplot(a_out2[:,2], showfliers=False, positions=[1.9])
# ax1.boxplot(a_out3[:,2], showfliers=False, positions=[2.9])
# ax1.set_ylim([0, 3000])

# ax2 = ax1.twinx()
# ax2.spines['right'].set_color('red')
# ax2.set_ylabel('Compute Time (ms)',font1)
# ax2.yaxis.label.set_color('red')
# ax2.tick_params(axis='y', colors='red')
# ax2.boxplot(a_out1[:,1]*1000, showfliers=False, positions=[1.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out2[:,1]*1000, showfliers=False, positions=[2.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out3[:,1]*1000, showfliers=False, positions=[3.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.set_xlim([0.5, 3.5])
# ax2.set_ylim([0, 100])

# plt.xticks([1,2,3], ('Outdoor Scene', 'Indoor Scene 1', 'Indoor Scene 2'))
# # print(time_se)
# # print(a_out3[:,2])
plt.grid()
plt.savefig("Log/time.pdf", dpi=1200)
plt.show()
