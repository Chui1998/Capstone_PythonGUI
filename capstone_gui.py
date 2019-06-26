import math
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import *
from tkinter import IntVar, StringVar, ttk

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.lines import Line2D

# 全局变量定义区
global_rpm = 0
fig = 0
ax = 0


class Application(Frame):
    # 初始化
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.serial_status_show = StringVar()
        self.rpm_text = IntVar()  # 测量转速
        self.set_rpm_show = IntVar()  # 显示STM32设定转速
        self.Kp_show = IntVar()  # 显示STM32比例参数
        self.Ki_show = IntVar()  # 显示STM32积分参数
        self.running_status_var = StringVar()  # 运转状态
        self.side_status_var = StringVar()  # 转向
        self.stable_var = StringVar()  # 稳定状态
        self.error_var=IntVar()#稳态误差
        self.side_status = True  # false 为 L 也即顺时针，此时继电器不工作
        self.running_status = False
        self.serial_status = False
        self.stable_status = False
        self.side_status_var.set('逆时针')
        self.serial_status_show.set('无')
        self.running_status_var.set('关')
        self.stable_var.set('不稳定')

        self.create_widgets()
        self.pack(expand=YES, fill=BOTH)
        self.window_init()

    # 窗口初始化
    def window_init(self):
        self.master.title('直流电机控制系统配置')
        self.master.geometry("{}x{}".format(800, 400))
        self.master.resizable(0, 0)

    # 获取选中串口名字并建立串口多线程
    def serial_connect_funtion(self):
        temp = str(self.serial_port_box.get())
        self.select_serial = temp[:temp.find(' - ')]  # 获取串口号文本
        self.connect_serial()
        self.thread = threading.Thread(
            target=self.listen_serial, args=())  # 创建串口多线程
        self.thread.start()  # 开始串口多线程

    # 连接选中串口
    def connect_serial(self):
        while True:  # 尝试连接串口，直到连接成功或者连接失败
            try:
                self.ser = serial.Serial(self.select_serial, 921600)
                print('open', self.select_serial, 'success')
                self.serial_status = True
                self.serial_status_show.set(self.select_serial)
                cmd = [0xA8, 0x0D, 0x0A]
                self.ser.write(cmd)
                self.update()
                break
                pass
            except serial.serialutil.SerialException:
                if len(self.select_serial) == False:
                    print('no serial port can use')
                else:
                    print(self.select_serial, 'is already occupied')
                self.serial_status_show.set('无')
                self.update()
                break
                pass

    # 监听串口
    def listen_serial(self):
        global global_rpm
        times=0
        if self.serial_status == True:
            while True:
                if self.serial_status == False:
                    break
                    pass
                try:
                    b = self.ser.read(1)
                    h = b.hex()
                    if h == 'f1':  # 修改运行状态为运行
                        self.running_status = True
                        self.running_status_var.set('运行')
                    elif h == 'f0':  # 修改运行状态为停止
                        self.running_status = False
                        self.running_status_var.set('停止')
                    elif h == 'f3':  # 修改运行方向为顺时针
                        self.side_status = False
                        self.side_status_var.set('逆时针')
                    elif h == 'f2':  # 修改运行方向为逆时针
                        self.side_status = True
                        self.side_status_var.set('顺时针')
                    elif h == 'f6':  # 接收转速数据，转速数据处理
                        power = 1000
                        rpm = 0
                        for i in range(4):
                            rpm = rpm + \
                                ord(self.ser.read(1))*power
                            power = power/10
                        print(rpm)
                        self.rpm_text.set(rpm)
                        global_rpm = rpm
                        if (abs(rpm-self.set_rpm))<=self.set_rpm*0.1:#判断是否稳定
                            times=times+1
                            stable_status=True
                            self.stable_var.set('稳定')
                            if self.max_rpm<rpm:
                                self.max_rpm=rpm
                            elif self.min_rpm>rpm:
                                self.min_rpm=rpm
                            error=self.max_rpm-self.min_rpm
                            self.error_var.set(error)
                            if times==50:
                                self.max_rpm=self.min_rpm=self.set_rpm
                                times=0
                        else:
                            self.stable_var.set('不稳定')
                            self.max_rpm=self.min_rpm=self.set_rpm
                            error=0
                            self.error_var.set(error)
                    elif h == 'f5':  # 停转了
                        rpm = 0
                        print(rpm)
                        self.rpm_text.set(rpm)
                        global_rpm = rpm
                    elif h == 'f4':  # 读取所有数据（串口连接建立后自动进行的）
                        power = 1000
                        self.set_rpm = 0
                        self.running_status = (ord(self.ser.read(1)))
                        self.side_status = (ord(self.ser.read(1)))
                        for i in range(4):
                            self.set_rpm = self.set_rpm+ord(self.ser.read(1))*power
                            power = power/10
                        self.Kp_show.set(ord(self.ser.read(1)))
                        self.Ki_show.set(ord(self.ser.read(1)))
                        self.set_rpm_show.set(self.set_rpm)
                        if self.running_status == True:
                            self.running_status_var.set('运行')
                        else:
                            self.running_status_var.set('停止')
                        if self.side_status == True:
                            self.side_status_var.set('顺时针')
                        else:
                            self.side_status_var.set('逆时针')
                        self.max_rpm=self.min_rpm=self.set_rpm
                        print('ok')
                    self.update()

                except serial.serialutil.SerialException:  # 串口失联error
                    print('error')
                    self.serial_status_show.set('无')
                    self.update()
                    break
                    pass

    # 断开串口连接
    def disconnect_serial(self):
        if self.serial_status == True:  # 是否已经连接上了串口，避免self.ser.is_open出错
            if self.ser.is_open:  # 串口是否开启
                self.serial_status = False
                self.ser.close()
                self.serial_status_show.set('无')
                self.update()
                print('close', self.select_serial, 'success')
        else:
            print('no serial is connected')

    # 发送启停命令
    def on_off_running(self):
        if self.serial_status == True:  # 是否已经连接上了串口
            self.running_status = not self.running_status  # 改变运行状态
            if self.running_status == True:  # 启动运转
                cmd = [0xA3, 0x01, 0x0D, 0x0A]
                self.ser.write(cmd)
                self.running_status_var.set('开')
                print('open running')
            else:  # 关闭运转
                cmd = [0xA3, 0x00, 0x0D, 0x0A]
                self.ser.write(cmd)
                self.running_status_var.set('关')
                print('close running')
        else:
            print('no serial can use')

    # 发送换向命令
    def left_right(self):
        if self.serial_status == True:
            self.side_status = not self.side_status
            if self.side_status == True:
                self.side_status_var.set('顺时针')
                cmd = [0xA4, 0x01, 0x0D, 0x0A]
                self.ser.write(cmd)
                print('right')
            else:
                self.side_status_var.set('逆时针')
                cmd = [0xA4, 0x00, 0x0D, 0x0A]
                print('left')
                self.ser.write(cmd)
        else:
            print('no serial can use')

    # 发送数据到SMT32
    def update_data2STM(self):
        if self.serial_status == True:  # 是否已经连接上了串口
            # 修改设定转速
            if self.set_rpm_entry.get() == '':  # 如果设置转速输入框为空，则不发送修改设定转速命令
                print('no set rpm')
            else:
                temp = int(self.set_rpm_entry.get())
                self.set_rpm_show.set(temp)
                cmd = [0xA5,  int(temp / 1000),
                       int(temp % 1000/100), int(temp % 100/10),
                       int(temp % 10), 0x0D, 0x0A]
                self.ser.write(cmd)
                print(cmd)
            # 修改P
            if self.set_Kp_entry.get() == '':
                print('no Kp data')
            else:
                temp = int(self.set_Kp_entry.get())
                self.Kp_show.set(temp)
                cmd = [0xA0, int(temp), 0x0D, 0x0A]
                time.sleep(0.25)
                self.ser.write(cmd)
                print(cmd)
            # 修改I
            if self.set_Ki_entry.get() == '':
                print('no Ki')
            else:
                temp = int(self.set_Ki_entry.get())
                self.Ki_show.set(temp)
                cmd = [0xA1, int(temp), 0x0D, 0x0A]
                time.sleep(0.25)
                self.ser.write(cmd)
                print(cmd)
            print('updating data 2 STM')
            self.update()
        else:
            print('no serial can use')

    # 框架2 串口、参数、按钮
    def fm2_function(self):
        self.fm2 = Frame(self)
        self.fm2_serial = Frame(self.fm2)  # 串口部分
        self.fm2_parameter = Frame(self.fm2)  # 参数部分
        self.fm2_button = Frame(self.fm2)  # 按钮部分

        self.fm2_parameter_set_rpm = Frame(self.fm2_parameter)  # 设定转速
        self.fm2_parameter_Kp = Frame(self.fm2_parameter)  # 比例参数
        self.fm2_parameter_Ki = Frame(self.fm2_parameter)  # 积分参数
        self.fm2_parameter_running = Frame(self.fm2_parameter)  # 运转状态
        self.fm2_parameter_side = Frame(self.fm2_parameter)  # 运转方向
        self.fm2_parameter_read_rpm = Frame(self.fm2_parameter)  # 读取测量转速
        self.fm2_parameter_stable = Frame(self.fm2_parameter)  # 稳定状态
        self.fm2_parameter_error = Frame(self.fm2_parameter)  # 稳态误差

        # 串口文本
        self.serial_title = Label(self.fm2_serial, text='机器号:',
                                  font=('微软雅黑', 14),
                                  fg='black')
        self.serial_title.pack(side=LEFT)
        # 串口选择下拉框
        self.serial_port_box = ttk.Combobox(
            self.fm2_serial, textvariable=1, width='40')
        self.serial_port_box["values"] = list(
            serial.tools.list_ports.comports())
        self.serial_port_box.current()
        self.serial_port_box.bind("<<serial_port_box_selected>>")
        self.serial_port_box.pack(side=LEFT)
        # 串口连接按钮
        self.serial_connect_button = Button(self.fm2_serial, text='连接',
                                            width='15', height='1',
                                            font=('微软雅黑', 8),
                                            command=self.serial_connect_funtion)
        self.serial_connect_button.pack(side=LEFT)
        # 串口断开连接按钮
        self.serial_disconnect_button = Button(self.fm2_serial, text='断开连接',
                                               width='15', height='1',
                                               font=('微软雅黑', 8),
                                               command=self.disconnect_serial)
        self.serial_disconnect_button.pack(side=LEFT)
        self.serial_connect_status_title = Label(self.fm2_serial, text='连接状态：',
                                                 font=('微软雅黑', 14),
                                                 fg='black')
        self.serial_connect_status_title.pack(side=LEFT)
        self.serial_connect_status_name = Label(self.fm2_serial,
                                                textvariable=self.serial_status_show,
                                                font=('微软雅黑', 14), fg='black')
        self.serial_connect_status_name.pack(side=LEFT)
        self.fm2_serial.pack(side=TOP)

        # 设定转速、PID参数
        # 设定转速文本
        self.set_rpm_title = Label(self.fm2_parameter_set_rpm,
                                   text='修改设定转速：',
                                   font=('微软雅黑', 14),
                                   fg='black')
        self.set_rpm_title.pack(side=LEFT)
        # 修改设定转速文本
        self.set_rpm_entry = Entry(self.fm2_parameter_set_rpm)
        self.set_rpm_entry.pack(side=LEFT)
        # 设定转速值
        self.read_set_rpm_title = Label(self.fm2_parameter_set_rpm,
                                        text='读取设定转速：',
                                        font=('微软雅黑', 14),
                                        fg='black')
        self.read_set_rpm_title.pack(side=LEFT)
        self.read_set_rpm_show = Label(self.fm2_parameter_set_rpm,
                                       textvariable=self.set_rpm_show,
                                       font=('微软雅黑', 14),
                                       fg='black')
        self.read_set_rpm_show.pack(side=LEFT)

        self.fm2_parameter_set_rpm.pack(side=TOP, anchor='nw')

        # 设定比例参数
        self.set_Kp_title = Label(self.fm2_parameter_Kp, text='修改比例参数：',
                                  font=('微软雅黑', 14), fg='black')
        self.set_Kp_title.pack(side=LEFT)
        self.set_Kp_entry = Entry(self.fm2_parameter_Kp)
        self.set_Kp_entry.pack(side=LEFT)
        self.read_Kp_title = Label(self.fm2_parameter_Kp, text='读取比例参数：',
                                   font=('微软雅黑', 14),
                                   fg='black')
        self.read_Kp_title.pack(side=LEFT)
        self.read_Kp = Label(self.fm2_parameter_Kp, textvariable=self.Kp_show,
                             font=('微软雅黑', 14),
                             fg='black')
        self.read_Kp.pack(side=LEFT)
        self.fm2_parameter_Kp.pack(side=TOP, anchor='nw')

        # 设定积分参数
        self.set_Ki_title = Label(self.fm2_parameter_Ki, text='修改积分参数：',
                                  font=('微软雅黑', 14), fg='black')
        self.set_Ki_title.pack(side=LEFT)
        self.set_Ki_entry = Entry(self.fm2_parameter_Ki)
        self.set_Ki_entry.pack(side=LEFT)
        self.read_Ki_title = Label(self.fm2_parameter_Ki, text='读取积分参数：',
                                   font=('微软雅黑', 14),
                                   fg='black')
        self.read_Ki_title.pack(side=LEFT)
        self.read_Ki = Label(self.fm2_parameter_Ki, textvariable=self.Ki_show,
                             font=('微软雅黑', 14),
                             fg='black')
        self.read_Ki.pack(side=LEFT)
        self.fm2_parameter_Ki.pack(side=TOP, anchor='nw')

        # 运行状态
        self.running_status_title = Label(self.fm2_parameter_running, text='运行状态：',
                                          font=('微软雅黑', 14),
                                          fg='black')
        self.running_status_title.pack(side=LEFT, anchor='nw')
        self.running_status_vartitle = Label(self.fm2_parameter_running,
                                             textvariable=self.running_status_var,
                                             font=('微软雅黑', 14),
                                             fg='black')
        self.running_status_vartitle.pack(side=LEFT, anchor='nw')
        self.fm2_parameter_running.pack(side=TOP, anchor='nw')

        # 转向
        self.side_status_title = Label(self.fm2_parameter_side, text='转向：',
                                       font=('微软雅黑', 14),
                                       fg='black')
        self.side_status_title.pack(side=LEFT, anchor='nw')
        self.side_status_vartitle = Label(self.fm2_parameter_side,
                                          textvariable=self.side_status_var,
                                          font=('微软雅黑', 14),
                                          fg='black')
        self.side_status_vartitle.pack(side=RIGHT, anchor='nw')
        self.fm2_parameter_side.pack(side=TOP, anchor='nw')

        # 测量转速
        self.read_rpm_title = Label(self.fm2_parameter_read_rpm,
                                    text='测量转速：', font=('微软雅黑', 14),
                                    fg='black')
        self.read_rpm_title.pack(side=LEFT, anchor='nw')
        self.read_rpm = Label(self.fm2_parameter_read_rpm,
                              textvariable=self.rpm_text,
                              font=('微软雅黑', 14), fg='black')
        self.read_rpm.pack(side=LEFT, anchor='nw')
        self.fm2_parameter_read_rpm.pack(side=TOP, anchor='nw')

        # 稳定状态
        self.stable_title = Label(self.fm2_parameter_stable, 
                                    text='稳定状态：', font=('微软雅黑', 14), 
                                    fg='black')
        self.stable_title.pack(side=LEFT, anchor='nw')
        self.stable_show = Label(self.fm2_parameter_stable, 
                                textvariable=self.stable_var, 
                                font=('微软雅黑', 14), fg='black')
        self.stable_show.pack(side=LEFT,anchor='nw')
        self.fm2_parameter_stable.pack(side=TOP,anchor='nw')

        #稳态误差
        self.error_title=Label(self.fm2_parameter_error, 
                                    text='稳定误差：', font=('微软雅黑', 14), 
                                    fg='black')
        self.error_title.pack(side=LEFT,anchor='nw')
        self.error_show = Label(self.fm2_parameter_error, 
                                textvariable=self.error_var, 
                                font=('微软雅黑', 14), fg='black')
        self.error_show.pack(side=LEFT,anchor='nw')
        self.fm2_parameter_error.pack(side=TOP,anchor='nw')

        self.fm2_parameter.pack(side=LEFT, anchor='nw')

        # 按钮部分 启停按钮 正转/反转按钮 发送参数到STM按钮
        self.on_off_button = Button(self.fm2_button, text='启动/停止', width='20', height='1',
                                    font=('微软雅黑', 10), command=self.on_off_running)
        self.on_off_button.pack(side=TOP)
        self.left_right_button = Button(self.fm2_button, text='正转/反转', width='20', height='1',
                                        font=('微软雅黑', 10), command=self.left_right)
        self.left_right_button.pack(side=TOP)
        self.update_data2STM_button = Button(self.fm2_button, text='发送数据到STM32',
                                             width='20', height='1', font=('微软雅黑', 10),
                                             command=self.update_data2STM)
        self.update_data2STM_button.pack(side=TOP)
        self.fm2_button.place(x=500, y=40, anchor='nw')

        self.fm2.place(x=0, y=100, anchor='nw')

    def create_widgets(self):
        # 框架1 大标题
        self.fm1 = Frame(self)
        self.titlelable = Label(self.fm1, text='直流电机控制系统',
                                font=('微软雅黑', 32))
        self.titlelable.pack()
        self.fm1.pack()
        # 框架2
        self.fm2_function()

        self.fm3 = Frame(self)
        self.graph_button = Button(self.fm3, text='转速调试示波器',
                                   width='30', height='5',
                                   font=('微软雅黑', 14), bg='gray',
                                   command=open_plt)
        self.graph_button.pack(side=TOP, anchor='n')
        self.fm3.pack(side=BOTTOM, anchor='n')


class Scope(object):
    def __init__(self, ax, maxt=20, dt=0.02):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(0, 7500)  # y轴区间
        self.ax.set_xlim(0, self.maxt)  # x轴区间
        ani = animation.FuncAnimation(fig, self.update, get_temp,
                                      interval=50, blit=True)
        plt.show()

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt:  # reset the arrays
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,


def get_temp():
    yield global_rpm


def open_plt():
    global fig
    global ax
    fig, ax = plt.subplots()
    scope = Scope(ax)


if __name__ == '__main__':
    app = Application()
    app.mainloop()

'''

      ┌─┐       ┌─┐ + +
   ┌──┘ ┴───────┘ ┴──┐++
   │                 │
   │       ───       │++ + + +
   ███████───███████ │+
   │                 │+
   │       ─┴─       │
   │                 │
   └───┐         ┌───┘
       │         │
       │         │   + +
       │         │
       │         └──────────────┐
       │                        │
       │                        ├─┐
       │                        ┌─┘
       │                        │
       └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +
         │ ─┤ ─┤       │ ─┤ ─┤
         └──┴──┘       └──┴──┘  + + + +
               代码无BUG!

'''
