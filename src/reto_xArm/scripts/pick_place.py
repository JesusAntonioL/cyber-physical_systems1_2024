#!/usr/bin/env python3
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI
import rospy
from std_msgs.msg import Int16  # Para enviar mensajes de texto simples
from std_msgs.msg import Bool


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()
        self.sub_rutina = 0 

        # Inicializa el nodo ROS y el publicador
        rospy.init_node('xarm_controller', anonymous=True)
        self.pub = rospy.Publisher('/xarm_movement', Bool, queue_size=10)
        self.sub = rospy.Subscriber("/xarm_routine", Int16, self.xarm_callback)

    def xarm_callback(self, msg):
        self.sub_rutina = msg.data
        print("callback b1")

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    # Robot Main Run
    def run(self):
        try:
            self._tcp_speed = 300
            self._tcp_acc = 1000
            code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            
            info_usuario = 10
            #sub_rutina = 0

            while self.is_alive and info_usuario == 10:
                #sub_rutina = int(input("Ingresa un numero: "))
                if self.sub_rutina == 1:
                    code = self._arm.set_servo_angle(angle=[-90.0, -59.4, -12.0, 70.6, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-90.0, -39.1, -78.0, 117.1, 92.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-188.4, -39.0, -69.5, 108.5, 87.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-183.9, 18.8, -127.8, 109.0, 91.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-183.9, 14.5, -116.8, 102.3, 91.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    time.sleep(1)
                    code = self._arm.set_tgpio_digital(0, 1, delay_sec=0)
                    if not self._check_code(code, 'set_tgpio_digital'):
                        return
                    time.sleep(1)
                    code = self._arm.set_servo_angle(angle=[-183.9, 18.8, -127.8, 109.0, 91.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-188.4, -39.0, -69.5, 108.5, 87.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-90.0, -39.1, -78.0, 117.1, 92.6], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-90.0, -59.4, -12.0, 70.6, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    self.pub.publish(True)
                    self.sub_rutina = 4

                elif self.sub_rutina == 2:
                    code = self._arm.set_servo_angle(angle=[-90.0, -59.4, -12.0, 70.6, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-90.0, -37.5, -81.3, 118.1, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-182.7, -37.5, -81.3, 118.1, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-181.5, 12.4, -119.1, 106.7, 1.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-181.5, 9.3, -110.6, 101.4, 1.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    time.sleep(1)
                    code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
                    if not self._check_code(code, 'set_tgpio_digital'):
                        return
                    time.sleep(1)
                    code = self._arm.set_servo_angle(angle=[-181.5, 12.4, -119.1, 106.7, 1.2], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-182.7, -37.5, -81.3, 118.1, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-90.0, -37.5, -81.3, 118.1, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    code = self._arm.set_servo_angle(angle=[-90.0, -59.4, -12.0, 70.6, 0.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    self.pub.publish(True)
                    self.sub_rutina = 4

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.31.198', baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()
