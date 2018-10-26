# coding: utf-8
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class wall_follow_fuzzy():
    def __init__(self):
        self.max_speed = 2.0
        self.min_wall_dist = 0.0
        self.max_wall_dist = 6.0
        self.steps = 0.1

    def init_fuzzy(self):
        '''
            Call this function to initialize the fuzzy control
        '''

        # Antecedents
        left_wall_dist  = ctrl.Antecedent(np.arange(0, self.max_wall_dist, self.steps), 'left wall distance')
        right_wall_dist = ctrl.Antecedent(np.arange(0, self.max_wall_dist, self.steps), 'right wall distance')

        # Consequents
        v_left_wheel  = ctrl.Consequent(np.arange(-self.max_speed, self.max_speed + 1.0, self.steps), 'left wheel speed')
        v_right_wheel = ctrl.Consequent(np.arange(-self.max_speed, self.max_speed + 1.0, self.steps), 'right wheel speed')

        # Membership functions for left_wall_distance
        left_wall_dist['small']       = fuzz.trimf(left_wall_dist.universe, [0, 0.2, 0.4])
        left_wall_dist['medium']      = fuzz.trapmf(left_wall_dist.universe, [0.3, 0.4, 0.5, 0.6])
        left_wall_dist['large']       = fuzz.trimf(left_wall_dist.universe, [0.55, 0.7, 0.9])
        left_wall_dist['extra-large'] = fuzz.trimf(left_wall_dist.universe, [0.8, 6.0, 6.0])

        # Membership functions for right_wall_distance
        right_wall_dist['small']       = fuzz.trimf(right_wall_dist.universe, [0, 0.2, 0.4])
        right_wall_dist['medium']      = fuzz.trapmf(right_wall_dist.universe, [0.3, 0.4, 0.5, 0.6])
        right_wall_dist['large']       = fuzz.trimf(right_wall_dist.universe, [0.55, 0.8, 0.9])
        right_wall_dist['extra-large'] = fuzz.trimf(right_wall_dist.universe, [0.8, 6.0, 6.0])

        # Membership functions for v_left_wheel
        v_left_wheel['positive'] = fuzz.trimf(v_left_wheel.universe, [0.0, self.max_speed-0.1, self.max_speed-0.1])
        v_left_wheel['negative'] = fuzz.trimf(v_right_wheel.universe, [-self.max_speed, -self.max_speed, 0.0])

        # Membership functions for v_right_wheel
        v_right_wheel['positive'] = fuzz.trimf(v_left_wheel.universe, [0.0, self.max_speed, self.max_speed])
        v_right_wheel['negative'] = fuzz.trimf(v_right_wheel.universe, [-self.max_speed+0.1, -self.max_speed+0.1, 0.0])

        # Fuzzy rules
        rules = {}

        rules['turn left 0'] = ctrl.Rule(right_wall_dist['small'], v_left_wheel['negative'])
        rules['turn left 1'] = ctrl.Rule(right_wall_dist['small'], v_right_wheel['positive'])
        rules['turn left 2'] = ctrl.Rule(left_wall_dist['large'], v_left_wheel['negative'])
        rules['turn left 3'] = ctrl.Rule(left_wall_dist['large'], v_right_wheel['positive'])

        rules['turn right 0'] = ctrl.Rule(left_wall_dist['small'], v_left_wheel['positive'])
        rules['turn right 1'] = ctrl.Rule(left_wall_dist['small'], v_right_wheel['negative'])
        rules['turn right 2'] = ctrl.Rule(right_wall_dist['large'], v_left_wheel['positive'])
        rules['turn right 3'] = ctrl.Rule(right_wall_dist['large'], v_right_wheel['negative'])

        rules['go on 0'] = ctrl.Rule(left_wall_dist['medium'] | right_wall_dist['medium'], v_left_wheel['positive'])
        rules['go on 1'] = ctrl.Rule(left_wall_dist['medium'] | right_wall_dist['medium'], v_right_wheel['positive'])
        rules['go on 2'] = ctrl.Rule(left_wall_dist['extra-large'] & right_wall_dist['extra-large'], v_left_wheel['positive']) # O erro era aqui, pois estava v_right_wheel
        rules['go on 3'] = ctrl.Rule(left_wall_dist['extra-large'] & right_wall_dist['extra-large'], v_right_wheel['positive'])

        # Sistema de controle e simulação
        sim_ctrl = ctrl.ControlSystem([rules[key] for key in rules])
        self.sim = ctrl.ControlSystemSimulation(sim_ctrl)

        print('Fuzzy wall follow initialized')

    def get_vel(self, left_wall_dist, right_wall_dist):
        '''
        	Returns the speed in both wheels
        	arg: distance in all 2 sensors (sensor 1 and sensor 8 respectively)
        	return: velocity tuple (left and right respectively)
        '''

        self.sim.input['left wall distance'] = left_wall_dist
        self.sim.input['right wall distance'] = right_wall_dist

        self.sim.compute()

        return self.sim.output['left wheel speed'], self.sim.output['right wheel speed']
