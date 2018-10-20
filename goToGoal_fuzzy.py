# coding: utf-8
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Antecedents
distance = ctrl.Antecedent(np.arange(0, 20, 0.5), 'distance') # em metros
angular_distance = ctrl.Antecedent(np.arange(0, 360, 1), 'angular distance') # graus ou radianos; angulo entre (Robot-Goal) e a orientação do robô (deve ser próximo de 180 graus)

# Consequents
right_wheel = ctrl.Consequent(np.arange(0, 4, 0.5), 'right wheel speed') # velocidade (angular ??) na roda direita
left_wheel = ctrl.Consequent(np.arange(0, 4, 0.5), 'left wheel speed') # velocidade na roda esquerda

# Membership functions for distance
distance['none'] = fuzz.trimf(distance.universe, [0, 0, 0.15])
distance['small'] = fuzz.trimf(distance.universe, [0, 0.2, 0.8])
distance['medium'] = fuzz.trapmf(distance.universe, [0.7, 1.5, 2, 3])
distance['big'] = fuzz.trapmf(distance.universe, [2.5, 5, 20, 20])

# Membership for angular distance
angular_distance['small from left'] = fuzz.trimf(angular_distance.universe, [0, 0, 15]) # diferença angular pequena pela esquerda
angular_distance['smaller from left'] = fuzz.trapmf(angular_distance.universe, [0, 0, 179, 180])
angular_distance['smaller from right'] = fuzz.trapmf(angular_distance.universe, [180, 181, 359, 360])
angular_distance['small from right'] = fuzz.trimf(angular_distance.universe, [345, 360, 360])

# Membership for right_wheel
right_wheel['zero'] = fuzz.trimf(right_wheel.universe, [0, 0, 0])
right_wheel['low'] = fuzz.trimf(right_wheel.universe, [0, 0, 1.5])
right_wheel['high'] = fuzz.trapmf(right_wheel.universe, [1, 2, 4, 4])

# Membership for left_wheel
left_wheel['zero'] = fuzz.trimf(left_wheel.universe, [0, 0, 0])
left_wheel['low'] = fuzz.trimf(left_wheel.universe, [0, 0, 1.5])
left_wheel['high'] = fuzz.trapmf(left_wheel.universe, [1, 2, 4, 4])

# Visualize membership
#distance.view()
#angular_distance.view()
#right_wheel.view()
#left_wheel.view()

#raw_input()

# Fuzzy rules
rules = {}
rules['stop_0'] = ctrl.Rule(distance['none'], right_wheel['zero'])
rules['stop_1'] = ctrl.Rule(distance['none'], left_wheel['zero'])

rules['turn_left_0'] = ctrl.Rule((distance['big'] | distance['medium']) & angular_distance['smaller from left'] & (~angular_distance['small from left']), right_wheel['high'])
rules['turn_left_1'] = ctrl.Rule((distance['big'] | distance['medium']) & angular_distance['smaller from left'] & (~angular_distance['small from left']), left_wheel['low'])

rules['turn_right_0'] = ctrl.Rule((distance['big'] | distance['medium']) & angular_distance['smaller from right'] & (~angular_distance['small from right']), right_wheel['low'])
rules['turn_right_1'] = ctrl.Rule((distance['big'] | distance['medium']) & angular_distance['smaller from right'] & (~angular_distance['small from right']), left_wheel['high'])

#rules['turn_left_2'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from left'], right_wheel['low'])
#rules['turn_left_3'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from left'], left_wheel['zero'])

#rules['turn_right_2'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from right'], right_wheel['zero'])
#rules['turn_right_3'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from right'], left_wheel['low'])

rules['turn_left_standing_0'] = ctrl.Rule(angular_distance['smaller from left'], right_wheel['low'])
rules['turn_left_standing_1'] = ctrl.Rule(angular_distance['smaller from left'], left_wheel['zero'])

rules['turn_right_standing_0'] = ctrl.Rule(angular_distance['smaller from right'], right_wheel['zero'])
rules['turn_right_standing_1'] = ctrl.Rule(angular_distance['smaller from right'], left_wheel['low'])

rules['follow_0'] = ctrl.Rule(angular_distance['small from right'] | angular_distance['small from left'], right_wheel['high'])
rules['follow_1'] = ctrl.Rule(angular_distance['small from right'] | angular_distance['small from left'], left_wheel['high'])

rules['follow_2'] = ctrl.Rule(distance['small'] & (angular_distance['small from right'] | angular_distance['small from left']), right_wheel['low'])
rules['follow_3'] = ctrl.Rule(distance['small'] & (angular_distance['small from right'] | angular_distance['small from left']), left_wheel['low'])

#for r in rules:
#    rules[r].view()

# Sistema de controle e simulação
goToGoal_ctrl = ctrl.ControlSystem([rules[key] for key in rules])
goToGoal_sim = ctrl.ControlSystemSimulation(goToGoal_ctrl)

# Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
# Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
def get_fuzzy_control(dist, ang_dist):
    goToGoal_sim.input['distance'] = dist
    goToGoal_sim.input['angular distance'] = ang_dist
    goToGoal_sim.compute()
    #print(goToGoal_sim.output)
    return goToGoal_sim.output

