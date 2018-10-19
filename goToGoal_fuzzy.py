# coding: utf-8
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Antecedents
distance = ctrl.Antecedent(np.arange(0, 20, 0.5), 'distance') # em metros
angular_distance = ctrl.Antecedent(np.arange(0, 360, 1), 'angular distance') # graus ou radianos; angulo entre (Robot-Goal) e a orientação do robô (deve ser próximo de 180 graus)

# Consequents
right_wheel = ctrl.Consequent(np.arange(0, 5, 0.5), 'right wheel speed') # velocidade (angular ??) na roda direita
left_wheel = ctrl.Consequent(np.arange(0, 5, 0.5), 'left wheel speed') # velocidade na roda esquerda

# Membership functions for distance
distance['none'] = fuzz.trimf(distance.universe, [0, 0, 0.1])
distance['small'] = fuzz.trimf(distance.universe, [0, 0, 1.5])
distance['medium'] = fuzz.trapmf(distance.universe, [1, 2, 2, 3])
distance['big'] = fuzz.trapmf(distance.universe, [2.5, 5, 20, 20])

# Membership for angular distance
angular_distance['small from left'] = fuzz.trimf(angular_distance.universe, [0, 0, 25]) # diferença angular pequena pela esquerda
angular_distance['smaller from left'] = fuzz.trapmf(angular_distance.universe, [0, 0, 179, 180])
angular_distance['smaller from right'] = fuzz.trapmf(angular_distance.universe, [180, 181, 359, 360])
angular_distance['small from right'] = fuzz.trimf(angular_distance.universe, [335, 360, 360])

# Membership for right_wheel
right_wheel['zero'] = fuzz.trimf(right_wheel.universe, [0, 0, 0])
right_wheel['low'] = fuzz.trimf(right_wheel.universe, [0, 0, 1.5])
right_wheel['high'] = fuzz.trapmf(right_wheel.universe, [1, 2.5, 5, 5])

# Membership for left_wheel
left_wheel['zero'] = fuzz.trimf(left_wheel.universe, [0, 0, 0])
left_wheel['low'] = fuzz.trimf(left_wheel.universe, [0, 0, 1.5])
left_wheel['high'] = fuzz.trapmf(left_wheel.universe, [1, 2.5, 5, 5])

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

rules['turn_left_slowly_0'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from left'], right_wheel['low'])
rules['turn_left_slowly_1'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from left'], left_wheel['zero'])

rules['turn_right_slowly_0'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from right'], right_wheel['zero'])
rules['turn_right_slowly_1'] = ctrl.Rule((distance['small'] | distance['medium']) & angular_distance['small from right'], left_wheel['low'])

rules['follow_0'] = ctrl.Rule(angular_distance['small from right'] | angular_distance['small from left'], right_wheel['low'])
rules['follow_1'] = ctrl.Rule(angular_distance['small from right'] | angular_distance['small from left'], left_wheel['low'])

#for r in rules:
#    rules[r].view()

# Sistema de controle e simulação
goToGoal_ctrl = ctrl.ControlSystem([rules[key] for key in rules])
goToGoal_sim = ctrl.ControlSystemSimulation(goToGoal_ctrl)

# Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
# Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
goToGoal_sim.input['distance'] = 2.0
goToGoal_sim.input['angular distance'] = 50.0

# Crunch the numbers
goToGoal_sim.compute()

# Visualize the output of the simulation
print(goToGoal_sim.output)
#tip.view(sim=tipping)

