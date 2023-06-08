"""
# Example lexicographic
from multiobjective import *
import lexicographic
model = ConcreteModel()
model.x = Var()
model.y = Var()
model.objective1 = Objective(expr=(model.x-model.y-3)**2, sense=minimize)
model.objective2 = Objective(expr=-((model.y-2)**2+1), sense=maximize)
model.c1 = Constraint(expr=model.x>=0)
model.c2 = Constraint(expr=model.y>=0)
mo = MultiObjective(model)
model_lex = lexicographic.lexicographic(mo,[2,1])
print(model_lex.x.value)
print(model_lex.y.value)
"""

"""
# Example linear scalarizing
from multiobjective import *
import linscalarizing
model = ConcreteModel()
model.x = Var()
model.y = Var()
model.objective1 = Objective(expr=(model.x-model.y-3)**2, sense=minimize)
model.objective2 = Objective(expr=-((model.y-2)**2+1), sense=maximize)
model.c1 = Constraint(expr=model.x>=0)
model.c2 = Constraint(expr=model.y>=0)
mo = MultiObjective(model)
model_lex = linscalarizing.linscalarizing(mo,[5,10])
print(model_lex.x.value)
print(model_lex.y.value)
"""


# # Example 1 epsilon constraint method
from multiobjective.epsilonconstraint import *
model = ConcreteModel()
model.x1 = Var()
model.x2 = Var()
model.objective1 = Objective(expr=model.x1, sense=maximize)
model.objective2 = Objective(expr=3*model.x1+4*model.x2, sense=maximize)
model.c1 = Constraint(expr=model.x1<=20)
model.c2 = Constraint(expr=model.x2<=40)
model.c3 = Constraint(expr=5*model.x1+4*model.x2 <= 200)
mo = MultiObjective(model, conver_to_min=True)
#payofftable,objs_range = compute_payofftable(mo)

ngrid = 5
ranges = {
    'objective1': [8, 20],
   # 'objective2': [160, 184] #[60, 184]
    'objective2': [60, 184]
}

pareto_front, vars_sol = epsilonconstr(mo, obj_main=None, payofftable=ranges, ngrid=5)

import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

# Create pandas frame with algorithm pareto front
df = pd.DataFrame(pareto_front, columns=list(mo.objs_dict.keys()))
# Create the figure:
fig = go.Figure()
# Create and style traces
fig.add_trace(go.Scatter(x=df[list(mo.objs_dict.keys())[0]],
                         y=df[list(mo.objs_dict.keys())[1]],
                         name='Algorithm pareto front',
                         line=dict(color='firebrick', width=4, dash='dot'),
                         mode='lines+markers'))
fig.show()

# Edit the layout
fig.update_layout(title='Pareto Frontier',
                   xaxis_title=list(mo.objs_dict.keys())[0],
                   yaxis_title=list(mo.objs_dict.keys())[1])

# Example 2 epsilon constraint method
# from multiobjective.epsilonconstraint import *
# model = ConcreteModel()
# model.x1 = Var(bounds=(0, 1))
# model.x2 = Var(bounds=(0, 1))
# model.x3 = Var(bounds=(0, 1))
# model.objective1 = Objective(expr=model.x1, sense=maximize)
# model.objective2 = Objective(expr=model.x2, sense=maximize)
# model.objective3 = Objective(expr=model.x3, sense=maximize)
# model.c1 = Constraint(expr=model.x1 + model.x2 + model.x3 <= 1)
# mo = MultiObjective(model, conver_to_min=True)
# #payofftable,objs_range = compute_payofftable(mo)
#
# ngrid = 11
#
# pareto_front, vars_sol = epsilonconstr(mo, obj_main=None, payofftable=None, ngrid=ngrid)

"""
# Example 3: ZDT problem for Jmetal
from multiobjective.epsilonconstraint import *
from multiobjective.jmetalinterface import encode_solution_to_jmetal, update_pyomo_solution_from_jmetal

#from jmetal.problem.multiobjective.zdt import

model = ConcreteModel()
model.indexvar = [i+1 for i in range(30)]
model.x = Var(model.indexvar, bounds=(0, 1), initialize=0)
model.f1 = Objective(expr=model.x[1], sense=minimize)
def gfun(model):
    g = sum(model.x[i] for i in model.indexvar)-model.x[1]
    constant = 9.0/(len(model.x)-1)
    return constant*g+1.0
model.g = gfun(model)

def hfun(model):
    return 1-sqrt(model.x[1]/model.g)
model.h = hfun(model)
model.f2 = Objective(expr=model.h*model.g, sense=minimize)
mo = MultiObjective(model, conver_to_min=True)
payofftable, objs_range = compute_payofftable(mo)
#ngrid = 1000
ngrid = 50
pareto_front, vars_sol = epsilonconstr(mo, obj_main=None, payofftable=None, ngrid=ngrid)


#########################
# Plot front with plotly
########################
from jmetal.util.solution import (
    read_solutions,
)
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

# Create pandas frame with algorithm pareto front
df = pd.DataFrame(pareto_front, columns=list(mo.objs_dict.keys()))
#fig = px.line(df, x=list(mo.objs_dict.keys())[0], y=list(mo.objs_dict.keys())[1], title='Pareto Front', markers=True)
# Create pandas frame with the exact pareto front
reference_front = read_solutions(filename="C:/Users/ANGEL/Documents/Git repositories/jMetalPy/resources/reference_front/ZDT1.pf")
exact_front = []
for sol in reference_front:
    exact_front.append(sol.objectives)
df_exact_front = pd.DataFrame(pareto_front, columns=list(mo.objs_dict.keys()))

# Create the figure:
fig = go.Figure()
# Create and style traces
fig.add_trace(go.Scatter(x=df[list(mo.objs_dict.keys())[0]],
                         y=df[list(mo.objs_dict.keys())[1]],
                         name='Algorithm pareto front',
                         line=dict(color='firebrick', width=4, dash='dot'),
                         mode='lines+markers'))

fig.add_trace(go.Scatter(x=df[list(mo.objs_dict.keys())[0]],
                         y=df[list(mo.objs_dict.keys())[1]],
                         name='Exact front',
                         line=dict(color='azure', width=4), #, dash='dash'),
                         mode='lines+markers'))
fig.show()

# Edit the layout
fig.update_layout(title='Pareto Frontier',
                   xaxis_title=list(mo.objs_dict.keys())[0],
                   yaxis_title=list(mo.objs_dict.keys())[1])



#soljmetal, mappyomo = encode_solution_to_jmetal(mo)
#print(soljmetal)
#update_pyomo_solution_from_jmetal(mo, soljmetal, mappyomo)
#for v in mo.pyo_model.component_data_objects(pyo.Var, active=True):
#    print('%s = %d', str(v), pyo.value(v))
"""

"""
# ZDT1 resuelto directamente con jmetal:
from jmetal.algorithm.multiobjective.nsgaii import NSGAII
from jmetal.lab.visualization import InteractivePlot, Plot
from jmetal.operator import PolynomialMutation, SBXCrossover
from jmetal.problem import ZDT6, ZDT1
from jmetal.util.observer import ProgressBarObserver, VisualizerObserver
from jmetal.util.solution import (
    print_function_values_to_file,
    print_variables_to_file,
    read_solutions,
)
from jmetal.util.termination_criterion import StoppingByEvaluations
from jmetal.operator.mutation import CompositeMutation
from jmetal.operator.crossover import CompositeCrossover
from multiobjective.jmetalinterface import PyomoProblem

solve_with_pyomo = True
if solve_with_pyomo:
    problem = PyomoProblem(mo)

    mutation_operator = CompositeMutation(
            [PolynomialMutation(probability=1.0 / problem.number_of_variables(), distribution_index=20)]
    )
    crossover_operator = CompositeCrossover(
            [SBXCrossover(probability=1.0, distribution_index=20)]
    )
else:
    problem = ZDT1()

    mutation_operator = PolynomialMutation(probability=1.0 / problem.number_of_variables(), distribution_index=20)
    crossover_operator = SBXCrossover(probability=1.0, distribution_index=20)


# Solve with jmetal:
max_evaluations = 10000
algorithm = NSGAII(
    problem=problem,
    population_size=100,
    offspring_population_size=100,
    mutation=mutation_operator,
    crossover=crossover_operator,
    termination_criterion=StoppingByEvaluations(max_evaluations=max_evaluations),
    )

problem.reference_front = read_solutions(filename="C:/Users/ANGEL/Documents/Git repositories/jMetalPy/resources/reference_front/ZDT1.pf")
algorithm.observable.register(observer=ProgressBarObserver(max=max_evaluations))
algorithm.observable.register(observer=VisualizerObserver(reference_front=problem.reference_front))

algorithm.run()
front = algorithm.get_result()

# Plot front
plot_front = Plot(
    title="Pareto front approximation. Problem: " + problem.name(),
    reference_front=problem.reference_front,
    axis_labels=problem.obj_labels,
)
plot_front.plot(front, label=algorithm.label, filename=algorithm.get_name())

# Plot interactive front
plot_front = InteractivePlot(
    title="Pareto front approximation. Problem: " + problem.name(),
    reference_front=problem.reference_front,
    axis_labels=problem.obj_labels,
)
plot_front.plot(front, label=algorithm.label, filename=algorithm.get_name())

# Save results to file
print_function_values_to_file(front, "FUN." + algorithm.label)
print_variables_to_file(front, "VAR." + algorithm.label)

print(f"Algorithm: {algorithm.get_name()}")
print(f"Problem: {problem.name()}")
print(f"Computing time: {algorithm.total_computing_time}")
"""

#import pyomo.core.plugins.transform.scaling as scale

# print("caca")
#
# from pyomo.environ import *
# # create the model
# model = ConcreteModel()
# model.x = Var(bounds=(-5, 5), initialize=1.0)
# model.y = Var(bounds=(0, 1), initialize=1.0)
# model.obj = Objective(expr=1e8*model.x + 1e6*model.y)
# model.con = Constraint(expr=model.x + model.y == 1.0)
# # create the scaling factors
# model.scaling_factor = Suffix(direction=Suffix.EXPORT)
# model.scaling_factor[model.obj] = 1e-6 # scale the objective
# model.scaling_factor[model.con] = 2.0  # scale the constraint
# model.scaling_factor[model.x] = 0.2    # scale the x variable
# # transform the model
# scaled_model = TransformationFactory('core.scale_model').create_using(model)
#
# TransformationFactory('core.scale_model').propagate_solution(scaled_model, model)
# # print the value of the objective function to show scaling has occurred
# print(value(model.x))
# print(value(scaled_model.scaled_x))
# print(value(scaled_model.scaled_x.lb))
# print(value(model.obj))
# print(value(scaled_model.scaled_obj))


"""
# Example goal programming
from multiobjective import *
import goalprogramming
model = ConcreteModel()
model.x = Var()
model.y = Var()
model.objective1 = Objective(expr=(model.x-model.y-3)**2, sense=minimize)
model.objective2 = Objective(expr=-((model.y-2)**2+1), sense=maximize)
model.c1 = Constraint(expr=model.x>=0)
model.c2 = Constraint(expr=model.y>=0)
mo = MultiObjective(model)
model_goal = goalprogramming.goalprogramming(mo,dict({"objective1":0.2,"objective2":1.1}),dict({"objective1":2,"objective2":1}))
print(model_goal.x.value)
print(model_goal.y.value)
"""

"""
# Example weighted programming
from multiobjective import *
import weightedmetrics
model = ConcreteModel()
model.x = Var()
model.y = Var()
model.objective1 = Objective(expr=(model.x-model.y-3)**2, sense=minimize)
model.objective2 = Objective(expr=-((model.y-2)**2+1), sense=maximize)
model.c1 = Constraint(expr=model.x>=0)
model.c2 = Constraint(expr=model.y>=0)
mo = MultiObjective(model)
model_goal = weightedmetrics.weightedmetrics(mo,dict({"objective1":0.2,"objective2":1.1}),dict({"objective1":2, "objective2":1}))
print(model_goal.x.value)
print(model_goal.y.value)
"""


# def pruebo_recursividad(ind):
#     print("Pruebo_recur: ",ind)
#     max = 10
#     cur = 1
#     while cur < max:
#         print("indcur",ind)
#         print("cur", cur)
#         jump = calculatejump(ind)
#         cur += jump
#
# def calculatejump(ind):
#     print("calculo jump: ", ind)
#     if ind > 2:
#         pruebo_recursividad(ind - 1)
#         return 1
#
#     print("-------")
#     print("acabo!!")
#     print("-------")
#     return 100
#
# pruebo_recursividad(5)