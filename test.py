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


# Example epsilon constraint method
from multiobjective import *
import lexicographic
from epsilonconstraint import *
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
payofftable = {
    'objective1': [8, 20],
   # 'objective2': [160, 184] #[60, 184]
    'objective2': [60, 184]
}

pareto_front, vars_sol = epsilonconstr(mo, obj_main=None, payofftable=None, ngrid=5)


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