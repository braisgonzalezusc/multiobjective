from multiobjective import *

# Example lexicographic
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

# Example linear scalarizing
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





