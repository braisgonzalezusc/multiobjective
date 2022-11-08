from multiobjective import *
model = ConcreteModel()
model.x = Var()
model.objective1 = Objective(expr=(model.x-1)**2, sense=minimize)
model.objective2 = Objective(expr=-(model.x-2)**2, sense=maximize)
mo = MultiObjective(model)
mo.sum_obj_funs()
mo.add_constraint(model.x >= 2)
model = mo.solve()
model.x.pprint()






