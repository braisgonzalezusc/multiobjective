import uuid
from pyomo.environ import *


def goalprogramming(mo, goals, w):
    delta_plus = {}
    delta_minus = {}
    for nam, goal in goals.items():
        var_id = mo.add_new_var()
        delta_plus[nam] = var_id
        var_id = mo.add_new_var()
        delta_minus[nam] = var_id

    new_obj_id = str(uuid.uuid4()).replace("-", "")
    new_obj_id = "a" + new_obj_id[1:]
    setattr(mo.pyo_model, new_obj_id, Objective(expr=sum([w[nam]*(mo.pyo_model.__getattribute__(delta_plus[nam])+mo.pyo_model.__getattribute__(delta_minus[nam])) for nam in list(goals.keys())])))
    mo.activate_objfun_by_name(new_obj_id)

    i = 0
    for nam, goal in goals.items():
        mo.add_constraint(mo.pyo_model.__getattribute__(nam).expr - mo.pyo_model.__getattribute__(delta_plus[nam]) + mo.pyo_model.__getattribute__(delta_minus[nam]) == goal)
        mo.add_constraint(mo.pyo_model.__getattribute__(delta_plus[nam]) >= 0)
        mo.add_constraint(mo.pyo_model.__getattribute__(delta_minus[nam]) >= 0)
        i = i + 1

    solved_model = mo.solve()
    mo.restore_original_model()
    return solved_model
