import uuid
from pyomo.environ import *


def weightedmetrics(mo, goals, w):
    var_id = mo.add_new_var()
    new_obj_id = str(uuid.uuid4()).replace("-", "")
    new_obj_id = "a" + new_obj_id[1:]
    setattr(mo.pyo_model, new_obj_id, Objective(expr=mo.pyo_model.__getattribute__(var_id)))
    mo.activate_objfun_by_name(new_obj_id)
    for nam, goal in goals.items():
        mo.add_constraint(mo.pyo_model.__getattribute__(var_id) >= w[nam]*(mo.pyo_model.__getattribute__(nam).expr - goal))
    solved_model = mo.solve()
    mo.restore_original_model()
    return solved_model
