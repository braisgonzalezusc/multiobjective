from pyomo.environ import *
import uuid


class MultiObjective:

    def __init__(self, pyo_model):
        self.pyo_model = pyo_model
        self.pyo_model_original = pyo_model
        self.obj_uuid = str(uuid.uuid4()).replace("-", "")
        self.cons_uuid = []

    def sum_obj_funs(self):
        new_obj = None
        for obj in self.pyo_model.component_objects(Objective):
            if new_obj is None:
                new_obj = obj.sense * obj
            else:
                new_obj += obj.sense * obj
            obj.deactivate()
        setattr(self.pyo_model, self.obj_uuid, Objective(expr=new_obj, sense=minimize))

    def add_constraint(self, expression):
        cons_id = str(uuid.uuid4()).replace("-", "")
        self.cons_uuid.append(cons_id)
        setattr(self.pyo_model, cons_id, Constraint(expr=expression))

    def solve(self):
        solver = SolverFactory('ipopt')
        solver.solve(self.pyo_model, tee=True)
        return self.pyo_model
