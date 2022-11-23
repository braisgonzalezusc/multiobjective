from pyomo.environ import *
import uuid


class MultiObjective:

    def __init__(self, pyo_model, converToMin=True):
        self.pyo_model = pyo_model
        self.pyo_model_original = pyo_model.clone()
        self.obj_uuid = str(uuid.uuid4()).replace("-", "")
        self.obj_uuid = "a" + self.obj_uuid[1:]
        self.cons_uuid = []
        if converToMin:
            self.convert_to_minimization()
        self.objs_dict = {
            v.name: v for v in
            self.pyo_model.component_objects(Objective)
        }

    def convert_to_minimization(self):
        for obj in self.pyo_model.component_objects(Objective):
            obj.expr = obj.sense * obj.expr
            obj.sense = 1

    def activate_one_objfun(self, index):
        counter = 1
        current_obj = None
        for obj in self.pyo_model.component_objects(Objective):
            if counter != index:
                obj.deactivate()
            else:
                obj.activate()
                current_obj = obj
            counter += 1
        return current_obj

    def activate_objfun_byName(self, objname):

        for obj in self.pyo_model.component_objects(Objective):
            if obj.name == objname:
                obj.activate()
            else:
                obj.deactivate()

    def weight_obj_fun(self, w):
        new_obj = None
        i = 0
        for obj in self.pyo_model.component_objects(Objective):
            if new_obj is None:
                new_obj = w[i] * obj.expr
            else:
                new_obj = new_obj + (w[i] * obj.expr)
            obj.deactivate()
            i = i + 1
        setattr(self.pyo_model, self.obj_uuid, Objective(expr=new_obj, sense=minimize))

    def add_constraint(self, expression):
        cons_id = str(uuid.uuid4()).replace("-", "")
        cons_id = "a" + cons_id[1:]
        self.cons_uuid.append(cons_id)
        setattr(self.pyo_model, cons_id, Constraint(expr=expression))

    def deactivate_cons_uuid(self):
        for con in self.pyo_model.component_objects(Constraint):
            if con.name in self.cons_uuid:
                con.deactivate()
                self.cons_uuid.remove(con.name)

    def solve(self):
        solver = SolverFactory('ipopt')
        solver.solve(self.pyo_model)
        return self.pyo_model