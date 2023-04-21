from pyomo.environ import *
import uuid


class MultiObjective:

    def __init__(self, pyo_model, conver_to_min=True):
        self.pyo_model = pyo_model.clone()
        self.pyo_model_original = pyo_model
        self.obj_uuid = str(uuid.uuid4()).replace("-", "")
        self.obj_uuid = "a" + self.obj_uuid[1:]
        self.cons_uuid = []
        self.vars_uuid = []
        self.conver_to_min = conver_to_min
        if self.conver_to_min:
            self.convert_to_minimization()
        self.objs_dict = {
            v.name: v for v in
            self.pyo_model.component_objects(Objective)
        }

    def restore_original_model(self):
        self.pyo_model = self.pyo_model_original.clone()
        self.cons_uuid = []
        self.vars_uuid = []
        if self.conver_to_min:
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

    def activate_objfun_by_name(self, objname):
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

    def check_feasibility(self):
        for con in self.pyo_model.component_objects(Constraint):
            if con.lb is None:
                lb = -1e100
            else:
                lb = con.lb
            if con.ub is None:
                ub = 1e100
            else:
                ub = con.ub
            if not (lb <= value(con) <= ub):
                return False
        return True

    def add_new_var(self):
        var_id = str(uuid.uuid4()).replace("-", "")
        var_id = "a" + var_id[1:]
        self.vars_uuid.append(var_id)
        setattr(self.pyo_model, var_id, Var())
        return var_id
