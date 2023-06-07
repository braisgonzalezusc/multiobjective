import pyomo.environ as pyo
import random
from jmetal.core.problem import Problem
from jmetal.core.solution import FloatSolution, CompositeSolution, PermutationSolution, BinarySolution, IntegerSolution

from multiobjective.multiobjective import MultiObjective
import logging

logging.basicConfig()
logger = logging.getLogger('jmetalinterface')

# objs_val = {
#     k: mo.objs_dict[k].expr() for k in mo.py_model.objs_dict
# }
# vars_val = {
#     str(v): pyo.value(v) for v in mo.py_model.component_data_objects(pyo.Var, active=True)
# }

class PyomoProblem(Problem):

    def __init__(self, mo: MultiObjective, options=None):
        super(PyomoProblem, self).__init__()
        self.mo = mo
        #soljemtal, self.pyomo_to_jmetal_var_name = encode_solution_to_jmetal(mo)

        self.obj_directions = [self.MINIMIZE]*len(mo.objs_dict.keys())
        self.obj_labels = list(mo.objs_dict.keys())
        #self.objective = [mo.objs_dict[k].expr() for k in mo.objs_dict]

        self.create_solution(computeobj=False)

        #self.variables = self.create_variables()
        #self.constraints = self._compute_constraints()

    def number_of_variables(self) -> int:
        return len(list(self.mo.pyo_model.component_data_objects(pyo.Var, active=True)))

    def number_of_objectives(self) -> int:
        return len(self.mo.objs_dict.keys())

    def number_of_constraints(self) -> int:
        return len(list(self.mo.pyo_model.component_objects(pyo.Constraint)))

    def create_solution(self, computeobj=False) -> CompositeSolution:

        logger.debug("------------------------")
        logger.debug("----INITIAL SOLUTION----")

        # Encode solution to jmetal:
        final_sol, self.pyomo_to_jmetal_var_name = encode_solution_to_jmetal(self.mo)

        # Compute objective function
        if computeobj:
            # Upddate pyomo var from jmetal solution vars
            update_pyomo_solution_from_jmetal(self.mo, final_sol, self.pyomo_to_jmetal_var_name)
            objective = []
            for ind, obj in enumerate(self.mo.objs_dict):
                objective.append(pyo.value(self.mo.objs_dict[obj]))
            print(objective)

        return final_sol

    def evaluate(self, solution: CompositeSolution) -> CompositeSolution:

        # Evaluate objective function
        logger.debug('-----------------------------')
        logger.debug('---Evaluation of solutions---')

        # Update pyomo variables from jemtal solution
        update_pyomo_solution_from_jmetal(self.mo, solution, self.pyomo_to_jmetal_var_name)

        # Compute the objectives functions:
        for ind, obj in enumerate(self.mo.objs_dict):
            solution.objectives[ind] = pyo.value(self.mo.objs_dict[obj])

        # Compute the constraints:
        self.__evaluate_constraints(solution)

        logger.debug("fitness = %s", solution.objectives)

        # Evaluate constraints
        # self.__evaluate_constraints(solution)
        # check_feasible(self.data_problem, new_vars)

        return solution

    def __evaluate_constraints(self, solution: CompositeSolution) -> None:

        for ind, con in enumerate(self.mo.pyo_model.component_objects(pyo.Constraint)):
            if con.lb is None:
                lb = -1e100
            else:
                lb = con.lb
            if con.ub is None:
                ub = 1e100
            else:
                ub = con.ub
            solution.constraints[ind] = max(0, lb - pyo.value(con)) + max(0, pyo.value(con) - ub)

    def _compute_constraints(self):
        constraints = []
        for con in self.mo.pyo_model.component_objects(pyo.Constraint):
            if con.lb is None:
                lb = -1e100
            else:
                lb = con.lb
            if con.ub is None:
                ub = 1e100
            else:
                ub = con.ub
            constraints.append(max(0, lb - pyo.value(con)) + max(0, pyo.value(con) - ub))

        return constraints

    def name(self) -> str:
        return "Pyomo Problem"


def encode_solution_to_jmetal(mo):
    pymodel = mo.pyo_model
    lbfloatvars = []
    ubfloatvars = []
    binaryvars = []
    lbintegervars = []
    ubintegervars = []
    jmetal_to_pyomo_varname_cont = []
    jmetal_to_pyomo_varname_bin = []
    jmetal_to_pyomo_varname_int = []
    for v in pymodel.component_data_objects(pyo.Var, active=True):
        if v.is_continuous():
            lbfloatvars.append(v.lb)
            ubfloatvars.append(v.ub)
            jmetal_to_pyomo_varname_cont.append(v.name)
        elif v.is_binary():
            binaryvars.append(v.value)
            jmetal_to_pyomo_varname_bin.append(v.name)
        elif v.is_integer(v):
            lbintegervars.append(v.lb)
            ubintegervars.append(v.lb)
            jmetal_to_pyomo_varname_int.append(v.name)
        else:
            print("WARNING: Unknown type of variable")

    soljmetal = []
    jmetal_to_pyomo_varname = []
    if len(lbfloatvars) > 0:
        # TODO: BETTER WAY OF COMPUTING NUMBER OF CONSTRAINTS?
        float_sol = FloatSolution(lower_bound=lbfloatvars,
                                  upper_bound=ubfloatvars,
                                  number_of_objectives=len(mo.objs_dict.keys()),
                                  number_of_constraints=len(list(mo.pyo_model.component_objects(pyo.Constraint)))
                                  )
        soljmetal.append(float_sol)
        # TODO: INITIALIZE THE VARIABLES IN TERMS OF PYOMO VARIABLE VALUES
        float_sol.variables = [
            random.uniform(lbfloatvars[i] * 1.0, ubfloatvars[i] * 1.0)
            for i in range(len(lbfloatvars))
        ]
        jmetal_to_pyomo_varname.append(jmetal_to_pyomo_varname_cont)
    elif len(binaryvars) > 0:
        soljmetal.append(BinarySolution(number_of_variables=len(binaryvars),
                                        number_of_objectives=len(mo.objs_dict.keys()),
                                        number_of_constraints=len(list(mo.pyo_model.component_objects(pyo.Constraint))))
                         )
        jmetal_to_pyomo_varname.append(jmetal_to_pyomo_varname_bin)
    elif len(lbintegervars) > 0:
        soljmetal.append(IntegerSolution(lower_bound=lbintegervars,
                                         upper_bound=ubintegervars,
                                         number_of_objectives=len(mo.objs_dict.keys()),
                                         number_of_constraints=len(list(mo.pyo_model.component_objects(pyo.Constraint))))
                         )
        jmetal_to_pyomo_varname.append(jmetal_to_pyomo_varname_bin)

    return CompositeSolution(soljmetal), jmetal_to_pyomo_varname


def update_pyomo_solution_from_jmetal(mo, soljmetal, jmetal_to_pyomo_varname):

    for ind_var, sol in enumerate(soljmetal.variables):
        jmetal_variables = sol.variables
        pyomo_varnames = jmetal_to_pyomo_varname[ind_var]
        for jmetal_value, pyomo_varname in zip(jmetal_variables, pyomo_varnames):
            var_pyomo = mo.pyo_model.find_component(pyomo_varname)
            var_pyomo.set_value(
                jmetal_value
            )
    return