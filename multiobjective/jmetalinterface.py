import pyomo.environ as pyo
import random
from jmetal.core.problem import Problem
from jmetal.core.solution import FloatSolution, BinarySolution, IntegerSolution, CompositeSolution, PermutationSolution

from multiobjective.multiobjective import MultiObjective
import logging

_inf = float('inf')

logging.basicConfig()
logger = logging.getLogger('jmetalinterface')
#logger.setLevel("DEBUG")

class jmetalSol(object):

    def __init__(
            self,
            type,
            lower_bound,
            upper_bound,
            pyomo_varname
    ):
        self.type = type
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.pyomo_varname = pyomo_varname

class PyomoProblem(Problem):

    def __init__(self, mo: MultiObjective, initialize_fun=None, options=None):
        super(PyomoProblem, self).__init__()
        self.mo = mo
        #soljemtal, self.pyomo_to_jmetal_var_name = encode_solution_to_jmetal(mo)

        self.obj_directions = [self.MINIMIZE]*len(mo.objs_dict.keys())
        self.obj_labels = list(mo.objs_dict.keys())
        #self.objective = [mo.objs_dict[k].expr() for k in mo.objs_dict]

        # Variables encoding:
        self.variables_encoding = self._default_encode_solution_to_jmetal()
        self.jmetal_to_pyomo_varname = self._create_jmetal_to_pyomo_varname_map()

        # Initialization function:
        self.initialize_fun = initialize_fun

    def number_of_variables(self) -> int:
        return len(list(self.mo.pyo_model.component_data_objects(pyo.Var, active=True)))

    def number_of_objectives(self) -> int:
        return len(self.mo.objs_dict.keys())

    def number_of_constraints(self) -> int:
        return len(list(self.mo.pyo_model.component_objects(pyo.Constraint)))

    def create_solution(self, computeobj=True) -> CompositeSolution:

        logger.debug("------------------------")
        logger.debug("----INITIAL SOLUTION----")

        # Encode solution to jmetal:
        solution = self._create_jmetalsol_from_variables_encoding()

        solution = self._default_initialization(solution)
        final_sol = CompositeSolution(solution)

        # Compute objective function
        if computeobj:
            # Upddate pyomo var from jmetal solution vars
            self._update_pyomo_solution_from_jmetal_solution(final_sol)
            objective = []
            for ind, obj in enumerate(self.mo.objs_dict):
                objective.append(pyo.value(self.mo.objs_dict[obj]))
            logger.debug("Objective function: = %s", objective)

        return final_sol

    def evaluate(self, solution: CompositeSolution) -> CompositeSolution:

        # Evaluate objective function
        logger.debug('-----------------------------')
        logger.debug('---Evaluation of solutions---')

        # Update pyomo variables from jemtal solution
        #update_pyomo_solution_from_jmetal(self.mo, solution, self.jmetal_to_pyomo_varname)
        self._update_pyomo_solution_from_jmetal_solution(solution)

        # Compute the objectives functions:
        for ind, obj in enumerate(self.mo.objs_dict):
            solution.objectives[ind] = pyo.value(self.mo.objs_dict[obj])

        # Compute the constraints:
        self._evaluate_constraints(solution)

        logger.debug("Fitness = %s", solution.objectives)

        # Evaluate constraints
        # self.__evaluate_constraints(solution)
        # check_feasible(self.data_problem, new_vars)

        return solution

    def _evaluate_constraints(self, solution: CompositeSolution) -> None:

        for ind, con in enumerate(self.mo.pyo_model.component_objects(pyo.Constraint)):
            if con.lb is None:
                lb = -_inf
            else:
                lb = con.lb
            if con.ub is None:
                ub = _inf
            else:
                ub = con.ub
            solution.constraints[ind] = max(0, lb - pyo.value(con)) + max(0, pyo.value(con) - ub)

    # def _compute_constraints(self):
    #     constraints = []
    #     for con in self.mo.pyo_model.component_objects(pyo.Constraint):
    #         if con.lb is None:
    #             lb = -_inf
    #         else:
    #             lb = con.lb
    #         if con.ub is None:
    #             ub = _inf
    #         else:
    #             ub = con.ub
    #         constraints.append(max(0, lb - pyo.value(con)) + max(0, pyo.value(con) - ub))
    #
    #     return constraints


    def _default_encode_solution_to_jmetal(self):
        pymodel = self.mo.pyo_model
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
        if len(jmetal_to_pyomo_varname_cont) > 0:
            float_sol = jmetalSol(type=FloatSolution,
                                  lower_bound=lbfloatvars,
                                  upper_bound=ubfloatvars,
                                  pyomo_varname=jmetal_to_pyomo_varname_cont
                                  )
            soljmetal.append(float_sol)
        elif len(jmetal_to_pyomo_varname_bin) > 0:
            bin_sol = jmetalSol(type=BinarySolution,
                                lower_bound=[0] * len(jmetal_to_pyomo_varname_bin),
                                upper_bound=[1] * len(jmetal_to_pyomo_varname_bin),
                                pyomo_varname=jmetal_to_pyomo_varname_bin
                                )
            soljmetal.append(bin_sol)
        elif len(jmetal_to_pyomo_varname_int) > 0:
            int_sol = jmetalSol(type=IntegerSolution,
                                lower_bound=lbintegervars,
                                upper_bound=ubintegervars,
                                pyomo_varname=jmetal_to_pyomo_varname_int
                                )
            soljmetal.append(int_sol)
        #else:
        # print("ERROR: Unknown type of variable")
        #             raise Exception("Unknown type of variable for jmetal interface")

        return soljmetal

    def _create_jmetal_to_pyomo_varname_map(self):
        jmetal_to_pyomo_varname = []
        for sol in self.variables_encoding:
            jmetal_to_pyomo_varname.append(sol.pyomo_varname)

        return jmetal_to_pyomo_varname

    def _create_jmetalsol_from_variables_encoding(self):

        solution = []
        for sol in self.variables_encoding:
            tipo = sol.type
            if tipo.__name__ == "FloatSolution":
                solution.append(
                    FloatSolution(lower_bound=sol.lower_bound,
                                  upper_bound=sol.upper_bound,
                                  number_of_objectives=self.number_of_objectives(),
                                  number_of_constraints=self.number_of_constraints()
                                  )
                )
            elif tipo.__name__ == "BinarySolution":
                solution.append(
                    BinarySolution(number_of_variables=len(sol.lower_bound),
                                   number_of_objectives=self.number_of_objectives(),
                                   number_of_constraints=self.number_of_constraints()
                                   )
                )
            elif tipo.__name__ == "IntegerSolution":
                solution.append(
                    IntegerSolution(lower_bound=sol.lower_bound,
                                    upper_bound=sol.upper_bound,
                                    number_of_objectives=self.number_of_objectives(),
                                    number_of_constraints=self.number_of_constraints()
                                    )
                )

            return solution

    @staticmethod
    def _default_initialization(solution):

        for sol in solution:
            if issubclass(type(sol), FloatSolution):
                sol.variables = [
                    random.uniform(sol.lower_bound[i] * 1.0, sol.upper_bound[i] * 1.0)
                    for i in range(len(sol.variables))
                ]
            elif issubclass(type(sol), BinarySolution):
                sol.variables = [
                    True if random.randint(0, 1) == 0 else False for _ in range(len(sol.variables))
                ]
                # TODO: WHEN JMETAL WORKS WITH BinarySolution it considers only one variables
                # with a specific number of bits -> See below
                # new_solution.variables[0] = [True if random.randint(0, 1) == 0 else False for _ in
                #                             range(self.number_of_bits)]
            elif issubclass(type(sol), IntegerSolution):
                sol.variables = [
                    round(random.uniform(sol.lower_bound[i] * 1.0, sol.upper_bound[i] * 1.0))
                    for i in range(len(sol.variables))
                ]

            return solution

    def _update_pyomo_solution_from_jmetal_solution(self, soljmetal):

        pyomo_model = self.mo.pyo_model
        for ind_var, sol in enumerate(soljmetal.variables):
            jmetal_variables = sol.variables
            pyomo_varnames = self.jmetal_to_pyomo_varname[ind_var]
            for jmetal_value, pyomo_varname in zip(jmetal_variables, pyomo_varnames):
                var_pyomo = pyomo_model.find_component(pyomo_varname)
                var_pyomo.set_value(
                    jmetal_value
                )

    def name(self) -> str:
        return "Pyomo Problem through jmetal interface"

# def encode_solution_to_jmetal(mo):
#     pymodel = mo.pyo_model
#     lbfloatvars = []
#     ubfloatvars = []
#     binaryvars = []
#     lbintegervars = []
#     ubintegervars = []
#     jmetal_to_pyomo_varname_cont = []
#     jmetal_to_pyomo_varname_bin = []
#     jmetal_to_pyomo_varname_int = []
#     for v in pymodel.component_data_objects(pyo.Var, active=True):
#         if v.is_continuous():
#             lbfloatvars.append(v.lb)
#             ubfloatvars.append(v.ub)
#             jmetal_to_pyomo_varname_cont.append(v.name)
#         elif v.is_binary():
#             binaryvars.append(v.value)
#             jmetal_to_pyomo_varname_bin.append(v.name)
#         elif v.is_integer(v):
#             lbintegervars.append(v.lb)
#             ubintegervars.append(v.lb)
#             jmetal_to_pyomo_varname_int.append(v.name)
#         else:
#             print("WARNING: Unknown type of variable")
#
#     soljmetal = []
#     jmetal_to_pyomo_varname = []
#     if len(jmetal_to_pyomo_varname_cont) > 0:
#         # TODO: BETTER WAY OF COMPUTING NUMBER OF CONSTRAINTS?
#         float_sol = FloatSolution(lower_bound=lbfloatvars,
#                                   upper_bound=ubfloatvars,
#                                   number_of_objectives=len(mo.objs_dict.keys()),
#                                   number_of_constraints=len(list(mo.pyo_model.component_objects(pyo.Constraint)))
#                                   )
#         soljmetal.append(float_sol)
#         # TODO: INITIALIZE THE VARIABLES IN TERMS OF PYOMO VARIABLE VALUES
#         float_sol.variables = [
#             random.uniform(lbfloatvars[i] * 1.0, ubfloatvars[i] * 1.0)
#             for i in range(len(lbfloatvars))
#         ]
#         jmetal_to_pyomo_varname.append(jmetal_to_pyomo_varname_cont)
#     elif len(jmetal_to_pyomo_varname_bin) > 0:
#         soljmetal.append(BinarySolution(number_of_variables=len(binaryvars),
#                                         number_of_objectives=len(mo.objs_dict.keys()),
#                                         number_of_constraints=len(list(mo.pyo_model.component_objects(pyo.Constraint))))
#                          )
#         jmetal_to_pyomo_varname.append(jmetal_to_pyomo_varname_bin)
#     elif len(jmetal_to_pyomo_varname_int) > 0:
#         soljmetal.append(IntegerSolution(lower_bound=lbintegervars,
#                                          upper_bound=ubintegervars,
#                                          number_of_objectives=len(mo.objs_dict.keys()),
#                                          number_of_constraints=len(list(mo.pyo_model.component_objects(pyo.Constraint))))
#                          )
#         jmetal_to_pyomo_varname.append(jmetal_to_pyomo_varname_int)
#
#     return CompositeSolution(soljmetal), jmetal_to_pyomo_varname



def update_initsol(soljmetal, initsol, jmetal_to_pyomo_varname):

    for ind_var, sol in enumerate(soljmetal.variables):
        jmetal_variables = sol.variables
        pyomo_varnames = jmetal_to_pyomo_varname[ind_var]
        for ind, name in enumerate(pyomo_varnames):
            sol.variables[ind] = initsol[name]



