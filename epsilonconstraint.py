import numpy as np
import pyomo.environ as pyo
import itertools
import lexicographic as lex
from multiobjective import *
from pyomo.opt import SolverStatus, TerminationCondition


def compute_payofftable(mo: MultiObjective):

    # TODO: ADD A SIMPLIER VERSION OF THIS CONSIDERING LESS ORDERS AND A FACTOR
    # for i in self.range_objectives():
    #for j_plus in range(i, i + self.objectives_count - 1):
    #    j = (j_plus % self.objectives_count) + 1
    #    if i != j:
    #         for j in self.range_objectives():
    #             self.nadir_values[j] = min(
    #                 round(min(self.payoff_table[i, j] for i in self.range_objectives()) * self.min_to_nadir_undercut, 0),
    #                 round(min(self.payoff_table[i, j] for i in self.range_objectives()) * (1/self.min_to_nadir_undercut), 0)
    #             )

    payofftable = []
    order = list(range(1, len(mo.objs_dict.keys()) + 1))
    order_permutations = itertools.permutations(order)
    for ord in order_permutations:
        print("order: ", ord)
        model_lex = lex.lexicographic(mo, ord)
        # We mutiply by -1 since later we model the problem as maximization one
        objs_val = {
            obj: -1*model_lex.__getattribute__(obj).expr() for obj in mo.objs_dict.keys()
        }
        # vars_val = {
        #    v.name: v.value for v in mo.pymodel.component_objects(pyo.Var)
        # }
        payofftable.append(list(objs_val.values()))

    # Get range for objectives functions:
    objs_min_max = {
        'min': np.asarray(payofftable).min(0), #np.minimum(*payofftable),
        'max': np.asarray(payofftable).max(0)  #np.maximum(*payofftable)
    }

    objs_range = {}
    for objind, objname in enumerate(mo.objs_dict):
        objs_range[objname] = [objs_min_max['min'][objind], objs_min_max['max'][objind]]

    return payofftable, objs_range


# import lexicographic
# model = ConcreteModel()
# model.x = Var()
# model.y = Var()
# model.objective1 = Objective(expr=(model.x-model.y-3)**2, sense=minimize)
# model.objective2 = Objective(expr=-((model.y-2)**2+1), sense=maximize)
# model.c1 = Constraint(expr=model.x>=0)
# model.c2 = Constraint(expr=model.y>=0)
# mo = MultiObjective(model)
# model_lex = lexicographic.lexicographic(mo,[2,1])
# print(model_lex.x.value)
# print(model_lex.y.value)


def epsilonconstr(mo, obj_main=None, payofftable=None, ngrid=5):
    solved_model = None
    pareto_front = []
    vars_sol = []
    flag_skip_points = []

    # Check what is the main objective function
    if obj_main is None:
        for k in mo.objs_dict.keys():  # Get the first objective in list
            obj_main = k
            break

    # Compute payofftable
    if payofftable is None:
        payofftable, objs_range = compute_payofftable(mo)

    # Grid points
    # TODO: INSTEAD OF WORKING WITH GRID POINTS DIRECTLY, USE THE INDICES FOR EACH OBJECTIVE!
    # THIS WILL AVOID APPROXIMATION PROBLEM TO SEE IF IT IS NECESSARY TO JUMP
    epsilon = {}
    stepsobj = {}
    for fobj in objs_range:
        if fobj != obj_main:
            epsilon[fobj] = list(np.linspace(objs_range[fobj][0], objs_range[fobj][1], num=ngrid))
            stepsobj[fobj] = (objs_range[fobj][1]-objs_range[fobj][0])/(ngrid-1)

    print("****epsilons****")
    print(epsilon)

    # Build original epsilon problem
    #pymodel = build_epsilon_model(mo, obj_main)

    # Build augmeconR problem
    pymodel = build_aumeconR_model(mo, obj_main, objs_range)

    grid_epsilons = list(itertools.product(*epsilon.values()))
    for step, eps_iter in enumerate(grid_epsilons):
        print('********')
        print('step: ', step)
        print('grid: ', eps_iter)
        if(eps_iter in flag_skip_points):
            print("Skip grid point: ", eps_iter)
            flag_skip_points.remove(eps_iter)
            continue

        # print(step,eps_iter)
        # Set the values to eps2, eps3,..., epsn
        for obj, epsval in zip(epsilon.keys(), eps_iter):
            pymodel.epsilons[obj] = epsval

        # TODO: CHECK IF THE MODEL IS FEASIBLE AND REMOVE STEPS IF IT IS NECESSARY
        result = mo.solver.solve(pymodel) # TODO: mo.solve return solved_model and status
        print("Status = %s" % result.solver.termination_condition)
        if (result.solver.status == SolverStatus.ok) and (
                result.solver.termination_condition == TerminationCondition.optimal):
            objs_val = {
                k: mo.objs_dict[k].expr() for k in pymodel.objs_dict
            }
            vars_val = {
                str(v): pyo.value(v) for v in pymodel.component_data_objects(pyo.Var, active=True)
            }
            print(objs_val)
            print(vars_val)

            pareto_front.append(list(objs_val.values()))
            vars_sol.append(vars_val)

            flag_skip_points = calculate_slack_jump(pymodel, stepsobj, flag_skip_points)

        elif result.solver.termination_condition == TerminationCondition.infeasible:
            flag_skip_points = calculate_infeasible_jump(pymodel, epsilon, flag_skip_points)
        else:
            # something else is wrong
            print(str(result.solver))

    return pareto_front, vars_sol


def build_epsilon_model(mo, obj_main):

    # We model the problems as a maximization one:
    mo.convert_to_maximization()

    # Set main objective as objective function
    mo.activate_objfun_by_name(obj_main)

    # Add constraints associated to epsilons: f2>= Eps2, f3>=Eps3,....fn>=Epsn
    # We are considering a minimization problem
    pymodel = mo.pyo_model
    pymodel.objs_dict = mo.objs_dict
    pymodel.objconset = list(mo.objs_dict.keys())
    pymodel.objconset.remove(obj_main)  # Remove the main objective function from the set
    pymodel.epsilons = pyo.Param(pymodel.objconset, within=pyo.Reals, mutable=True, initialize=0)

    def epsilon_constraints(pymodel, obj):
        return pymodel.objs_dict[obj].expr >= pymodel.epsilons[obj]
    pymodel.epsconstr = pyo.Constraint(pymodel.objconset, rule=epsilon_constraints)

    return pymodel

def build_aumeconR_model(mo, obj_main, obj_range):

    # We model the problems as a maximization one:
    mo.convert_to_maximization()

    # Set main objective as objective function
    #mo.activate_objfun_by_name(obj_main)

    # Add constraints associated to epsilons: f2>= Eps2, f3>=Eps3,....fn>=Epsn
    # We are considering a minimization problem
    pymodel = mo.pyo_model
    pymodel.objs_dict = mo.objs_dict
    pymodel.objconset = list(mo.objs_dict.keys())
    pymodel.objconset.remove(obj_main)  # Remove the main objective function from the set
    # Epsilon values of the constraints
    pymodel.epsilons = pyo.Param(pymodel.objconset, within=pyo.Reals, mutable=True, initialize=0)

    # New information relative to AUGMECON-R
    pymodel.eps = 10**(-6)
    obj_diff_range = {
        obj: obj_range[obj][1]-obj_range[obj][0] for obj in obj_range
    }
    pymodel.obj_diff_range = obj_diff_range
    # Define slack variables
    pymodel.slack_variables = pyo.Var(pymodel.objconset, within=pyo.NonNegativeReals, initialize=0)
    # Objective function weights:
    obj_weights = {
        obj_name: val for obj_name,val in zip(pymodel.objconset, [10**(-i) for i in range(0, len(pymodel.objconset))])
    }
    pymodel.weights = pyo.Param(pymodel.objconset, initialize=obj_weights)

    # Build new objective function:
    pymodel.obj_main = mo.objs_dict[obj_main].expr
    def ObjAumegconR(pymodel, obj_main):
        return pymodel.obj_main + \
            pymodel.eps*(
                sum(pymodel.weights[obj]*pymodel.slack_variables[obj]/pymodel.obj_diff_range[obj] for obj in pymodel.objconset))
    pymodel.augmeconRobj = Objective(rule=ObjAumegconR, sense=pyo.maximize, doc="augmeconRobj")
    mo.activate_objfun_by_name("augmeconRobj")

    def epsilon_constraints(pymodel, obj):
        return pymodel.objs_dict[obj].expr - pymodel.slack_variables[obj] == pymodel.epsilons[obj]
    pymodel.epsconstr = pyo.Constraint(pymodel.objconset, rule=epsilon_constraints)

    return pymodel

def calculate_slack_jump(pymodel, stepsobj, flag_skip_points):

    for obj in pymodel.objconset:
        # TODO: CONSIDER GRID POINTS AND JUMPS INTEGER TO AVOID PROBLEMS TO CHECK IF IT IS NECESSARY TO JUMP
        slack_val = pymodel.slack_variables[obj].value
        #if eps_iter == 0:
        #    slack_val = 20
        #else:
        #    slack_val = 0
        if slack_val > 0:
            skip_point = {
                obj: pymodel.epsilons[obj].value for obj in pymodel.objconset
            }
            actual_grid_obj = pymodel.epsilons[obj].value
            ingrid = True
            while ingrid:
                skip_point[obj] = skip_point[obj] + stepsobj[obj]
                if skip_point[obj] <= actual_grid_obj + slack_val:
                    flag_skip_points.append(tuple(skip_point.values()))
                else:
                    ingrid = False
        print("Slack skip points: ", flag_skip_points)
    return flag_skip_points


def calculate_infeasible_jump(pymodel, epsilons, flag_skip_points):

    infeas_grids_points = {
        obj: [point for point in epsilons[obj] if point >= pymodel.epsilons[obj].value] for obj in pymodel.objconset
    }
    #print('Infeasible grid points: ',infeas_grids_points)
    # Generate the different combination of grid points:
    # if (e2,...,ep) is infeasible => (e'2,...,e'p) is also infeasible with e'2>=e2,...,e'p>=ep
    infeas_grid_epsilons = list(itertools.product(*infeas_grids_points.values()))
    for step, eps_iter in enumerate(infeas_grid_epsilons):
        flag_skip_points.append(eps_iter)

    print("Infeasible skip points: ", flag_skip_points)
    return flag_skip_points


