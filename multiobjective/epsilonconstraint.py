import numpy as np
import pyomo.environ as pyo
import itertools
import multiobjective.lexicographic as lex
from multiobjective.multiobjective import *
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

    counter_obj = {
        obj: [i for i in range(len(epsilon[obj]))] for obj in epsilon
    }
    grid_counters = list(itertools.product(*counter_obj.values()))
    for step, count_iter in enumerate(grid_counters):
        print('********')
        print('step: ', step)
        print('grid: ', count_iter)
        if (count_iter in flag_skip_points):
            print("Skip grid point: ", count_iter)
            flag_skip_points.remove(count_iter)
            continue

        for obj, ind in zip(epsilon.keys(), count_iter):
            pymodel.epsilons[obj] = epsilon[obj][ind]

        #  TODO: CHECK IF THE MODEL IS FEASIBLE AND REMOVE STEPS IF IT IS NECESSARY (DONE)
        result = mo.solver.solve(pymodel) #  TODO: mo.solve return solved_model and status
        print("Status = %s" % result.solver.termination_condition)
        if (result.solver.status == SolverStatus.ok) and (
                result.solver.termination_condition == TerminationCondition.optimal):
            objs_val = {
                k: -mo.objs_dict[k].expr() for k in pymodel.objs_dict # TODO: RETURN SIGN OF OBJECTIVE VALUES DEPENDING ON ORIGINAL SENSE
            }
            vars_val = {
                str(v): pyo.value(v) for v in pymodel.component_data_objects(pyo.Var, active=True)
            }
            print(objs_val)
            print(vars_val)

            pareto_front.append(list(objs_val.values()))
            vars_sol.append(vars_val)

            flag_skip_points = calculate_slack_jump(pymodel, count_iter, epsilon, flag_skip_points)

        elif result.solver.termination_condition == TerminationCondition.infeasible:
            flag_skip_points = calculate_infeasible_jump(pymodel, count_iter, epsilon, flag_skip_points)
        else:
            # something else is wrong
            print(str(result.solver))
            break

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
        obj_name: val for obj_name, val in zip(pymodel.objconset, [10**(-i) for i in range(0, len(pymodel.objconset))])
    }
    pymodel.weights = pyo.Param(pymodel.objconset, initialize=obj_weights)

    # Build new objective function:
    pymodel.obj_main = mo.objs_dict[obj_main].expr

    def ObjAumegconR(pymodel):
        return pymodel.obj_main + \
            pymodel.eps*(
                sum(pymodel.weights[obj]*pymodel.slack_variables[obj]/pymodel.obj_diff_range[obj] for obj in pymodel.objconset))
    pymodel.augmeconRobj = Objective(rule=ObjAumegconR, sense=pyo.maximize, doc="augmeconRobj")
    mo.activate_objfun_by_name("augmeconRobj")

    def epsilon_constraints(pymodel, obj):
        return pymodel.objs_dict[obj].expr - pymodel.slack_variables[obj] == pymodel.epsilons[obj]
    pymodel.epsconstr = pyo.Constraint(pymodel.objconset, rule=epsilon_constraints)

    return pymodel

def calculate_slack_jump(pymodel, count_iter, epsilon, flag_skip_points):

    slack_skip_points = []
    for ind, obj in enumerate(pymodel.objconset):
        # Get slack value
        slack_val = pymodel.slack_variables[obj].value
        # if count_iter == (0,):
        #     slack_val = 13
        # else:
        #     slack_val = 0
        if slack_val > 0:
            ind_obj = count_iter[ind]
            max_obj = epsilon[obj][ind]+slack_val
            skip_point = list(count_iter)
            for i in range(ind_obj+1, len(epsilon[obj])):
                if epsilon[obj][i] <= max_obj:
                    skip_point[ind] = i
                    if skip_point not in flag_skip_points:
                        slack_skip_points.append(tuple(skip_point))
                else:  # Assuming that epsilon values are ordered from smaller to bigger numbers
                    break

    if(len(slack_skip_points)>0):
        print("New slack skip points: ", slack_skip_points)
        flag_skip_points += slack_skip_points
    return flag_skip_points


def calculate_infeasible_jump(pymodel, count_iter, epsilons, flag_skip_points):

    # Generate the different combination of grid points:
    # if (e2,...,ep) is infeasible => (e'2,...,e'p) is also infeasible with e'2>=e2,...,e'p>=ep
    infeas_grids_points = {
        obj: [i for i in range(count_iter[ind], len(epsilons[obj])) if epsilons[obj][i] >= pymodel.epsilons[obj].value]
            for ind, obj in enumerate(pymodel.objconset)
    }
    #print('Infeasible grid points: ',infeas_grids_points)
    infeas_skip_points = []
    for point_iter in list(itertools.product(*infeas_grids_points.values())):
        if point_iter not in flag_skip_points:
            infeas_skip_points.append(point_iter)

    # Remove current point from the list (necessary to avoid problems when one objective function is in the final index)
    infeas_skip_points.remove(count_iter)
    if (len(infeas_skip_points) > 0):
        print("New Infeasible skip points: ", infeas_skip_points)
        flag_skip_points += infeas_skip_points

    return flag_skip_points


