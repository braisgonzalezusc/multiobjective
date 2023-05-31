def lexicographic(mo, order):
    solved_model = None
    for i in order:
        objfun = mo.activate_one_objfun(i)
        solved_model = mo.solve()
        # TODO: CHECK IF THE SOLUTION IS FEASIBLE
        # Added method mo.check_feasibility() but I do not know why it is necessary to check feasibility in this case
        opt_val = objfun.expr()
        mo.add_constraint(objfun.expr <= opt_val)
    # Deactivate constraints:
    # TODO: MAYBE IS BETTER TO WORK WITH A CLONE OF THE ORIGINAL MODEL
    mo.restore_original_model()  # this seems to work without having to deactivate constraints
    return solved_model
