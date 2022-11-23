def lexicographic(mo, order):
    solved_model = None
    for i in order:
        objfun = mo.activate_one_objfun(i)
        solved_model = mo.solve()
        # TODO: CHECK IF THE SOLUTION IS FEASIBLE
        opt_val = objfun.expr()
        mo.add_constraint(objfun.expr <= opt_val)
    # Deactivate constraints:
    # TODO: MAYBE IS BETTER TO WORK WITH A CLONE OF THE ORIGINAL MODEL
    mo.deactivate_cons_uuid()
    return solved_model
