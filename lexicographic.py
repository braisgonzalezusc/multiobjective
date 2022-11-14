def lexicographic(mo, order):
    solved_model = None
    for i in order:
        objfun = mo.activate_one_objfun(i)
        solved_model = mo.solve()
        opt_val = objfun.expr()
        mo.add_constraint(objfun.expr <= opt_val)
    return solved_model
