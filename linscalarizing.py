def linscalarizing(mo, w):
    mo.weight_obj_fun(w)
    solved_model = mo.solve()
    return solved_model
