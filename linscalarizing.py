def linscalarizing(mo, w):
    mo.weight_obj_fun(w)
    solved_model = mo.solve()
    mo.restore_original_model()
    return solved_model
