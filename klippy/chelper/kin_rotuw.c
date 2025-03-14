// CoreXY UW rotational kinematics stepper pulse time generation

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

static double
cart_stepper_u_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    return move_get_coord(m, move_time).u;
}

static double
cart_stepper_w_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    return move_get_coord(m, move_time).w;
}

struct stepper_kinematics * __visible
rotuv_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'u') {
        sk->calc_position_cb = cart_stepper_u_calc_position;
        sk->active_flags = AF_U;
    } else if (axis == 'v') {
        sk->calc_position_cb = cart_stepper_w_calc_position;
        sk->active_flags = AF_W;
    }
    return sk;
}