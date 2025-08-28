// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // tanh, fabs, exp, isfinite
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "list.h" // list_node
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance

enum pa_method {
    PA_METHOD_LINEAR = 0,
    PA_METHOD_TANH,
    PA_METHOD_EXP,
    PA_METHOD_RECIP,
    PA_METHOD_SIGMOID
};

struct pa_params {
    int method;
    double pressure_advance, offset, linv, active_print_time;
    struct list_node node;
};

// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// Which is then "smoothed" using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(pa_position(x) * (smooth_time/2 - abs(t-x)) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//         / ((smooth_time/2)**2))

// Calculate the definitive integral of the motion formula:
//   position(t) = base + t * (start_v + t * half_accel)
static double
extruder_integrate(double base, double start_v, double half_accel
                   , double start, double end)
{
    double half_v = .5 * start_v, sixth_a = (1. / 3.) * half_accel;
    double si = start * (base + start * (half_v + start * sixth_a));
    double ei = end * (base + end * (half_v + end * sixth_a));
    return ei - si;
}

// Calculate the definitive integral of time weighted position:
//   weighted_position(t) = t * (base + t * (start_v + t * half_accel))
static double
extruder_integrate_time(double base, double start_v, double half_accel
                        , double start, double end)
{
    double half_b = .5 * base, third_v = (1. / 3.) * start_v;
    double eighth_a = .25 * half_accel;
    double si = start * start * (half_b + start * (third_v + start * eighth_a));
    double ei = end * end * (half_b + end * (third_v + end * eighth_a));
    return ei - si;
}

// Calculate non-linear pressure advance correction
static double
calc_nonlinear_pa(double pa_velocity, struct pa_params *pa)
{
    if (!pa || !isfinite(pa_velocity) || 
        !isfinite(pa->offset) || pa->offset == 0.0 || 
        !isfinite(pa->linv) || pa->linv == 0.0)
        return 0.0;
    
    double rel_v = pa_velocity / pa->linv;
    
    switch(pa->method) {
    case PA_METHOD_TANH:
        return pa->offset * tanh(rel_v);
        
    case PA_METHOD_EXP: {
        double abs_rel_v = fabs(rel_v);
        double sign = rel_v >= 0.0 ? 1.0 : -1.0;
        return pa->offset * sign * (1.0 - exp(-abs_rel_v));
    }
    
    case PA_METHOD_RECIP: {
        double abs_rel_v = fabs(rel_v);
        return pa->offset * rel_v / (1.0 + abs_rel_v);
    }
    
    case PA_METHOD_SIGMOID:
        if (fabs(rel_v) > 20.0) {
            rel_v = rel_v > 0 ? 20.0 : -20.0;
        }
        return pa->offset * (2.0 / (1.0 + exp(-rel_v)) - 1.0);
        
    default:
        return 0.0;
    }
}

static double
pa_velocity_integrate(struct move *m, double start, double end, double time_offset)
{
    if (start < 0.)
        start = 0.;
    if (end > m->move_t)
        end = m->move_t;
    
    double start_v = m->start_v;
    double ha = m->half_accel;
    double ivel = extruder_integrate(start_v, 2. * ha, 0., start, end);
    double wgt_vel = extruder_integrate_time(start_v, 2. * ha, 0., start, end);
    return wgt_vel - time_offset * ivel;
}

static double
pa_velocity_range_integrate(struct move *m, double move_time, double hst)
{
    // Calculate velocity integral for the current move
    double res = 0., start = move_time - hst, end = move_time + hst;
    res += pa_velocity_integrate(m, start, move_time, start);
    res -= pa_velocity_integrate(m, move_time, end, end);
    
    // Integrate over previous moves
    struct move *prev = m;
    while (unlikely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        res += pa_velocity_integrate(prev, start, prev->move_t, start);
    }
    
    // Integrate over future moves
    while (unlikely(end > m->move_t)) {
        end -= m->move_t;
        m = list_next_entry(m, node);
        res -= pa_velocity_integrate(m, 0., end, end);
    }
    return res;
}

struct extruder_stepper {
    struct stepper_kinematics sk;
    struct list_head pa_list;
    double half_smooth_time, inv_half_smooth_time2;
};

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);

    double base_pos = m->start_pos.x + move_get_distance(m, move_time);

    double hst = es->half_smooth_time;
    if (!hst)
        // Pressure advance not enabled
        return base_pos;
    
    // Determine PA parameters
    struct pa_params *pa = NULL;
    if (!list_empty(&es->pa_list)) {
        pa = list_last_entry(&es->pa_list, struct pa_params, node);
        while (unlikely(pa->active_print_time > m->print_time) &&
                !list_is_first(&pa->node, &es->pa_list)) {
            pa = list_prev_entry(pa, node);
        }
    }

    if (!pa || (pa->pressure_advance == 0.0 && pa->offset == 0.0)) {
        return base_pos;
    }

    double pa_velocity = 0.0;
    if (m->axes_r.y != 0.) {
        pa_velocity = pa_velocity_range_integrate(m, move_time, hst) * es->inv_half_smooth_time2;
    }

    double pa_adj = 0.0;
    if (pa->method == PA_METHOD_LINEAR) {
        pa_adj = pa->pressure_advance * pa_velocity;
    } else {
        pa_adj = calc_nonlinear_pa(pa_velocity, pa); 
    }

    return base_pos + pa_adj;
}

void __visible
extruder_set_pressure_advance(struct stepper_kinematics *sk, double print_time
                              , double pressure_advance, double smooth_time 
                              , int method, double offset, double linv)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = smooth_time * .5, old_hst = es->half_smooth_time;
    es->half_smooth_time = hst;
    es->sk.gen_steps_pre_active = es->sk.gen_steps_post_active = hst;

    // Cleanup old pressure advance parameters
    if (sk->last_flush_time > 0.0) {
        double cleanup_time = sk->last_flush_time - (old_hst > hst ? old_hst : hst);
        while (!list_empty(&es->pa_list)) {
            struct pa_params *first_pa = list_first_entry(
                    &es->pa_list, struct pa_params, node);
            if (list_is_last(&first_pa->node, &es->pa_list) ||
                list_next_entry(first_pa, node)->active_print_time >= cleanup_time)
                break;
            list_del(&first_pa->node);
            free(first_pa);
        }
    }

    if (! hst)
        return;
    es->inv_half_smooth_time2 = 1. / (hst * hst);

    // Check if we can reuse last parameters
    if (!list_empty(&es->pa_list)) {
        struct pa_params *last_pa = list_last_entry(&es->pa_list, struct pa_params, node);
        if (last_pa->pressure_advance == pressure_advance &&
            last_pa->method == method &&
            last_pa->offset == offset &&
            last_pa->linv == linv) {
            return;
        }
    }
    
    // Add new pressure advance parameters
    struct pa_params *pa = malloc(sizeof(*pa));
    memset(pa, 0, sizeof(*pa));
    pa->pressure_advance = pressure_advance;
    pa->active_print_time = print_time;
    pa->method = method;
    pa->offset = offset;
    pa->linv = linv != 0.0 ? linv : 1.0;
    list_add_tail(&pa->node, &es->pa_list);
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->sk.calc_position_cb = extruder_calc_position;
    es->sk.active_flags = AF_X;
    list_init(&es->pa_list);
    
    // Initialize with default linear PA parameters
    struct pa_params *pa = malloc(sizeof(*pa));
    memset(pa, 0, sizeof(*pa));
    pa->method = PA_METHOD_LINEAR;
    pa->linv = 1.0;
    list_add_tail(&pa->node, &es->pa_list);
    return &es->sk;
}

void __visible
extruder_stepper_free(struct stepper_kinematics *sk)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    while (!list_empty(&es->pa_list)) {
        struct pa_params *pa = list_first_entry(
                &es->pa_list, struct pa_params, node);
        list_del(&pa->node);
        free(pa);
    }
    free(sk);
}
