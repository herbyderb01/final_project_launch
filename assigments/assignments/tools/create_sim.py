"""create_sim.py creates a navigation simulation
"""
from typing import List


import numpy as np
import py_sim.dynamics.unicycle as uni
from py_sim.path_planning.path_generation import create_path
from py_sim.plotting.plot_constructor import create_plot_manifest
from py_sim.sensors.range_bearing import RangeBearingSensor
from py_sim.sim.generic_sim import SimParameters, start_simple_sim
from py_sim.sim.sim_modes import NavVectorFollower
from py_sim.tools.projections import LineCarrot
from py_sim.tools.sim_types import TwoDimArray, UnicycleControl, UnicycleState
from py_sim.vectorfield.vectorfields import G2GAvoid  # pylint: disable=unused-import
from py_sim.worlds.polygon_world import (
    generate_non_convex_obstacles,
)
from py_sim.tools.sim_types import UnicycleControl

NavSim = NavVectorFollower[uni.UnicycleState,
                           uni.UnicycleControl,
                           uni.UniVelVecParams,
                           uni.UnicycleParams]

def create_sim(create_initial_plan: bool = True) -> NavSim:
    """Creates a simulation with an optional plan

    Args:
        create_initial_plan: True indicates that an initial plan will be created, false indicates it will not
    """
    # Initialize the state and control
    vel_params = uni.UniVelVecParams(vd_field_max=5., k_wd= 5.)
    state_initial = UnicycleState(x = 0., y= 0., psi= 0.)

    # Create the vector field
    n_lines = 10 # Number of sensor lines
    vector_field = G2GAvoid(x_g=TwoDimArray(x=8., y=5.),
                            n_obs=n_lines,
                            v_max=vel_params.vd_field_max,
                            S=1.5,
                            R=1.,
                            sig=1.)

    # Create the obstacle world
    obstacle_world = generate_non_convex_obstacles()

    # Create the plan
    carrot = None
    plan = None
    if create_initial_plan:
        plan = create_path(start=TwoDimArray(x=state_initial.x, y=state_initial.y), end=vector_field.x_g, obstacle_world=obstacle_world, plan_type="voronoi")
    if plan is not None:
        line = np.array([plan[0], plan[1]])
        carrot = LineCarrot(line=line, s_dev_max=5., s_carrot=2.)

    # Create the manifest for the plotting
    plot_manifest = create_plot_manifest(initial_state=state_initial,
                                 y_limits=(-5, 10),
                                 x_limits=(-5, 25),
                                 position_dot=False,
                                 position_triangle=True,
                                 state_trajectory=True,
                                 time_series=True,
                                 vectorfield=vector_field,
                                 vector_res=0.5,
                                 world=obstacle_world,
                                 range_bearing_locations=True,
                                 range_bearing_lines=True,
                                 plan=plan,
                                 line_carrot=carrot)

    # Create the simulation
    params = SimParameters(initial_state=state_initial)
    params.sim_plot_period = 0.1
    params.sim_step = 0.1
    params.sim_update_period = 0.1
    params.tf = 5.
    sim = NavVectorFollower(params=params,
                            dynamics=uni.dynamics,                        # Unicycle
                            controller=uni.nonlinear_vector_field_control,
                            dynamic_params= uni.UnicycleParams(),
                            control_params=uni.UniNonlinearVecParams(k_v=1., k_w=1.),
                            n_inputs=UnicycleControl.n_inputs,
                            plots=plot_manifest,
                            vector_field=vector_field,
                            world=obstacle_world,
                            sensor=RangeBearingSensor(n_lines=n_lines, max_dist=4.),
                            carrot=carrot
                         )
    return sim

