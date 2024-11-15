#!/usr/bin/env python3

"""
    PRS_WORLDINIT.PY - CS4048 ROBOTICS

    DESIGNED AND WRITTEN BY GROUP XXX IN 2024
"""


import os
import numpy as np

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation import (
    ConstantVelocityExecutor,
    AStarPlanner,
    PRMPlanner,
    RRTPlanner
)
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

data_folder = get_data_folder()

GLOBAL_SCALE_VALUE = 7

def create_world():
    """
        Creates a world.
    """

    # Core idea: rooms indicate cities/towns, hallways indicate trainlines, locations indicate stations, objects indicate people.

    # Generate a new, blank Pyrobosim World.
    world_env = World()
    world_env.name = "UK_NetworkRail"

    # Set the location and object metadata
    world_env.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    world_env.add_room(
        name="Aberdeen",
        footprint=[
            [
                -2.07170063146296 * GLOBAL_SCALE_VALUE,
                57.18916183247538 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.07661007751247 * GLOBAL_SCALE_VALUE,
                57.17850167405052 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.0791718518682956 * GLOBAL_SCALE_VALUE,
                57.16109980377763 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.0776779658361306 * GLOBAL_SCALE_VALUE,
                57.15262566922996 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.072974224127506 * GLOBAL_SCALE_VALUE,
                57.14542832504807 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.067616074797371 * GLOBAL_SCALE_VALUE,
                57.14298910914252 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.0552080143864657 * GLOBAL_SCALE_VALUE,
                57.14264141101489 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.051352498509374 * GLOBAL_SCALE_VALUE,
                57.14008638300629 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.047279054663278 * GLOBAL_SCALE_VALUE,
                57.140317660287195 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.0474940748581503 * GLOBAL_SCALE_VALUE,
                57.13741434536735 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.0496493177210766 * GLOBAL_SCALE_VALUE,
                57.12801027625949 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1174885825518004 * GLOBAL_SCALE_VALUE,
                57.121996336556634 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.147750909995409 * GLOBAL_SCALE_VALUE,
                57.14244643548389 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.150472087845003 * GLOBAL_SCALE_VALUE,
                57.15768747054415 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.139425841435525 * GLOBAL_SCALE_VALUE,
                57.165078629229725 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1460968068365105 * GLOBAL_SCALE_VALUE,
                57.17322240354315 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.152519529678216 * GLOBAL_SCALE_VALUE,
                57.17543612767648 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.151241150218908 * GLOBAL_SCALE_VALUE,
                57.18018568283003 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1424499967889403 * GLOBAL_SCALE_VALUE,
                57.18522488716826 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1253877649235733 * GLOBAL_SCALE_VALUE,
                57.183716663332234 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1002141326436004 * GLOBAL_SCALE_VALUE,
                57.189863917122665 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.0712709900856225 * GLOBAL_SCALE_VALUE,
                57.189279119240695 * GLOBAL_SCALE_VALUE
            ]
        ],
        wall_width=0.00025 * GLOBAL_SCALE_VALUE,
        color=[0, 0, 0.5]
    )

    world_env.add_room(
        name="Dundee",
        footprint=[
            [
                -3.063571117823358 * GLOBAL_SCALE_VALUE,
                56.46490315370875 * GLOBAL_SCALE_VALUE
            ],
            [
                -3.059410148116399 * GLOBAL_SCALE_VALUE,
                56.4709720075937 * GLOBAL_SCALE_VALUE
            ],
            [
                -3.030484403524639 * GLOBAL_SCALE_VALUE,
                56.478321298351744 * GLOBAL_SCALE_VALUE
            ],
            [
                -3.0188543959727383 * GLOBAL_SCALE_VALUE,
                56.48245333594622 * GLOBAL_SCALE_VALUE
            ],
            [
                -3.0057462687343843 * GLOBAL_SCALE_VALUE,
                56.48174385913691 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.991128996929916 * GLOBAL_SCALE_VALUE,
                56.48412173882613 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.952587578284607 * GLOBAL_SCALE_VALUE,
                56.48034525473125 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.9327030998488794 * GLOBAL_SCALE_VALUE,
                56.47774903172251 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.9246828014855737 * GLOBAL_SCALE_VALUE,
                56.474067773564116 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.9312686297712105 * GLOBAL_SCALE_VALUE,
                56.46848019917428 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.959536874119749 * GLOBAL_SCALE_VALUE,
                56.463949679356176 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.9629559054721994 * GLOBAL_SCALE_VALUE,
                56.458938837133275 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.9737653335777168 * GLOBAL_SCALE_VALUE,
                56.454762726177734 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.989312898034086 * GLOBAL_SCALE_VALUE,
                56.45142002706598 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.9975204387079373 * GLOBAL_SCALE_VALUE,
                56.45142006177747 * GLOBAL_SCALE_VALUE
            ],
            [
                -3.0392084490281945 * GLOBAL_SCALE_VALUE,
                56.451299775679246 * GLOBAL_SCALE_VALUE
            ],
            [
                -3.0443924091730423 * GLOBAL_SCALE_VALUE,
                56.452254806238244 * GLOBAL_SCALE_VALUE
            ]
        ],
        wall_width=0.00025 * GLOBAL_SCALE_VALUE,
        color=[0, 0, 0.5]
    )

    world_env.add_room(
        name="Portlethen",
        footprint=[
            [
                -2.1460335132588 * GLOBAL_SCALE_VALUE,
                57.07239373976901 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.148821263900402 * GLOBAL_SCALE_VALUE,
                57.065428618039675 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1532765233456814 * GLOBAL_SCALE_VALUE,
                57.06086608504674 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1505766296126865 * GLOBAL_SCALE_VALUE,
                57.05052164675436 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1376166206273695 * GLOBAL_SCALE_VALUE,
                57.0507035956735 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1235185769474185 * GLOBAL_SCALE_VALUE,
                57.04802283458034 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1183083640709413 * GLOBAL_SCALE_VALUE,
                57.04996511707952 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1102668863345286 * GLOBAL_SCALE_VALUE,
                57.05377546624811 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.110502553665725 * GLOBAL_SCALE_VALUE,
                57.05679771630477 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.104715228208562 * GLOBAL_SCALE_VALUE,
                57.05722130081409 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.103463409023078 * GLOBAL_SCALE_VALUE,
                57.061468412783285 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1064260472890055 * GLOBAL_SCALE_VALUE,
                57.06159586372618 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.101228354254914 * GLOBAL_SCALE_VALUE,
                57.06491886084771 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.0920228052718812 * GLOBAL_SCALE_VALUE,
                57.06781116321625 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.092815165971274 * GLOBAL_SCALE_VALUE,
                57.07353743442121 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.09653321717542 * GLOBAL_SCALE_VALUE,
                57.07357709998152 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.10223280063002 * GLOBAL_SCALE_VALUE,
                57.07692993509011 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.120265023726347 * GLOBAL_SCALE_VALUE,
                57.07985976612065 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1248813572808842 * GLOBAL_SCALE_VALUE,
                57.079798206786734 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.134562901705408 * GLOBAL_SCALE_VALUE,
                57.07404492288285 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1449001945174473 * GLOBAL_SCALE_VALUE,
                57.07464806329065 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.146020785049643 * GLOBAL_SCALE_VALUE,
                57.07238523716504 * GLOBAL_SCALE_VALUE
            ]
        ],
        wall_width=0.00025 * GLOBAL_SCALE_VALUE,
        color=[0, 0, 0.5]
    )

    world_env.add_room(
        name="Stonehaven",
        footprint=[
            [
                -2.2102546555689457 * GLOBAL_SCALE_VALUE,
                56.976558815439574 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.2099033925729827 * GLOBAL_SCALE_VALUE,
                56.97092627618284 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.2035323398058893 * GLOBAL_SCALE_VALUE,
                56.96919905762104 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.205835682471246 * GLOBAL_SCALE_VALUE,
                56.966525385920875 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.205402022690805 * GLOBAL_SCALE_VALUE,
                56.96251416829185 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1999106397377943 * GLOBAL_SCALE_VALUE,
                56.961017660292185 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1942818794051107 * GLOBAL_SCALE_VALUE,
                56.95905046822645 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1998407651940965 * GLOBAL_SCALE_VALUE,
                56.95459635123561 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.190255779024085 * GLOBAL_SCALE_VALUE,
                56.95197381326898 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.191558142970621 * GLOBAL_SCALE_VALUE,
                56.94945802393357 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1997030002193867 * GLOBAL_SCALE_VALUE,
                56.94847512944142 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.1932716949501696 * GLOBAL_SCALE_VALUE,
                56.94653753820626 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.2095990405955206 * GLOBAL_SCALE_VALUE,
                56.94363973761725 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.2339644661308284 * GLOBAL_SCALE_VALUE,
                56.951982663972586 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.240290425036676 * GLOBAL_SCALE_VALUE,
                56.96551384292147 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.239144161125097 * GLOBAL_SCALE_VALUE,
                56.97123574040293 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.2320960632633273 * GLOBAL_SCALE_VALUE,
                56.97484112951716 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.224330611647929 * GLOBAL_SCALE_VALUE,
                56.97648638460487 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.2103980393041525 * GLOBAL_SCALE_VALUE,
                56.97648061065274 * GLOBAL_SCALE_VALUE
            ]
        ],
        wall_width=0.00025 * GLOBAL_SCALE_VALUE,
        color=[0, 0, 0.5]
    )

    world_env.add_room(
        name="Laurencekirk",
        footprint=[
            [
                -2.449235697677352 * GLOBAL_SCALE_VALUE,
                56.842108746878324 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.456760103950728 * GLOBAL_SCALE_VALUE,
                56.845474835771824 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4772539346132305 * GLOBAL_SCALE_VALUE,
                56.83794012681247 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.482479297550981 * GLOBAL_SCALE_VALUE,
                56.8325405065907 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4933373416991174 * GLOBAL_SCALE_VALUE,
                56.819430648822475 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4771704330357522 * GLOBAL_SCALE_VALUE,
                56.82252533708109 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.461693330175194 * GLOBAL_SCALE_VALUE,
                56.82670138633287 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4535682821404237 * GLOBAL_SCALE_VALUE,
                56.83326705617279 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4485759790101156 * GLOBAL_SCALE_VALUE,
                56.84188104838907 * GLOBAL_SCALE_VALUE
            ]
        ],
        wall_width=0.00025 * GLOBAL_SCALE_VALUE,
        color=[0, 0, 0.5]
    )

    world_env.add_room(
        name="Montrose",
        footprint=[
            [
                -2.4707823112536573 * GLOBAL_SCALE_VALUE,
                56.73508683630186 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4389110454449963 * GLOBAL_SCALE_VALUE,
                56.73456743907286 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.447083740519872 * GLOBAL_SCALE_VALUE,
                56.71589685018975 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4520431354404764 * GLOBAL_SCALE_VALUE,
                56.70400946139671 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4615251737149038 * GLOBAL_SCALE_VALUE,
                56.70324142705775 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.466914071482762 * GLOBAL_SCALE_VALUE,
                56.704182286310726 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.467344273617101 * GLOBAL_SCALE_VALUE,
                56.70193671209208 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.436411707543442 * GLOBAL_SCALE_VALUE,
                56.70223094504587 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.435624539354535 * GLOBAL_SCALE_VALUE,
                56.70020397214142 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.441068935787655 * GLOBAL_SCALE_VALUE,
                56.69851530015518 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.44224054909364 * GLOBAL_SCALE_VALUE,
                56.695970857670915 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4414579968631074 * GLOBAL_SCALE_VALUE,
                56.69373230932433 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4794167670603144 * GLOBAL_SCALE_VALUE,
                56.69580644770747 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.488451956105024 * GLOBAL_SCALE_VALUE,
                56.701070406358696 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.4858638100331234 * GLOBAL_SCALE_VALUE,
                56.7214761566147 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.471375237403663 * GLOBAL_SCALE_VALUE,
                56.735130903277025 * GLOBAL_SCALE_VALUE
            ]
        ],
        wall_width=0.00025 * GLOBAL_SCALE_VALUE,
        color=[0, 0, 0.5]
    )

    world_env.add_room(
        name="Arbroath",
        footprint=[
            [
                -2.5750007317033408 * GLOBAL_SCALE_VALUE,
                56.57595172261642 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.5665478822310206 * GLOBAL_SCALE_VALUE,
                56.57490956733048 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.544686455387364 * GLOBAL_SCALE_VALUE,
                56.564302600782156 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.5540219665511756 * GLOBAL_SCALE_VALUE,
                56.559530299195245 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.560825485085843 * GLOBAL_SCALE_VALUE,
                56.561123863997295 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.572796134345623 * GLOBAL_SCALE_VALUE,
                56.55866104079965 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.583316551261788 * GLOBAL_SCALE_VALUE,
                56.55388403509335 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.5928399563345863 * GLOBAL_SCALE_VALUE,
                56.551024588147214 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.6000510107967614 * GLOBAL_SCALE_VALUE,
                56.55142543011158 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.6084221980283644 * GLOBAL_SCALE_VALUE,
                56.54903508873866 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.6173671036948747 * GLOBAL_SCALE_VALUE,
                56.543547436149254 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.6259502368903895 * GLOBAL_SCALE_VALUE,
                56.556370206143214 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.621320199950702 * GLOBAL_SCALE_VALUE,
                56.56422585602695 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.6184175942537706 * GLOBAL_SCALE_VALUE,
                56.573017426709384 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.599021269482847 * GLOBAL_SCALE_VALUE,
                56.57722603264736 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.5876499092145195 * GLOBAL_SCALE_VALUE,
                56.57748035134122 * GLOBAL_SCALE_VALUE
            ],
            [
                -2.575431310565392 * GLOBAL_SCALE_VALUE,
                56.576031517087586 * GLOBAL_SCALE_VALUE
            ]
        ],
        wall_width=0.00025 * GLOBAL_SCALE_VALUE,
        color=[0, 0, 0.5]
    )

    world_env.add_hallway(
        room_start="Aberdeen",
        room_end="Portlethen",
        width=0.001 * GLOBAL_SCALE_VALUE,
        wall_width=0.00025  * GLOBAL_SCALE_VALUE,
        name="ECML_ABD_PLN",
        color=[0.25, 0, 0]
    )

    world_env.add_hallway(
        room_start="Portlethen",
        room_end="Stonehaven",
        name="ECML_PLN_STN",
        width=0.001 * GLOBAL_SCALE_VALUE,
        wall_width=0.00025  * GLOBAL_SCALE_VALUE,
        color=[0.25, 0, 0]
    )

    world_env.add_hallway(
        room_start="Stonehaven",
        room_end="Laurencekirk",
        name="ECML_STN_LAU",
        width=0.001,
        wall_width=0.00025,
        color=[0.25, 0, 0]
    )

    world_env.add_hallway(
        room_start="Laurencekirk",
        room_end="Montrose",
        name="ECML_PLN_MTS",
        width=0.001 * GLOBAL_SCALE_VALUE,
        wall_width=0.00025  * GLOBAL_SCALE_VALUE,
        color=[0.25, 0, 0]
    )

    world_env.add_hallway(
        room_start="Montrose",
        room_end="Arbroath",
        name="ECML_MNT_ARB",
        width=0.001 * GLOBAL_SCALE_VALUE,
        wall_width=0.00025  * GLOBAL_SCALE_VALUE,
        color=[0.25, 0, 0]
    )

    world_env.add_hallway(
        room_start="Arbroath",
        room_end="Dundee",
        name="ECML_ARB_DEE",
        width=0.001 * GLOBAL_SCALE_VALUE,
        wall_width=0.00025  * GLOBAL_SCALE_VALUE,
        color=[0.25, 0, 0]
    )

    # Create path planner
    planner_config = {
        "world": world_env,
        "bidirectional": True,
        "rrt_connect": True,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 0.5,
        "compress_path": True,
    }
    path_planner = RRTPlanner(**planner_config)

    # path_planner=AStarPlanner(
    #     world=world_env,
    #     grid_resolution=0.0005,
    #     grid_inflation_radius=0.00049,
    #     heuristic="euclidean",
    #     diagonal_motion=True,
    #     compress_path=True
    # )

    robot = Robot(
        name="SCOTRAIL_1FAA_ABD_DEE",
        radius=0.00005 * GLOBAL_SCALE_VALUE,
        max_linear_velocity=0.00001 * GLOBAL_SCALE_VALUE,
        max_angular_velocity=0.00001 * GLOBAL_SCALE_VALUE,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world_env.add_robot(robot, loc="Aberdeen")
    
    return world_env

    # world = World()

    # r1coords = [(100, 100), (100, 0), (0,0)]
    # world.add_room(name="EDB_EDINBURGH", footprint=r1coords, color=[0, 0, 0])

    # robot = Robot(
    #     name="1F37_ABD-EDB",
    #     radius=2,
    #     path_executor=ConstantVelocityExecutor(),
    # )

    # world.add_robot(robot, loc="EDB_EDINBURGH")

    # return world

if __name__ == "__main__":
    world = create_world()
    start_gui(world)

    world.add_robot(Robot(
        name="SCOTRAIL_1FBB_DEE_ABD",
        radius=0.00005,
    ), loc="Dundee")    
