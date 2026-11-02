"""
The names and values RoboCasa uses for the parts of its kitchens, objects and robot that
the atomic tasks refer to.
"""

from __future__ import annotations

from enum import StrEnum

# %% RoboCasa's own robot


class RoboCasaRobotFrame(StrEnum):
    """
    The frames of RoboCasa's own robot that its success checks read.
    """

    BASE = "mobilebase0_base"
    """
    The body of the mobile base, whose pose a navigation task is judged by.
    """

    GRIPPER = "right"
    """
    The gripper whose end effector site the checks of a gripper being far read.
    """


class RoboCasaRobotBodyPrefix(StrEnum):
    """
    How the names of the bodies of RoboCasa's own robot begin.
    """

    ARM = "robot0_"
    """
    The bodies of the arm.
    """

    MOBILE_BASE = "mobilebase0_"
    """
    The bodies of the mobile base.
    """

    GRIPPER = "gripper0_"
    """
    The bodies of the gripper.
    """


# %% fixtures


class FixtureJointRole(StrEnum):
    """
    The keys a fixture's ``_joint_names`` gives its joints by.
    """

    TEMPERATURE = "temperature"
    """
    The temperature knob of an oven or a toaster oven.
    """

    TIMER = "time"
    """
    The timer knob of a toaster oven.
    """

    DOOR = "door"
    """
    The door of a toaster oven.
    """

    RACK = "rack"
    """
    The rack of a dishwasher.
    """

    SWITCH = "switch"
    """
    The power switch of an electric kettle.
    """

    LID = "lid"
    """
    The lid of an electric kettle.
    """

    HEAD = "head"
    """
    The head of a stand mixer.
    """

    LEVER = "lever"
    """
    A lever of a toaster, followed by the index of its slot pair.
    """

    def of_slot_pair(self, slot_pair: int) -> str:
        """
        :param slot_pair: The index of a toaster's slot pair.
        :return: The key of this role's joint of that slot pair.
        """
        return f"{self}_{slot_pair}"


class SinkJoint(StrEnum):
    """
    How the names of a sink's joints end, after the sink's naming prefix.
    """

    HANDLE = "handle_joint"
    """
    The faucet handle lifted to let the water run.
    """

    SPOUT = "spout_joint"
    """
    The spout swung to either side.
    """

    TEMPERATURE_HANDLE = "handle_temp_joint"
    """
    The faucet handle swung towards hot or cold water.
    """


class RackKind(StrEnum):
    """
    The kinds of shelves an oven or a toaster oven slides out, as their joints are
    named.
    """

    RACK = "rack"
    """
    A wire rack.
    """

    TRAY = "tray"
    """
    A tray.
    """

    def joint_name(self, naming_prefix: str, level: int) -> str:
        """
        :param naming_prefix: The naming prefix of the oven.
        :param level: The level of the shelf, 0 at the bottom.
        :return: The name of the joint the shelf of this kind at that level slides on.
        """
        return f"{naming_prefix}{self}{level}_joint"


class SpoutSide(StrEnum):
    """
    The sides RoboCasa asks a sink's spout to be swung to.
    """

    LEFT = "left"
    """
    To the left.
    """

    RIGHT = "right"
    """
    To the right.
    """


class WaterTemperature(StrEnum):
    """
    The water temperatures RoboCasa asks a faucet to run at.
    """

    HOT = "hot"
    """
    Hot water.
    """

    COLD = "cold"
    """
    Cold water.
    """


class MicrowaveButton(StrEnum):
    """
    How the names of a microwave's button geoms end, after the microwave's name.
    """

    START = "start_button"
    """
    The button that starts the microwave.
    """

    STOP = "stop_button"
    """
    The button that stops it.
    """


class FixturePartSuffix(StrEnum):
    """
    How the names of fixture parts end that the tasks refer to, after the name or the
    naming prefix of their fixture.
    """

    MAIN_BODY = "_main"
    """
    The main body of a loose part, such as the blender's lid.
    """

    BLENDER_POWER_BUTTON = "_power_button_main"
    """
    The geom of the blender's power button.
    """

    COFFEE_RECEPTACLE_SITE = "receptacle_place_site"
    """
    The site under the coffee machine's spout a mug is put on.
    """


class FridgeCompartment(StrEnum):
    """
    The compartments of a fridge RoboCasa names.
    """

    FRIDGE = "fridge"
    """
    The cooling compartment.
    """


class FixtureRegionPrefix(StrEnum):
    """
    How the names of a fixture's inside regions begin.
    """

    FRIDGE_DRAWER = "fridge_drawer"
    """
    A drawer of the fridge.
    """

    FRIDGE_SHELF = "fridge_shelf"
    """
    A shelf of the fridge.
    """

    STAND_MIXER_BOWL = "bowl"
    """
    The bowl of the stand mixer.
    """


# %% objects


class RoboCasaObjectName(StrEnum):
    """
    The names RoboCasa gives the objects of its tasks.
    """

    OBJECT = "obj"
    """
    The object a task is about.
    """

    CONTAINER = "container"
    """
    The object another one is put on or in.
    """

    PLATE = "plate"
    """
    A plate.
    """

    OVEN_TRAY = "oven_tray"
    """
    The tray in an oven.
    """

    CHEESE = "cheese"
    """
    A wedge of cheese.
    """

    BREAD = "bread"
    """
    A slice of bread.
    """

    ICE_CUBE = "ice_cube1"
    """
    The first ice cube in a bowl.
    """

    CUP = "cup"
    """
    A glass of coffee.
    """

    DESSERT = "dessert"
    """
    A dessert.
    """

    COOKED_FOOD_CONTAINER = "cooked_food_container"
    """
    A container holding cooked food.
    """
