pycram.plan
===========

.. py:module:: pycram.plan


Attributes
----------

.. autoapisummary::

   pycram.plan.logger


Classes
-------

.. autoapisummary::

   pycram.plan.PlotAlignment
   pycram.plan.Plan
   pycram.plan.PlanNode
   pycram.plan.DesignatorNode
   pycram.plan.ActionNode
   pycram.plan.ResolvedActionNode
   pycram.plan.MotionNode


Functions
---------

.. autoapisummary::

   pycram.plan.managed_node


Module Contents
---------------

.. py:data:: logger

.. py:class:: PlotAlignment

   Bases: :py:obj:`enum.IntEnum`


   Enum where members are also (and must be) ints


   .. py:attribute:: HORIZONTAL
      :value: 0



   .. py:attribute:: VERTICAL
      :value: 1



.. py:class:: Plan(root: PlanNode, context: pycram.datastructures.dataclasses.Context)

   Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
   traverse the plan structure in depth first order and perform each PlanNode


   .. py:attribute:: current_plan
      :type:  typing_extensions.Optional[Plan]
      :value: None


      The plan that is currently being performed



   .. py:attribute:: current_node
      :type:  typing_extensions.Optional[PlanNode]
      :value: None


      The node, of the current_plan, that is currently being performed



   .. py:attribute:: on_start_callback
      :type:  ClassVar[typing_extensions.Dict[typing_extensions.Optional[typing_extensions.Union[typing_extensions.Type[pycram.robot_plans.ActionDescription], typing_extensions.Type[PlanNode]]], typing_extensions.List[typing_extensions.Callable]]]

      Callbacks to be called when a node of the given type is started.



   .. py:attribute:: on_end_callback
      :type:  ClassVar[typing_extensions.Dict[typing_extensions.Optional[typing_extensions.Union[typing_extensions.Type[pycram.robot_plans.ActionDescription], typing_extensions.Type[PlanNode]]], typing_extensions.List[typing_extensions.Callable]]]

      Callbacks to be called when a node of the given type is ended.



   .. py:attribute:: plan_graph


   .. py:attribute:: node_indices


   .. py:attribute:: root
      :type:  PlanNode


   .. py:attribute:: context


   .. py:attribute:: world


   .. py:attribute:: robot


   .. py:attribute:: super_plan
      :type:  Plan


   .. py:property:: nodes


   .. py:property:: edges


   .. py:method:: mount(other: Plan, mount_node: PlanNode = None)

      Mounts another plan to this plan. The other plan will be added as a child of the mount_node.

      :param other: The plan to be mounted
      :param mount_node: A node of this plan to which the other plan will be mounted. If None, the root of this plan will be used.



   .. py:method:: merge_nodes(node1: PlanNode, node2: PlanNode)

      Merges two nodes into one. The node2 will be removed and all its children will be added to node1.

      :param node1: Node which will remain in the plan
      :param node2: Node which will be removed from the plan



   .. py:method:: remove_node(node_for_removal: PlanNode)

      Removes a node from the plan. If the node is not in the plan, it will be ignored.

      :param node_for_removal: Node to be removed



   .. py:method:: add_node(node_for_adding: PlanNode, **attr)

      Adds a node to the plan. The node will not be connected to any other node of the plan.

      :param node_for_adding: Node to be added
      :param attr: Additional attributes to be added to the node



   .. py:method:: add_edge(u_of_edge: PlanNode, v_of_edge: PlanNode, **attr)

      Adds an edge to the plan. If one or both nodes are not in the plan, they will be added to the plan.

      :param u_of_edge: Origin node of the edge
      :param v_of_edge: Target node of the edge
      :param attr: Additional attributes to be added to the edge



   .. py:method:: add_edges_from(ebunch_to_add: typing_extensions.Iterable[typing_extensions.Tuple[PlanNode, PlanNode]], **attr)

      Adds edges to the plan from an iterable of tuples. If one or both nodes are not in the plan, they will be added to the plan.

      :param ebunch_to_add: Iterable of tuples of nodes to be added
      :param attr: Additional attributes to be added to the edges



   .. py:method:: add_nodes_from(nodes_for_adding: typing_extensions.Iterable[PlanNode], **attr)

      Adds nodes from an Iterable of nodes.

      :param nodes_for_adding: The iterable of nodes
      :param attr: Additional attributes to be added



   .. py:method:: insert_below(insert_node: PlanNode, insert_below: PlanNode)

      Inserts a node below the given node.

      :param insert_node: The node to be inserted
      :param insert_below: A node of the plan below which the given node should be added



   .. py:method:: perform() -> typing_extensions.Any

      Performs the root node of this plan.

      :return: The return value of the root node



   .. py:method:: resolve()

      Resolves the root node of this plan if it is a DesignatorNode

      :return: The resolved designator



   .. py:method:: flattened_parameters()

      The core parameter of this plan, as dict with paths as keys and the core type as value

      :return: A dict of the core types



   .. py:method:: re_perform()


   .. py:property:: actions
      :type: typing_extensions.List[ActionNode]



   .. py:property:: layers
      :type: typing_extensions.List[typing_extensions.List[PlanNode]]



   .. py:method:: bfs_layout(scale: float = 1.0, align: PlotAlignment = PlotAlignment.VERTICAL) -> typing_extensions.Dict[int, numpy.array]

      Generate a bfs layout for this circuit.

      :return: A dict mapping the node indices to 2d coordinates.



   .. py:method:: plot_plan_structure(scale: float = 1.0, align: PlotAlignment = PlotAlignment.HORIZONTAL) -> None

      Plots the kinematic structure of the world.
      The plot shows bodies as nodes and connections as edges in a directed graph.



   .. py:method:: add_on_start_callback(callback: typing_extensions.Callable[[PlanNode], None], action_type: typing_extensions.Optional[typing_extensions.Type[pycram.robot_plans.ActionDescription], typing_extensions.Type[PlanNode]] = None)
      :classmethod:


      Adds a callback to be called when an action of the given type is started.

      :param callback: The callback to be called
      :param action_type: The type of the action, if None, the callback will be called for all actions



   .. py:method:: add_on_end_callback(callback: typing_extensions.Callable[[PlanNode], None], action_type: typing_extensions.Optional[typing_extensions.Type[pycram.robot_plans.ActionDescription], typing_extensions.Type[PlanNode]] = None)
      :classmethod:


      Adds a callback to be called when an action of the given type is ended.

      :param callback: The callback to be called
      :param action_type: The type of the action



   .. py:method:: remove_on_start_callback(callback: typing_extensions.Callable[[PlanNode], None], action_type: typing_extensions.Optional[typing_extensions.Type[pycram.robot_plans.ActionDescription], typing_extensions.Type[PlanNode]] = None)
      :classmethod:


      Removes a callback to be called when an action of the given type is started.

      :param callback: The callback to be removed
      :param action_type: The type of the action



   .. py:method:: remove_on_end_callback(callback: typing_extensions.Callable[[PlanNode], None], action_type: typing_extensions.Optional[typing_extensions.Type[pycram.robot_plans.ActionDescription], typing_extensions.Type[PlanNode]] = None)
      :classmethod:


      Removes a callback to be called when an action of the given type is ended.

      :param callback: The callback to be removed
      :param action_type: The type of the action



.. py:function:: managed_node(func: typing_extensions.Callable) -> typing_extensions.Callable

   Decorator which manages the state of a node, including the start and end time, status and reason of failure as well
   as the setting of the current node in the plan.

   :param func: Reference to the perform function of the node
   :return: The wrapped perform function


.. py:class:: PlanNode

   .. py:attribute:: status
      :type:  pycram.datastructures.enums.TaskStatus

      The status of the node from the TaskStatus enum.



   .. py:attribute:: start_time
      :type:  typing_extensions.Optional[datetime.datetime]

      The starting time of the function, optional



   .. py:attribute:: end_time
      :type:  typing_extensions.Optional[datetime.datetime]
      :value: None


      The ending time of the function, optional



   .. py:attribute:: reason
      :type:  typing_extensions.Optional[pycram.failures.PlanFailure]
      :value: None


      The reason of failure if the action failed.



   .. py:attribute:: result
      :type:  typing_extensions.Optional[typing_extensions.Any]
      :value: None


      Result from the execution of this node



   .. py:attribute:: plan
      :type:  typing_extensions.Optional[Plan]
      :value: None


      Reference to the plan to which this node belongs



   .. py:property:: index
      :type: int



   .. py:property:: parent
      :type: typing_extensions.Optional[PlanNode]


      The parent node of this node, None if this is the root node

      :return: The parent node



   .. py:property:: children
      :type: typing_extensions.List[PlanNode]


      All children nodes of this node

      :return:  A list of child nodes



   .. py:property:: recursive_children
      :type: typing_extensions.List[PlanNode]


      Recursively lists all children and their children.

      :return: A list of all nodes below this node



   .. py:property:: subtree
      :type: Plan


      Creates a new plan with this node as the new root

      :return: A new plan



   .. py:property:: all_parents
      :type: typing_extensions.List[PlanNode]


      Returns all nodes above this node until the root node. The order is from this node to the root node.

      :return: A list of all nodes above this



   .. py:property:: is_leaf
      :type: bool


      Returns True if this node is a leaf node

      :return: True if this node is a leaf node



   .. py:method:: flattened_parameters()

      The core types pf this node as dict

      :return: The flattened parameter



   .. py:method:: __hash__()


   .. py:method:: perform(*args, **kwargs)


   .. py:method:: interrupt()

      Interrupts the execution of this node and all nodes below



   .. py:method:: resume()

      Resumes the execution of this node and all nodes below



   .. py:method:: pause()

      Suspends the execution of this node and all nodes below.



.. py:class:: DesignatorNode

   Bases: :py:obj:`PlanNode`


   .. py:attribute:: designator_ref
      :type:  pycram.designator.DesignatorDescription
      :value: None


      Reference to the Designator in this node



   .. py:attribute:: designator_type
      :type:  typing_extensions.Optional[typing_extensions.Any]
      :value: None


      The action and that is performed or None if nothing was performed



   .. py:attribute:: kwargs
      :type:  typing_extensions.Dict[str, typing_extensions.Any]
      :value: None


      kwargs of the action in this node



   .. py:method:: __post_init__()


   .. py:method:: __hash__()


   .. py:method:: __repr__(*args, **kwargs)


   .. py:method:: flattened_parameters() -> typing_extensions.Dict[str, pycram.has_parameters.leaf_types]

      The core types of the parameters of this node as dict with paths as keys and the core type as value.
      This resolves the parameters to its type not the actual value.

      :return: The core types of this action



   .. py:method:: flatten() -> typing_extensions.Dict[str, pycram.has_parameters.leaf_types]

      Flattens the parameters of this node to a dict with the parameter as  key and the value as value.

      :return: A dict of the flattened parameters



.. py:class:: ActionNode

   Bases: :py:obj:`DesignatorNode`


   A node in the plan representing an ActionDesignator description


   .. py:attribute:: action_iter
      :type:  typing_extensions.Iterator[pycram.robot_plans.ActionDescription]
      :value: None


      Iterator over the current evaluation state of the ActionDesignator Description



   .. py:attribute:: designator_type
      :type:  typing_extensions.Type[pycram.robot_plans.ActionDescription]
      :value: None


      Class of the ActionDesignator



   .. py:method:: __hash__()


   .. py:method:: perform()

      Performs this node by resolving the ActionDesignator description to the next resolution and then performing the
      result.

      :return: Return value of the resolved action node



   .. py:method:: __repr__(*args, **kwargs)


.. py:class:: ResolvedActionNode

   Bases: :py:obj:`DesignatorNode`


   A node representing a resolved ActionDesignator with fully specified parameters


   .. py:attribute:: designator_ref
      :type:  pycram.robot_plans.ActionDescription
      :value: None


      Reference to the ActionDesignator in this node



   .. py:attribute:: designator_type
      :type:  typing_extensions.Type[pycram.robot_plans.ActionDescription]
      :value: None


      Class of the ActionDesignator



   .. py:attribute:: execution_data
      :type:  pycram.datastructures.dataclasses.ExecutionData
      :value: None


      Additional data that  is collected before and after the execution of the action.



   .. py:attribute:: motion_executor
      :type:  pycram.motion_executor.MotionExecutor
      :value: None


      Instance of the MotionExecutor used to execute the motion chart of the sub-motions of this action.



   .. py:attribute:: _last_mod
      :type:  semantic_digital_twin.world_description.world_modification.WorldModelModificationBlock
      :value: None


      The last model modification block before the execution of this node. Used to check if the model has changed during execution.



   .. py:method:: __hash__()


   .. py:method:: collect_motions() -> typing_extensions.List[giskardpy.motion_statechart.graph_node.Task]

      Collects all child motions of this action. A motion is considered if it is a direct child of this action node,
      i.e. there is no other action node between this action node and the motion.



   .. py:method:: construct_msc()

      Builds a giskard Motion State Chart (MSC) from the collected motions of this action node.



   .. py:method:: execute_msc()

      Executes the constructed MSC.



   .. py:method:: log_execution_data_pre_perform()

      Creates a ExecutionData object and logs additional information about the execution of this node.



   .. py:method:: log_execution_data_post_perform()

      Writes additional information to the ExecutionData object after performing this node.



   .. py:method:: perform()

      Performs this node by performing the resolved action designator in zit

      :return: The return value of the resolved ActionDesignator



   .. py:method:: __repr__(*args, **kwargs)


.. py:class:: MotionNode

   Bases: :py:obj:`DesignatorNode`


   A node in the plan representing a MotionDesignator


   .. py:attribute:: designator_ref
      :type:  typing_extensions.Type[pycram.robot_plans.BaseMotion]
      :value: None


      Reference to the MotionDesignator



   .. py:method:: __hash__()


   .. py:method:: perform()

      Performs this node by performing the respective MotionDesignator. Additionally, checks if one of the parents has
      the status INTERRUPTED and aborts the perform if that is the case.

      :return: The return value of the Motion Designator



   .. py:method:: __repr__(*args, **kwargs)


   .. py:method:: flatten()

      Flattens the parameters of this node to a dict with the parameter as  key and the value as value.

      :return: A dict of the flattened parameters



   .. py:method:: flattened_parameters()

      The core types of the parameters of this node as dict with paths as keys and the core type as value.
      This resolves the parameters to its type not the actual value.

      :return: The core types of this action



   .. py:property:: parent_action_node

      Returns the next resolved action node in the plan above this motion node.



