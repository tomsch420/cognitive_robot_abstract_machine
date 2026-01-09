pycram.visualization
====================

.. py:module:: pycram.visualization


Functions
---------

.. autoapisummary::

   pycram.visualization.plot_rustworkx_interactive
   pycram.visualization.calculate_layout_positions
   pycram.visualization.build_nx_graph
   pycram.visualization._object_params_with_properties
   pycram.visualization._collect_properties
   pycram.visualization._format_params
   pycram.visualization._escape_html


Module Contents
---------------

.. py:function:: plot_rustworkx_interactive(graph: Any, *, node_params: Optional[Dict[int, Dict[str, Any]]] = None, node_label: Optional[Callable[[int, Any], str]] = None, attributes: Optional[Sequence[str]] = None, layout: str = 'spring', start: Optional[int] = None, title: str = 'Rustworkx Graph', width: int = 1200, height: int = 800)

   Plot an interactive visualization of a rustworkx graph.

   - Click on a node to show its parameters in a side panel.
   - Hover shows the node label.

   Parameters
   ----------
   graph:
       A rustworkx.PyGraph or rustworkx.PyDiGraph instance.
   node_params:
       Optional mapping from node index to a dict of parameters to display when
       the node is clicked. If not provided and the node payload is a dict,
       those items will be used. If provided together with ``attributes``, the
       displayed parameters will be filtered to the given attribute names.
   node_label:
       Optional callable that takes (index, payload) and returns a label string
       for the node. By default it tries to use ``payload.get('label')`` or
       ``str(payload)``.
   attributes:
       Optional list of attribute names to show from the parameters. Ignored if
       parameters are not dict-like.
   layout:
       Layout algorithm to use: "spring", "kamada_kawai", or "bfs".
   start:
       Optional start node index for "bfs" layout.
   title:
       Plot title.
   width, height:
       Figure size in pixels.

   Notes
   -----
   This function imports bokeh lazily so that it does not add a hard runtime
   dependency unless you call it. Install with `pip install bokeh`.


.. py:function:: calculate_layout_positions(layout: str, nx_g: networkx.Graph, start: Optional[int] = None) -> Dict[int, Tuple[float, float]]

   Calculates node positions based on the selected layout.
   :param layout: Layout name, e.g. "spring", "kamada_kawai", "bfs
   :param nx_g: networkx graph
   :param start: Optional start node index for "bfs" layout.
   :return: A dictionary mapping node indices to 2d coordinates.


.. py:function:: build_nx_graph(graph: Any, node_params, attributes, node_label) -> networkx.Graph

   Convert a rustworkx graph to a networkx graph.


.. py:function:: _object_params_with_properties(payload: Any) -> Optional[Dict[str, Any]]

   Build a parameter dictionary from a node payload by combining:
   - public attributes from payload.__dict__ (if present)
   - readable @property attributes defined on the payload's class
   - if payload is a dict, return it (excluding 'label')

   Private attributes (starting with '_') and the key 'label' are excluded.
   Values that raise on access are skipped. Callables are skipped.


.. py:function:: _collect_properties(payload) -> Dict[str, Any]

.. py:function:: _format_params(params: Optional[Dict[str, Any]]) -> str

   Return HTML for parameter dict suitable for the side panel.


.. py:function:: _escape_html(value: Any) -> str

