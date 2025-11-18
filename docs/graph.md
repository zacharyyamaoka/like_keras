

- Would be cool if it was interactive
    - Can zoom in and out, expand collapse components, view dataflow and hz, filter topics, a bit fil
- Would be cool if it was editable? [no out of scope]

Keras uses pydot and graph viz:
- https://github.com/keras-team/keras/blob/v3.12.0/keras/src/utils/model_visualization.py#L405

Pytrees has many options:
- https://github.com/splintered-reality/py_trees_js
- https://py-trees.readthedocs.io/en/devel/visualisation.html (using pydot)
- https://github.com/splintered-reality/py_trees_ros_viewer

Groot has alot of nice features
- https://www.behaviortree.dev/groot/

vis.j2 

but seems now using d3? https://stackoverflow.com/questions/28595732/vis-js-how-to-expand-collapse-nodes-with-mouse-click

Ok I do like the idea of having both a static but then also a very hackable javascript one that can just run in the browser, essentially like meshcat..


Development: web infrastructure accelerates development velocity—Viser benefits from
mature libraries like React [76] and three.js for UI development and 3D rendering.


Dora uses mermaid.js
https://dora-rs.ai/docs/api/cli/#graph

Which seems to use d3: Many thanks to the d3 and dagre-d3 projects for providing the graphical layout and drawing libraries!

Mermaid Js is cool beacuse its in markdown which is good for AI to read as well :)


https://github.com/dora-rs/dora/tree/main/binaries/cli/src/command

https://github.com/dora-rs/dora/blob/main/binaries/cli/src/command/graph.rs
https://github.com/dora-rs/dora/blob/main/binaries/cli/src/command/graph/mermaid-template.html

So I like the idea of being able to compile your self into markdown essentially, and then also you can have the full js viz option I think..


If I use a python web framework, that is the lightest wegith. Not need to maintain a seperate library... you can hook to it straight from the logger etc..

You can work directly with js:
- Cytoscape.js
- d3.js
- vis.js

Or with python web frameworks..

https://dash.plotly.com/cytoscape

## Implementation (Nov 2025)

Graph visualization is now implemented using:
- Static graphs: Mermaid.js (similar to Dora)
- Interactive graphs: Dash Cytoscape

Usage:
```python
# Static visualization
system.plot_graph(to_file='graph.html', show=True)

# Get Mermaid diagram string
mermaid_str = system.to_mermaid()

# Interactive viewer
system.launch_viewer(port=8050)
```

Files:
- `lk/utils/plot_graph.py` - Core visualization functions
- `examples/ex_03_graph_visualization.py` - Example usage
