# Vault map: Zach's prior thinking on a component / communication / wiring API

Vault root `/home/bam/zach_brain` (paths below are relative to it). Provenance flag: **[Z]** = Zach's own words (`familiarity: Created by Me`, Apple/Notion notes, verbatim prompts); **[AI]** = an AI note he has not rewritten (read adversarially). Nearly all of `Projects/like keras/*2026-07-1x*` is [AI].

Key finding up front: the vault already contains the exact answer to the Nov 15 "wiring from outside" correction, written 2025-12 to 2026-07. The Nov 15 prompt ("Daily Note - Nov 15 2026.md" in `Sources/Daily Notes/`) is a return to the **Dec 2025 library** (`Sources/Apple Notes/A start`, `How to know it's right`), not the July 2026 GUI/substrate work.

## 1. Component model in his words

| Concept | His words (verbatim) | Source |
|---|---|---|
| **Component** | "Now I just have systems and components which are atomic cannot break apart further. Within then you just have pure funtions I can do whatever u want…" [Z] | `Sources/Apple Notes/A start (APPLE NOTE).md` |
| **Component = layer, System = model, Node = exec environment** | "component = layer, system = model, but add node = the runtime execution environment (different heads-on computers)", Keras "is a single-node forward pass" [AI summary of Z note] | `Projects/like keras/Prior Design Notes ... (2025-11 → 2026-04).md`, `Sources/Apple Notes/How to know it's right (APPLE NOTE).md` |
| **Port** | "Ports > ROS topics ... explicitly see the connections there are no name space issues". "a black box with ports that describe how it works" [Z] | `A start`, `Sources/Notion Import/Notebook & Private Up to Import/Dec 7 2025 - Robmosys (ARCHIVE).md` |
| **Wiring** | "key idea to avoid duplication only define the inputs for components ... This is like subscribe not publish" [Z]. Settled form: `comp1(buzz=comp2.o.fizz)` / `env(action=agent.o.action, fizz=comp2.o.bar, dar=env.o.rar)`; "you can even do self connections!"; all ports public | `A start`; `Projects/like keras/archive/docs from vscode/docs/002_components/002_components.md` |
| **Bulk/cross-cutting wiring** | `logger.batch_wire('.logs', [sys1, sys2, comp1, ...])`, `with ContextWire(tick, e_stop): ...` ("Potentially we can even pass in entire systems!") [Z] | `002_components.md` |
| **System** | "System (pure wiring, no computation)"; System is itself a Component whose `forward()` defines the graph. Model B chosen: components are leaves, "subsystem" = a System used inside a System. Flattening trivial; "No hidden serialization inside a Component" | `.../examples/basic/ex_01_classic_bandit/docs/references/async-sync-components.md`, `define_once_then-trace_or_call.md`; `.../docs/dev/sub-components.md` |
| **Deployment** | "Nodes define where components run (which machine, which engine)"; `Node('jetson', machine='192.168.1.10', engine='dora')`; later `FR - Deployment maps components or systems to Dora processes and machines` (Implemented 2026-07-28, "the authored Python graph says what the system means; a separate deployment object says where it runs"). He wants it called `placement:` (Aug 12: "A genuinely different concept wearing the same word ... the Kubernetes meaning") | `docs/dev/engines.md`; `Sources/Notes/FR - Deployment maps ...md`; `Sources/Daily Notes/Daily Note - Aug 12 2026.md` |
| **Binding is external, per instance** | "Geometry is a type. Wiring is an instance. They cannot share a container." CAN ids / adapter serial belong to an external per-machine binding; a CAN id inside the description "rotted" in the archive and made 4 arms claim id 11 | `Projects/like keras/Hardware Binding — Where the Wiring Is Authored (2026-08-12).md` [AI, but verified against code] |
| **Datasheet** | "You should be able to use the component completely without knowing anything about it's internal construction ... achieved by having a Datasheet"; "datasheet and the port on the outside" [Z] | `Maps & Views/Questions/Q - What do good sub components look like？.md` |

### Standard facilities every component has
- **Diagnostics and logs as output ports**, composed by mixin: `action, logs, diagnostics, tick = agent(env.o.obs)`. Archive docs: `DiagnosticsMixin.Outputs` (`diagnostics`) and `LoggingMixin.Outputs` (`logs`) merged by walking the MRO into auto-generated ports (`lk/common/component/docs/component_architecture.md`, `callback_standardization.md`).
- **Tick and e_stop as input ports**, wired from outside: `agent(tick=comp2.o.tick1, e_stop=comp1.o.e_stop, obs=...)` (`002_components.md`).
- **Config / state as ordinary data**: "configuration and state are also just parameters you pass in and out", anatomy `config + state + inputs/outputs`, `get_config/from_config/get_state/set_state`; "You should be able to save/load config & optionally state to files" (`002_components.md`, `lk/common/component/docs/FR.md`).
- **Parameter services and hardware request/response ports**: `set_*`/`get_*` for internal config, `*_cmd`/`*_query` for hardware; "Outputs are always nouns" (`port_naming_cheatsheet.md`).
- **Lifecycle** is opt-in by mixin (ISP): "All components seem the same from the outside. Very important! Allows you do to interpreter like pattern" yet `LifecycleMixin` not forced (`moteus_servos.md`; states from ROS 2 managed-node lifecycle in `docs/dev/lifecycle.md`).
- His own list of what every robot system re-implements: "Diagonistics / Logging / Replay / Configuration / Launch ... Testing / Hardware Interface ... whatever system is basically going to have to re implement the exact same things... because its just like part of the domain" (`Q - What do good sub components look like？.md`).
- Diagnostics record spec (not a like_keras note but the settled schema): `World Model/Chunks & Concepts/C - Diagnostics Record (What We Learned Since ROS DiagnosticStatus).md` (UNKNOWN is zero value, `expected_period_s`, typed measurements, `code` when not OK).

## 2. Principles and taste

- **Early binding of semantics, late binding of technology** (RobMoSys quote note `World Model/Quotes/“Early binding of semantics and late binding of technology”.md`): "the real product is the linking/decomposition model (semantics, bind early), not the visual (technology, bind late)". This is the justification for a backend-independent comm API.
- **External composition**: "The single variable that decides whether a component is a real black box is where the wiring is authored" [AI sharpening of his Jun 29 ROS/DORA reflection]; ROS topics = service locator, dora yml = dependency injection (`World Model/Chunks & Concepts/C - External Composition (Wiring Lives Outside the Component).md`). His own words: "Even though we had these messages ... I was directly calling the functions from the other classes ... We are not doing true Ports. Ah yes. its beacuse you register the topics within the node. DORA rs is different, you do the linking at the end as a final step!"
- **Keras shape**: components "behave just like functions"; "You can always execute the layers in eager mode"; Sequential/Functional/Subclass interchangeable; `summary()` and `plot_model` as separate utils; `get_config/from_config`; "Keras is perfect ... at the system level its connecting the components together and then you can implement each of those components using custom classes" (`the-zen-of-keras.md`, `002_components.md`).
- **Eager default, symbolic on demand**: three designs weighed (thread-local mode, smart ports, explicit). Landed on "define once, use twice": same `forward()` wires under `wiring_mode()` and executes eagerly (`define_once_then-trace_or_call.md`, `smart_port_architecture.md`). "Ports on the outside, Msgs on the inside"; port signature read from the `__call__` type hints.
- **Python-first, internal DSL, no config soup**: "No more XML files, config files, etc. one clean Internal DSL"; "Python as ground truth. Generate all other file types from that"; Dec 2025: "use the highest-level language possible" with C++/Rust only for the section that needs it (`Principles of Robot System Design.md`, Notion Import). Explicit note: C++ is "10-100x faster" for pointclouds, yet "the bottleneck is not computation speed".
- **File-first / one identity many views** (`like keras — Evolution and Key Influences (2026-07-16).md`), but this is the GUI half and is now Clank's.
- **Pure core, imperative shell**: "In my ROS nodes I didn't like how very simple nodes look actually very complicated because they had all this debugging/logging/viz msg construction mixed in. so I split it into sense/process/action" (`001_principles.md`). Component rules: atomic outputs, no blocking, at least one output (status), no mutation of inputs, "Data on wires must be immutable ... Absolutely essential" (`component_architecture.md`, `002_components.md`).
- **Cheap correct path**: "An abstraction that is merely offered gets bypassed the moment it costs more than cheating" [AI] - so ports must be fast and isolation enforced.
- **Compactness**: "As little code as possible", "Its more important for the interface to be simple than the implementation!", Road Ahead (Jul 26): "the entire python database is less than a million lines of code ... 'Simplicity is the ultimate sophistication'". Also: "I think I realised with Like keras, actually it really wasn't add much over just a normal python function."
- **Opinionated like `black`**: "Any color you like"; "Freedom of choice, freedom from choice" (`001_principles.md`). Write for "someone in 10 years who is not a domain expert".
- **Off-the-shelf first**: "Remember, I am not looking to reinvent the wheel here but select the right path from many options" (`Q - What do good sub components look like？`).
- **Blocks compose recursively**: a Model IS a Layer; `Block.__call__(state)->state`; closure under composition (`DP - A back end antagonistic Python library.md`, `C - Closure Under Composition ...`).
- **Block-view syntax (Jul 2026)**: NamedTuple return per component (0-1 output = bare type), kwargs binding at call site, ANF (every output named), feedback must pass an initial value (unit delay), sticky instance names plus explicit rename command (`Sources/Notes/DP - Underlying Python Syntax for Block View.md`).

## 3. Decided and rejected

**Decided**
- Two primitives only: Component (atomic leaf) and System (composition, itself a component). No sub-components inside a Component (`sub-components.md`, "Previously I had the possibility to have sub components but I think this was confusing").
- Wiring by inbound kwargs only (subscribe, not publish); ports public; wiring authored last, outside.
- Backends by compilation: `system.launch()` native, `launch(transport="dora")`, `compile_to_transport("dora", "x.yml")`, ROS2 "future" (`transport-system.md`, `launch-system.md`). Dora = recommended deployment backend.
- Data on wires immutable; messages typed (robot_msgs-style).
- Graph viz via Mermaid (static, same as dora CLI) plus Cytoscape (interactive) (`graph.md`).
- Lifecycle by mixin, not forced.
- Single repo / modular monolith at first (Clank Q&A "Mono Repo vs Single Repo").

**Rejected / killed**
- ROS-style wiring inside the component (his Nov 15 correction, above) and hiding local-vs-remote behind a Python call: "(actually wrong!)".
- Hidden remote calls inside a component: "might be a cheap in-process call or a network round-trip ... invisible in the user's Python code" (sub-components note).
- `from_id("ClassicBandit-v1")` magic strings: "I think we should move away from from_id() as its not fully configurable" (`making-components.md`).
- Parsing user Python statically: "execute/introspect, never parse" (Kill Pass 2.3: TorchScript 45% vs Dynamo 93%).
- Closed facet enum, URDF as hardware truth, "visual comprehension is the moat" (Kill Pass 1.2-1.5).
- Putting CAN ids inside RobotDescription; auto-discovery as the source of truth ("I cannot depend on auto discovery for connecting", Hardware Binding).
- ROS as the starting point: "ROS is a tool, not the starting point for system design" (`Principles of Robot System Design.md`).
- Full IDE embedding / many libraries: "my goal would now be to use more vanilla python" (PROJECT - like keras (v0)).

## 4. Lessons from like keras v1 to v4 (what should shape a lean comm library)

1. **Scope creep killed it.** v0 PROJECT: "3 years in and I don't have a code base I can work with"; July 16 Substrate v0.1 ("2,300 lines") then: "Ok This really completely missed the bullet honestly. It doesn't feel at all like I was imagining it." The GUI/IDE absorbed everything; Nov 15: "like keras used to be a very big project but now its just one thing, its like the Communication API." Keep the library to components, ports, systems, deployments.
2. **"I ended up having a lot of libraries ... use more vanilla python."** Components should be plain callables; the framework must be deletable. The substrate test `rm -rf lk/` (vanilla step functions never import `lk`) is the model: keep it as a rule.
3. **Learnings from v1** (`Sources/Notes/Learnings from like keras v1.md`) is a stub: only "Why is keras good? [[the-zen-of-keras]]". The Zen list (functions, eager mode, three construction styles, `summary`/plot, separation of loss/optimizer/metrics, ergonomic verbs) is the v1 lesson.
4. **v4 feedback** (`Like Keras V4 implementation feedback — 2026-07-31.md`) is about the Workbench lens shelf, not the comm API. Nothing there for this library.
5. **Dora tracer bullet works**: eager sync callables, PortRefs wire a graph, Deployment maps to dora processes with distinct PIDs; Daily Jul 28 struck through "Dora RS for inter component communication". Sep 1: pipelines inside one process need "no async actually", so intra-process sequence can run eagerly.
6. **Sync vs async**: he wanted one component that handles both ("Ideally we just have one component that can handle the async and sync cases"); the archive's answer was SyncComponent (`forward`) vs AsyncComponent (`on_<port>` callbacks) testable synchronously. Unresolved elegance.
7. **Cycles need state**: "State is what legally breaks cycles"; every feedback loop needs a unit delay (`Vocabulary & Models — The Missing Axis Is Time ...md`; Block View Thinking Pass). His vocabulary "has almost no words for time"; ticks, clocks and rates must be first-class port metadata.
8. **Adoption, not abstraction, is the bottleneck**: RobMoSys/SmartSoft reached TRL6 and stayed academic (quote note above). Keep the API one-page learnable.
9. **Ports carry contracts** (rate, QoS) or DDS-style mismatches only surface at runtime (Kill Pass 2.5); structure composes, properties (timing, safety) do not.
10. **Per-process discovery fails silently**: a manifest of expected devices beats discovery (Hardware Binding). Applies to deployments: validate "every leaf placed exactly once, unwired boundaries fail".

## 5. Open questions he left

- Where does wiring live for **parallel / conditional / feedback** composition (not just sequential)? ("DP - A back end antagonistic... For the library: to flesh out")
- How does a composite expose its own **datasheet** without leaking internals?
- The one interface: a robotics `Layer.__call__`? `step(state)->state` vs dataflow node? (DP library note)
- **Request/response vs pure ports**: archive has `set_/get_` and `*_cmd/*_query` ports (service-shaped). His Nov 15 line says client/service is the "wiring within a component" he rejected. Needs a ruling: are queries just paired ports wired externally, or banned?
- "What if you want to pass functions between the components? I think that's fine…" (A start) is undefined for cross-process.
- Dead wires / runtime-configurable formats (RealSense) in the port model.
- Time: tick vs timestamps, one clock vs many (Missing Axis note). Sync systems get tick numbers, async only timestamps.
- Lifecycle: mixin vs always-on; `activate()` supervision.
- Registry vs explicit imports for hot-swap implementations ("I think we can support both").
- C++ placement: "maybe it's actually in c++ you don't know!" (Apple Note): the interface must hide the language; `.compile("cpp")` was an idea, not built.
- Whether an optional **model/sampled `nn-approx` component** belongs in this lib (Apple note, probably out of scope now).
- The Monkey (Kill Pass): does a starved write surface stay a daily tool? Relevant only to GUI.
- Nov 15 asks that remain open: logging / health / params defined once as standard ports; and "it's almost like an IP camera" - plug in, controllers exposed on the network (nothing in the vault on discovery for this; ONVIF note `World Model/Chunks & Concepts/C - ONVIF ...md` exists but is camera-oriented).

## Files to read first
`Sources/Apple Notes/A start (APPLE NOTE).md`; `Sources/Apple Notes/How to know it's right (APPLE NOTE).md`; `Projects/like keras/archive/docs from vscode/docs/002_components/002_components.md`; `.../docs/dev/{sub-components,engines,transport-system,launch-system}.md`; `.../lk/common/component/docs/{component_architecture,callback_standardization,port_naming_cheatsheet,FR}.md`; `Maps & Views/Questions/Q - What do good sub components look like？.md`; `World Model/Chunks & Concepts/C - External Composition ....md`; `Projects/like keras/FRDPARRC/DPs/DP - A back end antagonistic Python library.md`; `Sources/Notes/DP - Underlying Python Syntax for Block View.md`; `Projects/like keras/like keras — Kill Pass and the Frontier of the Known (2026-07-16).md`; `Projects/like keras/Hardware Binding ... (2026-08-12).md`; `Sources/Notes/Like Keras - The Road Ahead - July 26 2026.md`. Canvases: `Maps & Views/like keras domain modelling.canvas` (only MDP agent/env boxes: Policy, Sensors, Actuators, State Estimator, Reward) and `like keras (DECISION TREE).canvas` (links to IDE-backend, vocabulary, UI-framework questions); neither holds comm-API content. No standalone `ros2_control` design note exists: only `Sources/Courses/2022-06_ROSConFr_What-is-new-in-ros2_control.pdf`, `Sources/Notion Import/Notebook (Jul 2025)/March 14 - 5 bar linkage ros 2 control.md`, and the Hardware Station concept note (swap hardware plugin, controllers stay the same).
