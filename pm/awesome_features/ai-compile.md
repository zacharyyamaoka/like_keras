

Abstract away language.

Select the one that will give you the best run time performance

Workflow is to generally implement first in python

- Test that behaviour is as intended in python
- Write unit tests in python

- Send a prompt to an ai to reimplement the system in the selected language. Right now we just support cpp
- You never really do any viz at this level, its just a drop in replacement at run time basically

How to verify that the implemtnation is correct?

We want to feed in a bunch of value into both components and make sure that the outputs come out the same

- We may want data to be time series in case that matters
- We may want the data to be from an actualy dataset distrbution..

- Msgs support defintinos of their data distrbution via `data_dist`
- the simplest datadist is like a uniform box with min max, by default there is no dist.this will throw an error


- Define the distributino for the input (unfirom, pareto, etc.) and sample it
- Iterate through a real data set (feeding in the datapoints in time series, and potetially at real time, or with rtf)
- That real deteset can be generated randomly or values can be in time series, etc..

- Ok it seems the end point is a dataset of input/output values

- Would be nice to have histogram to view the different values (they could be dataclasses though btw)

Then if its a pure function, you can just test its inputs and outputs



```python



```

What will this require?

- A method of comparing two text files with the outputs of the distribution (ground truth could be text file, or actually better I think another dataset.)

- I like text files because it becomes easy to instrospect. Easy to use diff/diff. And you can compare various differences at the same time, to get a better idea of what is eactly going on.

- We can then view the differences in a number of ways. one is text diff diff, but perhaps graphical analysis will be better suited.


- original component/system (You can mock between any inputs and outputs basically)
- A util prompting function that takes in the orginal component, a target language, and outputs a prompt that you can copy paste into any code editor to generate the new code (likely cursor agentic, beacuse it can set up everything) (key things it can set up are the acceptance test, doc explantions regarding the api of the new language, etc.)

- The new compiled comopnents which does the mapping between thoose inputs/outputs in your desired programming langauage

- a dataset of input output values. May be randomly organized, may be ordered. May come from randomly sampling (n_samples = 10000) the msg.data_dist or from historical data. May come from combing different histocral datasets. So we need a dataset object and we also need to add a DataDistMixin or just subclass it... to the base msg...

- A way to Feed in the inputs into the new component and store the outputs into another dataset. Option to feed in component at realtime if they are timestamped (helpful if time ordering matters) or with rtf (real time factor to just go as fast as possible). This seems like a generally helpful feature. Essential its like a rosbag playback ( this can be used more generally, alot of the funcitons in here can be really..)

- For sensors, its more challenging. If you can implement the driver as a component then you can mock the next level of input output.
- At some point though then you can manually mock the sensor input say, Put a paper of a known color infron tof the camera, and take an image, or look at a static scene, running both at the same time? Ok love that yes... snaps... actually the sensor may not support that. highly likely the driver does not support reading into two different things at the same time. So you can run seperatly. If a gazebo simulator etc then easy to just replay the exact same world...

- Compare the datasets. If its like a table, you can match the keys and then compare the outputs. First verify that the keys match, and the relative tick count or relative time is the same (abs time may be different... with flags you can perhaps set the acceptance criteria)
- Then compare the values...

- Output diff diffs to a text file for analysis
- Output histogram of both distrbutions (they should be overlapping perfectly or you can see the difference)
- Output heatmap of the to see differences etc
- Output image to see which pixels are wrong, etc.

---

## Implementation Summary

The AI-compile and NN-compile infrastructure has been implemented with the following components:

### Core Infrastructure (`lk/common/dataset/`)

**Dataset Class** (`dataset.py`)
- Backend-agnostic API for time-series data storage
- Factory methods: `Dataset.create()` and `Dataset.load()`
- Dictionary-style access: `dataset['key']`
- Supports multiple storage backends

**Storage Backends** (`backends/`)
- `DatasetBackend` - Abstract base class defining backend interface
- `MemoryBackend` - Fast in-memory storage (temporary)
- `PickleBackend` - Persistent file storage (Python-specific)
- Stubs for future: `MCAPBackend`, `ReductStoreBackend`

**DataRecorder Component** (`recorder.py`)
- Records data flowing through component ports
- Dynamic port creation: `recorder.add_input('name', MsgType)`
- Configurable sampling rate and ring buffer
- Auto-timestamp and auto-flush options

**DataPlayback Component** (`playback.py`)
- Plays back recorded datasets through output ports
- Real-time factor (RTF) control for time-accurate replay
- Support for looping and start offset
- Compatible with component system

**Comparison Utilities** (`diff.py`)
- `diffdiff()` - Git-like comparison of two datasets
- Detailed difference reporting by key
- Numeric comparison with tolerance
- Support for Msg types, arrays, dicts, primitives

**Visualization Stubs** (`viz.py`)
- `plot_histogram()` - Distribution visualization
- `plot_heatmap()` - Difference heatmaps
- `plot_timeseries()` - Time-series plots
- `plot_comparison()` - Overlay comparisons
- Full implementations to be added based on needs

### Prompt Utilities (`lk/utils/prompt.py`)

**Prompt Dataclass**
- Stores all information for AI-assisted compilation
- Fields: target language, I/O specs, source code, API docs, examples, dataset path
- `to_markdown()` - Generate formatted prompt
- `save()` - Export to markdown file

**Helper Functions**
- `create_prompt()` - Generate prompts from components or manual specs
- `send_prompt()` - Deliver prompts (clipboard/file/print)
- `Prompt.from_component()` - Extract specs from component instances

### Examples (`lk/common/dataset/examples/`)

**AI-Compile Workflow** (`ex_01_ai_compile_workflow.py`)
- Complete end-to-end demonstration
- Vector normalizer component → C++ compilation
- Dataset generation → Recording → Comparison
- Shows perfect match vs error detection

**NN-Compile Workflow** (`ex_02_nn_compile_workflow.py`)
- Distillation workflow demonstration
- PD controller → Neural network approximation
- Training data collection → Evaluation → Comparison
- Active learning and A/B testing concepts

### Key Design Principles

1. **Backend Agnostic** - Easy to swap storage (memory, pickle, MCAP, ReductStore)
2. **Component-Based** - DataRecorder/DataPlayback are first-class components
3. **Type-Safe** - All ports use Msg subclasses for type checking
4. **Time-Aware** - Timestamps, RTF control, time-series ordering
5. **Diffable** - Comprehensive comparison with detailed reporting

### Usage Patterns

**Basic Recording**
```python
dataset = Dataset.create(backend='memory')
recorder = DataRecorder(dataset=dataset)
recorder.add_input('sensor', SensorMsg)
recorder.record('sensor', msg, timestamp=t)
```

**Playback with RTF**
```python
dataset = Dataset.load('recording.pkl')
playback = DataPlayback(dataset=dataset, config=DataPlayback.Config(rate=2.0))
playback.play_realtime()  # 2x speed
```

**Verification**
```python
result = diffdiff(dataset_original, dataset_new, tol=1e-6)
if result.all_match:
    print("✓ Implementations match!")
else:
    print(result.summary())
```

**Prompting**
```python
prompt = create_prompt(component=my_component, target_language='cpp')
send_prompt(prompt, method='clipboard')
```

### Future Enhancements

- MCAP backend for ROS 2 compatibility
- ReductStore backend for production time-series DB
- Full matplotlib-based visualization implementations
- PyTorch DataLoader integration
- Consensus recording for active learning
- Model versioning and A/B testing infrastructure
- Automatic regression test generation