

Abstract away code

- very light weight wrapper perhaps support pytorch right now, that just provides some conviences for training a model to fit the data with pytorch?
- We don't want to supprot to much as you sould have full control over how the training happenings, etc.
- Perhaps its just about making it easy to make a dataloader or something from the recorded data...

- greatly inspired by Micheal Lasky of eletric sheep, who first made their system approx. the slam system
- Inspired by how Boston Dynamics incoperated all their previous work into the new ai models via this "distillation"




What is required?

- A method of hooking into and number of inputs and outputs, during run time and generating a dataset
- if sync then datashould can be organized by ticks, if async then perhaps just timestamp


Use Cases:

1. Fitting calibration model





- Once you fit the model you can then use it as a drop in replacement for the one you aproximated

- Helper to auto generate the component
- Or you may want to use it along side the component, the classical component or another neural network for that matter can provide consuensos or guard rails, or active learning.


Cool use cases:

- Hyper spectrcal camera

 We have a spectrcal camera sensor?

 We have an rgb sensor...

 I want a model that takes in the rgb camera and produce the hyperspectral camera output

 Ok so make a datarecording hook that connects to the output of the rgb camera and the output of the hyper spectrcal camera.

 Now whenever we run the program, it will autoatically be generating the dataset. We can set some paramters around sample freq or expiry date, etc.. ring buffer, etc..

 Now we train a model thats input is the rgb and output is the hyperspectrcal camera...

 In the full case, we have the full oracle hyper spectrcal model that takes in rgb/hyper and outputs semantic seg, object dtection.

 here we now create a model that tattachs agian to the rgb camera output as input and outputs the hyper, semangtic seg and object detection.

 Once we fit that model.. 

 for deployed robots we may use it as a drop in replacement for the spectrcal sensor?

 color camera -> Model -> spectrcal msg (other people can now connect to this... you would do the replacement in code manually)

 or we can run the two together for continual learning, which we should be doing anyways!

 - When we update the model in any place it should updated in all the usages thorughout the codebase. Actually we perhaps want more fine grained control? Perhaps we the configuration we can pin_versino=True or let it be floating in which case it will always use the latest

 - I think we can have multiple of the models running in parrlale so that we can compare their performance, they are fed the same inputs, and we can see their outputs and them compare them just like we compare the compnents of different classes...

 - We can always have that orcale running... 
 - We should support an active learning pipeline... we record data where there is not consensus.. or something...

 In this case we want Parrallel datarecorder?

 - I think you can just record each dataset full seperatly and then you can sparisy the dataset by doing consuses between multiple of them, as a post processing step. Or if data storage is a concern and you don't want to buffer this huge thing you can perhaps do it more in real time.

 A python way to add data recorders would be to literatly just edit the code and add a datarecorder(inputs, outputs)
 
 Perhaps you can wrap multiple data recorders then in a CensusRecorder(datarecord1, etc.) 

 You can test the Conesusrecorder(dtarecord1, datarecord1) Should always be in consuses because your duplicating! but much mroe helpful to do different models.


 - Kinematic calibration

 - Dynamics calibration


 - Distrilation of the entire system into an end to end network

 - Distillation of various parts of the system into end to end policy


 ---
 Furhter more for NN compile

Please Create a ConesnsusRecorder(Recorder) - which filters based on if the multiple inputs are matching or not

the default recording has standard methods to control the recording rate but we can subclass it to create more specialized ones

Also create a ConesnsusDx this takes in multiple inputs just like Conesus Recorder and creates a diaognistics status based on if they are matching or not...

and tracks things like how often they are tracking etc...

@diagonistics 

check out diagonistics for some of the ideas there

lets also create a util that is a to_dataloader

in pytorch utils

@torch_utils.py 

so the workflow should be super easyto just record then create a dataloader 

perhaps we need to Dataset

and also toDataLoader
https://docs.pytorch.org/tutorials/beginner/basics/data_tutorial.html 

show an example of then using pytorch to train a simple 3 layer sequential neural net to approximate the inputs and outputs

I think we may need to use the @data_dist.py is_flattenable propety to check if we can actually turn it into neural net input...

lets add that is_flatenable to the msg base class... @msg.py 

so I guess we need to flatten the inputs into a vector to feed in? this is actually not that straight forward....

well the msgs can define a nice way to do it...
perhaps I can have a hook here to define a custom flatten function though or something...

It would be nice if all we need to do is like provide the model
 
We do this:

class NeuralNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.flatten = nn.Flatten()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(28*28, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, 10)
        )

    def forward(self, x):
        x = self.flatten(x)
        logits = self.linear_relu_stack(x)
        return logits

model = NeuralNetwork().to(device)
print(model)

and this:

loss_fn = nn.CrossEntropyLoss()
optimizer = torch.optim.SGD(model.parameters(), lr=1e-3)

Hmm noo. Iactually don't want this to wrap around the model etc at all. The think It hsould just provide a nice dataset and datalaoder that you can integrated directly into your normal pytorch workflow.
def test(dataloader, model, loss_fn):
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    model.eval()
    test_loss, correct = 0, 0
    with torch.no_grad():
        for X, y in dataloader:
            X, y = X.to(device), y.to(device)
            pred = model(X)
            test_loss += loss_fn(pred, y).item()
            correct += (pred.argmax(1) == y).type(torch.float).sum().item()
    test_loss /= num_batches
    correct /= size
    print(f"Test Error: \n Accuracy: {(100*correct):>0.1f}%, Avg loss: {test_loss:>8f} \n")

Then when you are happy with the model...

here is where potetially you can help...

I think the same code that we used to help generate the dataloader...

NNCompileComponent(Model)

^ this would be nice. if we just have a drop in component where we can put in the model...

Actuall it can just be a NNComponent(Model) This may require specificgin the transform your self? hmmm

TorchNN(Component) perhaps this is the component type?

Ok explore some different ideas here...




---

I am now thinkink about descirbing the entire system, even the input streams that go outside the computer?
for a complete self learning system