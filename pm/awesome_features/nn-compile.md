

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