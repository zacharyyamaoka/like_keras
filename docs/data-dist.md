

Should PoseStamped

have x = FloatDist()

or x: float and x_dist: FloatDist?

Goal: We want to be able to describe a datadistrbution range on input/output values in the exact same way as gym spaces. 

We want thoose classes to also be functional as classes them selves though. The issue with gym is that its purely just a spec. you need to sample it to be functional...

Interesting do I actually want that?

Mabyes its enough to have PoseStamped.Dist as a type...

PoseStamped.Dist.uniform(x,x).sample()

I do agree there is a seperation between the message it self... you only really care about that informaiton at the interferace... 

the idea is to keep the interface message clear...

input(name, type, dist:Optional[describes acceptable bounds])

input("pose", PoseStamped, PoseStamped.Dist.uniform(0, 1))

or input("pose", PoseStamped(dist=PoseStamped.Dist.uniform(0, 1)))

https://gymnasium.farama.org/api/spaces/


Both of these are actually bad!

input("pose", PoseStamped, PoseStamped.Dist.uniform(0, 1))

or input("pose", PoseStamped(dist=PoseStamped.Dist.uniform(0, 1)))

Its cleaner to define either as a type or a Tyoe DataDist its redundant to have both!

Type is the basic way but DataDist is the way that lets you do input validation, or send automatica diaognistics warnings if outside dist, and easily sample the dist during testing

input("pose", PoseStamped())
input("pose", PoseStamped.Dist.unfirom())

Or if we mixin directly...

input("pose", PoseStamped.unfirom()) # ok cool I do like that static consturctor... its just a different init. I also like it being based around the PoseSTamped message so I don't need to go like

from lk.msgs import PoseStampedDist 
etc. its just readily acceisble right there...


Idea, there is a clear seperation between a dist and an actual message. You almost never need both at the same time.

When specificing a input/output spec, we don't have a specific message so its a great time to specifiy a dist. (this is essentially the gym idea)

Would it be helpful to know what dist a msg came from? Poetailyl but not really, thats why I don't think the message needs to carry that information.

