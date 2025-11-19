


Inspired by: https://gymnasium.farama.org/api/spaces/

Key Design Decision:

- Will not directly mix in with the basic datatype
- PoseStamped() vs PoseStampedSpace
- Idea is that you have a space that returns a conconcrete class
- I do like the idea of inerhintance though, as then you can treat it like a normal PoseStamped if you want..
- Poteially this can be a helper?