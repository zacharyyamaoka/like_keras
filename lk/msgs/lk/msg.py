class Msg:
    def __init__(self):
        pass

    @classmethod
    def from_similar_type(cls, similar_type):
        msg = cls()
        for attr in vars(similar_type):
            if hasattr(msg, attr):
                setattr(msg, attr, getattr(similar_type, attr))
        return msg
