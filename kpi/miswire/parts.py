"""Small components the mis-wirings and the valid graphs are built from."""

import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.sensor import JointState


class Src(lk.Component):
    value = lk.Out(JointState)

    def forward(self):
        return None


class Snk(lk.Component):
    value = lk.In(JointState)

    def forward(self, value):
        return None


class QSnk(lk.Component):
    value = lk.In(JointState, queue=4)

    def forward(self, value):
        return None


class ServoSnk(lk.Component):
    value = lk.In(ServoState)

    def forward(self, value):
        return None


class Through(lk.Component):
    value = lk.In(JointState)
    out = lk.Out(JointState)

    def forward(self, value):
        return value


class QThrough(lk.Component):
    value = lk.In(JointState, queue=4)
    out = lk.Out(JointState)

    def forward(self, value):
        return value


class LagThrough(lk.Component):
    value = lk.In(JointState, lagged=True)
    out = lk.Out(JointState)

    def forward(self, value):
        return value


class ObjSrc(lk.Component):
    """Any Python object may travel an in-process Python edge (CONTRACT §4.3 check 5)."""

    value = lk.Out(dict)

    def forward(self):
        return {"any": "object"}


class ObjSnk(lk.Component):
    value = lk.In(dict, queue=4)

    def forward(self, value):
        return None


class Join(lk.Component):
    """Steps on a queued trigger; reads `back` from its previous step (a same-group cycle closed by a lag)."""

    trigger = lk.In(JointState, queue=4)
    back = lk.In(JointState, lagged=True)
    out = lk.Out(JointState)

    def forward(self, trigger, back):
        return trigger


class Pair(lk.Component):
    """Two queue inputs: steps when both hold a message (gather, CONTRACT §5.2)."""

    left = lk.In(JointState, queue=4)
    right = lk.In(JointState, queue=4)

    def forward(self, left, right):
        return None
