import os
import numpy_eigen
from libplanner_interfaces_python import *

# aliases/typedefs
class Position2d(PointDouble): pass
class Position2dStamped(PointDoubleStamped): pass
class MapIndex(PointInt64): pass
class InterpolatedMapIndex(PointDouble): pass
class MapSize(PointUnsignedInt): pass

# Make enum types pickleable
def _isEnumType(o):
    return isinstance(o, type) and issubclass(o,int) and not (o is int)

def _tuple2enum(enum, value):
    enum = getattr(libplanner_interfaces_python, enum)
    e = enum.values.get(value,None)
    if e is None:
        e = enum(value)
    return e

def _registerEnumPicklers(): 
    from copy_reg import constructor, pickle
    def reduce_enum(e):
        enum = type(e).__name__.split('.')[-1]
        return ( _tuple2enum, ( enum, int(e) ) )
    constructor( _tuple2enum)
    for e in [ e for e in vars(libplanner_interfaces_python).itervalues() if _isEnumType(e) ]:
        pickle(e, reduce_enum)

_registerEnumPicklers()
