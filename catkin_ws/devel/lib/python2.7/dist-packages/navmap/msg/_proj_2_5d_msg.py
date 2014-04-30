"""autogenerated by genpy from navmap/proj_2_5d_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import navmap.msg

class proj_2_5d_msg(genpy.Message):
  _md5sum = "79c33411ebcb4d92753eedd76f986950"
  _type = "navmap/proj_2_5d_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Reference points
point2d_t[] refPoints

# Target points
point2d_t[] targetPoints

================================================================================
MSG: navmap/point2d_t
float32 x
float32 y

"""
  __slots__ = ['refPoints','targetPoints']
  _slot_types = ['navmap/point2d_t[]','navmap/point2d_t[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       refPoints,targetPoints

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(proj_2_5d_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.refPoints is None:
        self.refPoints = []
      if self.targetPoints is None:
        self.targetPoints = []
    else:
      self.refPoints = []
      self.targetPoints = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.refPoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.refPoints:
        _x = val1
        buff.write(_struct_2f.pack(_x.x, _x.y))
      length = len(self.targetPoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.targetPoints:
        _x = val1
        buff.write(_struct_2f.pack(_x.x, _x.y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.refPoints is None:
        self.refPoints = None
      if self.targetPoints is None:
        self.targetPoints = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.refPoints = []
      for i in range(0, length):
        val1 = navmap.msg.point2d_t()
        _x = val1
        start = end
        end += 8
        (_x.x, _x.y,) = _struct_2f.unpack(str[start:end])
        self.refPoints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.targetPoints = []
      for i in range(0, length):
        val1 = navmap.msg.point2d_t()
        _x = val1
        start = end
        end += 8
        (_x.x, _x.y,) = _struct_2f.unpack(str[start:end])
        self.targetPoints.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.refPoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.refPoints:
        _x = val1
        buff.write(_struct_2f.pack(_x.x, _x.y))
      length = len(self.targetPoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.targetPoints:
        _x = val1
        buff.write(_struct_2f.pack(_x.x, _x.y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.refPoints is None:
        self.refPoints = None
      if self.targetPoints is None:
        self.targetPoints = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.refPoints = []
      for i in range(0, length):
        val1 = navmap.msg.point2d_t()
        _x = val1
        start = end
        end += 8
        (_x.x, _x.y,) = _struct_2f.unpack(str[start:end])
        self.refPoints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.targetPoints = []
      for i in range(0, length):
        val1 = navmap.msg.point2d_t()
        _x = val1
        start = end
        end += 8
        (_x.x, _x.y,) = _struct_2f.unpack(str[start:end])
        self.targetPoints.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2f = struct.Struct("<2f")
