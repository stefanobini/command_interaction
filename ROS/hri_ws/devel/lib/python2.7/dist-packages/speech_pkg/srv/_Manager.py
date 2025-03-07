# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from speech_pkg/ManagerRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import speech_pkg.msg

class ManagerRequest(genpy.Message):
  _md5sum = "21e1815396969a6f82c0ae7d856b83d3"
  _type = "speech_pkg/ManagerRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
float64 end_time"""
  __slots__ = ['data']
  _slot_types = ['speech_pkg/SpeechData']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ManagerRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.data is None:
        self.data = speech_pkg.msg.SpeechData()
    else:
      self.data = speech_pkg.msg.SpeechData()

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
      length = len(self.data.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sh'%length
      buff.write(struct.Struct(pattern).pack(*self.data.data))
      _x = self
      buff.write(_get_struct_h2d().pack(_x.data.doa, _x.data.start_time, _x.data.end_time))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.data is None:
        self.data = speech_pkg.msg.SpeechData()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sh'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.data.data = s.unpack(str[start:end])
      _x = self
      start = end
      end += 18
      (_x.data.doa, _x.data.start_time, _x.data.end_time,) = _get_struct_h2d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.data.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sh'%length
      buff.write(self.data.data.tostring())
      _x = self
      buff.write(_get_struct_h2d().pack(_x.data.doa, _x.data.start_time, _x.data.end_time))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.data is None:
        self.data = speech_pkg.msg.SpeechData()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sh'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.data.data = numpy.frombuffer(str[start:end], dtype=numpy.int16, count=length)
      _x = self
      start = end
      end += 18
      (_x.data.doa, _x.data.start_time, _x.data.end_time,) = _get_struct_h2d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_h2d = None
def _get_struct_h2d():
    global _struct_h2d
    if _struct_h2d is None:
        _struct_h2d = struct.Struct("<h2d")
    return _struct_h2d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from speech_pkg/ManagerResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ManagerResponse(genpy.Message):
  _md5sum = "24842bc754e0f5cc982338eca1269251"
  _type = "speech_pkg/ManagerResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool flag
"""
  __slots__ = ['flag']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       flag

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ManagerResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.flag is None:
        self.flag = False
    else:
      self.flag = False

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
      _x = self.flag
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.flag,) = _get_struct_B().unpack(str[start:end])
      self.flag = bool(self.flag)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.flag
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.flag,) = _get_struct_B().unpack(str[start:end])
      self.flag = bool(self.flag)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class Manager(object):
  _type          = 'speech_pkg/Manager'
  _md5sum = '426efa73e6c8d460d7543a8bf442644f'
  _request_class  = ManagerRequest
  _response_class = ManagerResponse
