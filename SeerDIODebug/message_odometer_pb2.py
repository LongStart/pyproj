# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message_odometer.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import message_header_pb2 as message__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message_odometer.proto',
  package='rbk.protocol',
  syntax='proto3',
  serialized_pb=_b('\n\x16message_odometer.proto\x12\x0crbk.protocol\x1a\x14message_header.proto\"\xdd\x01\n\x10Message_Odometer\x12,\n\x06header\x18\x01 \x01(\x0b\x32\x1c.rbk.protocol.Message_Header\x12\r\n\x05\x63ycle\x18\x02 \x01(\r\x12\t\n\x01x\x18\x03 \x01(\x01\x12\t\n\x01y\x18\x04 \x01(\x01\x12\r\n\x05\x61ngle\x18\x05 \x01(\x01\x12\x0f\n\x07is_stop\x18\x06 \x01(\x08\x12\r\n\x05vel_x\x18\x07 \x01(\x01\x12\r\n\x05vel_y\x18\x08 \x01(\x01\x12\x12\n\nvel_rotate\x18\t \x01(\x01\x12\x0f\n\x07\x65ncoder\x18\n \x03(\x05\x12\x13\n\x0bsteer_angle\x18\x0b \x01(\x01\x62\x06proto3')
  ,
  dependencies=[message__header__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_MESSAGE_ODOMETER = _descriptor.Descriptor(
  name='Message_Odometer',
  full_name='rbk.protocol.Message_Odometer',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='rbk.protocol.Message_Odometer.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cycle', full_name='rbk.protocol.Message_Odometer.cycle', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='x', full_name='rbk.protocol.Message_Odometer.x', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='rbk.protocol.Message_Odometer.y', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='angle', full_name='rbk.protocol.Message_Odometer.angle', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='is_stop', full_name='rbk.protocol.Message_Odometer.is_stop', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vel_x', full_name='rbk.protocol.Message_Odometer.vel_x', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vel_y', full_name='rbk.protocol.Message_Odometer.vel_y', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vel_rotate', full_name='rbk.protocol.Message_Odometer.vel_rotate', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='encoder', full_name='rbk.protocol.Message_Odometer.encoder', index=9,
      number=10, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steer_angle', full_name='rbk.protocol.Message_Odometer.steer_angle', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=63,
  serialized_end=284,
)

_MESSAGE_ODOMETER.fields_by_name['header'].message_type = message__header__pb2._MESSAGE_HEADER
DESCRIPTOR.message_types_by_name['Message_Odometer'] = _MESSAGE_ODOMETER

Message_Odometer = _reflection.GeneratedProtocolMessageType('Message_Odometer', (_message.Message,), dict(
  DESCRIPTOR = _MESSAGE_ODOMETER,
  __module__ = 'message_odometer_pb2'
  # @@protoc_insertion_point(class_scope:rbk.protocol.Message_Odometer)
  ))
_sym_db.RegisterMessage(Message_Odometer)


# @@protoc_insertion_point(module_scope)
