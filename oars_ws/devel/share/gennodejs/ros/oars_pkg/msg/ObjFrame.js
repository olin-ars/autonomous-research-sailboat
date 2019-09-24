// Auto-generated. Do not edit!

// (in-package oars_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObjFrame {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.left = null;
      this.right = null;
      this.top = null;
      this.bottom = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('top')) {
        this.top = initObj.top
      }
      else {
        this.top = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('bottom')) {
        this.bottom = initObj.bottom
      }
      else {
        this.bottom = new std_msgs.msg.Float32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjFrame
    // Serialize message field [name]
    bufferOffset = std_msgs.msg.String.serialize(obj.name, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.left, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.right, buffer, bufferOffset);
    // Serialize message field [top]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.top, buffer, bufferOffset);
    // Serialize message field [bottom]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.bottom, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjFrame
    let len;
    let data = new ObjFrame(null);
    // Deserialize message field [name]
    data.name = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [top]
    data.top = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [bottom]
    data.bottom = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.name);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'oars_pkg/ObjFrame';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87318109dd924e4711aff92aa4479132';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String name
    std_msgs/Float32 left
    std_msgs/Float32 right
    std_msgs/Float32 top
    std_msgs/Float32 bottom
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjFrame(null);
    if (msg.name !== undefined) {
      resolved.name = std_msgs.msg.String.Resolve(msg.name)
    }
    else {
      resolved.name = new std_msgs.msg.String()
    }

    if (msg.left !== undefined) {
      resolved.left = std_msgs.msg.Float32.Resolve(msg.left)
    }
    else {
      resolved.left = new std_msgs.msg.Float32()
    }

    if (msg.right !== undefined) {
      resolved.right = std_msgs.msg.Float32.Resolve(msg.right)
    }
    else {
      resolved.right = new std_msgs.msg.Float32()
    }

    if (msg.top !== undefined) {
      resolved.top = std_msgs.msg.Float32.Resolve(msg.top)
    }
    else {
      resolved.top = new std_msgs.msg.Float32()
    }

    if (msg.bottom !== undefined) {
      resolved.bottom = std_msgs.msg.Float32.Resolve(msg.bottom)
    }
    else {
      resolved.bottom = new std_msgs.msg.Float32()
    }

    return resolved;
    }
};

module.exports = ObjFrame;
