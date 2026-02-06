// Auto-generated. Do not edit!

// (in-package feetech_controls.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class headcontrols {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.arucoid = null;
      this.type = null;
      this.headjointvalues1 = null;
      this.headjointvalues2 = null;
    }
    else {
      if (initObj.hasOwnProperty('arucoid')) {
        this.arucoid = initObj.arucoid
      }
      else {
        this.arucoid = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('headjointvalues1')) {
        this.headjointvalues1 = initObj.headjointvalues1
      }
      else {
        this.headjointvalues1 = 0.0;
      }
      if (initObj.hasOwnProperty('headjointvalues2')) {
        this.headjointvalues2 = initObj.headjointvalues2
      }
      else {
        this.headjointvalues2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type headcontrols
    // Serialize message field [arucoid]
    bufferOffset = _serializer.int16(obj.arucoid, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int16(obj.type, buffer, bufferOffset);
    // Serialize message field [headjointvalues1]
    bufferOffset = _serializer.float32(obj.headjointvalues1, buffer, bufferOffset);
    // Serialize message field [headjointvalues2]
    bufferOffset = _serializer.float32(obj.headjointvalues2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type headcontrols
    let len;
    let data = new headcontrols(null);
    // Deserialize message field [arucoid]
    data.arucoid = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [headjointvalues1]
    data.headjointvalues1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [headjointvalues2]
    data.headjointvalues2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'feetech_controls/headcontrols';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1bdf8924bc3daacda9a890695dd945ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 arucoid
    int16 type
    float32 headjointvalues1
    float32 headjointvalues2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new headcontrols(null);
    if (msg.arucoid !== undefined) {
      resolved.arucoid = msg.arucoid;
    }
    else {
      resolved.arucoid = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.headjointvalues1 !== undefined) {
      resolved.headjointvalues1 = msg.headjointvalues1;
    }
    else {
      resolved.headjointvalues1 = 0.0
    }

    if (msg.headjointvalues2 !== undefined) {
      resolved.headjointvalues2 = msg.headjointvalues2;
    }
    else {
      resolved.headjointvalues2 = 0.0
    }

    return resolved;
    }
};

module.exports = headcontrols;
