// Auto-generated. Do not edit!

// (in-package feetech_controls.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class torqcontrolRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.idrec = null;
    }
    else {
      if (initObj.hasOwnProperty('idrec')) {
        this.idrec = initObj.idrec
      }
      else {
        this.idrec = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type torqcontrolRequest
    // Serialize message field [idrec]
    bufferOffset = _arraySerializer.int64(obj.idrec, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type torqcontrolRequest
    let len;
    let data = new torqcontrolRequest(null);
    // Deserialize message field [idrec]
    data.idrec = _arrayDeserializer.int64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.idrec.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'feetech_controls/torqcontrolRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6a8e3aac2c8c93d18c703f7cf71f88e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64[] idrec
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new torqcontrolRequest(null);
    if (msg.idrec !== undefined) {
      resolved.idrec = msg.idrec;
    }
    else {
      resolved.idrec = []
    }

    return resolved;
    }
};

class torqcontrolResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.idret = null;
    }
    else {
      if (initObj.hasOwnProperty('idret')) {
        this.idret = initObj.idret
      }
      else {
        this.idret = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type torqcontrolResponse
    // Serialize message field [idret]
    bufferOffset = _arraySerializer.bool(obj.idret, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type torqcontrolResponse
    let len;
    let data = new torqcontrolResponse(null);
    // Deserialize message field [idret]
    data.idret = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.idret.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'feetech_controls/torqcontrolResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e389d92e542b074a29a72824fadc4d53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool[] idret
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new torqcontrolResponse(null);
    if (msg.idret !== undefined) {
      resolved.idret = msg.idret;
    }
    else {
      resolved.idret = []
    }

    return resolved;
    }
};

module.exports = {
  Request: torqcontrolRequest,
  Response: torqcontrolResponse,
  md5sum() { return 'c7fd7facee233a061827f86f586e5d13'; },
  datatype() { return 'feetech_controls/torqcontrol'; }
};
