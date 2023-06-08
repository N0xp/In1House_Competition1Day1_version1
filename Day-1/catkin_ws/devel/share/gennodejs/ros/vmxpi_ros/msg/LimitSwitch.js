// Auto-generated. Do not edit!

// (in-package vmxpi_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LimitSwitch {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.limitswitch = null;
    }
    else {
      if (initObj.hasOwnProperty('limitswitch')) {
        this.limitswitch = initObj.limitswitch
      }
      else {
        this.limitswitch = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LimitSwitch
    // Serialize message field [limitswitch]
    bufferOffset = _arraySerializer.bool(obj.limitswitch, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LimitSwitch
    let len;
    let data = new LimitSwitch(null);
    // Deserialize message field [limitswitch]
    data.limitswitch = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.limitswitch.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vmxpi_ros/LimitSwitch';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cabe49bdbfdd4a3cf170660bf17038a4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool[] limitswitch
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LimitSwitch(null);
    if (msg.limitswitch !== undefined) {
      resolved.limitswitch = msg.limitswitch;
    }
    else {
      resolved.limitswitch = []
    }

    return resolved;
    }
};

module.exports = LimitSwitch;
