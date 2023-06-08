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

class UniqueID {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.word1 = null;
      this.word2 = null;
      this.word3 = null;
    }
    else {
      if (initObj.hasOwnProperty('word1')) {
        this.word1 = initObj.word1
      }
      else {
        this.word1 = '';
      }
      if (initObj.hasOwnProperty('word2')) {
        this.word2 = initObj.word2
      }
      else {
        this.word2 = '';
      }
      if (initObj.hasOwnProperty('word3')) {
        this.word3 = initObj.word3
      }
      else {
        this.word3 = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UniqueID
    // Serialize message field [word1]
    bufferOffset = _serializer.string(obj.word1, buffer, bufferOffset);
    // Serialize message field [word2]
    bufferOffset = _serializer.string(obj.word2, buffer, bufferOffset);
    // Serialize message field [word3]
    bufferOffset = _serializer.string(obj.word3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UniqueID
    let len;
    let data = new UniqueID(null);
    // Deserialize message field [word1]
    data.word1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [word2]
    data.word2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [word3]
    data.word3 = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.word1);
    length += _getByteLength(object.word2);
    length += _getByteLength(object.word3);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vmxpi_ros/UniqueID';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd3fe5a40e07321a3fd3e3e35d9b0c93b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string word1
    string word2
    string word3
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UniqueID(null);
    if (msg.word1 !== undefined) {
      resolved.word1 = msg.word1;
    }
    else {
      resolved.word1 = ''
    }

    if (msg.word2 !== undefined) {
      resolved.word2 = msg.word2;
    }
    else {
      resolved.word2 = ''
    }

    if (msg.word3 !== undefined) {
      resolved.word3 = msg.word3;
    }
    else {
      resolved.word3 = ''
    }

    return resolved;
    }
};

module.exports = UniqueID;
