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

class TitanInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.deviceID = null;
      this.verMaj = null;
      this.verMin = null;
      this.verBuild = null;
      this.hardware = null;
      this.hardwareRev = null;
    }
    else {
      if (initObj.hasOwnProperty('deviceID')) {
        this.deviceID = initObj.deviceID
      }
      else {
        this.deviceID = 0;
      }
      if (initObj.hasOwnProperty('verMaj')) {
        this.verMaj = initObj.verMaj
      }
      else {
        this.verMaj = 0;
      }
      if (initObj.hasOwnProperty('verMin')) {
        this.verMin = initObj.verMin
      }
      else {
        this.verMin = 0;
      }
      if (initObj.hasOwnProperty('verBuild')) {
        this.verBuild = initObj.verBuild
      }
      else {
        this.verBuild = 0;
      }
      if (initObj.hasOwnProperty('hardware')) {
        this.hardware = initObj.hardware
      }
      else {
        this.hardware = 0;
      }
      if (initObj.hasOwnProperty('hardwareRev')) {
        this.hardwareRev = initObj.hardwareRev
      }
      else {
        this.hardwareRev = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TitanInfo
    // Serialize message field [deviceID]
    bufferOffset = _serializer.int32(obj.deviceID, buffer, bufferOffset);
    // Serialize message field [verMaj]
    bufferOffset = _serializer.int32(obj.verMaj, buffer, bufferOffset);
    // Serialize message field [verMin]
    bufferOffset = _serializer.int32(obj.verMin, buffer, bufferOffset);
    // Serialize message field [verBuild]
    bufferOffset = _serializer.int32(obj.verBuild, buffer, bufferOffset);
    // Serialize message field [hardware]
    bufferOffset = _serializer.int32(obj.hardware, buffer, bufferOffset);
    // Serialize message field [hardwareRev]
    bufferOffset = _serializer.int32(obj.hardwareRev, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TitanInfo
    let len;
    let data = new TitanInfo(null);
    // Deserialize message field [deviceID]
    data.deviceID = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [verMaj]
    data.verMaj = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [verMin]
    data.verMin = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [verBuild]
    data.verBuild = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [hardware]
    data.hardware = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [hardwareRev]
    data.hardwareRev = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vmxpi_ros/TitanInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c9aee76b3d5cddac787b5c807fa2b41';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 deviceID
    int32 verMaj
    int32 verMin
    int32 verBuild
    int32 hardware
    int32 hardwareRev
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TitanInfo(null);
    if (msg.deviceID !== undefined) {
      resolved.deviceID = msg.deviceID;
    }
    else {
      resolved.deviceID = 0
    }

    if (msg.verMaj !== undefined) {
      resolved.verMaj = msg.verMaj;
    }
    else {
      resolved.verMaj = 0
    }

    if (msg.verMin !== undefined) {
      resolved.verMin = msg.verMin;
    }
    else {
      resolved.verMin = 0
    }

    if (msg.verBuild !== undefined) {
      resolved.verBuild = msg.verBuild;
    }
    else {
      resolved.verBuild = 0
    }

    if (msg.hardware !== undefined) {
      resolved.hardware = msg.hardware;
    }
    else {
      resolved.hardware = 0
    }

    if (msg.hardwareRev !== undefined) {
      resolved.hardwareRev = msg.hardwareRev;
    }
    else {
      resolved.hardwareRev = 0
    }

    return resolved;
    }
};

module.exports = TitanInfo;
