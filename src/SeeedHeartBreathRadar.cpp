#include "SeeedHeartBreathRadar.h"

SeeedHeartBreathRadar::~SeeedHeartBreathRadar() {
  if( m_extPayloadBuf ) {
    free( m_extPayloadBuf );
  }
}

void SeeedHeartBreathRadar::resetFrame() {
  m_framePos = 0;
  if( m_extPayloadBuf ) {
    free( m_extPayloadBuf );
    m_extPayloadBuf = nullptr;
  }
  m_frame.length = 0;
  m_runningSum = 0;
  m_calcChecksum = 0;
}

constexpr uint8_t SeeedHeartBreathRadar::calcCrc( uint8_t current, uint8_t next ) {
  return 0xFF & (current + next);
}

constexpr uint8_t SeeedHeartBreathRadar::calcCrc( uint8_t current, const uint8_t *next, size_t nextLen ) {
  while (nextLen > 0) {
    current = calcCrc( current, *next );
    next++;
    nextLen--;
  }
  return current;
}

bool SeeedHeartBreathRadar::recvRadarBytes() {
  while (m_serial->available() > 0) {
    uint8_t nextByte = (uint8_t)m_serial->read();
    m_runningSum = calcCrc( m_runningSum, nextByte );
    m_framePos++;
    if (m_framePos == 1) {
      m_frame.head[0] = nextByte;
      if (m_frame.head[0] != HEAD_0) {
        resetFrame();
        if (m_debugLevel > 0) {
          Serial.printf( "byte %02X \n", m_frame.head[0] );
        }
      }
    } else if (m_framePos == 2) {
      m_frame.head[1] = nextByte;
      if (m_frame.head[1] == HEAD_1) {
        //Serial.printf( "msg { %02X%02X, ", m_frame.head[0], m_frame.head[1] );
      } else {
        if (m_debugLevel > 1) {
          Serial.printf( "byte %02X \n", m_frame.head[1] );
        }
        resetFrame();
      }
    } else if (m_framePos == 3) {
      m_frame.topic = nextByte;
      //Serial.printf( "topic = %02X, ", (int)m_frame.topic );
    } else if (m_framePos == 4) {
      m_frame.op = nextByte;
      //Serial.printf( "op = %02X, \n", (int)m_frame.op );
    } else if (m_framePos == 5) {
      #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      m_frame.length = nextByte << 8;
      #else
      m_frame.length = nextByte;
      #endif
    } else if (m_framePos == 6) {
      #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      m_frame.length |= nextByte;
      #else
      m_frame.length |= nextByte << 8;
      #endif
      // regect frames that proport to be too long
      if (m_frame.length > 2048) {
        if (m_debugLevel > 0) {
          Serial.printf( "Frame too long %u \n", (unsigned)m_frame.length );
        }
        resetFrame();
      } else if (m_frame.length > sizeof(m_frame.payloadbuf1)) {
        if (m_extPayloadBuf) {
          free( m_extPayloadBuf );
        }
        m_extPayloadBuf = (decltype(m_extPayloadBuf))malloc( m_frame.length );
      }
    } else if (m_framePos >= 7 && m_framePos < m_frame.length + 7U) {
      if (m_frame.length > sizeof(m_frame.payloadbuf1)) {
        if (m_extPayloadBuf) {
          m_extPayloadBuf[m_framePos - 7] = nextByte;
        }
      } else {
        m_frame.payloadbuf1[m_framePos - 7] = nextByte;
      }
      m_calcChecksum = m_runningSum;
    } else if (m_framePos >= 7 && m_framePos == m_frame.length + 7U) {
      m_frame.checksum = nextByte;
      if (m_frame.checksum != m_calcChecksum) {
        if (m_debugLevel > 0) {
          Serial.printf( "checksum mismatch: expected %u got %u\n", (unsigned)m_calcChecksum, (unsigned)m_frame.checksum );
        }
      }
    } else if (m_framePos >= 8 && m_framePos == m_frame.length + 8U) {
      m_frame.foot[0] = nextByte;
    } else if (m_framePos >= 9 && m_framePos == m_frame.length + 9U) {
      m_frame.foot[1] = nextByte;
      if (m_frame.foot[0] == FOOT_0 && m_frame.foot[1] == FOOT_1) {
        //Serial.printf( "msg { %02X%02X %02X %02X, %u, val=%u \n", m_frame.head[0], m_frame.head[1], m_frame.topic, m_frame.op, (unsigned)m_frame.length, (unsigned)m_frame.payloadbuf1[0] );
        handleFrame();
      } else {
        if (m_debugLevel > 0) {
          Serial.printf( " invalid packet footer %02X%02X (in msg %02X %02X, %u, val=%u)\n", m_frame.foot[0], m_frame.foot[1], m_frame.topic, m_frame.op, (unsigned)m_frame.length, (unsigned)m_frame.payloadbuf1[0] );
        }
      }
      resetFrame();
    }
  }
  return true;
}

bool SeeedHeartBreathRadar::handlePersonInfoFrame() {
  if (m_debugLevel > 1) {
    Serial.print( "Human detection " );
    Serial.print( personInfoOperationToString( (PersonInfoOperation)m_frame.op ) );
    Serial.print( " " );
  }
  switch (m_frame.op) {
  case OD_GET_ANGLE:
    m_angles.first = (unsigned)((m_frame.payloadbuf1[0] << 8) | m_frame.payloadbuf1[1]);
    m_angles.second = (unsigned)((m_frame.payloadbuf1[2] << 8) | m_frame.payloadbuf1[3]);
    if (m_debugLevel > 1) {
      Serial.print( m_angles.first );
      Serial.print( " x " );
      Serial.println( m_angles.second );
    }
    break;
  case OD_GET_MOVEMENT_STATE:
    m_movementState = (MovementVal)m_frame.payloadbuf1[0];
    if (m_debugLevel > 1) {
      Serial.println( movementValToString( m_movementState ) );
    }
    break;
  case OD_GET_MOVEMENT_LEVEL:
    m_movementLevel = m_frame.payloadbuf1[0];
    if (m_debugLevel > 1) {
      Serial.println( (unsigned)m_frame.payloadbuf1[0] );
    }
    break;
  case OD_GET_DISTANCE:
    m_distance = (unsigned)((m_frame.payloadbuf1[0] << 8) | m_frame.payloadbuf1[1]);
    if (m_debugLevel > 1) {
      Serial.println( m_distance );
    }
    break;
  default:
    if (m_debugLevel > 1) {
      if (m_frame.length == 1 ) {
        Serial.println( m_frame.payloadbuf1[0] );
      } else if (m_frame.length == 2) {
        Serial.println( (unsigned)((m_frame.payloadbuf1[0] << 8) | m_frame.payloadbuf1[1]) );
      } else {
        for (size_t byte_i = 0; byte_i < m_frame.length; byte_i++) {
          Serial.printf( "%02X ", (unsigned)m_frame.payloadbuf1[byte_i] );
        }
        Serial.println( "" );
      }
    }
    break;
  }
  return true;
}

bool SeeedHeartBreathRadar::handleRespirationInfoFrame() {
  switch (m_frame.op) {
  case OD_GET_BREATH_STATE:
    m_rr_state = (VitalsStateVal)m_frame.payloadbuf1[0];
    if (m_debugLevel > 1) {
      Serial.print( "Breathing " );
      Serial.println( vitalsStateValToString( m_rr_state ) );
    }
    break;
  case OD_GET_BREATH_RATE:
    m_rr = m_frame.payloadbuf1[0];
    if (m_debugLevel > 1) {
      Serial.print( "Breathing rate " );
      Serial.println( m_rr );
    }
    break;
  case OD_GET_BREATH_INTENSITY:
    if (m_debugLevel > 1) {
      Serial.print( "Breathing intensity " );
      Serial.println( m_frame.payloadbuf1[0] );
    }
    break;
  case OD_GET_BREATH_CONFIDENCE:
    if (m_debugLevel > 1) {
      Serial.print( "Breathing confidence " );
      Serial.println( m_frame.payloadbuf1[0] );
    }
    break;
  case OD_GET_BREATH_WAVE:
    if (m_debugLevel > 1) {
      Serial.print( "Breathing wave " );
      Serial.println( m_frame.payloadbuf1[0] );
    }
    break;
  }
  return true;
}

bool SeeedHeartBreathRadar::handlePersonLocationAnomalyFrame() {
  m_outOfRange = ! m_frame.payloadbuf1[0];
  if (m_outOfRange) {
      m_distance = 0;
  }

  if (m_debugLevel > 1) {
    Serial.print( "person is " );
    Serial.println( m_outOfRange ? "out of range" : "in range" );
  }
  return true;
}

bool SeeedHeartBreathRadar::handleHeartInfoFrame() {
  switch (m_frame.op) {
  case OD_GET_HEART_STATE:
    m_hr_state = (VitalsStateVal)m_frame.payloadbuf1[0];
    if (m_debugLevel > 1) {
      Serial.print( "Heartbeat " );
      Serial.println( vitalsStateValToString( m_hr_state ) );
    }
    break;
  case OD_GET_HEART_RATE:
    m_hr = m_frame.payloadbuf1[0];
    if (m_debugLevel > 1) {
      Serial.print( "Heart rate " );
      Serial.println( m_frame.payloadbuf1[0] );
    }
    break;
  default:
    if (m_debugLevel > 1) {
      Serial.printf( "processing topic %s %s\n", topicToString( (ControlTopic)m_frame.topic ), heartOperationsToString( (HeartOperations)m_frame.op) );
    }
    break;
  }
  return true;
}

bool SeeedHeartBreathRadar::handleHeartbeatIdFrame() {
  if (m_debugLevel > 1) {
    Serial.printf( "processing topic %s\n", topicToString( (ControlTopic)m_frame.topic ) );
  }
  return false;
}

bool SeeedHeartBreathRadar::handleProductInfoFrame() {
  if (m_debugLevel > 1) {
    Serial.printf( "processing topic %s\n", topicToString( (ControlTopic)m_frame.topic ) );
  }
  return false;
}

bool SeeedHeartBreathRadar::handleOtaUpgradeFrame() {
  if (m_debugLevel > 1) {
    Serial.printf( "processing topic %s\n", topicToString( (ControlTopic)m_frame.topic ) );
  }
  return false;
}

bool SeeedHeartBreathRadar::handleRadarTestFrame() {
  if (m_debugLevel > 1) {
    Serial.printf( "processing topic %s\n", topicToString( (ControlTopic)m_frame.topic ) );
  }
  return false;
}

bool SeeedHeartBreathRadar::handleOperatingStatusFrame() {
  if (m_debugLevel > 1) {
    Serial.printf( "processing topic %s\n", topicToString( (ControlTopic)m_frame.topic ) );
  }
  return false;
}

bool SeeedHeartBreathRadar::handleLocationInfoFrame() {
  if (m_debugLevel > 1) {
    Serial.printf( "processing topic %s\n", topicToString( (ControlTopic)m_frame.topic ) );
  }
  return false;
}

bool SeeedHeartBreathRadar::handleSleepInfoFrame() {
  if (m_debugLevel > 1) {
    Serial.printf( "processing topic %s\n", topicToString( (ControlTopic)m_frame.topic ) );
  }
  return false;
}

bool SeeedHeartBreathRadar::handleFrame() {
  switch ((ControlTopic)m_frame.topic) {
  case CD_HEARTBEAT_ID: return handleHeartbeatIdFrame();
  case CD_PRODUCT_INFO: return handleProductInfoFrame();
  case CD_OTA_UPDRADE: return handleOtaUpgradeFrame();
  case CD_RADAR_TEST: return handleRadarTestFrame();
  case CD_OPERATING_STATUS: return handleOperatingStatusFrame();
  case CD_LOCATION_INFO: return handleLocationInfoFrame();
  case CD_LOC_DET_ANOMAL: return handlePersonLocationAnomalyFrame();
  case CD_HUMAN_PRESENCE: return handlePersonInfoFrame();
  case CD_RESPIRATION_INFO: return handleRespirationInfoFrame();
  case CD_SLEEP_INFO: return handleSleepInfoFrame();
  case CD_HEARTBEAT_INFO: return handleHeartInfoFrame();
  default: return true;
  }
}


bool SeeedHeartBreathRadar::sendFrame( const SeeedHeartBreathRadar::Frame &frame ) {
  if (frame.length > 32) {
    return false;
  }
  uint8_t buf[offsetof( Frame, payloadbuf1 ) + frame.length + 1 + 2];
  size_t pos = 0;
  buf[pos++] = HEAD_0;
  buf[pos++] = HEAD_1;
  buf[pos++] = frame.topic;
  buf[pos++] = frame.op;
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  buf[pos++] = 0xFF & (frame.length >> 8);
  buf[pos++] = 0xFF & frame.length;
  #else
  buf[pos++] = 0xFF & frame.length;
  buf[pos++] = 0xFF & (frame.length >> 8);
  #endif
  for (size_t pay_i = 0; pay_i < frame.length; pay_i++) {
    buf[pos++] = frame.payloadbuf1[pay_i];
  }
  size_t checkLen = pos;
  uint8_t checksum = 0;
  for (size_t sum_i = 0; sum_i < checkLen; sum_i++ ) {
    checksum = 0xFF & (checksum + buf[sum_i]);
  }
  buf[pos++] = checksum;
  buf[pos++] = FOOT_0;
  buf[pos++] = FOOT_1;

  return m_serial->write( buf, pos ) > 0;
}

bool SeeedHeartBreathRadar::requestProductInfo( SeeedHeartBreathRadar::ProductInfoOpertion op ) {
  Frame frame = { { HEAD_0, HEAD_1 }, ControlTopic::CD_PRODUCT_INFO, op, 1, { 0x0f } };
  return sendFrame( frame );
}

bool SeeedHeartBreathRadar::requestOperatingStatus() {
  Frame frame = { { HEAD_0, HEAD_1 }, ControlTopic::CD_OPERATING_STATUS, OD_GET_OPERATING_STATUS, 1, { 0x0f } };
  return sendFrame( frame );
}
