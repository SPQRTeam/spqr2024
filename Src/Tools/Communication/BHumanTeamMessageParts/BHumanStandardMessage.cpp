/**
 * @file BHumanStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHumanStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include <algorithm>

template<typename T>
inline void writeVal(void*& data, T value)
{
  *reinterpret_cast<T*&>(data)++ = value;
}

template<typename T>
inline T readVal(const void*& data)
{
  return *reinterpret_cast<T*&>(data)++;
}

#if BHUMAN_MESSAGE_INCLUDE_HEADER
BHumanStandardMessage::BHumanStandardMessage() :
  version(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
{
  const char* init = BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER;
  for(unsigned int i = 0; i < sizeof(header); ++i)
    header[i] = init[i];
}
#else
BHumanStandardMessage::BHumanStandardMessage(){}
#endif

int BHumanStandardMessage::sizeOfBHumanMessage() const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 13, "This method is not adjusted for the current message version");

  #if BHUMAN_MESSAGE_INCLUDE_HEADER
  return sizeof(header)
         + sizeof(version)
         + sizeof(magicNumber)
         + sizeof(timestamp)
  #else
  return sizeof(timestamp)
  #endif
         + sizeof(uint32_t) // size of compressedContainer (23 bits), requestsNTPMessage (1 bit), NTP reply bitset (8 bits)
         + static_cast<int>(ntpMessages.size()) * 5
         + static_cast<int>(compressedContainer.size());
}

void BHumanStandardMessage::write(void* data) const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 13, "This method is not adjusted for the current message version");

  const void* const begin = data; //just for length check

  #if BHUMAN_MESSAGE_INCLUDE_HEADER
  for(unsigned i = 0; i < sizeof(header); ++i)
    writeVal<char>(data, header[i]);
  writeVal<uint8_t>(data, version);
  writeVal<uint8_t>(data, magicNumber);
  #endif
  writeVal<uint32_t>(data, timestamp);

  static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 8, "This code only works for exactly eight robots per team.");
  std::sort(const_cast<std::vector<BNTPMessage>&>(ntpMessages).begin(), const_cast<std::vector<BNTPMessage>&>(ntpMessages).end(), [&](const BNTPMessage& a, const BNTPMessage& b) {return a.receiver < b.receiver; });
  uint16_t ntpReceivers = 0;
  if(!ntpMessages.empty())
  {
    auto ntpMessagesItr = ntpMessages.cbegin();
    for(unsigned int i = 0; i < BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, ntpReceivers <<= 1)
    {
      if(ntpMessagesItr == ntpMessages.cend())
        continue;
      else if(ntpMessagesItr->receiver == i + 1)
      {
        ntpReceivers |= 1;
        ++ntpMessagesItr;
        ASSERT(ntpMessagesItr == ntpMessages.cend() || ntpMessagesItr->receiver > i + 1);
      }
    }
  }
  ASSERT(compressedContainer.size() < (1ull << 23));
  writeVal<uint32_t>(data, static_cast<uint32_t>((compressedContainer.size() << 9) | (requestsNTPMessage ? (1u << 8) : 0u) | (ntpReceivers >> 1)));

  for(const BNTPMessage& ntpMessage : ntpMessages)
  {
    const uint32_t requestReceiptDiffCutted = std::min(timestamp - ntpMessage.requestReceipt, 0xFFFu);
    writeVal<uint32_t>(data, ntpMessage.requestOrigination | ((requestReceiptDiffCutted & 0xF00) << 20));
    writeVal<uint8_t>(data, requestReceiptDiffCutted & 0xFF);
  }

  std::memcpy(data, compressedContainer.data(), compressedContainer.size());
  reinterpret_cast<char*&>(data) += compressedContainer.size();
  //std::cout << "valore effettivo scritto = " << (reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) << ", sizeofbhmessage = " << sizeOfBHumanMessage() << std::endl; //SIM-D
  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
}

bool BHumanStandardMessage::read(const void* data)
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 13, "This method is not adjusted for the current message version");

  const void* const begin = data; //just for length check

  ntpMessages.clear();

  #if BHUMAN_MESSAGE_INCLUDE_HEADER

  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != readVal<const char>(data))
      return false;

  version = readVal<const uint8_t>(data);
  if(version != BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
    return false;

  magicNumber = readVal<const uint8_t>(data);

  #endif

  timestamp = readVal<const uint32_t>(data);

  static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 8, "This code only works for exactly eight robots per team (but can be easily adjusted).");
  const uint32_t ntpAndSizeContainer = readVal<const uint32_t>(data);
  compressedContainer.resize(ntpAndSizeContainer >> 9);
  requestsNTPMessage = (ntpAndSizeContainer & (1u << 8)) != 0;
  uint16_t runner = 1u << 8;
  for(uint8_t i = 1; runner != 0; ++i)
  {
    if(ntpAndSizeContainer & (runner >>= 1))
    {
      ntpMessages.emplace_back();
      BNTPMessage& message = ntpMessages.back();
      message.receiver = i;

      const uint32_t timeStruct32 = readVal<const uint32_t>(data);
      const uint8_t timeStruct8 = readVal<const uint8_t>(data);

      message.requestOrigination = timeStruct32 & 0xFFFFFFF;
      message.requestReceipt = timestamp - (((timeStruct32 >> 20) & 0xF00) | static_cast<uint32_t>(timeStruct8));
    }
  }

  std::memcpy(compressedContainer.data(), data, compressedContainer.size());
  reinterpret_cast<const char*&>(data) += compressedContainer.size();
  //std::cout << "valore effettivo letto = " << (reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) << ", sizeofbhmessage = " << sizeOfBHumanMessage() << std::endl; //SIM-D
  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
  return true;
}