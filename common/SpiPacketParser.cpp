#include "SpiPacketParser.hpp"

#include <cstdio>

// standard
#include <memory>

// libraries
#include <nlohmann/json.hpp>

// shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"
#include "depthai-shared/datatype/RawImgDetections.hpp"


// StreamPacket structure ->  || imgframepixels... , serialized_object, object_type, serialized_object_size ||
// object_type -> DataType(int), serialized_object_size -> int

namespace dai {

// Reads int from little endian format
inline int readIntLE(uint8_t* data) {
    return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
}

std::vector<std::uint8_t> serializeData(const std::shared_ptr<RawBuffer>& data) {
    std::vector<std::uint8_t> ser;
    if(!data) return ser;

    // Serialization:
    // 1. fill vector with bytes from data.data
    // 2. serialize and append metadata
    // 3. append datatype enum (4B LE)
    // 4. append size (4B LE) of serialized metadata

    std::vector<std::uint8_t> metadata;
    DatatypeEnum datatype;
    data->serialize(metadata, datatype);
    uint32_t metadataSize = metadata.size();

    // 4B datatype & 4B metadata size
    std::uint8_t leDatatype[4];
    std::uint8_t leMetadataSize[4];
    for(int i = 0; i < 4; i++) leDatatype[i] = (static_cast<std::int32_t>(datatype) >> (i * 8)) & 0xFF;
    for(int i = 0; i < 4; i++) leMetadataSize[i] = (metadataSize >> i * 8) & 0xFF;

    ser.insert(ser.end(), data->data.begin(), data->data.end());
    ser.insert(ser.end(), metadata.begin(), metadata.end());
    ser.insert(ser.end(), leDatatype, leDatatype + sizeof(leDatatype));
    ser.insert(ser.end(), leMetadataSize, leMetadataSize + sizeof(leMetadataSize));
    return ser;
}

// this is a workaround to prevent using up the limited heap on the ESP32. The serializeData method makes two copies of the data.
// We're essentially working around it by just generating the footer half of the packet and then concatinating it with the data half when sending.
std::vector<std::uint8_t> serializeFooter(const std::shared_ptr<RawBuffer>& data) {
    std::vector<std::uint8_t> ser;
    if(!data) return ser;

    // Serialization:
    // 1. serialize and append metadata
    // 2. append datatype enum (4B LE)
    // 3. append size (4B LE) of serialized metadata

    std::vector<std::uint8_t> metadata;
    DatatypeEnum datatype;
    data->serialize(metadata, datatype);
    uint32_t metadataSize = metadata.size();

    // 4B datatype & 4B metadata size
    std::uint8_t leDatatype[4];
    std::uint8_t leMetadataSize[4];
    for(int i = 0; i < 4; i++) leDatatype[i] = (static_cast<std::int32_t>(datatype) >> (i * 8)) & 0xFF;
    for(int i = 0; i < 4; i++) leMetadataSize[i] = (metadataSize >> i * 8) & 0xFF;

    ser.insert(ser.end(), metadata.begin(), metadata.end());
    ser.insert(ser.end(), leDatatype, leDatatype + sizeof(leDatatype));
    ser.insert(ser.end(), leMetadataSize, leMetadataSize + sizeof(leMetadataSize));
    return ser;
}


}  // namespace dai
