#ifndef SHARED_SPI_API_H
#define SHARED_SPI_API_H

#include "spi_messaging.h"
#include "spi_protocol.h"

#include "depthai-shared/datatype/DatatypeEnum.hpp"
// TODO - unneeded, preferably move to a common include
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"
#include "depthai-shared/datatype/RawSpatialImgDetections.hpp"
#include "depthai-shared/datatype/RawSpatialLocations.hpp"
#include "depthai-shared/datatype/RawSystemInformation.hpp"
#include "depthai-shared/datatype/RawTracklets.hpp"

namespace dai {
static const char* NOSTREAM = "";

struct Data {
    uint32_t size;
    uint8_t* data;
};

struct Metadata {
    uint32_t size;
    uint8_t* data;
    dai::DatatypeEnum type;
};

struct Message {
    Data raw_data;
    Metadata raw_meta;
    dai::DatatypeEnum type;     // exposing type here as well, for easier access.
};


class SpiApi {
    private:
        uint8_t (*send_spi_impl)(const char* spi_send_packet);
        uint8_t (*recv_spi_impl)(char* recvbuf);

        void (*chunk_message_cb)(char* curr_packet, uint32_t chunk_size, uint32_t message_size);

        SpiProtocolInstance* spi_proto_instance;
        SpiProtocolPacket* spi_send_packet;

        uint8_t generic_send_spi(const char* spi_send_packet);
        uint8_t generic_recv_spi(char* recvbuf);
        uint8_t spi_get_size(SpiGetSizeResp *response, spi_command get_size_cmd, const char * stream_name);
        uint8_t spi_get_message(SpiGetMessageResp *response, spi_command get_mess_cmd, const char * stream_name, uint32_t size);
        uint8_t spi_get_message_partial(SpiGetMessageResp *response, const char * stream_name, uint32_t offset, uint32_t size);
    public:
        SpiApi();
        ~SpiApi();

        // debug stuff
        void debug_print_hex(uint8_t * data, int len);
        void debug_print_char(char * data, int len);

        // refs to callbacks
        void set_send_spi_impl(uint8_t (*passed_send_spi)(const char*));
        void set_recv_spi_impl(uint8_t (*passed_recv_spi)(char*));

        // base SPI API methods
        std::vector<std::string> spi_get_streams();
        uint8_t spi_pop_messages();
        uint8_t spi_pop_message(const char * stream_name);
        uint8_t req_message(Message* received_msg, const char* stream_name);
        void free_message(Message* received_msg);

        // methods for requesting only metadata or data
        uint8_t req_data(Data *requested_data, const char* stream_name);
        uint8_t req_metadata(Metadata *requested_data, const char* stream_name);
        uint8_t req_data_partial(Data *requested_data, const char* stream_name, uint32_t offset, uint32_t offset_size);

        template<typename MSG>
        void parse_message(const uint8_t* meta_pointer, int meta_length, MSG& obj){
            nlohmann::json jser = nlohmann::json::from_msgpack(meta_pointer, meta_pointer + (meta_length));
            nlohmann::from_json(jser, obj);
        }

        template<typename MSG>
        void parse_metadata(Metadata *passed_metadata, MSG& parsed_return){
            parse_message(passed_metadata->data, passed_metadata->size, parsed_return);
        }

        // methods for receiving a large message piece by piece
        void chunk_message(const char* stream_name);
        void set_chunk_packet_cb(void (*passed_chunk_message_cb)(char*, uint32_t, uint32_t));
};




}  // namespace dai

#endif
