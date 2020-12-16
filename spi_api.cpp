#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <cassert>

#include "spi_api.hpp"

#include "SpiPacketParser.hpp"

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"
#include "depthai-shared/datatype/RawImgDetections.hpp"

#define DEBUG_CMD 0
#define debug_cmd_print(...) \
    do { if (DEBUG_CMD) fprintf(stderr, __VA_ARGS__); } while (0)

#define DEBUG_MESSAGE_CONTENTS 0

namespace dai {

void SpiApi::debug_print_hex(uint8_t * data, int len){
    for(int i=0; i<len; i++){
        if(i%80==0){
            printf("\n");
        }
        printf("%02x", data[i]);
    }
    printf("\n");
}

void SpiApi::debug_print_char(char * data, int len){
    for(int i=0; i<len; i++){
        printf("%c", data[i]);
    }
    printf("\n");
}



SpiApi::SpiApi(){
    chunk_message_cb = NULL;

    spi_proto_instance = (SpiProtocolInstance*) malloc(sizeof(SpiProtocolInstance));
    spi_send_packet = (SpiProtocolPacket*) malloc(sizeof(SpiProtocolPacket));
    spi_protocol_init(spi_proto_instance);
}

SpiApi::~SpiApi(){
    free(spi_proto_instance);
    free(spi_send_packet);
}

void SpiApi::set_send_spi_impl(uint8_t (*passed_send_spi)(char*)){
    send_spi_impl = passed_send_spi;
}

void SpiApi::set_recv_spi_impl(uint8_t (*passed_recv_spi)(char*)){
    recv_spi_impl = passed_recv_spi;
}

uint8_t SpiApi::generic_send_spi(char* spi_send_packet){
    return (*send_spi_impl)(spi_send_packet); 
}

uint8_t SpiApi::generic_recv_spi(char* recvbuf){
    return (*recv_spi_impl)(recvbuf);
}

uint8_t SpiApi::spi_get_size(SpiGetSizeResp *response, spi_command get_size_cmd, const char * stream_name){
    assert(isGetSizeCmd(get_size_cmd));

    uint8_t success = 0;
    debug_cmd_print("sending spi_get_size cmd.\n");
    spi_generate_command(spi_send_packet, get_size_cmd, strlen(stream_name)+1, stream_name);
    generic_send_spi((char*)spi_send_packet);

    debug_cmd_print("receive spi_get_size response from remote device...\n");
    char recvbuf[BUFF_MAX_SIZE] = {0};
    uint8_t recv_success = generic_recv_spi(recvbuf);

    if(recv_success){
        if(recvbuf[0]==START_BYTE_MAGIC){
            SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spi_proto_instance, (uint8_t*)recvbuf, sizeof(recvbuf));
            spi_parse_get_size_resp(response, spiRecvPacket->data);
            success = 1;
            
        }else if(recvbuf[0] != 0x00){
            printf("*************************************** got a half/non aa packet ************************************************\n");
            success = 0;
        }
    } else {
        printf("failed to recv packet\n");
        success = 0;
    }

    return success;
}

uint8_t SpiApi::spi_get_message(SpiGetMessageResp *response, spi_command get_mess_cmd, const char * stream_name, uint32_t size){
    assert(isGetMessageCmd(get_mess_cmd));

    uint8_t success = 0;
    debug_cmd_print("sending spi_get_message cmd.\n");
    spi_generate_command(spi_send_packet, get_mess_cmd, strlen(stream_name)+1, stream_name);
    generic_send_spi((char*)spi_send_packet);

    uint32_t total_recv = 0;
    int debug_skip = 0;
    while(total_recv < size){
        if(debug_skip%20 == 0){
            debug_cmd_print("receive spi_get_message response from remote device... %d/%d\n", total_recv, size);
        }
        debug_skip++;

        char recvbuf[BUFF_MAX_SIZE] = {0};
        uint8_t recv_success = generic_recv_spi(recvbuf);
        if(recv_success){
            if(recvbuf[0]==START_BYTE_MAGIC){
                SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spi_proto_instance, (uint8_t*)recvbuf, sizeof(recvbuf));
                uint32_t remaining_data = size-total_recv;
                if ( remaining_data < PAYLOAD_MAX_SIZE ){
                    memcpy(response->data+total_recv, spiRecvPacket->data, remaining_data);
                    total_recv += remaining_data;
                } else {
                    memcpy(response->data+total_recv, spiRecvPacket->data, PAYLOAD_MAX_SIZE);
                    total_recv += PAYLOAD_MAX_SIZE;
                }

            }else if(recvbuf[0] != 0x00){
                printf("*************************************** got a half/non aa packet ************************************************\n");
                break;
            }
        } else {
            printf("failed to recv packet\n");
            break;
        }
    }


    if(total_recv==size){
        spi_parse_get_message(response, size, get_mess_cmd);

        if(DEBUG_MESSAGE_CONTENTS){
            printf("data_size: %d\n", response->data_size);
            debug_print_hex((uint8_t*)response->data, response->data_size);
        }
        success = 1;
    } else {
        success = 0;
    }

    return success;
}








uint8_t SpiApi::spi_pop_messages(){
    SpiStatusResp response;
    uint8_t success = 0;

    debug_cmd_print("sending POP_MESSAGES cmd.\n");
    spi_generate_command(spi_send_packet, POP_MESSAGES, strlen(NOSTREAM)+1, NOSTREAM);
    generic_send_spi((char*)spi_send_packet);

    debug_cmd_print("receive POP_MESSAGES response from remote device...\n");
    char recvbuf[BUFF_MAX_SIZE] = {0};
    uint8_t recv_success = generic_recv_spi(recvbuf);

    if(recv_success){
        if(recvbuf[0]==START_BYTE_MAGIC){
            SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spi_proto_instance, (uint8_t*)recvbuf, sizeof(recvbuf));
            spi_status_resp(&response, spiRecvPacket->data);
            if(response.status == SPI_MSG_SUCCESS_RESP){
                success = 1;
            }
            
        }else if(recvbuf[0] != 0x00){
            printf("*************************************** got a half/non aa packet ************************************************\n");
            success = 0;
        }
    } else {
        printf("failed to recv packet\n");
        success = 0;
    }

    return success;
}

uint8_t SpiApi::spi_pop_message(const char * stream_name){
    uint8_t success = 0;
    SpiStatusResp response;

    debug_cmd_print("sending POP_MESSAGE cmd.\n");
    spi_generate_command(spi_send_packet, POP_MESSAGE, strlen(stream_name)+1, stream_name);
    generic_send_spi((char*)spi_send_packet);

    debug_cmd_print("receive POP_MESSAGE response from remote device...\n");
    char recvbuf[BUFF_MAX_SIZE] = {0};
    uint8_t recv_success = generic_recv_spi(recvbuf);

    if(recv_success){
        if(recvbuf[0]==START_BYTE_MAGIC){
            SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spi_proto_instance, (uint8_t*)recvbuf, sizeof(recvbuf));
            spi_status_resp(&response, spiRecvPacket->data);
            if(response.status == SPI_MSG_SUCCESS_RESP){
                success = 1;
            }
            
        }else if(recvbuf[0] != 0x00){
            printf("*************************************** got a half/non aa packet ************************************************\n");
            success = 0;
        }
    } else {
        printf("failed to recv packet\n");
        success = 0;
    }

    return success;
}

std::vector<std::string> SpiApi::spi_get_streams(){
    SpiGetStreamsResp response;
    std::vector<std::string> streams;

    debug_cmd_print("sending GET_STREAMS cmd.\n");
    spi_generate_command(spi_send_packet, GET_STREAMS, 1, NOSTREAM);
    generic_send_spi((char*)spi_send_packet);

    debug_cmd_print("receive GET_STREAMS response from remote device...\n");
    char recvbuf[BUFF_MAX_SIZE] = {0};
    uint8_t recv_success = generic_recv_spi(recvbuf);

    if(recv_success){
        if(recvbuf[0]==START_BYTE_MAGIC){
            SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spi_proto_instance, (uint8_t*)recvbuf, sizeof(recvbuf));
            spi_parse_get_streams_resp(&response, spiRecvPacket->data);

            std::string currStr;            
            for(int i=0; i<response.numStreams; i++){
                currStr = response.stream_names[i];
                streams.push_back(currStr);
            }
        }else if(recvbuf[0] != 0x00){
            printf("*************************************** got a half/non aa packet ************************************************\n");
        }
    } else {
        printf("failed to recv packet\n");
    }

    return streams;
}


uint8_t SpiApi::req_data(Data *requested_data, const char* stream_name){
    uint8_t req_success = 0;
    SpiGetMessageResp get_message_resp;

    // do a get_size before trying to retreive message.
    SpiGetSizeResp get_size_resp;
    req_success = spi_get_size(&get_size_resp, GET_SIZE, stream_name);
    debug_cmd_print("response: %d\n", get_size_resp.size);

    // get message (assuming we got size)
    if(req_success){
        get_message_resp.data = (uint8_t*) malloc(get_size_resp.size);
        if(!get_message_resp.data){
            printf("failed to allocate %d bytes", get_size_resp.size);
        }

        req_success = spi_get_message(&get_message_resp, GET_MESSAGE, stream_name, get_size_resp.size);
        if(req_success){
            requested_data->data = get_message_resp.data;
            requested_data->size = get_message_resp.data_size;
        }
    }

    return req_success;
}

uint8_t SpiApi::req_metadata(Metadata *requested_data, const char* stream_name){
    uint8_t req_success = 0;
    SpiGetMessageResp get_message_resp;

    // do a get_size before trying to retreive message.
    SpiGetSizeResp get_size_resp;
    req_success = spi_get_size(&get_size_resp, GET_METASIZE, stream_name);
    debug_cmd_print("response: %d\n", get_size_resp.size);

    // get message (assuming we got size)
    if(req_success){
        get_message_resp.data = (uint8_t*) malloc(get_size_resp.size);
        if(!get_message_resp.data){
            printf("failed to allocate %d bytes", get_size_resp.size);
        }

        req_success = spi_get_message(&get_message_resp, GET_METADATA, stream_name, get_size_resp.size);
        if(req_success){
            requested_data->data = get_message_resp.data;
            requested_data->size = get_message_resp.data_size;
            requested_data->type = (dai::DatatypeEnum) get_message_resp.data_type;
        }
    }

    return req_success;
}




uint8_t SpiApi::req_message(Message* received_msg, const char* stream_name){
    uint8_t req_success = 0;
    uint8_t req_data_success = 0;
    uint8_t req_meta_success = 0;

    Metadata raw_meta;
    Data raw_data;


    // ----------------------------------------
    // example of receiving messages.
    // ----------------------------------------
    // the req_data method allocates memory for the received packet. we need to be sure to free it when we're done with it.
    req_data_success = req_data(&raw_data, stream_name);

    // ----------------------------------------
    // example of getting message metadata
    // ----------------------------------------
    // the req_metadata method allocates memory for the received packet. we need to be sure to free it when we're done with it.
    req_meta_success = req_metadata(&raw_meta, stream_name);

    
    if(req_data_success && req_meta_success){
        received_msg->raw_data = raw_data;
        received_msg->raw_meta = raw_meta;
        received_msg->type = raw_meta.type;
        req_success = 1;
    }

    return req_success;
}

void SpiApi::free_message(Message* received_msg){
    free(received_msg->raw_data.data);
    free(received_msg->raw_meta.data);
}



void SpiApi::set_chunk_packet_cb(void (*passed_chunk_message_cb)(char*, uint32_t, uint32_t)){
    chunk_message_cb = passed_chunk_message_cb;
}

void SpiApi::chunk_message(const char* stream_name){
    uint8_t req_success = 0;

    // do a get_size before trying to retreive message.
    SpiGetSizeResp get_size_resp;
    req_success = spi_get_size(&get_size_resp, GET_SIZE, stream_name);
    debug_cmd_print("get_size_resp: %d\n", get_size_resp.size);

    if(req_success){
        // send a get message command (assuming we got size)
        spi_generate_command(spi_send_packet, GET_MESSAGE, strlen(stream_name)+1, stream_name);
        generic_send_spi((char *)spi_send_packet);

        uint32_t message_size = get_size_resp.size;
        uint32_t total_recv = 0;
        while(total_recv < message_size){
            char recvbuf[BUFF_MAX_SIZE] = {0};
            req_success = generic_recv_spi(recvbuf);
            if(req_success){
                if(recvbuf[0]==START_BYTE_MAGIC){
                    SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spi_proto_instance, (uint8_t*)recvbuf, sizeof(recvbuf));
                    uint32_t remaining_data = message_size-total_recv;
                    uint32_t curr_packet_size = 0;
                    if ( remaining_data < PAYLOAD_MAX_SIZE ){
                        curr_packet_size = remaining_data;
                    } else {
                        curr_packet_size = PAYLOAD_MAX_SIZE;
                    }

                    if(chunk_message_cb != NULL){
                        chunk_message_cb((char*)spiRecvPacket->data, curr_packet_size, message_size);
                        if(DEBUG_MESSAGE_CONTENTS){
                            debug_print_hex((uint8_t*)spiRecvPacket->data, curr_packet_size);
                        }
                    } else {
                        printf("WARNING: chunk_message called without setting callback!");
                    }
                    total_recv += curr_packet_size;

                }else if(recvbuf[0] != 0x00){
                    printf("*************************************** got a half/non aa packet ************************************************\n");
                    req_success = 0;
                    break;
                }
            } else {
                printf("failed to recv packet\n");
                req_success = 0;
                break;
            }
        }
    }
}


template<typename T>
void parseMessage(uint8_t* metaPointer, int metaLength, T& obj){
    nlohmann::json jser = nlohmann::json::from_msgpack(metaPointer, metaPointer + (metaLength));
    nlohmann::from_json(jser, obj);
}

template<typename T>
void SpiApi::parse_metadata(Metadata *passed_metadata, T& parsed_return){
    dai::parseMessage(passed_metadata->data, passed_metadata->size, parsed_return);
}

// Explicit template instantiation
template void SpiApi::parse_metadata(Metadata *passed_metadata, dai::RawNNData& parsed_return);
template void SpiApi::parse_metadata(Metadata *passed_metadata, dai::RawImgFrame& parsed_return);
template void SpiApi::parse_metadata(Metadata *passed_metadata, dai::RawImgDetections& parsed_return);



}  // namespace dai

