/*
    Copyright (c) 2014 Contributors as noted in the AUTHORS file

    Distributed under the Boost Software License, Version 1.0. (See accompanying
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/
#include <arpa/inet.h>

namespace net {

template<class T> T ntoh(T) = delete;
uint32_t ntoh(uint32_t v) { return ntohl(v); }
uint16_t ntoh(uint16_t v) { return ntohs(v); }
uint8_t ntoh(uint8_t v) { return v; }
int8_t ntoh(int8_t v) { return v; }
char ntoh(char v) { return v; }

template<class T> T hton(T) = delete;
uint32_t hton(uint32_t v) { return htonl(v); }
uint16_t hton(uint16_t v) { return htons(v); }
uint8_t hton(uint8_t v) { return v; }
int8_t hton(int8_t v) { return v; }
char hton(char v) { return v; }

} // namespace net
