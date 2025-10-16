/*
 This file handles the connection between Unreal Engine and the simulator
*/

#pragma once
#include <string>
#include <iostream>
#include <string>
#include <cstring>      // for memset
#include <arpa/inet.h>  // for sockaddr_in, inet_pton
#include <unistd.h>     // for close()


struct PhysicsState; // forward declaration

class UnrealClient {
public:
    UnrealClient(const std::string& host, int port);
    void connect();
    void sendState(const PhysicsState& state);

private:
    std::string host_;
    int port_;
    bool connected_;
    int sockfd_; // stores socket handle returned by socket()
};
