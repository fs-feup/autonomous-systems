#include "unreal_client.hpp"
#include <iostream>

UnrealClient::UnrealClient(const std::string& host, int port)
    : host_(host), port_(port), connected_(false) {}

UnrealClient::~UnrealClient() {
    if (connected_) {
        // close connection
        close(sockfd_);
        connected_ = false;
    }
}

void UnrealClient::connect() {
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0); // this creates a TCP socket and returns the socket file descriptor
    if (sockfd_ < 0) {
        std::cerr << "Error creating socket\n";
        return;
    }

    sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr)); // initialize to zero to avoid garbage values
    serv_addr.sin_family = AF_INET; // set family to AF_INET for IPv4 -> swap to AF_INET6 for IPv6
    serv_addr.sin_port = htons(port_); // convert port number to network byte order because network is usually big-endian

    if (inet_pton(AF_INET, host_.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported \n";
        return;
    }

    if (::connect(sockfd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed \n";
        return;
    }

    connected_ = true;
    std::cout << "Connected to Unreal at " << host_ << ":" << port_ << "\n";
}

void UnrealClient::sendState(const PhysicsState& state) {
    if (!connected_) return;
    // serialize and send state to Unreal
}
