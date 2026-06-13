#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <exception>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

//const char* SOCKET_PATH = "/tmp/munin.sock";

class SocketListener {
public:
    const char* sock_path = "";
    int server_endpoint = -1;
    int client_endpoint = -1;
    char data[256];
    bool peerConnected = false;

    SocketListener() = delete;
    SocketListener(std::string socket_path) : sock_path(socket_path.c_str()) {
        
        unlink(this->sock_path);

        this->server_endpoint = socket(AF_UNIX, SOCK_STREAM, 0);

        if (this->server_endpoint == -1) {
            // TODO: Output the path of the UNIX socket as well
            std::runtime_error("Failed to get an endpoint for the UNIX socket");
        }

        unlink(this->sock_path);
        struct sockaddr_un addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, this->sock_path, sizeof(addr.sun_path) - 1);

        if (bind(this->server_endpoint, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
            close(this->server_endpoint);
            std::runtime_error("Failed to bind the socket to the endpoint");
        }

        if (listen(this->server_endpoint, 5) == -1) {
            std::cerr << "Listen failed\n";
            close(this->server_endpoint);
            std::runtime_error("Failed to listen to the socket. Try running with sudo.");
        }
    };

    ~SocketListener() {
        close(this->server_endpoint);
        close(this->client_endpoint);
        unlink(this->sock_path);
    }

    /**
     * @brief
     * @note This method is blocking due to the call to `accept`
     */
    void attemptConnection() {
        this->client_endpoint = accept(this->server_endpoint, nullptr, nullptr);
        if (this->client_endpoint == -1) {
            std::runtime_error("Failed to accept socket connection");
        } else {
            this->peerConnected = true;
        }
    }

    std::string fetchData() {
        std::memset(data, 0, sizeof(data));
        //char old_data[256];
        //memcpy(old_data, this->data,256);
        ssize_t bytes_received = recv(this->client_endpoint, data, sizeof(data) - 1, 0);
        if (!this->peerConnected) {
            this->attemptConnection();
        } 

        if (bytes_received == 0) {
            //memcpy(this->data, old_data, 256);
            std::cout << "Peer disconnected from the socket\n";
            this->peerConnected = false;
            return "";
        } else if (bytes_received < 0){
            std::cout << "Failed the read from socket"<< std::strerror(errno) << errno<<"\n";
            return "";
        } else {
            //std::cout << this->data << "\n";   
            return this->data;
        }
    }
};