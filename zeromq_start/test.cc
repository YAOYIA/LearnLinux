// receive_gaussian_params.cpp
#include <zmq.hpp>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main() {
    // 创建 ZeroMQ 上下文和 SUB 套接字
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    
    // 连接到 Python 服务器（如果使用 host 网络，容器与主机共享网络，因此可以使用 localhost）
    subscriber.connect("tcp://localhost:5555");
    
    // 订阅所有消息（空字符串表示订阅所有主题）
    const char* filter = "";
    subscriber.setsockopt(ZMQ_SUBSCRIBE, filter, 0);
    
    std::cout << "Waiting for Gaussian parameters..." << std::endl;
    zmq::message_t message;
    subscriber.recv(message, zmq::recv_flags::none);
    
    std::string msg_str(static_cast<char*>(message.data()), message.size());
    std::cout << "Received message:" << std::endl;
    std::cout << msg_str << std::endl;
    
    try {
        json j = json::parse(msg_str);
        // 打印 means3D 的第一个数据验证
        if (j.contains("means3D") && !j["means3D"].empty()) {
            std::cout << "First means3D: ";
            for (auto &val : j["means3D"][0]) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }
    
    std::cout << "Gaussian parameters received successfully!" << std::endl;
    return 0;
}
