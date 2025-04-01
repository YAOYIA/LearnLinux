#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <zmq.hpp>

int main() {
    // 创建 ZeroMQ 上下文和 REP 类型的 socket，绑定到端口 5555
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    socket.bind("tcp://*:5555");
    
    std::cout << "C++ 服务器已启动，等待请求..." << std::endl;
    
    while (true) {
        zmq::message_t request;
        // 接收来自 Python 客户端的请求消息
        socket.recv(request, zmq::recv_flags::none);
        std::string req_str(static_cast<char*>(request.data()), request.size());
        std::cout << "接收到请求: " << req_str << std::endl;
        
        // 使用字符串流解析消息
        std::istringstream iss(req_str);
        std::string operation;
        iss >> operation;
        
        // 将剩余部分解析为数字列表
        std::vector<double> numbers;
        double value;
        while (iss >> value) {
            numbers.push_back(value);
        }
        
        std::string reply_str;
        if (operation == "compute_stats") {
            // 计算总和与平均值
            double sum = 0.0;
            for (double num : numbers) {
                sum += num;
            }
            double avg = numbers.empty() ? 0.0 : sum / numbers.size();
            
            // 构造返回的字符串
            reply_str = "sum=" + std::to_string(sum) + " average=" + std::to_string(avg);
        } else {
            reply_str = "error: unsupported operation";
        }
        
        // 将返回字符串发送回客户端
        zmq::message_t reply(reply_str.size());
        memcpy(reply.data(), reply_str.c_str(), reply_str.size());
        socket.send(reply, zmq::send_flags::none);
    }
    
    return 0;
}
