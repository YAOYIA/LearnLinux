import zmq

def main():
    # 创建 ZeroMQ 上下文和 REQ 类型的 socket
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    
    # 定义任务参数，操作和数字用空格分隔
    message = "compute_stats 10 20 30 40 50"
    print("发送消息到 C++ 服务器:", message)
    socket.send_string(message)
    
    # 等待并接收 C++ 服务器返回的结果
    reply = socket.recv_string()
    print("收到 C++ 服务器返回的结果:", reply)

if __name__ == "__main__":
    main()
