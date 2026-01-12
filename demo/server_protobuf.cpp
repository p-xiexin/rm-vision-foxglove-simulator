#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <queue>
#include <thread>
#include <unordered_set>
#include <condition_variable>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>

#include "foxglove/SceneUpdate.pb.h"

std::atomic<bool> running(true);
std::queue<std::pair<std::string, foxglove::ChannelId>> messageQueue;
std::mutex queueMutex;
std::condition_variable cond_;

// 获取当前时间的纳秒数
static uint64_t nanosecondsSinceEpoch()
{
	return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
						std::chrono::system_clock::now().time_since_epoch())
						.count());
}

// 将指定描述符及其所有依赖项序列化为字符串，用作通道模式
static std::string SerializeFdSet(const google::protobuf::Descriptor *toplevelDescriptor)
{
	google::protobuf::FileDescriptorSet fdSet;
	std::queue<const google::protobuf::FileDescriptor *> toAdd;
	toAdd.push(toplevelDescriptor->file());
	std::unordered_set<std::string> seenDependencies;
	while (!toAdd.empty())
	{
		const google::protobuf::FileDescriptor *next = toAdd.front();
		toAdd.pop();
		next->CopyTo(fdSet.add_file());
		for (int i = 0; i < next->dependency_count(); ++i)
		{
			const auto &dep = next->dependency(i);
			if (seenDependencies.find(dep->name()) == seenDependencies.end())
			{
				seenDependencies.insert(dep->name());
				toAdd.push(dep);
			}
		}
	}
	return fdSet.SerializeAsString();
}

// 设置四元数表示的轴角度
// 参考：https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
static void setAxisAngle(foxglove::Quaternion *q, double x, double y, double z, double angle)
{
	double s = std::sin(angle / 2);
	q->set_x(x * s);
	q->set_y(y * s);
	q->set_z(z * s);
	q->set_w(std::cos(angle / 2));
}

// 后台线程函数，用于发布消息
void backgroundThreadFunction(std::unique_ptr<foxglove::ServerInterface<foxglove::ConnHandle>> &server)
{
	while (running)
	{
		std::pair<std::string, foxglove::ChannelId> msgInfo;
		{
			std::unique_lock<std::mutex> lock(queueMutex);
			// 当队列为空时等待
			cond_.wait(lock, [] { return !messageQueue.empty() || !running; });
			// 检查是否收到终止信号，如果是，则退出循环
			if (!running){
				std::cout << "back ground thread exit" << std::endl;
				break;
			}
			msgInfo = messageQueue.front();
			messageQueue.pop();
			lock.unlock();
		}
		server->broadcastMessage(msgInfo.second, nanosecondsSinceEpoch(),
								 reinterpret_cast<const uint8_t *>(msgInfo.first.data()),
								 msgInfo.first.size());
	}
}

int main()
{
	// 定义日志处理函数
	const auto logHandler = [](foxglove::WebSocketLogLevel, char const *msg)
	{
		std::cout << msg << std::endl;
	};

	// 创建 WebSocket 服务器实例
	foxglove::ServerOptions serverOptions;
	auto server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>(
		"C++ Protobuf example server", logHandler, serverOptions);

	// 定义服务器处理程序
	foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;
	// 订阅处理程序，当有客户端订阅频道时被调用
	hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle)
	{
		std::cout << "first client subscribed to " << chanId << std::endl;
	};
	// 取消订阅处理程序，当最后一个客户端取消订阅频道时被调用
	hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle)
	{
		std::cout << "last client unsubscribed from " << chanId << std::endl;
	};
	server->setHandlers(std::move(hdlrs));
	server->start("0.0.0.0", 8765); // 启动服务器，监听 8765 端口

	// 添加频道并序列化频道模式
	const auto channelIds = server->addChannels({{
		.topic = "example_msg",
		.encoding = "protobuf",
		.schemaName = foxglove::SceneUpdate::descriptor()->full_name(),
		.schema = foxglove::base64Encode(SerializeFdSet(foxglove::SceneUpdate::descriptor())),
	}});
	const auto chanId = channelIds.front();

	// 注册信号处理函数，用于捕获 Ctrl+C 信号
	std::signal(SIGINT, [](int sig)
				{
					running = false;
					cond_.notify_all(); // 通知后台线程结束等待
					std::cerr << "received signal " << sig << ", shutting down" << std::endl;
				});

	// 启动后台线程
	std::thread bgThread(backgroundThreadFunction, std::ref(server));

	// 主循环，持续发送场景更新消息直到接收到关闭信号
	while (running)
	{
		const auto now = nanosecondsSinceEpoch(); // 获取当前时间
		foxglove::SceneUpdate msg;				  // 创建场景更新消息
		auto *entity = msg.add_entities();
		*entity->mutable_timestamp() = google::protobuf::util::TimeUtil::NanosecondsToTimestamp(now); // 设置时间戳
		entity->set_frame_id("root");																  // 设置帧 ID
		auto *cube = entity->add_cubes();															  // 添加立方体
		auto *size = cube->mutable_size();
		size->set_x(1);
		size->set_y(1);
		size->set_z(1);
		auto *position = cube->mutable_pose()->mutable_position(); // 设置立方体位置
		position->set_x(2);
		position->set_y(0);
		position->set_z(0);
		auto *orientation = cube->mutable_pose()->mutable_orientation(); // 设置立方体方向
		setAxisAngle(orientation, 0, 0, 1, double(now) / 1e9 * 0.5);	 // 根据时间设置旋转角度
		auto *color = cube->mutable_color();							 // 设置立方体颜色
		color->set_r(0.6);
		color->set_g(0.2);
		color->set_b(1);
		color->set_a(1);

		const auto serializedMsg = msg.SerializeAsString(); // 序列化消息

		// 将消息放入队列
		{
			std::unique_lock<std::mutex> lock(queueMutex);
			messageQueue.push({serializedMsg, chanId});
			cond_.notify_one(); // 通知后台线程有新消息到达
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 等待 50 毫秒
	}

	// 等待后台线程结束
	if (bgThread.joinable()){
		bgThread.join();
	}

	// 移除频道并停止服务器
	server->removeChannels({chanId});
	server->stop();

	return 0;
}
