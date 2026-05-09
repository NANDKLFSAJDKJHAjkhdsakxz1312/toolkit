#include "ui_cmd_server.h"

#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    UiCmdServer dds_server;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    spdlog::info("收到退出信号，正在清理...");
    return 0;
}