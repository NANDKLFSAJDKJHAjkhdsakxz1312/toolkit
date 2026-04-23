#include "ui_cmd_server.h"

#include <thread>
#include <chrono>

int main() {
      UiCmdServer dds_server;

      while (true) {
          //dds_server.log_match_status();
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      return 0;
}