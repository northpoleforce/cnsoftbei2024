#include <iostream>
#include <string>
#include <cstdlib>

int executeRemoteCommand(const std::string& hostname, const std::string& username, const std::string& password, const std::string& command) {
    std::string sshCommand = "sshpass -p '" + password + "' ssh " + username + "@" + hostname + " \"" + command + "\"";
    int result = system(sshCommand.c_str());
    return result;
}

int main() {
    std::string hostname = "192.168.123.13";
    std::string username = "unitree";
    std::string password = "123";  // 明文密码
    std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/start.wav";

    int result = executeRemoteCommand(hostname, username, password, command);
    if (result == 0) {
        std::cout << "Command executed successfully" << std::endl;
    } else {
        std::cout << "Failed to execute command" << std::endl;
    }

    return result;
}