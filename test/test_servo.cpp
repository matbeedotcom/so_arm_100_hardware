#include <SCServo_Linux/SCServo.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int main() {
    SMS_STS st3215;
    
    // Check if port exists
    int fd = open("/dev/ttyUSB0", O_RDWR);
    if (fd < 0) {
        std::cerr << "Error opening port: " << strerror(errno) << std::endl;
        return 1;
    }
    close(fd);
    
    std::cout << "Attempting to initialize at 1000000 baud..." << std::endl;
    if(!st3215.begin(1000000, "/dev/ttyUSB0")) {
        std::cerr << "Failed to initialize motors" << std::endl;
        return 1;
    }
    std::cout << "Serial port initialized" << std::endl;
    
    // Try to communicate with each servo
    for(int id = 1; id <= 6; id++) {
        std::cout << "Testing servo " << id << "..." << std::endl;
        
        // First ping the servo
        if (st3215.Ping(id) != -1) {
            std::cout << "  Servo " << id << " responded to ping" << std::endl;
            
            // Set to position control mode
            if (st3215.Mode(id, 0)) {
                std::cout << "  Set to position control mode" << std::endl;
                
                // Now try to read data
                if (st3215.FeedBack(id) != -1) {
                    int pos = st3215.ReadPos(id);
                    double voltage = st3215.ReadVoltage(id) / 10.0;
                    double temp = st3215.ReadTemper(id);
                    int load = st3215.ReadLoad(id);
                    
                    std::cout << "  Position: " << pos << std::endl;
                    std::cout << "  Voltage: " << voltage << "V" << std::endl;
                    std::cout << "  Temperature: " << temp << "°C" << std::endl;
                    std::cout << "  Load: " << load << std::endl;
                } else {
                    std::cout << "  Failed to read feedback" << std::endl;
                }
            } else {
                std::cout << "  Failed to set mode" << std::endl;
            }
        } else {
            std::cout << "  No ping response from servo " << id << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
} 