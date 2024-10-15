#include "CommandReceiver.h"
#include <iostream>
MAKE_MODULE(CommandReceiver, infrastructure);

CommandReceiver::CommandReceiver(){
    if(theRobotInfo.number != RobotInfo::RoleNumber::controlled) return;

    bool result = socket_read.bind(SERVER_IP.c_str(), RECEIVE_PORT);
    socket_read.setBlocking(false);

    socket_write.setTarget(SERVER_IP.c_str(), SEND_PORT);
    socket_write.setBlocking(false);
}


void CommandReceiver::update(HumanCommand& command) {

    if (theRobotInfo.number != RobotInfo::RoleNumber::controlled) return;
    
    char buffer[BUFFER_SIZE];
    int n = socket_read.read(buffer, BUFFER_SIZE);

    // Primo campo (command): unsigned char
    // se il campo buffer[0] (campo del body motion) è 0, non sovrascrivere comando
    HumanCommand::CommandBody received_command_body = static_cast<HumanCommand::CommandBody>(buffer[0]);
    // Secondo campo (command): unsigned char
    // se il campo buffer[1] (campo del head motion) è 0, non sovrascrivere comando
    HumanCommand::CommandHead received_command_head = static_cast<HumanCommand::CommandHead>(buffer[1]);

    // Terzo campo (strategy): unsigned char
    HumanCommand::Strategy strategy = static_cast<HumanCommand::Strategy>(buffer[2]);

    // Quarto campo (x_pos): int (4 byte)
    float x_pos;
    std::memcpy(&x_pos, buffer + 3, sizeof(x_pos));

    // Quinto campo (y_pos): int (4 byte)
    float y_pos;
    std::memcpy(&y_pos, buffer + 7, sizeof(y_pos));

    if (n > 0) {
        OUTPUT_TEXT("Received command:");
        OUTPUT_TEXT(" - command body: " << HumanCommand::CommandBody2String(received_command_body));
        OUTPUT_TEXT(" - command head: " << HumanCommand::CommandHead2String(received_command_head));
        OUTPUT_TEXT(" - strategy: " << HumanCommand::Strategy2String(strategy));
        OUTPUT_TEXT(" - x = " << x_pos << ", y = " << y_pos);
        OUTPUT_TEXT("----------------------------------");

        SystemCall::say(HumanCommand::CommandBody2String(received_command_body));

        if(received_command_body != HumanCommand::CommandBody::BaseCommandBody){
            command.commandBody = received_command_body;
            command.x = x_pos;
            command.y = y_pos;
        }
        if(received_command_head != HumanCommand::CommandHead::BaseCommandHead)
            command.commandHead = received_command_head;
    
        command.strategy = strategy;
    }
    return;
}