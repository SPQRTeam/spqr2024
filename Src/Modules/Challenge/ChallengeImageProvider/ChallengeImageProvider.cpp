#include "ChallengeImageProvider.h"
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

MAKE_MODULE(ChallengeImageProvider, behaviorControl);

void ChallengeImageProvider::update(ChallengeImage& theChallengeImage){
    //Establish a connection if not already done
    if (sock == -1 && !establishConnection())
        return;

    //Sent the message and close the connection if something goes wrong. 
    //In the next iteration a new connection will be established.
    if (theFrameInfo.getTimeSince(timeLastSent) >= sendInterval || theFrameInfo.time < timeLastSent) {
        if(!sendImage())
            closeConnection();
        else
            timeLastSent = theFrameInfo.time;
    }
}

bool ChallengeImageProvider::establishConnection() {

    int port;
    if (Thread::getCurrentThreadName() == "Upper")
        port = 5432;
    else if (Thread::getCurrentThreadName() == "Lower")
        port = 5431;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        return false;
    }

    sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(port); 
    if (inet_pton(AF_INET, ip_addr, &server.sin_addr) <= 0) {
        close(sock);
        sock = -1;
        return false;
    }

    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        close(sock);
        sock = -1;
        return false;
    }

    return true;
}

void ChallengeImageProvider::closeConnection() {
    if (sock != -1) {
        close(sock);
        sock = -1;
    }
}

bool ChallengeImageProvider::sendImage() {
    frame = theECImage.grayscaled;
    int imageSize = frame.height * frame.width * sizeof(PixelTypes::GrayscaledPixel);
    int n;
    const PixelTypes::GrayscaledPixel* data = frame.getImage();
    while (imageSize > 0) {
        n = send(sock, data, imageSize, 0);
        if (n < 0) {
            return false;  
        }
        imageSize -= n;
        data += n;
    }
    return true;
}

