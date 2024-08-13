#include "FaceLightClient.h"
#include <cstdlib>

int main(){
    FaceLightClient client;
    /* Same Color Test */
    client.setAllLed(client.red);
    client.sendCmd();
    usleep(1500000);
    client.setAllLed(client.blue);
    client.sendCmd();
    usleep(1500000);
    client.setAllLed(client.red);
    client.sendCmd();
    usleep(1500000);
    client.setAllLed(client.blue);
    client.sendCmd();
    usleep(1500000);
    client.setAllLed(client.black);
    client.sendCmd();

    return 0;
}
