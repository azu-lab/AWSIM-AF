#include "SendScenario.h"
#include <QDebug>
#include <iostream>

using namespace std;

SendScenarioThread::SendScenarioThread()
{



}

void SendScenarioThread::run()
{
    tcpSocket = new QTcpSocket();
    tcpSocket->connectToHost("127.0.0.1",qint16(9999));

    if(tcpSocket->waitForConnected())
    {
        qDebug()<<"Connected to server!";
    }
    else
    {
        qDebug()<<"Failed to connect to server.";
        return;
    }
    while(1)
    {

    }
}
