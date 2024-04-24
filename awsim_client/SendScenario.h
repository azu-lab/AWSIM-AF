#ifndef SENDSCENARIO_H
#define SENDSCENARIO_H

#include <QThread>
#include <QTcpSocket>

class SendScenarioThread : public QThread
{
    Q_OBJECT

public:
    QTcpSocket* tcpSocket;
    SendScenarioThread();
    void run() override;

};

#endif // SENDSCENARIO_H
