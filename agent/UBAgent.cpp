#include "UBAgent.h"
#include "UBNetwork.h"

#include "UBConfig.h"

#include <QTime>
#include <QTimer>
#include <QCommandLineParser>

#include "Vehicle.h"
#include "TCPLink.h"
#include "QGCApplication.h"

UBAgent::UBAgent(QObject *parent) : QObject(parent),
    m_mav(nullptr)
{
    m_net = new UBNetwork;
    connect(m_net, SIGNAL(dataReady(quint8, QByteArray)), this, SLOT(dataReadyEvent(quint8, QByteArray)));

    m_timer = new QTimer;
    connect(m_timer, SIGNAL(timeout()), this, SLOT(missionTracker()));

    startAgent();
}

void UBAgent::startAgent() {
    QCommandLineParser parser;
    parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);

    parser.addOptions({
        {{"I", "instance"}, "Set instance (ID) of the agent", "id"},
    });

//    parser.process(*QCoreApplication::instance());
    parser.parse(QCoreApplication::arguments());

    quint8 id = parser.value("I").toUInt();
    LinkConfiguration* link = nullptr;
    if (id) {
        quint32 port = 10 * id + STL_PORT + 3;
        TCPConfiguration* tcp = new TCPConfiguration(tr("TCP Port %1").arg(port));
        tcp->setAddress(QHostAddress::LocalHost);
        tcp->setPort(port);

        link = tcp;
    } else {
        SerialConfiguration* serial = new SerialConfiguration("Serial Port");
        serial->setBaud(BAUD_RATE);
        serial->setPortName(SERIAL_PORT);

        link = serial;
    }

    link->setDynamic();
    link->setAutoConnect();

    LinkManager* linkManager = qgcApp()->toolbox()->linkManager();
    linkManager->addConfiguration(link);
    linkManager->linkConfigurationsChanged();

    connect(qgcApp()->toolbox()->multiVehicleManager(), SIGNAL(vehicleAdded(Vehicle*)), this, SLOT(vehicleAddedEvent(Vehicle*)));
    connect(qgcApp()->toolbox()->multiVehicleManager(), SIGNAL(vehicleRemoved(Vehicle*)), this, SLOT(vehicleRemovedEvent(Vehicle*)));

    m_net->connectToHost(QHostAddress::LocalHost, 10 * id + NET_PORT);
    m_timer->start(1000.0 * MISSION_TRACK_DELAY);

    qInfo() << "Indoors test only, starting MISSION directly (waiting a few seconds before broadcast)";
    m_mission_stage = STAGE_MISSION;
}

void UBAgent::setMAV(Vehicle* mav) {
    if (m_mav) {
        disconnect(m_mav, SIGNAL(armedChanged(bool)), this, SLOT(armedChangedEvent(bool)));
        disconnect(m_mav, SIGNAL(flightModeChanged(QString)), this, SLOT(flightModeChangedEvent(QString)));
    }

    m_mav = mav;

    if (m_mav) {
        connect(m_mav, SIGNAL(armedChanged(bool)), this, SLOT(armedChangedEvent(bool)));
        connect(m_mav, SIGNAL(flightModeChanged(QString)), this, SLOT(flightModeChangedEvent(QString)));
    }
}

void UBAgent::vehicleAddedEvent(Vehicle* mav) {
    if (!mav || m_mav == mav) {
        return;
    }

    setMAV(mav);
    m_net->setID(mav->id());

    qInfo() << "New MAV connected with ID: " << m_mav->id();
}

void UBAgent::vehicleRemovedEvent(Vehicle* mav) {
    if (!mav || m_mav != mav) {
        return;
    }

    setMAV(nullptr);
    m_net->setID(0);

    qInfo() << "MAV disconnected with ID: " << mav->id();
}

void UBAgent::armedChangedEvent(bool armed) {
    if (!armed) {
        if (m_mission_state!=STATE_LAND) { //do not interfere landing procedure.
            m_mission_state = STATE_IDLE;
        }
        return;
    }

//	if (m_mav->altitudeRelative()->rawValue().toDouble() > POINT_ZONE) {
//        qWarning() << "The mission can not start while the drone is airborne!";
//        return;
//    }

    qInfo() << "UAV armed. This is indoors test, we are not going to take off! just resetting the mission";

//    m_mav->setGuidedMode(true);
    if (!m_mav->guidedMode()) {
        qWarning() << "The mission can not start while the drone is not in Guided mode!";
        return;
    }

    m_mission_data.reset();
    m_mission_stage = STAGE_MISSION;

}

void UBAgent::flightModeChangedEvent(QString mode) {
    qInfo() << mode;
    // automatic arm after switching from Land to Guided. be careful!
    if (m_mission_data.previousFlightMode == "Land" && 
        mode=="Guided" && !m_mav->armed()) {
            m_mav->setArmed(true);
    }
    m_mission_data.previousFlightMode = mode;
}

void UBAgent::dataReadyEvent(quint8 srcID, QByteArray data) {
    Q_UNUSED(data)
    qInfo() << "I am UAV " << m_mav->id() << " - Packet received from " << srcID;
}

void UBAgent::missionTracker() {
    switch (m_mission_state) {
    case STATE_IDLE:
        stateIdle();
        break;
    case STATE_TAKEOFF:
        stateTakeoff();
        break;
    case STATE_MISSION:
        stateMission();
        break;
    case STATE_LAND:
        stateLand();
        break;
    default:
        break;
    }
}

void UBAgent::stateIdle() {
}

void UBAgent::stateTakeoff() {
}

void UBAgent::stateLand() {
}

void UBAgent::stateMission() {
    static QGeoCoordinate dest;

    if (m_mission_data.tick < (1.0 / MISSION_TRACK_DELAY)) {
        m_mission_data.tick++;
    } else {
        m_net->sendData(BROADCAST_ID, QByteArray(1, MAV_CMD_NAV_TAKEOFF));
	    qInfo() << "I am UAV " << m_mav->id() << " - Broadcast packet sent!";
	    qInfo() << "Going to idle, you can change mode to LAND and back to GUIDED to repeat mission.";
		m_mission_data.reset();
        m_mission_stage = STAGE_IDLE;
    }
}
