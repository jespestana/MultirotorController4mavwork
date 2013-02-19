/*
 * ConsoleUI.h
 *
 *  Created on: 15/08/2011
 *      Author: Ignacio Mellado
 *      Modified  : 31/10/2012
 *      Author: Jesus Pestana
 */

#include <atlante.h>
#include <droneproxy.h>
#include <ncurses.h>
#include "MyDrone.h"

class ConsoleUI : public virtual DroneProxy::Logs::LogConsumer, public virtual DroneProxy::Comm::ChannelEventListener {
private:
	WINDOW *logWin;
	MyDrone *drone;
	cvg_bool isInit;
	DroneProxy::Threading::Mutex consoleMutex;
	FeedbackPacket lastDroneInfo;

protected:
	void refreshInfo();
	cvgString channelStateToString(DroneProxy::Comm::Channel::State st);
	void writeLabel(cvg_uint y, cvg_uint x, const char *label, cvg_int valueLength, cvg_double value);

public:
	ConsoleUI(MyDrone *drone);
	virtual ~ConsoleUI();

	void init();
	void destroy();

	void doLoop();
	virtual void logConsume(DroneProxy::Logs::LogProducer *producer, const cvgString &str);

	virtual void gotData(DroneProxy::Comm::Channel *channel, void *data);
};




