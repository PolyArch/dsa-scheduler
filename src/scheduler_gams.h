#ifndef __SB__SCHEDULE_GAMS_H__
#define __SB__SCHEDULE_GAMS_H__

#include "scheduler.h"

class SchedulerGAMS : public GamsScheduler {
	public:
	SchedulerGAMS(SB_CONFIG::SbModel* sbModel) : GamsScheduler(sbModel){}
	bool schedule(SbPDG* sbPDG,Schedule*& schedule);
};

#endif
