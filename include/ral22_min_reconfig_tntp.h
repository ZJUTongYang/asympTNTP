#ifndef _RAL22_MIN_RECONFIG_TNTP_

#include <vector>
#include "definitions.h"

void min_reconfig_tntp(std::vector<ContinuousTrackingMotion> CTM, std::vector<std::vector<int> >& all_topo_motions);

void all_possible_tntp(std::vector<ContinuousTrackingMotion> CTM, std::vector<std::vector<int> >& all_possible_solutions, int next_CTM_index);

#endif