#include "../include/pacecat_playback.h"

PaceCatPlaybackDriver::PaceCatPlaybackDriver()
{
	m_readerplayback = NULL;
}

PaceCatPlaybackDriver::~PaceCatPlaybackDriver()
{
}

void PaceCatPlaybackDriver::openPlaybackThread()
{
	//raw_bytes暂定3
	m_parserplayback = ParserOpen(3, true, ANYONE, 0, 0, 0, 1, CommandList{}, (char*)"", 0);
	m_hubplayback = new PubHub;
	m_hubplayback->nfan = 0;
	m_hubplayback->offsetangle = -1; 
	pthread_mutex_init(&m_hubplayback->mtx, NULL);
	pthread_cond_init(&m_hubplayback->fans_not_full, NULL);
	m_readerplayback = StartPlaybackReader(m_parserplayback, m_hubplayback, m_argdataplayback.log_path, m_argdataplayback.log_file_format);
}

bool PaceCatPlaybackDriver::checkPlaybackIsRun()
{
    PubHub *hub = m_hubplayback;
    int checkAngle = (m_argdataplayback.from_zero==true?0:180);
	bool isfind=false;
	pthread_mutex_lock(&hub->mtx);
    for (int i = 0; i < hub->nfan; i++)
    {
        int idx = getFirstidx(*(hub->fans[i]), checkAngle);
        if (idx >= 0)
        {
            hub->offsetangle = hub->fans[i]->angle;
            hub->offsetidx   = idx;
            m_parserplayback->isrun = true;
			DEBUG("playback start work,offset angle %d offset idx %d raw_bytes:%d", hub->offsetangle / 10, hub->offsetidx, m_parserplayback->raw_mode);
			isfind=true;
			break;
        }
    }
	pthread_cond_broadcast(&hub->fans_not_full);
    pthread_mutex_unlock(&hub->mtx);
	return isfind;
}

int PaceCatPlaybackDriver::GetPlaybackAllFans(PubHub* pub, ArgDataPlayback& argdata)
{
	PubHub *hub = (PubHub *)pub;
	RawData *fans[MAX_FANS];

	pthread_mutex_lock(&hub->mtx);
	int cnt = 0, nfan = 0;
	for (int i = 1; i < hub->nfan; i++)
	{
		if (hub->fans[i]->angle == hub->offsetangle)
		{
			cnt = i;
			break;
		}
	}
	if (cnt > 0)
	{
		if (hub->offsetidx > 0)
		{
			nfan = cnt + 1;
			for (int i = 0; i <= cnt; i++)
			{
				fans[i] = hub->fans[i];
			}
			for (int i = cnt; i < hub->nfan; i++)
				hub->fans[i - cnt] = hub->fans[i];
			hub->nfan -= cnt;
		}
		else
		{
			nfan = cnt;
			for (int i = 0; i < cnt; i++)
			{
				fans[i] = hub->fans[i];
			}
			for (int i = cnt; i < hub->nfan; i++)
				hub->fans[i - cnt] = hub->fans[i];
			hub->nfan -= cnt;
		}
		pthread_cond_broadcast(&hub->fans_not_full);
	}
	pthread_mutex_unlock(&hub->mtx);

	if (cnt > 0)
	{
		if(argdata.rotation_direction==-1)
		{
			if(fans[0]->counterclockwise!=-1)
			{
				argdata.rotation_direction = fans[0]->counterclockwise;
			}
			else
			{
				argdata.rotation_direction = 1;
			}
		}

		bool circle = true;
		int total = fans[0]->span;

		for (int i = 0; i < nfan - 1; i++)
		{
			if ((fans[i]->angle + fans[i]->span) % 3600 != fans[i + 1]->angle)
			{
				circle = false;
			}
			total += fans[i + 1]->span;
		}

		if (!circle || (total != 3600 && hub->offsetidx == 0) || ((total != 3600 + fans[0]->span) && hub->offsetidx > 0))
		{
			if (!circle)
				ERROR((int)DROP_PACKET, Error::GetErrorString(DROP_PACKET).c_str());

			for (int i = 0; i < cnt; i++)
				delete fans[i];
			cnt = 0;
		}
	}

	if (cnt > 0)
	{
		if (argdata.soft_resample && argdata.resample > 0)
		{
			for (int i = 0; i < cnt; i++)
			{
				int NN = fans[i]->span / (10 * argdata.resample);
				if (NN < fans[i]->N)
				{
					resample(fans[i], NN);
				}
				else if (NN > fans[i]->N)
				{
					ERROR((int)SAMPLE_TOO_SMALL, Error::GetErrorString(SAMPLE_TOO_SMALL).c_str());
				}
			}
		}

		for (int j = hub->offsetidx; j < fans[0]->N; j++)
		{
			hub->consume.push_back(fans[0]->points[j]);
		}
		for (int i = 1; i < cnt; i++)
		{
			for (int j = 0; j < fans[i]->N; j++)
			{
				hub->consume.push_back(fans[i]->points[j]);
			}
		}
		for (int j = 0; j < hub->offsetidx; j++)
		{
			hub->consume.push_back(fans[cnt]->points[j]);
		}

		if(argdata.playback_rate==-1)
		{
			double fantime1 = double(fans[1]->ts[0]) + double(fans[1]->ts[1]) / 1000000000.0;
			double fantime2 = double(fans[2]->ts[0]) + double(fans[2]->ts[1]) / 1000000000.0;
			double point_interval = (fantime2 - fantime1) / fans[1]->N;
			double ti = double(fans[0]->ts[0]) + double(fans[0]->ts[1]) / 1000000000.0;
			double tx = double(fans[nfan-1]->ts[0]) + double(fans[nfan-1]->ts[1]) / 1000000000.0;
			double ts_beg = ti + point_interval * hub->offsetidx;
			double ts_end = tx + point_interval * hub->offsetidx;
			hub->ts_beg[0] = (uint32_t)ts_beg;
			hub->ts_beg[1] = (ts_beg - hub->ts_beg[0]) * 1000000000.0;
			hub->ts_end[0] = (uint32_t)ts_end;
			hub->ts_end[1] = (ts_end - hub->ts_end[0]) * 1000000000.0;
		}
		else //自定义频率
		{
			double scan_time = 1.0 / argdata.playback_rate;

			struct timeval tv;
			gettimeofday(&tv, NULL);
			double ts_beg = tv.tv_sec + tv.tv_usec * 1e-6;
			double ts_end = ts_beg + scan_time;

			hub->ts_beg[0] = (uint32_t)ts_beg;
			hub->ts_beg[1] = (ts_beg - hub->ts_beg[0]) * 1e9;
			hub->ts_end[0] = (uint32_t)ts_end;
			hub->ts_end[1] = (ts_end - hub->ts_end[0]) * 1e9;
		}

		for (int i = 0; i < cnt; i++)
			delete fans[i];

		if (cnt > 0)
		{
			bool res = checkZeroDistance(hub->consume, argdata.error_scale);
			if (res)
				hub->error_num = 0;
			else
			{
				hub->error_num++;
				if (hub->error_num >= argdata.error_circle)
				{
					ERROR((int)DISTANCE_ZERO_LARGE, Error::GetErrorString(DISTANCE_ZERO_LARGE).c_str());
					hub->error_num = 0;
				}
			}
		}

		int N = hub->consume.size();
		double angle_increment = M_PI * 2 / N;
		Fitter *fitter = &argdata.fitter;
		if (fitter->isopen)
			filter(hub->consume, fitter->max_range * 1000, fitter->min_range * 1000, fitter->max_range_difference * 1000, fitter->filter_window, angle_increment);

		// 模拟频率	
		double t0 = hub->ts_beg[0] + hub->ts_beg[1] * 1e-9;
		double t1 = hub->ts_end[0] + hub->ts_end[1] * 1e-9;
		double duration_us = (t1 - t0) * 1e6;

		if(duration_us>=0)
		{
			usleep(duration_us);
		}
	}
	return cnt;
}

double PaceCatPlaybackDriver::ROSAng(double ang)
{
	if (m_argdataplayback.rotation_direction + m_argdataplayback.reversed != 1)
	{
		return ang > 180 ? 360 - ang : -ang;
	}
	else
	{
		return ang > 180 ? ang - 360 : ang;
	}
}

int PaceCatPlaybackDriver::GetCount(std::vector<DataPoint> data, double min_deg, double max_deg, double &min_pos, double &max_pos)
{
	int N = 0;

	for (unsigned int i = 0; i < data.size(); i++)
	{
		double deg = ROSAng(data[i].degree);

		if (deg < min_deg || deg > max_deg)
			continue;
		if (N == 0)
		{
			min_pos = deg;
			max_pos = deg;
		}
		else
		{
			if (min_pos > deg)
				min_pos = deg;
			if (max_pos < deg)
				max_pos = deg;
		}
		N++;
	}
	return N;
}