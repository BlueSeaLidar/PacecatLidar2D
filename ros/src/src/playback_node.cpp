#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "../sdk/include/pacecat_playback.h"

PaceCatPlaybackDriver *m_driver = NULL;

void PublishLaserScan(ros::Publisher &laser_pub, HPublish pub)
{
	ArgDataPlayback &argdata = m_driver->getArgDataPlayback();

	PubHub *hub = (PubHub *)pub;
	sensor_msgs::LaserScan msg;
	int N = hub->consume.size();

	msg.header.stamp.sec = hub->ts_beg[0];
	msg.header.stamp.nsec = hub->ts_beg[1];

	double ti = double(hub->ts_beg[0]) + double(hub->ts_beg[1]) / 1000000000.0;
	double tx = double(hub->ts_end[0]) + double(hub->ts_end[1]) / 1000000000.0;
	msg.scan_time = tx - ti;
	msg.time_increment = msg.scan_time / (N-1);

	msg.header.frame_id = argdata.frame_id;

	double min_deg = argdata.min_angle * 180 / M_PI;
	double max_deg = argdata.max_angle * 180 / M_PI;

	msg.range_min = argdata.min_dist;
	msg.range_max = argdata.max_dist;

	if (argdata.with_angle_filter)
	{
		double min_pos, max_pos;
		N = m_driver->GetCount(hub->consume, min_deg, max_deg, min_pos, max_pos);
		msg.angle_min = min_pos * M_PI / 180;
		msg.angle_max = max_pos * M_PI / 180;
		msg.angle_increment = (msg.angle_max - msg.angle_min) / (N - 1);
	}
	else
	{
		if (argdata.from_zero)
		{
			msg.angle_min = 0;
			msg.angle_max = 2 * M_PI * (N - 1) / N;
			msg.angle_increment = M_PI * 2 / N;
		}
		else
		{
			msg.angle_min = -M_PI;
			msg.angle_max = M_PI - (2 * M_PI) / N;
			msg.angle_increment = M_PI * 2 / N;
		}
	}

	msg.intensities.resize(N);
	msg.ranges.resize(N);
	int idx = 0;
	if (argdata.reversed + argdata.rotation_direction != 1)
	{
		for (int i = hub->consume.size() - 1; i >= 0; i--)
		{
			double deg = m_driver->ROSAng(hub->consume[i].degree);

			if (argdata.with_angle_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			double d = hub->consume[i].distance / 1000.0;

			bool custom = false;
			for (int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;  
                    
			msg.intensities[idx] = hub->consume[i].confidence;
			idx++;
		}
	}
	else
	{
		for (int i = 0; i <= hub->consume.size() - 1; i++)
		{
			double deg = m_driver->ROSAng(hub->consume[i].degree);
			if (argdata.with_angle_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			double d = hub->consume[i].distance / 1000.0;

			bool custom = false;
			for (int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;

			msg.intensities[idx] = hub->consume[i].confidence;
			idx++;
		}
	}
	laser_pub.publish(msg);
}

void PublishCloud(ros::Publisher &cloud_pub, HPublish pub)
{
	ArgDataPlayback &argdata = m_driver->getArgDataPlayback();

	PubHub *hub = (PubHub *)pub;
	sensor_msgs::PointCloud msg;
	int N = hub->consume.size();

	msg.header.stamp.sec = hub->ts_beg[0];
	msg.header.stamp.nsec = hub->ts_beg[1];

	msg.header.frame_id = argdata.frame_id;

	double min_deg = argdata.min_angle * 180 / M_PI;
	double max_deg = argdata.max_angle * 180 / M_PI;

	if (argdata.with_angle_filter)
	{
		double min_pos, max_pos;
		N = m_driver->GetCount(hub->consume, min_deg, max_deg, min_pos, max_pos);
	}

	msg.points.resize(N);
	msg.channels.resize(1);

	msg.channels[0].name = "intensities";
	msg.channels[0].values.resize(N);

	int idx = 0;

	if (argdata.reversed + argdata.rotation_direction != 1)
	{
		for (int i = hub->consume.size() - 1; i >= 0; i--)
		{
			double deg = m_driver->ROSAng(hub->consume[i].degree);

			if (argdata.with_angle_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			double d = hub->consume[i].distance / 1000.0;

			bool custom = false;
			for (int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
			{
				msg.points[idx].x = std::numeric_limits<float>::infinity();
				msg.points[idx].y = std::numeric_limits<float>::infinity();
				msg.points[idx].z = std::numeric_limits<float>::infinity();
				msg.channels[0].values[idx] = std::numeric_limits<uint16_t>::infinity();
				idx++;
			}
			else
			{
				float a = hub->consume[i].degree * M_PI / 180.0;
				msg.points[idx].x = cos(a) * d;
				msg.points[idx].y = sin(a) * d*-1;
				msg.points[idx].z = 0;
				msg.channels[0].values[idx] = hub->consume[i].confidence;
				idx++;
			}
		}
	}
	else
	{
		for (int i = 0; i <= hub->consume.size() - 1; i++)
		{
			double deg = m_driver->ROSAng(hub->consume[i].degree);

			if (argdata.with_angle_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			double d = hub->consume[i].distance / 1000.0;

			bool custom = false;
			for (int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
			{
				msg.points[idx].x = std::numeric_limits<float>::infinity();
				msg.points[idx].y = std::numeric_limits<float>::infinity();
				msg.points[idx].z = std::numeric_limits<float>::infinity();
				msg.channels[0].values[idx] = std::numeric_limits<uint16_t>::infinity();
				idx++;
			}
			else
			{
				float a = hub->consume[i].degree * M_PI / 180.0;
				msg.points[idx].x = cos(a) * d;
				msg.points[idx].y = sin(a) * d;
				msg.points[idx].z = 0;
				msg.channels[0].values[idx] = hub->consume[i].confidence;
				idx++;
			}
		}
	}

	if(argdata.output_cloud2)
	{
		sensor_msgs::PointCloud2 laserCloudMsg;
    	convertPointCloudToPointCloud2(msg, laserCloudMsg);
		cloud_pub.publish(laserCloudMsg);
	}
	else
	{
		cloud_pub.publish(msg);
	}
}

bool get_range_param(ros::NodeHandle nh, const char *name, Range &range)
{
	std::vector<double> rg;
	if (nh.getParam(name, rg))
	{
		if (rg.size() == 2 && rg[0] < rg[1])
		{
			range.min = rg[0] * 180 / M_PI;
			range.max = rg[1] * 180 / M_PI;
			return true;
		}
	}
	return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pacecat_laser_publisher");
    ros::NodeHandle node_handle;
    ros::NodeHandle priv_nh("~");

    m_driver = new PaceCatPlaybackDriver();

    ArgDataPlayback &argdata = m_driver->getArgDataPlayback();

    priv_nh.param("frame_id", argdata.frame_id, std::string("LH_laser"));
    priv_nh.param("topic_scan", argdata.topic_scan, std::string("scan"));
	priv_nh.param("topic_cloud", argdata.topic_cloud, std::string("cloud"));
	priv_nh.param("output_scan", argdata.output_scan, true);
	priv_nh.param("output_cloud", argdata.output_cloud, false);
	priv_nh.param("output_cloud2", argdata.output_cloud2, false);
    priv_nh.param("log_path", argdata.log_path, std::string("/tmp/1.txt"));
	priv_nh.param("log_file_format", argdata.log_file_format, std::string("pcap"));
    priv_nh.param("playback_rate", argdata.playback_rate, -1);
	priv_nh.param("rotation_direction", argdata.rotation_direction, -1);
	priv_nh.param("hard_resample", argdata.hard_resample, false);
	priv_nh.param("soft_resample", argdata.soft_resample, false);
	priv_nh.param("resample_res", argdata.resample, -1.0);
	priv_nh.param("error_circle", argdata.error_circle, 3);
	priv_nh.param("error_scale", argdata.error_scale, 0.9);	
	priv_nh.param("with_angle_filter", argdata.with_angle_filter, false);
	priv_nh.param("min_angle", argdata.min_angle, -M_PI);
	priv_nh.param("max_angle", argdata.max_angle, M_PI);
	priv_nh.param("min_dist", argdata.min_dist, 0.1);
	priv_nh.param("max_dist", argdata.max_dist, 50.0);
	priv_nh.param("from_zero", argdata.from_zero, false);
	priv_nh.param("reversed", argdata.reversed, false);

	for (int i = 1;; i++)
	{
		char name[32];
		sprintf(name, "mask%d", i);
		Range range;
		if (!get_range_param(priv_nh, name, range))
			break;
		argdata.masks.push_back(range);
	}

	priv_nh.param("filter_open", argdata.fitter.isopen, false);
	priv_nh.param("filter_type", argdata.fitter.type, 1);
	priv_nh.param("max_range", argdata.fitter.max_range, 20.0);
	priv_nh.param("min_range", argdata.fitter.min_range, 0.5);
	priv_nh.param("max_range_difference", argdata.fitter.max_range_difference, 0.1);
	priv_nh.param("filter_window", argdata.fitter.filter_window, 1);

    m_driver->openPlaybackThread();

    ros::Publisher laser_pub, cloud_pub, cloud2_pub;
    if (argdata.output_scan)
    {
        laser_pub = node_handle.advertise<sensor_msgs::LaserScan>(argdata.topic_scan, 50);
    }
    if (argdata.output_cloud)
    {
        cloud_pub = node_handle.advertise<sensor_msgs::PointCloud>(argdata.topic_cloud, 50);
    }
    else if (argdata.output_cloud2)
    {
        cloud2_pub = node_handle.advertise<sensor_msgs::PointCloud2>(argdata.topic_cloud, 50);
    }

	while (ros::ok())
	{
		ros::spinOnce();
		bool idle = true;

        PubHub *hub = m_driver->getPlaybackHub();
        if (hub->nfan == 0)
            continue;
        bool ret = false;
        if (hub->offsetangle == -1)
        {
            ret = m_driver->checkPlaybackIsRun();
            continue;
        }

        int n = m_driver->GetPlaybackAllFans(hub, argdata);
        if (n > 0)
        {
			/* ---- 每 10 秒打印一次频率 ---- */
			static ros::Time t0 = ros::Time::now();
			static int cnt = 0;

			++cnt;

			if ((ros::Time::now() - t0).toSec() >= 10.0) {
				double dt = (ros::Time::now() - t0).toSec();
				DEBUG("GetPlaybackAllFans  freq = %.2f Hz", cnt / dt);
				t0 = ros::Time::now();
				cnt = 0;
			}
			/* ----------------------------- */
			
            idle = false;
            if (argdata.output_scan)
            {
                PublishLaserScan(laser_pub, hub);
            }
            if (argdata.output_cloud)
            {
                PublishCloud(cloud_pub, hub);
            }
            else if (argdata.output_cloud2)
            {
                PublishCloud(cloud2_pub, hub);
            }
            hub->consume.clear();
        }	
		if (idle)
		{
			ros::Duration(0.001).sleep();
		}
	}
    return 0;
}