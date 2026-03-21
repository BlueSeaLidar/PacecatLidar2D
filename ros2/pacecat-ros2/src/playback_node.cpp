// #include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "../sdk/include/pacecat_playback.h"

PaceCatPlaybackDriver *m_driver = NULL;

void PublishLaserScan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub, HPublish pub)
{
	ArgDataPlayback &argdata = m_driver->getArgDataPlayback();

	PubHub *hub = (PubHub *)pub;
	sensor_msgs::msg::LaserScan msg;
	int N = hub->consume.size();

	msg.header.stamp.sec = hub->ts_beg[0];
	msg.header.stamp.nanosec = hub->ts_beg[1];

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
			for (unsigned int k = 0; k < argdata.masks.size() && !custom; k++)
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
		for (int i = 0; i <= (int)(hub->consume.size() - 1); i++)
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
			for (unsigned int k = 0; k < argdata.masks.size() && !custom; k++)
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
	laser_pub->publish(msg);
}

struct CloutPointData
{
	float x;
	float y;
	float z;
	uint8_t c;
};

void PublishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub2, HPublish pub)
{
	ArgDataPlayback &argdata = m_driver->getArgDataPlayback();

	PubHub *hub = (PubHub *)pub;
	sensor_msgs::msg::PointCloud2 msg;
	int N = hub->consume.size();
	msg.header.stamp.sec = hub->ts_beg[0];
	msg.header.stamp.nanosec = hub->ts_beg[1];

	msg.header.frame_id = argdata.frame_id;

	double min_deg = argdata.min_angle * 180 / M_PI;
	double max_deg = argdata.max_angle * 180 / M_PI;

	// if (argdata.with_angle_filter)
	// {
	// 	double min_pos, max_pos;
	// 	N = m_driver->GetCount(hub->consume, min_deg, max_deg, min_pos, max_pos);
	// }

	msg.height = 1;
	msg.width = N;
	msg.fields.resize(4);

	msg.fields[0].name = "x";
	msg.fields[0].offset = 0;
	msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	msg.fields[0].count = 1;

	msg.fields[1].name = "y";
	msg.fields[1].offset = 4; // 假设float是4字节
	msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	msg.fields[1].count = 1;

	msg.fields[2].name = "z";
	msg.fields[2].offset = 8;
	msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	msg.fields[2].count = 1;

	msg.fields[3].name = "intensity";
	msg.fields[3].offset = 12;
	msg.fields[3].datatype = sensor_msgs::msg::PointField::INT32;
	msg.fields[3].count = 1;

	msg.is_bigendian = false;  
    msg.point_step = 16; // 3个float * 4字节/float  
    msg.row_step = msg.point_step * msg.width;  
    msg.data.resize(msg.row_step); 
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
			for (unsigned int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}
			CloutPointData cpdata;
			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
			{
			
				cpdata.x =std::numeric_limits<float>::infinity();;
				cpdata.y = std::numeric_limits<float>::infinity();;
				cpdata.z = std::numeric_limits<float>::infinity();;
				cpdata.c= std::numeric_limits<uint8_t>::infinity();;	
				
			}
			else
			{
				float a = hub->consume[i].degree * M_PI / 180.0;
				cpdata.x = cos(a) * d;
				cpdata.y = sin(a) * d * -1;
				cpdata.z = 0;
				cpdata.c= hub->consume[i].confidence;	
			}
			memcpy(&msg.data[i * msg.point_step], &cpdata, msg.point_step);
			//DEBUG("%d %ld",msg.point_step,sizeof(CloutPointData));
			idx++;

		}
	}
	else
	{

		// DEBUG("%s %d %f %f\n", __FUNCTION__, __LINE__, hub->consume[0].degree,hub->consume[hub->consume.size()-1].degree);
		for (unsigned int i = 0; i <= hub->consume.size() - 1; i++)
		{
			double deg = m_driver->ROSAng(hub->consume[i].degree);
			if (argdata.with_angle_filter)
			{
				// DEBUG("%lf %lf %lf %lf",hub->consume[i].degree,deg,min_deg,max_deg);
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}
			double d = hub->consume[i].distance / 1000.0;
			bool custom = false;
			for (unsigned int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			CloutPointData cpdata;
			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
			{
			
				cpdata.x =std::numeric_limits<float>::infinity();;
				cpdata.y = std::numeric_limits<float>::infinity();;
				cpdata.z = std::numeric_limits<float>::infinity();;
				cpdata.c= std::numeric_limits<uint8_t>::infinity();;		
			}
			else
			{
				float a = hub->consume[i].degree * M_PI / 180.0;
				cpdata.x = cos(a) * d;
				cpdata.y = sin(a) * d ;
				cpdata.z = 0;
				cpdata.c= hub->consume[i].confidence;	
			}
			memcpy(&msg.data[i * msg.point_step], &cpdata, msg.point_step);
			//DEBUG("%d %ld",msg.point_step,sizeof(CloutPointData));
			idx++;
		}
	}
	cloud_pub2->publish(msg);
}

#define READ_PARAM(TYPE, NAME, VAR, INIT)     \
	VAR = INIT;                               \
	node->declare_parameter<TYPE>(NAME, VAR); \
	node->get_parameter(NAME, VAR);

bool get_range_param(std::shared_ptr<rclcpp::Node> node, const char *name, Range &range)
{
	std::string mask;
	READ_PARAM(std::string, name, mask, "");
	if (mask != "")
	{
		int index = mask.find(",");
		float min = atof(mask.substr(0, index).c_str());
		float max = atof(mask.substr(index + 1).c_str());
		if (min < max)
		{
			range.min = min * 180 / M_PI;
			range.max = max * 180 / M_PI;
			return true;
		}
	}
	return false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pacecat_playback");

    rclcpp::QoS qos(0);
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    m_driver = new PaceCatPlaybackDriver();

    ArgDataPlayback &argdata = m_driver->getArgDataPlayback();

    READ_PARAM(std::string, "frame_id", argdata.frame_id, "LH_laser");
    READ_PARAM(std::string, "topic_scan", argdata.topic_scan, std::string("scan"));
	READ_PARAM(std::string, "topic_cloud", argdata.topic_cloud, std::string("cloud"));
    READ_PARAM(bool, "output_scan", argdata.output_scan, true);
	READ_PARAM(bool, "output_cloud2", argdata.output_cloud2, false);
    READ_PARAM(std::string, "log_path", argdata.log_path, std::string("/tmp/1.txt"));
    READ_PARAM(std::string, "log_file_format", argdata.log_file_format, std::string("pcap"));
    READ_PARAM(int, "playback_rate", argdata.playback_rate, -1);
    READ_PARAM(int, "rotation_direction", argdata.rotation_direction, -1);
	READ_PARAM(bool, "hard_resample", argdata.hard_resample, false);
    READ_PARAM(bool, "soft_resample", argdata.soft_resample, false);
    READ_PARAM(double, "resample_res", argdata.resample, -1.0);
    READ_PARAM(int, "error_circle", argdata.error_circle, 3);
    READ_PARAM(double, "error_scale", argdata.error_scale, 0.9);
	READ_PARAM(bool, "with_angle_filter", argdata.with_angle_filter, false);
	READ_PARAM(double, "min_angle", argdata.min_angle, -M_PI);
	READ_PARAM(double, "max_angle", argdata.max_angle, M_PI);
	READ_PARAM(double, "min_dist", argdata.min_dist, 0.1);
	READ_PARAM(double, "max_dist", argdata.max_dist, 50.0);
	READ_PARAM(bool, "from_zero", argdata.from_zero, false);
	READ_PARAM(bool, "reversed", argdata.reversed, false);

	for (int i = 1;; i++)
	{
		char name[32];
		sprintf(name, "mask%d", i);
		Range range;
		if (!get_range_param(node, name, range))
			break;
		argdata.masks.push_back(range);
	}

	READ_PARAM(bool, "filter_open", argdata.fitter.isopen, false);
	READ_PARAM(int, "filter_type", argdata.fitter.type, 1);
	READ_PARAM(double, "max_range", argdata.fitter.max_range, 20.0);
	READ_PARAM(double, "min_range", argdata.fitter.min_range, 0.5);
	READ_PARAM(double, "max_range_difference", argdata.fitter.max_range_difference, 0.1);
	READ_PARAM(int, "filter_window", argdata.fitter.filter_window, 1);

    m_driver->openPlaybackThread();

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub2;

    if (argdata.output_scan)
    {
		laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(argdata.topic_scan, qos);
	}

    if (argdata.output_cloud2)
    {
		cloud_pub2 = node->create_publisher<sensor_msgs::msg::PointCloud2>(argdata.topic_cloud, qos);
	}

	rclcpp::WallRate loop_rate(100);
	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		bool idle = true;

        PubHub *hub = m_driver->getPlaybackHub();
        if (hub->nfan == 0)
            continue;
        if (hub->offsetangle == -1)
        {
            m_driver->checkPlaybackIsRun();
            continue;
        }

        int n = m_driver->GetPlaybackAllFans(hub, argdata);
        if (n > 0)
        {
			/* ---- 每 10 秒打印一次频率 ---- */
			static rclcpp::Time t0 = node->now();   // 节点时间
			static int cnt = 0;

			++cnt;

			if ((node->now() - t0).seconds() >= 10.0) {
				double dt = (node->now() - t0).seconds();
				DEBUG("GetPlaybackAllFans  freq = %.2f Hz", cnt / dt);
				t0 = node->now();
				cnt = 0;
			}
			/* ----------------------------- */
			
            idle = false;
            if (argdata.output_scan)
            {
                PublishLaserScan(laser_pub, hub);
            }
            if (argdata.output_cloud2)
            {
                PublishCloud(cloud_pub2, hub);
            }
            hub->consume.clear();
        }	
		if (idle)
		{
			loop_rate.sleep();
		}
	}
    return 0;
}