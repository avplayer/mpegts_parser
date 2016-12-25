//
// Copyright (C) 2016 Jack.
//
// Author: jack
// Email:  jack.wgm at gmail dot com
//

#pragma once

#include <set>
#include <map>
#include <vector>
#include <string>
#include <cinttypes>
#include <bitset>

namespace util {

	struct mpegts_info
	{
		mpegts_info()
			: pid_(-1)
			, start_(false)
			, cc_(0)
			, type_(reserve)
			, pts_(-1)
			, dts_(-1)
			, is_video_(false)
			, is_audio_(false)
			, payload_begin_(nullptr)
			, payload_end_(nullptr)
		{}

		int pid_;
		bool start_;
		int cc_;
		enum pkt_t
		{
			pat,
			pmt,
			reserve,
			idr,
			data,
			nullpkt,
		} type_;
		int64_t pts_;
		int64_t dts_;
		bool is_video_;
		bool is_audio_;
		uint8_t* payload_begin_;
		uint8_t* payload_end_;
	};

	class mpegts_parser
	{
		// c++11 noncopyable.
		mpegts_parser(const mpegts_parser&) = delete;
		mpegts_parser& operator=(const mpegts_parser&) = delete;

	public:
		mpegts_parser();
		~mpegts_parser();

	public:
		bool do_parser(const uint8_t* parse_ptr, mpegts_info& info);
		std::vector<uint8_t>& matadata();
		int stream_type() const;

	protected:
		inline bool do_internal_parser(const uint8_t* parse_ptr, mpegts_info& info);

	protected:
		std::bitset<0x2000> m_video_elementary_PIDs;
		std::bitset<0x2000> m_audio_elementary_PIDs;
		std::vector<uint8_t> m_matadata;
		std::bitset<0x2000> m_pmt_pids;
		std::vector<int8_t> m_cc_pids;	// 每个pid的连续计数.
		// 在2个start之间, 是否已经确定帧类型, 如果已经确定, 那么就不必再找了.
		std::bitset<0x2000> m_type_pids;
		bool m_has_pat;
		// bool m_has_sps_pps;
		// key = stream type id, value = stream type name.
		std::map<uint8_t, std::string> m_stream_types;
		int m_stream_type;
	};
}
