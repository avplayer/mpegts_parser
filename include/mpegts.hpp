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

	enum {
		unkown_type = 0x00,
		video_mpeg1 = 0x01,
		video_mpeg2 = 0x02,
		audio_mpeg1 = 0x03,
		audio_mpeg2 = 0x04,
		private_section = 0x05,
		private_data = 0x06,
		audio_aac = 0x0f,
		audio_aac_latm = 0x11,
		video_mpeg4 = 0x10,
		metadata = 0x15,
		video_h264 = 0x1b,
		video_hevc = 0x24,
		video_cavs = 0x42,
		video_vc1 = 0xea,
		video_dirac = 0xd1,
		audio_ac3 = 0x81,
		audio_dts = 0x82,
		audio_truehd = 0x83,
		audio_eac3 = 0x87,
	};

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
			, stream_type_(0)
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
		int stream_type_;
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

		uint8_t stream_type(uint16_t pid) const;
		std::string stream_name(uint16_t pid) const;

	protected:
		inline bool do_internal_parser(const uint8_t* parse_ptr, mpegts_info& info);
		inline void do_parse_h264(const uint8_t* ptr, const uint8_t* end, mpegts_info& info);
		inline void do_parse_hevc(const uint8_t* ptr, const uint8_t* end, mpegts_info& info);
		inline void do_parse_mpeg2(const uint8_t* ptr, const uint8_t* end, mpegts_info& info);

	protected:
		std::bitset<0x2000> m_video_elementary_PIDs;
		std::bitset<0x2000> m_audio_elementary_PIDs;
		std::vector<uint8_t> m_matadata;
		std::bitset<0x2000> m_pmt_pids;
		std::vector<int8_t> m_cc_pids;	// 每个pid的连续计数.
		// 在2个start之间, 是否已经确定帧类型, 如果已经确定, 那么就不必再找了.
		std::bitset<0x2000> m_type_pids;
		bool m_has_pat;
		// key = stream type id, value = stream type name.
		std::map<uint8_t, std::string> m_stream_types;
		// key = pid, value = stream type id.
		std::vector<uint8_t> m_streams;
	};
}
