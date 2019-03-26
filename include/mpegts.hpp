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
#include <algorithm>
#include <cinttypes>
#include <bitset>

namespace util {

	class byte_streambuf
	{
		// c++11 noncopyable.
		byte_streambuf(const byte_streambuf&) = delete;
		byte_streambuf& operator=(const byte_streambuf&) = delete;

		enum { buffer_delta = 256 };

	public:
		byte_streambuf();
		~byte_streambuf();

		// 支持move构造.
		byte_streambuf(byte_streambuf&& rhs);

	public:
		void clear();
		void shrink_to_fit();
		size_t size() const noexcept;
		size_t capacity() const noexcept;
		const uint8_t* data() const noexcept;
		uint8_t* prepare(size_t n);

		void commit(size_t n);
		void consume(size_t n);

	protected:
		void reserve(std::size_t n);
		void setg(uint8_t* f, uint8_t* n, uint8_t* l);
		void setp(uint8_t* f, uint8_t* l);
		void setp(uint8_t* f, uint8_t* n, uint8_t* l);

		uint8_t* pptr() const;
		uint8_t* gptr() const;
		uint8_t* epptr() const;
		uint8_t* eback() const;
		uint8_t* egptr() const;

	private:
		std::vector<uint8_t> m_buffer;

		uint8_t* m_get_first;
		uint8_t* m_get_next;
		uint8_t* m_get_last;

		uint8_t* m_put_first;
		uint8_t* m_put_next;
		uint8_t* m_put_last;

		size_t m_max_size;
	};

	enum {
		unkown_type = 0x00,
		video_mpeg1 = 0x01,
		video_mpeg2 = 0x02,
		audio_mpeg1 = 0x03,
		audio_mpeg2 = 0x04,
		private_section = 0x05,
		private_data = 0x06,
		iso_13818_1_pes = 0x07,
		iso_13522_mheg = 0x08,
		itu_t_rec_h_222_1 = 0x09,
		iso_13818_6_type_a = 0x0a,
		iso_13818_6_type_b = 0x0b,
		iso_13818_6_type_c = 0x0c,
		iso_13818_6_type_d = 0x0d,
		iso_13818_1_auxiliary = 0x0e,
		audio_aac = 0x0f,
		video_mpeg4 = 0x10,
		audio_aac_latm = 0x11,
		iso_14496_1_pes = 0x12,
		iso_14496_1_sections = 0x13,
		iso_13818_6_SDP = 0x14,
		metadata = 0x15,
		video_h264 = 0x1b,
		video_hevc = 0x24,
		video_cavs = 0x42,
		video_dirac = 0xd1,
		video_vc1 = 0xea,
		audio_pcm_bluray = 0x80,
		audio_ac3 = 0x81,
		audio_dts_0 = 0x82,
		audio_truehd = 0x83,
		audio_eac3_0 = 0x84,
		audio_dts_1 = 0x85,
		audio_dts_2 = 0x86,
		audio_eac3_1 = 0x87,
		audio_eac3_2 = 0xa1,
		audio_dts_3 = 0xa2,
		hdmv_pgs_subtitle = 0x90,
		audio_dts_4 = 0x8a,
	};

	enum {
		av_picture_type_none = 0,	///< Undefined
		av_picture_type_i,			///< Intra
		av_picture_type_p,			///< Predicted
		av_picture_type_b,			///< Bi-dir predicted
		av_picture_type_s,			///< S(GMC)-VOP MPEG4
		av_picture_type_si,			///< Switching Intra
		av_picture_type_sp,			///< Switching Predicted
		av_picture_type_bi,			///< BI type
	};

	struct mpegts_info
	{
		mpegts_info()
			: pid_(-1)
			, start_(false)
			, cc_(0)
			, crc_(0xffffffff)
			, pict_type_(av_picture_type_none)
			, type_(reserve)
			, pcr_(-1)
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
		uint32_t crc_;
		enum pkt_t
		{
			pat,
			pmt,
			reserve,
			idr,
			data,
			nullpkt,
		} type_;
		int pict_type_;
		int64_t pcr_;
		int64_t pts_;
		int64_t dts_;
		bool is_video_;
		bool is_audio_;
		int stream_type_;
		uint8_t* payload_begin_;
		uint8_t* payload_end_;
	};

	struct stream_info
	{
		int pid_;
		enum
		{
			stream_video,
			stream_audio,
		} type_;

		int stream_type_;
	};


	uint32_t crc32(const uint8_t* data, size_t len);

	class mpegts_parser
	{
		// c++11 noncopyable.
		mpegts_parser(const mpegts_parser&) = delete;
		mpegts_parser& operator=(const mpegts_parser&) = delete;

	public:
		mpegts_parser();
		~mpegts_parser();

	public:
		bool do_parser(const uint8_t* parse_ptr, mpegts_info& info, bool check_crc = false);
		std::vector<uint8_t>& matadata();

		uint8_t stream_type(uint16_t pid) const;
		std::string stream_name(uint16_t pid) const;
		uint16_t stream_type(const std::string& name) const;

	public:
		// 初始化用于编码到ts的流信息.
		bool init_streams(const std::vector<stream_info>& streams);

		// 添加数据到ts编码器中.
		bool mux_stream(const mpegts_info& info);

		// 获取已经编码的ts数据大小.
		size_t mpegts_size() const;
		// 从已经编码的ts数据缓冲中取出指定大小的ts数据.
		void fetch_mpegts(uint8_t* data, int size);

	protected:
		inline bool do_internal_parser(const uint8_t* parse_ptr, mpegts_info& info, bool check_crc = false);
		inline void do_parse_h264(const uint8_t* ptr, const uint8_t* end, mpegts_info& info);
		inline void do_parse_hevc(const uint8_t* ptr, const uint8_t* end, mpegts_info& info);
		inline void do_parse_mpeg2(const uint8_t* ptr, const uint8_t* end, mpegts_info& info);

		void add_pat(uint8_t* ts);
		void add_pmt(uint8_t* ts);

	protected:
		std::bitset<0x2000> m_video_elementary_PIDs;
		std::bitset<0x2000> m_audio_elementary_PIDs;
		std::vector<uint8_t> m_matadata;
		std::bitset<0x2000> m_pmt_pids;
		std::vector<int8_t> m_cc_pids;	// 每个pid的连续计数.
		// 在2个start之间, 是否已经确定帧类型, 如果已经确定, 那么就不必再找了.
		std::bitset<0x2000> m_type_pids;
		bool m_has_pat;
		int16_t m_pcr_pid;
		// key = stream type id, value = stream type name.
		std::map<uint8_t, std::string> m_stream_types;
		// key = pid, value = stream type id.
		std::vector<uint8_t> m_streams;

		// ts编码相关信息.
		std::map<int, mpegts_info> m_mpegts;
		int m_pmt_pid;
		int64_t m_packet_count;
		int64_t m_first_pcr;
		int64_t m_total_bytes;
		int m_pcr_packet_count;
		int m_pat_count;
		int m_pmt_count;
		byte_streambuf m_mpegts_data;
	};
}
